#!/usr/bin/env python3
"""YOLO-seg + yellow HSV + registered depth landmine detonator localizer.

This node is deliberately independent from the legacy manipulation stack.  It
never reads Gazebo model states.  Gazebo truth is reserved for the test manager
and is not part of this node's inputs or target calculation.
"""

import collections
import copy
import json
import math
import os
import re
import threading
import time

import cv2
import message_filters
import numpy as np
import rospy
import tf.transformations as tft
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped TF conversion
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from std_srvs.srv import Trigger, TriggerResponse


NO_DETECTION = "NO_DETECTION"
NO_DETONATOR = "NO_DETONATOR"
INVALID_DEPTH = "INVALID_DEPTH"
TF_FAILED = "TF_FAILED"
TARGET_UNSTABLE = "TARGET_UNSTABLE"


def _odd_kernel(value):
    value = max(int(value), 1)
    return value if value % 2 else value + 1


def depth_to_metres(depth, encoding):
    """Return a float32 metric depth image or raise ValueError."""
    if encoding == "32FC1":
        return depth.astype(np.float32, copy=False)
    if encoding in ("16UC1", "mono16"):
        return depth.astype(np.float32) * 0.001
    raise ValueError("unsupported depth encoding {!r}".format(encoding))


def robust_mask_cloud(mask, depth_m, intrinsics, depth_min, depth_max,
                      erode_pixels, minimum_samples, mad_scale,
                      minimum_mad_gate):
    """Back-project robust in-mask depth pixels into a metric camera cloud."""
    work = mask.astype(np.uint8)
    if erode_pixels > 0:
        size = 2 * int(erode_pixels) + 1
        work = cv2.erode(work, np.ones((size, size), np.uint8), iterations=1)

    vv, uu = np.nonzero(work)
    if len(uu) < minimum_samples:
        return None, 0
    zz = depth_m[vv, uu].astype(np.float64)
    valid = np.isfinite(zz) & (zz >= depth_min) & (zz <= depth_max)
    uu, vv, zz = uu[valid], vv[valid], zz[valid]
    if len(zz) < minimum_samples:
        return None, int(len(zz))

    median_z = float(np.median(zz))
    mad = float(np.median(np.abs(zz - median_z)))
    gate = max(float(minimum_mad_gate), float(mad_scale) * 1.4826 * mad)
    inliers = np.abs(zz - median_z) <= gate
    uu, vv, zz = uu[inliers], vv[inliers], zz[inliers]
    if len(zz) < minimum_samples:
        return None, int(len(zz))

    fx, fy, cx, cy = intrinsics
    xx = (uu.astype(np.float64) - cx) * zz / fx
    yy = (vv.astype(np.float64) - cy) * zz / fy
    points = np.column_stack((xx, yy, zz)).astype(np.float64, copy=False)
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < minimum_samples:
        return None, int(points.shape[0])
    return points, int(points.shape[0])


def robust_mask_points(mask, depth_m, intrinsics, depth_min, depth_max,
                       erode_pixels, minimum_samples, mad_scale,
                       minimum_mad_gate):
    """Compatibility wrapper returning the median of the physical point cloud."""
    points, count = robust_mask_cloud(
        mask, depth_m, intrinsics, depth_min, depth_max, erode_pixels,
        minimum_samples, mad_scale, minimum_mad_gate,
    )
    if points is None:
        return None, count
    return np.median(points, axis=0), count


def estimate_top_surface(points_gravity, quantile, band_m, minimum_samples):
    """Estimate the upper face from a robust gravity-Z quantile and top band."""
    points = np.asarray(points_gravity, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        return None, 0
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < minimum_samples:
        return None, int(points.shape[0])
    threshold = float(np.quantile(points[:, 2], quantile))
    top = points[points[:, 2] >= threshold - float(band_m)]
    if top.shape[0] < minimum_samples:
        order = np.argsort(points[:, 2])
        top = points[order[-minimum_samples:]]
    estimate = np.asarray(
        [np.median(top[:, 0]), np.median(top[:, 1]), np.median(top[:, 2])],
        dtype=np.float64,
    )
    return estimate, int(top.shape[0])


def fit_local_ground_plane(points_gravity, target_xy, minimum_samples,
                           residual_floor, mad_scale, minimum_xy_span):
    """Robustly fit ``z=ax+by+c`` and evaluate it at ``target_xy``."""
    points = np.asarray(points_gravity, dtype=np.float64)
    target_xy = np.asarray(target_xy, dtype=np.float64)
    if (points.ndim != 2 or points.shape[1] != 3
            or points.shape[0] < minimum_samples):
        return None, 0
    points = points[np.all(np.isfinite(points), axis=1)]
    if points.shape[0] < minimum_samples:
        return None, int(points.shape[0])
    centred = points[:, :2] - target_xy[:2]
    if (np.ptp(centred[:, 0]) < minimum_xy_span
            or np.ptp(centred[:, 1]) < minimum_xy_span):
        return None, int(points.shape[0])
    design = np.column_stack((centred[:, 0], centred[:, 1], np.ones(len(points))))
    keep = np.ones(len(points), dtype=bool)
    for _ in range(4):
        if int(np.count_nonzero(keep)) < minimum_samples:
            return None, int(np.count_nonzero(keep))
        active = design[keep]
        if np.linalg.matrix_rank(active) < 3:
            return None, int(np.count_nonzero(keep))
        coefficients, _residuals, _rank, _singular = np.linalg.lstsq(
            active, points[keep, 2], rcond=None
        )
        residual = points[:, 2] - design.dot(coefficients)
        centre = float(np.median(residual[keep]))
        mad = float(np.median(np.abs(residual[keep] - centre)))
        gate = max(float(residual_floor), float(mad_scale) * 1.4826 * mad)
        updated = np.abs(residual - centre) <= gate
        if np.array_equal(updated, keep):
            break
        keep = updated
    if int(np.count_nonzero(keep)) < minimum_samples:
        return None, int(np.count_nonzero(keep))
    coefficients, _residuals, _rank, _singular = np.linalg.lstsq(
        design[keep], points[keep, 2], rcond=None
    )
    value = float(coefficients[2])
    return (value if math.isfinite(value) else None,
            int(np.count_nonzero(keep)))


def largest_component(mask, min_area, max_area):
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    candidates = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            candidates.append((area, label, centroids[label]))
    if not candidates:
        return None, None, 0
    area, label, centroid = max(candidates, key=lambda item: item[0])
    return labels == label, (float(centroid[0]), float(centroid[1])), area


def valid_components(mask, min_area, max_area):
    """Return every area-valid component so a source prior can choose among them."""
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask.astype(np.uint8), connectivity=8
    )
    result = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if min_area <= area <= max_area:
            centroid = centroids[label]
            result.append(
                (
                    labels == label,
                    (float(centroid[0]), float(centroid[1])),
                    area,
                )
            )
    return result


def color_roi_debug_metadata(yellow_all, roi_mask, filtered_yellow,
                             expected_pixel, roi_bounds, prior_radius):
    """Summarize colour gating without turning the map prior into a target.

    These values are diagnostic only.  In particular, ``expected_pixel`` is
    the selected UAV-map association prior; no 3-D target is generated from it.
    Keeping the raw and post-morphology counts makes a zero-colour/occlusion
    failure distinguishable from an area or shape rejection.
    """
    yellow_all = np.asarray(yellow_all, dtype=bool)
    roi_mask = np.asarray(roi_mask, dtype=bool)
    filtered_yellow = np.asarray(filtered_yellow, dtype=bool)
    if (yellow_all.shape != roi_mask.shape
            or yellow_all.shape != filtered_yellow.shape
            or yellow_all.ndim != 2):
        raise ValueError("colour diagnostic masks must be equal-size 2-D arrays")
    x0, y0, x1, y1 = [int(value) for value in roi_bounds]
    height, width = yellow_all.shape
    metadata = {
        "color_roi_xyxy": [x0, y0, x1, y1],
        "yellow_pixels_full_raw": int(np.count_nonzero(yellow_all)),
        "yellow_pixels_roi_raw": int(np.count_nonzero(yellow_all & roi_mask)),
        "yellow_pixels_roi_filtered": int(np.count_nonzero(filtered_yellow)),
        "expected_source_pixel": None,
        "expected_source_in_image": None,
        "expected_source_in_color_roi": None,
        "expected_source_prior_radius_px": int(prior_radius),
        "expected_source_prior_circle_in_image": None,
    }
    if expected_pixel is None:
        return metadata
    u, v = float(expected_pixel[0]), float(expected_pixel[1])
    radius = float(prior_radius)
    metadata.update({
        "expected_source_pixel": [u, v],
        "expected_source_in_image": bool(0.0 <= u < width and 0.0 <= v < height),
        "expected_source_in_color_roi": bool(x0 <= u < x1 and y0 <= v < y1),
        "expected_source_prior_circle_in_image": bool(
            radius <= u < width - radius and radius <= v < height - radius
        ),
    })
    return metadata


class WristMineLocalizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.processing = threading.Lock()
        self.latest_pair = None
        self.camera_info = None
        self.expected_source_pose = None
        # A close coarse-approach view can put the fingers and wrist in most of
        # the annulus used for terrain fitting.  Keep the last *directly*
        # validated local-ground observation from this action as a bounded
        # reference.  Confirmation-epoch resets deliberately preserve it;
        # task/attempt changes clear it below so it can never cross mines.
        self.validated_ground_reference = None
        self.last_processed_stamp = rospy.Time(0)
        # Confirmation epochs are explicit manipulation-stage boundaries.
        # A reset clears the three-frame deque and rejects already queued RGB-D
        # pairs whose sensor stamp predates the reset, so a new atomic target
        # can never contain LOOK/PREGRASP samples from the previous stage.
        self.confirmation_epoch = rospy.Time(0)
        self.history = collections.deque()
        self.last_debug_save = 0.0
        self.attempt_id = "standalone"
        self.debug_sequence = 0

        self.localization_mode = str(
            rospy.get_param("~localization_mode", "hybrid")
        ).strip().lower()
        if self.localization_mode not in {"hybrid", "color_only"}:
            raise rospy.ROSInitException(
                "localization_mode must be 'hybrid' or 'color_only'"
            )
        self.use_yolo = self.localization_mode == "hybrid"
        configured_model_path = rospy.get_param("~model_path", "")
        self.model_path = (
            os.path.abspath(os.path.expanduser(configured_model_path))
            if configured_model_path else ""
        )
        if self.use_yolo and not os.path.isfile(self.model_path):
            raise rospy.ROSInitException(
                "segmentation model does not exist: {}".format(self.model_path)
            )

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/camera/color/camera_info"
        )
        self.camera_frame = rospy.get_param(
            "~camera_frame", "realsense_camera_optical_frame"
        )
        self.target_frame = rospy.get_param("~target_frame", "ur5_base_link")
        self.gravity_frame = rospy.get_param("~gravity_frame", "map")

        self.confidence = float(rospy.get_param("~confidence_threshold", 0.45))
        self.iou = float(rospy.get_param("~iou_threshold", 0.50))
        self.image_size = int(rospy.get_param("~image_size", 960))
        self.device = str(rospy.get_param("~device", "cpu"))
        self.torch_threads = int(rospy.get_param("~torch_threads", 4))
        self.rate = float(rospy.get_param("~inference_rate", 2.0))
        self.view_scale = float(rospy.get_param("~inference_view_scale", 0.40))
        if not 0.1 <= self.view_scale <= 1.0:
            raise rospy.ROSInitException("inference_view_scale must be in [0.1, 1.0]")
        self.canvas_value = int(rospy.get_param("~inference_canvas_value", 48))
        self.canvas_value = min(max(self.canvas_value, 0), 255)
        self.class_id = int(rospy.get_param("~target_class_id", 0))
        self.class_name = str(rospy.get_param("~target_class_name", "landmine"))
        self.max_det = int(rospy.get_param("~maximum_detections", 4))

        self.hsv_lower = np.asarray(
            rospy.get_param("~yellow_hsv_lower", [15, 80, 65]), dtype=np.uint8
        )
        self.hsv_upper = np.asarray(
            rospy.get_param("~yellow_hsv_upper", [42, 255, 255]), dtype=np.uint8
        )
        self.open_kernel = _odd_kernel(rospy.get_param("~yellow_open_kernel", 3))
        self.close_kernel = _odd_kernel(rospy.get_param("~yellow_close_kernel", 5))
        self.min_yellow_area = int(rospy.get_param("~minimum_yellow_area_px", 18))
        self.max_yellow_area = int(rospy.get_param("~maximum_yellow_area_px", 10000))
        self.min_yellow_fraction = float(
            rospy.get_param("~minimum_yellow_fraction", 0.002)
        )
        self.max_yellow_fraction = float(
            rospy.get_param("~maximum_yellow_fraction", 0.55)
        )
        # HSV is always the precise detonator locator. YOLO normally supplies
        # the safety mask; if the aerially-trained model misses the close wrist
        # view, a bounded image ROI plus 3-D arm-workspace checks provides a
        # conservative colour-only fallback.
        self.color_fallback_enabled = bool(
            rospy.get_param("~color_roi_fallback_enabled", True)
        )
        self.color_fallback_roi = [float(value) for value in rospy.get_param(
            "~color_fallback_roi", [0.10, 0.10, 0.90, 0.99]
        )]
        if (len(self.color_fallback_roi) != 4
                or not 0.0 <= self.color_fallback_roi[0] < self.color_fallback_roi[2] <= 1.0
                or not 0.0 <= self.color_fallback_roi[1] < self.color_fallback_roi[3] <= 1.0):
            raise rospy.ROSInitException(
                "color_fallback_roi must be [xmin,ymin,xmax,ymax] fractions"
            )
        self.color_fallback_min_area = int(
            rospy.get_param("~color_fallback_minimum_area_px", 18)
        )
        self.color_fallback_max_area = int(
            rospy.get_param("~color_fallback_maximum_area_px", 10000)
        )
        self.color_fallback_min_fill = float(
            rospy.get_param("~color_fallback_minimum_fill", 0.25)
        )
        self.color_fallback_min_aspect = float(
            rospy.get_param("~color_fallback_minimum_aspect", 0.25)
        )
        self.color_fallback_max_aspect = float(
            rospy.get_param("~color_fallback_maximum_aspect", 4.0)
        )
        self.color_fallback_confidence = float(
            rospy.get_param("~color_fallback_confidence", 0.50)
        )
        self.expected_source_topic = rospy.get_param(
            "~expected_source_topic", "/mine_grasp/expected_source_pose"
        )
        self.expected_source_roi_radius = max(
            int(rospy.get_param("~expected_source_roi_radius_px", 150)), 20
        )
        self.color_fallback_min_reach = float(
            rospy.get_param("~color_fallback_minimum_planar_reach", 0.25)
        )
        self.color_fallback_max_reach = float(
            rospy.get_param("~color_fallback_maximum_planar_reach", 0.62)
        )
        self.color_fallback_min_z = float(
            rospy.get_param("~color_fallback_minimum_target_z", -0.45)
        )
        self.color_fallback_max_z = float(
            rospy.get_param("~color_fallback_maximum_target_z", -0.18)
        )

        self.sync_slop = float(rospy.get_param("~sync_slop", 0.04))
        self.max_frame_age = float(rospy.get_param("~maximum_frame_age", 1.0))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.25))
        self.depth_min = float(rospy.get_param("~depth_min_m", 0.05))
        self.depth_max = float(rospy.get_param("~depth_max_m", 3.0))
        self.erode_pixels = int(rospy.get_param("~depth_erode_pixels", 2))
        self.min_depth_samples = int(rospy.get_param("~minimum_depth_samples", 15))
        self.mad_scale = float(rospy.get_param("~mad_scale", 3.5))
        self.min_mad_gate = float(rospy.get_param("~minimum_mad_gate_m", 0.002))
        self.detonator_height = float(rospy.get_param("~detonator_height_m", 0.060))
        self.top_surface_quantile = float(
            rospy.get_param("~top_surface_quantile", 0.80)
        )
        self.top_surface_band = float(
            rospy.get_param("~top_surface_band_m", 0.006)
        )
        self.top_surface_min_samples = max(
            int(rospy.get_param("~top_surface_minimum_samples", 12)), 3
        )
        self.ground_ring_inner_scale = float(
            rospy.get_param("~ground_ring_inner_scale", 1.25)
        )
        self.ground_ring_outer_scale = float(
            rospy.get_param("~ground_ring_outer_scale", 2.10)
        )
        self.ground_ring_min_inner_px = int(
            rospy.get_param("~ground_ring_min_inner_px", 8)
        )
        self.ground_ring_max_outer_px = int(
            rospy.get_param("~ground_ring_max_outer_px", 180)
        )
        self.ground_min_samples = max(
            int(rospy.get_param("~ground_minimum_samples", 40)), 10
        )
        self.ground_plane_residual_floor = float(
            rospy.get_param("~ground_plane_residual_floor_m", 0.006)
        )
        self.ground_plane_mad_scale = float(
            rospy.get_param("~ground_plane_mad_scale", 3.5)
        )
        self.ground_plane_min_xy_span = float(
            rospy.get_param("~ground_plane_minimum_xy_span_m", 0.04)
        )
        self.expected_mine_top_height = float(
            rospy.get_param("~expected_mine_top_height_m", 0.085)
        )
        self.mine_top_height_tolerance = float(
            rospy.get_param("~mine_top_height_tolerance_m", 0.030)
        )
        self.require_local_ground = bool(
            rospy.get_param("~require_local_ground", True)
        )
        self.validated_ground_fallback_enabled = bool(
            rospy.get_param("~validated_ground_fallback_enabled", True)
        )
        self.validated_ground_fallback_max_top_shift = float(
            rospy.get_param(
                "~validated_ground_fallback_max_top_shift_m", 0.020
            )
        )
        self.validated_ground_fallback_max_age = float(
            rospy.get_param("~validated_ground_fallback_max_age_s", 30.0)
        )
        if not 0.5 <= self.top_surface_quantile <= 0.98:
            raise rospy.ROSInitException("top_surface_quantile must be in [0.5, 0.98]")
        if not 0.001 <= self.top_surface_band <= 0.02:
            raise rospy.ROSInitException("top_surface_band_m outside 1-20 mm")
        if not 0.0 < self.ground_ring_inner_scale < self.ground_ring_outer_scale:
            raise rospy.ROSInitException("ground ring scales are invalid")
        if not 0.01 <= self.mine_top_height_tolerance <= 0.04:
            raise rospy.ROSInitException("mine top height tolerance outside 10-40 mm")
        if not 0.001 <= self.validated_ground_fallback_max_top_shift <= 0.05:
            raise rospy.ROSInitException(
                "validated ground fallback top shift outside 1-50 mm"
            )
        if not 1.0 <= self.validated_ground_fallback_max_age <= 120.0:
            raise rospy.ROSInitException(
                "validated ground fallback age outside 1-120 s"
            )

        self.minimum_frames = int(rospy.get_param("~minimum_confirmed_frames", 3))
        self.max_xy_std = float(rospy.get_param("~maximum_xy_std", 0.01))
        self.max_z_std = float(rospy.get_param("~maximum_z_std", 0.01))
        self.max_confirmation_gap = float(
            rospy.get_param("~maximum_confirmation_gap", 1.0)
        )
        self.debug_save_path = os.path.abspath(os.path.expanduser(
            rospy.get_param("~debug_save_path", "")
        )) if rospy.get_param("~debug_save_path", "") else ""
        configured_debug_directory = rospy.get_param("~debug_output_directory", "")
        if configured_debug_directory:
            self.debug_output_directory = os.path.abspath(
                os.path.expanduser(configured_debug_directory)
            )
        elif self.debug_save_path:
            self.debug_output_directory = os.path.dirname(self.debug_save_path)
        else:
            self.debug_output_directory = ""
        self.debug_save_interval = float(rospy.get_param("~debug_save_interval", 1.0))

        self.model = None
        if self.use_yolo:
            try:
                import torch
                from ultralytics import YOLO

                torch.set_num_threads(max(self.torch_threads, 1))
                try:
                    torch.set_num_interop_threads(1)
                except RuntimeError:
                    pass
                self.model = YOLO(self.model_path)
            except Exception as exc:
                raise rospy.ROSInitException(
                    "failed to load YOLO model: {}".format(exc)
                )
            if getattr(self.model, "task", None) != "segment":
                raise rospy.ROSInitException("model task must be segment")
            loaded_name = str(
                getattr(self.model, "names", {}).get(self.class_id, "")
            )
            if loaded_name and loaded_name != self.class_name:
                raise rospy.ROSInitException(
                    "class {} is {!r}, expected {!r}".format(
                        self.class_id, loaded_name, self.class_name
                    )
                )
        elif not self.color_fallback_enabled:
            raise rospy.ROSInitException(
                "color_only requires color_roi_fallback_enabled=true"
            )

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.target_pub = rospy.Publisher(
            "/mine_grasp/target_pose", PoseStamped, queue_size=1
        )
        self.observation_pub = rospy.Publisher(
            "/mine_grasp/target_observation", PoseWithCovarianceStamped,
            queue_size=1,
        )
        self.valid_pub = rospy.Publisher(
            "/mine_grasp/target_valid", Bool, queue_size=1, latch=True
        )
        self.debug_pub = rospy.Publisher(
            "/mine_grasp/debug_image", Image, queue_size=1
        )
        self.confidence_pub = rospy.Publisher(
            "/mine_grasp/confidence", Float32, queue_size=1
        )
        self.std_pub = rospy.Publisher(
            "/mine_grasp/target_std", Float32MultiArray, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            "/mine_grasp/localizer_status", String, queue_size=1, latch=True
        )
        self.reset_confirmation_service = rospy.Service(
            rospy.get_param(
                "~reset_confirmation_service",
                "/mine_grasp/reset_target_confirmation",
            ),
            Trigger,
            self._reset_confirmation_cb,
        )

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1
        )
        rospy.Subscriber(
            self.expected_source_topic,
            PoseStamped,
            self._expected_source_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~attempt_topic", "/mine_grasp/attempt_id"),
            String,
            self._attempt_cb,
            queue_size=1,
        )
        queue_size = int(rospy.get_param("~sync_queue_size", 5))
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image, queue_size=1)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=queue_size, slop=self.sync_slop,
            allow_headerless=False
        )
        self.sync.registerCallback(self._pair_cb)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate, 0.1)), self._tick)
        self._invalidate("WAITING_FOR_CAMERA", "waiting for synchronized RGB-D")
        rospy.loginfo(
            "[WristLocalizer] mode=%s model=%s conf=%.2f imgsz=%d %s + %s -> %s",
            self.localization_mode,
            self.model_path if self.use_yolo else "disabled",
            self.confidence, self.image_size,
            self.rgb_topic, self.depth_topic, self.target_frame
        )

    def _camera_info_cb(self, msg):
        with self.lock:
            self.camera_info = msg

    def _expected_source_cb(self, msg):
        # A task change must not mix confirmation frames from two mines.
        with self.processing:
            epoch = rospy.Time.now()
            with self.lock:
                self.expected_source_pose = (
                    copy.deepcopy(msg) if msg.header.frame_id else None
                )
                self.confirmation_epoch = epoch
            self.validated_ground_reference = None
            self.history.clear()
            # ``target_valid`` is latched.  Clearing only the local history left
            # the previous task's valid=True visible until another camera frame
            # completed processing, which is unsafe when an Action switches to a
            # different mine.  Invalidate synchronously with the prior change.
            self.valid_pub.publish(Bool(data=False))
            self.status_pub.publish(String(
                data="{}: selected-mine association prior changed; "
                     "waiting for fresh confirmed RGB-D frames".format(
                         TARGET_UNSTABLE
                     )
            ))

    def _attempt_cb(self, msg):
        value = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(msg.data).strip())
        if not value:
            value = "standalone"
        with self.processing:
            epoch = rospy.Time.now()
            with self.lock:
                self.attempt_id = value[:96]
                self.debug_sequence = 0
                self.confirmation_epoch = epoch
            self.validated_ground_reference = None
            # An attempt ID is also a freshness boundary.  Do not let the
            # first post-action atomic observation inherit confirmation frames
            # accumulated before the PICK request was accepted.
            self.history.clear()
            self.valid_pub.publish(Bool(data=False))
            self.status_pub.publish(String(
                data="{}: new grasp attempt; waiting for fresh RGB-D frames".format(
                    TARGET_UNSTABLE
                )
            ))

    def _reset_confirmation_cb(self, _request):
        """Start a synchronous, sensor-stamped confirmation epoch.

        ``processing`` serializes this callback with ``_tick``.  Therefore any
        in-flight inference is either fully published before the reset or its
        samples are cleared here.  Queued image pairs are filtered separately
        by ``confirmation_epoch`` before inference.
        """
        with self.processing:
            epoch = rospy.Time.now()
            with self.lock:
                self.confirmation_epoch = epoch
            self.history.clear()
            self.valid_pub.publish(Bool(data=False))
            detail = (
                "{}: confirmation epoch reset at {:.9f}; waiting for {} "
                "post-epoch RGB-D frames".format(
                    TARGET_UNSTABLE, epoch.to_sec(), self.minimum_frames
                )
            )
            self.status_pub.publish(String(data=detail))
        return TriggerResponse(success=True, message="{:.9f}".format(epoch.to_sec()))

    def _expected_source_pixel(self, stamp, camera_frame, info):
        """Project the selected UAV mine as an association prior, never a target."""
        with self.lock:
            expected = (
                None
                if self.expected_source_pose is None
                else copy.deepcopy(self.expected_source_pose)
            )
        if expected is None:
            return None
        expected.header.stamp = stamp
        try:
            camera_point = self.tf_buffer.transform(
                expected, camera_frame, rospy.Duration(self.tf_timeout)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            rospy.logwarn_throttle(
                1.0, "[WristLocalizer] selected-mine projection TF failed: %s", exc
            )
            return None
        z = float(camera_point.pose.position.z)
        if not math.isfinite(z) or z <= self.depth_min:
            return None
        x = float(camera_point.pose.position.x)
        y = float(camera_point.pose.position.y)
        u = float(info.K[0]) * x / z + float(info.K[2])
        v = float(info.K[4]) * y / z + float(info.K[5])
        if not math.isfinite(u) or not math.isfinite(v):
            return None
        return u, v

    def _pair_cb(self, rgb, depth):
        with self.lock:
            self.latest_pair = (rgb, depth)

    def _invalidate(self, code, detail, confidence=0.0, debug=None, header=None,
                    metadata=None):
        self.history.clear()
        self.valid_pub.publish(Bool(data=False))
        self.confidence_pub.publish(Float32(data=float(confidence)))
        self.status_pub.publish(String(data="{}: {}".format(code, detail)))
        if debug is not None and header is not None:
            debug_metadata = dict(metadata or {})
            debug_metadata.update({
                "state": code,
                "detail": detail,
                "confidence": confidence,
            })
            self._publish_debug(
                debug, header, debug_metadata,
            )

    def _publish_debug(self, image, header, metadata=None):
        try:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            msg.header = header
            self.debug_pub.publish(msg)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "debug image conversion failed: %s", exc)
        if (not self.debug_output_directory
                or time.monotonic() - self.last_debug_save < self.debug_save_interval):
            return
        with self.lock:
            attempt_id = self.attempt_id
            self.debug_sequence += 1
            sequence = self.debug_sequence
        directory = os.path.join(self.debug_output_directory, attempt_id)
        os.makedirs(directory, exist_ok=True)
        stamp_text = "{:010d}_{:09d}".format(
            int(header.stamp.secs), int(header.stamp.nsecs)
        )
        root = os.path.join(
            directory, "perception_{:05d}_{}".format(sequence, stamp_text)
        )
        image_path = root + ".png"
        image_temp = root + ".writing.png"
        if not cv2.imwrite(image_temp, image):
            rospy.logwarn("[WristLocalizer] failed to save %s", image_temp)
            return
        os.replace(image_temp, image_path)
        payload = {
            "attempt_id": attempt_id,
            "sequence": sequence,
            "image_stamp": header.stamp.to_sec(),
            "frame_id": header.frame_id,
            "wall_time": time.time(),
            "image_path": image_path,
        }
        if metadata:
            payload.update(metadata)
        json_path = root + ".json"
        json_temp = json_path + ".writing"
        with open(json_temp, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(json_temp, json_path)
        self.last_debug_save = time.monotonic()

    def _tick(self, _event):
        if not self.processing.acquire(False):
            return
        try:
            with self.lock:
                pair = self.latest_pair
                info = self.camera_info
            if pair is None or info is None:
                self._invalidate("WAITING_FOR_CAMERA", "missing RGB-D or CameraInfo")
                return
            rgb_msg, depth_msg = pair
            if rgb_msg.header.stamp == self.last_processed_stamp:
                return
            self.last_processed_stamp = rgb_msg.header.stamp
            with self.lock:
                confirmation_epoch = self.confirmation_epoch
            if rgb_msg.header.stamp <= confirmation_epoch:
                # This pair was already queued when the executor reset the
                # stage.  It must not become one of the three confirmations.
                return
            self._process(rgb_msg, depth_msg, info)
        except Exception as exc:
            rospy.logerr_throttle(2.0, "[WristLocalizer] unexpected error: %s", exc)
            self._invalidate("INTERNAL_ERROR", str(exc))
        finally:
            self.processing.release()

    @staticmethod
    def _transform_cloud(points, transform):
        quaternion = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        rotation = tft.quaternion_matrix(quaternion)[:3, :3]
        translation = np.asarray([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float64)
        return np.asarray(points, dtype=np.float64).dot(rotation.T) + translation

    def _ground_camera_cloud(self, mine_mask, yellow_mask, localization_source,
                             depth_m, intrinsics):
        if localization_source == "YOLO+HSV":
            base = mine_mask.astype(np.uint8)
            inner_px = max(self.ground_ring_min_inner_px // 2, 3)
            outer_px = max(inner_px + 4, 28)
        else:
            base = yellow_mask.astype(np.uint8)
            _x, _y, width, height = cv2.boundingRect(base)
            scale = max(width, height, 1)
            inner_px = max(
                self.ground_ring_min_inner_px,
                int(round(scale * self.ground_ring_inner_scale)),
            )
            outer_px = min(
                self.ground_ring_max_outer_px,
                max(inner_px + 4,
                    int(round(scale * self.ground_ring_outer_scale))),
            )
        inner = cv2.dilate(
            base,
            np.ones((2 * inner_px + 1, 2 * inner_px + 1), np.uint8),
            iterations=1,
        )
        outer = cv2.dilate(
            base,
            np.ones((2 * outer_px + 1, 2 * outer_px + 1), np.uint8),
            iterations=1,
        )
        ring = (outer > 0) & (inner == 0)
        vv, uu = np.nonzero(ring)
        if len(uu) < self.ground_min_samples:
            return None, int(len(uu)), inner_px, outer_px
        zz = depth_m[vv, uu].astype(np.float64)
        valid = (
            np.isfinite(zz) & (zz >= self.depth_min) & (zz <= self.depth_max)
        )
        uu = uu[valid].astype(np.float64)
        vv = vv[valid].astype(np.float64)
        zz = zz[valid]
        if len(zz) < self.ground_min_samples:
            return None, int(len(zz)), inner_px, outer_px
        fx, fy, cx, cy = intrinsics
        points = np.column_stack((
            (uu - cx) * zz / fx,
            (vv - cy) * zz / fy,
            zz,
        ))
        points = points[np.all(np.isfinite(points), axis=1)]
        return points, int(points.shape[0]), inner_px, outer_px

    def _validated_ground_fallback(self, top_point, stamp):
        """Return a task-local ground reference for an occluded close view.

        The current yellow top is always measured from the current RGB-D
        frame.  Only the ground height may come from the most recent stable
        observation whose annulus passed the physical mine-height gate.  A
        small 3-D top displacement and a short sensor-stamp age prevent this
        from hiding a wrong mine, stale task or genuine target jump.
        """
        if not self.validated_ground_fallback_enabled:
            return None, {"ground_fallback_rejection": "disabled"}
        reference = copy.deepcopy(self.validated_ground_reference)
        if not reference:
            return None, {"ground_fallback_rejection": "no validated reference"}

        try:
            reference_top = np.asarray(
                reference["top_surface_gravity"], dtype=np.float64
            )
            reference_ground_z = float(reference["ground_z"])
            reference_stamp = float(reference["stamp"])
            age = float(stamp.to_sec()) - reference_stamp
            top_shift = float(np.linalg.norm(
                np.asarray(top_point, dtype=np.float64) - reference_top
            ))
            fallback_height = float(top_point[2] - reference_ground_z)
        except (KeyError, TypeError, ValueError, OverflowError):
            return None, {"ground_fallback_rejection": "invalid reference"}

        diagnostics = {
            "ground_fallback_reference_stamp": reference_stamp,
            "ground_fallback_reference_age_s": age,
            "ground_fallback_top_shift_m": top_shift,
            "ground_fallback_height_m": fallback_height,
        }
        values = (reference_ground_z, age, top_shift, fallback_height)
        if not all(math.isfinite(value) for value in values):
            diagnostics["ground_fallback_rejection"] = "non-finite geometry"
            return None, diagnostics
        if age < 0.0 or age > self.validated_ground_fallback_max_age:
            diagnostics["ground_fallback_rejection"] = "reference is stale"
            return None, diagnostics
        if top_shift > self.validated_ground_fallback_max_top_shift:
            diagnostics["ground_fallback_rejection"] = "current top moved too far"
            return None, diagnostics
        if (abs(fallback_height - self.expected_mine_top_height)
                > self.mine_top_height_tolerance):
            diagnostics["ground_fallback_rejection"] = (
                "current top is inconsistent with validated ground"
            )
            return None, diagnostics
        diagnostics["ground_fallback_rejection"] = ""
        return reference_ground_z, diagnostics

    def _remember_direct_ground_reference(self, metadata, stamp):
        """Remember only a stable, directly fitted ground observation."""
        if metadata.get("ground_validation_source") != "local_ring":
            return
        with self.lock:
            confirmation_epoch = float(self.confirmation_epoch.to_sec())
        current = self.validated_ground_reference
        if (current is not None
                and current.get("confirmation_epoch") == confirmation_epoch):
            # Freeze the first stable direct fit in this settled-stage epoch.
            # Frames seen while the arm starts its next motion must not move
            # the ground anchor toward the fingers entering the annulus.
            return
        try:
            top = [float(value) for value in metadata["top_surface_gravity"]]
            ground_z = float(metadata["ground_z"])
            stamp_sec = float(stamp.to_sec())
        except (KeyError, TypeError, ValueError, OverflowError):
            return
        if len(top) != 3 or not all(math.isfinite(value) for value in (
                top[0], top[1], top[2], ground_z, stamp_sec)):
            return
        self.validated_ground_reference = {
            "top_surface_gravity": top,
            "ground_z": ground_z,
            "stamp": stamp_sec,
            "confirmation_epoch": confirmation_epoch,
        }

    def _candidate_target(self, candidate, depth_m, intrinsics, stamp,
                          camera_frame):
        """Resolve one ranked colour component into a safe 3-D target.

        Candidate ranking is deliberately kept separate from validation.  A
        closer/larger yellow blob is only a preference; it must not prevent a
        later component with valid registered depth and workspace geometry from
        being considered.
        """
        (_, _, mine_mask, yellow_mask, _, _, _, _, localization_source) = candidate
        camera_cloud, sample_count = robust_mask_cloud(
            yellow_mask, depth_m, intrinsics, self.depth_min, self.depth_max,
            self.erode_pixels, self.min_depth_samples, self.mad_scale,
            self.min_mad_gate
        )
        if camera_cloud is None:
            return (
                None,
                sample_count,
                INVALID_DEPTH,
                "only {} robust yellow depth samples".format(sample_count),
                {},
            )

        try:
            # Transform every physical pixel/depth tuple at the image stamp;
            # taking a camera-frame median first is invalid on slopes/oblique
            # views because it creates a synthetic ray.
            transform = self.tf_buffer.lookup_transform(
                self.gravity_frame, camera_frame, stamp,
                rospy.Duration(self.tf_timeout),
            )
            gravity_cloud = self._transform_cloud(camera_cloud, transform)
            top_point, top_count = estimate_top_surface(
                gravity_cloud,
                self.top_surface_quantile,
                self.top_surface_band,
                self.top_surface_min_samples,
            )
            if top_point is None:
                return (
                    None, sample_count, INVALID_DEPTH,
                    "top point band has only {} samples".format(top_count),
                    {"top_band_samples": top_count},
                )
            ground_camera, ground_raw_count, inner_px, outer_px = (
                self._ground_camera_cloud(
                    mine_mask, yellow_mask, localization_source,
                    depth_m, intrinsics,
                )
            )
            ground_z = None
            ground_inliers = 0
            if ground_camera is not None:
                ground_gravity = self._transform_cloud(ground_camera, transform)
                ground_z, ground_inliers = fit_local_ground_plane(
                    ground_gravity,
                    top_point[:2],
                    self.ground_min_samples,
                    self.ground_plane_residual_floor,
                    self.ground_plane_mad_scale,
                    self.ground_plane_min_xy_span,
                )
            metadata = {
                "localization_source": localization_source,
                "yellow_depth_samples": sample_count,
                "top_band_samples": top_count,
                "top_surface_gravity": [float(value) for value in top_point],
                "ground_raw_samples": ground_raw_count,
                "ground_inliers": ground_inliers,
                "ground_ring_inner_px": inner_px,
                "ground_ring_outer_px": outer_px,
                "ground_z": ground_z,
                "ground_validation_source": (
                    "local_ring" if ground_z is not None else "unavailable"
                ),
            }
            if ground_z is None and self.require_local_ground:
                fallback_ground_z, fallback_metadata = (
                    self._validated_ground_fallback(top_point, stamp)
                )
                metadata.update(fallback_metadata)
                if fallback_ground_z is None:
                    return (
                        None, sample_count, INVALID_DEPTH,
                        "local ground plane unavailable ({} ring samples)".format(
                            ground_raw_count
                        ), metadata,
                    )
                ground_z = fallback_ground_z
                metadata["ground_z"] = ground_z
                metadata["ground_validation_source"] = "validated_prior_stage"
            height_above_ground = (
                None if ground_z is None else float(top_point[2] - ground_z)
            )
            metadata["top_height_above_ground_m"] = height_above_ground
            if (height_above_ground is not None
                    and abs(height_above_ground - self.expected_mine_top_height)
                    > self.mine_top_height_tolerance):
                rejected_ground_z = ground_z
                rejected_height = height_above_ground
                fallback_ground_z, fallback_metadata = (
                    self._validated_ground_fallback(top_point, stamp)
                )
                metadata.update(fallback_metadata)
                if fallback_ground_z is None:
                    return (
                        None, sample_count, NO_DETONATOR,
                        "detonator top height {:.3f} m differs from expected {:.3f} "
                        "by more than {:.3f} m".format(
                            height_above_ground,
                            self.expected_mine_top_height,
                            self.mine_top_height_tolerance,
                        ), metadata,
                    )
                ground_z = fallback_ground_z
                height_above_ground = float(top_point[2] - ground_z)
                metadata.update({
                    "rejected_local_ground_z": rejected_ground_z,
                    "rejected_local_top_height_m": rejected_height,
                    "ground_z": ground_z,
                    "top_height_above_ground_m": height_above_ground,
                    "ground_validation_source": "validated_prior_stage",
                })
                rospy.logwarn_throttle(
                    1.0,
                    "[WristLocalizer] rejected occluded close-view ground "
                    "z=%.3f (top height %.3f); using task-local validated "
                    "ground z=%.3f after %.3f m top shift",
                    rejected_ground_z,
                    rejected_height,
                    ground_z,
                    float(fallback_metadata["ground_fallback_top_shift_m"]),
                )
            top_gravity = PoseStamped()
            top_gravity.header.stamp = stamp
            top_gravity.header.frame_id = self.gravity_frame
            top_gravity.pose.position.x = float(top_point[0])
            top_gravity.pose.position.y = float(top_point[1])
            top_gravity.pose.position.z = float(top_point[2])
            top_gravity.pose.orientation.w = 1.0
            centre_gravity = PoseStamped()
            centre_gravity.header = copy.deepcopy(top_gravity.header)
            centre_gravity.pose = copy.deepcopy(top_gravity.pose)
            centre_gravity.pose.position.z -= self.detonator_height * 0.5
            centre_target = self.tf_buffer.transform(
                centre_gravity, self.target_frame, rospy.Duration(self.tf_timeout)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            return None, sample_count, TF_FAILED, str(exc), {}

        xyz = (
            float(centre_target.pose.position.x),
            float(centre_target.pose.position.y),
            float(centre_target.pose.position.z),
        )
        if not all(math.isfinite(value) for value in xyz):
            return (
                None,
                sample_count,
                TF_FAILED,
                "candidate transform produced NaN or Inf",
                metadata,
            )

        if localization_source == "COLOR_ROI":
            planar_reach = math.hypot(xyz[0], xyz[1])
            if (not self.color_fallback_min_reach <= planar_reach
                    <= self.color_fallback_max_reach
                    or not self.color_fallback_min_z <= xyz[2]
                    <= self.color_fallback_max_z):
                return (
                    None,
                    sample_count,
                    NO_DETONATOR,
                    "colour candidate outside wrist workspace: "
                    "reach={:.3f} z={:.3f}".format(planar_reach, xyz[2]),
                    metadata,
                )

        # target_pose is a measured point, not a commanded tool attitude.
        # Publish an explicit neutral quaternion so camera orientation can never
        # leak into the analytic grasp-pose generator.
        centre_target.pose.orientation.x = 0.0
        centre_target.pose.orientation.y = 0.0
        centre_target.pose.orientation.z = 0.0
        centre_target.pose.orientation.w = 1.0
        return centre_target, sample_count, "", "", metadata

    def _process(self, rgb_msg, depth_msg, info):
        stamp = rgb_msg.header.stamp
        if (rospy.Time.now() - stamp).to_sec() > self.max_frame_age:
            self._invalidate("STALE_FRAME", "RGB frame is too old")
            return
        if abs((rgb_msg.header.stamp - depth_msg.header.stamp).to_sec()) > self.sync_slop:
            self._invalidate("UNALIGNED_RGBD", "RGB/depth stamp delta exceeds slop")
            return
        if rgb_msg.header.frame_id != depth_msg.header.frame_id:
            self._invalidate(
                "UNALIGNED_RGBD", "RGB/depth frame IDs differ: {} vs {}".format(
                    rgb_msg.header.frame_id, depth_msg.header.frame_id
                )
            )
            return
        if self.camera_frame and rgb_msg.header.frame_id != self.camera_frame:
            self._invalidate(
                "CAMERA_FRAME_MISMATCH", "received {}, configured {}".format(
                    rgb_msg.header.frame_id, self.camera_frame
                )
            )
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            depth_m = depth_to_metres(depth_raw, depth_msg.encoding)
        except (CvBridgeError, ValueError) as exc:
            self._invalidate(INVALID_DEPTH, str(exc))
            return

        if rgb.shape[:2] != depth_m.shape[:2]:
            self._invalidate(
                "UNALIGNED_RGBD", "RGB {} and depth {} dimensions differ".format(
                    rgb.shape[:2], depth_m.shape[:2]
                )
            )
            return
        if info.width != rgb.shape[1] or info.height != rgb.shape[0]:
            self._invalidate("CAMERA_INFO_MISMATCH", "CameraInfo dimensions differ")
            return

        debug = rgb.copy()
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        yellow_all = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper) > 0
        expected_pixel = self._expected_source_pixel(
            stamp, rgb_msg.header.frame_id, info
        )
        # Each entry is (rank, candidate).  Rank establishes preference only;
        # depth/TF/workspace validation is performed for every candidate below.
        candidate_options = []
        color_debug_metadata = None
        inference_rgb = None
        viewport = None
        if self.use_yolo:
            inference_rgb, viewport = self._scaled_inference_view(rgb)
            results = self.model.predict(
                source=inference_rgb,
                task="segment",
                classes=[self.class_id],
                conf=self.confidence,
                iou=self.iou,
                imgsz=self.image_size,
                device=self.device,
                max_det=self.max_det,
                verbose=False,
            )
            result = results[0]
            has_yolo = (
                result.boxes is not None
                and result.masks is not None
                and len(result.boxes) > 0
            )
            if has_yolo:
                masks = result.masks.data.detach().cpu().numpy()
                confidences = result.boxes.conf.detach().cpu().numpy()
                classes = result.boxes.cls.detach().cpu().numpy().astype(int)
            else:
                masks = np.empty((0, 1, 1), dtype=np.float32)
                confidences = np.empty((0,), dtype=np.float32)
                classes = np.empty((0,), dtype=np.int32)
        else:
            has_yolo = False
            masks = np.empty((0, 1, 1), dtype=np.float32)
            confidences = np.empty((0,), dtype=np.float32)
            classes = np.empty((0,), dtype=np.int32)

        for index, (mask_float, confidence, class_id) in enumerate(
                zip(masks, confidences, classes)):
            if class_id != self.class_id:
                continue
            if mask_float.shape != inference_rgb.shape[:2]:
                mask_float = cv2.resize(
                    mask_float, (inference_rgb.shape[1], inference_rgb.shape[0]),
                    interpolation=cv2.INTER_NEAREST
                )
            mask_float = self._mask_to_original(mask_float, viewport, rgb.shape[:2])
            mine_mask = mask_float > 0.5
            mine_area = int(np.count_nonzero(mine_mask))
            if mine_area == 0:
                continue

            # Crucial safety rule: yellow pixels outside the YOLO mine mask are
            # never considered a detonator candidate.
            yellow = (yellow_all & mine_mask).astype(np.uint8)
            yellow = cv2.morphologyEx(
                yellow, cv2.MORPH_OPEN,
                np.ones((self.open_kernel, self.open_kernel), np.uint8)
            )
            yellow = cv2.morphologyEx(
                yellow, cv2.MORPH_CLOSE,
                np.ones((self.close_kernel, self.close_kernel), np.uint8)
            )
            for component, centroid, area in valid_components(
                    yellow, self.min_yellow_area, self.max_yellow_area):
                fraction = float(area) / float(mine_area)
                if not self.min_yellow_fraction <= fraction <= self.max_yellow_fraction:
                    continue
                score = float(confidence) + min(fraction, 0.2)
                candidate = (
                    score,
                    float(confidence),
                    mine_mask,
                    component,
                    centroid,
                    area,
                    fraction,
                    index,
                    "YOLO+HSV",
                )
                # Lower rank wins, hence the negative confidence/fraction score.
                candidate_options.append((-score, candidate))

        if not candidate_options and self.color_fallback_enabled:
            height, width = rgb.shape[:2]
            x0 = int(round(self.color_fallback_roi[0] * width))
            y0 = int(round(self.color_fallback_roi[1] * height))
            x1 = int(round(self.color_fallback_roi[2] * width))
            y1 = int(round(self.color_fallback_roi[3] * height))
            roi_mask = np.zeros((height, width), dtype=np.uint8)
            roi_mask[y0:y1, x0:x1] = 1
            yellow = (yellow_all & (roi_mask > 0)).astype(np.uint8)
            yellow = cv2.morphologyEx(
                yellow, cv2.MORPH_OPEN,
                np.ones((self.open_kernel, self.open_kernel), np.uint8)
            )
            yellow = cv2.morphologyEx(
                yellow, cv2.MORPH_CLOSE,
                np.ones((self.close_kernel, self.close_kernel), np.uint8)
            )
            color_debug_metadata = color_roi_debug_metadata(
                yellow_all,
                roi_mask,
                yellow,
                expected_pixel,
                (x0, y0, x1, y1),
                self.expected_source_roi_radius,
            )
            # Draw the configured ROI and map-association prior even when no
            # yellow component survives.  Previously the exact frames that
            # failed NO_DETONATOR omitted both, hiding out-of-view failures.
            cv2.rectangle(
                debug, (x0, y0), (x1 - 1, y1 - 1), (0, 180, 255), 1
            )
            if expected_pixel is not None:
                centre = (
                    int(round(expected_pixel[0])),
                    int(round(expected_pixel[1])),
                )
                cv2.circle(
                    debug, centre, self.expected_source_roi_radius,
                    (255, 0, 255), 1,
                )
                cv2.drawMarker(
                    debug, centre, (255, 0, 255), cv2.MARKER_TILTED_CROSS,
                    14, 2,
                )
            components = valid_components(
                yellow,
                self.color_fallback_min_area,
                self.color_fallback_max_area,
            )
            valid = []
            for component, centroid, area in components:
                bx, by, bw, bh = cv2.boundingRect(component.astype(np.uint8))
                aspect = float(bw) / float(max(bh, 1))
                fill = float(area) / float(max(bw * bh, 1))
                if (self.color_fallback_min_aspect <= aspect
                        <= self.color_fallback_max_aspect
                        and fill >= self.color_fallback_min_fill):
                    if expected_pixel is None:
                        rank = -float(area)
                    else:
                        rank = math.hypot(
                            centroid[0] - expected_pixel[0],
                            centroid[1] - expected_pixel[1],
                        )
                    valid.append(
                        (rank, component, centroid, area, fill, aspect)
                    )
            for rank, component, centroid, area, fill, _ in valid:
                support = cv2.dilate(
                    component.astype(np.uint8),
                    np.ones((15, 15), np.uint8),
                    iterations=1,
                ) > 0
                fraction = float(area) / float(max((x1 - x0) * (y1 - y0), 1))
                candidate = (
                    self.color_fallback_confidence + min(fill, 1.0) * 0.1,
                    self.color_fallback_confidence,
                    support,
                    component,
                    centroid,
                    area,
                    fraction,
                    -1,
                    "COLOR_ROI",
                )
                candidate_options.append((rank, candidate))

        if not candidate_options:
            for mask_float in masks:
                if mask_float.shape != inference_rgb.shape[:2]:
                    mask_float = cv2.resize(mask_float, (inference_rgb.shape[1], inference_rgb.shape[0]),
                                            interpolation=cv2.INTER_NEAREST)
                mask_float = self._mask_to_original(mask_float, viewport, rgb.shape[:2])
                contours, _ = cv2.findContours((mask_float > 0.5).astype(np.uint8),
                                                cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(debug, contours, -1, (255, 80, 0), 2)
            code = (
                NO_DETONATOR
                if has_yolo or self.localization_mode == "color_only"
                else NO_DETECTION
            )
            if self.localization_mode == "color_only":
                raw_count = (
                    color_debug_metadata["yellow_pixels_roi_raw"]
                    if color_debug_metadata is not None else 0
                )
                filtered_count = (
                    color_debug_metadata["yellow_pixels_roi_filtered"]
                    if color_debug_metadata is not None else 0
                )
                if raw_count == 0:
                    detail = (
                        "colour ROI contains no HSV-yellow pixels; detonator "
                        "is outside/occluded or its colour is absent"
                    )
                elif filtered_count == 0:
                    detail = (
                        "HSV-yellow pixels in colour ROI did not survive "
                        "morphology/area gating"
                    )
                else:
                    detail = (
                        "colour ROI yellow components failed bounded "
                        "area/shape gates"
                    )
            elif has_yolo:
                detail = "no valid yellow component inside mine mask/colour ROI"
            else:
                detail = "YOLO found no landmine and colour ROI found no safe detonator"
            cv2.putText(debug, code, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)
            self._invalidate(code, detail,
                             confidence=(float(np.max(confidences))
                                         if len(confidences) else 0.0), debug=debug,
                             header=rgb_msg.header,
                             metadata=color_debug_metadata)
            return

        intrinsics = (float(info.K[0]), float(info.K[4]),
                      float(info.K[2]), float(info.K[5]))
        ordered_candidates = [
            candidate for _, candidate in sorted(
                candidate_options, key=lambda item: item[0]
            )
        ]
        selected = None
        failures = []
        for candidate in ordered_candidates:
            (centre_target, sample_count, failure_code, failure_detail,
             geometry_metadata) = (
                self._candidate_target(
                    candidate,
                    depth_m,
                    intrinsics,
                    stamp,
                    rgb_msg.header.frame_id,
                )
            )
            if centre_target is not None:
                selected = (
                    candidate, centre_target, sample_count, geometry_metadata
                )
                break
            failures.append((
                candidate, failure_code, failure_detail, sample_count,
                geometry_metadata,
            ))

        if selected is None:
            # Every ranked component was evaluated.  Report the highest-ranked
            # failure while making the attempted candidate count explicit.
            candidate, code, detail, _, geometry_metadata = failures[0]
            (_, confidence, mine_mask, yellow_mask, centroid, _, _, _, _) = candidate
            self._draw_masks(debug, mine_mask, yellow_mask, centroid)
            detail = "{}; rejected all {} colour candidate(s)".format(
                detail, len(failures)
            )
            self._invalidate(
                code,
                detail + "; geometry=" + json.dumps(
                    geometry_metadata, ensure_ascii=False, sort_keys=True
                ),
                confidence=confidence,
                debug=debug,
                header=rgb_msg.header,
            )
            return

        best, centre_target, sample_count, geometry_metadata = selected
        (_, confidence, mine_mask, yellow_mask, centroid, yellow_area,
         fraction, _, localization_source) = best

        point = np.asarray([
            centre_target.pose.position.x,
            centre_target.pose.position.y,
            centre_target.pose.position.z,
        ], dtype=np.float64)
        now_sec = stamp.to_sec()
        if self.history and now_sec - self.history[-1][0] > self.max_confirmation_gap:
            self.history.clear()
        self.history.append((now_sec, point, confidence))
        while len(self.history) > self.minimum_frames:
            self.history.popleft()

        positions = np.asarray([entry[1] for entry in self.history])
        std = np.std(positions, axis=0) if len(positions) > 1 else np.full(3, np.inf)
        stable = (
            len(self.history) >= self.minimum_frames
            and float(max(std[0], std[1])) <= self.max_xy_std
            and float(std[2]) <= self.max_z_std
        )

        self._draw_masks(debug, mine_mask, yellow_mask, centroid)
        colour = (0, 200, 0) if stable else (0, 165, 255)
        height_text = geometry_metadata.get("top_height_above_ground_m")
        ground_source = geometry_metadata.get(
            "ground_validation_source", "unknown"
        )
        text = "{} src={} conf={:.3f} n={} xyz=({:.3f},{:.3f},{:.3f}) std=({:.3f},{:.3f},{:.3f}) top_h={} ground={}".format(
            "VALID" if stable else TARGET_UNSTABLE, localization_source,
            confidence, len(self.history),
            point[0], point[1], point[2], std[0], std[1], std[2],
            "?" if height_text is None else "{:.3f}".format(height_text),
            ground_source,
        )
        cv2.putText(debug, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, colour, 1, cv2.LINE_AA)
        cv2.putText(debug, "yellow_px={} depth_inliers={}".format(
            yellow_area, sample_count), (10, 48), cv2.FONT_HERSHEY_SIMPLEX,
            0.45, colour, 1, cv2.LINE_AA)
        debug_metadata = dict(geometry_metadata)
        debug_metadata.update({
            "state": "VALID" if stable else TARGET_UNSTABLE,
            "confidence": float(confidence),
            "confirmed_frames": len(self.history),
            "target_xyz": [float(value) for value in point],
            "target_std": [float(value) for value in std],
            "yellow_area_px": int(yellow_area),
            "yellow_depth_inliers": int(sample_count),
        })
        self._publish_debug(debug, rgb_msg.header, debug_metadata)
        self.confidence_pub.publish(Float32(data=confidence))
        self.std_pub.publish(Float32MultiArray(data=[
            float(std[0]), float(std[1]), float(std[2])
        ]))

        if not stable:
            self.valid_pub.publish(Bool(data=False))
            self.status_pub.publish(String(data=text))
            return

        median = np.median(positions, axis=0)
        centre_target.pose.position.x = float(median[0])
        centre_target.pose.position.y = float(median[1])
        centre_target.pose.position.z = float(median[2])
        centre_target.header.stamp = stamp
        observation = PoseWithCovarianceStamped()
        observation.header = copy.deepcopy(centre_target.header)
        observation.pose.pose = copy.deepcopy(centre_target.pose)
        observation.pose.covariance = [0.0] * 36
        for index, value in zip((0, 7, 14), std):
            observation.pose.covariance[index] = max(float(value) ** 2, 1e-10)
        # The target is a point; its neutral quaternion is not an observed mine
        # attitude.  Mark orientation as deliberately uninformative.
        for index in (21, 28, 35):
            observation.pose.covariance[index] = 1.0e3
        # Update the fallback anchor only from a stable direct ring fit.  A
        # fallback-confirmed close view may publish its newly measured top, but
        # cannot recursively refresh or drift the ground reference.
        self._remember_direct_ground_reference(geometry_metadata, stamp)
        self.observation_pub.publish(observation)
        self.target_pub.publish(centre_target)
        self.valid_pub.publish(Bool(data=True))
        self.status_pub.publish(String(data=text))

    @staticmethod
    def _draw_masks(image, mine_mask, yellow_mask, centroid):
        overlay = image.copy()
        overlay[mine_mask] = (255, 80, 0)
        overlay[yellow_mask] = (0, 255, 255)
        cv2.addWeighted(overlay, 0.35, image, 0.65, 0.0, dst=image)
        contours, _ = cv2.findContours(mine_mask.astype(np.uint8),
                                        cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (255, 120, 0), 2)
        contours, _ = cv2.findContours(yellow_mask.astype(np.uint8),
                                        cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(image, contours, -1, (0, 255, 255), 2)
        if centroid is not None:
            cv2.drawMarker(image, (int(round(centroid[0])), int(round(centroid[1]))),
                           (0, 0, 255), cv2.MARKER_CROSS, 14, 2)

    def _scaled_inference_view(self, image):
        """Scale the wrist view into a same-size canvas and return its viewport."""
        height, width = image.shape[:2]
        if abs(self.view_scale - 1.0) < 1e-6:
            return image, (0, 0, width, height)
        scaled_width = max(int(round(width * self.view_scale)), 1)
        scaled_height = max(int(round(height * self.view_scale)), 1)
        scaled = cv2.resize(
            image, (scaled_width, scaled_height), interpolation=cv2.INTER_AREA
        )
        x0 = (width - scaled_width) // 2
        y0 = (height - scaled_height) // 2
        canvas = np.full_like(image, self.canvas_value)
        canvas[y0:y0 + scaled_height, x0:x0 + scaled_width] = scaled
        return canvas, (x0, y0, scaled_width, scaled_height)

    @staticmethod
    def _mask_to_original(mask, viewport, output_shape):
        x0, y0, width, height = viewport
        cropped = mask[y0:y0 + height, x0:x0 + width]
        output_height, output_width = output_shape
        return cv2.resize(
            cropped, (output_width, output_height), interpolation=cv2.INTER_NEAREST
        )


def main():
    rospy.init_node("wrist_mine_localizer")
    WristMineLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
