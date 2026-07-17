#!/usr/bin/env python3
"""Run YOLO instance segmentation on the UAV down camera and localize mines.

The node never consumes Gazebo model truth.  Each segmentation mask is combined
with the synchronized depth image, unprojected with CameraInfo, and transformed
to ``map`` at the image timestamp.  The output is intentionally frame-local;
multi-frame confirmation and persistent map storage live in
``mine_map_fusion_node.py``.
"""

import json
import math
import os
import threading
import time

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, SetBoolResponse
from mobile_manipulator.srv import DetectLandmines, DetectLandminesRequest

class _Scalar:
    def __init__(self, value): self.value = value
    def item(self): return self.value

class _TRTBoxes:
    def __init__(self, detections):
        self.detections = detections
        self.cls = [_Scalar(d.class_id) for d in detections]
        self.conf = [_Scalar(d.confidence) for d in detections]

    def __len__(self):
        return len(self.detections)

class _TRTMaskData:
    def __init__(self, masks): self._masks = masks
    def detach(self): return self
    def cpu(self): return self
    def numpy(self): return np.asarray(self._masks)

class _TRTMasks:
    def __init__(self, masks): self.data = _TRTMaskData(masks)

class _TRTResult:
    def __init__(self, detections, masks):
        self.boxes = _TRTBoxes(detections)
        self.masks = _TRTMasks(masks)

from uav_truth_tracker.msg import MineDetection, MineDetectionArray


class MineSegLocalizer:
    def __init__(self):
        self.model_path = os.path.abspath(
            os.path.expanduser(rospy.get_param("~model_path", ""))
        )
        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/mine_camera/rgb/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/mine_camera/depth/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/mine_camera/rgb/camera_info"
        )
        self.camera_frame_override = rospy.get_param(
            "~camera_frame", "mine_camera_optical_frame"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.output_topic = rospy.get_param(
            "~output_topic", "/mine_detection/raw"
        )
        self.debug_image_topic = rospy.get_param(
            "~debug_image_topic", "/mine_detection/debug_image"
        )
        self.status_topic = rospy.get_param(
            "~status_topic", "/mine_detection/localizer_status"
        )
        self.inference_time_topic = rospy.get_param(
            "~inference_time_topic", "/mine_detection/inference_ms"
        )
        self.pause_after_survey_complete = bool(
            rospy.get_param("~pause_after_survey_complete", False)
        )
        self.survey_status_topic = rospy.get_param(
            "~survey_status_topic", "/mine_survey/status"
        )

        self.confidence = min(
            max(float(rospy.get_param("~confidence", 0.65)), 0.01), 0.99
        )
        self.iou_threshold = min(
            max(float(rospy.get_param("~iou_threshold", 0.50)), 0.05), 0.95
        )
        self.imgsz = max(int(rospy.get_param("~imgsz", 960)), 64)
        self.inference_rate = max(
            float(rospy.get_param("~inference_rate", 1.0)), 0.1
        )
        self.manipulation_inference_rate = max(
            float(rospy.get_param("~manipulation_inference_rate", 0.25)), 0.25
        )
        self.timer_rate = max(
            float(rospy.get_param("~processing_timer_rate", 4.0)),
            self.inference_rate,
        )
        self.goal_rate_restore_duration = max(
            float(rospy.get_param("~goal_rate_restore_duration", 5.0)), 1.0
        )
        self.device = str(rospy.get_param("~device", "cpu"))
        self.inference_backend = str(rospy.get_param("~inference_backend", "ultralytics")).lower()
        self.torch_threads = max(int(rospy.get_param("~torch_threads", 4)), 1)
        self.max_detections = max(int(rospy.get_param("~max_detections", 20)), 1)
        self.target_class_id = int(rospy.get_param("~target_class_id", 0))
        self.target_class_name = str(
            rospy.get_param("~target_class_name", "landmine")
        ).lower()

        self.sync_slop = min(
            max(float(rospy.get_param("~sync_slop", 0.05)), 0.0), 0.20
        )
        # ApproximateTimeSynchronizer is only a queueing aid.  Geometry is
        # accepted with a much tighter, explicit RGB/depth time gate below so a
        # moving UAV never combines a colour mask with depth from another pose.
        self.max_rgb_depth_stamp_delta = min(
            max(
                float(rospy.get_param("~max_rgb_depth_stamp_delta", 0.02)),
                0.0,
            ),
            self.sync_slop,
        )
        self.depth_min = max(float(rospy.get_param("~depth_min", 0.20)), 0.01)
        self.depth_max = max(
            float(rospy.get_param("~depth_max", 20.0)), self.depth_min
        )
        self.min_depth_samples = max(
            int(rospy.get_param("~min_depth_samples", 12)), 1
        )
        self.min_mask_area = max(int(rospy.get_param("~min_mask_area", 20)), 1)
        self.erode_pixels = max(int(rospy.get_param("~erode_pixels", 1)), 0)
        self.depth_mad_scale = max(
            float(rospy.get_param("~depth_mad_scale", 4.0)), 1.0
        )
        self.depth_mad_floor = max(
            float(rospy.get_param("~depth_mad_floor", 0.03)), 0.001
        )
        # Validate detections against the ground surrounding the segmented mine,
        # not against an absolute map z.  An absolute z gate silently discards
        # every otherwise valid mine placed on a hill.
        self.ground_ring_inner_px = max(
            int(rospy.get_param("~ground_ring_inner_px", 3)), 1
        )
        self.ground_ring_outer_px = max(
            int(rospy.get_param("~ground_ring_outer_px", 18)),
            self.ground_ring_inner_px + 2,
        )
        self.ground_min_samples = max(
            int(rospy.get_param("~ground_min_samples", 20)), 5
        )
        self.min_height_above_ground = float(
            rospy.get_param("~min_height_above_ground", -0.06)
        )
        self.max_height_above_ground = float(
            rospy.get_param("~max_height_above_ground", 0.18)
        )
        self.require_local_ground = bool(
            rospy.get_param("~require_local_ground", False)
        )
        self.ground_plane_mad_scale = max(
            float(rospy.get_param("~ground_plane_mad_scale", 4.0)), 1.0
        )
        self.ground_plane_residual_floor = max(
            float(rospy.get_param("~ground_plane_residual_floor", 0.025)),
            0.001,
        )
        self.ground_plane_min_xy_span = max(
            float(rospy.get_param("~ground_plane_min_xy_span", 0.04)), 0.005
        )
        if self.min_height_above_ground >= self.max_height_above_ground:
            raise rospy.ROSInitException(
                "min_height_above_ground must be below max_height_above_ground"
            )
        self.tf_timeout = max(float(rospy.get_param("~tf_timeout", 0.15)), 0.0)
        self.tf_cache_time = max(
            float(rospy.get_param("~tf_cache_time", 60.0)),
            self.tf_timeout + 1.0,
        )
        self.allow_latest_tf = bool(
            rospy.get_param("~allow_latest_tf_fallback", False)
        )
        if self.allow_latest_tf:
            rospy.logwarn(
                "[MineLocalizer] ~allow_latest_tf_fallback is deprecated and "
                "ignored; mine geometry always uses the depth-image timestamp"
            )

        if self.inference_backend == "tensorrt":
            rospy.loginfo("[MineLocalizer] using TensorRT C++ service /landmine/detect")
            rospy.wait_for_service("/landmine/detect", timeout=30.0)
            self.trt_detect = rospy.ServiceProxy("/landmine/detect", DetectLandmines)
            self.model = None
        elif not self.model_path:
            raise rospy.ROSInitException("~model_path is required")
        if not os.path.isfile(self.model_path):
            raise rospy.ROSInitException(
                "landmine segmentation model not found: {}".format(self.model_path)
            )

        if self.inference_backend == "tensorrt":
            pass
        else:
          try:
            import torch
            from ultralytics import YOLO

            torch.set_num_threads(self.torch_threads)
            try:
                torch.set_num_interop_threads(1)
            except RuntimeError:
                pass
            self.model = YOLO(self.model_path)
          except Exception as exc:
            raise rospy.ROSInitException(
                "failed to load Ultralytics model {}: {}".format(
                    self.model_path, exc
                )
            )

        if self.inference_backend != "tensorrt" and getattr(self.model, "task", None) != "segment":
            raise rospy.ROSInitException(
                "model task must be 'segment', got {!r}".format(
                    getattr(self.model, "task", None)
                )
            )

        names = getattr(self.model, "names", {}) if self.model is not None else {}
        loaded_name = str(names.get(self.target_class_id, "")).lower()
        if loaded_name and loaded_name != self.target_class_name:
            rospy.logwarn(
                "[MineLocalizer] class %d is %r, configured name is %r; class id wins",
                self.target_class_id,
                loaded_name,
                self.target_class_name,
            )

        self.bridge = CvBridge()
        # CPU inference can take well over ten seconds in the complete
        # simulation.  Keep enough TF history to query the *depth-image* stamp;
        # increasing this cache is safe and is not a latest-TF fallback.
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rospy.Duration(self.tf_cache_time)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_info = None
        self.latest_pair = None
        self.latest_lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.last_processed_stamp = None
        self.tf_fallback_count = 0
        self.sync_rejected_frames = 0
        self.invalid_timestamp_frames = 0
        self.processed_frames = 0
        self.inference_enabled = bool(
            rospy.get_param("~enabled", True)
        )
        self.manipulation_active = False
        self.goal_rate_restore_until = 0.0
        self.last_goal_signatures = {}
        self.last_inference_wall = 0.0
        self.last_debug_stamp = None

        self.detection_pub = rospy.Publisher(
            self.output_topic, MineDetectionArray, queue_size=2
        )
        self.debug_pub = rospy.Publisher(
            self.debug_image_topic, Image, queue_size=1
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=2, latch=True
        )
        self.inference_time_pub = rospy.Publisher(
            self.inference_time_topic, Float32, queue_size=2
        )
        rospy.Subscriber(
            self.survey_status_topic,
            String,
            self._survey_status_cb,
            queue_size=1,
        )
        self.manual_goal_topic = rospy.get_param(
            "~manual_goal_topic", "/uav/manual_goal"
        )
        self.survey_goal_topic = rospy.get_param(
            "~survey_goal_topic", "/uav/survey_goal"
        )
        for topic in rospy.get_param(
                "~goal_topics",
                [self.manual_goal_topic, self.survey_goal_topic]):
            rospy.Subscriber(
                topic,
                PoseStamped,
                self._goal_cb,
                callback_args=str(topic),
                queue_size=2,
            )
        rospy.Subscriber(
            rospy.get_param(
                "~grasp_status_topic", "/mine_grasp/executor_status"
            ),
            String,
            self._grasp_status_cb,
            queue_size=5,
        )
        rospy.Service(
            "/mine_detection/set_enabled", SetBool, self._set_enabled_cb
        )

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_info_cb, queue_size=1
        )
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image, queue_size=1)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=4, slop=self.sync_slop,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._sync_cb)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.timer_rate), self._process_latest
        )

        if self.pause_after_survey_complete:
            rospy.logwarn(
                "[MineLocalizer] pause_after_survey_complete is deprecated and "
                "ignored; inference remains enabled with reversible throttling"
            )

        self._publish_status("waiting_for_camera", 0, 0.0, "")
        rospy.loginfo(
            "[MineLocalizer] ready model=%s task=segment class=%s/%d conf=%.2f "
            "imgsz=%d rate=%.1fHz device=%s rgb=%s depth=%s -> %s",
            self.model_path,
            self.target_class_name,
            self.target_class_id,
            self.confidence,
            self.imgsz,
            self.inference_rate,
            self.device,
            self.rgb_topic,
            self.depth_topic,
            self.output_topic,
        )

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _survey_status_cb(self, msg):
        if msg.data.strip().upper() != "COMPLETE":
            return
        self._publish_status(
            "survey_complete_continuing",
            0,
            0.0,
            "UAV coverage COMPLETE; camera and inference remain online",
        )
        rospy.loginfo(
            "[MineLocalizer] UAV coverage COMPLETE; continuing inference"
        )

    @staticmethod
    def _goal_signature(msg):
        return (
            str(msg.header.frame_id),
            round(float(msg.pose.position.x), 4),
            round(float(msg.pose.position.y), 4),
            round(float(msg.pose.position.z), 4),
            round(float(msg.pose.orientation.x), 5),
            round(float(msg.pose.orientation.y), 5),
            round(float(msg.pose.orientation.z), 5),
            round(float(msg.pose.orientation.w), 5),
        )

    def _goal_cb(self, msg, topic):
        signature = self._goal_signature(msg)
        # The survey node intentionally refreshes its current waypoint every
        # two seconds.  Only a changed waypoint is a new survey target;
        # otherwise the refresh permanently defeats the 0.25 Hz arm-load
        # throttle.  Manual clicks are never deduplicated, so clicking the same
        # location again still restores inference immediately.
        if (topic == self.survey_goal_topic
                and self.last_goal_signatures.get(topic) == signature):
            return
        self.last_goal_signatures[topic] = signature
        self.inference_enabled = True
        self.goal_rate_restore_until = (
            time.monotonic() + self.goal_rate_restore_duration
        )
        self._publish_status(
            "flight_rate_restored", 0, 0.0,
            "new UAV goal automatically enabled inference at flight rate",
        )

    def _grasp_status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            stage = str(payload.get("stage", ""))
        except (TypeError, ValueError):
            return
        terminal = {
            "READY", "COMPLETE", "FAILED", "RESET", "PLACE_COMPLETE",
            "TRANSPORT_LOCKED", "TRANSPORT_FAILED", "TRANSPORT_LOCK_LOST",
        }
        self.manipulation_active = bool(stage and stage not in terminal)

    def _set_enabled_cb(self, request):
        self.inference_enabled = bool(request.data)
        state = "enabled" if self.inference_enabled else "disabled"
        self._publish_status(
            "manual_" + state, 0, 0.0,
            "manual SetBool control; a new UAV goal will re-enable inference",
        )
        return SetBoolResponse(
            success=True, message="mine detection {}".format(state)
        )

    def _desired_inference_rate(self):
        if time.monotonic() <= self.goal_rate_restore_until:
            return self.inference_rate
        if self.manipulation_active:
            return max(self.manipulation_inference_rate, 0.25)
        return self.inference_rate

    def _sync_cb(self, rgb_msg, depth_msg):
        # Keep only the newest synchronized pair.  This prevents CPU inference
        # from building an old-frame queue while Gazebo publishes at 8 Hz.
        with self.latest_lock:
            self.latest_pair = (rgb_msg, depth_msg)

    def _validated_depth_stamp(self, rgb_msg, depth_msg):
        """Return the depth stamp only when both sensor stamps are trustworthy."""
        rgb_stamp = rgb_msg.header.stamp
        depth_stamp = depth_msg.header.stamp
        if rgb_stamp == rospy.Time() or depth_stamp == rospy.Time():
            return None, "zero RGB/depth timestamp"
        delta = abs((rgb_stamp - depth_stamp).to_sec())
        if delta > self.max_rgb_depth_stamp_delta:
            return None, (
                "RGB/depth timestamp delta {:.6f}s exceeds {:.6f}s"
            ).format(delta, self.max_rgb_depth_stamp_delta)
        # The reconstructed point is a depth measurement, therefore its TF must
        # be queried at the depth-image timestamp (never at Time(0)).
        return depth_stamp, ""

    @staticmethod
    def _quat_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-12:
            return np.eye(3, dtype=np.float64)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.asarray(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _lookup_transform(self, camera_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                camera_frame,
                stamp,
                rospy.Duration(self.tf_timeout),
            )
        except Exception as exact_exc:
            rospy.logwarn_throttle(
                1.0,
                "[MineLocalizer] exact-time TF %s -> %s unavailable at depth "
                "stamp %.6f: %s",
                camera_frame,
                self.map_frame,
                stamp.to_sec(),
                exact_exc,
            )
            return None

    @staticmethod
    def _depth_to_meters(depth_msg, bridge):
        depth = np.asarray(
            bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        )
        if depth.dtype == np.uint16 or depth_msg.encoding in ("16UC1", "mono16"):
            return depth.astype(np.float32) * 0.001
        return depth.astype(np.float32)

    @staticmethod
    def _scaled_intrinsics(info, image_shape):
        height_px, width_px = image_shape[:2]
        info_width = int(info.width) if info.width else width_px
        info_height = int(info.height) if info.height else height_px
        scale_x = float(width_px) / max(info_width, 1)
        scale_y = float(height_px) / max(info_height, 1)
        fx = float(info.K[0]) * scale_x
        fy = float(info.K[4]) * scale_y
        cx = float(info.K[2]) * scale_x
        cy = float(info.K[5]) * scale_y
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
            return None
        if fx <= 0.0 or fy <= 0.0:
            return None
        return fx, fy, cx, cy

    def _depth_mad_inliers(self, uu, vv, zz, minimum_samples):
        """Reject isolated/background depths while retaining a sloped surface."""
        if zz.size < minimum_samples:
            return None
        median_depth = float(np.median(zz))
        mad = float(np.median(np.abs(zz - median_depth)))
        gate = max(
            self.depth_mad_floor,
            self.depth_mad_scale * 1.4826 * mad,
        )
        keep = np.abs(zz - median_depth) <= gate
        if int(np.count_nonzero(keep)) < minimum_samples:
            return None
        return uu[keep], vv[keep], zz[keep]

    def _mask_measurement(self, mask, depth, info):
        if mask.shape != depth.shape[:2]:
            mask = cv2.resize(
                mask.astype(np.uint8),
                (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        else:
            mask = mask.astype(bool)

        mask_area = int(np.count_nonzero(mask))
        if mask_area < self.min_mask_area:
            return None

        sample_mask = mask.astype(np.uint8)
        if self.erode_pixels > 0:
            kernel_size = 2 * self.erode_pixels + 1
            kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
            eroded = cv2.erode(sample_mask, kernel, iterations=1)
            if int(np.count_nonzero(eroded)) >= self.min_depth_samples:
                sample_mask = eroded

        ys, xs = np.nonzero(sample_mask)
        if xs.size == 0:
            return None
        values = depth[ys, xs]
        valid = (
            np.isfinite(values)
            & (values >= self.depth_min)
            & (values <= self.depth_max)
        )
        if int(np.count_nonzero(valid)) < self.min_depth_samples:
            return None

        xs = xs[valid].astype(np.float64)
        ys = ys[valid].astype(np.float64)
        values = values[valid].astype(np.float64)
        inliers = self._depth_mad_inliers(
            xs, ys, values, self.min_depth_samples
        )
        if inliers is None:
            return None
        xs, ys, values = inliers

        intrinsics = self._scaled_intrinsics(info, depth.shape)
        if intrinsics is None:
            return None
        fx, fy, cx, cy = intrinsics
        # Preserve the physical correlation between every pixel and its own
        # depth.  Independently combining median(u), median(v), median(depth)
        # creates a synthetic ray and is especially wrong on a slope/tilted UAV.
        camera_points = np.column_stack(
            (
                (xs - cx) * values / fx,
                (ys - cy) * values / fy,
                values,
            )
        )
        finite_points = np.all(np.isfinite(camera_points), axis=1)
        camera_points = camera_points[finite_points]
        xs = xs[finite_points]
        ys = ys[finite_points]
        values = values[finite_points]
        if camera_points.shape[0] < self.min_depth_samples:
            return None
        return (
            camera_points,
            float(np.median(values)),
            mask_area,
            float(np.median(xs)),
            float(np.median(ys)),
            mask,
        )

    def _points_to_map(self, camera_points, transform):
        points = np.asarray(camera_points, dtype=np.float64)
        one_point = points.ndim == 1
        if one_point:
            points = points.reshape(1, 3)
        if points.ndim != 2 or points.shape[1] != 3:
            return None
        rotation = self._quat_matrix(transform.transform.rotation)
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        mapped = points.dot(rotation.T) + translation
        if one_point:
            return mapped[0] if np.all(np.isfinite(mapped[0])) else None
        return mapped[np.all(np.isfinite(mapped), axis=1)]

    def _to_map(self, camera_point, transform):
        return self._points_to_map(camera_point, transform)

    def _fit_local_ground_plane(self, map_points, target_xy):
        """Robustly fit z=a*dx+b*dy+c and return z at target_xy."""
        points = np.asarray(map_points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            return None, 0
        points = points[np.all(np.isfinite(points), axis=1)]
        if points.shape[0] < self.ground_min_samples:
            return None, int(points.shape[0])

        target_xy = np.asarray(target_xy, dtype=np.float64).reshape(2)
        dx = points[:, 0] - target_xy[0]
        dy = points[:, 1] - target_xy[1]
        if (
            float(np.ptp(dx)) < self.ground_plane_min_xy_span
            or float(np.ptp(dy)) < self.ground_plane_min_xy_span
        ):
            return None, int(points.shape[0])
        design = np.column_stack((dx, dy, np.ones(points.shape[0])))
        z_values = points[:, 2]
        keep = np.ones(points.shape[0], dtype=bool)
        coefficients = None
        for _iteration in range(4):
            if int(np.count_nonzero(keep)) < self.ground_min_samples:
                return None, int(np.count_nonzero(keep))
            active_design = design[keep]
            if np.linalg.matrix_rank(active_design) < 3:
                return None, int(np.count_nonzero(keep))
            coefficients, _residuals, _rank, _singular = np.linalg.lstsq(
                active_design, z_values[keep], rcond=None
            )
            residual = z_values - design.dot(coefficients)
            active_residual = residual[keep]
            residual_center = float(np.median(active_residual))
            residual_mad = float(
                np.median(np.abs(active_residual - residual_center))
            )
            residual_gate = max(
                self.ground_plane_residual_floor,
                self.ground_plane_mad_scale * 1.4826 * residual_mad,
            )
            new_keep = np.abs(residual - residual_center) <= residual_gate
            if int(np.count_nonzero(new_keep)) < self.ground_min_samples:
                return None, int(np.count_nonzero(new_keep))
            if np.array_equal(new_keep, keep):
                break
            keep = new_keep

        active_design = design[keep]
        if np.linalg.matrix_rank(active_design) < 3:
            return None, int(np.count_nonzero(keep))
        coefficients, _residuals, _rank, _singular = np.linalg.lstsq(
            active_design, z_values[keep], rcond=None
        )
        # dx=dy=0 at the requested target, so the intercept is its local ground.
        ground_z = float(coefficients[2])
        return (
            ground_z if math.isfinite(ground_z) else None,
            int(np.count_nonzero(keep)),
        )

    def _local_ground_z(
        self, object_mask, depth, info, transform, target_map_point=None
    ):
        """Fit terrain around one mine and evaluate it at the mine's map XY."""
        if transform is None:
            return None, 0
        mask = object_mask.astype(np.uint8)
        if mask.shape != depth.shape[:2]:
            mask = cv2.resize(
                mask,
                (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
        inner_size = 2 * self.ground_ring_inner_px + 1
        outer_size = 2 * self.ground_ring_outer_px + 1
        inner = cv2.dilate(
            mask, np.ones((inner_size, inner_size), dtype=np.uint8), iterations=1
        )
        outer = cv2.dilate(
            mask, np.ones((outer_size, outer_size), dtype=np.uint8), iterations=1
        )
        ring = (outer > 0) & (inner == 0)
        vv, uu = np.nonzero(ring)
        if uu.size < self.ground_min_samples:
            return None, int(uu.size)
        zz = depth[vv, uu].astype(np.float64)
        valid = (
            np.isfinite(zz)
            & (zz >= self.depth_min)
            & (zz <= self.depth_max)
        )
        uu = uu[valid].astype(np.float64)
        vv = vv[valid].astype(np.float64)
        zz = zz[valid]
        if zz.size < self.ground_min_samples:
            return None, int(zz.size)

        inliers = self._depth_mad_inliers(
            uu, vv, zz, self.ground_min_samples
        )
        if inliers is None:
            return None, 0
        uu, vv, zz = inliers

        intrinsics = self._scaled_intrinsics(info, depth.shape)
        if intrinsics is None:
            return None, 0
        fx, fy, cx, cy = intrinsics
        camera_points = np.column_stack((
            (uu - cx) * zz / fx,
            (vv - cy) * zz / fy,
            zz,
        ))
        map_points = self._points_to_map(camera_points, transform)
        if map_points is None or map_points.shape[0] < self.ground_min_samples:
            count = 0 if map_points is None else int(map_points.shape[0])
            return None, count
        if target_map_point is None:
            target_xy = np.median(map_points[:, :2], axis=0)
        else:
            target_xy = np.asarray(target_map_point, dtype=np.float64)[:2]
        return self._fit_local_ground_plane(map_points, target_xy)

    def _publish_status(self, state, detections, inference_ms, detail):
        payload = {
            "state": state,
            "detections": int(detections),
            "inference_ms": round(float(inference_ms), 3),
            "processed_frames": int(self.processed_frames),
            "tf_latest_fallbacks": int(self.tf_fallback_count),
            "sync_rejected_frames": int(self.sync_rejected_frames),
            "invalid_timestamp_frames": int(self.invalid_timestamp_frames),
            "inference_enabled": bool(self.inference_enabled),
            "manipulation_active": bool(self.manipulation_active),
            "requested_rate_hz": float(self._desired_inference_rate()),
            "detail": str(detail),
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _publish_empty(self, stamp, detail, image=None, frame_id=""):
        output = MineDetectionArray()
        output.header.stamp = stamp
        output.header.frame_id = self.map_frame
        self.detection_pub.publish(output)
        if image is not None:
            self._publish_debug(image, [], stamp, frame_id)
        self._publish_status(detail, 0, 0.0, detail)

    def _publish_debug(self, image, annotations, stamp, frame_id, mode_text="INFERENCE OK"):
        debug = image.copy()
        for annotation in annotations:
            mask = annotation["mask"]
            confidence = annotation["confidence"]
            point = annotation.get("point")
            valid = point is not None
            color = (0, 220, 0) if valid else (0, 165, 255)
            overlay = np.zeros_like(debug)
            overlay[mask] = color
            debug = cv2.addWeighted(debug, 1.0, overlay, 0.32, 0.0)
            contours, _ = cv2.findContours(
                mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cv2.drawContours(debug, contours, -1, color, 2)
            u = int(annotation["u"])
            v = int(annotation["v"])
            if valid:
                relative_height = annotation.get("height_above_ground")
                suffix = (
                    " h={:.2f}".format(relative_height)
                    if relative_height is not None else " h=?"
                )
                label = "mine {:.2f} map({:.2f},{:.2f}){}".format(
                    confidence, point[0], point[1], suffix
                )
            else:
                label = "mine {:.2f} {}".format(
                    confidence,
                    annotation.get("detail") or "no-depth/tf",
                )
            cv2.putText(
                debug,
                label,
                (max(0, u - 80), max(22, v - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                2,
                cv2.LINE_AA,
            )
        cv2.putText(
            debug,
            "{} | YOLO11s-seg conf>={:.2f} valid={}".format(
                mode_text,
                self.confidence,
                sum(a.get("point") is not None for a in annotations),
            ),
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        self.debug_pub.publish(msg)

    def _publish_passthrough_debug(self, rgb_msg, mode_text):
        stamp_key = (rgb_msg.header.stamp.secs, rgb_msg.header.stamp.nsecs)
        if stamp_key == self.last_debug_stamp:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception as exc:
            self._publish_status("debug_conversion_failed", 0, 0.0, str(exc))
            return
        self.last_debug_stamp = stamp_key
        self._publish_debug(
            image, [], rgb_msg.header.stamp, rgb_msg.header.frame_id, mode_text
        )

    def _process_latest(self, _event):
        if not self.processing_lock.acquire(False):
            return
        try:
            with self.latest_lock:
                pair = self.latest_pair
            if pair is None or self.camera_info is None:
                self._publish_status("waiting_for_camera", 0, 0.0, "")
                return
            rgb_msg, depth_msg = pair
            stamp_key = (
                rgb_msg.header.stamp.secs,
                rgb_msg.header.stamp.nsecs,
                depth_msg.header.stamp.secs,
                depth_msg.header.stamp.nsecs,
            )
            if not self.inference_enabled:
                self._publish_passthrough_debug(
                    rgb_msg, "INFERENCE MANUALLY DISABLED (CAMERA OK)"
                )
                self._publish_status(
                    "inference_disabled", 0, 0.0,
                    "camera frames continue; call /mine_detection/set_enabled "
                    "or send a UAV goal to resume",
                )
                return
            desired_rate = self._desired_inference_rate()
            now_wall = time.monotonic()
            if now_wall - self.last_inference_wall < 1.0 / desired_rate:
                self._publish_passthrough_debug(
                    rgb_msg,
                    "CAMERA OK | INFERENCE THROTTLED {:.2f} Hz".format(
                        desired_rate
                    ),
                )
                return
            if stamp_key == self.last_processed_stamp:
                return
            self.last_processed_stamp = stamp_key
            self.last_inference_wall = now_wall
            stamp, stamp_error = self._validated_depth_stamp(rgb_msg, depth_msg)
            if stamp is None:
                if "zero" in stamp_error:
                    self.invalid_timestamp_frames += 1
                    state = "invalid_timestamp"
                else:
                    self.sync_rejected_frames += 1
                    state = "rgb_depth_unsynchronized"
                rospy.logwarn_throttle(
                    1.0, "[MineLocalizer] rejecting sensor pair: %s", stamp_error
                )
                self._publish_passthrough_debug(
                    rgb_msg, "CAMERA OK | {}".format(state.upper())
                )
                self._publish_status(state, 0, 0.0, stamp_error)
                return

            try:
                image = self.bridge.imgmsg_to_cv2(
                    rgb_msg, desired_encoding="bgr8"
                )
                depth = self._depth_to_meters(depth_msg, self.bridge)
            except Exception as exc:
                rospy.logwarn_throttle(
                    1.0, "[MineLocalizer] image conversion failed: %s", exc
                )
                self._publish_status("image_conversion_failed", 0, 0.0, str(exc))
                return

            started = time.monotonic()
            try:
                if self.inference_backend == "tensorrt":
                    response = self.trt_detect(DetectLandminesRequest(image=rgb_msg))
                    trt_masks = []
                    for mask_msg in response.masks:
                        trt_masks.append(self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8") > 0)
                    results = [_TRTResult(response.detections, trt_masks)]
                else:
                    results = self.model.predict(
                    source=image,
                    imgsz=self.imgsz,
                    conf=self.confidence,
                    iou=self.iou_threshold,
                    classes=[self.target_class_id],
                    max_det=self.max_detections,
                    device=self.device,
                    retina_masks=True,
                    verbose=False,
                    )
            except Exception as exc:
                rospy.logerr_throttle(
                    1.0, "[MineLocalizer] inference failed: %s", exc
                )
                self._publish_debug(
                    image, [], stamp, rgb_msg.header.frame_id,
                    "INFERENCE FAILED (CAMERA OK)",
                )
                self._publish_status("inference_failed", 0, 0.0, str(exc))
                return
            inference_ms = (time.monotonic() - started) * 1000.0
            self.inference_time_pub.publish(Float32(data=inference_ms))
            self.processed_frames += 1

            output = MineDetectionArray()
            output.header.stamp = stamp
            output.header.frame_id = self.map_frame
            annotations = []
            ground_rejected = 0
            ground_unavailable = 0
            if results and results[0].masks is not None and results[0].boxes is not None:
                result = results[0]
                masks = result.masks.data.detach().cpu().numpy()
                boxes = result.boxes
                camera_frame = (
                    self.camera_frame_override
                    or depth_msg.header.frame_id
                    or rgb_msg.header.frame_id
                    or self.camera_info.header.frame_id
                )
                transform = self._lookup_transform(camera_frame, stamp)
                for index in range(min(len(masks), len(boxes))):
                    class_id = int(boxes.cls[index].item())
                    confidence = float(boxes.conf[index].item())
                    if class_id != self.target_class_id:
                        continue
                    mask = masks[index] >= 0.5
                    if mask.shape != image.shape[:2]:
                        mask = cv2.resize(
                            mask.astype(np.uint8),
                            (image.shape[1], image.shape[0]),
                            interpolation=cv2.INTER_NEAREST,
                        ).astype(bool)
                    measurement = self._mask_measurement(
                        mask, depth, self.camera_info
                    )
                    if measurement is None:
                        ys, xs = np.nonzero(mask)
                        annotations.append(
                            {
                                "mask": mask,
                                "confidence": confidence,
                                "u": float(np.median(xs)) if xs.size else 0.0,
                                "v": float(np.median(ys)) if ys.size else 0.0,
                                "point": None,
                            }
                        )
                        continue
                    camera_points, depth_m, mask_area, u, v, used_mask = measurement
                    map_point = None
                    if transform is not None:
                        map_points = self._points_to_map(camera_points, transform)
                        if (
                            map_points is not None
                            and map_points.shape[0] >= self.min_depth_samples
                        ):
                            # Transform every physical pixel/depth tuple first;
                            # only then take a robust representative in map.
                            map_point = np.median(map_points, axis=0)
                    ground_z = None
                    height_above_ground = None
                    ground_samples = 0
                    rejection_detail = ""
                    if map_point is not None:
                        ground_z, ground_samples = self._local_ground_z(
                            used_mask,
                            depth,
                            self.camera_info,
                            transform,
                            target_map_point=map_point,
                        )
                        if ground_z is None:
                            ground_unavailable += 1
                            if self.require_local_ground:
                                rejection_detail = "no local ground depth ring"
                                map_point = None
                        else:
                            height_above_ground = float(map_point[2] - ground_z)
                            if not (
                                self.min_height_above_ground
                                <= height_above_ground
                                <= self.max_height_above_ground
                            ):
                                rejection_detail = (
                                    "relative height {:.2f}m outside {:.2f}..{:.2f}m"
                                ).format(
                                    height_above_ground,
                                    self.min_height_above_ground,
                                    self.max_height_above_ground,
                                )
                                ground_rejected += 1
                                map_point = None
                    debug_mask = used_mask
                    if debug_mask.shape != image.shape[:2]:
                        debug_mask = cv2.resize(
                            debug_mask.astype(np.uint8),
                            (image.shape[1], image.shape[0]),
                            interpolation=cv2.INTER_NEAREST,
                        ).astype(bool)
                    annotations.append(
                        {
                            "mask": debug_mask,
                            "confidence": confidence,
                            "u": u,
                            "v": v,
                            "point": map_point,
                            "ground_z": ground_z,
                            "height_above_ground": height_above_ground,
                            "ground_samples": ground_samples,
                            "detail": rejection_detail,
                        }
                    )
                    if map_point is None:
                        continue
                    detection = MineDetection()
                    detection.detection_id = len(output.detections)
                    detection.confidence = confidence
                    detection.position.x = float(map_point[0])
                    detection.position.y = float(map_point[1])
                    detection.position.z = float(map_point[2])
                    detection.depth = depth_m
                    detection.mask_area = mask_area
                    output.detections.append(detection)

            self.detection_pub.publish(output)
            self._publish_debug(
                image, annotations, stamp, rgb_msg.header.frame_id,
                "INFERENCE OK",
            )
            self.last_debug_stamp = (
                rgb_msg.header.stamp.secs, rgb_msg.header.stamp.nsecs
            )
            self._publish_status(
                "ok",
                len(output.detections),
                inference_ms,
                "ground_rejected={} ground_unavailable={}".format(
                    ground_rejected, ground_unavailable
                ),
            )
            rospy.loginfo_throttle(
                1.0,
                "[MineLocalizerDiag] frame=%d detections=%d inference=%.1fms "
                "tf_fallbacks=%d",
                self.processed_frames,
                len(output.detections),
                inference_ms,
                self.tf_fallback_count,
            )
        finally:
            self.processing_lock.release()


def main():
    rospy.init_node("mine_seg_localizer")
    MineSegLocalizer()
    rospy.spin()


if __name__ == "__main__":
    main()
