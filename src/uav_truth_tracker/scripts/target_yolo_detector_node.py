#!/usr/bin/env python3
"""Online UAV semantic detector using YOLO RGB detections and aligned depth."""

import math
import os

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray


class TargetYoloDetector:
    def __init__(self):
        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/iris0/camera/rgb/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/iris0/camera/depth/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/iris0/camera/rgb/camera_info"
        )
        self.fixed_frame = rospy.get_param("~fixed_frame", "map")
        self.camera_frame_override = rospy.get_param("~camera_frame", "")
        self.model_path = os.path.expanduser(rospy.get_param("~model_path", ""))
        self.target_class_name = rospy.get_param("~target_class_name", "uav").lower()
        self.target_class_id = int(rospy.get_param("~target_class_id", 0))
        self.min_confidence = min(
            max(float(rospy.get_param("~min_confidence", 0.45)), 0.0), 1.0
        )
        self.image_size = max(int(rospy.get_param("~image_size", 640)), 64)
        self.process_rate = max(float(rospy.get_param("~process_rate", 10.0)), 0.1)
        self.depth_min = max(float(rospy.get_param("~depth_min", 0.3)), 0.0)
        self.depth_max = max(float(rospy.get_param("~depth_max", 30.0)), self.depth_min)
        self.depth_roi_scale = min(
            max(float(rospy.get_param("~depth_roi_scale", 0.55)), 0.05), 1.0
        )
        self.depth_percentile = min(
            max(float(rospy.get_param("~depth_percentile", 20.0)), 0.0), 100.0
        )
        self.min_depth_samples = max(int(rospy.get_param("~min_depth_samples", 5)), 1)
        self.tf_timeout = max(float(rospy.get_param("~tf_timeout", 0.05)), 0.0)
        self.allow_latest_tf_fallback = rospy.get_param(
            "~allow_latest_tf_fallback", True
        )
        self.pose_covariance = max(
            float(rospy.get_param("~pose_covariance", 0.08)), 1e-6
        )
        self.output_pose_topic = rospy.get_param(
            "~output_pose_topic", "/target_semantic/pose"
        )
        self.output_pose_cov_topic = rospy.get_param(
            "~output_pose_cov_topic", "/target_semantic/pose_cov"
        )
        self.output_valid_topic = rospy.get_param(
            "~output_valid_topic", "/target_semantic/valid"
        )
        self.output_confidence_topic = rospy.get_param(
            "~output_confidence_topic", "/target_semantic/confidence"
        )
        self.output_marker_topic = rospy.get_param(
            "~output_marker_topic", "/target_semantic/marker"
        )
        self.output_debug_image_topic = rospy.get_param(
            "~output_debug_image_topic", "/target_semantic/debug_image"
        )

        if not self.model_path:
            raise rospy.ROSInitException("~model_path is required")
        if not os.path.isfile(self.model_path):
            raise rospy.ROSInitException("YOLO model not found: {}".format(self.model_path))
        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise rospy.ROSInitException(
                "ultralytics is required: python3 -m pip install --user ultralytics ({})".format(
                    exc
                )
            )

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_info = None
        self.depth_image = None
        self.depth_stamp = None
        self.last_process_time = rospy.Time(0)

        self.pose_pub = rospy.Publisher(self.output_pose_topic, PoseStamped, queue_size=5)
        self.pose_cov_pub = rospy.Publisher(
            self.output_pose_cov_topic, PoseWithCovarianceStamped, queue_size=5
        )
        self.valid_pub = rospy.Publisher(self.output_valid_topic, Bool, queue_size=5)
        self.confidence_pub = rospy.Publisher(
            self.output_confidence_topic, Float32, queue_size=5
        )
        self.marker_pub = rospy.Publisher(
            self.output_marker_topic, MarkerArray, queue_size=5
        )
        self.debug_image_pub = rospy.Publisher(
            self.output_debug_image_topic, Image, queue_size=1
        )

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1
        )
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)

        rospy.logwarn(
            "[TargetYoloDetector] online detector does not subscribe to /uav1 odom."
        )
        rospy.loginfo(
            "[TargetYoloDetector] model=%s rgb=%s depth=%s camera_info=%s output=%s "
            "class=%s/%d min_conf=%.2f rate=%.1f",
            self.model_path,
            self.rgb_topic,
            self.depth_topic,
            self.camera_info_topic,
            self.output_pose_topic,
            self.target_class_name,
            self.target_class_id,
            self.min_confidence,
            self.process_rate,
        )

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0, "[TargetYoloDetector] depth conversion failed: %s", exc
            )
            return
        depth = np.asarray(depth)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        self.depth_image = depth
        self.depth_stamp = msg.header.stamp

    @staticmethod
    def _quat_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
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
                self.fixed_frame,
                camera_frame,
                stamp,
                rospy.Duration(self.tf_timeout),
            )
        except Exception as exc:
            if not self.allow_latest_tf_fallback:
                rospy.logwarn_throttle(
                    1.0,
                    "[TargetYoloDetector] TF %s -> %s unavailable: %s",
                    camera_frame,
                    self.fixed_frame,
                    exc,
                )
                return None
            try:
                return self.tf_buffer.lookup_transform(
                    self.fixed_frame,
                    camera_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_timeout),
                )
            except Exception as latest_exc:
                rospy.logwarn_throttle(
                    1.0,
                    "[TargetYoloDetector] latest TF %s -> %s unavailable: %s",
                    camera_frame,
                    self.fixed_frame,
                    latest_exc,
                )
                return None

    def _target_box(self, result):
        best = None
        best_conf = -1.0
        names = result.names
        for box in result.boxes:
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            class_name = str(names.get(class_id, class_id)).lower()
            if class_id != self.target_class_id and class_name != self.target_class_name:
                continue
            if confidence < self.min_confidence or confidence <= best_conf:
                continue
            coords = box.xyxy[0].cpu().numpy().astype(float)
            best = (coords[0], coords[1], coords[2], coords[3])
            best_conf = confidence
        return best, best_conf

    def _depth_for_bbox(self, bbox):
        if self.depth_image is None:
            return None
        x1, y1, x2, y2 = bbox
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        half_w = 0.5 * (x2 - x1) * self.depth_roi_scale
        half_h = 0.5 * (y2 - y1) * self.depth_roi_scale
        height, width = self.depth_image.shape[:2]
        ix1 = max(0, min(width - 1, int(cx - half_w)))
        iy1 = max(0, min(height - 1, int(cy - half_h)))
        ix2 = max(ix1 + 1, min(width, int(math.ceil(cx + half_w))))
        iy2 = max(iy1 + 1, min(height, int(math.ceil(cy + half_h))))
        roi = self.depth_image[iy1:iy2, ix1:ix2]
        valid = (
            np.isfinite(roi)
            & (roi >= self.depth_min)
            & (roi <= self.depth_max)
        )
        values = roi[valid]
        if values.size < self.min_depth_samples:
            return None
        return float(np.percentile(values, self.depth_percentile))

    def _camera_point_to_fixed(self, u, v, depth, stamp, camera_frame):
        if self.camera_info is None:
            return None
        fx = float(self.camera_info.K[0])
        fy = float(self.camera_info.K[4])
        cx = float(self.camera_info.K[2])
        cy = float(self.camera_info.K[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        camera_point = np.array(
            [(u - cx) * depth / fx, (v - cy) * depth / fy, depth],
            dtype=np.float64,
        )
        transform = self._lookup_transform(camera_frame, stamp)
        if transform is None:
            return None
        rotation = self._quat_matrix(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        return rotation.dot(camera_point) + translation

    def _publish_invalid(self, reason):
        self.confidence_pub.publish(Float32(data=0.0))
        self.valid_pub.publish(Bool(data=False))
        rospy.loginfo_throttle(
            1.0, "[TargetYoloDetectorDiag] valid=False reason=%s", reason
        )

    def _publish_valid(self, point, confidence, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.fixed_frame
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        pose.pose.orientation.w = 1.0

        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = pose.header
        pose_cov.pose.pose = pose.pose
        cov = self.pose_covariance / max(confidence, 0.1)
        pose_cov.pose.covariance[0] = cov
        pose_cov.pose.covariance[7] = cov
        pose_cov.pose.covariance[14] = cov * 1.5
        pose_cov.pose.covariance[21] = 1.0
        pose_cov.pose.covariance[28] = 1.0
        pose_cov.pose.covariance[35] = 1.0

        marker = Marker()
        marker.header = pose.header
        marker.ns = "target_semantic"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.30
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.95
        marker.lifetime = rospy.Duration(0.5)

        self.confidence_pub.publish(Float32(data=confidence))
        self.valid_pub.publish(Bool(data=True))
        self.pose_pub.publish(pose)
        self.pose_cov_pub.publish(pose_cov)
        self.marker_pub.publish(MarkerArray(markers=[marker]))
        rospy.loginfo_throttle(
            1.0,
            "[TargetYoloDetectorDiag] valid=True confidence=%.2f pose=(%.2f %.2f %.2f)",
            confidence,
            point[0],
            point[1],
            point[2],
        )

    def _publish_debug_image(self, image, bbox, confidence, stamp, frame_id):
        debug = image.copy()
        if bbox is not None:
            x1, y1, x2, y2 = [int(value) for value in bbox]
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                debug,
                "uav {:.2f}".format(confidence),
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
        msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        self.debug_image_pub.publish(msg)

    def rgb_cb(self, msg):
        now = rospy.Time.now()
        if (
            self.last_process_time != rospy.Time(0)
            and (now - self.last_process_time).to_sec() < 1.0 / self.process_rate
        ):
            return
        self.last_process_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._publish_invalid("rgb_conversion_failed")
            rospy.logwarn_throttle(1.0, "[TargetYoloDetector] RGB conversion failed: %s", exc)
            return
        results = self.model.predict(
            source=image,
            imgsz=self.image_size,
            conf=self.min_confidence,
            verbose=False,
        )
        if not results:
            self._publish_invalid("no_result")
            return
        bbox, confidence = self._target_box(results[0])
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else now
        self._publish_debug_image(image, bbox, confidence, stamp, msg.header.frame_id)
        if bbox is None:
            self._publish_invalid("no_uav_bbox")
            return
        depth = self._depth_for_bbox(bbox)
        if depth is None:
            self._publish_invalid("no_depth_in_bbox")
            return
        u = 0.5 * (bbox[0] + bbox[2])
        v = 0.5 * (bbox[1] + bbox[3])
        camera_frame = self.camera_frame_override or msg.header.frame_id
        point = self._camera_point_to_fixed(u, v, depth, stamp, camera_frame)
        if point is None:
            self._publish_invalid("tf_or_camera_info_unavailable")
            return
        self._publish_valid(point, confidence, stamp)


def main():
    rospy.init_node("target_yolo_detector_node")
    TargetYoloDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
