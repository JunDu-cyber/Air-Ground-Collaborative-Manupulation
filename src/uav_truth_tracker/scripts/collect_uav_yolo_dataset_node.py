#!/usr/bin/env python3
"""Collect Gazebo RGB images and offline YOLO labels for the target UAV."""

import math
import os
import re

import cv2
import numpy as np
import rospy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image


class UavYoloDatasetCollector:
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
        self.uav1_odom_topic = rospy.get_param(
            "~uav1_odom_topic", "/uav1/mavros/local_position/odom"
        )
        self.fixed_frame = rospy.get_param("~fixed_frame", "map")
        self.camera_frame_override = rospy.get_param("~camera_frame", "")
        self.dataset_dir = os.path.expanduser(
            rospy.get_param("~dataset_dir", "~/uav_yolo_dataset")
        )
        self.sample_rate = max(float(rospy.get_param("~sample_rate", 5.0)), 0.1)
        self.max_images = max(int(rospy.get_param("~max_images", 3000)), 0)
        self.max_new_images = max(int(rospy.get_param("~max_new_images", 0)), 0)
        self.dataset_tag = self._sanitize_tag(
            rospy.get_param("~dataset_tag", "default")
        )
        self.val_stride = max(int(rospy.get_param("~val_stride", 5)), 2)
        self.negative_sample_ratio = min(
            max(float(rospy.get_param("~negative_sample_ratio", 0.25)), 0.0), 1.0
        )
        self.uav1_spawn = np.array(
            [
                float(rospy.get_param("~uav1_spawn_x", 2.0)),
                float(rospy.get_param("~uav1_spawn_y", 0.0)),
                float(rospy.get_param("~uav1_spawn_z", 0.0)),
            ],
            dtype=np.float64,
        )
        self.target_size = np.array(
            [
                float(rospy.get_param("~target_size_x", 1.0)),
                float(rospy.get_param("~target_size_y", 1.0)),
                float(rospy.get_param("~target_size_z", 0.35)),
            ],
            dtype=np.float64,
        )
        self.min_bbox_pixels = max(int(rospy.get_param("~min_bbox_pixels", 12)), 2)
        self.depth_visibility_check = rospy.get_param(
            "~depth_visibility_check", True
        )
        self.depth_tolerance = max(
            float(rospy.get_param("~depth_tolerance", 1.2)), 0.1
        )
        self.min_depth_match_fraction = min(
            max(float(rospy.get_param("~min_depth_match_fraction", 0.01)), 0.0), 1.0
        )
        self.tf_timeout = max(float(rospy.get_param("~tf_timeout", 0.05)), 0.0)
        self.allow_latest_tf_fallback = rospy.get_param(
            "~allow_latest_tf_fallback", True
        )

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_info = None
        self.depth_image = None
        self.depth_stamp = None
        self.uav1_odom = None
        self.last_save_time = rospy.Time(0)
        self.image_count = 0
        self.positive_count = 0
        self.negative_count = 0

        for split in ("train", "val"):
            os.makedirs(os.path.join(self.dataset_dir, "images", split), exist_ok=True)
            os.makedirs(os.path.join(self.dataset_dir, "labels", split), exist_ok=True)
        self._load_existing_counts()
        self.session_start_count = self.image_count
        self._write_dataset_yaml()

        rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1
        )
        rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)
        rospy.Subscriber(
            self.uav1_odom_topic, Odometry, self.uav1_odom_cb, queue_size=1
        )
        rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)

        rospy.logwarn(
            "[UavYoloDatasetCollector] OFFLINE DATASET TOOL: subscribes to %s for labels. "
            "Do not run this node in the online chase chain.",
            self.uav1_odom_topic,
        )
        rospy.loginfo(
            "[UavYoloDatasetCollector] rgb=%s depth=%s camera_info=%s dataset=%s "
            "rate=%.1f max_images=%d max_new_images=%d tag=%s "
            "target_size=(%.2f %.2f %.2f)",
            self.rgb_topic,
            self.depth_topic,
            self.camera_info_topic,
            self.dataset_dir,
            self.sample_rate,
            self.max_images,
            self.max_new_images,
            self.dataset_tag,
            self.target_size[0],
            self.target_size[1],
            self.target_size[2],
        )

    @staticmethod
    def _sanitize_tag(value):
        tag = re.sub(r"[^A-Za-z0-9_-]+", "_", str(value).strip())
        return tag.strip("_") or "default"

    def _write_dataset_yaml(self):
        path = os.path.join(self.dataset_dir, "uav_dataset.yaml")
        data = {
            "path": self.dataset_dir,
            "train": "images/train",
            "val": "images/val",
            "names": {0: "uav"},
        }
        with open(path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)

    def _load_existing_counts(self):
        max_index = -1
        for split in ("train", "val"):
            label_dir = os.path.join(self.dataset_dir, "labels", split)
            for name in os.listdir(label_dir):
                if not name.endswith(".txt"):
                    continue
                try:
                    max_index = max(max_index, int(name.split("_", 1)[0]))
                except ValueError:
                    pass
                path = os.path.join(label_dir, name)
                if os.path.getsize(path) > 0:
                    self.positive_count += 1
                else:
                    self.negative_count += 1
        self.image_count = max_index + 1

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0, "[UavYoloDatasetCollector] depth conversion failed: %s", exc
            )
            return
        depth = np.asarray(depth)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        self.depth_image = depth
        self.depth_stamp = msg.header.stamp

    def uav1_odom_cb(self, msg):
        self.uav1_odom = msg

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
                camera_frame,
                self.fixed_frame,
                stamp,
                rospy.Duration(self.tf_timeout),
            )
        except Exception as exc:
            if not self.allow_latest_tf_fallback:
                rospy.logwarn_throttle(
                    1.0,
                    "[UavYoloDatasetCollector] TF %s -> %s unavailable: %s",
                    self.fixed_frame,
                    camera_frame,
                    exc,
                )
                return None
            try:
                return self.tf_buffer.lookup_transform(
                    camera_frame,
                    self.fixed_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_timeout),
                )
            except Exception as latest_exc:
                rospy.logwarn_throttle(
                    1.0,
                    "[UavYoloDatasetCollector] latest TF %s -> %s unavailable: %s",
                    self.fixed_frame,
                    camera_frame,
                    latest_exc,
                )
                return None

    def _target_corners_map(self):
        pose = self.uav1_odom.pose.pose
        center = np.array(
            [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64
        ) + self.uav1_spawn
        rotation = self._quat_matrix(pose.orientation)
        half = 0.5 * self.target_size
        corners = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    local = np.array([sx * half[0], sy * half[1], sz * half[2]])
                    corners.append(center + rotation.dot(local))
        return center, corners

    def _transform_points(self, transform, points):
        rotation = self._quat_matrix(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        return [rotation.dot(point) + translation for point in points]

    def _project_bbox(self, image, stamp, camera_frame):
        if self.camera_info is None or self.uav1_odom is None:
            return None
        transform = self._lookup_transform(camera_frame, stamp)
        if transform is None:
            return None
        center_map, corners_map = self._target_corners_map()
        center_cam = self._transform_points(transform, [center_map])[0]
        corners_cam = self._transform_points(transform, corners_map)
        if center_cam[2] <= 0.1:
            return None

        fx = float(self.camera_info.K[0])
        fy = float(self.camera_info.K[4])
        cx = float(self.camera_info.K[2])
        cy = float(self.camera_info.K[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        pixels = []
        for point in corners_cam:
            if point[2] <= 0.05:
                continue
            pixels.append(
                (fx * point[0] / point[2] + cx, fy * point[1] / point[2] + cy)
            )
        if len(pixels) < 4:
            return None

        height, width = image.shape[:2]
        x1 = max(0.0, min(pixel[0] for pixel in pixels))
        y1 = max(0.0, min(pixel[1] for pixel in pixels))
        x2 = min(float(width - 1), max(pixel[0] for pixel in pixels))
        y2 = min(float(height - 1), max(pixel[1] for pixel in pixels))
        if x2 - x1 < self.min_bbox_pixels or y2 - y1 < self.min_bbox_pixels:
            return None
        bbox = (x1, y1, x2, y2)
        if self.depth_visibility_check and not self._depth_matches(bbox, center_cam[2]):
            return None
        return bbox

    def _depth_matches(self, bbox, target_depth):
        if self.depth_image is None:
            return False
        x1, y1, x2, y2 = bbox
        height, width = self.depth_image.shape[:2]
        ix1 = max(0, min(width - 1, int(x1)))
        iy1 = max(0, min(height - 1, int(y1)))
        ix2 = max(ix1 + 1, min(width, int(math.ceil(x2))))
        iy2 = max(iy1 + 1, min(height, int(math.ceil(y2))))
        roi = self.depth_image[iy1:iy2, ix1:ix2]
        valid = np.isfinite(roi) & (roi > 0.1)
        if not np.any(valid):
            return False
        values = roi[valid]
        matches = np.abs(values - target_depth) <= self.depth_tolerance
        return float(np.count_nonzero(matches)) / float(values.size) >= self.min_depth_match_fraction

    def _should_save_negative(self):
        if self.negative_sample_ratio <= 0.0:
            return False
        total = self.positive_count + self.negative_count
        desired_negatives = self.negative_sample_ratio * max(total + 1, 1)
        return self.negative_count < desired_negatives

    def _save(self, image, bbox, stamp):
        split = "val" if self.image_count % self.val_stride == 0 else "train"
        name = "{:06d}_{}_{:010d}_{:09d}".format(
            self.image_count, self.dataset_tag, stamp.secs, stamp.nsecs
        )
        image_path = os.path.join(self.dataset_dir, "images", split, name + ".jpg")
        label_path = os.path.join(self.dataset_dir, "labels", split, name + ".txt")
        if not cv2.imwrite(image_path, image):
            rospy.logwarn("[UavYoloDatasetCollector] failed to write %s", image_path)
            return

        label = ""
        if bbox is not None:
            height, width = image.shape[:2]
            x1, y1, x2, y2 = bbox
            xc = 0.5 * (x1 + x2) / width
            yc = 0.5 * (y1 + y2) / height
            bw = (x2 - x1) / width
            bh = (y2 - y1) / height
            label = "0 {:.8f} {:.8f} {:.8f} {:.8f}\n".format(xc, yc, bw, bh)
            self.positive_count += 1
        else:
            self.negative_count += 1
        with open(label_path, "w", encoding="utf-8") as stream:
            stream.write(label)
        self.image_count += 1
        rospy.loginfo_throttle(
            1.0,
            "[UavYoloDatasetCollector] saved=%d positive=%d negative=%d dataset=%s",
            self.image_count,
            self.positive_count,
            self.negative_count,
            self.dataset_dir,
        )

    def rgb_cb(self, msg):
        if self.max_images > 0 and self.image_count >= self.max_images:
            rospy.signal_shutdown("dataset collection complete")
            return
        if (
            self.max_new_images > 0
            and self.image_count - self.session_start_count >= self.max_new_images
        ):
            rospy.signal_shutdown("dataset collection session complete")
            return
        now = rospy.Time.now()
        if (
            self.last_save_time != rospy.Time(0)
            and (now - self.last_save_time).to_sec() < 1.0 / self.sample_rate
        ):
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0, "[UavYoloDatasetCollector] RGB conversion failed: %s", exc
            )
            return
        camera_frame = self.camera_frame_override or msg.header.frame_id
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else now
        bbox = self._project_bbox(image, stamp, camera_frame)
        if bbox is None and not self._should_save_negative():
            return
        self._save(image, bbox, stamp)
        self.last_save_time = now


def main():
    rospy.init_node("collect_uav_yolo_dataset_node")
    UavYoloDatasetCollector()
    rospy.spin()


if __name__ == "__main__":
    main()
