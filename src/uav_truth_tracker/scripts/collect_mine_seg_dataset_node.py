#!/usr/bin/env python3
"""Truth-assisted Gazebo collector for a one-class landmine segmentation dataset.

Gazebo model truth is used only to create offline labels.  The visible silhouette
is rendered from the simple landmine geometry and checked against synchronized
depth, so background pixels inside a bounding box are not labelled as mine.
"""

import hashlib
import json
import math
import os
import re
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rospy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import CameraInfo, Image


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


def mine_mesh(radius=0.068, disc_height=0.025, block_size=(0.04, 0.04, 0.06)):
    """Return vertices/triangles matching gazebo_models/landmine/model.sdf."""
    vertices = []
    triangles = []
    segments = 32
    for z in (0.0, disc_height):
        for i in range(segments):
            a = 2.0 * math.pi * i / segments
            vertices.append((radius * math.cos(a), radius * math.sin(a), z))
    bottom_center = len(vertices); vertices.append((0.0, 0.0, 0.0))
    top_center = len(vertices); vertices.append((0.0, 0.0, disc_height))
    for i in range(segments):
        j = (i + 1) % segments
        triangles.extend([
            (i, j, segments + j), (i, segments + j, segments + i),
            (bottom_center, j, i),
            (top_center, segments + i, segments + j),
        ])

    sx, sy, sz = block_size
    z0 = disc_height
    base = len(vertices)
    for z in (z0, z0 + sz):
        for y in (-sy/2.0, sy/2.0):
            for x in (-sx/2.0, sx/2.0):
                vertices.append((x, y, z))
    # indices at each z: (-x,-y),(+x,-y),(-x,+y),(+x,+y)
    faces = [(0,1,3,2), (4,6,7,5), (0,4,5,1), (2,3,7,6), (0,2,6,4), (1,5,7,3)]
    for a, b, c, d in faces:
        triangles.extend([(base+a, base+b, base+c), (base+a, base+c, base+d)])
    return np.asarray(vertices, np.float64), np.asarray(triangles, np.int32)


class MineSegDatasetCollector:
    def __init__(self):
        self.rgb_topic = rospy.get_param("~rgb_topic", "/mine_camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/mine_camera/depth/image_raw")
        self.info_topic = rospy.get_param("~camera_info_topic", "/mine_camera/rgb/camera_info")
        self.fixed_frame = rospy.get_param("~fixed_frame", "map")
        self.camera_frame_override = rospy.get_param("~camera_frame", "mine_camera_optical_frame")
        self.model_pattern = re.compile(rospy.get_param("~mine_model_regex", r"^landmine(_.*)?$"))
        self.dataset_dir = Path(os.path.expanduser(rospy.get_param("~dataset_dir", "~/mine_yolo_dataset")))
        self.session = self._clean(rospy.get_param("~session", "default"))
        self.requested_split = rospy.get_param("~split", "auto")
        self.session_size = max(int(rospy.get_param("~session_size", 500)), 1)
        self.split = self._block_split(0) if self.requested_split == "auto" else self.requested_split
        if self.split not in ("train", "val", "test"):
            raise rospy.ROSInitException("~split must be auto/train/val/test")
        self.sample_rate = max(float(rospy.get_param("~sample_rate", 2.0)), 0.1)
        # The OpenNI Gazebo plugin publishes RGB and depth on separate timers;
        # their stamps can drift by more than 10 ms. Camera-settle gating below
        # prevents cross-pose pairing, so 40 ms keeps throughput without
        # reintroducing the teleport race.
        self.sync_slop = min(max(float(rospy.get_param("~sync_slop", 0.04)), 0.0), 0.05)
        self.max_new_images = max(int(rospy.get_param("~max_new_images", 0)), 0)
        self.negative_ratio = min(max(float(rospy.get_param("~negative_ratio", 0.30)), 0.0), 0.9)
        self.min_mask_area = max(int(rospy.get_param("~min_mask_area", 40)), 4)
        # The disc top is only 13--25 mm above the ground.  A 35 mm tolerance
        # therefore accepts plain ground as a visible mine when camera TF and
        # RGB are one teleport apart.  Gazebo depth is noise-free here, so a
        # much tighter gate is both safe and necessary.
        self.depth_tolerance = min(max(float(rospy.get_param("~depth_tolerance", 0.008)), 0.002), 0.012)
        self.depth_visibility = bool(rospy.get_param("~depth_visibility", True))
        self.allow_latest_tf = bool(rospy.get_param("~allow_latest_tf", False))
        self.require_camera_settle = bool(rospy.get_param("~require_camera_settle", False))
        self.camera_settle_time = max(float(rospy.get_param("~camera_settle_time", 0.10)), 0.0)
        self.scene_settle_time = max(float(rospy.get_param("~scene_settle_time", 0.25)), 0.0)
        # Exact JPEG hashes already prevent true duplicates.  The old 8x8
        # perceptual hash discarded valid small-object views because a mine can
        # disappear during such aggressive downsampling.  Negative disables it.
        self.dedupe_hamming = int(rospy.get_param("~dedupe_hamming", -1))
        self.world_name = rospy.get_param("~world_name", "unknown")
        self.domain_tag = rospy.get_param("~domain_tag", "standard")
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_info = None
        self.models = {}
        self.distractors = {}
        self.last_save = rospy.Time(0)
        self.last_hash = None
        self.saved_session = 0
        self.positive = 0
        self.negative = 0
        self.vertices, self.triangles = mine_mesh()

        for split in ("train", "val", "test"):
            (self.dataset_dir / "images" / split).mkdir(parents=True, exist_ok=True)
            (self.dataset_dir / "labels" / split).mkdir(parents=True, exist_ok=True)
            (self.dataset_dir / "metadata").mkdir(parents=True, exist_ok=True)
        self._write_yaml()
        self._write_manifest()
        self.next_index = self._next_index()
        self.exact_hashes = self._existing_hashes()

        rospy.Subscriber(self.info_topic, CameraInfo, self._info_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models_cb, queue_size=1)
        rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], 8, self.sync_slop, allow_headerless=False)
        sync.registerCallback(self._image_cb)
        self._sync = sync
        rospy.loginfo("[MineDataset] session=%s split=%s dataset=%s rgb=%s depth=%s",
                      self.session, self.split, self.dataset_dir, self.rgb_topic, self.depth_topic)

    @staticmethod
    def _clean(value):
        return re.sub(r"[^A-Za-z0-9_-]+", "_", str(value)).strip("_") or "default"

    @staticmethod
    def _session_split(session):
        bucket = int(hashlib.sha1(session.encode("utf-8")).hexdigest()[:8], 16) % 100
        return "train" if bucket < 70 else ("val" if bucket < 85 else "test")

    @staticmethod
    def _block_split(block):
        # Temporary raw-storage buckets only. curate_mine_dataset.py rebuilds
        # the final split with seed+cycle as the indivisible scene group.
        bucket = block % 7
        return "train" if bucket < 5 else ("val" if bucket == 5 else "test")

    def _write_yaml(self):
        with (self.dataset_dir / "mine_dataset.yaml").open("w", encoding="utf-8") as f:
            yaml.safe_dump({"path": str(self.dataset_dir), "train": "images/train",
                            "val": "images/val", "test": "images/test",
                            "names": {0: "landmine"}}, f, sort_keys=False)

    def _write_manifest(self):
        path = self.dataset_dir / "manifest.yaml"
        if path.exists():
            return
        data = {
            "format": "ultralytics-seg", "version": 2, "class_names": {0: "landmine"},
            "collector": "collect_mine_seg_dataset_node.py",
            "camera": {"rgb_topic": self.rgb_topic, "depth_topic": self.depth_topic,
                       "camera_info_topic": self.info_topic, "frame": self.camera_frame_override},
            "split_policy": "raw buckets only; prepare by randomizer_seed + domain_cycle before training",
            "truth_policy": "Gazebo model truth is for offline labels only",
            "capture_guard": {"sync_slop": self.sync_slop,
                              "depth_tolerance": self.depth_tolerance,
                              "camera_settle_time": self.camera_settle_time,
                              "scene_settle_time": self.scene_settle_time},
        }
        path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")

    def _next_index(self):
        maximum = -1
        for split in ("train", "val", "test"):
            for p in (self.dataset_dir / "images" / split).glob("*.jpg"):
                try: maximum = max(maximum, int(p.name.split("_", 1)[0]))
                except ValueError: pass
        return maximum + 1

    def _existing_hashes(self):
        hashes = set()
        for split in ("train", "val", "test"):
            for path in (self.dataset_dir / "images" / split).glob("*.jpg"):
                hashes.add(hashlib.sha256(path.read_bytes()).digest())
        return hashes

    def _info_cb(self, msg):
        self.camera_info = msg

    def _models_cb(self, msg):
        named_poses = dict(zip(msg.name, msg.pose))
        self.models = {name: pose for name, pose in named_poses.items()
                       if self.model_pattern.match(name)}
        self.distractors = {name: pose for name, pose in named_poses.items()
                            if name.startswith("distractor_disc_")}

    @staticmethod
    def _global_param(name, default):
        try:
            return rospy.get_param(name, default)
        except (KeyError, rospy.ROSException):
            return default

    def _capture_state_is_stable(self, stamp):
        """Reject frames rendered during a camera teleport or scene refresh."""
        if bool(self._global_param("/mine_dataset/randomizer_busy", False)):
            return False
        scene_change = float(self._global_param("/mine_dataset/randomizer_last_change_time", -1.0))
        if scene_change >= 0.0 and stamp.to_sec() - scene_change < self.scene_settle_time:
            return False
        if not self.require_camera_settle:
            return True
        if bool(self._global_param("/mine_dataset/camera_moving", True)):
            return False
        pose_seq = int(self._global_param("/mine_dataset/camera_pose_seq", -1))
        last_move = float(self._global_param("/mine_dataset/camera_last_move_time", -1.0))
        return pose_seq >= 0 and last_move >= 0.0 and stamp.to_sec() - last_move >= self.camera_settle_time

    def _lookup(self, camera_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(camera_frame, self.fixed_frame, stamp,
                                                   rospy.Duration(0.08))
        except Exception as exc:
            if not self.allow_latest_tf:
                rospy.logwarn_throttle(2.0, "[MineDataset] stamped TF unavailable: %s", exc)
                return None
            try:
                return self.tf_buffer.lookup_transform(camera_frame, self.fixed_frame,
                                                       rospy.Time(0), rospy.Duration(0.08))
            except Exception:
                return None

    def _map_to_camera(self, transform, points):
        r = quat_matrix(transform.transform.rotation)
        t = np.array([transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z])
        return points.dot(r.T) + t

    @staticmethod
    def _raster_triangle(zbuf, uv, z):
        h, w = zbuf.shape
        x0 = max(0, int(math.floor(np.min(uv[:, 0])))); x1 = min(w-1, int(math.ceil(np.max(uv[:, 0]))))
        y0 = max(0, int(math.floor(np.min(uv[:, 1])))); y1 = min(h-1, int(math.ceil(np.max(uv[:, 1]))))
        if x1 < x0 or y1 < y0:
            return
        a, b, c = uv
        den = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
        if abs(den) < 1e-8:
            return
        yy, xx = np.mgrid[y0:y1+1, x0:x1+1]
        w0 = ((b[1]-c[1])*(xx-c[0]) + (c[0]-b[0])*(yy-c[1])) / den
        w1 = ((c[1]-a[1])*(xx-c[0]) + (a[0]-c[0])*(yy-c[1])) / den
        w2 = 1.0 - w0 - w1
        inside = (w0 >= -1e-5) & (w1 >= -1e-5) & (w2 >= -1e-5)
        invz = w0/z[0] + w1/z[1] + w2/z[2]
        zz = np.where(invz > 1e-8, 1.0/invz, np.inf)
        view = zbuf[y0:y1+1, x0:x1+1]
        np.minimum(view, np.where(inside, zz, np.inf), out=view)

    def _model_mask(self, pose, shape, transform, depth):
        model_r = quat_matrix(pose.orientation)
        model_t = np.array([pose.position.x, pose.position.y, pose.position.z])
        points_map = self.vertices.dot(model_r.T) + model_t
        points_cam = self._map_to_camera(transform, points_map)
        if np.count_nonzero(points_cam[:, 2] > 0.05) < 3:
            return None
        fx, fy, cx, cy = (float(self.camera_info.K[i]) for i in (0, 4, 2, 5))
        uv = np.column_stack((fx*points_cam[:,0]/points_cam[:,2] + cx,
                              fy*points_cam[:,1]/points_cam[:,2] + cy))
        zbuf = np.full(shape, np.inf, np.float32)
        for tri in self.triangles:
            p = points_cam[tri]
            if np.all(p[:, 2] > 0.05):
                self._raster_triangle(zbuf, uv[tri], p[:, 2])
        mask = np.isfinite(zbuf)
        if self.depth_visibility:
            valid = mask & np.isfinite(depth) & (depth > 0.05)
            delta = np.full(shape, np.inf, np.float32)
            np.subtract(depth, zbuf, out=delta, where=valid)
            mask &= valid & (np.abs(delta) <= self.depth_tolerance)
        mask = mask.astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3,3), np.uint8))
        return mask

    @staticmethod
    def _contour_label(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0.0
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        eps = max(1.0, 0.005 * cv2.arcLength(contour, True))
        contour = cv2.approxPolyDP(contour, eps, True).reshape(-1, 2)
        return contour if len(contour) >= 3 else None, area

    @staticmethod
    def _ahash(image):
        small = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (8,8), interpolation=cv2.INTER_AREA)
        bits = small > np.mean(small)
        return bits.reshape(-1)

    def _should_negative(self):
        total = self.positive + self.negative
        return self.negative < self.negative_ratio * max(total + 1, 1)

    def _save(self, image, labels, metadata, stamp):
        digest = self._ahash(image) if self.dedupe_hamming >= 0 else None
        if (digest is not None and self.last_hash is not None and
                np.count_nonzero(digest != self.last_hash) <= self.dedupe_hamming):
            return
        block = self.saved_session // self.session_size
        active_session = f"{self.session}_{block:03d}"
        active_split = (self._block_split(block) if self.requested_split == "auto"
                        else self.requested_split)
        name = f"{self.next_index:07d}_{active_session}_{stamp.secs:010d}_{stamp.nsecs:09d}"
        image_path = self.dataset_dir / "images" / active_split / (name + ".jpg")
        label_path = self.dataset_dir / "labels" / active_split / (name + ".txt")
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 94])
        if not ok:
            return
        encoded_bytes = encoded.tobytes()
        exact_hash = hashlib.sha256(encoded_bytes).digest()
        if exact_hash in self.exact_hashes:
            return
        image_path.write_bytes(encoded_bytes)
        h, w = image.shape[:2]
        lines = []
        for contour in labels:
            norm = contour.astype(np.float64)
            norm[:,0] = np.clip(norm[:,0] / w, 0.0, 1.0)
            norm[:,1] = np.clip(norm[:,1] / h, 0.0, 1.0)
            lines.append("0 " + " ".join(f"{v:.8f}" for v in norm.reshape(-1)))
        label_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")
        metadata.update({"image": image_path.name, "split": active_split, "session": active_session,
                         "stamp": {"secs": stamp.secs, "nsecs": stamp.nsecs},
                         "world": self.world_name, "domain_tag": self.domain_tag,
                         "domain_cycle": int(rospy.get_param("/mine_dataset/domain_cycle", -1)),
                         "light_brightness": float(rospy.get_param("/mine_dataset/light_brightness", -1.0)),
                         "instances": len(labels)})
        with (self.dataset_dir / "metadata" / f"{active_split}.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(metadata, ensure_ascii=False) + "\n")
        self.positive += int(bool(labels)); self.negative += int(not labels)
        self.exact_hashes.add(exact_hash)
        self.next_index += 1; self.saved_session += 1; self.last_hash = digest
        rospy.loginfo_throttle(1.0, "[MineDataset] saved=%d session=%d positive=%d negative=%d",
                               self.next_index, self.saved_session, self.positive, self.negative)

    def _image_cb(self, rgb_msg, depth_msg):
        if self.max_new_images and self.saved_session >= self.max_new_images:
            rospy.signal_shutdown("mine dataset session complete")
            return
        now = rospy.Time.now()
        if self.last_save != rospy.Time(0) and (now-self.last_save).to_sec() < 1.0/self.sample_rate:
            return
        if self.camera_info is None:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = np.asarray(self.bridge.imgmsg_to_cv2(depth_msg, "passthrough"))
            depth = depth.astype(np.float32) * (0.001 if depth.dtype == np.uint16 else 1.0)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[MineDataset] image conversion failed: %s", exc)
            return
        stamp = rgb_msg.header.stamp if rgb_msg.header.stamp != rospy.Time() else now
        if not self._capture_state_is_stable(stamp):
            return
        # Throttle expensive mesh rasterization attempts, not only successful
        # saves. Otherwise a negative view that is currently over quota is
        # processed at the full 8 Hz sensor rate and the callback queue falls
        # behind the 20 s TF cache.
        self.last_save = now
        frame = self.camera_frame_override or rgb_msg.header.frame_id
        transform = self._lookup(frame, stamp)
        if transform is None:
            return
        labels, mines = [], []
        for name, pose in sorted(self.models.items()):
            mask = self._model_mask(pose, image.shape[:2], transform, depth)
            contour, area = self._contour_label(mask) if mask is not None else (None, 0.0)
            if contour is not None and area >= self.min_mask_area:
                labels.append(contour)
                mines.append({"name": name, "map_position": [pose.position.x, pose.position.y, pose.position.z],
                              "visible_mask_area": area})
        if not labels and not self._should_negative():
            return
        t = transform.transform
        distractors = [
            {"name": name, "map_position": [pose.position.x, pose.position.y, pose.position.z]}
            for name, pose in sorted(self.distractors.items())
        ]
        metadata = {"camera_frame": frame,
                    "camera_intrinsics": [self.camera_info.K[0], self.camera_info.K[4],
                                          self.camera_info.K[2], self.camera_info.K[5]],
                    "map_to_camera": {"translation": [t.translation.x, t.translation.y, t.translation.z],
                                      "quaternion": [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]},
                    "mines": mines,
                    "distractors": distractors,
                    "randomizer_seed": int(self._global_param("/mine_dataset/randomizer_seed", -1)),
                    "camera_pose_seq": int(self._global_param("/mine_dataset/camera_pose_seq", -1))}
        self._save(image, labels, metadata, stamp)


def main():
    rospy.init_node("collect_mine_seg_dataset")
    MineSegDatasetCollector()
    rospy.spin()


if __name__ == "__main__":
    main()
