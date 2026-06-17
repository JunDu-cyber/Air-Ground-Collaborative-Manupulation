#!/usr/bin/env python3
"""Velodyne PointCloud2 cluster detector for Stage 6 target observations."""

import math
from collections import defaultdict, deque

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tft
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Header, UInt32
from visualization_msgs.msg import Marker, MarkerArray


def finite_xyz(point):
    return all(math.isfinite(v) for v in point[:3])


def dist3(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class TargetLidarDetector:
    def __init__(self):
        self.fixed_frame = rospy.get_param("~fixed_frame", "map")
        self.lidar_cloud_topic = rospy.get_param(
            "~lidar_cloud_topic", "/uav0/velodyne_points"
        )
        self.uav0_odom_topic = rospy.get_param(
            "~uav0_odom_topic", "/uav0/mavros/local_position/odom"
        )
        self.output_pose_topic = rospy.get_param(
            "~output_pose_topic", "/target_observation/pose"
        )
        self.pose_cov_topic = rospy.get_param(
            "~pose_cov_topic", "/target_observation/pose_cov"
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/target_observation/marker"
        )
        self.debug_cloud_topic = rospy.get_param(
            "~debug_cloud_topic", "/target_observation/debug_cloud"
        )
        self.valid_topic = rospy.get_param("~valid_topic", "/target_observation/valid")
        self.confidence_topic = rospy.get_param(
            "~confidence_topic", "/target_observation/confidence"
        )
        self.point_count_topic = rospy.get_param(
            "~point_count_topic", "/target_observation/point_count"
        )

        self.range_min = float(rospy.get_param("~range_min", 0.5))
        self.range_max = float(rospy.get_param("~range_max", 20.0))
        self.z_min = float(rospy.get_param("~z_min", 2.0))
        self.z_max = float(rospy.get_param("~z_max", 5.0))
        self.voxel_leaf_size = max(float(rospy.get_param("~voxel_leaf_size", 0.08)), 1e-4)
        self.cluster_tolerance = max(
            float(rospy.get_param("~cluster_tolerance", 0.25)), 1e-4
        )
        self.min_cluster_size = max(int(rospy.get_param("~min_cluster_size", 5)), 1)
        self.max_cluster_size = max(
            int(rospy.get_param("~max_cluster_size", 500)), self.min_cluster_size
        )
        self.target_size_min = max(float(rospy.get_param("~target_size_min", 0.15)), 0.0)
        self.target_size_max = max(
            float(rospy.get_param("~target_size_max", 1.5)), self.target_size_min
        )
        self.expected_target_size = float(rospy.get_param("~expected_target_size", 0.6))
        self.enable_shape_filter = bool(rospy.get_param("~enable_shape_filter", True))
        self.target_xy_size_min = max(
            float(rospy.get_param("~target_xy_size_min", 0.05)), 0.0
        )
        self.target_xy_size_max = max(
            float(rospy.get_param("~target_xy_size_max", 1.20)),
            self.target_xy_size_min,
        )
        self.target_z_size_min = max(
            float(rospy.get_param("~target_z_size_min", 0.0)), 0.0
        )
        self.target_z_size_max = max(
            float(rospy.get_param("~target_z_size_max", 0.80)),
            self.target_z_size_min,
        )
        self.target_aspect_ratio_max = max(
            float(rospy.get_param("~target_aspect_ratio_max", 12.0)), 1.0
        )
        self.expected_target_xy_size = max(
            float(rospy.get_param("~expected_target_xy_size", 0.55)), 1e-3
        )
        self.expected_target_z_size = max(
            float(rospy.get_param("~expected_target_z_size", 0.25)), 1e-3
        )
        self.reference_distance_weight = max(
            float(rospy.get_param("~reference_distance_weight", 0.65)), 0.0
        )
        self.static_clutter_filter_enable = bool(
            rospy.get_param("~static_clutter_filter_enable", False)
        )
        self.static_clutter_min_candidates = max(
            int(rospy.get_param("~static_clutter_min_candidates", 2)), 2
        )
        self.static_clutter_cell_size = max(
            float(rospy.get_param("~static_clutter_cell_size", 0.40)), 0.05
        )
        self.static_clutter_confirm_count = max(
            int(rospy.get_param("~static_clutter_confirm_count", 5)), 2
        )
        self.static_clutter_max_spread = max(
            float(rospy.get_param("~static_clutter_max_spread", 0.15)), 0.0
        )
        self.static_clutter_memory = max(
            float(rospy.get_param("~static_clutter_memory", 5.0)), 0.5
        )
        self.static_clutter_startup_hold = bool(
            rospy.get_param("~static_clutter_startup_hold", True)
        )
        self.allow_reference_fallback = bool(
            rospy.get_param("~allow_reference_fallback", True)
        )
        self.use_estimator_gating = bool(rospy.get_param("~use_estimator_gating", True))
        self.gating_radius = max(float(rospy.get_param("~gating_radius", 2.0)), 0.0)
        self.max_detection_gap = max(
            float(rospy.get_param("~max_detection_gap", 0.5)), 0.0
        )
        self.continuity_enable = bool(rospy.get_param("~continuity_enable", True))
        self.continuity_max_age = max(
            float(rospy.get_param("~continuity_max_age", 2.0)), 0.0
        )
        self.continuity_radius = max(
            float(rospy.get_param("~continuity_radius", 3.0)), 0.0
        )
        self.reject_far_without_reference = bool(
            rospy.get_param("~reject_far_without_reference", True)
        )
        self.max_ungated_range = max(
            float(rospy.get_param("~max_ungated_range", 12.0)), self.range_min
        )
        self.near_reference_bonus_radius = max(
            float(rospy.get_param("~near_reference_bonus_radius", 6.0)), 0.0
        )
        self.publish_debug_cloud = bool(rospy.get_param("~publish_debug_cloud", True))
        self.use_receive_time = bool(rospy.get_param("~use_receive_time", True))
        self.max_logged_cloud_points = max(
            int(rospy.get_param("~max_logged_cloud_points", 200000)), 1000
        )

        self.estimator_state_topic = rospy.get_param(
            "~estimator_state_topic", "/target_estimator/state"
        )
        self.estimator_intercept_topic = rospy.get_param(
            "~estimator_intercept_topic", "/target_estimator/intercept_point"
        )
        self.gating_source = rospy.get_param("~gating_source", "state").lower()
        if self.gating_source not in ("state", "intercept", "auto"):
            rospy.logwarn(
                "[TargetLidarDetector] unknown gating_source '%s', using state",
                self.gating_source,
            )
            self.gating_source = "state"

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.last_frame_log = None
        self.last_diag_log = rospy.Time(0)

        self.uav0_pos = None
        self.uav0_stamp = None
        self.estimator_state_pos = None
        self.estimator_state_stamp = None
        self.estimator_intercept_pos = None
        self.estimator_intercept_stamp = None
        self.last_detection_pos = None
        self.last_detection_stamp = None
        self.static_clutter_cells = {}
        self.static_clutter_learning_frames = 0
        self.static_clutter_ready = False

        self.pose_pub = rospy.Publisher(self.output_pose_topic, PoseStamped, queue_size=10)
        self.pose_cov_pub = rospy.Publisher(
            self.pose_cov_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=10)
        self.debug_cloud_pub = rospy.Publisher(
            self.debug_cloud_topic, PointCloud2, queue_size=2
        )
        self.valid_pub = rospy.Publisher(self.valid_topic, Bool, queue_size=10)
        self.confidence_pub = rospy.Publisher(self.confidence_topic, Float32, queue_size=10)
        self.point_count_pub = rospy.Publisher(self.point_count_topic, UInt32, queue_size=10)

        self.odom_sub = rospy.Subscriber(
            self.uav0_odom_topic, Odometry, self.uav0_odom_cb, queue_size=10
        )
        self.cloud_sub = rospy.Subscriber(
            self.lidar_cloud_topic, PointCloud2, self.cloud_cb, queue_size=1
        )
        self.estimator_state_sub = rospy.Subscriber(
            self.estimator_state_topic, Odometry, self.estimator_state_cb, queue_size=5
        )
        self.estimator_intercept_sub = rospy.Subscriber(
            self.estimator_intercept_topic,
            PoseStamped,
            self.estimator_intercept_cb,
            queue_size=5,
        )

        rospy.loginfo(
            "[TargetLidarDetector] cloud=%s output=%s marker=%s debug_cloud=%s "
            "fixed_frame=%s uav0_odom=%s estimator_state=%s estimator_intercept=%s "
            "range=[%.2f %.2f] z=[%.2f %.2f] voxel=%.3f cluster_tol=%.2f "
            "cluster_size=[%d %d] target_size=[%.2f %.2f] estimator_gating=%s "
            "shape_filter=%s xy=[%.2f %.2f] z_extent=[%.2f %.2f] aspect_max=%.2f "
            "static_clutter=%s min_candidates=%d confirm=%d cell=%.2f spread=%.2f "
            "startup_hold=%s reference_fallback=%s gating_radius=%.2f continuity=%s radius=%.2f age=%.2f max_ungated_range=%.2f",
            self.lidar_cloud_topic,
            self.output_pose_topic,
            self.marker_topic,
            self.debug_cloud_topic,
            self.fixed_frame,
            self.uav0_odom_topic,
            self.estimator_state_topic,
            self.estimator_intercept_topic,
            self.range_min,
            self.range_max,
            self.z_min,
            self.z_max,
            self.voxel_leaf_size,
            self.cluster_tolerance,
            self.min_cluster_size,
            self.max_cluster_size,
            self.target_size_min,
            self.target_size_max,
            self.use_estimator_gating,
            self.enable_shape_filter,
            self.target_xy_size_min,
            self.target_xy_size_max,
            self.target_z_size_min,
            self.target_z_size_max,
            self.target_aspect_ratio_max,
            self.static_clutter_filter_enable,
            self.static_clutter_min_candidates,
            self.static_clutter_confirm_count,
            self.static_clutter_cell_size,
            self.static_clutter_max_spread,
            self.static_clutter_startup_hold,
            self.allow_reference_fallback,
            self.gating_radius,
            self.continuity_enable,
            self.continuity_radius,
            self.continuity_max_age,
            self.max_ungated_range,
        )

    def uav0_odom_cb(self, msg):
        self.uav0_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        self.uav0_stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()

    def estimator_state_cb(self, msg):
        self.estimator_state_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        self.estimator_state_stamp = (
            msg.header.stamp if msg.header.stamp else rospy.Time.now()
        )

    def estimator_intercept_cb(self, msg):
        self.estimator_intercept_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self.estimator_intercept_stamp = (
            msg.header.stamp if msg.header.stamp else rospy.Time.now()
        )

    def _fresh(self, stamp, now, max_age):
        if stamp is None:
            return False
        return 0.0 <= (now - stamp).to_sec() <= max_age

    def _select_gating_point(self, now):
        if self.continuity_enable and self.last_detection_pos is not None:
            if self._fresh(self.last_detection_stamp, now, self.continuity_max_age):
                return self.last_detection_pos, "last_detection"

        if not self.use_estimator_gating:
            return None, "disabled"
        sources = []
        if self.gating_source in ("state", "auto"):
            sources.append(("estimator_state", self.estimator_state_pos, self.estimator_state_stamp))
        if self.gating_source in ("intercept", "auto"):
            sources.append(
                ("estimator_intercept", self.estimator_intercept_pos, self.estimator_intercept_stamp)
            )
        for name, pos, stamp in sources:
            if pos is not None and self._fresh(stamp, now, self.max_detection_gap):
                return pos, name
        return None, "none"

    def _lookup_transform(self, source_frame, stamp):
        if not source_frame:
            return None
        if source_frame == self.fixed_frame:
            return None
        lookup_stamp = rospy.Time(0) if self.use_receive_time else stamp
        return self.tf_buffer.lookup_transform(
            self.fixed_frame,
            source_frame,
            lookup_stamp,
            rospy.Duration(0.03),
        )

    def _transform_points(self, points, source_frame, stamp):
        if source_frame == self.fixed_frame:
            return points, True, self.fixed_frame, "same_frame"
        try:
            tf_msg = self._lookup_transform(source_frame, stamp)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0,
                "[TargetLidarDetector] TF %s -> %s failed: %s; using source frame points",
                source_frame,
                self.fixed_frame,
                exc,
            )
            return points, False, source_frame or "", "tf_failed"

        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        mat = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        out = []
        for x, y, z, src_range in points:
            tx = mat[0][0] * x + mat[0][1] * y + mat[0][2] * z + trans.x
            ty = mat[1][0] * x + mat[1][1] * y + mat[1][2] * z + trans.y
            tz = mat[2][0] * x + mat[2][1] * y + mat[2][2] * z + trans.z
            out.append((tx, ty, tz, src_range))
        return out, True, self.fixed_frame, "tf_ok"

    def _read_cloud(self, msg):
        raw_points = msg.width * msg.height
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
            if finite_xyz(p):
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                src_range = math.sqrt(x * x + y * y + z * z)
                points.append((x, y, z, src_range))
                if len(points) >= self.max_logged_cloud_points:
                    break
        return raw_points, points

    def _filter_points(self, points, tf_ok):
        filtered = []
        for x, y, z, src_range in points:
            if self.uav0_pos is not None and tf_ok:
                range_value = dist3((x, y, z), self.uav0_pos)
            else:
                range_value = src_range
            if range_value < self.range_min or range_value > self.range_max:
                continue
            if z < self.z_min or z > self.z_max:
                continue
            filtered.append((x, y, z))
        return filtered

    def _filter_debug_stats(self, points, tf_ok):
        if not points:
            return None
        min_range = float("inf")
        max_range = 0.0
        min_z = float("inf")
        max_z = -float("inf")
        rejected_by_range = 0
        rejected_by_z_after_range = 0
        passed = 0
        for x, y, z, src_range in points:
            if self.uav0_pos is not None and tf_ok:
                range_value = dist3((x, y, z), self.uav0_pos)
            else:
                range_value = src_range
            min_range = min(min_range, range_value)
            max_range = max(max_range, range_value)
            min_z = min(min_z, z)
            max_z = max(max_z, z)
            if range_value < self.range_min or range_value > self.range_max:
                rejected_by_range += 1
                continue
            if z < self.z_min or z > self.z_max:
                rejected_by_z_after_range += 1
                continue
            passed += 1
        return {
            "min_range": min_range,
            "max_range": max_range,
            "min_z": min_z,
            "max_z": max_z,
            "rejected_by_range": rejected_by_range,
            "rejected_by_z_after_range": rejected_by_z_after_range,
            "passed": passed,
        }

    def _voxel_downsample(self, points):
        if not points:
            return []
        leaf = self.voxel_leaf_size
        buckets = {}
        counts = defaultdict(int)
        for x, y, z in points:
            key = (
                int(math.floor(x / leaf)),
                int(math.floor(y / leaf)),
                int(math.floor(z / leaf)),
            )
            if key not in buckets:
                buckets[key] = [0.0, 0.0, 0.0]
            buckets[key][0] += x
            buckets[key][1] += y
            buckets[key][2] += z
            counts[key] += 1
        return [
            (s[0] / counts[k], s[1] / counts[k], s[2] / counts[k])
            for k, s in buckets.items()
        ]

    def _cluster_points(self, points):
        if not points:
            return []
        cell = self.cluster_tolerance
        grid = defaultdict(list)
        for idx, point in enumerate(points):
            key = (
                int(math.floor(point[0] / cell)),
                int(math.floor(point[1] / cell)),
                int(math.floor(point[2] / cell)),
            )
            grid[key].append(idx)

        visited = [False] * len(points)
        clusters = []
        tol2 = self.cluster_tolerance * self.cluster_tolerance
        for start in range(len(points)):
            if visited[start]:
                continue
            visited[start] = True
            q = deque([start])
            cluster = []
            while q:
                idx = q.popleft()
                cluster.append(idx)
                p = points[idx]
                key = (
                    int(math.floor(p[0] / cell)),
                    int(math.floor(p[1] / cell)),
                    int(math.floor(p[2] / cell)),
                )
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dz in (-1, 0, 1):
                            for nb in grid.get((key[0] + dx, key[1] + dy, key[2] + dz), []):
                                if visited[nb]:
                                    continue
                                np = points[nb]
                                d2 = (
                                    (p[0] - np[0]) ** 2
                                    + (p[1] - np[1]) ** 2
                                    + (p[2] - np[2]) ** 2
                                )
                                if d2 <= tol2:
                                    visited[nb] = True
                                    q.append(nb)
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                clusters.append(cluster)
        return clusters

    def _cluster_info(self, points, indices):
        xs = [points[i][0] for i in indices]
        ys = [points[i][1] for i in indices]
        zs = [points[i][2] for i in indices]
        n = float(len(indices))
        centroid = (sum(xs) / n, sum(ys) / n, sum(zs) / n)
        sx = max(xs) - min(xs)
        sy = max(ys) - min(ys)
        sz = max(zs) - min(zs)
        diag = math.sqrt(sx * sx + sy * sy + sz * sz)
        xy_diag = math.sqrt(sx * sx + sy * sy)
        extents = sorted((sx, sy, sz), reverse=True)
        aspect_ratio = extents[0] / max(extents[1], 0.05)
        if self.uav0_pos is not None:
            range_value = dist3(centroid, self.uav0_pos)
        else:
            range_value = math.sqrt(
                centroid[0] * centroid[0]
                + centroid[1] * centroid[1]
                + centroid[2] * centroid[2]
            )
        return {
            "indices": indices,
            "centroid": centroid,
            "point_count": len(indices),
            "bbox_size_x": sx,
            "bbox_size_y": sy,
            "bbox_size_z": sz,
            "bbox_diag": diag,
            "bbox_xy_diag": xy_diag,
            "aspect_ratio": aspect_ratio,
            "range": range_value,
            "z": centroid[2],
        }

    def _shape_valid(self, info):
        if not self.enable_shape_filter:
            return True
        return (
            self.target_xy_size_min
            <= info["bbox_xy_diag"]
            <= self.target_xy_size_max
            and self.target_z_size_min
            <= info["bbox_size_z"]
            <= self.target_z_size_max
            and info["aspect_ratio"] <= self.target_aspect_ratio_max
        )

    def _static_cell_key(self, centroid):
        cell = self.static_clutter_cell_size
        return (
            int(math.floor(centroid[0] / cell)),
            int(math.floor(centroid[1] / cell)),
            int(math.floor(centroid[2] / cell)),
        )

    def _filter_static_clutter(self, infos, now):
        if not self.static_clutter_filter_enable:
            return infos, 0

        expired = [
            key
            for key, entry in self.static_clutter_cells.items()
            if (now - entry["last_stamp"]).to_sec() > self.static_clutter_memory
        ]
        for key in expired:
            del self.static_clutter_cells[key]

        can_learn = len(infos) >= self.static_clutter_min_candidates
        if can_learn and not self.static_clutter_ready:
            self.static_clutter_learning_frames += 1
            if self.static_clutter_learning_frames >= self.static_clutter_confirm_count:
                self.static_clutter_ready = True
        for info in infos:
            key = self._static_cell_key(info["centroid"])
            entry = self.static_clutter_cells.get(key)
            if entry is None:
                if not can_learn:
                    info["static_clutter"] = False
                    continue
                self.static_clutter_cells[key] = {
                    "origin": info["centroid"],
                    "last_stamp": now,
                    "count": 1,
                }
                info["static_clutter"] = False
                continue
            if not can_learn:
                info["static_clutter"] = (
                    entry["count"] >= self.static_clutter_confirm_count
                )
                continue
            if dist3(info["centroid"], entry["origin"]) <= self.static_clutter_max_spread:
                entry["count"] += 1
            else:
                entry["origin"] = info["centroid"]
                entry["count"] = 1
            entry["last_stamp"] = now
            info["static_clutter"] = (
                entry["count"] >= self.static_clutter_confirm_count
            )

        filtered = [info for info in infos if not info.get("static_clutter", False)]
        if (
            self.static_clutter_startup_hold
            and can_learn
            and not self.static_clutter_ready
        ):
            return [], len(infos)
        return filtered, len(infos) - len(filtered)

    def _score_cluster(self, info, gating_point):
        size_cost = abs(info["bbox_diag"] - self.expected_target_size)
        xy_size_cost = abs(info["bbox_xy_diag"] - self.expected_target_xy_size)
        z_size_cost = abs(info["bbox_size_z"] - self.expected_target_z_size)
        aspect_cost = max(info["aspect_ratio"] - 1.0, 0.0) / max(
            self.target_aspect_ratio_max - 1.0, 1e-3
        )
        range_cost = max(info["range"] - self.range_min, 0.0) / max(
            self.range_max - self.range_min, 1e-3
        )
        point_bonus = min(info["point_count"], 30) / 30.0
        score = (
            0.25 * size_cost
            + 0.30 * xy_size_cost
            + 0.20 * z_size_cost
            + 0.15 * aspect_cost
            + 0.15 * range_cost
            - 0.10 * point_bonus
        )
        if gating_point is not None:
            reference_radius = max(self.gating_radius, self.continuity_radius, 1e-3)
            score += self.reference_distance_weight * min(
                dist3(info["centroid"], gating_point) / reference_radius,
                2.0,
            )
            if dist3(info["centroid"], gating_point) <= self.near_reference_bonus_radius:
                score -= 0.15
        return score

    def _select_cluster(self, infos, now):
        if not infos:
            return None, None, "none"

        gating_point, gating_name = self._select_gating_point(now)
        if gating_point is None and self.reject_far_without_reference:
            infos = [info for info in infos if info["range"] <= self.max_ungated_range]
            if not infos:
                return None, None, "far_without_reference"

        if len(infos) == 1:
            return infos[0], gating_point, "single_cluster"

        if gating_point is not None:
            radius = self.continuity_radius if gating_name == "last_detection" else self.gating_radius
            gated = [
                info
                for info in infos
                if dist3(info["centroid"], gating_point) <= radius
            ]
            if gated:
                selected = min(
                    gated, key=lambda info: self._score_cluster(info, gating_point)
                )
                return selected, gating_point, gating_name
            if self.allow_reference_fallback:
                selected = min(
                    infos, key=lambda info: self._score_cluster(info, gating_point)
                )
                return selected, gating_point, "{}_fallback_score".format(gating_name)
            return None, gating_point, "{}_no_near_cluster".format(gating_name)

        selected = min(infos, key=lambda info: self._score_cluster(info, gating_point))
        return selected, gating_point, "score"

    def _confidence(self, selected, cluster_count, gating_point):
        if selected is None:
            return 0.0
        size_error = abs(selected["bbox_diag"] - self.expected_target_size)
        size_quality = max(0.0, 1.0 - size_error / max(self.expected_target_size, 1e-3))
        xy_error = abs(selected["bbox_xy_diag"] - self.expected_target_xy_size)
        xy_quality = max(
            0.0, 1.0 - xy_error / max(self.expected_target_xy_size, 1e-3)
        )
        z_error = abs(selected["bbox_size_z"] - self.expected_target_z_size)
        z_quality = max(
            0.0, 1.0 - z_error / max(self.expected_target_z_size, 1e-3)
        )
        point_quality = min(selected["point_count"], 30) / 30.0
        range_quality = max(0.0, 1.0 - selected["range"] / max(self.range_max, 1e-3))
        gate_quality = 1.0
        if gating_point is not None:
            gate_quality = max(
                0.0, 1.0 - dist3(selected["centroid"], gating_point) / max(self.gating_radius, 1e-3)
            )
        clutter_penalty = 1.0 / max(cluster_count, 1)
        return max(
            0.05,
            min(
                1.0,
                0.20 * size_quality
                + 0.20 * xy_quality
                + 0.15 * z_quality
                + 0.20 * point_quality
                + 0.15 * range_quality
                + 0.05 * gate_quality
                + 0.05 * clutter_penalty,
            ),
        )

    def _publish_invalid(self, header, frame_id):
        self.valid_pub.publish(Bool(data=False))
        self.confidence_pub.publish(Float32(data=0.0))
        self.point_count_pub.publish(UInt32(data=0))
        self._publish_markers(header, frame_id, [], None)

    def _publish_pose(self, header, frame_id, selected, confidence):
        # Publish valid + confidence BEFORE the pose so that subscribers
        # (fusion, estimator) see the updated flags when they process the
        # pose callback.  Reversing this order causes a one-message race:
        # the pose arrives while valid is still the stale False from the
        # previous cycle, and the observation is silently rejected.
        self.valid_pub.publish(Bool(data=True))
        self.confidence_pub.publish(Float32(data=confidence))
        self.point_count_pub.publish(UInt32(data=selected["point_count"]))

        pose = PoseStamped()
        pose.header = Header(stamp=header.stamp, frame_id=frame_id)
        pose.pose.position.x = selected["centroid"][0]
        pose.pose.position.y = selected["centroid"][1]
        pose.pose.position.z = selected["centroid"][2]
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        cov = PoseWithCovarianceStamped()
        cov.header = pose.header
        cov.pose.pose = pose.pose
        variance = max(0.02, 0.5 * (1.0 - confidence))
        cov.pose.covariance[0] = variance
        cov.pose.covariance[7] = variance
        cov.pose.covariance[14] = variance
        cov.pose.covariance[21] = 1.0
        cov.pose.covariance[28] = 1.0
        cov.pose.covariance[35] = 1.0
        self.pose_cov_pub.publish(cov)
        self.last_detection_pos = selected["centroid"]
        self.last_detection_stamp = header.stamp

    def _publish_debug_cloud(self, header, frame_id, points):
        if not self.publish_debug_cloud:
            return
        cloud = pc2.create_cloud_xyz32(Header(stamp=header.stamp, frame_id=frame_id), points)
        self.debug_cloud_pub.publish(cloud)

    def _sphere_marker(self, header, marker_id, ns, pos, color, scale):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def _bbox_marker(self, header, marker_id, info, selected):
        marker = Marker()
        marker.header = header
        marker.ns = "target_lidar_selected" if selected else "target_lidar_candidates"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = info["centroid"][0]
        marker.pose.position.y = info["centroid"][1]
        marker.pose.position.z = info["centroid"][2]
        marker.scale.x = max(info["bbox_size_x"], 0.08)
        marker.scale.y = max(info["bbox_size_y"], 0.08)
        marker.scale.z = max(info["bbox_size_z"], 0.08)
        if selected:
            marker.color.r = 1.0
            marker.color.g = 0.35
            marker.color.b = 0.0
            marker.color.a = 0.65
        else:
            marker.color.r = 0.0
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.25
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def _text_marker(self, header, marker_id, selected, confidence):
        marker = Marker()
        marker.header = header
        marker.ns = "target_lidar_label"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = selected["centroid"][0]
        marker.pose.position.y = selected["centroid"][1]
        marker.pose.position.z = selected["centroid"][2] + 0.55
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.text = "LiDAR target\npts={} conf={:.2f}".format(
            selected["point_count"], confidence
        )
        marker.text += "\nxy={:.2f} z={:.2f} ar={:.1f}".format(
            selected["bbox_xy_diag"],
            selected["bbox_size_z"],
            selected["aspect_ratio"],
        )
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def _publish_markers(self, header, frame_id, infos, selected, confidence=0.0):
        marker_header = Header(stamp=header.stamp, frame_id=frame_id)
        markers = MarkerArray()
        selected_id = id(selected) if selected is not None else None
        for i, info in enumerate(infos[:30]):
            is_selected = id(info) == selected_id
            markers.markers.append(self._bbox_marker(marker_header, i, info, is_selected))
        if selected is not None:
            markers.markers.append(
                self._sphere_marker(
                    marker_header,
                    1000,
                    "target_lidar_centroid",
                    selected["centroid"],
                    (1.0, 0.1, 0.1, 0.95),
                    0.20,
                )
            )
            markers.markers.append(
                self._text_marker(marker_header, 1001, selected, confidence)
            )
        self.marker_pub.publish(markers)

    def _log_diag(
        self,
        raw_points,
        finite_points,
        tf_ok,
        filtered_points,
        voxel_points,
        raw_cluster_count,
        cluster_count,
        size_rejected,
        shape_rejected,
        static_rejected,
        selected,
        published_pose,
        confidence,
        frame_id,
        selection_reason,
        filter_stats=None,
    ):
        now = rospy.Time.now()
        if (now - self.last_diag_log).to_sec() < 1.0:
            return
        self.last_diag_log = now
        if filtered_points == 0 and filter_stats is not None:
            rospy.logwarn(
                "[TargetLidarDetectorFilter] filtered_points=0 "
                "input_range=[%.2f %.2f] input_z=[%.2f %.2f] "
                "rejected_by_range=%d rejected_by_z_after_range=%d "
                "params_range=[%.2f %.2f] params_z=[%.2f %.2f]",
                filter_stats["min_range"],
                filter_stats["max_range"],
                filter_stats["min_z"],
                filter_stats["max_z"],
                filter_stats["rejected_by_range"],
                filter_stats["rejected_by_z_after_range"],
                self.range_min,
                self.range_max,
                self.z_min,
                self.z_max,
            )
        selected_points = selected["point_count"] if selected else 0
        selected_range = selected["range"] if selected else float("nan")
        selected_z = selected["z"] if selected else float("nan")
        selected_diag = selected["bbox_diag"] if selected else float("nan")
        rospy.loginfo(
            "[TargetLidarDetector] raw_points=%d finite_points=%d tf_ok=%s "
            "filtered_points=%d voxel_points=%d raw_cluster_count=%d cluster_count=%d "
            "size_rejected=%d shape_rejected=%d static_rejected=%d selected_points=%d "
            "selected_range=%.2f selected_z=%.2f selected_bbox_diag=%.2f "
            "published_pose=%s confidence=%.2f frame_id=%s selection=%s",
            raw_points,
            finite_points,
            tf_ok,
            filtered_points,
            voxel_points,
            raw_cluster_count,
            cluster_count,
            size_rejected,
            shape_rejected,
            static_rejected,
            selected_points,
            selected_range,
            selected_z,
            selected_diag,
            published_pose,
            confidence,
            frame_id,
            selection_reason,
        )

    def cloud_cb(self, msg):
        if msg.header.frame_id != self.last_frame_log:
            self.last_frame_log = msg.header.frame_id
            rospy.loginfo(
                "[TargetLidarDetector] cloud.header.frame_id='%s'",
                msg.header.frame_id,
            )

        stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        raw_points, source_points = self._read_cloud(msg)
        transformed, tf_ok, working_frame, _ = self._transform_points(
            source_points, msg.header.frame_id, stamp
        )
        filtered = self._filter_points(transformed, tf_ok)
        filter_stats = self._filter_debug_stats(transformed, tf_ok) if not filtered else None
        voxel = self._voxel_downsample(filtered)
        clusters = self._cluster_points(voxel)

        infos = []
        size_rejected = 0
        shape_rejected = 0
        for cluster in clusters:
            info = self._cluster_info(voxel, cluster)
            if not self.target_size_min <= info["bbox_diag"] <= self.target_size_max:
                size_rejected += 1
                continue
            if not self._shape_valid(info):
                shape_rejected += 1
                continue
            infos.append(info)

        now = rospy.Time.now()
        infos, static_rejected = self._filter_static_clutter(infos, now)
        selected, gating_point, selection_reason = self._select_cluster(
            infos, now
        )
        confidence = self._confidence(selected, len(infos), gating_point)
        header = Header(stamp=stamp, frame_id=working_frame)

        if selected is not None:
            self._publish_pose(header, working_frame, selected, confidence)
            self._publish_markers(header, working_frame, infos, selected, confidence)
            published_pose = True
        else:
            self._publish_invalid(header, working_frame)
            published_pose = False

        self._publish_debug_cloud(header, working_frame, voxel)
        self._log_diag(
            raw_points=raw_points,
            finite_points=len(source_points),
            tf_ok=tf_ok,
            filtered_points=len(filtered),
            voxel_points=len(voxel),
            raw_cluster_count=len(clusters),
            cluster_count=len(infos),
            size_rejected=size_rejected,
            shape_rejected=shape_rejected,
            static_rejected=static_rejected,
            selected=selected,
            published_pose=published_pose,
            confidence=confidence,
            frame_id=working_frame,
            selection_reason=selection_reason,
            filter_stats=filter_stats,
        )


def main():
    rospy.init_node("target_lidar_detector_node")
    TargetLidarDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
