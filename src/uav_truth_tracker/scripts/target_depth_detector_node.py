#!/usr/bin/env python3
"""Detect the target UAV from a forward depth camera observation stream."""

import math
from collections import defaultdict, deque

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Float32, Header
from visualization_msgs.msg import Marker, MarkerArray


class TargetDepthDetector:
    def __init__(self):
        self.detector_input_mode = rospy.get_param(
            "~detector_input_mode", "depth_points"
        ).lower()
        if self.detector_input_mode not in ("depth_points", "depth_image"):
            rospy.logwarn(
                "[TargetDepthDetector] invalid detector_input_mode=%s, using depth_points",
                self.detector_input_mode,
            )
            self.detector_input_mode = "depth_points"

        self.fixed_frame = rospy.get_param("~fixed_frame", "map")
        self.depth_points_topic = rospy.get_param(
            "~depth_points_topic", "/iris0/camera/depth/points"
        )
        self.depth_image_topic = rospy.get_param(
            "~depth_image_topic", "/iris0/camera/depth/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/iris0/camera/depth/camera_info"
        )
        self.uav0_odom_topic = rospy.get_param(
            "~uav0_odom_topic", "/uav0/mavros/local_position/odom"
        )
        self.estimator_state_topic = rospy.get_param(
            "~estimator_state_topic", "/target_estimator/state"
        )

        self.output_pose_topic = rospy.get_param(
            "~output_pose_topic", "/target_visual/pose"
        )
        self.pose_cov_topic = rospy.get_param(
            "~output_pose_cov_topic",
            rospy.get_param("~pose_cov_topic", "/target_visual/pose_cov"),
        )
        self.confidence_topic = rospy.get_param(
            "~output_confidence_topic",
            rospy.get_param("~confidence_topic", "/target_visual/confidence"),
        )
        self.valid_topic = rospy.get_param(
            "~output_valid_topic", rospy.get_param("~valid_topic", "/target_visual/valid")
        )
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/target_visual/marker"
        )
        self.object_cloud_topic = rospy.get_param(
            "~object_cloud_topic", "/target_visual/object_cloud"
        )
        self.source_debug_image_topic = rospy.get_param(
            "~source_debug_image_topic", "/target_visual/source_debug_image"
        )

        self.depth_min = max(rospy.get_param("~depth_min", 0.5), 0.0)
        self.depth_max = max(rospy.get_param("~depth_max", 20.0), self.depth_min)
        self.z_min = rospy.get_param("~z_min", 1.0)
        self.z_max = rospy.get_param("~z_max", 8.0)
        self.cluster_tolerance = max(rospy.get_param("~cluster_tolerance", 0.20), 0.01)
        self.min_cluster_size = max(int(rospy.get_param("~min_cluster_size", 10)), 1)
        self.max_cluster_size = max(
            int(rospy.get_param("~max_cluster_size", 50000)), self.min_cluster_size
        )
        self.target_size_min = max(rospy.get_param("~target_size_min", 0.10), 0.0)
        self.target_size_max = max(
            rospy.get_param("~target_size_max", 2.0), self.target_size_min
        )
        self.use_estimator_roi = rospy.get_param("~use_estimator_roi", False)
        self.roi_radius = max(rospy.get_param("~roi_radius", 2.0), 0.0)
        self.max_jump_distance = max(rospy.get_param("~max_jump_distance", 3.0), 0.0)
        self.continuity_weight = max(rospy.get_param("~continuity_weight", 1.0), 0.0)
        self.publish_object_cloud = rospy.get_param("~publish_object_cloud", True)
        self.max_sample_points = max(int(rospy.get_param("~max_sample_points", 25000)), 100)
        self.voxel_leaf_size = max(rospy.get_param("~voxel_leaf_size", 0.04), 0.0)
        self.process_rate = max(rospy.get_param("~process_rate", 10.0), 0.1)
        self.min_publish_confidence = self._clamp(
            rospy.get_param("~min_publish_confidence", 0.45), 0.0, 1.0
        )
        self.low_confidence_covariance = max(
            rospy.get_param("~low_confidence_covariance", 0.08), 1e-6
        )
        self.high_confidence_covariance = max(
            rospy.get_param("~high_confidence_covariance", 0.02), 1e-6
        )
        self.invalid_publish_period = max(
            rospy.get_param("~invalid_publish_period", 0.2), 0.05
        )
        self.tf_lookup_timeout = max(rospy.get_param("~tf_lookup_timeout", 0.05), 0.0)
        self.allow_latest_tf_fallback = rospy.get_param(
            "~allow_latest_tf_fallback", True
        )
        self.center_correction_enable = rospy.get_param(
            "~center_correction_enable", True
        )
        self.target_center_depth_offset = max(
            rospy.get_param("~target_center_depth_offset", 0.25), 0.0
        )
        self.continuity_reject_enable = rospy.get_param(
            "~continuity_reject_enable", True
        )
        self.continuity_reject_max_age = max(
            rospy.get_param("~continuity_reject_max_age", 1.0), 0.0
        )

        self.tf_listener = tf.TransformListener()
        self.uav0_pos = None
        self.estimator_pos = None
        self.camera_info = None
        self.last_sensor_pos = None
        self.last_detection = None
        self.last_detection_stamp = None
        self.last_process_time = rospy.Time(0)
        self.last_invalid_pub = rospy.Time(0)
        self._was_valid = False
        self.cv_bridge = None
        self.cv2 = None
        self.np = None

        self.pose_pub = rospy.Publisher(self.output_pose_topic, PoseStamped, queue_size=5)
        self.pose_cov_pub = rospy.Publisher(
            self.pose_cov_topic, PoseWithCovarianceStamped, queue_size=5
        )
        self.confidence_pub = rospy.Publisher(
            self.confidence_topic, Float32, queue_size=5
        )
        self.valid_pub = rospy.Publisher(self.valid_topic, Bool, queue_size=5)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=5)
        self.object_cloud_pub = rospy.Publisher(
            self.object_cloud_topic, PointCloud2, queue_size=2
        )
        self.debug_image_pub = rospy.Publisher(
            self.source_debug_image_topic, Image, queue_size=2
        )

        rospy.Subscriber(self.uav0_odom_topic, Odometry, self.uav0_odom_cb, queue_size=20)
        rospy.Subscriber(
            self.estimator_state_topic, Odometry, self.estimator_state_cb, queue_size=10
        )
        if self.detector_input_mode == "depth_points":
            rospy.Subscriber(
                self.depth_points_topic, PointCloud2, self.depth_points_cb, queue_size=1
            )
        else:
            self._load_cv_tools()
            rospy.Subscriber(
                self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1
            )
            rospy.Subscriber(
                self.depth_image_topic, Image, self.depth_image_cb, queue_size=1
            )

        rospy.loginfo(
            "[TargetDepthDetector] mode=%s fixed_frame=%s depth_points=%s depth_image=%s "
            "camera_info=%s output=%s depth=[%.2f %.2f] z=[%.2f %.2f] "
            "cluster_tol=%.2f size=[%d %d] target_extent=[%.2f %.2f] roi=%s/%.2f "
            "max_jump=%.2f publish_cloud=%s max_sample_points=%d "
            "tf_timeout=%.3f latest_tf_fallback=%s center_offset=%.2f continuity_reject=%s",
            self.detector_input_mode,
            self.fixed_frame,
            self.depth_points_topic,
            self.depth_image_topic,
            self.camera_info_topic,
            self.output_pose_topic,
            self.depth_min,
            self.depth_max,
            self.z_min,
            self.z_max,
            self.cluster_tolerance,
            self.min_cluster_size,
            self.max_cluster_size,
            self.target_size_min,
            self.target_size_max,
            self.use_estimator_roi,
            self.roi_radius,
            self.max_jump_distance,
            self.publish_object_cloud,
            self.max_sample_points,
            self.tf_lookup_timeout,
            self.allow_latest_tf_fallback,
            self.target_center_depth_offset if self.center_correction_enable else 0.0,
            self.continuity_reject_enable,
        )
        rospy.logwarn(
            "[TargetDepthDetector] online detector does not subscribe to /uav1 odom; "
            "/uav1/mavros/local_position/odom is reserved for evaluator/debug only."
        )

    @staticmethod
    def _clamp(value, low, high):
        return min(max(value, low), high)

    @staticmethod
    def _finite_point(point):
        return all(math.isfinite(v) for v in point)

    @staticmethod
    def _dist(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _norm3(x, y, z):
        return math.sqrt(x * x + y * y + z * z)

    def uav0_odom_cb(self, msg):
        p = msg.pose.pose.position
        self.uav0_pos = (p.x, p.y, p.z)

    def estimator_state_cb(self, msg):
        p = msg.pose.pose.position
        self.estimator_pos = (p.x, p.y, p.z)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def _should_process(self, stamp):
        if self.process_rate <= 0.0:
            return True
        now = stamp if stamp != rospy.Time() else rospy.Time.now()
        min_dt = 1.0 / self.process_rate
        if (now - self.last_process_time).to_sec() < min_dt:
            return False
        self.last_process_time = now
        return True

    def _sample_cloud_points(self, cloud):
        total = max(int(cloud.width) * max(int(cloud.height), 1), 1)
        if cloud.height > 1 and cloud.width > 1 and total > self.max_sample_points:
            step = max(int(math.ceil(math.sqrt(float(total) / self.max_sample_points))), 1)
            uvs = [
                (u, v)
                for v in range(0, cloud.height, step)
                for u in range(0, cloud.width, step)
            ]
            iterator = pc2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=True, uvs=uvs
            )
            return [tuple(p[:3]) for p in iterator if self._finite_point(p[:3])]

        stride = max(int(math.ceil(float(total) / self.max_sample_points)), 1)
        out = []
        for idx, p in enumerate(
            pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if idx % stride != 0:
                continue
            point = tuple(p[:3])
            if self._finite_point(point):
                out.append(point)
        return out

    def _lookup_transform(self, source_frame, stamp):
        source_frame = source_frame.lstrip("/")
        target_frame = self.fixed_frame.lstrip("/")
        if not source_frame or source_frame == target_frame:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        when = stamp if stamp != rospy.Time() else rospy.Time(0)
        try:
            self.tf_listener.waitForTransform(
                target_frame,
                source_frame,
                when,
                rospy.Duration(self.tf_lookup_timeout),
            )
            return self.tf_listener.lookupTransform(target_frame, source_frame, when)
        except Exception as exc:
            if not self.allow_latest_tf_fallback or when == rospy.Time(0):
                raise
            try:
                self.tf_listener.waitForTransform(
                    target_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_lookup_timeout),
                )
                rospy.logwarn_throttle(
                    2.0,
                    "[TargetDepthDetector] TF %s -> %s unavailable at cloud stamp %.3f: %s; using latest TF fallback",
                    source_frame,
                    self.fixed_frame,
                    when.to_sec(),
                    exc,
                )
                return self.tf_listener.lookupTransform(
                    target_frame, source_frame, rospy.Time(0)
                )
            except Exception:
                raise exc

    def _transform_points(self, points, source_frame, stamp):
        try:
            trans, quat = self._lookup_transform(source_frame, stamp)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "[TargetDepthDetector] TF %s -> %s unavailable: %s; publishing invalid visual observation",
                source_frame,
                self.fixed_frame,
                exc,
            )
            return None
        self.last_sensor_pos = (trans[0], trans[1], trans[2])
        matrix = tf.transformations.quaternion_matrix(quat)
        transformed = []
        for x, y, z in points:
            tx = trans[0] + matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z
            ty = trans[1] + matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z
            tz = trans[2] + matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z
            transformed.append((tx, ty, tz))
        return transformed

    def _correct_visual_centroid(self, centroid):
        """Push centroid away from camera in XY only (not Z).

        The depth camera sees the near surface of the target, not the centre.
        We push the centroid 'target_center_depth_offset' metres further away
        in the *horizontal* plane so the position better represents the
        target's centre of mass.  We deliberately do NOT push Z because
        when the camera is below the target the old code systematically
        inflated the estimated altitude, causing UAV0 to fly upward.
        """
        if (
            not self.center_correction_enable
            or self.target_center_depth_offset <= 0.0
        ):
            return centroid
        origin = self.last_sensor_pos or self.uav0_pos
        if origin is None:
            return centroid
        vx = centroid[0] - origin[0]
        vy = centroid[1] - origin[1]
        norm_xy = math.sqrt(vx * vx + vy * vy)
        if norm_xy < 1e-6:
            return centroid
        scale = self.target_center_depth_offset / norm_xy
        return (
            centroid[0] + vx * scale,
            centroid[1] + vy * scale,
            centroid[2],  # Z unchanged
        )

    def _voxel_downsample(self, points):
        if self.voxel_leaf_size <= 0.0 or not points:
            return points
        leaf = self.voxel_leaf_size
        buckets = {}
        counts = defaultdict(int)
        for point in points:
            key = (
                int(math.floor(point[0] / leaf)),
                int(math.floor(point[1] / leaf)),
                int(math.floor(point[2] / leaf)),
            )
            if key not in buckets:
                buckets[key] = [0.0, 0.0, 0.0]
            buckets[key][0] += point[0]
            buckets[key][1] += point[1]
            buckets[key][2] += point[2]
            counts[key] += 1
        return [
            (
                sums[0] / counts[key],
                sums[1] / counts[key],
                sums[2] / counts[key],
            )
            for key, sums in buckets.items()
        ]

    def _filter_depth_points(self, raw_points, map_points):
        filtered = []
        for raw, mapped in zip(raw_points, map_points):
            if not self._finite_point(raw) or not self._finite_point(mapped):
                continue
            depth = self._norm3(raw[0], raw[1], raw[2])
            if depth < self.depth_min or depth > self.depth_max:
                continue
            if mapped[2] < self.z_min or mapped[2] > self.z_max:
                continue
            if (
                self.use_estimator_roi
                and self.estimator_pos is not None
                and self.roi_radius > 0.0
                and self._dist(mapped, self.estimator_pos) > self.roi_radius
            ):
                continue
            filtered.append(mapped)
        return filtered

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
        offsets = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
        ]
        for start in range(len(points)):
            if visited[start]:
                continue
            visited[start] = True
            queue = deque([start])
            cluster = []
            while queue:
                idx = queue.popleft()
                cluster.append(points[idx])
                point = points[idx]
                key = (
                    int(math.floor(point[0] / cell)),
                    int(math.floor(point[1] / cell)),
                    int(math.floor(point[2] / cell)),
                )
                for ox, oy, oz in offsets:
                    for other in grid.get((key[0] + ox, key[1] + oy, key[2] + oz), []):
                        if visited[other]:
                            continue
                        q = points[other]
                        dx = point[0] - q[0]
                        dy = point[1] - q[1]
                        dz = point[2] - q[2]
                        if dx * dx + dy * dy + dz * dz <= tol2:
                            visited[other] = True
                            queue.append(other)
            clusters.append(cluster)
        return clusters

    def _cluster_info(self, cluster):
        xs = [point[0] for point in cluster]
        ys = [point[1] for point in cluster]
        zs = [point[2] for point in cluster]
        centroid = (
            sum(xs) / len(xs),
            sum(ys) / len(ys),
            sum(zs) / len(zs),
        )
        size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        max_extent = max(size)
        min_extent = min(size)
        bbox_diag = math.sqrt(size[0] * size[0] + size[1] * size[1] + size[2] * size[2])
        continuity = 0.0
        if self.last_detection is not None:
            continuity = max(
                0.0,
                1.0 - self._dist(centroid, self.last_detection) / max(self.max_jump_distance, 1e-6),
            )
        return {
            "cluster": cluster,
            "centroid": centroid,
            "size": size,
            "max_extent": max_extent,
            "min_extent": min_extent,
            "bbox_diag": bbox_diag,
            "count": len(cluster),
            "continuity": continuity,
        }

    def _score_cluster(self, info):
        count = info["count"]
        max_extent = info["max_extent"]
        if count < self.min_cluster_size or count > self.max_cluster_size:
            return None
        if max_extent < self.target_size_min or max_extent > self.target_size_max:
            return None
        if (
            self.last_detection is not None
            and self.max_jump_distance > 0.0
            and self._dist(info["centroid"], self.last_detection)
            > self.max_jump_distance
            and self.estimator_pos is not None
        ):
            return None
        if (
            self.continuity_reject_enable
            and self.last_detection is not None
            and self.last_detection_stamp is not None
            and self.max_jump_distance > 0.0
        ):
            age = (rospy.Time.now() - self.last_detection_stamp).to_sec()
            if (
                age <= self.continuity_reject_max_age
                and self._dist(info["centroid"], self.last_detection)
                > self.max_jump_distance
            ):
                return None

        ideal_extent = 0.45 * (self.target_size_min + self.target_size_max)
        size_error = abs(max_extent - ideal_extent) / max(self.target_size_max, 1e-6)
        count_score = min(float(count) / max(self.min_cluster_size * 8.0, 1.0), 1.0)
        continuity = info["continuity"] if self.last_detection is not None else 0.5
        compactness = max(0.0, 1.0 - size_error)
        score = (
            1.0 * count_score
            + 1.0 * compactness
            + self.continuity_weight * continuity
        )
        return score

    def _select_cluster(self, clusters):
        best = None
        best_score = -float("inf")
        infos = []
        for cluster in clusters:
            info = self._cluster_info(cluster)
            score = self._score_cluster(info)
            info["score"] = score if score is not None else float("nan")
            infos.append(info)
            if score is not None and score > best_score:
                best = info
                best_score = score
        return best, infos

    def _confidence_for_cluster(self, info):
        count_conf = min(float(info["count"]) / max(self.min_cluster_size * 8.0, 1.0), 1.0)
        extent = info["max_extent"]
        size_conf = 1.0
        if extent < self.target_size_min:
            size_conf = extent / max(self.target_size_min, 1e-6)
        elif extent > self.target_size_max:
            size_conf = self.target_size_max / max(extent, 1e-6)
        continuity = info["continuity"] if self.last_detection is not None else 0.7
        confidence = 0.15 + 0.45 * count_conf + 0.25 * size_conf + 0.15 * continuity
        return self._clamp(confidence, 0.0, 1.0)

    def _publish_valid(self, stamp, info, confidence):
        centroid = self._correct_visual_centroid(info["centroid"])
        publish_info = dict(info)
        publish_info["centroid"] = centroid
        header = Header(stamp=stamp, frame_id=self.fixed_frame)

        # Publish valid + confidence BEFORE the pose so that subscribers
        # (fusion, estimator) see the updated flags when they process the
        # pose callback.  Reversing this order causes a one-message race:
        # the pose arrives while valid is still the stale False from the
        # previous cycle, and the observation is silently rejected.
        self.confidence_pub.publish(Float32(data=confidence))
        self.valid_pub.publish(Bool(data=confidence >= self.min_publish_confidence))

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = centroid[0]
        pose.pose.position.y = centroid[1]
        pose.pose.position.z = centroid[2]
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        cov = self.high_confidence_covariance + (
            1.0 - confidence
        ) * (self.low_confidence_covariance - self.high_confidence_covariance)
        pose_cov = PoseWithCovarianceStamped()
        pose_cov.header = header
        pose_cov.pose.pose = pose.pose
        pose_cov.pose.covariance[0] = cov
        pose_cov.pose.covariance[7] = cov
        pose_cov.pose.covariance[14] = cov * 1.2
        pose_cov.pose.covariance[21] = 1.0
        pose_cov.pose.covariance[28] = 1.0
        pose_cov.pose.covariance[35] = 1.0
        self.pose_cov_pub.publish(pose_cov)
        self.marker_pub.publish(self._build_markers(header, publish_info, confidence, True))

        if self.publish_object_cloud:
            self.object_cloud_pub.publish(pc2.create_cloud_xyz32(header, info["cluster"]))

        self.last_detection = centroid
        self.last_detection_stamp = stamp
        self._was_valid = True

    def _publish_invalid(self, stamp, reason):
        now = rospy.Time.now()
        # Always publish valid=false immediately on transition from valid to
        # invalid.  The rate-limit *only* applies to consecutive invalid
        # publications — otherwise the fusion node and estimator will keep
        # consuming a stale valid=true for up to invalid_publish_period
        # seconds after the target has actually left the field of view.
        if self._was_valid:
            pass  # force immediate publication
        elif (now - self.last_invalid_pub).to_sec() < self.invalid_publish_period:
            return
        self.last_invalid_pub = now
        self._was_valid = False
        header = Header(stamp=stamp if stamp != rospy.Time() else now, frame_id=self.fixed_frame)
        self.confidence_pub.publish(Float32(data=0.0))
        self.valid_pub.publish(Bool(data=False))
        self.marker_pub.publish(self._build_invalid_marker(header, reason))
        if self.publish_object_cloud:
            self.object_cloud_pub.publish(pc2.create_cloud_xyz32(header, []))

    def _build_markers(self, header, info, confidence, valid):
        markers = MarkerArray()
        centroid = info["centroid"]
        size = info["size"]

        cube = Marker()
        cube.header = header
        cube.ns = "target_visual_bbox"
        cube.id = 0
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose.position.x = centroid[0]
        cube.pose.position.y = centroid[1]
        cube.pose.position.z = centroid[2]
        cube.pose.orientation.w = 1.0
        cube.scale.x = max(size[0], 0.05)
        cube.scale.y = max(size[1], 0.05)
        cube.scale.z = max(size[2], 0.05)
        cube.color.r = 0.0 if valid else 1.0
        cube.color.g = 0.9 if valid else 0.2
        cube.color.b = 0.3
        cube.color.a = 0.35
        cube.lifetime = rospy.Duration(0.5)
        markers.markers.append(cube)

        sphere = Marker()
        sphere.header = header
        sphere.ns = "target_visual_centroid"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = centroid[0]
        sphere.pose.position.y = centroid[1]
        sphere.pose.position.z = centroid[2]
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.18
        sphere.scale.y = 0.18
        sphere.scale.z = 0.18
        sphere.color.r = 0.0
        sphere.color.g = 1.0
        sphere.color.b = 0.2
        sphere.color.a = 0.9
        sphere.lifetime = rospy.Duration(0.5)
        markers.markers.append(sphere)

        text = Marker()
        text.header = header
        text.ns = "target_visual_text"
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = centroid[0]
        text.pose.position.y = centroid[1]
        text.pose.position.z = centroid[2] + 0.45
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.text = "visual conf={:.2f} pts={} extent={:.2f}".format(
            confidence, info["count"], info["max_extent"]
        )
        text.lifetime = rospy.Duration(0.5)
        markers.markers.append(text)
        return markers

    def _build_invalid_marker(self, header, reason):
        marker = Marker()
        marker.header = header
        marker.ns = "target_visual_text"
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = self.z_min + 0.5
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 0.6
        marker.color.b = 0.0
        marker.color.a = 0.85
        marker.text = "visual invalid: {}".format(reason)
        marker.lifetime = rospy.Duration(0.5)
        delete_bbox = Marker()
        delete_bbox.header = header
        delete_bbox.ns = "target_visual_bbox"
        delete_bbox.id = 0
        delete_bbox.action = Marker.DELETE
        delete_centroid = Marker()
        delete_centroid.header = header
        delete_centroid.ns = "target_visual_centroid"
        delete_centroid.id = 1
        delete_centroid.action = Marker.DELETE
        return MarkerArray(markers=[delete_bbox, delete_centroid, marker])

    def depth_points_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if not self._should_process(stamp):
            return
        raw_points = self._sample_cloud_points(msg)
        if not raw_points:
            self._publish_invalid(stamp, "empty cloud")
            return
        map_points = self._transform_points(raw_points, msg.header.frame_id, stamp)
        if map_points is None:
            self._publish_invalid(stamp, "missing tf")
            return
        filtered = self._filter_depth_points(raw_points, map_points)
        filtered = self._voxel_downsample(filtered)
        if len(filtered) < self.min_cluster_size:
            rospy.loginfo_throttle(
                1.0,
                "[TargetDepthDetectorDiag] raw_points=%d filtered_points=%d cluster_count=0 selected=False reason=not_enough_filtered",
                len(raw_points),
                len(filtered),
            )
            self._publish_invalid(stamp, "not enough filtered points")
            return
        clusters = self._cluster_points(filtered)
        selected, _infos = self._select_cluster(clusters)
        if selected is None:
            rospy.loginfo_throttle(
                1.0,
                "[TargetDepthDetectorDiag] raw_points=%d filtered_points=%d cluster_count=%d selected=False reason=no_target_sized_cluster",
                len(raw_points),
                len(filtered),
                len(clusters),
            )
            self._publish_invalid(stamp, "no target-sized cluster")
            return
        confidence = self._confidence_for_cluster(selected)
        corrected = self._correct_visual_centroid(selected["centroid"])
        rospy.loginfo_throttle(
            1.0,
            "[TargetDepthDetectorDiag] raw_points=%d filtered_points=%d cluster_count=%d selected=True selected_points=%d raw_centroid=(%.2f %.2f %.2f) corrected_centroid=(%.2f %.2f %.2f) confidence=%.2f",
            len(raw_points),
            len(filtered),
            len(clusters),
            selected["count"],
            selected["centroid"][0],
            selected["centroid"][1],
            selected["centroid"][2],
            corrected[0],
            corrected[1],
            corrected[2],
            confidence,
        )
        self._publish_valid(stamp, selected, confidence)

    def _load_cv_tools(self):
        try:
            from cv_bridge import CvBridge
            import cv2
            import numpy as np

            self.cv_bridge = CvBridge()
            self.cv2 = cv2
            self.np = np
        except Exception as exc:
            rospy.logerr(
                "[TargetDepthDetector] depth_image mode requires cv_bridge, cv2, and numpy: %s",
                exc,
            )

    def _depth_image_to_meters(self, msg):
        if self.cv_bridge is None or self.cv2 is None or self.np is None:
            return None
        try:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "[TargetDepthDetector] failed to convert depth image: %s", exc
            )
            return None
        depth = image.astype("float32")
        if msg.encoding in ("16UC1", "mono16"):
            depth *= 0.001
        return depth

    def _select_blob(self, depth):
        np = self.np
        cv2 = self.cv2
        mask = np.isfinite(depth)
        mask = np.logical_and(mask, depth >= self.depth_min)
        mask = np.logical_and(mask, depth <= self.depth_max)
        mask_u8 = (mask.astype("uint8")) * 255
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_u8, 8)
        best_label = 0
        best_score = -float("inf")
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_cluster_size or area > self.max_cluster_size:
                continue
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            w = int(stats[label, cv2.CC_STAT_WIDTH])
            h = int(stats[label, cv2.CC_STAT_HEIGHT])
            if w <= 0 or h <= 0:
                continue
            blob_depth = depth[labels == label]
            median_depth = float(np.median(blob_depth[np.isfinite(blob_depth)]))
            if not math.isfinite(median_depth):
                continue
            compact = float(area) / float(max(w * h, 1))
            score = min(float(area) / max(self.min_cluster_size * 8.0, 1.0), 1.0)
            score += min(compact, 1.0)
            score += 1.0 / max(median_depth, 1.0)
            if self.last_detection is not None:
                score += self.continuity_weight * 0.2
            if score > best_score:
                best_score = score
                best_label = label
        if best_label == 0:
            return None, labels, mask_u8
        return best_label, labels, mask_u8

    def _transform_pose_to_fixed(self, pose, source_frame, stamp):
        try:
            source_frame = source_frame.lstrip("/")
            trans, quat = self._lookup_transform(source_frame, stamp)
            self.last_sensor_pos = (trans[0], trans[1], trans[2])
            matrix = tf.transformations.quaternion_matrix(quat)
            x, y, z = pose
            tx = trans[0] + matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z
            ty = trans[1] + matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z
            tz = trans[2] + matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z
            return tx, ty, tz
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "[TargetDepthDetector] TF %s -> %s unavailable for depth image: %s",
                source_frame,
                self.fixed_frame,
                exc,
            )
            return None

    def depth_image_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if not self._should_process(stamp):
            return
        if self.camera_info is None:
            self._publish_invalid(stamp, "missing camera_info")
            return
        depth = self._depth_image_to_meters(msg)
        if depth is None:
            self._publish_invalid(stamp, "image conversion failed")
            return
        selected_label, labels, mask_u8 = self._select_blob(depth)
        if selected_label is None:
            self._publish_invalid(stamp, "no depth blob")
            return

        np = self.np
        indices = np.argwhere(labels == selected_label)
        if indices.size == 0:
            self._publish_invalid(stamp, "empty blob")
            return
        v_mean = float(np.mean(indices[:, 0]))
        u_mean = float(np.mean(indices[:, 1]))
        blob_depth = depth[labels == selected_label]
        z = float(np.median(blob_depth[np.isfinite(blob_depth)]))
        fx = float(self.camera_info.K[0])
        fy = float(self.camera_info.K[4])
        cx = float(self.camera_info.K[2])
        cy = float(self.camera_info.K[5])
        if fx <= 1e-6 or fy <= 1e-6 or not math.isfinite(z):
            self._publish_invalid(stamp, "invalid intrinsics/depth")
            return
        x = (u_mean - cx) * z / fx
        y = (v_mean - cy) * z / fy
        mapped = self._transform_pose_to_fixed((x, y, z), msg.header.frame_id, stamp)
        if mapped is None:
            self._publish_invalid(stamp, "missing tf")
            return
        if mapped[2] < self.z_min or mapped[2] > self.z_max:
            self._publish_invalid(stamp, "blob outside z bounds")
            return

        count = int(indices.shape[0])
        info = {
            "cluster": [mapped],
            "centroid": mapped,
            "size": (0.2, 0.2, 0.2),
            "max_extent": 0.2,
            "count": count,
            "continuity": 0.5,
        }
        confidence = self._clamp(0.35 + min(count / 800.0, 0.5), 0.0, 0.9)
        self._publish_valid(stamp, info, confidence)

        if self.cv_bridge is not None:
            try:
                debug = self.cv2.applyColorMap(mask_u8, self.cv2.COLORMAP_TURBO)
                self.debug_image_pub.publish(
                    self.cv_bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                )
            except Exception:
                pass


def main():
    rospy.init_node("target_depth_detector_node")
    TargetDepthDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
