#!/usr/bin/env python3
"""Priority fusion for visual target observations with LiDAR fallback."""

import math

import rospy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, Header, String
from visualization_msgs.msg import Marker, MarkerArray


class Observation:
    def __init__(self):
        self.pose = None
        self.pose_cov = None
        self.confidence = 0.0
        self.valid = False
        self.pose_stamp = None
        self.valid_stamp = None
        self.confidence_stamp = None


class TargetSensorFusion:
    def __init__(self):
        self.visual_pose_topic = rospy.get_param(
            "~visual_pose_topic", "/target_visual/pose"
        )
        self.visual_pose_cov_topic = rospy.get_param(
            "~visual_pose_cov_topic", "/target_visual/pose_cov"
        )
        self.visual_confidence_topic = rospy.get_param(
            "~visual_confidence_topic", "/target_visual/confidence"
        )
        self.visual_valid_topic = rospy.get_param(
            "~visual_valid_topic", "/target_visual/valid"
        )
        self.lidar_pose_topic = rospy.get_param(
            "~lidar_pose_topic", "/target_observation/pose"
        )
        self.lidar_pose_cov_topic = rospy.get_param(
            "~lidar_pose_cov_topic", "/target_observation/pose_cov"
        )
        self.lidar_confidence_topic = rospy.get_param(
            "~lidar_confidence_topic", "/target_observation/confidence"
        )
        self.lidar_valid_topic = rospy.get_param(
            "~lidar_valid_topic", "/target_observation/valid"
        )
        self.output_pose_topic = rospy.get_param(
            "~output_pose_topic", "/target_fused/pose"
        )
        self.output_pose_cov_topic = rospy.get_param(
            "~output_pose_cov_topic", "/target_fused/pose_cov"
        )
        self.output_confidence_topic = rospy.get_param(
            "~output_confidence_topic", "/target_fused/confidence"
        )
        self.output_valid_topic = rospy.get_param(
            "~output_valid_topic", "/target_fused/valid"
        )
        self.output_source_topic = rospy.get_param(
            "~output_source_topic", "/target_fused/source"
        )
        self.output_marker_topic = rospy.get_param(
            "~output_marker_topic", "/target_fused/marker"
        )
        self.output_object_cloud_topic = rospy.get_param(
            "~output_object_cloud_topic", "/target_fused/object_cloud"
        )

        self.visual_min_confidence = self._clamp(
            rospy.get_param("~visual_min_confidence", 0.45), 0.0, 1.0
        )
        self.lidar_min_confidence = self._clamp(
            rospy.get_param("~lidar_min_confidence", 0.25), 0.0, 1.0
        )
        self.max_age = max(
            rospy.get_param("~max_observation_age", rospy.get_param("~max_age", 0.4)),
            0.0,
        )
        self.consistency_threshold = max(
            rospy.get_param("~consistency_threshold", 1.5), 0.0
        )
        self.visual_high_confidence = self._clamp(
            rospy.get_param("~visual_high_confidence", 0.60), 0.0, 1.0
        )
        self.fusion_mode = rospy.get_param("~fusion_mode", "priority").lower()
        if self.fusion_mode not in ("priority", "weighted"):
            rospy.logwarn(
                "[TargetSensorFusion] invalid fusion_mode=%s, using priority",
                self.fusion_mode,
            )
            self.fusion_mode = "priority"
        self.prefer_visual = rospy.get_param("~prefer_visual", True)
        self.require_consistency_for_visual = rospy.get_param(
            "~require_consistency_for_visual", True
        )
        self.visual_override_margin = self._clamp(
            rospy.get_param("~visual_override_margin", 0.35), 0.0, 1.0
        )
        self.continuity_selection_enable = rospy.get_param(
            "~continuity_selection_enable", True
        )
        self.continuity_selection_max_age = max(
            rospy.get_param("~continuity_selection_max_age", 2.0), 0.0
        )
        self.continuity_selection_margin = max(
            rospy.get_param("~continuity_selection_margin", 0.40), 0.0
        )
        self.publish_rate = max(rospy.get_param("~publish_rate", 20.0), 1.0)
        self.default_visual_covariance = max(
            rospy.get_param("~default_visual_covariance", 0.04), 1e-6
        )
        self.default_lidar_covariance = max(
            rospy.get_param("~default_lidar_covariance", 0.20), 1e-6
        )
        self.output_smoothing_alpha = self._clamp(
            rospy.get_param("~output_smoothing_alpha", 0.35), 0.0, 1.0
        )
        self.output_smoothing_max_jump = max(
            rospy.get_param("~output_smoothing_max_jump", 1.5), 0.0
        )
        self.reject_large_jumps = rospy.get_param("~reject_large_jumps", True)
        self.max_jump_reference_age = max(
            rospy.get_param("~max_jump_reference_age", 15.0), 0.0
        )
        self.visual_max_jump_distance = max(
            rospy.get_param("~visual_max_jump_distance", 6.0), 0.0
        )
        self.lidar_max_jump_distance = max(
            rospy.get_param("~lidar_max_jump_distance", 8.0), 0.0
        )
        self.lidar_fallback_visual_gap = max(
            rospy.get_param("~lidar_fallback_visual_gap", 0.45), 0.0
        )
        self.lidar_requires_recent_visual = rospy.get_param(
            "~lidar_requires_recent_visual", False
        )
        self.lidar_recent_visual_max_age = max(
            rospy.get_param("~lidar_recent_visual_max_age", 1.5), 0.0
        )
        self.lidar_recent_visual_gate_radius = max(
            rospy.get_param("~lidar_recent_visual_gate_radius", 2.0), 0.0
        )
        self.hold_last_on_none = rospy.get_param("~hold_last_on_none", True)
        self.hold_last_max_age = max(
            rospy.get_param("~hold_last_max_age", 0.60), 0.0
        )
        self.hold_confidence_scale = self._clamp(
            rospy.get_param("~hold_confidence_scale", 0.65), 0.0, 1.0
        )
        self.clamp_output_motion = rospy.get_param("~clamp_output_motion", True)
        self.max_output_speed_xy = max(
            rospy.get_param("~max_output_speed_xy", 2.5), 0.01
        )
        self.max_output_speed_z = max(
            rospy.get_param("~max_output_speed_z", 0.8), 0.01
        )
        self.min_output_step_xy = max(
            rospy.get_param("~min_output_step_xy", 0.08), 0.0
        )
        self.min_output_step_z = max(
            rospy.get_param("~min_output_step_z", 0.04), 0.0
        )
        self.clamp_output_z_bounds = rospy.get_param("~clamp_output_z_bounds", False)
        self.output_z_min = rospy.get_param("~output_z_min", 0.5)
        self.output_z_max = rospy.get_param("~output_z_max", 6.0)
        if self.output_z_max < self.output_z_min:
            self.output_z_min, self.output_z_max = self.output_z_max, self.output_z_min
        self.visual_z_filter_enable = rospy.get_param(
            "~visual_z_filter_enable", False
        )
        self.visual_z_min = rospy.get_param("~visual_z_min", self.output_z_min)
        self.visual_z_max = rospy.get_param("~visual_z_max", self.output_z_max)
        if self.visual_z_max < self.visual_z_min:
            self.visual_z_min, self.visual_z_max = self.visual_z_max, self.visual_z_min
        self.lidar_z_filter_enable = rospy.get_param("~lidar_z_filter_enable", False)
        self.lidar_z_min = rospy.get_param("~lidar_z_min", self.output_z_min)
        self.lidar_z_max = rospy.get_param("~lidar_z_max", self.output_z_max)
        if self.lidar_z_max < self.lidar_z_min:
            self.lidar_z_min, self.lidar_z_max = self.lidar_z_max, self.lidar_z_min

        self.visual = Observation()
        self.lidar = Observation()
        self.last_output_pose = None
        self.last_output_stamp = None
        self.last_output_source = "none"
        self.last_valid_output_pose = None
        self.last_valid_output_stamp = None
        self.last_valid_output_source = "none"
        self.last_valid_output_confidence = 0.0

        self.pose_pub = rospy.Publisher(self.output_pose_topic, PoseStamped, queue_size=5)
        self.pose_cov_pub = rospy.Publisher(
            self.output_pose_cov_topic, PoseWithCovarianceStamped, queue_size=5
        )
        self.confidence_pub = rospy.Publisher(
            self.output_confidence_topic, Float32, queue_size=5
        )
        self.valid_pub = rospy.Publisher(self.output_valid_topic, Bool, queue_size=5)
        self.source_pub = rospy.Publisher(self.output_source_topic, String, queue_size=5)
        self.marker_pub = rospy.Publisher(
            self.output_marker_topic, MarkerArray, queue_size=5
        )
        self.object_cloud_pub = rospy.Publisher(
            self.output_object_cloud_topic, PointCloud2, queue_size=2
        )

        rospy.Subscriber(
            self.visual_pose_topic,
            PoseStamped,
            lambda msg: self.pose_cb(msg, self.visual),
            queue_size=10,
        )
        rospy.Subscriber(
            self.visual_pose_cov_topic,
            PoseWithCovarianceStamped,
            lambda msg: self.pose_cov_cb(msg, self.visual),
            queue_size=10,
        )
        rospy.Subscriber(
            self.visual_confidence_topic,
            Float32,
            lambda msg: self.confidence_cb(msg, self.visual),
            queue_size=10,
        )
        rospy.Subscriber(
            self.visual_valid_topic,
            Bool,
            lambda msg: self.valid_cb(msg, self.visual),
            queue_size=10,
        )
        rospy.Subscriber(
            self.lidar_pose_topic,
            PoseStamped,
            lambda msg: self.pose_cb(msg, self.lidar),
            queue_size=10,
        )
        rospy.Subscriber(
            self.lidar_pose_cov_topic,
            PoseWithCovarianceStamped,
            lambda msg: self.pose_cov_cb(msg, self.lidar),
            queue_size=10,
        )
        rospy.Subscriber(
            self.lidar_confidence_topic,
            Float32,
            lambda msg: self.confidence_cb(msg, self.lidar),
            queue_size=10,
        )
        rospy.Subscriber(
            self.lidar_valid_topic,
            Bool,
            lambda msg: self.valid_cb(msg, self.lidar),
            queue_size=10,
        )

        rospy.loginfo(
            "[TargetSensorFusion] visual=(%s %s %s %s) lidar=(%s %s %s %s) output=%s "
            "mode=%s prefer_visual=%s min_conf=(visual %.2f lidar %.2f) "
            "visual_high=%.2f max_age=%.2f consistency=%.2f "
            "require_consistency=%s override_margin=%.2f smoothing_alpha=%.2f "
            "continuity_selection=%s/%.2fs margin=%.2f "
            "reject_large_jumps=%s jump_ref_age=%.2f visual_jump=%.2f lidar_jump=%.2f "
            "lidar_fallback_visual_gap=%.2f lidar_requires_visual=%s/%.2fs/%.2fm "
            "hold_last=%s/%.2f clamp_motion=%s speed_xy=%.2f speed_z=%.2f z_bounds=%s[%.2f %.2f] "
            "source_z_filter visual=%s[%.2f %.2f] lidar=%s[%.2f %.2f]",
            self.visual_pose_topic,
            self.visual_pose_cov_topic,
            self.visual_confidence_topic,
            self.visual_valid_topic,
            self.lidar_pose_topic,
            self.lidar_pose_cov_topic,
            self.lidar_confidence_topic,
            self.lidar_valid_topic,
            self.output_pose_topic,
            self.fusion_mode,
            self.prefer_visual,
            self.visual_min_confidence,
            self.lidar_min_confidence,
            self.visual_high_confidence,
            self.max_age,
            self.consistency_threshold,
            self.require_consistency_for_visual,
            self.visual_override_margin,
            self.output_smoothing_alpha,
            self.continuity_selection_enable,
            self.continuity_selection_max_age,
            self.continuity_selection_margin,
            self.reject_large_jumps,
            self.max_jump_reference_age,
            self.visual_max_jump_distance,
            self.lidar_max_jump_distance,
            self.lidar_fallback_visual_gap,
            self.lidar_requires_recent_visual,
            self.lidar_recent_visual_max_age,
            self.lidar_recent_visual_gate_radius,
            self.hold_last_on_none,
            self.hold_last_max_age,
            self.clamp_output_motion,
            self.max_output_speed_xy,
            self.max_output_speed_z,
            self.clamp_output_z_bounds,
            self.output_z_min,
            self.output_z_max,
            self.visual_z_filter_enable,
            self.visual_z_min,
            self.visual_z_max,
            self.lidar_z_filter_enable,
            self.lidar_z_min,
            self.lidar_z_max,
        )
        rospy.logwarn(
            "[TargetSensorFusion] online fusion does not subscribe to /uav1 odom and does not estimate target velocity."
        )

    @staticmethod
    def _clamp(value, low, high):
        return min(max(value, low), high)

    @staticmethod
    def _pos(msg):
        p = msg.pose.position
        return p.x, p.y, p.z

    @staticmethod
    def _dist(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def _age(now, stamp):
        if stamp is None:
            return float("nan")
        return (now - stamp).to_sec()

    def pose_cb(self, msg, obs):
        obs.pose = msg
        obs.pose_stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )

    def pose_cov_cb(self, msg, obs):
        obs.pose_cov = msg

    def confidence_cb(self, msg, obs):
        obs.confidence = self._clamp(float(msg.data), 0.0, 1.0)
        obs.confidence_stamp = rospy.Time.now()

    def valid_cb(self, msg, obs):
        obs.valid = bool(msg.data)
        obs.valid_stamp = rospy.Time.now()

    def _z_valid(self, obs, source):
        if obs.pose is None:
            return False
        z = obs.pose.pose.position.z
        if source == "visual":
            enabled = self.visual_z_filter_enable
            z_min = self.visual_z_min
            z_max = self.visual_z_max
        else:
            enabled = self.lidar_z_filter_enable
            z_min = self.lidar_z_min
            z_max = self.lidar_z_max
        if not enabled:
            return True
        if z_min <= z <= z_max:
            return True
        rospy.logwarn_throttle(
            0.5,
            "[TargetSensorFusionReject] source=%s rejected_z=%.2f bounds=[%.2f %.2f]",
            source,
            z,
            z_min,
            z_max,
        )
        return False

    def _fresh(self, obs, now, min_confidence, source):
        if obs.pose is None or not obs.valid:
            return False
        stamp = obs.pose_stamp
        if stamp is None:
            return False
        if self.max_age > 0.0 and (now - stamp).to_sec() > self.max_age:
            return False
        if obs.confidence < min_confidence:
            return False
        if not self._z_valid(obs, source):
            return False
        if source == "lidar" and not self._lidar_has_recent_visual_confirmation(now):
            return False
        return True

    def _lidar_has_recent_visual_confirmation(self, now):
        if not self.lidar_requires_recent_visual:
            return True
        if self.visual.pose is None or self.visual.pose_stamp is None or self.lidar.pose is None:
            return False
        age = (now - self.visual.pose_stamp).to_sec()
        if age < 0.0 or (
            self.lidar_recent_visual_max_age > 0.0
            and age > self.lidar_recent_visual_max_age
        ):
            rospy.loginfo_throttle(
                1.0,
                "[TargetSensorFusionSemanticGate] rejecting_lidar reason=no_recent_visual age=%.2f max_age=%.2f",
                age,
                self.lidar_recent_visual_max_age,
            )
            return False
        distance = self._dist(self._pos(self.lidar.pose), self._pos(self.visual.pose))
        if (
            self.lidar_recent_visual_gate_radius > 0.0
            and distance > self.lidar_recent_visual_gate_radius
        ):
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusionSemanticGate] rejecting_lidar reason=outside_visual_gate distance=%.2f radius=%.2f visual_age=%.2f",
                distance,
                self.lidar_recent_visual_gate_radius,
                age,
            )
            return False
        return True

    def _choose(self, now):
        visual_ok = self._fresh(
            self.visual, now, self.visual_min_confidence, "visual"
        )
        lidar_ok = self._fresh(self.lidar, now, self.lidar_min_confidence, "lidar")
        if not visual_ok and not lidar_ok:
            return None, None, "none", 0.0
        if visual_ok and not lidar_ok:
            return self.visual.pose, self.visual.pose_cov, "visual", self.visual.confidence
        if lidar_ok and not visual_ok:
            if (
                self.prefer_visual
                and self.visual.pose_stamp is not None
                and self.lidar_fallback_visual_gap > 0.0
                and (now - self.visual.pose_stamp).to_sec() <= self.lidar_fallback_visual_gap
            ):
                rospy.loginfo_throttle(
                    0.5,
                    "[TargetSensorFusionHold] visual recently dropped %.2fs ago; holding instead of switching immediately to LiDAR",
                    (now - self.visual.pose_stamp).to_sec(),
                )
                return None, None, "hold_visual_gap", 0.0
            return self.lidar.pose, self.lidar.pose_cov, "lidar", self.lidar.confidence

        visual_pos = self._pos(self.visual.pose)
        lidar_pos = self._pos(self.lidar.pose)
        distance = self._dist(visual_pos, lidar_pos)
        if distance < self.consistency_threshold:
            if self.fusion_mode == "weighted":
                return self._weighted_pose(now), None, "fused", max(
                    self.visual.confidence, self.lidar.confidence
                )
            if self.prefer_visual:
                return self.visual.pose, self.visual.pose_cov, "visual", self.visual.confidence
            return self.lidar.pose, self.lidar.pose_cov, "lidar", self.lidar.confidence

        # --- sensors disagree ---
        if self.lidar_requires_recent_visual:
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusionSemanticGate] sensors_disagree=%.2f choosing semantic visual",
                distance,
            )
            return self.visual.pose, self.visual.pose_cov, "visual", self.visual.confidence

        # Confidence from either geometric detector can be high for a static
        # object. When the sensors disagree, prefer the candidate that remains
        # clearly closer to the most recent valid fused position. This is
        # position association only; the fusion node still does not estimate
        # target velocity.
        continuity_choice = self._choose_by_continuity(now, visual_pos, lidar_pos)
        if continuity_choice == "visual":
            return self.visual.pose, self.visual.pose_cov, "visual", self.visual.confidence
        if continuity_choice == "lidar":
            return self.lidar.pose, self.lidar.pose_cov, "lidar", self.lidar.confidence

        # If the visual target is not consistent with a valid LiDAR target,
        # keep visual primary only for consistent observations. This blocks
        # high-confidence depth false positives from nearby placed objects.
        if self.require_consistency_for_visual:
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusion] visual_lidar_disagree=%.2f visual_conf=%.2f lidar_conf=%.2f; require_consistency_for_visual=true, choosing lidar",
                distance,
                self.visual.confidence,
                self.lidar.confidence,
            )
            return self.lidar.pose, self.lidar.pose_cov, "lidar", self.lidar.confidence

        # When visual is clearly the better source (high confidence or
        # significantly above LiDAR), trust it despite the disagreement.
        if (
            self.prefer_visual
            and self.visual.confidence >= self.visual_high_confidence
        ):
            return self.visual.pose, self.visual.pose_cov, "visual", self.visual.confidence

        if (
            self.prefer_visual
            and self.visual.confidence >= self.lidar.confidence + self.visual_override_margin
        ):
            return self.visual.pose, self.visual.pose_cov, "visual", self.visual.confidence

        # Fallback: pick the higher-confidence sensor rather than rejecting
        # both.  Rejecting both (source=none) starves the estimator and forces
        # it into PREDICT_ONLY/LOST even when one sensor has a usable
        # observation — the disagreement is more likely to be LiDAR noise than
        # a genuine visual failure.
        if self.visual.confidence >= self.lidar.confidence:
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusion] visual_lidar_disagree=%.2f visual_conf=%.2f lidar_conf=%.2f; picking higher-confidence visual",
                distance,
                self.visual.confidence,
                self.lidar.confidence,
            )
            return self.visual.pose, self.visual.pose_cov, "visual", self.visual.confidence
        rospy.logwarn_throttle(
            0.5,
            "[TargetSensorFusion] visual_lidar_disagree=%.2f visual_conf=%.2f lidar_conf=%.2f; picking higher-confidence lidar",
            distance,
            self.visual.confidence,
            self.lidar.confidence,
        )
        return self.lidar.pose, self.lidar.pose_cov, "lidar", self.lidar.confidence

    def _choose_by_continuity(self, now, visual_pos, lidar_pos):
        if (
            not self.continuity_selection_enable
            or self.last_valid_output_pose is None
            or self.last_valid_output_stamp is None
        ):
            return None
        ref_age = (now - self.last_valid_output_stamp).to_sec()
        if (
            ref_age < 0.0
            or (
                self.continuity_selection_max_age > 0.0
                and ref_age > self.continuity_selection_max_age
            )
        ):
            return None

        reference = self._pos(self.last_valid_output_pose)
        visual_distance = self._dist(visual_pos, reference)
        lidar_distance = self._dist(lidar_pos, reference)
        margin = self.continuity_selection_margin
        if visual_distance + margin < lidar_distance:
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusionAssociation] sensors_disagree choosing=visual "
                "visual_to_last=%.2f lidar_to_last=%.2f margin=%.2f ref_age=%.2f",
                visual_distance,
                lidar_distance,
                margin,
                ref_age,
            )
            return "visual"
        if lidar_distance + margin < visual_distance:
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusionAssociation] sensors_disagree choosing=lidar "
                "visual_to_last=%.2f lidar_to_last=%.2f margin=%.2f ref_age=%.2f",
                visual_distance,
                lidar_distance,
                margin,
                ref_age,
            )
            return "lidar"
        return None

    def _weighted_pose(self, now):
        visual_pos = self._pos(self.visual.pose)
        lidar_pos = self._pos(self.lidar.pose)
        wv = max(self.visual.confidence, 1e-3)
        wl = max(self.lidar.confidence, 1e-3)
        denom_xy = wv + wl
        # LiDAR measures Z more accurately than a forward-facing depth camera,
        # so we boost LiDAR's weight on the Z axis by 3x.
        wl_z = wl * 3.0
        denom_z = wv + wl_z
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.visual.pose.header.frame_id or self.lidar.pose.header.frame_id
        pose.pose.position.x = (wv * visual_pos[0] + wl * lidar_pos[0]) / denom_xy
        pose.pose.position.y = (wv * visual_pos[1] + wl * lidar_pos[1]) / denom_xy
        pose.pose.position.z = (wv * visual_pos[2] + wl_z * lidar_pos[2]) / denom_z
        pose.pose.orientation.w = 1.0
        return pose

    def _default_covariance(self, source, confidence):
        base = (
            self.default_visual_covariance
            if source in ("visual", "fused")
            else self.default_lidar_covariance
        )
        cov = base / max(confidence, 0.1)
        return cov

    def _publish_pose_cov(self, pose, pose_cov, source, confidence):
        out = PoseWithCovarianceStamped()
        out.header = pose.header
        out.pose.pose = pose.pose
        if pose_cov is not None:
            out.pose.covariance = pose_cov.pose.covariance
        else:
            cov = self._default_covariance(source, confidence)
            out.pose.covariance[0] = cov
            out.pose.covariance[7] = cov
            out.pose.covariance[14] = cov * 1.2
            out.pose.covariance[21] = 1.0
            out.pose.covariance[28] = 1.0
            out.pose.covariance[35] = 1.0
        self.pose_cov_pub.publish(out)

    def _build_marker(self, pose, source, confidence):
        markers = MarkerArray()
        marker = Marker()
        marker.header = pose.header
        marker.ns = "target_fused"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = 0.26
        marker.scale.y = 0.26
        marker.scale.z = 0.26
        if source == "visual":
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.25
        elif source == "lidar":
            marker.color.r, marker.color.g, marker.color.b = 0.2, 0.55, 1.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.8, 0.0
        marker.color.a = 0.9
        marker.lifetime = rospy.Duration(0.5)
        markers.markers.append(marker)

        text = Marker()
        text.header = pose.header
        text.ns = "target_fused_text"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = pose.pose.position
        text.pose.position.z += 0.5
        text.pose.orientation.w = 1.0
        text.scale.z = 0.26
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 0.95
        text.text = "{} conf={:.2f}".format(source, confidence)
        text.lifetime = rospy.Duration(0.5)
        markers.markers.append(text)
        return markers

    def _smooth_pose(self, pose, source):
        pose = self._clamp_pose_z_bounds(pose)
        if (
            self.last_output_pose is None
            or self.output_smoothing_alpha <= 0.0
            or self.output_smoothing_alpha >= 1.0
        ):
            self.last_output_pose = pose
            self.last_output_stamp = pose.header.stamp
            self.last_output_source = source
            return pose
        if source != self.last_output_source:
            self.last_output_pose = pose
            self.last_output_stamp = pose.header.stamp
            self.last_output_source = source
            return pose
        if pose.header.frame_id != self.last_output_pose.header.frame_id:
            self.last_output_pose = pose
            self.last_output_stamp = pose.header.stamp
            self.last_output_source = source
            return pose

        current = self._pos(pose)
        previous_pose = self._clamp_pose_z_bounds(self.last_output_pose)
        previous = self._pos(previous_pose)
        if self.clamp_output_motion:
            pose = self._clamp_pose_step(pose, previous)
            current = self._pos(pose)

        alpha = self.output_smoothing_alpha
        smoothed = PoseStamped()
        smoothed.header = pose.header
        smoothed.pose.position.x = previous[0] + alpha * (current[0] - previous[0])
        smoothed.pose.position.y = previous[1] + alpha * (current[1] - previous[1])
        smoothed.pose.position.z = previous[2] + alpha * (current[2] - previous[2])
        smoothed.pose.orientation = pose.pose.orientation
        if smoothed.pose.orientation.w == 0.0:
            smoothed.pose.orientation.w = 1.0
        smoothed = self._clamp_pose_z_bounds(smoothed)
        self.last_output_pose = smoothed
        self.last_output_stamp = smoothed.header.stamp
        self.last_output_source = source
        return smoothed

    def _copy_pose(self, pose):
        out = PoseStamped()
        out.header = pose.header
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = pose.pose.position.z
        out.pose.orientation = pose.pose.orientation
        if out.pose.orientation.w == 0.0:
            out.pose.orientation.w = 1.0
        return out

    def _clamp_pose_z_bounds(self, pose):
        if not self.clamp_output_z_bounds:
            return pose
        z = pose.pose.position.z
        limited_z = self._clamp(z, self.output_z_min, self.output_z_max)
        if abs(limited_z - z) < 1e-6:
            return pose
        out = PoseStamped()
        out.header = pose.header
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = limited_z
        out.pose.orientation = pose.pose.orientation
        if out.pose.orientation.w == 0.0:
            out.pose.orientation.w = 1.0
        rospy.logwarn_throttle(
            0.5,
            "[TargetSensorFusionClamp] z_bound candidate_z=%.2f limited_z=%.2f bounds=[%.2f %.2f]",
            z,
            limited_z,
            self.output_z_min,
            self.output_z_max,
        )
        return out

    def _clamp_pose_step(self, pose, previous):
        stamp = pose.header.stamp if pose.header.stamp != rospy.Time() else rospy.Time.now()
        if self.last_output_stamp is None:
            return pose
        dt = max((stamp - self.last_output_stamp).to_sec(), 1.0 / self.publish_rate)
        current = self._pos(pose)
        dx = current[0] - previous[0]
        dy = current[1] - previous[1]
        dz = current[2] - previous[2]
        dxy = math.sqrt(dx * dx + dy * dy)
        max_xy = max(self.max_output_speed_xy * dt, self.min_output_step_xy)
        max_z = max(self.max_output_speed_z * dt, self.min_output_step_z)

        out = PoseStamped()
        out.header = pose.header
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = pose.pose.position.z
        out.pose.orientation = pose.pose.orientation
        if out.pose.orientation.w == 0.0:
            out.pose.orientation.w = 1.0
        if dxy > max_xy and dxy > 1e-6:
            scale = max_xy / dxy
            out.pose.position.x = previous[0] + dx * scale
            out.pose.position.y = previous[1] + dy * scale
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusionClamp] xy_step=%.2f limited_to=%.2f candidate=(%.2f %.2f %.2f) previous=(%.2f %.2f %.2f)",
                dxy,
                max_xy,
                current[0],
                current[1],
                current[2],
                previous[0],
                previous[1],
                previous[2],
            )
        if abs(dz) > max_z:
            out.pose.position.z = previous[2] + math.copysign(max_z, dz)
            rospy.logwarn_throttle(
                0.5,
                "[TargetSensorFusionClamp] z_step=%.2f limited_to=%.2f candidate_z=%.2f previous_z=%.2f",
                abs(dz),
                max_z,
                current[2],
                previous[2],
            )
        out.pose.orientation.w = 1.0
        return out

    def _jump_rejected(self, pose, source, now):
        if not self.reject_large_jumps or self.last_valid_output_pose is None:
            return False
        if self.last_valid_output_stamp is None:
            return False
        if pose.header.frame_id != self.last_valid_output_pose.header.frame_id:
            return False
        ref_age = (now - self.last_valid_output_stamp).to_sec()
        if self.max_jump_reference_age > 0.0 and ref_age > self.max_jump_reference_age:
            return False

        current = self._pos(pose)
        previous = self._pos(self.last_valid_output_pose)
        jump = self._dist(current, previous)
        threshold = (
            self.lidar_max_jump_distance
            if source == "lidar"
            else self.visual_max_jump_distance
        )
        if threshold <= 0.0 or jump <= threshold:
            return False

        rospy.logwarn_throttle(
            0.5,
            "[TargetSensorFusionReject] source=%s rejected_large_jump=%.2f threshold=%.2f ref_age=%.2f previous=(%.2f %.2f %.2f) candidate=(%.2f %.2f %.2f)",
            source,
            jump,
            threshold,
            ref_age,
            previous[0],
            previous[1],
            previous[2],
            current[0],
            current[1],
            current[2],
        )
        return True

    def _publish_none(self, now):
        if self._publish_hold(now):
            return
        self.last_output_pose = None
        self.last_output_stamp = None
        self.last_output_source = "none"
        visual_age = self._age(now, self.visual.pose_stamp)
        lidar_age = self._age(now, self.lidar.pose_stamp)
        visual_ok = self._fresh(
            self.visual, now, self.visual_min_confidence, "visual"
        )
        lidar_ok = self._fresh(self.lidar, now, self.lidar_min_confidence, "lidar")
        rospy.loginfo_throttle(
            1.0,
            "[TargetSensorFusionDiag] source=none visual_ok=%s visual_valid=%s visual_conf=%.2f visual_age=%.2f lidar_ok=%s lidar_valid=%s lidar_conf=%.2f lidar_age=%.2f max_age=%.2f",
            visual_ok,
            self.visual.valid,
            self.visual.confidence,
            visual_age,
            lidar_ok,
            self.lidar.valid,
            self.lidar.confidence,
            lidar_age,
            self.max_age,
        )
        self.valid_pub.publish(Bool(data=False))
        self.confidence_pub.publish(Float32(data=0.0))
        self.source_pub.publish(String(data="none"))
        header = Header(stamp=now, frame_id="map")
        delete_sphere = Marker()
        delete_sphere.header = header
        delete_sphere.ns = "target_fused"
        delete_sphere.id = 0
        delete_sphere.action = Marker.DELETE
        text = Marker()
        text.header = header
        text.ns = "target_fused_text"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.scale.z = 0.25
        text.color.r = 1.0
        text.color.g = 0.6
        text.color.b = 0.0
        text.color.a = 0.8
        text.text = "fused source=none"
        text.lifetime = rospy.Duration(0.5)
        self.marker_pub.publish(MarkerArray(markers=[delete_sphere, text]))

    def _publish_hold(self, now):
        if (
            not self.hold_last_on_none
            or self.last_valid_output_pose is None
            or self.last_valid_output_stamp is None
            or self.hold_last_max_age <= 0.0
        ):
            return False
        hold_age = (now - self.last_valid_output_stamp).to_sec()
        if hold_age < 0.0 or hold_age > self.hold_last_max_age:
            return False
        pose = PoseStamped()
        pose.header = self.last_valid_output_pose.header
        pose.header.stamp = now
        pose.pose.position.x = self.last_valid_output_pose.pose.position.x
        pose.pose.position.y = self.last_valid_output_pose.pose.position.y
        pose.pose.position.z = self.last_valid_output_pose.pose.position.z
        pose.pose.orientation = self.last_valid_output_pose.pose.orientation
        if pose.pose.orientation.w == 0.0:
            pose.pose.orientation.w = 1.0
        pose = self._clamp_pose_z_bounds(pose)
        confidence = self._clamp(
            self.last_valid_output_confidence * self.hold_confidence_scale,
            0.0,
            1.0,
        )
        source = self.last_valid_output_source + "_hold"
        rospy.loginfo_throttle(
            0.5,
            "[TargetSensorFusionHold] source=%s hold_age=%.2f confidence=%.2f pose=(%.2f %.2f %.2f)",
            source,
            hold_age,
            confidence,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        # valid + confidence BEFORE pose — same ordering fix as above.
        # The hold publishes valid=False; if pose arrives first the
        # estimator would see the *previous* valid=True and accept the
        # stale held position as a real observation.
        self.confidence_pub.publish(Float32(data=confidence))
        self.valid_pub.publish(Bool(data=False))
        self.source_pub.publish(String(data=source))
        self.pose_pub.publish(pose)
        self._publish_pose_cov(pose, None, self.last_valid_output_source, confidence)
        self.marker_pub.publish(self._build_marker(pose, source, confidence))
        self.last_output_pose = pose
        self.last_output_stamp = now
        self.last_output_source = source
        return True

    def publish(self):
        now = rospy.Time.now()
        pose, pose_cov, source, confidence = self._choose(now)
        if pose is None:
            self._publish_none(now)
            return
        out_pose = PoseStamped()
        out_pose.header = pose.header
        if out_pose.header.stamp == rospy.Time():
            out_pose.header.stamp = now
        out_pose.pose.position.x = pose.pose.position.x
        out_pose.pose.position.y = pose.pose.position.y
        out_pose.pose.position.z = pose.pose.position.z
        out_pose.pose.orientation = pose.pose.orientation
        if out_pose.pose.orientation.w == 0.0:
            out_pose.pose.orientation.w = 1.0
        if self._jump_rejected(out_pose, source, now):
            self._publish_none(now)
            return
        out_pose = self._smooth_pose(out_pose, source)
        rospy.loginfo_throttle(
            1.0,
            "[TargetSensorFusionDiag] source=%s confidence=%.2f visual_valid=%s visual_conf=%.2f lidar_valid=%s lidar_conf=%.2f pose=(%.2f %.2f %.2f)",
            source,
            confidence,
            self.visual.valid,
            self.visual.confidence,
            self.lidar.valid,
            self.lidar.confidence,
            out_pose.pose.position.x,
            out_pose.pose.position.y,
            out_pose.pose.position.z,
        )
        # Publish valid + confidence BEFORE pose so that the estimator
        # sees the correct validity flag when the pose callback fires.
        # Without this, a stale valid=False from a previous hold cycle
        # causes the estimator to reject a perfectly good observation.
        self.confidence_pub.publish(Float32(data=confidence))
        self.valid_pub.publish(Bool(data=True))
        self.source_pub.publish(String(data=source))
        self.pose_pub.publish(out_pose)
        self._publish_pose_cov(out_pose, pose_cov, source, confidence)
        self.marker_pub.publish(self._build_marker(out_pose, source, confidence))
        self.last_valid_output_pose = out_pose
        self.last_valid_output_stamp = now
        self.last_valid_output_source = source
        self.last_valid_output_confidence = confidence

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


def main():
    rospy.init_node("target_sensor_fusion_node")
    TargetSensorFusion().run()


if __name__ == "__main__":
    main()
