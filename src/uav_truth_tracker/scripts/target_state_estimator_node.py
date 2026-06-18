#!/usr/bin/env python3
"""Estimate target state and publish an observation-only intercept point."""

import math
from collections import deque

import rospy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Header, String, UInt32
from visualization_msgs.msg import Marker, MarkerArray


class TargetStateEstimator:
    FORMAL_MODELS = ("kalman_cv", "kalman_ca")
    DEBUG_MODELS = (
        "current",
        "linear_observation",
        "learned_circle",
        "circle_configured_debug",
    )

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.chaser_odom_topic = rospy.get_param(
            "~chaser_odom_topic", "/uav0/mavros/local_position/odom"
        )
        self.target_odom_topic = rospy.get_param(
            "~target_odom_topic", "/uav1/mavros/local_position/odom"
        )
        self.observation_source = rospy.get_param(
            "~observation_source", "truth_odom"
        ).lower()
        self.target_pose_topic = rospy.get_param(
            "~target_pose_topic", "/target_observation/pose"
        )
        self.target_pose_cov_topic = rospy.get_param(
            "~target_pose_cov_topic", "/target_observation/pose_cov"
        )
        self.prefer_pose_cov_observation = rospy.get_param(
            "~prefer_pose_cov_observation", False
        )
        self.use_receive_time_for_target_pose = rospy.get_param(
            "~use_receive_time_for_target_pose", True
        )
        self.observation_confidence_topic = rospy.get_param(
            "~observation_confidence_topic", "/target_observation/confidence"
        )
        self.observation_valid_topic = rospy.get_param(
            "~observation_valid_topic", "/target_observation/valid"
        )
        self.max_observation_age = max(
            rospy.get_param("~max_observation_age", 0.5), 0.0
        )
        self.max_prediction_only_age = max(
            rospy.get_param("~max_prediction_only_age", 1.5), self.max_observation_age
        )
        self.use_observation_confidence = rospy.get_param(
            "~use_observation_confidence", True
        )
        self.min_update_confidence = self._clamp(
            rospy.get_param("~min_update_confidence", 0.25), 0.0, 1.0
        )
        self.low_confidence_update_floor = self._clamp(
            rospy.get_param("~low_confidence_update_floor", 0.25), 0.0, 1.0
        )
        self.low_confidence_noise_threshold = self._clamp(
            rospy.get_param("~low_confidence_noise_threshold", 0.55), 0.0, 1.0
        )
        self.adaptive_measurement_noise = rospy.get_param(
            "~adaptive_measurement_noise", True
        )
        self.max_predict_only_time = max(
            rospy.get_param("~max_predict_only_time", 0.8), 0.0
        )
        self.freeze_prediction_after_lost = rospy.get_param(
            "~freeze_prediction_after_lost", True
        )
        self.max_covariance_trace = max(
            rospy.get_param("~max_covariance_trace", 100.0), 0.0
        )
        self.target_lost_timeout = max(
            rospy.get_param("~target_lost_timeout", 1.5), self.max_predict_only_time
        )
        self.hold_last_observation_time = max(
            rospy.get_param("~hold_last_observation_time", 0.3), 0.0
        )
        self.predict_only_publish = rospy.get_param("~predict_only_publish", True)

        self.state_topic = rospy.get_param("~state_topic", "/target_estimator/state")
        self.intercept_topic = rospy.get_param(
            "~intercept_topic", "/target_estimator/intercept_point"
        )
        self.t_go_topic = rospy.get_param("~t_go_topic", "/target_estimator/t_go")
        self.debug_marker_topic = rospy.get_param(
            "~debug_marker_topic", "/target_estimator/debug_marker"
        )
        self.tracking_state_topic = rospy.get_param(
            "~tracking_state_topic", "/target_estimator/tracking_state"
        )
        self.observation_age_topic = rospy.get_param(
            "~observation_age_topic", "/target_estimator/observation_age"
        )
        self.estimator_confidence_topic = rospy.get_param(
            "~estimator_confidence_topic", "/target_estimator/observation_confidence"
        )
        self.rejected_observation_count_topic = rospy.get_param(
            "~rejected_observation_count_topic",
            "/target_estimator/rejected_observation_count",
        )

        self.use_spawn_offsets = rospy.get_param("~use_spawn_offsets", True)
        self.uav0_spawn_offset = (
            rospy.get_param("~uav0_spawn_x", 0.0),
            rospy.get_param("~uav0_spawn_y", 0.0),
            rospy.get_param("~uav0_spawn_z", 0.0),
        )
        self.uav1_spawn_offset = (
            rospy.get_param("~uav1_spawn_x", 2.0),
            rospy.get_param("~uav1_spawn_y", 0.0),
            rospy.get_param("~uav1_spawn_z", 0.0),
        )

        self.estimator_model = rospy.get_param("~estimator_model", "kalman_cv").lower()
        self.assumed_chaser_speed_xy = max(
            rospy.get_param("~assumed_chaser_speed_xy", 1.2), 0.05
        )
        self.assumed_chaser_speed_z = max(
            rospy.get_param("~assumed_chaser_speed_z", 0.8), 0.05
        )
        self.min_prediction_time = max(
            rospy.get_param("~min_prediction_time", 0.3), 0.0
        )
        self.max_prediction_time = max(
            rospy.get_param("~max_prediction_time", 2.5), self.min_prediction_time
        )
        self.intercept_iterations = max(
            int(rospy.get_param("~intercept_iterations", 5)), 1
        )
        self.publish_rate = max(rospy.get_param("~publish_rate", 20.0), 1.0)
        self.min_observations_for_prediction = max(
            int(rospy.get_param("~min_observations_for_prediction", 5)), 1
        )
        self.velocity_fit_window = max(
            rospy.get_param("~velocity_fit_window", 0.8), 0.05
        )
        self.min_velocity_fit_points = max(
            int(rospy.get_param("~min_velocity_fit_points", 4)), 2
        )
        self.velocity_smoothing_alpha = min(
            max(rospy.get_param("~velocity_smoothing_alpha", 0.35), 0.0), 1.0
        )
        self.max_target_speed_xy = max(
            rospy.get_param("~max_target_speed_xy", 2.5), 0.05
        )
        self.max_target_speed_z = max(
            rospy.get_param("~max_target_speed_z", 1.5), 0.05
        )
        self.reject_observation_jumps = rospy.get_param(
            "~reject_observation_jumps", False
        )
        self.max_observation_jump_speed_xy = max(
            rospy.get_param("~max_observation_jump_speed_xy", 4.0), 0.05
        )
        self.max_observation_jump_speed_z = max(
            rospy.get_param("~max_observation_jump_speed_z", 2.5), 0.05
        )
        self.max_observation_jump_distance_xy = max(
            rospy.get_param("~max_observation_jump_distance_xy", 5.0), 0.05
        )
        self.max_observation_jump_distance_z = max(
            rospy.get_param("~max_observation_jump_distance_z", 3.0), 0.05
        )
        self.kf_velocity_blend = min(
            max(rospy.get_param("~kf_velocity_blend", 0.05), 0.0), 1.0
        )
        self.kf_accel_blend = min(
            max(rospy.get_param("~kf_accel_blend", 0.02), 0.0), 1.0
        )
        self.max_target_acc_xy = max(
            rospy.get_param("~max_target_acc_xy", 0.7), 0.05
        )
        self.max_target_acc_z = max(
            rospy.get_param("~max_target_acc_z", 0.4), 0.05
        )
        self.ca_accel_prediction_horizon = max(
            rospy.get_param("~ca_accel_prediction_horizon", 1.0), 0.0
        )

        self.kf_process_noise_pos = max(
            rospy.get_param("~kf_process_noise_pos", 0.05), 1e-9
        )
        self.kf_process_noise_vel = max(
            rospy.get_param("~kf_process_noise_vel", 0.2), 1e-9
        )
        self.kf_process_noise_acc = max(
            rospy.get_param("~kf_process_noise_acc", 0.05), 1e-9
        )
        self.kf_measurement_noise = max(
            rospy.get_param("~kf_measurement_noise", 0.1), 1e-9
        )

        self.observation_buffer_size = max(
            int(rospy.get_param("~observation_buffer_size", 80)), 3
        )
        self.min_circle_fit_points = max(
            int(rospy.get_param("~min_circle_fit_points", 20)), 3
        )
        self.circle_fit_max_residual = max(
            rospy.get_param("~circle_fit_max_residual", 0.8), 0.0
        )
        self.learned_circle_min_radius = max(
            rospy.get_param("~learned_circle_min_radius", 0.5), 0.01
        )
        self.learned_circle_max_radius = max(
            rospy.get_param("~learned_circle_max_radius", 20.0),
            self.learned_circle_min_radius,
        )

        self.target_center_x = 0.0
        self.target_center_y = 0.0
        self.target_radius = 1.0
        self.target_speed = 0.0
        self.target_direction = 1.0
        self.infer_target_direction = True
        self.target_height = 0.0
        self.target_center_frame = "uav1_local"
        if self.estimator_model in ("circle_configured_debug", "circle_configured"):
            rospy.logwarn(
                "[TargetStateEstimator] circle_configured_debug uses target trajectory parameters and must not be used as a formal predictor."
            )
            self.target_center_x = rospy.get_param("~target_center_x", 2.0)
            self.target_center_y = rospy.get_param("~target_center_y", 0.0)
            self.target_radius = max(rospy.get_param("~target_radius", 3.0), 0.05)
            self.target_speed = max(rospy.get_param("~target_speed", 0.6), 0.0)
            self.target_direction = rospy.get_param("~target_direction", 1.0)
            self.infer_target_direction = rospy.get_param("~infer_target_direction", True)
            self.target_height = rospy.get_param("~target_height", 0.0)
            self.target_center_frame = rospy.get_param(
                "~target_center_frame", "uav1_local"
            ).lower()
            if self.target_center_frame not in ("uav1_local", "map"):
                rospy.logwarn(
                    "[TargetStateEstimator] unknown target_center_frame '%s', using uav1_local",
                    self.target_center_frame,
                )
                self.target_center_frame = "uav1_local"

        self.marker_scale = rospy.get_param("~marker_scale", 0.45)

        self.chaser_odom = None
        self.chaser_common = None
        self.last_observation_stamp = None
        self.last_observation_pos = None
        self.observed_pos = None
        self.latest_observation_confidence = 1.0
        self.latest_observation_valid = True
        self.latest_pose_covariance = None
        self.received_target_pose = False
        self.latest_update_allowed = False
        self.latest_update_rejected_reason = "no_observation"
        self.rejected_observation_count = 0
        self.has_intercept_point = False
        self.tracking_state = "LOST"
        self.raw_velocity = (0.0, 0.0, 0.0)
        self.raw_acceleration = (0.0, 0.0, 0.0)
        self.velocity_initialized = False
        self.acceleration_initialized = False
        self.last_velocity_fit = None
        self.last_velocity_fit_stamp = None
        self.observations = deque(maxlen=self.observation_buffer_size)

        self.cv_x = None
        self.cv_p = None
        self.cv_stamp = None
        self.ca_x = None
        self.ca_p = None
        self.ca_stamp = None
        self.last_circle_fit = None
        self.active_model = self.estimator_model
        self.fallback_reason = ""

        self.state_pub = rospy.Publisher(self.state_topic, Odometry, queue_size=10)
        self.intercept_pub = rospy.Publisher(
            self.intercept_topic, PoseStamped, queue_size=10
        )
        self.t_go_pub = rospy.Publisher(self.t_go_topic, Float32, queue_size=10)
        self.marker_pub = rospy.Publisher(
            self.debug_marker_topic, MarkerArray, queue_size=10
        )
        self.tracking_state_pub = rospy.Publisher(
            self.tracking_state_topic, String, queue_size=10
        )
        self.observation_age_pub = rospy.Publisher(
            self.observation_age_topic, Float32, queue_size=10
        )
        self.observation_confidence_pub = rospy.Publisher(
            self.estimator_confidence_topic, Float32, queue_size=10
        )
        self.rejected_observation_count_pub = rospy.Publisher(
            self.rejected_observation_count_topic, UInt32, queue_size=10
        )

        self.chaser_sub = rospy.Subscriber(
            self.chaser_odom_topic, Odometry, self.chaser_odom_cb, queue_size=10
        )
        if self.observation_source == "truth_odom":
            self.target_sub = rospy.Subscriber(
                self.target_odom_topic, Odometry, self.target_odom_cb, queue_size=10
            )
        elif self.observation_source == "target_pose":
            self.target_sub = rospy.Subscriber(
                self.target_pose_topic, PoseStamped, self.target_pose_cb, queue_size=10
            )
            self.target_pose_cov_sub = rospy.Subscriber(
                self.target_pose_cov_topic,
                PoseWithCovarianceStamped,
                self.target_pose_cov_cb,
                queue_size=10,
            )
            self.observation_confidence_sub = rospy.Subscriber(
                self.observation_confidence_topic,
                Float32,
                self.observation_confidence_cb,
                queue_size=10,
            )
            self.observation_valid_sub = rospy.Subscriber(
                self.observation_valid_topic,
                Bool,
                self.observation_valid_cb,
                queue_size=10,
            )
            rospy.loginfo(
                "[TargetStateEstimator] observation_source=target_pose; target observations are expected in common/map and no uav1 spawn offset is applied"
            )
        else:
            rospy.logwarn(
                "[TargetStateEstimator] unknown observation_source '%s', using truth_odom",
                self.observation_source,
            )
            self.observation_source = "truth_odom"
            self.target_sub = rospy.Subscriber(
                self.target_odom_topic, Odometry, self.target_odom_cb, queue_size=10
            )

        rospy.loginfo(
            "[TargetStateEstimator] model=%s formal_models=%s debug_models=%s source=%s chaser=%s target_odom=%s target_pose=%s state=%s intercept=%s t_go=%s marker=%s offsets=%s uav0=(%.2f %.2f %.2f) uav1=(%.2f %.2f %.2f) speed_xy=%.2f speed_z=%.2f pred=[%.2f %.2f] rate=%.1f max_obs_age=%.2f velocity_fit_window=%.2f smooth=%.2f max_target_speed=(%.2f %.2f) kf_blend=(vel %.2f acc %.2f) max_acc=(%.2f %.2f) ca_accel_horizon=%.2f",
            self.estimator_model,
            ",".join(self.FORMAL_MODELS),
            ",".join(self.DEBUG_MODELS),
            self.observation_source,
            self.chaser_odom_topic,
            self.target_odom_topic,
            self.target_pose_topic,
            self.state_topic,
            self.intercept_topic,
            self.t_go_topic,
            self.debug_marker_topic,
            self.use_spawn_offsets,
            self.uav0_spawn_offset[0],
            self.uav0_spawn_offset[1],
            self.uav0_spawn_offset[2],
            self.uav1_spawn_offset[0],
            self.uav1_spawn_offset[1],
            self.uav1_spawn_offset[2],
            self.assumed_chaser_speed_xy,
            self.assumed_chaser_speed_z,
            self.min_prediction_time,
            self.max_prediction_time,
            self.publish_rate,
            self.max_observation_age,
            self.velocity_fit_window,
            self.velocity_smoothing_alpha,
            self.max_target_speed_xy,
            self.max_target_speed_z,
            self.kf_velocity_blend,
            self.kf_accel_blend,
            self.max_target_acc_xy,
            self.max_target_acc_z,
            self.ca_accel_prediction_horizon,
        )

    @staticmethod
    def _norm_xy(x, y):
        return math.sqrt(x * x + y * y)

    @staticmethod
    def _norm3(x, y, z):
        return math.sqrt(x * x + y * y + z * z)

    @staticmethod
    def _clamp(value, low, high):
        return min(max(value, low), high)

    @staticmethod
    def _pos_from_odom(msg):
        p = msg.pose.pose.position
        return p.x, p.y, p.z

    @staticmethod
    def _pos_from_pose(msg):
        p = msg.pose.position
        return p.x, p.y, p.z

    def _to_common(self, position, spawn_offset):
        if not self.use_spawn_offsets:
            return position
        return (
            position[0] + spawn_offset[0],
            position[1] + spawn_offset[1],
            position[2] + spawn_offset[2],
        )

    def _target_center_common(self, target_pos=None):
        if self.target_center_frame == "map":
            base = (self.target_center_x, self.target_center_y, 0.0)
        else:
            base = self._to_common(
                (self.target_center_x, self.target_center_y, 0.0),
                self.uav1_spawn_offset,
            )
        if self.target_height > 0.0:
            z = self.target_height
            if self.use_spawn_offsets and self.target_center_frame == "uav1_local":
                z += self.uav1_spawn_offset[2]
        elif target_pos is not None:
            z = target_pos[2]
        else:
            z = base[2]
        return base[0], base[1], z

    def chaser_odom_cb(self, msg):
        self.chaser_odom = msg
        self.chaser_common = self._to_common(
            self._pos_from_odom(msg), self.uav0_spawn_offset
        )

    def target_odom_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        self._handle_observation(
            self._to_common(self._pos_from_odom(msg), self.uav1_spawn_offset), stamp
        )

    def target_pose_cb(self, msg):
        if self.prefer_pose_cov_observation:
            return
        self.received_target_pose = True
        stamp = (
            rospy.Time.now()
            if self.use_receive_time_for_target_pose
            else msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "[TargetStateEstimator] target_pose frame_id='%s' differs from estimator frame_id='%s'; assuming pose is already in common/map coordinates",
                msg.header.frame_id,
                self.frame_id,
            )
        self._handle_observation(self._pos_from_pose(msg), stamp)

    def target_pose_cov_cb(self, msg):
        self.latest_pose_covariance = list(msg.pose.covariance)
        self.received_target_pose = True
        if self.prefer_pose_cov_observation:
            self.latest_observation_valid = True
            self.latest_observation_confidence = max(
                self.latest_observation_confidence, self.low_confidence_update_floor
            )
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            rospy.logwarn_throttle(
                5.0,
                "[TargetStateEstimator] target_pose_cov frame_id='%s' differs from estimator frame_id='%s'; assuming pose is already in common/map coordinates",
                msg.header.frame_id,
                self.frame_id,
            )
        if self.prefer_pose_cov_observation:
            stamp = (
                rospy.Time.now()
                if self.use_receive_time_for_target_pose
                else msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            )
            p = msg.pose.pose.position
            self._handle_observation((p.x, p.y, p.z), stamp)

    def observation_confidence_cb(self, msg):
        self.latest_observation_confidence = self._clamp(float(msg.data), 0.0, 1.0)

    def observation_valid_cb(self, msg):
        self.latest_observation_valid = bool(msg.data)

    def _observation_age(self, now=None):
        if self.last_observation_stamp is None:
            return float("nan")
        if now is None:
            now = rospy.Time.now()
        return max((now - self.last_observation_stamp).to_sec(), 0.0)

    def _target_lost(self, now=None):
        if self.observation_source != "target_pose":
            return False
        age = self._observation_age(now)
        return math.isfinite(age) and age > self.max_observation_age

    def _tracking_state(self, now=None):
        if self.observation_source != "target_pose":
            return "TRACKING"
        age = self._observation_age(now)
        if not math.isfinite(age):
            return "LOST"
        if age <= self.max_observation_age:
            return "TRACKING"
        if age <= self.max_predict_only_time:
            return "PREDICT_ONLY"
        return "LOST"

    def _covariance_trace(self):
        matrices = []
        if self.cv_p is not None:
            matrices.append(self.cv_p)
        if self.ca_p is not None:
            matrices.append(self.ca_p)
        if not matrices:
            return float("nan")
        active = self.ca_p if self.active_model == "kalman_ca" and self.ca_p is not None else matrices[0]
        return sum(active[i][i] for i in range(min(len(active), len(active[0]))))

    def _measurement_noise_diag(self):
        confidence = max(self.latest_observation_confidence, 0.05)
        low_confidence_scale = 1.0
        if (
            self.use_observation_confidence
            and confidence < self.low_confidence_noise_threshold
        ):
            low_confidence_scale = self.low_confidence_noise_threshold / confidence
        if self.adaptive_measurement_noise and self.latest_pose_covariance:
            cov = self.latest_pose_covariance
            diag = [
                max(float(cov[0]), 1e-6),
                max(float(cov[7]), 1e-6),
                max(float(cov[14]), 1e-6),
            ]
            return [value * low_confidence_scale for value in diag]
        if self.adaptive_measurement_noise and self.use_observation_confidence:
            noise = self.kf_measurement_noise / confidence
            return [noise] * 3
        return [self.kf_measurement_noise] * 3

    def _observation_update_status(self):
        if self.observation_source != "target_pose":
            return True, "truth_odom"
        if not self.latest_observation_valid:
            return False, "observation_valid_false"
        if not self.use_observation_confidence:
            return True, "confidence_disabled"
        effective_floor = min(self.min_update_confidence, self.low_confidence_update_floor)
        if self.latest_observation_confidence < effective_floor:
            return False, "confidence_below_{:.2f}".format(effective_floor)
        return True, "accepted_low_confidence" if (
            self.latest_observation_confidence < self.low_confidence_noise_threshold
        ) else "accepted"

    def _observation_jump_status(self, pos, stamp):
        if not self.reject_observation_jumps:
            return True, "jump_check_disabled"
        if self.observation_source != "target_pose":
            return True, "truth_odom"
        if self.last_observation_stamp is None or self.last_observation_pos is None:
            return True, "first_observation"
        dt = (stamp - self.last_observation_stamp).to_sec()
        if dt <= 1e-3:
            return True, "zero_dt"
        dx = pos[0] - self.last_observation_pos[0]
        dy = pos[1] - self.last_observation_pos[1]
        dz = pos[2] - self.last_observation_pos[2]
        distance_xy = self._norm_xy(dx, dy)
        distance_z = abs(dz)
        speed_xy = distance_xy / dt
        speed_z = distance_z / dt
        # Reject jumps that are unreasonably large *or* unreasonably fast.
        # The target moves inside a ~1 m Z band at ≤ 0.26 m/s, so a 1.5 m
        # jump or 1.0 m/s speed in Z is already a clear outlier.
        if (
            distance_xy > self.max_observation_jump_distance_xy
            or speed_xy > self.max_observation_jump_speed_xy
        ):
            return False, "observation_jump_xy_{:.1f}m_{:.1f}mps".format(
                distance_xy, speed_xy
            )
        if (
            distance_z > self.max_observation_jump_distance_z
            or speed_z > self.max_observation_jump_speed_z
        ):
            return False, "observation_jump_z_{:.1f}m_{:.1f}mps".format(
                distance_z, speed_z
            )
        # --- Kalman-consistency gate ---
        # Once the Kalman has converged, reject observations whose Z
        # coordinate is improbably far from the filterʼs own prediction.
        # The gate width scales with the Kalman Z-variance so that a
        # well-converged filter is strict while an uncertain one remains
        # open to corrections.  This catches stale held poses that slip
        # through valid-ordering races without hardcoding any altitude.
        if self.cv_x is not None:
            kf_z = self.cv_x[2]
            sigma_z = math.sqrt(max(self.cv_p[2][2], 0.01))
            gate = max(1.5, 4.0 * sigma_z)
            if abs(pos[2] - kf_z) > gate:
                return False, "kalman_z_gate_{:.1f}_kf_{:.1f}_sigma_{:.2f}".format(
                    pos[2], kf_z, sigma_z
                )
        return True, "accepted"

    def _handle_observation(self, pos, stamp):
        update_allowed, reason = self._observation_update_status()
        if update_allowed:
            update_allowed, reason = self._observation_jump_status(pos, stamp)
        self.latest_update_allowed = update_allowed
        self.latest_update_rejected_reason = "none" if update_allowed else reason
        if not update_allowed:
            self.rejected_observation_count += 1
            rospy.logwarn_throttle(
                1.0,
                "[TargetStateEstimator] received_target_pose=%s observation_valid=%s observation_confidence=%.2f update_allowed=%s update_rejected_reason=%s tracking_state=%s rejected_observation_count=%d",
                self.received_target_pose,
                self.latest_observation_valid,
                self.latest_observation_confidence,
                update_allowed,
                self.latest_update_rejected_reason,
                self.tracking_state,
                self.rejected_observation_count,
            )
            return
        rospy.loginfo_throttle(
            1.0,
            "[TargetStateEstimator] received_target_pose=%s observation_valid=%s observation_confidence=%.2f update_allowed=%s update_rejected_reason=none tracking_state=%s measurement_noise=%s",
            self.received_target_pose,
            self.latest_observation_valid,
            self.latest_observation_confidence,
            update_allowed,
            self.tracking_state,
            ["{:.3f}".format(v) for v in self._measurement_noise_diag()],
        )
        instant_velocity = None
        if self.last_observation_stamp is not None and self.last_observation_pos is not None:
            dt = (stamp - self.last_observation_stamp).to_sec()
            if dt > 1e-3:
                instant_velocity = (
                    (pos[0] - self.last_observation_pos[0]) / dt,
                    (pos[1] - self.last_observation_pos[1]) / dt,
                    (pos[2] - self.last_observation_pos[2]) / dt,
                )
        self.observed_pos = pos
        self.last_observation_stamp = stamp
        self.last_observation_pos = pos
        self.observations.append((stamp.to_sec(), pos))
        measured_velocity = self._fit_observation_velocity()
        if measured_velocity is None:
            measured_velocity = instant_velocity
        if measured_velocity is not None:
            measured_velocity = self._limit_target_velocity(measured_velocity)
            if self.last_velocity_fit is not None and self.last_velocity_fit_stamp is not None:
                dt_vel = (stamp - self.last_velocity_fit_stamp).to_sec()
                if dt_vel > 1e-3:
                    measured_acceleration = self._limit_target_acceleration(
                        (
                            (measured_velocity[0] - self.last_velocity_fit[0]) / dt_vel,
                            (measured_velocity[1] - self.last_velocity_fit[1]) / dt_vel,
                            (measured_velocity[2] - self.last_velocity_fit[2]) / dt_vel,
                        )
                    )
                    if not self.acceleration_initialized:
                        self.raw_acceleration = measured_acceleration
                        self.acceleration_initialized = True
                    else:
                        alpha = self.kf_accel_blend
                        self.raw_acceleration = (
                            self.raw_acceleration[0]
                            + alpha * (measured_acceleration[0] - self.raw_acceleration[0]),
                            self.raw_acceleration[1]
                            + alpha * (measured_acceleration[1] - self.raw_acceleration[1]),
                            self.raw_acceleration[2]
                            + alpha * (measured_acceleration[2] - self.raw_acceleration[2]),
                        )
            self.last_velocity_fit = measured_velocity
            self.last_velocity_fit_stamp = stamp
            if not self.velocity_initialized:
                self.raw_velocity = measured_velocity
                self.velocity_initialized = True
            else:
                alpha = self.velocity_smoothing_alpha
                self.raw_velocity = (
                    self.raw_velocity[0] + alpha * (measured_velocity[0] - self.raw_velocity[0]),
                    self.raw_velocity[1] + alpha * (measured_velocity[1] - self.raw_velocity[1]),
                    self.raw_velocity[2] + alpha * (measured_velocity[2] - self.raw_velocity[2]),
                )
        self._update_kalman_cv(pos, stamp, update=True)
        self._update_kalman_ca(pos, stamp, update=True)

    def _fit_observation_velocity(self):
        if len(self.observations) < self.min_velocity_fit_points:
            return None

        latest_t = self.observations[-1][0]
        samples = [
            sample for sample in self.observations
            if latest_t - sample[0] <= self.velocity_fit_window
        ]
        if len(samples) < self.min_velocity_fit_points:
            return None

        times = [sample[0] for sample in samples]
        t_mean = sum(times) / float(len(times))
        denom = sum((t - t_mean) * (t - t_mean) for t in times)
        if denom <= 1e-6:
            return None

        velocities = []
        for axis in range(3):
            values = [sample[1][axis] for sample in samples]
            v_mean = sum(values) / float(len(values))
            slope = sum(
                (t - t_mean) * (value - v_mean)
                for t, value in zip(times, values)
            ) / denom
            velocities.append(slope)
        return velocities[0], velocities[1], velocities[2]

    def _limit_target_velocity(self, velocity):
        vx, vy, vz = velocity
        speed_xy = self._norm_xy(vx, vy)
        if speed_xy > self.max_target_speed_xy and speed_xy > 1e-6:
            scale = self.max_target_speed_xy / speed_xy
            vx *= scale
            vy *= scale
        vz = self._clamp(vz, -self.max_target_speed_z, self.max_target_speed_z)
        return vx, vy, vz

    def _limit_target_acceleration(self, acceleration):
        ax, ay, az = acceleration
        acc_xy = self._norm_xy(ax, ay)
        if acc_xy > self.max_target_acc_xy and acc_xy > 1e-6:
            scale = self.max_target_acc_xy / acc_xy
            ax *= scale
            ay *= scale
        az = self._clamp(az, -self.max_target_acc_z, self.max_target_acc_z)
        return ax, ay, az

    @staticmethod
    def _identity(n):
        return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]

    @staticmethod
    def _transpose(a):
        return [list(row) for row in zip(*a)]

    @staticmethod
    def _mat_mul(a, b):
        rows = len(a)
        cols = len(b[0])
        inner = len(b)
        out = [[0.0 for _ in range(cols)] for _ in range(rows)]
        for i in range(rows):
            for k in range(inner):
                aik = a[i][k]
                if abs(aik) <= 1e-15:
                    continue
                for j in range(cols):
                    out[i][j] += aik * b[k][j]
        return out

    @staticmethod
    def _mat_vec_mul(a, x):
        return [sum(row[j] * x[j] for j in range(len(x))) for row in a]

    @staticmethod
    def _mat_add(a, b):
        return [
            [a[i][j] + b[i][j] for j in range(len(a[0]))]
            for i in range(len(a))
        ]

    @staticmethod
    def _mat_sub(a, b):
        return [
            [a[i][j] - b[i][j] for j in range(len(a[0]))]
            for i in range(len(a))
        ]

    @staticmethod
    def _diag(values):
        return [
            [values[i] if i == j else 0.0 for j in range(len(values))]
            for i in range(len(values))
        ]

    @staticmethod
    def _inv3(m):
        a, b, c = m[0]
        d, e, f = m[1]
        g, h, i = m[2]
        det = (
            a * (e * i - f * h)
            - b * (d * i - f * g)
            + c * (d * h - e * g)
        )
        if abs(det) < 1e-12:
            return None
        inv_det = 1.0 / det
        return [
            [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
            [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
            [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
        ]

    def _kalman_update(self, x, p, z, h, r_diag):
        hx = self._mat_vec_mul(h, x)
        y = [z[i] - hx[i] for i in range(3)]
        ht = self._transpose(h)
        s = self._mat_add(self._mat_mul(self._mat_mul(h, p), ht), self._diag(r_diag))
        s_inv = self._inv3(s)
        if s_inv is None:
            return x, p
        k = self._mat_mul(self._mat_mul(p, ht), s_inv)
        x_new = [
            x[i] + sum(k[i][j] * y[j] for j in range(3))
            for i in range(len(x))
        ]
        i_mat = self._identity(len(x))
        ikh = self._mat_sub(i_mat, self._mat_mul(k, h))
        kr = self._mat_mul(self._mat_mul(k, self._diag(r_diag)), self._transpose(k))
        p_new = self._mat_add(self._mat_mul(self._mat_mul(ikh, p), self._transpose(ikh)), kr)
        for i in range(len(p_new)):
            for j in range(i + 1, len(p_new[0])):
                avg = 0.5 * (p_new[i][j] + p_new[j][i])
                p_new[i][j] = avg
                p_new[j][i] = avg
        return x_new, p_new

    def _propagate_kalman(self, x, p, f_fn, q_scale, pos, stamp, update, n_states,
                          vel_start, acc_start=None):
        """Multi-step Kalman predict + optional update.

        Breaks large dt into sub-steps to keep the discrete-time linearization
        accurate and to preserve correct process-noise scaling.  A single-step
        propagation at dt > 1.0 s systematically underestimates the state
        advance and the covariance growth, which causes the filter to trust
        stale velocity estimates and can lead to divergence on long runs.
        """
        if x is None:
            # initialise on first call via the normal init branch
            return

        raw_dt = max((stamp - self.cv_stamp).to_sec() if n_states == 6
                     else (stamp - self.ca_stamp).to_sec(), 1e-3)

        # sub-step ceiling – keep per-step dt well inside the linear-approximation
        # regime while limiting the total number of micro-steps
        max_sub_dt = 0.25
        remaining = raw_dt
        while remaining > 1e-6:
            sub_dt = min(remaining, max_sub_dt)
            remaining -= sub_dt

            f = f_fn(sub_dt)
            q = self._diag([s * sub_dt for s in q_scale])
            x_new = self._mat_vec_mul(f, x)
            p_new = self._mat_add(
                self._mat_mul(self._mat_mul(f, p), self._transpose(f)), q)

            # numerical safeguard: clamp diagonal to non-negative
            for i in range(len(p_new)):
                if p_new[i][i] < 0.0:
                    p_new[i][i] = 0.0

            x, p = x_new, p_new

        # update the persistent state *before* the optional measurement step
        if n_states == 6:
            self.cv_x, self.cv_p, self.cv_stamp = x, p, stamp
        else:
            self.ca_x, self.ca_p, self.ca_stamp = x, p, stamp

        if not update:
            return

        h = [[0.0 for _ in range(n_states)] for _ in range(3)]
        for axis in range(3):
            h[axis][axis] = 1.0
        x_up, p_up = self._kalman_update(
            x, p, pos, h, self._measurement_noise_diag())
        if n_states == 6:
            self.cv_x, self.cv_p = x_up, p_up
            self._blend_kalman_velocity(self.cv_x, vel_start)
        else:
            self.ca_x, self.ca_p = x_up, p_up
            self._blend_kalman_velocity(self.ca_x, vel_start)
            self._blend_kalman_acceleration(self.ca_x, acc_start)

    def _update_kalman_cv(self, pos, stamp, update=True):
        if self.cv_x is None:
            self.cv_x = [pos[0], pos[1], pos[2],
                         self.raw_velocity[0], self.raw_velocity[1], self.raw_velocity[2]]
            self.cv_p = self._diag([0.5, 0.5, 0.5, 1.0, 1.0, 1.0])
            self.cv_stamp = stamp
            return

        def f_fn(sub_dt):
            f = self._identity(6)
            for axis in range(3):
                f[axis][axis + 3] = sub_dt
            return f

        q_scale = [self.kf_process_noise_pos] * 3 + [self.kf_process_noise_vel] * 3
        self._propagate_kalman(
            self.cv_x, self.cv_p, f_fn, q_scale,
            pos, stamp, update, 6, vel_start=3)

    def _update_kalman_ca(self, pos, stamp, update=True):
        if self.ca_x is None:
            self.ca_x = [
                pos[0], pos[1], pos[2],
                self.raw_velocity[0], self.raw_velocity[1], self.raw_velocity[2],
                0.0, 0.0, 0.0,
            ]
            self.ca_p = self._diag([0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0])
            self.ca_stamp = stamp
            return

        def f_fn(sub_dt):
            f = self._identity(9)
            half_dt2 = 0.5 * sub_dt * sub_dt
            for axis in range(3):
                f[axis][axis + 3] = sub_dt
                f[axis][axis + 6] = half_dt2
                f[axis + 3][axis + 6] = sub_dt
            return f

        q_scale = (
            [self.kf_process_noise_pos] * 3
            + [self.kf_process_noise_vel] * 3
            + [self.kf_process_noise_acc] * 3
        )
        self._propagate_kalman(
            self.ca_x, self.ca_p, f_fn, q_scale,
            pos, stamp, update, 9, vel_start=3, acc_start=6)

    def _blend_kalman_velocity(self, state, velocity_start):
        if not self.velocity_initialized or self.kf_velocity_blend <= 0.0:
            return
        alpha = self.kf_velocity_blend
        for axis in range(3):
            idx = velocity_start + axis
            state[idx] = state[idx] + alpha * (self.raw_velocity[axis] - state[idx])
        limited = self._limit_target_velocity(
            (state[velocity_start], state[velocity_start + 1], state[velocity_start + 2])
        )
        for axis in range(3):
            state[velocity_start + axis] = limited[axis]

    def _blend_kalman_acceleration(self, state, acceleration_start):
        if not self.acceleration_initialized or self.kf_accel_blend <= 0.0:
            return
        alpha = self.kf_accel_blend
        for axis in range(3):
            idx = acceleration_start + axis
            state[idx] = state[idx] + alpha * (
                self.raw_acceleration[axis] - state[idx]
            )
        limited = self._limit_target_acceleration(
            (
                state[acceleration_start],
                state[acceleration_start + 1],
                state[acceleration_start + 2],
            )
        )
        for axis in range(3):
            state[acceleration_start + axis] = limited[axis]

    @staticmethod
    def _solve_3x3(a, b):
        matrix = [row[:] + [rhs] for row, rhs in zip(a, b)]
        for pivot in range(3):
            best = max(range(pivot, 3), key=lambda row: abs(matrix[row][pivot]))
            if abs(matrix[best][pivot]) < 1e-9:
                return None
            if best != pivot:
                matrix[pivot], matrix[best] = matrix[best], matrix[pivot]
            scale = matrix[pivot][pivot]
            for col in range(pivot, 4):
                matrix[pivot][col] /= scale
            for row in range(3):
                if row == pivot:
                    continue
                factor = matrix[row][pivot]
                for col in range(pivot, 4):
                    matrix[row][col] -= factor * matrix[pivot][col]
        return matrix[0][3], matrix[1][3], matrix[2][3]

    def _fit_learned_circle(self):
        points = list(self.observations)[-self.observation_buffer_size:]
        if len(points) < self.min_circle_fit_points:
            return None, "need {} observations, have {}".format(
                self.min_circle_fit_points, len(points)
            )

        xs = [sample[1][0] for sample in points]
        ys = [sample[1][1] for sample in points]
        n = float(len(points))
        sum_x = sum(xs)
        sum_y = sum(ys)
        sum_xx = sum(x * x for x in xs)
        sum_yy = sum(y * y for y in ys)
        sum_xy = sum(x * y for x, y in zip(xs, ys))
        radii_sq = [x * x + y * y for x, y in zip(xs, ys)]

        lhs = [
            [sum_xx, sum_xy, sum_x],
            [sum_xy, sum_yy, sum_y],
            [sum_x, sum_y, n],
        ]
        rhs = [
            -sum(x * r2 for x, r2 in zip(xs, radii_sq)),
            -sum(y * r2 for y, r2 in zip(ys, radii_sq)),
            -sum(radii_sq),
        ]
        solution = self._solve_3x3(lhs, rhs)
        if solution is None:
            return None, "circle fit matrix is singular"

        d, e, f = solution
        cx = -0.5 * d
        cy = -0.5 * e
        radius_sq = cx * cx + cy * cy - f
        if radius_sq <= 0.0:
            return None, "circle fit produced non-positive radius"
        radius = math.sqrt(radius_sq)
        if radius < self.learned_circle_min_radius or radius > self.learned_circle_max_radius:
            return None, "learned radius {:.2f} outside [{:.2f}, {:.2f}]".format(
                radius, self.learned_circle_min_radius, self.learned_circle_max_radius
            )

        residual = math.sqrt(
            sum(
                (math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy)) - radius) ** 2
                for x, y in zip(xs, ys)
            )
            / n
        )
        if residual > self.circle_fit_max_residual:
            return None, "circle residual {:.2f} above max {:.2f}".format(
                residual, self.circle_fit_max_residual
            )

        times = [sample[0] for sample in points]
        duration = times[-1] - times[0]
        if duration <= 0.1:
            return None, "observation duration too short"

        angles = []
        previous = None
        offset = 0.0
        for x, y in zip(xs, ys):
            theta = math.atan2(y - cy, x - cx)
            if previous is not None:
                delta = theta + offset - previous
                if delta > math.pi:
                    offset -= 2.0 * math.pi
                elif delta < -math.pi:
                    offset += 2.0 * math.pi
            unwrapped = theta + offset
            angles.append(unwrapped)
            previous = unwrapped

        t_mean = sum(times) / n
        theta_mean = sum(angles) / n
        denom = sum((t - t_mean) * (t - t_mean) for t in times)
        if denom <= 1e-6:
            return None, "observation timestamps are degenerate"
        omega = sum((t - t_mean) * (theta - theta_mean) for t, theta in zip(times, angles)) / denom
        if abs(omega) < 1e-4:
            return None, "learned angular speed too small"

        return {
            "center": (cx, cy, points[-1][1][2]),
            "radius": radius,
            "omega": omega,
            "residual": residual,
            "points": len(points),
        }, ""

    def _arrival_time(self, chaser_pos, target_pos):
        distance_xy = self._norm_xy(
            target_pos[0] - chaser_pos[0], target_pos[1] - chaser_pos[1]
        )
        distance_z = abs(target_pos[2] - chaser_pos[2])
        t_xy = distance_xy / self.assumed_chaser_speed_xy
        t_z = distance_z / self.assumed_chaser_speed_z
        return self._clamp(max(t_xy, t_z), self.min_prediction_time, self.max_prediction_time)

    def _solve_intercept(self, base_pos, future_fn):
        if self.chaser_common is None:
            return base_pos, 0.0
        t_go = self._arrival_time(self.chaser_common, base_pos)
        future = base_pos
        for _ in range(self.intercept_iterations):
            future = future_fn(t_go)
            t_go = self._arrival_time(self.chaser_common, future)
        return future_fn(t_go), t_go

    def _current_prediction(self):
        pos = self.observed_pos
        vel = self.raw_velocity
        return pos, vel, pos, 0.0, "current", ""

    def _linear_prediction(self):
        pos = self.observed_pos
        vel = self.raw_velocity
        future_fn = lambda t: (
            pos[0] + vel[0] * t,
            pos[1] + vel[1] * t,
            pos[2] + vel[2] * t,
        )
        pred, t_go = self._solve_intercept(pos, future_fn)
        return pos, vel, pred, t_go, "linear_observation", ""

    def _kalman_cv_prediction(self):
        # Formal model: state = [px, py, pz, vx, vy, vz],
        # p_pred = p_est + v_est * t_go.
        if self.cv_x is None:
            return self._linear_prediction()
        pos = (self.cv_x[0], self.cv_x[1], self.cv_x[2])
        vel = (self.cv_x[3], self.cv_x[4], self.cv_x[5])
        future_fn = lambda t: (
            pos[0] + vel[0] * t,
            pos[1] + vel[1] * t,
            pos[2] + vel[2] * t,
        )
        pred, t_go = self._solve_intercept(pos, future_fn)
        return pos, vel, pred, t_go, "kalman_cv", ""

    def _kalman_ca_prediction(self):
        # Formal model: state = [px, py, pz, vx, vy, vz, ax, ay, az],
        # p_pred = p_est + v_est * t_go + 0.5 * a_est * t_go^2.
        if self.ca_x is None:
            pos, vel, pred, t_go, _, reason = self._kalman_cv_prediction()
            return pos, vel, pred, t_go, "kalman_cv", reason
        pos = (self.ca_x[0], self.ca_x[1], self.ca_x[2])
        vel = (self.ca_x[3], self.ca_x[4], self.ca_x[5])
        acc = (self.ca_x[6], self.ca_x[7], self.ca_x[8])

        def future_fn(t):
            h = self.ca_accel_prediction_horizon
            if t <= h or h <= 0.0:
                return (
                    pos[0] + vel[0] * t + 0.5 * acc[0] * t * t,
                    pos[1] + vel[1] * t + 0.5 * acc[1] * t * t,
                    pos[2] + vel[2] * t + 0.5 * acc[2] * t * t,
                )
            return (
                pos[0] + vel[0] * t + acc[0] * h * (t - 0.5 * h),
                pos[1] + vel[1] * t + acc[1] * h * (t - 0.5 * h),
                pos[2] + vel[2] * t + acc[2] * h * (t - 0.5 * h),
            )

        pred, t_go = self._solve_intercept(pos, future_fn)
        return pos, vel, pred, t_go, "kalman_ca", ""

    def _learned_circle_prediction(self):
        fit, reason = self._fit_learned_circle()
        self.last_circle_fit = fit
        if fit is None:
            pos, vel, pred, t_go, _, _ = self._kalman_cv_prediction()
            return pos, vel, pred, t_go, "kalman_cv", "learned_circle fallback: " + reason

        if self.cv_x is not None:
            pos = (self.cv_x[0], self.cv_x[1], self.cv_x[2])
            vel = (self.cv_x[3], self.cv_x[4], self.cv_x[5])
        else:
            pos = self.observed_pos
            vel = self.raw_velocity
        center = fit["center"]
        radius = fit["radius"]
        omega = fit["omega"]
        theta_now = math.atan2(pos[1] - center[1], pos[0] - center[0])
        future_fn = lambda t: (
            center[0] + radius * math.cos(theta_now + omega * t),
            center[1] + radius * math.sin(theta_now + omega * t),
            pos[2] + vel[2] * t,
        )
        pred, t_go = self._solve_intercept(pos, future_fn)
        tangent_vel = (
            -radius * omega * math.sin(theta_now),
            radius * omega * math.cos(theta_now),
            vel[2],
        )
        return pos, tangent_vel, pred, t_go, "learned_circle", ""

    def _circle_configured_debug_prediction(self):
        pos = self.observed_pos
        vel = self.raw_velocity
        center = self._target_center_common(pos)
        theta = math.atan2(pos[1] - center[1], pos[0] - center[0])
        xy_speed = self._norm_xy(vel[0], vel[1])
        target_speed = self.target_speed if self.target_speed > 1e-6 else xy_speed
        direction = 1.0 if self.target_direction >= 0.0 else -1.0
        if self.infer_target_direction and xy_speed > 1e-3:
            rx = pos[0] - center[0]
            ry = pos[1] - center[1]
            cross_z = rx * vel[1] - ry * vel[0]
            if abs(cross_z) > 1e-4:
                direction = 1.0 if cross_z > 0.0 else -1.0
        omega = direction * target_speed / self.target_radius
        future_fn = lambda t: (
            center[0] + self.target_radius * math.cos(theta + omega * t),
            center[1] + self.target_radius * math.sin(theta + omega * t),
            center[2],
        )
        pred, t_go = self._solve_intercept(pos, future_fn)
        return pos, vel, pred, t_go, "circle_configured_debug", ""

    def _build_prediction(self):
        self.last_circle_fit = None
        self.fallback_reason = ""
        if self.observed_pos is None:
            return None

        model = self.estimator_model
        if model == "current":
            rospy.logwarn_throttle(
                10.0,
                "[TargetStateEstimator] current is a legacy/debug baseline, not a formal predictor. Formal models: %s",
                ",".join(self.FORMAL_MODELS),
            )
            result = self._current_prediction()
        elif model in ("linear", "linear_observation"):
            rospy.logwarn_throttle(
                10.0,
                "[TargetStateEstimator] linear_observation is a legacy/debug baseline, not a formal predictor. Formal models: %s",
                ",".join(self.FORMAL_MODELS),
            )
            result = self._linear_prediction()
        elif model == "kalman_cv":
            result = self._kalman_cv_prediction()
        elif model == "kalman_ca":
            result = self._kalman_ca_prediction()
        elif model == "learned_circle":
            rospy.logwarn_throttle(
                10.0,
                "[TargetStateEstimator] learned_circle is a legacy periodic baseline, not part of the formal CV/CA evaluation.",
            )
            result = self._learned_circle_prediction()
        elif model in ("circle_configured_debug", "circle_configured"):
            rospy.logwarn_throttle(
                10.0,
                "[TargetStateEstimator] circle_configured_debug uses target trajectory parameters and must not be used as a formal predictor.",
            )
            result = self._circle_configured_debug_prediction()
        else:
            rospy.logwarn_throttle(
                5.0,
                "[TargetStateEstimator] unknown estimator_model '%s', using kalman_cv",
                model,
            )
            result = self._kalman_cv_prediction()

        self.active_model = result[4]
        self.fallback_reason = result[5]
        return result

    def _predict_only_step(self, now):
        if self.observed_pos is None:
            return
        if self.cv_x is not None:
            self._update_kalman_cv(self.observed_pos, now, update=False)
        if self.ca_x is not None:
            self._update_kalman_ca(self.observed_pos, now, update=False)

    def _publish_state(self, stamp, pos, vel):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = "target_estimator"
        msg.pose.pose.position.x = pos[0]
        msg.pose.pose.position.y = pos[1]
        msg.pose.pose.position.z = pos[2]
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = vel[0]
        msg.twist.twist.linear.y = vel[1]
        msg.twist.twist.linear.z = vel[2]
        self.state_pub.publish(msg)

    def _publish_intercept(self, stamp, pred):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = pred[0]
        msg.pose.position.y = pred[1]
        msg.pose.position.z = pred[2]
        msg.pose.orientation.w = 1.0
        self.intercept_pub.publish(msg)
        self.has_intercept_point = True

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
        marker.lifetime = rospy.Duration(1.0)
        return marker

    def _line_marker(self, header, marker_id, ns, points, color, width):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for point in points:
            marker.points.append(Point(x=point[0], y=point[1], z=point[2]))
        marker.lifetime = rospy.Duration(1.0)
        return marker

    def _text_marker(self, header, marker_id, ns, pos, text):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2] + 0.7
        marker.scale.z = 0.32
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = text
        marker.lifetime = rospy.Duration(1.0)
        return marker

    def _publish_markers(self, stamp, observed, est_pos, est_vel, pred, t_go):
        header = Header(stamp=stamp, frame_id=self.frame_id)
        markers = MarkerArray()
        markers.markers.append(
            self._sphere_marker(
                header, 0, "target_estimator_observed", observed, (0.0, 0.4, 1.0, 0.85), self.marker_scale * 0.75
            )
        )
        markers.markers.append(
            self._sphere_marker(
                header, 1, "target_estimator_state", est_pos, (0.0, 1.0, 1.0, 0.85), self.marker_scale * 0.75
            )
        )
        markers.markers.append(
            self._sphere_marker(
                header, 2, "target_estimator_intercept", pred, (1.0, 0.05, 0.05, 0.95), self.marker_scale
            )
        )
        if self.chaser_common is not None:
            markers.markers.append(
                self._line_marker(
                    header, 3, "target_estimator_uav0_to_intercept", [self.chaser_common, pred], (0.2, 1.0, 0.2, 0.85), 0.05
                )
            )
        markers.markers.append(
            self._line_marker(
                header, 4, "target_estimator_target_to_intercept", [observed, pred], (1.0, 0.65, 0.0, 0.85), 0.04
            )
        )
        if self.chaser_common is not None:
            dx = observed[0] - self.chaser_common[0]
            dy = observed[1] - self.chaser_common[1]
            dz = observed[2] - self.chaser_common[2]
            distance_xy = self._norm_xy(dx, dy)
            distance_z = abs(dz)
            distance_3d = self._norm3(dx, dy, dz)
        else:
            distance_xy = float("nan")
            distance_z = float("nan")
            distance_3d = float("nan")
        text = (
            "estimator_model={}\nobservation_source={}\nstatus={}\nobs_age={:.2f}\nconfidence={:.2f}\nt_go={:.2f}\nv_est=({:.2f} {:.2f} {:.2f})\n"
            "time_since_last_valid={:.2f}\ncovariance_trace={:.2f}\ndistance_xy={:.2f}\ndistance_z={:.2f}\ndistance_3d={:.2f}"
        ).format(
            self.active_model,
            self.observation_source,
            self.tracking_state,
            self._observation_age(header.stamp),
            self.latest_observation_confidence,
            t_go,
            est_vel[0],
            est_vel[1],
            est_vel[2],
            self._observation_age(header.stamp),
            self._covariance_trace(),
            distance_xy,
            distance_z,
            distance_3d,
        )
        if self.fallback_reason:
            text += "\nfallback={}".format(self.fallback_reason[:60])
        markers.markers.append(
            self._text_marker(header, 5, "target_estimator_text", pred, text)
        )
        self.marker_pub.publish(markers)

    def publish_estimate(self):
        stamp = rospy.Time.now()
        self.tracking_state = self._tracking_state(stamp)
        age = self._observation_age(stamp)
        covariance_trace = self._covariance_trace()
        if (
            self.max_covariance_trace > 0.0
            and math.isfinite(covariance_trace)
            and covariance_trace > self.max_covariance_trace
        ):
            self.tracking_state = "LOST"
            self.latest_update_rejected_reason = "covariance_trace_exceeded"
        self.tracking_state_pub.publish(String(data=self.tracking_state))
        self.observation_age_pub.publish(
            Float32(data=float(age) if math.isfinite(age) else float("nan"))
        )
        self.observation_confidence_pub.publish(
            Float32(data=float(self.latest_observation_confidence))
        )
        self.rejected_observation_count_pub.publish(
            UInt32(data=max(int(self.rejected_observation_count), 0))
        )
        if self.tracking_state == "PREDICT_ONLY":
            self._predict_only_step(stamp)
        elif self.tracking_state == "LOST" and self.observed_pos is not None:
            if self.freeze_prediction_after_lost:
                rospy.logwarn_throttle(
                    1.0,
                    "[TargetStateEstimator] LOST: freezing Kalman prediction after %.2fs without valid observation; covariance_trace=%.2f",
                    age,
                    covariance_trace,
                )
            elif age <= self.target_lost_timeout:
                self._predict_only_step(stamp)
            elif not self.predict_only_publish:
                rospy.logwarn_throttle(
                    1.0,
                    "[TargetStateEstimator] target_lost: observation age %.2fs exceeds target_lost_timeout %.2fs; suppressing intercept output",
                    age,
                    self.target_lost_timeout,
                )
                return

        prediction = self._build_prediction()
        if prediction is None:
            rospy.loginfo_throttle(
                1.0,
                "[TargetStateEstimator] waiting for target observation received_target_pose=%s observation_valid=%s observation_confidence=%.2f update_allowed=%s update_rejected_reason=%s tracking_state=%s time_since_last_valid=%.2f has_estimator_state=%s has_intercept_point=%s rejected_observation_count=%d",
                self.received_target_pose,
                self.latest_observation_valid,
                self.latest_observation_confidence,
                self.latest_update_allowed,
                self.latest_update_rejected_reason,
                self.tracking_state,
                age,
                False,
                self.has_intercept_point,
                self.rejected_observation_count,
            )
            return
        if self.tracking_state != "TRACKING":
            rospy.logwarn_throttle(
                1.0,
                "[TargetStateEstimator] %s: no valid target_pose update for %.2fs; publishing prediction output",
                self.tracking_state,
                age,
            )
            if age > self.max_prediction_only_age and not self.predict_only_publish:
                rospy.logwarn_throttle(
                    1.0,
                    "[TargetStateEstimator] target_lost: observation age %.2fs exceeds max_prediction_only_age %.2fs; suppressing stale intercept output",
                    age,
                    self.max_prediction_only_age,
                )
                return
        if self.chaser_common is None:
            rospy.loginfo_throttle(3.0, "[TargetStateEstimator] waiting for chaser odom")

        est_pos, est_vel, pred, t_go, _, _ = prediction
        self._publish_state(stamp, est_pos, est_vel)
        self._publish_intercept(stamp, pred)
        self.t_go_pub.publish(Float32(data=float(t_go)))
        self._publish_markers(stamp, self.observed_pos, est_pos, est_vel, pred, t_go)

        rospy.loginfo_throttle(
            1.0,
            "[TargetStateEstimator] model=%s active=%s received_target_pose=%s observation_valid=%s observation_confidence=%.2f update_allowed=%s update_rejected_reason=%s tracking_state=%s time_since_last_valid=%.2f has_estimator_state=%s has_intercept_point=%s rejected_observation_count=%d covariance_trace=%.2f obs=(%.2f %.2f %.2f) est=(%.2f %.2f %.2f) vel=(%.2f %.2f %.2f) pred=(%.2f %.2f %.2f) t_go=%.2f fallback='%s'",
            self.estimator_model,
            self.active_model,
            self.received_target_pose,
            self.latest_observation_valid,
            self.latest_observation_confidence,
            self.latest_update_allowed,
            self.latest_update_rejected_reason,
            self.tracking_state,
            age,
            prediction is not None,
            self.has_intercept_point,
            self.rejected_observation_count,
            self._covariance_trace(),
            self.observed_pos[0],
            self.observed_pos[1],
            self.observed_pos[2],
            est_pos[0],
            est_pos[1],
            est_pos[2],
            est_vel[0],
            est_vel[1],
            est_vel[2],
            pred[0],
            pred[1],
            pred[2],
            t_go,
            self.fallback_reason,
        )

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_estimate()
            rate.sleep()


def main():
    rospy.init_node("target_state_estimator_node")
    TargetStateEstimator().run()


if __name__ == "__main__":
    main()
