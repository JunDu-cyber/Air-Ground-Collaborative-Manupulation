#!/usr/bin/env python3
"""Chase a truth target with MAVROS velocity setpoints."""

import math
from collections import deque

import rospy
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32, Header, String
from visualization_msgs.msg import Marker


class Uav0VelocityGuidance:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.mavros_ns = rospy.get_param("~mavros_ns", "/uav0/mavros").rstrip("/")
        self.auto_arm = rospy.get_param("~auto_arm", False)
        self.auto_offboard = rospy.get_param("~auto_offboard", False)
        self.offboard_request_interval = max(
            rospy.get_param("~offboard_request_interval", 1.0), 0.1
        )
        self.chaser_odom_topic = rospy.get_param(
            "~chaser_odom_topic", "/uav0/mavros/local_position/odom"
        )
        self.target_odom_topic = rospy.get_param(
            "~target_odom_topic", "/uav1/mavros/local_position/odom"
        )
        self.target_input_mode = rospy.get_param("~target_input_mode", "estimator").lower()
        if self.target_input_mode not in ("estimator", "internal"):
            rospy.logwarn(
                "[Uav0VelocityGuidance] unknown target_input_mode '%s', using estimator",
                self.target_input_mode,
            )
            self.target_input_mode = "estimator"
        self.estimator_state_topic = rospy.get_param(
            "~estimator_state_topic", "/target_estimator/state"
        )
        self.estimator_intercept_topic = rospy.get_param(
            "~estimator_intercept_topic", "/target_estimator/intercept_point"
        )
        self.estimator_t_go_topic = rospy.get_param(
            "~estimator_t_go_topic", "/target_estimator/t_go"
        )
        self.estimator_tracking_state_topic = rospy.get_param(
            "~estimator_tracking_state_topic", "/target_estimator/tracking_state"
        )
        self.estimator_observation_age_topic = rospy.get_param(
            "~estimator_observation_age_topic", "/target_estimator/observation_age"
        )
        self.estimator_observation_confidence_topic = rospy.get_param(
            "~estimator_observation_confidence_topic",
            "/target_estimator/observation_confidence",
        )
        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic", "/uav0/mavros/setpoint_velocity/cmd_vel"
        )
        self.marker_topic = rospy.get_param("~marker_topic", "/uav0/guidance/marker")
        self.intercept_pose_topic = rospy.get_param(
            "~intercept_pose_topic", "/uav0/guidance/intercept_point"
        )
        self.position_setpoint_topic = rospy.get_param(
            "~position_setpoint_topic", "/uav0/mavros/setpoint_position/local"
        )
        self.capture_success_topic = rospy.get_param(
            "~capture_success_topic", "/uav0/capture/success"
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

        self.kp = rospy.get_param("~Kp", 0.6)
        self.kd = rospy.get_param("~Kd", 0.15)
        self.kp_z = rospy.get_param("~Kp_z", 0.8)
        self.kd_z = rospy.get_param("~Kd_z", 0.15)
        self.guidance_mode = rospy.get_param("~guidance_mode", "los").lower()
        self.guidance_dimension = rospy.get_param("~guidance_dimension", "split_3d").lower()
        if self.guidance_dimension not in ("xy_only", "split_3d", "los_3d"):
            rospy.logwarn(
                "[Uav0VelocityGuidance] unknown guidance_dimension '%s', using split_3d",
                self.guidance_dimension,
            )
            self.guidance_dimension = "split_3d"
        self.guidance_target = rospy.get_param("~guidance_target", "intercept").lower()
        self.feedforward_scale = min(
            max(rospy.get_param("~feedforward_scale", 0.65), 0.0), 1.0
        )
        self.min_closing_speed = max(rospy.get_param("~min_closing_speed", 0.25), 0.0)
        self.max_closing_speed = max(
            rospy.get_param("~max_closing_speed", 1.4), self.min_closing_speed
        )
        self.capture_closing_speed = max(
            rospy.get_param("~capture_closing_speed", 0.35), 0.0
        )
        self.vxy_max = max(rospy.get_param("~vxy_max", 2.0), 0.01)
        self.vz_max = max(rospy.get_param("~vz_max", 0.5), 0.01)
        self.axy_max = max(rospy.get_param("~axy_max", 1.2), 0.01)
        self.az_max = max(rospy.get_param("~az_max", 0.4), 0.01)
        self.command_rate = max(rospy.get_param("~command_rate", 20.0), 1.0)
        self.estimator_input_timeout = max(
            rospy.get_param("~estimator_input_timeout", 1.0), 0.0
        )
        self.predict_only_speed_scale = self._clamp(
            rospy.get_param("~predict_only_speed_scale", 0.7), 0.0, 1.0
        )
        self.lost_target_behavior = rospy.get_param(
            "~lost_target_behavior", "hold"
        ).lower()
        if self.lost_target_behavior not in ("hold", "search_last", "continue_predict"):
            rospy.logwarn(
                "[Uav0VelocityGuidance] unknown lost_target_behavior '%s', using hold",
                self.lost_target_behavior,
            )
            self.lost_target_behavior = "hold"
        self.lost_target_speed_scale = self._clamp(
            rospy.get_param("~lost_target_speed_scale", 0.3), 0.0, 1.0
        )
        self.max_predict_only_guidance_time = max(
            rospy.get_param("~max_predict_only_guidance_time", 1.0), 0.0
        )
        self.lost_recovery_enable = rospy.get_param("~lost_recovery_enable", True)
        self.lost_recovery_timeout = max(
            rospy.get_param("~lost_recovery_timeout", 5.0), 0.0
        )
        self.lost_recovery_search_z = max(
            rospy.get_param("~lost_recovery_search_z", 3.0), 0.0
        )
        self.lost_recovery_descent_speed = self._clamp(
            rospy.get_param("~lost_recovery_descent_speed", 0.8), 0.0, self.vz_max
        )
        self.capture_use_3d_distance = rospy.get_param("~capture_use_3d_distance", True)
        self.terminal_guidance_enable = rospy.get_param(
            "~terminal_guidance_enable", False
        )
        self.terminal_switch_distance = max(
            rospy.get_param("~terminal_switch_distance", 1.5), 0.0
        )
        self.terminal_max_observation_age = max(
            rospy.get_param("~terminal_max_observation_age", 0.8), 0.0
        )
        self.terminal_prediction_time = max(
            rospy.get_param("~terminal_prediction_time", 0.15), 0.0
        )
        self.terminal_vz_max = max(rospy.get_param("~terminal_vz_max", 0.35), 0.01)
        self.terminal_z_deadband = max(
            rospy.get_param("~terminal_z_deadband", 0.10), 0.0
        )
        self.guidance_z_limit_enable = rospy.get_param(
            "~guidance_z_limit_enable", False
        )
        self.guidance_z_min = rospy.get_param("~guidance_z_min", 0.5)
        self.guidance_z_max = rospy.get_param("~guidance_z_max", 5.5)
        if self.guidance_z_max < self.guidance_z_min:
            self.guidance_z_min, self.guidance_z_max = (
                self.guidance_z_max,
                self.guidance_z_min,
            )
        self.altitude_guard_enable = rospy.get_param(
            "~altitude_guard_enable", False
        )
        self.altitude_guard_min_z = rospy.get_param("~altitude_guard_min_z", 0.5)
        self.altitude_guard_max_z = rospy.get_param("~altitude_guard_max_z", 5.5)
        if self.altitude_guard_max_z < self.altitude_guard_min_z:
            self.altitude_guard_min_z, self.altitude_guard_max_z = (
                self.altitude_guard_max_z,
                self.altitude_guard_min_z,
            )
        self.altitude_guard_gain = max(
            rospy.get_param("~altitude_guard_gain", 0.8), 0.0
        )
        self.altitude_guard_max_correction = max(
            rospy.get_param("~altitude_guard_max_correction", 0.6), 0.0
        )

        self.yaw_mode = rospy.get_param("~yaw_mode", "face_target").lower()
        self.yaw_rate_max = max(rospy.get_param("~yaw_rate_max", 0.6), 0.0)
        self.kyaw = rospy.get_param("~Kyaw", 1.0)
        self.face_guidance_before_move = rospy.get_param(
            "~face_guidance_before_move", True
        )
        self.yaw_gate_xy_scale = self._clamp(
            rospy.get_param("~yaw_gate_xy_scale", 0.4), 0.0, 1.0
        )
        self.yaw_align_threshold = max(
            rospy.get_param("~yaw_align_threshold", 0.25), 0.0
        )
        self.yaw_align_hold_time = max(
            rospy.get_param("~yaw_align_hold_time", 0.3), 0.0
        )
        self.yaw_align_min_distance = max(
            rospy.get_param("~yaw_align_min_distance", 0.5), 0.0
        )
        self.allow_vertical_during_yaw_align = rospy.get_param(
            "~allow_vertical_during_yaw_align", True
        )
        self.use_yaw_reacquire = rospy.get_param("~use_yaw_reacquire", False)
        self.reacquire_state_topic = rospy.get_param(
            "~reacquire_state_topic", "/target_reacquire/state"
        )
        self.reacquire_yaw_target_topic = rospy.get_param(
            "~reacquire_yaw_target_topic", "/target_reacquire/yaw_target"
        )
        self.reacquire_yaw_target_max_age = max(
            rospy.get_param("~reacquire_yaw_target_max_age", 0.5), 0.0
        )

        self.capture_slowdown_distance = max(
            rospy.get_param("~capture_slowdown_distance", 1.5), 0.01
        )
        self.stop_distance = max(rospy.get_param("~stop_distance", 0.5), 0.0)
        if self.capture_slowdown_distance <= self.stop_distance:
            rospy.logwarn(
                "[Uav0VelocityGuidance] capture_slowdown_distance %.2f <= stop_distance %.2f; increasing slowdown distance to keep a nonzero slowdown band.",
                self.capture_slowdown_distance,
                self.stop_distance,
            )
            self.capture_slowdown_distance = self.stop_distance + 0.5
        self.stop_behavior = rospy.get_param("~stop_behavior", "target_velocity").lower()
        self.near_target_velocity_scale = min(
            max(rospy.get_param("~near_target_velocity_scale", 0.6), 0.0), 1.0
        )
        self.velocity_source = rospy.get_param("~velocity_source", "diff").lower()
        self.min_twist_speed = rospy.get_param("~min_twist_speed", 0.03)
        self.stop_on_capture_success = rospy.get_param("~stop_on_capture_success", True)
        self.assumed_chaser_speed_xy = max(
            rospy.get_param("~assumed_chaser_speed_xy", 1.2), 0.05
        )
        self.assumed_chaser_speed_z = max(
            rospy.get_param("~assumed_chaser_speed_z", 0.5), 0.05
        )
        self.assumed_chaser_speed = max(
            rospy.get_param("~assumed_chaser_speed", self.assumed_chaser_speed_xy), 0.05
        )
        self.min_prediction_time = max(
            rospy.get_param("~min_prediction_time", 0.2), 0.0
        )
        self.max_prediction_time = max(
            rospy.get_param("~max_prediction_time", 2.5), self.min_prediction_time
        )
        self.intercept_iterations = max(
            int(rospy.get_param("~intercept_iterations", 5)), 1
        )
        self.intercept_scan_steps = max(
            int(rospy.get_param("~intercept_scan_steps", 80)), 8
        )
        self.target_motion_mode = rospy.get_param("~target_motion_mode", "circle").lower()
        self.requested_prediction_model = rospy.get_param(
            "~prediction_model", "linear_observation"
        ).lower()
        self.prediction_model = self._resolve_prediction_model(
            self.requested_prediction_model
        )
        self.active_prediction_model = self.prediction_model
        self.learned_circle_enable = rospy.get_param("~learned_circle_enable", False)
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
        self.target_center_x = rospy.get_param("~target_center_x", 2.0)
        self.target_center_y = rospy.get_param("~target_center_y", 0.0)
        self.target_radius = max(rospy.get_param("~target_radius", 3.0), 0.05)
        self.target_speed = max(rospy.get_param("~target_speed", 0.6), 0.0)
        self.use_measured_target_speed = rospy.get_param(
            "~use_measured_target_speed", True
        )
        self.target_direction = rospy.get_param("~target_direction", 1.0)
        self.infer_target_direction = rospy.get_param("~infer_target_direction", True)
        self.target_height = rospy.get_param("~target_height", 0.0)
        self.target_center_frame = rospy.get_param(
            "~target_center_frame", "uav1_local"
        ).lower()
        if self.target_center_frame not in ("uav1_local", "map"):
            rospy.logwarn(
                "[Uav0VelocityGuidance] unknown target_center_frame '%s', using uav1_local",
                self.target_center_frame,
            )
            self.target_center_frame = "uav1_local"
        self.marker_scale = rospy.get_param("~marker_scale", 0.45)
        self.circle_projection_warn_distance = max(
            rospy.get_param("~circle_projection_warn_distance", 0.5), 0.0
        )

        self.chaser_odom = None
        self.target_odom = None
        self.estimator_state = None
        self.estimator_intercept_pos = None
        self.estimator_t_go = None
        self.estimator_state_stamp = None
        self.estimator_intercept_stamp = None
        self.estimator_t_go_stamp = None
        self.chaser_velocity = (0.0, 0.0, 0.0)
        self.target_velocity = (0.0, 0.0, 0.0)
        self.last_chaser_stamp = None
        self.last_target_stamp = None
        self.last_chaser_pos = None
        self.last_target_pos = None
        self.last_cmd = [0.0, 0.0, 0.0]
        self.last_command_time = None
        self.yaw_aligned_since = None
        self.position_setpoint_count = 0
        self.capture_success = False
        self.estimator_tracking_state = "TRACKING"
        self.estimator_observation_age = 0.0
        self.estimator_observation_confidence = 1.0
        self.reacquire_state = "VISUAL_TRACK"
        self.reacquire_yaw_target = None
        self.reacquire_yaw_target_stamp = None
        self.target_observations = deque(maxlen=self.observation_buffer_size)
        self.last_circle_fit = None
        self.learned_circle_in_use = False
        self.prediction_fallback_reason = ""
        self.mavros_state = State()
        self.last_offboard_request_time = None
        self.last_zero_velocity_reason = "not_commanded_yet"

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, TwistStamped, queue_size=10)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)
        self.intercept_pose_pub = rospy.Publisher(
            self.intercept_pose_topic, PoseStamped, queue_size=10
        )
        # RViz path visualization
        self.path_pub = rospy.Publisher(
            self.mavros_ns + "/guidance/path", Path, queue_size=2
        )
        self.intercept_line_pub = rospy.Publisher(
            self.mavros_ns + "/guidance/intercept_line", Path, queue_size=2
        )
        self._path_msg = Path()
        self._path_msg.header.frame_id = "map"
        self._path_max_len = 600  # 30s at 20Hz
        self.state_sub = rospy.Subscriber(
            self.mavros_ns + "/state", State, self.state_cb, queue_size=10
        )
        self.set_mode_srv = rospy.ServiceProxy(self.mavros_ns + "/set_mode", SetMode)
        self.arming_srv = rospy.ServiceProxy(self.mavros_ns + "/cmd/arming", CommandBool)
        self.chaser_sub = rospy.Subscriber(
            self.chaser_odom_topic, Odometry, self.chaser_odom_cb, queue_size=10
        )
        if self.target_input_mode == "internal":
            self.target_sub = rospy.Subscriber(
                self.target_odom_topic, Odometry, self.target_odom_cb, queue_size=10
            )
        else:
            self.estimator_state_sub = rospy.Subscriber(
                self.estimator_state_topic,
                Odometry,
                self.estimator_state_cb,
                queue_size=10,
            )
            self.estimator_intercept_sub = rospy.Subscriber(
                self.estimator_intercept_topic,
                PoseStamped,
                self.estimator_intercept_cb,
                queue_size=10,
            )
            self.estimator_t_go_sub = rospy.Subscriber(
                self.estimator_t_go_topic,
                Float32,
                self.estimator_t_go_cb,
                queue_size=10,
            )
            self.estimator_tracking_state_sub = rospy.Subscriber(
                self.estimator_tracking_state_topic,
                String,
                self.estimator_tracking_state_cb,
                queue_size=10,
            )
            self.estimator_observation_age_sub = rospy.Subscriber(
                self.estimator_observation_age_topic,
                Float32,
                self.estimator_observation_age_cb,
                queue_size=10,
            )
            self.estimator_observation_confidence_sub = rospy.Subscriber(
                self.estimator_observation_confidence_topic,
                Float32,
                self.estimator_observation_confidence_cb,
                queue_size=10,
            )
        self.position_setpoint_sub = rospy.Subscriber(
            self.position_setpoint_topic,
            PoseStamped,
            self.position_setpoint_cb,
            queue_size=10,
        )
        self.capture_success_sub = rospy.Subscriber(
            self.capture_success_topic,
            Bool,
            self.capture_success_cb,
            queue_size=10,
        )
        if self.use_yaw_reacquire:
            self.reacquire_state_sub = rospy.Subscriber(
                self.reacquire_state_topic,
                String,
                self.reacquire_state_cb,
                queue_size=10,
            )
            self.reacquire_yaw_target_sub = rospy.Subscriber(
                self.reacquire_yaw_target_topic,
                Float32,
                self.reacquire_yaw_target_cb,
                queue_size=10,
            )

        rospy.loginfo(
            "[Uav0VelocityGuidance] mavros_ns=%s auto_offboard=%s auto_arm=%s target_input_mode=%s chaser=%s target=%s estimator_state=%s estimator_intercept=%s estimator_t_go=%s cmd=%s marker=%s intercept_pose=%s position_setpoint_watch=%s capture_success=%s stop_on_capture=%s offsets=%s uav0_offset=(%.2f %.2f %.2f) uav1_offset=(%.2f %.2f %.2f) target_center_frame=%s mode=%s dimension=%s guidance_target=%s requested_model=%s resolved_model=%s target_motion=%s learned_circle_enable=%s buffer=%d min_fit=%d max_residual=%.2f radius_range=[%.2f %.2f] Kp=%.2f Kd=%.2f Kp_z=%.2f Kd_z=%.2f ff=%.2f close=[%.2f %.2f] capture_close=%.2f vxy_max=%.2f vz_max=%.2f axy_max=%.2f az_max=%.2f rate=%.1f yaw_mode=%s face_before_move=%s yaw_gate_xy_scale=%.2f yaw_align=%.2f hold=%.2f min_dist=%.2f allow_vertical_during_yaw=%s use_yaw_reacquire=%s reacquire_state=%s reacquire_yaw=%s stop=%.2f slowdown=%.2f stop_behavior=%s velocity_source=%s assumed_speed_xy=%.2f assumed_speed_z=%.2f pred=[%.2f %.2f] capture_use_3d=%s terminal=%s terminal_dist=%.2f terminal_obs_age=%.2f terminal_t=%.2f terminal_vz=%.2f z_limit=%s[%.2f %.2f] altitude_guard=%s[%.2f %.2f]",
            self.mavros_ns,
            self.auto_offboard,
            self.auto_arm,
            self.target_input_mode,
            self.chaser_odom_topic,
            self.target_odom_topic,
            self.estimator_state_topic,
            self.estimator_intercept_topic,
            self.estimator_t_go_topic,
            self.cmd_vel_topic,
            self.marker_topic,
            self.intercept_pose_topic,
            self.position_setpoint_topic,
            self.capture_success_topic,
            self.stop_on_capture_success,
            self.use_spawn_offsets,
            self.uav0_spawn_offset[0],
            self.uav0_spawn_offset[1],
            self.uav0_spawn_offset[2],
            self.uav1_spawn_offset[0],
            self.uav1_spawn_offset[1],
            self.uav1_spawn_offset[2],
            self.target_center_frame,
            self.guidance_mode,
            self.guidance_dimension,
            self.guidance_target,
            self.requested_prediction_model,
            self.prediction_model,
            self.target_motion_mode,
            self.learned_circle_enable,
            self.observation_buffer_size,
            self.min_circle_fit_points,
            self.circle_fit_max_residual,
            self.learned_circle_min_radius,
            self.learned_circle_max_radius,
            self.kp,
            self.kd,
            self.kp_z,
            self.kd_z,
            self.feedforward_scale,
            self.min_closing_speed,
            self.max_closing_speed,
            self.capture_closing_speed,
            self.vxy_max,
            self.vz_max,
            self.axy_max,
            self.az_max,
            self.command_rate,
            self.yaw_mode,
            self.face_guidance_before_move,
            self.yaw_gate_xy_scale,
            self.yaw_align_threshold,
            self.yaw_align_hold_time,
            self.yaw_align_min_distance,
            self.allow_vertical_during_yaw_align,
            self.use_yaw_reacquire,
            self.reacquire_state_topic,
            self.reacquire_yaw_target_topic,
            self.stop_distance,
            self.capture_slowdown_distance,
            self.stop_behavior,
            self.velocity_source,
            self.assumed_chaser_speed_xy,
            self.assumed_chaser_speed_z,
            self.min_prediction_time,
            self.max_prediction_time,
            self.capture_use_3d_distance,
            self.terminal_guidance_enable,
            self.terminal_switch_distance,
            self.terminal_max_observation_age,
            self.terminal_prediction_time,
            self.terminal_vz_max,
            self.guidance_z_limit_enable,
            self.guidance_z_min,
            self.guidance_z_max,
            self.altitude_guard_enable,
            self.altitude_guard_min_z,
            self.altitude_guard_max_z,
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
    def _resolve_prediction_model(prediction_model):
        if prediction_model in ("auto", "target"):
            return "auto"
        if prediction_model in ("linear", "linear_observation"):
            return "linear_observation"
        if prediction_model in ("circle", "configured", "circle_configured"):
            return "circle_configured"
        return prediction_model

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
            z = (
                self.target_height
                + (
                    self.uav1_spawn_offset[2]
                    if self.use_spawn_offsets
                    and self.target_center_frame == "uav1_local"
                    else 0.0
                )
            )
        elif target_pos is not None:
            z = target_pos[2]
        else:
            z = base[2]
        return base[0], base[1], z

    @staticmethod
    def _quat_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle):
        if not math.isfinite(angle):
            return 0.0
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _pos(odom):
        p = odom.pose.pose.position
        return p.x, p.y, p.z

    def state_cb(self, msg):
        self.mavros_state = msg

    @staticmethod
    def _vel(odom):
        v = odom.twist.twist.linear
        return v.x, v.y, v.z

    def _estimate_velocity(self, msg, last_stamp, last_pos, spawn_offset):
        twist_vel = self._vel(msg)
        twist_speed = self._norm3(twist_vel[0], twist_vel[1], twist_vel[2])
        if self.velocity_source == "twist" and twist_speed >= self.min_twist_speed:
            return twist_vel

        if self.velocity_source == "auto" and twist_speed >= self.min_twist_speed:
            return twist_vel

        pos = self._to_common(self._pos(msg), spawn_offset)
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if last_stamp is None or last_pos is None:
            return twist_vel

        dt = (stamp - last_stamp).to_sec()
        if dt <= 1e-3:
            return twist_vel

        return (
            (pos[0] - last_pos[0]) / dt,
            (pos[1] - last_pos[1]) / dt,
            (pos[2] - last_pos[2]) / dt,
        )

    def chaser_odom_cb(self, msg):
        self.chaser_velocity = self._estimate_velocity(
            msg,
            self.last_chaser_stamp,
            self.last_chaser_pos,
            self.uav0_spawn_offset,
        )
        self.chaser_odom = msg
        self.last_chaser_stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )
        self.last_chaser_pos = self._to_common(self._pos(msg), self.uav0_spawn_offset)

    def target_odom_cb(self, msg):
        self.target_velocity = self._estimate_velocity(
            msg,
            self.last_target_stamp,
            self.last_target_pos,
            self.uav1_spawn_offset,
        )
        self.target_odom = msg
        self.last_target_stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )
        self.last_target_pos = self._to_common(self._pos(msg), self.uav1_spawn_offset)
        self.target_observations.append(
            (self.last_target_stamp.to_sec(), self.last_target_pos)
        )

    def estimator_state_cb(self, msg):
        self.estimator_state = msg
        self.target_velocity = self._vel(msg)
        self.estimator_state_stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )

    def estimator_intercept_cb(self, msg):
        p = msg.pose.position
        self.estimator_intercept_pos = (p.x, p.y, p.z)
        self.estimator_intercept_stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )

    def estimator_t_go_cb(self, msg):
        self.estimator_t_go = max(float(msg.data), 0.0)
        self.estimator_t_go_stamp = rospy.Time.now()

    def estimator_tracking_state_cb(self, msg):
        state = msg.data.strip().upper()
        if state in ("TRACKING", "PREDICT_ONLY", "LOST"):
            self.estimator_tracking_state = state

    def estimator_observation_age_cb(self, msg):
        self.estimator_observation_age = max(float(msg.data), 0.0)

    def estimator_observation_confidence_cb(self, msg):
        self.estimator_observation_confidence = self._clamp(float(msg.data), 0.0, 1.0)

    def reacquire_state_cb(self, msg):
        state = msg.data.strip().upper()
        if state:
            self.reacquire_state = state

    def reacquire_yaw_target_cb(self, msg):
        self.reacquire_yaw_target = float(msg.data)
        self.reacquire_yaw_target_stamp = rospy.Time.now()

    def capture_success_cb(self, msg):
        if bool(msg.data):
            self.capture_success = True
        elif not self.stop_on_capture_success:
            self.capture_success = False

    def position_setpoint_cb(self, _msg):
        self.position_setpoint_count += 1
        rospy.logerr_throttle(
            2.0,
            "[Uav0VelocityGuidance] detected position setpoints on %s while velocity guidance is active. Stop hover/position follower publishers for /uav0, otherwise PX4 may receive conflicting OFFBOARD commands.",
            self.position_setpoint_topic,
        )

    def _dt(self, now):
        if self.last_command_time is None:
            self.last_command_time = now
            return 1.0 / self.command_rate
        dt = (now - self.last_command_time).to_sec()
        self.last_command_time = now
        if dt <= 1e-3:
            return 1.0 / self.command_rate
        return min(dt, 0.25)
    
    def _slowdown_scales(self, distance):
        if distance <= self.stop_distance:
            return 0.0, 1.0

        if distance >= self.capture_slowdown_distance:
            return 1.0, 1.0

        span = max(self.capture_slowdown_distance - self.stop_distance, 1e-3)
        chase_scale = self._clamp((distance - self.stop_distance) / span, 0.0, 1.0)
        follow_scale = self.near_target_velocity_scale + (
            1.0 - self.near_target_velocity_scale
        ) * chase_scale
        return chase_scale, follow_scale

    def _clamp_prediction_time(self, t_go):
        return min(max(t_go, self.min_prediction_time), self.max_prediction_time)

    def _estimate_arrival_time(self, chaser_pos, target_pos):
        distance_xy = self._norm_xy(
            target_pos[0] - chaser_pos[0],
            target_pos[1] - chaser_pos[1],
        )
        distance_z = abs(target_pos[2] - chaser_pos[2])
        t_xy = distance_xy / self.assumed_chaser_speed_xy
        t_z = distance_z / self.assumed_chaser_speed_z
        return self._clamp_prediction_time(max(t_xy, t_z))

    def _target_direction_sign(self, target_pos, target_vel):
        direction = 1.0 if self.target_direction >= 0.0 else -1.0
        xy_speed = self._norm_xy(target_vel[0], target_vel[1])
        if self.infer_target_direction and xy_speed > 1e-3:
            center = self._target_center_common(target_pos)
            rx = target_pos[0] - center[0]
            ry = target_pos[1] - center[1]
            cross_z = rx * target_vel[1] - ry * target_vel[0]
            if abs(cross_z) > 1e-4:
                direction = 1.0 if cross_z > 0.0 else -1.0
        return direction

    def _target_speed_for_prediction(self, target_vel):
        measured_speed = self._norm_xy(target_vel[0], target_vel[1])
        if self.use_measured_target_speed and measured_speed > 1e-4:
            return measured_speed
        if self.target_speed > 1e-6:
            return self.target_speed
        return measured_speed

    def _linear_future_target_position(self, target_pos, target_vel, t_go):
        return (
            target_pos[0] + target_vel[0] * t_go,
            target_pos[1] + target_vel[1] * t_go,
            target_pos[2] + target_vel[2] * t_go,
        )

    def _configured_circle_future_position(self, target_pos, target_vel, t_go):
        center = self._target_center_common(target_pos)
        theta = math.atan2(
            target_pos[1] - center[1],
            target_pos[0] - center[0],
        )
        target_speed = self._target_speed_for_prediction(target_vel)
        direction = self._target_direction_sign(target_pos, target_vel)
        theta_future = theta + direction * (target_speed / self.target_radius) * t_go
        return (
            center[0] + self.target_radius * math.cos(theta_future),
            center[1] + self.target_radius * math.sin(theta_future),
            center[2],
        )

    def _configured_circle_point(self, theta, center):
        return (
            center[0] + self.target_radius * math.cos(theta),
            center[1] + self.target_radius * math.sin(theta),
            center[2],
        )

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
        points = list(self.target_observations)[-self.observation_buffer_size :]
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
        if radius < self.learned_circle_min_radius:
            return None, "learned radius {:.2f} below min {:.2f}".format(
                radius, self.learned_circle_min_radius
            )
        if radius > self.learned_circle_max_radius:
            return None, "learned radius {:.2f} above max {:.2f}".format(
                radius, self.learned_circle_max_radius
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
        omega = sum(
            (t - t_mean) * (theta - theta_mean)
            for t, theta in zip(times, angles)
        ) / denom
        if abs(omega) < 1e-4:
            return None, "learned angular speed too small"

        fit = {
            "center": (cx, cy, points[-1][1][2]),
            "radius": radius,
            "omega": omega,
            "residual": residual,
            "points": len(points),
        }
        return fit, ""

    def _update_prediction_state(self):
        self.learned_circle_in_use = False
        self.prediction_fallback_reason = ""
        self.last_circle_fit = None

        if self.prediction_model == "auto":
            if self.learned_circle_enable:
                fit, reason = self._fit_learned_circle()
                self.last_circle_fit = fit
                if fit is not None:
                    self.active_prediction_model = "learned_circle"
                    self.learned_circle_in_use = True
                    return
                self.prediction_fallback_reason = reason
            self.active_prediction_model = "linear_observation"
            return

        if self.prediction_model == "learned_circle":
            fit, reason = self._fit_learned_circle()
            self.last_circle_fit = fit
            if fit is not None:
                self.active_prediction_model = "learned_circle"
                self.learned_circle_in_use = True
                return
            self.active_prediction_model = "linear_observation"
            self.prediction_fallback_reason = reason
            return

        self.active_prediction_model = self.prediction_model

    def _configured_line_future_position(self, target_pos, target_vel, t_go):
        center = self._target_center_common(target_pos)
        x_min = center[0] - self.target_radius
        x_max = center[0] + self.target_radius
        segment_len = x_max - x_min
        if segment_len <= 1e-6:
            return center

        target_speed = self._target_speed_for_prediction(target_vel)
        if target_speed <= 1e-4:
            target_speed = self.target_speed
        if target_speed <= 1e-4:
            return self._linear_future_target_position(target_pos, target_vel, t_go)

        direction = 1.0 if self.target_direction >= 0.0 else -1.0
        if abs(target_vel[0]) > 1e-3:
            direction = 1.0 if target_vel[0] > 0.0 else -1.0

        s_now = self._clamp(target_pos[0] - x_min, 0.0, segment_len)
        phase = (s_now + direction * target_speed * t_go) % (2.0 * segment_len)
        if phase <= segment_len:
            x = x_min + phase
        else:
            x = x_max - (phase - segment_len)
        return x, center[1], center[2]

    def _warn_if_target_off_configured_circle(self, target_pos):
        if self.active_prediction_model != "circle_configured":
            return

        center = self._target_center_common(target_pos)
        radius_now = self._norm_xy(
            target_pos[0] - center[0],
            target_pos[1] - center[1],
        )
        error = abs(radius_now - self.target_radius)
        if error > self.circle_projection_warn_distance:
            rospy.logwarn_throttle(
                2.0,
                "[Uav0VelocityGuidance] /uav1 common truth is %.2fm away from the configured circle radius. Red intercept marker is constrained to configured/known common center=(%.2f %.2f), radius=%.2f; this is a debug mode, so check target_center/target_radius/target_center_frame/spawn offsets.",
                error,
                center[0],
                center[1],
                self.target_radius,
            )

    def _learned_circle_future_position(self, target_pos, target_vel, t_go):
        fit = self.last_circle_fit
        if fit is None:
            return self._linear_future_target_position(target_pos, target_vel, t_go)

        center = fit["center"]
        radius = fit["radius"]
        omega = fit["omega"]
        theta_now = math.atan2(target_pos[1] - center[1], target_pos[0] - center[0])
        theta_future = theta_now + omega * t_go
        return (
            center[0] + radius * math.cos(theta_future),
            center[1] + radius * math.sin(theta_future),
            target_pos[2],
        )

    def _truth_circle_future_position(self, target_pos, target_vel, t_go):
        speed = self._target_speed_for_prediction(target_vel)
        if speed <= 1e-4:
            return self._linear_future_target_position(target_pos, target_vel, t_go)

        direction = 1.0 if self.target_direction >= 0.0 else -1.0
        vx_unit = target_vel[0] / speed
        vy_unit = target_vel[1] / speed

        # Left normal of the measured velocity. For CCW motion the center is left of velocity.
        left_x = -vy_unit
        left_y = vx_unit
        center_x = target_pos[0] + direction * self.target_radius * left_x
        center_y = target_pos[1] + direction * self.target_radius * left_y

        theta = math.atan2(target_pos[1] - center_y, target_pos[0] - center_x)
        theta_future = theta + direction * (speed / self.target_radius) * t_go
        z = self._target_center_common(target_pos)[2]
        return (
            center_x + self.target_radius * math.cos(theta_future),
            center_y + self.target_radius * math.sin(theta_future),
            z,
        )

    def _future_target_position(self, target_pos, target_vel, t_go):
        model = self.active_prediction_model

        if model == "current":
            return target_pos

        if model == "learned_circle":
            return self._learned_circle_future_position(target_pos, target_vel, t_go)

        if model == "circle_truth":
            return self._truth_circle_future_position(target_pos, target_vel, t_go)

        if model == "circle_configured":
            return self._configured_circle_future_position(target_pos, target_vel, t_go)

        if model == "line":
            return self._configured_line_future_position(target_pos, target_vel, t_go)

        if model != "linear_observation":
            rospy.logwarn_throttle(
                5.0,
                "[Uav0VelocityGuidance] unknown prediction_model '%s', using linear_observation",
                model,
            )
        return self._linear_future_target_position(target_pos, target_vel, t_go)

    def _linear_intercept_time(self, chaser_pos, target_pos, target_vel):
        rx = target_pos[0] - chaser_pos[0]
        ry = target_pos[1] - chaser_pos[1]
        rz = target_pos[2] - chaser_pos[2]
        vx = target_vel[0]
        vy = target_vel[1]
        vz = target_vel[2]
        s = self.assumed_chaser_speed

        a = vx * vx + vy * vy + vz * vz - s * s
        b = 2.0 * (rx * vx + ry * vy + rz * vz)
        c = rx * rx + ry * ry + rz * rz

        if c <= 1e-6:
            return self.min_prediction_time

        if abs(a) < 1e-6:
            if abs(b) < 1e-6:
                return self._clamp_prediction_time(math.sqrt(c) / s)
            root = -c / b
            return self._clamp_prediction_time(root if root > 0.0 else math.sqrt(c) / s)

        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return self._clamp_prediction_time(math.sqrt(c) / s)

        sqrt_disc = math.sqrt(disc)
        roots = [(-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a)]
        positive_roots = [root for root in roots if root > 0.0]
        if not positive_roots:
            return self._clamp_prediction_time(math.sqrt(c) / s)
        return self._clamp_prediction_time(min(positive_roots))

    def _compute_intercept_point(self, chaser_pos, target_pos, target_vel):
        if self.guidance_target == "current" or self.active_prediction_model == "current":
            return target_pos, self._estimate_arrival_time(chaser_pos, target_pos)

        t_go = self._scan_intercept_time(chaser_pos, target_pos, target_vel)
        return self._future_target_position(target_pos, target_vel, t_go), t_go

    def _intercept_residual(self, chaser_pos, target_pos, target_vel, t_go):
        future = self._future_target_position(target_pos, target_vel, t_go)
        return self._estimate_arrival_time(chaser_pos, future) - t_go

    def _scan_intercept_time(self, chaser_pos, target_pos, target_vel):
        low = self.min_prediction_time
        high = self.max_prediction_time
        if high <= low + 1e-3:
            return low

        previous_t = low
        previous_f = self._intercept_residual(
            chaser_pos, target_pos, target_vel, previous_t
        )
        best_t = previous_t
        best_abs_f = abs(previous_f)

        for step in range(1, self.intercept_scan_steps + 1):
            t = low + (high - low) * float(step) / float(self.intercept_scan_steps)
            f = self._intercept_residual(chaser_pos, target_pos, target_vel, t)
            abs_f = abs(f)
            if abs_f < best_abs_f:
                best_abs_f = abs_f
                best_t = t

            if f <= 0.0 <= previous_f:
                a = previous_t
                b = t
                for _ in range(12):
                    mid = 0.5 * (a + b)
                    mid_f = self._intercept_residual(
                        chaser_pos, target_pos, target_vel, mid
                    )
                    if mid_f <= 0.0:
                        b = mid
                    else:
                        a = mid
                return self._clamp_prediction_time(b)

            previous_t = t
            previous_f = f

        return self._clamp_prediction_time(best_t)

    def _closing_speed(self, distance, v_rel_los):
        closing_speed = self.kp * distance + self.kd * max(v_rel_los, 0.0)
        closing_speed = self._clamp(
            closing_speed, self.min_closing_speed, self.max_closing_speed
        )

        if distance < self.capture_slowdown_distance:
            span = max(self.capture_slowdown_distance - self.stop_distance, 1e-3)
            blend = self._clamp((distance - self.stop_distance) / span, 0.0, 1.0)
            near_speed = max(self.capture_closing_speed, self.min_closing_speed)
            closing_speed = near_speed + blend * (closing_speed - near_speed)

        return closing_speed

    def _build_pd_velocity(self, rx, ry, rz, vrx, vry, vrz, vt, distance):
        chase_scale, follow_scale = self._slowdown_scales(distance)
        vx = follow_scale * vt[0] + chase_scale * (self.kp * rx + self.kd * vrx)
        vy = follow_scale * vt[1] + chase_scale * (self.kp * ry + self.kd * vry)
        vz = follow_scale * vt[2] + chase_scale * (self.kp * rz + self.kd * vrz)
        return vx, vy, vz, chase_scale, 0.0

    def _build_los_velocity(self, gx, gy, gz, rx, ry, rz, vrx, vry, vrz, vt, t_go):
        horizontal_distance = self._norm_xy(gx, gy)
        if self.guidance_dimension == "los_3d":
            distance_3d = self._norm3(gx, gy, gz)
            if distance_3d > 1e-3:
                ux = gx / distance_3d
                uy = gy / distance_3d
                uz = gz / distance_3d
            else:
                ux, uy, uz = 0.0, 0.0, 0.0

            v_rel_los = vrx * ux + vry * uy + vrz * uz
            closing_speed = self._closing_speed(distance_3d, v_rel_los)
            vx = self.feedforward_scale * vt[0] + closing_speed * ux
            vy = self.feedforward_scale * vt[1] + closing_speed * uy
            vz = self.feedforward_scale * vt[2] + closing_speed * uz
            return vx, vy, vz, 1.0, closing_speed

        if horizontal_distance > 1e-3:
            ux = gx / horizontal_distance
            uy = gy / horizontal_distance
        else:
            ux = 0.0
            uy = 0.0

        v_rel_los = vrx * ux + vry * uy
        closing_speed = self._closing_speed(horizontal_distance, v_rel_los)

        if horizontal_distance <= self.stop_distance:
            if self.stop_behavior == "zero":
                vx, vy = 0.0, 0.0
            else:
                vx, vy = self.feedforward_scale * vt[0], self.feedforward_scale * vt[1]
            chase_scale = 0.0
        else:
            vx = self.feedforward_scale * vt[0] + closing_speed * ux
            vy = self.feedforward_scale * vt[1] + closing_speed * uy
            chase_scale = 1.0

        if self.guidance_dimension == "xy_only":
            vz = 0.0
        else:
            # Z intercept time is floored higher than XY so that small
            # estimator Z errors at close range donʼt get amplified into
            # large vertical velocity commands.  When t_go = 0.3 s a 0.1 m
            # Z offset would produce 0.33 m/s upward — enough to climb
            # above the LiDAR FOV in a few seconds.
            z_t_go = max(t_go, 0.6)
            vz = (gz / z_t_go) + self.kp_z * rz + self.kd_z * vrz
        return vx, vy, vz, chase_scale, closing_speed

    def _terminal_guidance_active(self, distance_3d):
        if not self.terminal_guidance_enable:
            return False
        if self.target_input_mode != "estimator":
            return False
        if distance_3d > self.terminal_switch_distance:
            return False
        if self.estimator_tracking_state == "TRACKING":
            return True
        if self.terminal_max_observation_age <= 0.0:
            return False
        return self.estimator_observation_age <= self.terminal_max_observation_age

    def _terminal_guidance_point(self, pt, vt):
        t = self.terminal_prediction_time
        return (
            pt[0] + vt[0] * t,
            pt[1] + vt[1] * t,
            pt[2] + vt[2] * t,
        )

    def _limit_guidance_point_z(self, pos, name):
        if not self.guidance_z_limit_enable:
            return pos
        z = self._clamp(pos[2], self.guidance_z_min, self.guidance_z_max)
        if abs(z - pos[2]) > 1e-6:
            rospy.logwarn_throttle(
                0.5,
                "[Uav0VelocityGuidanceZLimit] %s_z=%.2f limited_z=%.2f bounds=[%.2f %.2f]",
                name,
                pos[2],
                z,
                self.guidance_z_min,
                self.guidance_z_max,
            )
        return (pos[0], pos[1], z)

    def _altitude_guard_velocity(self, z, vz):
        if not self.altitude_guard_enable:
            return vz, False
        guarded = False
        if z >= self.altitude_guard_max_z:
            correction = -self.altitude_guard_gain * (z - self.altitude_guard_max_z)
            correction = self._clamp(
                correction,
                -self.altitude_guard_max_correction,
                0.0,
            )
            vz = min(vz, correction)
            guarded = True
        elif z <= self.altitude_guard_min_z:
            correction = self.altitude_guard_gain * (self.altitude_guard_min_z - z)
            correction = self._clamp(
                correction,
                0.0,
                self.altitude_guard_max_correction,
            )
            vz = max(vz, correction)
            guarded = True
        if guarded:
            rospy.logwarn_throttle(
                0.5,
                "[Uav0VelocityGuidanceAltitudeGuard] z=%.2f vz_cmd=%.2f bounds=[%.2f %.2f]",
                z,
                vz,
                self.altitude_guard_min_z,
                self.altitude_guard_max_z,
            )
        return vz, guarded

    def _build_sphere_marker(self, header, marker_id, ns, position, color, scale):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = rospy.Duration(1.0)
        return marker

    def _build_line_marker(self, header, marker_id, ns, points, color, width):
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

    def _build_text_marker(self, header, marker_id, ns, position, text):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2] + 0.55
        marker.scale.z = 0.32
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = text
        marker.lifetime = rospy.Duration(1.0)
        return marker

    def _publish_guidance_markers(
        self,
        stamp,
        chaser_pos,
        target_pos,
        target_vel,
        intercept_pos,
        t_go,
        distance_xy,
        distance_z,
        distance_3d,
        v_cmd,
        yaw_error,
        yaw_aligned,
    ):
        header = Header(stamp=stamp, frame_id=self.frame_id)
        if self.target_input_mode == "estimator":
            predicted_path = [target_pos, intercept_pos]
        else:
            predicted_path = [
                self._future_target_position(
                    target_pos, target_vel, t_go * float(i) / 16.0
                )
                for i in range(17)
            ]
        self.marker_pub.publish(
            self._build_sphere_marker(
                header,
                0,
                "uav0_guidance_target_current",
                target_pos,
                (0.0, 0.7, 1.0, 0.85),
                self.marker_scale * 0.75,
            )
        )
        self.marker_pub.publish(
            self._build_sphere_marker(
                header,
                1,
                "uav0_guidance_intercept",
                intercept_pos,
                (1.0, 0.05, 0.05, 0.95),
                self.marker_scale,
            )
        )
        self.marker_pub.publish(
            self._build_line_marker(
                header,
                2,
                "uav0_guidance_chaser_to_intercept",
                [chaser_pos, intercept_pos],
                (0.2, 1.0, 0.2, 0.85),
                0.05,
            )
        )
        self.marker_pub.publish(
            self._build_line_marker(
                header,
                3,
                "uav0_guidance_target_to_intercept",
                [target_pos, intercept_pos],
                (1.0, 0.65, 0.0, 0.85),
                0.04,
            )
        )
        self.marker_pub.publish(
            self._build_line_marker(
                header,
                6,
                "uav0_guidance_predicted_path",
                predicted_path,
                (1.0, 0.65, 0.0, 0.45),
                0.025,
            )
        )
        self.marker_pub.publish(
            self._build_text_marker(
                header,
                4,
                "uav0_guidance_intercept_text",
                intercept_pos,
                "distance_xy={:.2f}\ndistance_z={:.2f}\ndistance_3d={:.2f}\nt_go={:.2f}\nv_cmd_xy={:.2f}\nv_cmd_z={:.2f}\nyaw_error={:.2f}\nyaw_aligned={}\nguidance_dimension={}\nprediction_model={}".format(
                    distance_xy,
                    distance_z,
                    distance_3d,
                    t_go,
                    self._norm_xy(v_cmd[0], v_cmd[1]),
                    v_cmd[2],
                    yaw_error,
                    yaw_aligned,
                    self.guidance_dimension,
                    self.active_prediction_model,
                ),
            )
        )

        if self.active_prediction_model == "learned_circle" and self.last_circle_fit:
            center = self.last_circle_fit["center"]
            radius = self.last_circle_fit["radius"]
            learned_points = [
                (
                    center[0] + radius * math.cos(2.0 * math.pi * float(i) / 72.0),
                    center[1] + radius * math.sin(2.0 * math.pi * float(i) / 72.0),
                    target_pos[2],
                )
                for i in range(73)
            ]
            self.marker_pub.publish(
                self._build_line_marker(
                    header,
                    5,
                    "uav0_guidance_learned_circle",
                    learned_points,
                    (0.65, 0.25, 1.0, 0.65),
                    0.03,
                )
            )
            return

        if self.active_prediction_model not in ("circle_configured", "line"):
            return

        center = self._target_center_common(target_pos)
        if self.active_prediction_model == "line":
            configured_points = [
                (center[0] - self.target_radius, center[1], center[2]),
                (center[0] + self.target_radius, center[1], center[2]),
            ]
            configured_ns = "uav0_guidance_configured_line"
        else:
            configured_points = [
                self._configured_circle_point(2.0 * math.pi * float(i) / 72.0, center)
                for i in range(73)
            ]
            configured_ns = "uav0_guidance_configured_circle"
        self.marker_pub.publish(
            self._build_line_marker(
                header,
                5,
                configured_ns,
                configured_points,
                (0.75, 0.75, 0.75, 0.45),
                0.025,
            )
        )
        if self.active_prediction_model == "circle_configured":
            self.marker_pub.publish(
                self._build_text_marker(
                    header,
                    7,
                    "uav0_guidance_configured_mode_text",
                    (center[0], center[1], center[2]),
                    "configured/known trajectory mode",
                )
            )

    def _publish_intercept_pose(self, stamp, intercept_pos):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = intercept_pos[0]
        pose.pose.position.y = intercept_pos[1]
        pose.pose.position.z = intercept_pos[2]
        pose.pose.orientation.w = 1.0
        self.intercept_pose_pub.publish(pose)

    def _publish_paths(self, stamp, uav0_pos, intercept_pos):
        """Publish UAV0 flight path and intercept line for RViz."""
        # --- UAV0 flight trail ---
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = uav0_pos[0]
        pose.pose.position.y = uav0_pos[1]
        pose.pose.position.z = uav0_pos[2]
        pose.pose.orientation.w = 1.0
        self._path_msg.poses.append(pose)
        if len(self._path_msg.poses) > self._path_max_len:
            self._path_msg.poses = self._path_msg.poses[-self._path_max_len:]
        self._path_msg.header.stamp = stamp
        self.path_pub.publish(self._path_msg)

        # --- Intercept line: UAV0 → intercept point ---
        line = Path()
        line.header.stamp = stamp
        line.header.frame_id = "map"
        p0 = PoseStamped()
        p0.header = line.header
        p0.pose.position.x = uav0_pos[0]
        p0.pose.position.y = uav0_pos[1]
        p0.pose.position.z = uav0_pos[2]
        p0.pose.orientation.w = 1.0
        p1 = PoseStamped()
        p1.header = line.header
        p1.pose.position.x = intercept_pos[0]
        p1.pose.position.y = intercept_pos[1]
        p1.pose.position.z = intercept_pos[2]
        p1.pose.orientation.w = 1.0
        line.poses = [p0, p1]
        self.intercept_line_pub.publish(line)

    def maybe_request_offboard_and_arm(self):
        if not self.auto_offboard and not self.auto_arm:
            return
        if not self.mavros_state.connected:
            rospy.logwarn_throttle(
                2.0,
                "[Uav0VelocityGuidance] waiting for %s/state before requesting OFFBOARD/arm",
                self.mavros_ns,
            )
            return

        now = rospy.Time.now()
        if (
            self.last_offboard_request_time is not None
            and (now - self.last_offboard_request_time).to_sec()
            < self.offboard_request_interval
        ):
            return
        self.last_offboard_request_time = now

        if self.auto_offboard and self.mavros_state.mode != "OFFBOARD":
            try:
                self.set_mode_srv(custom_mode="OFFBOARD")
                rospy.loginfo_throttle(
                    2.0,
                    "[Uav0VelocityGuidance] OFFBOARD requested for %s while velocity setpoints are active",
                    self.mavros_ns,
                )
            except rospy.ServiceException as exc:
                rospy.logerr_throttle(
                    2.0, "[Uav0VelocityGuidance] OFFBOARD request failed: %s", exc
                )

        if self.auto_arm and not self.mavros_state.armed:
            try:
                self.arming_srv(True)
                rospy.loginfo_throttle(
                    2.0, "[Uav0VelocityGuidance] arm requested for %s", self.mavros_ns
                )
            except rospy.ServiceException as exc:
                rospy.logerr_throttle(
                    2.0, "[Uav0VelocityGuidance] arm request failed: %s", exc
                )

    def _limit_velocity(self, vx, vy, vz):
        vxy = self._norm_xy(vx, vy)
        if vxy > self.vxy_max and vxy > 1e-6:
            scale = self.vxy_max / vxy
            vx *= scale
            vy *= scale

        vz = self._clamp(vz, -self.vz_max, self.vz_max)
        return vx, vy, vz

    def _limit_acceleration(self, vx, vy, vz, dt):
        dvx = vx - self.last_cmd[0]
        dvy = vy - self.last_cmd[1]
        dvz = vz - self.last_cmd[2]

        dvxy = self._norm_xy(dvx, dvy)
        max_dvxy = self.axy_max * dt
        if dvxy > max_dvxy and dvxy > 1e-6:
            scale = max_dvxy / dvxy
            dvx *= scale
            dvy *= scale

        max_dvz = self.az_max * dt
        dvz = self._clamp(dvz, -max_dvz, max_dvz)

        limited = [
            self.last_cmd[0] + dvx,
            self.last_cmd[1] + dvy,
            self.last_cmd[2] + dvz,
        ]
        self.last_cmd = limited
        return limited

    def _estimator_input_fresh(self, now):
        if self.target_input_mode != "estimator":
            return True
        if self.estimator_tracking_state == "PREDICT_ONLY":
            return self.estimator_observation_age <= self.max_predict_only_guidance_time
        if self.estimator_tracking_state == "LOST":
            return self.lost_target_behavior in ("search_last", "continue_predict")
        if self.estimator_input_timeout <= 0.0:
            return True
        stamps = (
            self.estimator_state_stamp,
            self.estimator_intercept_stamp,
            self.estimator_t_go_stamp,
        )
        if any(stamp is None for stamp in stamps):
            return False
        oldest_age = max((now - stamp).to_sec() for stamp in stamps)
        if oldest_age > self.estimator_input_timeout:
            rospy.logwarn_throttle(
                1.0,
                "[Uav0VelocityGuidance] estimator input stale for %.2fs; holding zero velocity until LiDAR observations recover",
                oldest_age,
            )
            return False
        return True

    def _guidance_speed_scale(self):
        if self.target_input_mode != "estimator":
            return 1.0

        if self.estimator_tracking_state == "TRACKING":
            return 1.0

        # During PREDICT_ONLY: smooth ramp from 1.0 down to predict_only_speed_scale
        # over max_predict_only_guidance_time, so there's no abrupt speed cut.
        # If already close to the target (< capture_slowdown_distance), keep full speed
        # to complete the intercept rather than slowing down.
        obs_age = self.estimator_observation_age
        if self.estimator_tracking_state == "PREDICT_ONLY":
            # Check if we're already close — don't slow down mid-approach
            if self._is_close_to_target():
                return max(self.predict_only_speed_scale, 0.9)
            if self.max_predict_only_guidance_time > 0.0:
                blend = min(obs_age / self.max_predict_only_guidance_time, 1.0)
                return 1.0 - blend * (1.0 - self.predict_only_speed_scale)
            return self.predict_only_speed_scale

        if self.estimator_tracking_state == "LOST":
            if self.lost_target_behavior in ("search_last", "continue_predict"):
                # If close to target, maintain higher speed
                if self._is_close_to_target():
                    return max(self.lost_target_speed_scale, 0.8)
                return self.lost_target_speed_scale
            return 0.0

        return 1.0

    def _is_close_to_target(self):
        """Check if chaser is within capture_slowdown_distance of the last known target."""
        if self.chaser_odom is None:
            return False
        pc = self._to_common(self._pos(self.chaser_odom), self.uav0_spawn_offset)
        # Use estimator state or intercept pos as reference
        if self.estimator_intercept_pos is not None:
            pt = self.estimator_intercept_pos
        elif self.estimator_state is not None:
            pt = self._pos(self.estimator_state)
        else:
            return False
        dist = self._norm3(pt[0] - pc[0], pt[1] - pc[1], pt[2] - pc[2])
        return dist <= self.capture_slowdown_distance * 2.0

    def _yaw_rate_command(self, rx, ry):
        if self.yaw_mode == "none":
            return 0.0

        if self.yaw_mode not in ("face_target", "face_guidance"):
            rospy.logwarn_throttle(
                5.0,
                "[Uav0VelocityGuidance] unknown yaw_mode '%s', using none",
                self.yaw_mode,
            )
            return 0.0

        if self._norm_xy(rx, ry) <= 1e-3 or self.chaser_odom is None:
            return 0.0

        desired_yaw = math.atan2(ry, rx)
        current_yaw = self._quat_to_yaw(self.chaser_odom.pose.pose.orientation)
        yaw_error = self._wrap_angle(desired_yaw - current_yaw)
        return self._clamp(
            self.kyaw * yaw_error, -self.yaw_rate_max, self.yaw_rate_max
        )

    def _yaw_command_and_alignment(self, rx, ry, now):
        if self.yaw_mode == "none":
            self.yaw_aligned_since = now
            return 0.0, 0.0, True

        if self.yaw_mode not in ("face_target", "face_guidance"):
            rospy.logwarn_throttle(
                5.0,
                "[Uav0VelocityGuidance] unknown yaw_mode '%s', using none",
                self.yaw_mode,
            )
            self.yaw_aligned_since = now
            return 0.0, 0.0, True

        if self._norm_xy(rx, ry) <= 1e-3 or self.chaser_odom is None:
            self.yaw_aligned_since = now
            return 0.0, 0.0, True

        desired_yaw = math.atan2(ry, rx)
        current_yaw = self._quat_to_yaw(self.chaser_odom.pose.pose.orientation)
        yaw_error = self._wrap_angle(desired_yaw - current_yaw)
        yaw_rate = self._clamp(
            self.kyaw * yaw_error, -self.yaw_rate_max, self.yaw_rate_max
        )

        if abs(yaw_error) <= self.yaw_align_threshold:
            if self.yaw_aligned_since is None:
                self.yaw_aligned_since = now
            yaw_aligned = (
                now - self.yaw_aligned_since
            ).to_sec() >= self.yaw_align_hold_time
        else:
            self.yaw_aligned_since = None
            yaw_aligned = False

        return yaw_rate, yaw_error, yaw_aligned

    def _reacquire_yaw_fresh(self, now):
        if (
            not self.use_yaw_reacquire
            or self.reacquire_yaw_target is None
            or self.reacquire_yaw_target_stamp is None
            or self.chaser_odom is None
        ):
            return False
        if self.reacquire_state == "VISUAL_TRACK":
            return False
        return (now - self.reacquire_yaw_target_stamp).to_sec() <= self.reacquire_yaw_target_max_age

    def _yaw_to_angle_command_and_alignment(self, desired_yaw, now):
        if self.chaser_odom is None or self.yaw_mode == "none":
            self.yaw_aligned_since = now
            return 0.0, 0.0, True
        current_yaw = self._quat_to_yaw(self.chaser_odom.pose.pose.orientation)
        yaw_error = self._wrap_angle(desired_yaw - current_yaw)
        yaw_rate = self._clamp(
            self.kyaw * yaw_error, -self.yaw_rate_max, self.yaw_rate_max
        )
        if abs(yaw_error) <= self.yaw_align_threshold:
            if self.yaw_aligned_since is None:
                self.yaw_aligned_since = now
            yaw_aligned = (
                now - self.yaw_aligned_since
            ).to_sec() >= self.yaw_align_hold_time
        else:
            self.yaw_aligned_since = None
            yaw_aligned = False
        return yaw_rate, yaw_error, yaw_aligned

    def _offboard_ready(self):
        return self.mavros_state.mode == "OFFBOARD" and self.mavros_state.armed

    def _zero_velocity_reason(self, v_cmd, fallback_reason="none"):
        norm = self._norm3(v_cmd[0], v_cmd[1], v_cmd[2])
        if norm > 0.03:
            return "moving"
        if fallback_reason != "none":
            return fallback_reason
        if not self._offboard_ready():
            return "offboard_not_ready"
        return "near_zero_command"

    def _log_diagnostic(self, v_cmd, reason_if_zero_velocity):
        rospy.loginfo_throttle(
            1.0,
            "[Uav0VelocityGuidanceDiag] target_input_mode=%s received_estimator_state=%s received_intercept_point=%s tracking_state=%s lost_target_behavior=%s v_cmd=(%.2f %.2f %.2f) v_cmd_norm=%.2f reason_if_zero_velocity=%s",
            self.target_input_mode,
            self.estimator_state is not None,
            self.estimator_intercept_pos is not None,
            self.estimator_tracking_state,
            self.lost_target_behavior,
            v_cmd[0],
            v_cmd[1],
            v_cmd[2],
            self._norm3(v_cmd[0], v_cmd[1], v_cmd[2]),
            reason_if_zero_velocity,
        )

    def build_command(self):
        now = rospy.Time.now()
        dt = self._dt(now)

        cmd = TwistStamped()
        cmd.header.stamp = now
        cmd.header.frame_id = self.frame_id

        if self.stop_on_capture_success and self.capture_success:
            self.last_cmd = [0.0, 0.0, 0.0]
            rospy.loginfo_throttle(
                1.0,
                "[Uav0VelocityGuidance] capture_success is true; holding zero velocity.",
            )
            self._log_diagnostic((0.0, 0.0, 0.0), "capture_success_stop")
            return cmd

        target_ready = (
            self.target_odom is not None
            if self.target_input_mode == "internal"
            else self.estimator_state is not None
            and self.estimator_intercept_pos is not None
            and self.estimator_t_go is not None
            and self._estimator_input_fresh(now)
        )
        if self.chaser_odom is None or not target_ready:
            rospy.loginfo_throttle(
                3.0,
                "[Uav0VelocityGuidance] waiting for chaser odom and target input (%s)",
                self.target_input_mode,
            )
            if self.chaser_odom is None:
                zero_reason = "no_chaser_odom"
            elif self.target_input_mode == "estimator" and self.estimator_state is None:
                zero_reason = "no_estimator_state"
            elif self.target_input_mode == "estimator" and self.estimator_intercept_pos is None:
                zero_reason = "no_intercept_point"
            elif self.target_input_mode == "estimator" and self.estimator_tracking_state == "LOST" and self.lost_target_behavior == "hold":
                zero_reason = "target_lost_hold"
            else:
                zero_reason = "no_target_input"
            self.last_cmd = self._limit_acceleration(0.0, 0.0, 0.0, dt)
            cmd.twist.linear.x = self.last_cmd[0]
            cmd.twist.linear.y = self.last_cmd[1]
            cmd.twist.linear.z = self.last_cmd[2]
            if self._reacquire_yaw_fresh(now):
                yaw_rate, _yaw_error, _yaw_aligned = (
                    self._yaw_to_angle_command_and_alignment(
                        self.reacquire_yaw_target, now
                    )
                )
                cmd.twist.angular.z = yaw_rate
            self._log_diagnostic(
                (cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z),
                zero_reason,
            )
            return cmd

        pc_raw = self._pos(self.chaser_odom)
        pc = self._to_common(pc_raw, self.uav0_spawn_offset)
        vc = self.chaser_velocity
        if self.target_input_mode == "internal":
            pt_raw = self._pos(self.target_odom)
            pt = self._to_common(pt_raw, self.uav1_spawn_offset)
            vt = self.target_velocity
            self._update_prediction_state()
            self._warn_if_target_off_configured_circle(pt)
            intercept_pos, t_go = self._compute_intercept_point(pc, pt, vt)
        else:
            pt_raw = self._pos(self.estimator_state)
            pt = pt_raw
            vt = self._vel(self.estimator_state)
            self.active_prediction_model = "estimator"
            self.learned_circle_in_use = False
            self.last_circle_fit = None
            self.prediction_fallback_reason = ""
            intercept_pos = self.estimator_intercept_pos
            t_go = self.estimator_t_go

        pt = self._limit_guidance_point_z(pt, "target")
        intercept_pos = self._limit_guidance_point_z(intercept_pos, "intercept")

        rx = pt[0] - pc[0]
        ry = pt[1] - pc[1]
        rz = pt[2] - pc[2]
        gx = intercept_pos[0] - pc[0]
        gy = intercept_pos[1] - pc[1]
        gz = intercept_pos[2] - pc[2]
        vrx = vt[0] - vc[0]
        vry = vt[1] - vc[1]
        vrz = vt[2] - vc[2]
        distance_xy = self._norm_xy(rx, ry)
        distance_z = abs(rz)
        distance_3d = self._norm3(rx, ry, rz)
        distance_for_capture = distance_3d if self.capture_use_3d_distance else distance_xy
        terminal_guidance_active = self._terminal_guidance_active(distance_3d)

        if terminal_guidance_active:
            intercept_pos = self._terminal_guidance_point(pt, vt)
            gx = intercept_pos[0] - pc[0]
            gy = intercept_pos[1] - pc[1]
            gz = intercept_pos[2] - pc[2]
            t_go = max(self.terminal_prediction_time, 1.0 / self.command_rate)

        if distance_for_capture <= self.stop_distance:
            if self.stop_behavior == "zero":
                vx, vy, vz = 0.0, 0.0, 0.0
            else:
                vx, vy, vz = vt
            chase_scale = 0.0
            closing_speed = 0.0
            zero_reason = "stop_distance"
        elif self.guidance_mode == "pd":
            vx, vy, vz, chase_scale, closing_speed = self._build_pd_velocity(
                rx, ry, rz, vrx, vry, vrz, vt, distance_for_capture
            )
            zero_reason = "none"
        else:
            if self.guidance_mode != "los":
                rospy.logwarn_throttle(
                    5.0,
                    "[Uav0VelocityGuidance] unknown guidance_mode '%s', using los",
                    self.guidance_mode,
                )
            vx, vy, vz, chase_scale, closing_speed = self._build_los_velocity(
                gx, gy, gz, rx, ry, rz, vrx, vry, vrz, vt, t_go
            )
            zero_reason = "none"

        state_speed_scale = self._guidance_speed_scale()
        if state_speed_scale <= 0.0:
            vx, vy, vz = 0.0, 0.0, 0.0
            chase_scale = 0.0
            closing_speed = 0.0
            zero_reason = (
                "target_lost_hold"
                if self.estimator_tracking_state == "LOST"
                else "tracking_state_speed_zero"
            )
        elif state_speed_scale < 1.0:
            vx *= state_speed_scale
            vy *= state_speed_scale
            vz *= state_speed_scale
            chase_scale *= state_speed_scale
            closing_speed *= state_speed_scale

        if terminal_guidance_active:
            if distance_z <= self.terminal_z_deadband:
                vz = 0.0
            else:
                vz = self._clamp(vz, -self.terminal_vz_max, self.terminal_vz_max)

        if self.yaw_mode == "face_guidance":
            yaw_rx, yaw_ry = gx, gy
        else:
            yaw_rx, yaw_ry = rx, ry
        yaw_override_active = self._reacquire_yaw_fresh(now)
        if yaw_override_active:
            yaw_rate, yaw_error, yaw_aligned = self._yaw_to_angle_command_and_alignment(
                self.reacquire_yaw_target, now
            )
        else:
            yaw_rate, yaw_error, yaw_aligned = self._yaw_command_and_alignment(
                yaw_rx, yaw_ry, now
            )
        yaw_gate_active = (
            self.face_guidance_before_move
            and self.yaw_mode != "none"
            and (
                yaw_override_active
                or self._norm_xy(yaw_rx, yaw_ry) >= self.yaw_align_min_distance
            )
            and not yaw_aligned
        )
        if yaw_gate_active:
            vx *= self.yaw_gate_xy_scale
            vy *= self.yaw_gate_xy_scale
            if not self.allow_vertical_during_yaw_align:
                vz = 0.0
            chase_scale *= self.yaw_gate_xy_scale
            closing_speed *= self.yaw_gate_xy_scale
            if self._norm3(vx, vy, vz) <= 0.03:
                zero_reason = "yaw_gate"

        vx, vy, vz = self._limit_velocity(vx, vy, vz)
        vx, vy, vz = self._limit_acceleration(vx, vy, vz, dt)
        vz, altitude_guard_active = self._altitude_guard_velocity(pc[2], vz)
        if altitude_guard_active:
            self.last_cmd[2] = vz

        # --- LOST recovery: climb to search altitude & return to search area ---
        # When no valid observation has arrived for longer than
        # lost_recovery_timeout, climb to lost_recovery_search_z so that
        # the LiDAR and depth camera can reacquire the target.
        # After extended LOST (>10s), fly back to the target circle center
        # instead of chasing stale predictions.
        obs_stale = (
            self.estimator_observation_confidence < 0.01
            and self.estimator_observation_age > self.lost_recovery_timeout
        )
        if (
            self.lost_recovery_enable
            and self.target_input_mode == "estimator"
            and obs_stale
        ):
            target_z = self.lost_recovery_search_z
            dz_to_search = target_z - pc[2]
            stale_s = self.estimator_observation_age - self.lost_recovery_timeout
            recovery_speed = min(0.3 + 0.15 * stale_s, self.lost_recovery_descent_speed)

            if abs(dz_to_search) > 0.15:
                vz = math.copysign(recovery_speed, dz_to_search)
            else:
                vz = 0.0

            # After extended LOST, fly back to target circle center
            if stale_s > 8.0:
                # Use known target center instead of stale prediction
                center = self._target_center_common(pt)
                if center and not any(math.isnan(c) for c in center):
                    rx_center = center[0] - pc[0]
                    ry_center = center[1] - pc[1]
                else:
                    rx_center = self.target_center_x - pc[0]
                    ry_center = self.target_center_y - pc[1]
                d_center = self._norm_xy(rx_center, ry_center)
                if d_center > 0.5:
                    scale = min(self.vxy_max, 2.5 * d_center) / d_center
                    vx = rx_center * scale
                    vy = ry_center * scale
                else:
                    vx, vy = 0.0, 0.0
            else:
                # Short LOST: push toward last known intercept
                gx_norm = self._norm_xy(gx, gy)
                if gx_norm > 0.3:
                    scale = 0.8 * self.vxy_max / gx_norm
                    vx = gx * scale
                    vy = gy * scale
            self.last_cmd = [vx, vy, vz]
            zero_reason = "lost_recovery_climb"

        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        cmd.twist.angular.z = yaw_rate
        reason_if_zero_velocity = self._zero_velocity_reason((vx, vy, vz), zero_reason)
        self.last_zero_velocity_reason = reason_if_zero_velocity
        self._publish_guidance_markers(
            now,
            pc,
            pt,
            vt,
            intercept_pos,
            t_go,
            distance_xy,
            distance_z,
            distance_3d,
            (vx, vy, vz),
            yaw_error,
            yaw_aligned,
        )
        self._publish_intercept_pose(now, intercept_pos)
        self._publish_paths(now, pc, intercept_pos)

        if self.target_input_mode == "internal":
            circle_center = self._target_center_common(pt)
        else:
            circle_center = (float("nan"), float("nan"), float("nan"))
        if self.last_circle_fit:
            learned_center = self.last_circle_fit["center"]
            learned_radius = self.last_circle_fit["radius"]
            learned_omega = self.last_circle_fit["omega"]
            learned_residual = self.last_circle_fit["residual"]
        else:
            learned_center = (float("nan"), float("nan"), float("nan"))
            learned_radius = float("nan")
            learned_omega = float("nan")
            learned_residual = float("nan")

        rospy.loginfo_throttle(
            1.0,
            "[Uav0VelocityGuidance] raw_uav0=(%.2f %.2f %.2f) raw_uav1=(%.2f %.2f %.2f) p_uav0_common=(%.2f %.2f %.2f) p_uav1_common=(%.2f %.2f %.2f) uav0_offset=(%.2f %.2f %.2f) uav1_offset=(%.2f %.2f %.2f) r=(%.2f %.2f %.2f) distance_xy=%.2f distance_z=%.2f distance_3d=%.2f guidance_mode=%s guidance_dimension=%s target=%s requested_model=%s prediction_model=%s tracking_state=%s obs_age=%.2f obs_conf=%.2f speed_scale=%.2f target_center_frame=%s circle_center_common=(%.2f %.2f %.2f) ff=%.2f closing=%.2f t_go=%.2f terminal_guidance=%s p_guidance=(%.2f %.2f %.2f) v_target=(%.2f %.2f %.2f) v_uav0=(%.2f %.2f %.2f) learned_circle=%s learned_center=(%.2f %.2f %.2f) learned_radius=%.2f learned_omega=%.3f learned_residual=%.2f fallback='%s' v_cmd=(%.2f %.2f %.2f) yaw_mode=%s yaw_error=%.2f yaw_aligned=%s yaw_gate=%s yaw_rate=%.2f",
            pc_raw[0],
            pc_raw[1],
            pc_raw[2],
            pt_raw[0],
            pt_raw[1],
            pt_raw[2],
            pc[0],
            pc[1],
            pc[2],
            pt[0],
            pt[1],
            pt[2],
            self.uav0_spawn_offset[0],
            self.uav0_spawn_offset[1],
            self.uav0_spawn_offset[2],
            self.uav1_spawn_offset[0],
            self.uav1_spawn_offset[1],
            self.uav1_spawn_offset[2],
            rx,
            ry,
            rz,
            distance_xy,
            distance_z,
            distance_3d,
            self.guidance_mode,
            self.guidance_dimension,
            self.guidance_target,
            self.prediction_model,
            self.active_prediction_model,
            self.estimator_tracking_state,
            self.estimator_observation_age,
            self.estimator_observation_confidence,
            state_speed_scale,
            self.target_center_frame,
            circle_center[0],
            circle_center[1],
            circle_center[2],
            self.feedforward_scale,
            closing_speed,
            t_go,
            terminal_guidance_active,
            intercept_pos[0],
            intercept_pos[1],
            intercept_pos[2],
            vt[0],
            vt[1],
            vt[2],
            vc[0],
            vc[1],
            vc[2],
            self.learned_circle_in_use,
            learned_center[0],
            learned_center[1],
            learned_center[2],
            learned_radius,
            learned_omega,
            learned_residual,
            self.prediction_fallback_reason,
            vx,
            vy,
            vz,
            self.yaw_mode,
            yaw_error,
            yaw_aligned,
            yaw_gate_active,
            cmd.twist.angular.z,
        )
        self._log_diagnostic((vx, vy, vz), reason_if_zero_velocity)
        return cmd

    def run(self):
        rate = rospy.Rate(self.command_rate)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.build_command())
            self.maybe_request_offboard_and_arm()
            rate.sleep()


def main():
    rospy.init_node("uav0_velocity_guidance_node")
    Uav0VelocityGuidance().run()


if __name__ == "__main__":
    main()
