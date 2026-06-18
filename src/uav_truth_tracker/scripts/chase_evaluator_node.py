#!/usr/bin/env python3
"""Record online chase and prediction data to CSV for offline analysis."""

import csv
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String, UInt32


class ChaseEvaluator:
    def __init__(self):
        self.uav0_odom_topic = rospy.get_param(
            "~uav0_odom_topic", "/uav0/mavros/local_position/odom"
        )
        self.uav1_odom_topic = rospy.get_param(
            "~uav1_odom_topic", "/uav1/mavros/local_position/odom"
        )
        self.estimator_state_topic = rospy.get_param(
            "~estimator_state_topic", "/target_estimator/state"
        )
        self.intercept_topic = rospy.get_param(
            "~intercept_topic", "/target_estimator/intercept_point"
        )
        self.t_go_topic = rospy.get_param("~t_go_topic", "/target_estimator/t_go")
        self.cmd_vel_topic = rospy.get_param(
            "~cmd_vel_topic", "/uav0/mavros/setpoint_velocity/cmd_vel"
        )
        self.capture_success_topic = rospy.get_param(
            "~capture_success_topic", "/uav0/capture/success"
        )
        self.target_observation_topic = rospy.get_param(
            "~target_observation_topic", "/target_observation/pose"
        )
        self.detection_confidence_topic = rospy.get_param(
            "~detection_confidence_topic", "/target_observation/confidence"
        )
        self.detection_valid_topic = rospy.get_param(
            "~detection_valid_topic", "/target_observation/valid"
        )
        self.target_visual_topic = rospy.get_param(
            "~target_visual_topic", "/target_visual/pose"
        )
        self.target_visual_confidence_topic = rospy.get_param(
            "~target_visual_confidence_topic", "/target_visual/confidence"
        )
        self.target_visual_valid_topic = rospy.get_param(
            "~target_visual_valid_topic", "/target_visual/valid"
        )
        self.target_lidar_topic = rospy.get_param(
            "~target_lidar_topic", "/target_observation/pose"
        )
        self.target_lidar_confidence_topic = rospy.get_param(
            "~target_lidar_confidence_topic", "/target_observation/confidence"
        )
        self.target_lidar_valid_topic = rospy.get_param(
            "~target_lidar_valid_topic", "/target_observation/valid"
        )
        self.target_fused_topic = rospy.get_param(
            "~target_fused_topic", "/target_fused/pose"
        )
        self.target_fused_confidence_topic = rospy.get_param(
            "~target_fused_confidence_topic", "/target_fused/confidence"
        )
        self.target_fused_valid_topic = rospy.get_param(
            "~target_fused_valid_topic", "/target_fused/valid"
        )
        self.target_fused_source_topic = rospy.get_param(
            "~target_fused_source_topic", "/target_fused/source"
        )
        self.sensor_valid_age = max(rospy.get_param("~sensor_valid_age", 0.5), 0.0)
        self.detection_point_count_topic = rospy.get_param(
            "~detection_point_count_topic", "/target_observation/point_count"
        )
        self.tracking_state_topic = rospy.get_param(
            "~tracking_state_topic", "/target_estimator/tracking_state"
        )
        self.observation_age_topic = rospy.get_param(
            "~observation_age_topic", "/target_estimator/observation_age"
        )
        self.detector_state_topic = rospy.get_param(
            "~detector_state_topic", "/target_observation/detector_state"
        )
        self.candidate_count_topic = rospy.get_param(
            "~candidate_count_topic", "/target_observation/candidate_count"
        )
        self.range_valid_cluster_count_topic = rospy.get_param(
            "~range_valid_cluster_count_topic",
            "/target_observation/range_valid_cluster_count",
        )
        self.selected_cluster_id_topic = rospy.get_param(
            "~selected_cluster_id_topic", "/target_observation/selected_cluster_id"
        )
        self.selected_cluster_point_count_topic = rospy.get_param(
            "~selected_cluster_point_count_topic",
            "/target_observation/selected_cluster_point_count",
        )
        self.selected_cluster_score_topic = rospy.get_param(
            "~selected_cluster_score_topic",
            "/target_observation/selected_cluster_score",
        )
        self.second_best_score_topic = rospy.get_param(
            "~second_best_score_topic", "/target_observation/second_best_score"
        )
        self.association_margin_topic = rospy.get_param(
            "~association_margin_topic", "/target_observation/association_margin"
        )
        self.hit_count_topic = rospy.get_param(
            "~hit_count_topic", "/target_observation/hit_count"
        )
        self.miss_count_topic = rospy.get_param(
            "~miss_count_topic", "/target_observation/miss_count"
        )
        self.reacquire_count_topic = rospy.get_param(
            "~reacquire_count_topic", "/target_observation/reacquire_count"
        )
        self.time_since_last_detection_topic = rospy.get_param(
            "~time_since_last_detection_topic",
            "/target_observation/time_since_last_detection",
        )
        self.gate_radius_topic = rospy.get_param(
            "~gate_radius_topic", "/target_observation/gate_radius"
        )
        self.use_estimator_gating_topic = rospy.get_param(
            "~use_estimator_gating_topic",
            "/target_observation/use_estimator_gating",
        )
        self.selected_distance_to_gate_topic = rospy.get_param(
            "~selected_distance_to_gate_topic",
            "/target_observation/selected_distance_to_gate",
        )
        self.observation_source = rospy.get_param("~observation_source", "truth_odom")
        self.lidar_cloud_topic = rospy.get_param(
            "~lidar_cloud_topic", "/uav0/velodyne_points"
        )
        self.sample_rate = max(rospy.get_param("~sample_rate", 20.0), 1.0)
        self.estimator_model = rospy.get_param("~estimator_model", "kalman_cv")
        self.target_mode = rospy.get_param("~target_mode", "circle_z_sine")
        self.uav1_speed = rospy.get_param("~uav1_speed", float("nan"))
        self.z_amplitude = rospy.get_param("~z_amplitude", float("nan"))

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

        log_dir = os.path.expanduser(
            rospy.get_param("~log_dir", "~/uav_intercept_logs")
        )
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        default_name = "{}_{}_{}.csv".format(
            self.target_mode, self.estimator_model, timestamp
        )
        self.csv_path = os.path.expanduser(
            rospy.get_param("~csv_path", os.path.join(log_dir, default_name))
        )

        self.uav0_pos = None
        self.uav1_pos = None
        self.target_est_pos = None
        self.target_est_vel = None
        self.pred_pos = None
        self.target_observation_pos = None
        self.target_observation_stamp = None
        self.detection_valid_msg = False
        self.detection_confidence = 0.0
        self.detection_point_count = 0
        self.target_visual_pos = None
        self.target_visual_stamp = None
        self.target_visual_valid_msg = False
        self.target_visual_confidence = 0.0
        self.target_lidar_pos = None
        self.target_lidar_stamp = None
        self.target_lidar_valid_msg = False
        self.target_lidar_confidence = 0.0
        self.target_fused_pos = None
        self.target_fused_stamp = None
        self.target_fused_valid_msg = False
        self.target_fused_confidence = 0.0
        self.target_fused_source = "none"
        self.visual_lost_count = 0
        self.lidar_fallback_count = 0
        self.fused_none_duration = 0.0
        self.previous_visual_valid = None
        self.previous_fused_source = "none"
        self.last_row_time = None
        self.tracking_state = "LOST"
        self.observation_age = float("nan")
        self.detector_state = "SEARCH"
        self.candidate_count = 0
        self.range_valid_cluster_count = 0
        self.selected_cluster_id = 0
        self.selected_cluster_point_count = 0
        self.selected_cluster_score = float("nan")
        self.second_best_score = float("nan")
        self.association_margin = float("nan")
        self.hit_count = 0
        self.miss_count = 0
        self.reacquire_count = 0
        self.time_since_last_detection = float("nan")
        self.gate_radius = float("nan")
        self.use_estimator_gating = False
        self.selected_distance_to_gate = float("nan")
        self.t_go = float("nan")
        self.v_cmd = (0.0, 0.0, 0.0)
        self.capture_success = False
        self.start_time = rospy.Time.now()

        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "time",
                "uav0_x", "uav0_y", "uav0_z",
                "uav1_x", "uav1_y", "uav1_z",
                "target_est_x", "target_est_y", "target_est_z",
                "target_est_vx", "target_est_vy", "target_est_vz",
                "pred_x", "pred_y", "pred_z",
                "t_go",
                "distance_xy", "distance_z", "distance_3d",
                "v_cmd_x", "v_cmd_y", "v_cmd_z",
                "estimator_model",
                "target_mode",
                "uav1_speed",
                "z_amplitude",
                "capture_success",
                "observation_source",
                "lidar_cloud_topic",
                "target_observation_x", "target_observation_y", "target_observation_z",
                "detection_valid",
                "detection_confidence",
                "detection_point_count",
                "detection_age",
                "target_visual_valid",
                "target_visual_confidence",
                "target_visual_x", "target_visual_y", "target_visual_z",
                "target_visual_error_3d",
                "target_lidar_valid",
                "target_lidar_confidence",
                "target_lidar_x", "target_lidar_y", "target_lidar_z",
                "target_lidar_error_3d",
                "target_fused_valid",
                "target_fused_confidence",
                "target_fused_source",
                "target_fused_x", "target_fused_y", "target_fused_z",
                "target_fused_error_3d",
                "target_estimator_state_x", "target_estimator_state_y", "target_estimator_state_z",
                "target_estimator_error_3d",
                "visual_lost_count",
                "lidar_fallback_count",
                "fused_none_duration",
                "tracking_state",
                "observation_age",
                "detector_state",
                "candidate_count",
                "range_valid_cluster_count",
                "selected_cluster_id",
                "selected_cluster_point_count",
                "selected_cluster_score",
                "second_best_score",
                "association_margin",
                "hit_count",
                "miss_count",
                "reacquire_count",
                "time_since_last_detection",
                "gate_radius",
                "use_estimator_gating",
                "selected_distance_to_gate",
                "target_lost",
                "predict_only",
                "detection_error_3d",
                "detection_error_xy",
                "detection_error_z",
            ],
        )
        self.writer.writeheader()

        rospy.Subscriber(self.uav0_odom_topic, Odometry, self.uav0_cb, queue_size=20)
        rospy.Subscriber(self.uav1_odom_topic, Odometry, self.uav1_cb, queue_size=20)
        rospy.Subscriber(
            self.estimator_state_topic, Odometry, self.estimator_state_cb, queue_size=20
        )
        rospy.Subscriber(self.intercept_topic, PoseStamped, self.intercept_cb, queue_size=20)
        rospy.Subscriber(
            self.target_observation_topic,
            PoseStamped,
            self.target_observation_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.detection_confidence_topic,
            Float32,
            self.detection_confidence_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.detection_valid_topic,
            Bool,
            self.detection_valid_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_visual_topic,
            PoseStamped,
            self.target_visual_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_visual_confidence_topic,
            Float32,
            self.target_visual_confidence_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_visual_valid_topic,
            Bool,
            self.target_visual_valid_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_lidar_topic,
            PoseStamped,
            self.target_lidar_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_lidar_confidence_topic,
            Float32,
            self.target_lidar_confidence_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_lidar_valid_topic,
            Bool,
            self.target_lidar_valid_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_fused_topic,
            PoseStamped,
            self.target_fused_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_fused_confidence_topic,
            Float32,
            self.target_fused_confidence_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_fused_valid_topic,
            Bool,
            self.target_fused_valid_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.target_fused_source_topic,
            String,
            self.target_fused_source_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.detection_point_count_topic,
            UInt32,
            self.detection_point_count_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.tracking_state_topic,
            String,
            self.tracking_state_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            self.observation_age_topic,
            Float32,
            self.observation_age_cb,
            queue_size=20,
        )
        rospy.Subscriber(self.detector_state_topic, String, self.detector_state_cb, queue_size=20)
        rospy.Subscriber(self.candidate_count_topic, UInt32, self.candidate_count_cb, queue_size=20)
        rospy.Subscriber(
            self.range_valid_cluster_count_topic,
            UInt32,
            self.range_valid_cluster_count_cb,
            queue_size=20,
        )
        rospy.Subscriber(self.selected_cluster_id_topic, UInt32, self.selected_cluster_id_cb, queue_size=20)
        rospy.Subscriber(
            self.selected_cluster_point_count_topic,
            UInt32,
            self.selected_cluster_point_count_cb,
            queue_size=20,
        )
        rospy.Subscriber(self.selected_cluster_score_topic, Float32, self.selected_cluster_score_cb, queue_size=20)
        rospy.Subscriber(self.second_best_score_topic, Float32, self.second_best_score_cb, queue_size=20)
        rospy.Subscriber(self.association_margin_topic, Float32, self.association_margin_cb, queue_size=20)
        rospy.Subscriber(self.hit_count_topic, UInt32, self.hit_count_cb, queue_size=20)
        rospy.Subscriber(self.miss_count_topic, UInt32, self.miss_count_cb, queue_size=20)
        rospy.Subscriber(self.reacquire_count_topic, UInt32, self.reacquire_count_cb, queue_size=20)
        rospy.Subscriber(
            self.time_since_last_detection_topic,
            Float32,
            self.time_since_last_detection_cb,
            queue_size=20,
        )
        rospy.Subscriber(self.gate_radius_topic, Float32, self.gate_radius_cb, queue_size=20)
        rospy.Subscriber(self.use_estimator_gating_topic, Bool, self.use_estimator_gating_cb, queue_size=20)
        rospy.Subscriber(
            self.selected_distance_to_gate_topic,
            Float32,
            self.selected_distance_to_gate_cb,
            queue_size=20,
        )
        rospy.Subscriber(self.t_go_topic, Float32, self.t_go_cb, queue_size=20)
        rospy.Subscriber(self.cmd_vel_topic, TwistStamped, self.cmd_vel_cb, queue_size=20)
        rospy.Subscriber(
            self.capture_success_topic, Bool, self.capture_success_cb, queue_size=20
        )

        rospy.loginfo("[ChaseEvaluator] writing %s", self.csv_path)

    def _to_common(self, pos, offset):
        if not self.use_spawn_offsets:
            return pos
        return pos[0] + offset[0], pos[1] + offset[1], pos[2] + offset[2]

    @staticmethod
    def _odom_pos(msg):
        p = msg.pose.pose.position
        return p.x, p.y, p.z

    def uav0_cb(self, msg):
        self.uav0_pos = self._to_common(self._odom_pos(msg), self.uav0_spawn_offset)

    def uav1_cb(self, msg):
        self.uav1_pos = self._to_common(self._odom_pos(msg), self.uav1_spawn_offset)

    def estimator_state_cb(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.target_est_pos = (p.x, p.y, p.z)
        self.target_est_vel = (v.x, v.y, v.z)

    def intercept_cb(self, msg):
        p = msg.pose.position
        self.pred_pos = (p.x, p.y, p.z)

    def target_observation_cb(self, msg):
        p = msg.pose.position
        self.target_observation_pos = (p.x, p.y, p.z)
        self.target_observation_stamp = (
            msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        )

    def detection_confidence_cb(self, msg):
        self.detection_confidence = float(msg.data)

    def detection_valid_cb(self, msg):
        self.detection_valid_msg = bool(msg.data)

    def detection_point_count_cb(self, msg):
        self.detection_point_count = int(msg.data)

    @staticmethod
    def _pose_pos(msg):
        p = msg.pose.position
        return p.x, p.y, p.z

    @staticmethod
    def _pose_stamp(msg):
        return msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def target_visual_cb(self, msg):
        self.target_visual_pos = self._pose_pos(msg)
        self.target_visual_stamp = self._pose_stamp(msg)

    def target_visual_confidence_cb(self, msg):
        self.target_visual_confidence = float(msg.data)

    def target_visual_valid_cb(self, msg):
        self.target_visual_valid_msg = bool(msg.data)

    def target_lidar_cb(self, msg):
        self.target_lidar_pos = self._pose_pos(msg)
        self.target_lidar_stamp = self._pose_stamp(msg)

    def target_lidar_confidence_cb(self, msg):
        self.target_lidar_confidence = float(msg.data)

    def target_lidar_valid_cb(self, msg):
        self.target_lidar_valid_msg = bool(msg.data)

    def target_fused_cb(self, msg):
        self.target_fused_pos = self._pose_pos(msg)
        self.target_fused_stamp = self._pose_stamp(msg)

    def target_fused_confidence_cb(self, msg):
        self.target_fused_confidence = float(msg.data)

    def target_fused_valid_cb(self, msg):
        self.target_fused_valid_msg = bool(msg.data)

    def target_fused_source_cb(self, msg):
        self.target_fused_source = msg.data.strip().lower() or "none"

    def tracking_state_cb(self, msg):
        self.tracking_state = msg.data.strip().upper()

    def observation_age_cb(self, msg):
        self.observation_age = float(msg.data)

    def detector_state_cb(self, msg):
        self.detector_state = msg.data.strip().upper()

    def candidate_count_cb(self, msg):
        self.candidate_count = int(msg.data)

    def range_valid_cluster_count_cb(self, msg):
        self.range_valid_cluster_count = int(msg.data)

    def selected_cluster_id_cb(self, msg):
        self.selected_cluster_id = int(msg.data)

    def selected_cluster_point_count_cb(self, msg):
        self.selected_cluster_point_count = int(msg.data)

    def selected_cluster_score_cb(self, msg):
        self.selected_cluster_score = float(msg.data)

    def second_best_score_cb(self, msg):
        self.second_best_score = float(msg.data)

    def association_margin_cb(self, msg):
        self.association_margin = float(msg.data)

    def hit_count_cb(self, msg):
        self.hit_count = int(msg.data)

    def miss_count_cb(self, msg):
        self.miss_count = int(msg.data)

    def reacquire_count_cb(self, msg):
        self.reacquire_count = int(msg.data)

    def time_since_last_detection_cb(self, msg):
        self.time_since_last_detection = float(msg.data)

    def gate_radius_cb(self, msg):
        self.gate_radius = float(msg.data)

    def use_estimator_gating_cb(self, msg):
        self.use_estimator_gating = bool(msg.data)

    def selected_distance_to_gate_cb(self, msg):
        self.selected_distance_to_gate = float(msg.data)

    def t_go_cb(self, msg):
        self.t_go = float(msg.data)

    def cmd_vel_cb(self, msg):
        v = msg.twist.linear
        self.v_cmd = (v.x, v.y, v.z)

    def capture_success_cb(self, msg):
        self.capture_success = bool(msg.data)

    @staticmethod
    def _distances(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        distance_xy = (dx * dx + dy * dy) ** 0.5
        distance_3d = (dx * dx + dy * dy + dz * dz) ** 0.5
        return distance_xy, abs(dz), distance_3d

    def _fresh_sensor_valid(self, pos, stamp, valid_msg):
        if pos is None or not valid_msg:
            return False
        if stamp is None or self.sensor_valid_age <= 0.0:
            return True
        return (rospy.Time.now() - stamp).to_sec() <= self.sensor_valid_age

    def _error_3d(self, pos, valid):
        if not valid or pos is None or self.uav1_pos is None:
            return float("nan")
        return self._distances(pos, self.uav1_pos)[2]

    @staticmethod
    def _safe_pos(pos):
        return pos or (float("nan"), float("nan"), float("nan"))

    def write_row(self):
        if self.uav0_pos is None or self.uav1_pos is None:
            rospy.loginfo_throttle(3.0, "[ChaseEvaluator] waiting for UAV odometry")
            return
        target_est_pos = self.target_est_pos or (float("nan"), float("nan"), float("nan"))
        target_est_vel = self.target_est_vel or (float("nan"), float("nan"), float("nan"))
        pred_pos = self.pred_pos or (float("nan"), float("nan"), float("nan"))
        obs_pos = self.target_observation_pos or (
            float("nan"),
            float("nan"),
            float("nan"),
        )
        detection_valid = self.target_observation_pos is not None and self.detection_valid_msg
        if self.target_observation_stamp is not None:
            detection_age = max((rospy.Time.now() - self.target_observation_stamp).to_sec(), 0.0)
        else:
            detection_age = float("nan")
        if detection_valid and self.uav1_pos is not None:
            detection_error_xy, detection_error_z, detection_error_3d = self._distances(
                obs_pos, self.uav1_pos
            )
        else:
            detection_error_xy = float("nan")
            detection_error_z = float("nan")
            detection_error_3d = float("nan")
        visual_valid = self._fresh_sensor_valid(
            self.target_visual_pos,
            self.target_visual_stamp,
            self.target_visual_valid_msg,
        )
        lidar_valid = self._fresh_sensor_valid(
            self.target_lidar_pos,
            self.target_lidar_stamp,
            self.target_lidar_valid_msg,
        )
        fused_valid = self._fresh_sensor_valid(
            self.target_fused_pos,
            self.target_fused_stamp,
            self.target_fused_valid_msg,
        )
        visual_pos = self._safe_pos(self.target_visual_pos)
        lidar_pos = self._safe_pos(self.target_lidar_pos)
        fused_pos = self._safe_pos(self.target_fused_pos)
        visual_error_3d = self._error_3d(self.target_visual_pos, visual_valid)
        lidar_error_3d = self._error_3d(self.target_lidar_pos, lidar_valid)
        fused_error_3d = self._error_3d(self.target_fused_pos, fused_valid)
        target_estimator_error_3d = self._error_3d(
            self.target_est_pos, self.target_est_pos is not None
        )
        distance_xy, distance_z, distance_3d = self._distances(self.uav0_pos, self.uav1_pos)
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        row_dt = (
            max(elapsed - self.last_row_time, 0.0)
            if self.last_row_time is not None
            else 0.0
        )
        if self.previous_visual_valid is True and not visual_valid:
            self.visual_lost_count += 1
        fused_source_for_stats = (
            self.target_fused_source if fused_valid else "none"
        )
        if self.previous_fused_source != "lidar" and fused_source_for_stats == "lidar":
            self.lidar_fallback_count += 1
        if fused_source_for_stats == "none":
            self.fused_none_duration += row_dt
        self.previous_visual_valid = visual_valid
        self.previous_fused_source = fused_source_for_stats
        self.last_row_time = elapsed
        self.writer.writerow(
            {
                "time": elapsed,
                "uav0_x": self.uav0_pos[0],
                "uav0_y": self.uav0_pos[1],
                "uav0_z": self.uav0_pos[2],
                "uav1_x": self.uav1_pos[0],
                "uav1_y": self.uav1_pos[1],
                "uav1_z": self.uav1_pos[2],
                "target_est_x": target_est_pos[0],
                "target_est_y": target_est_pos[1],
                "target_est_z": target_est_pos[2],
                "target_est_vx": target_est_vel[0],
                "target_est_vy": target_est_vel[1],
                "target_est_vz": target_est_vel[2],
                "pred_x": pred_pos[0],
                "pred_y": pred_pos[1],
                "pred_z": pred_pos[2],
                "t_go": self.t_go,
                "distance_xy": distance_xy,
                "distance_z": distance_z,
                "distance_3d": distance_3d,
                "v_cmd_x": self.v_cmd[0],
                "v_cmd_y": self.v_cmd[1],
                "v_cmd_z": self.v_cmd[2],
                "estimator_model": self.estimator_model,
                "target_mode": self.target_mode,
                "uav1_speed": self.uav1_speed,
                "z_amplitude": self.z_amplitude,
                "capture_success": int(self.capture_success),
                "observation_source": self.observation_source,
                "lidar_cloud_topic": self.lidar_cloud_topic,
                "target_observation_x": obs_pos[0],
                "target_observation_y": obs_pos[1],
                "target_observation_z": obs_pos[2],
                "detection_valid": int(detection_valid),
                "detection_confidence": self.detection_confidence,
                "detection_point_count": self.detection_point_count,
                "detection_age": detection_age,
                "target_visual_valid": int(visual_valid),
                "target_visual_confidence": self.target_visual_confidence,
                "target_visual_x": visual_pos[0],
                "target_visual_y": visual_pos[1],
                "target_visual_z": visual_pos[2],
                "target_visual_error_3d": visual_error_3d,
                "target_lidar_valid": int(lidar_valid),
                "target_lidar_confidence": self.target_lidar_confidence,
                "target_lidar_x": lidar_pos[0],
                "target_lidar_y": lidar_pos[1],
                "target_lidar_z": lidar_pos[2],
                "target_lidar_error_3d": lidar_error_3d,
                "target_fused_valid": int(fused_valid),
                "target_fused_confidence": self.target_fused_confidence,
                "target_fused_source": fused_source_for_stats,
                "target_fused_x": fused_pos[0],
                "target_fused_y": fused_pos[1],
                "target_fused_z": fused_pos[2],
                "target_fused_error_3d": fused_error_3d,
                "target_estimator_state_x": target_est_pos[0],
                "target_estimator_state_y": target_est_pos[1],
                "target_estimator_state_z": target_est_pos[2],
                "target_estimator_error_3d": target_estimator_error_3d,
                "visual_lost_count": self.visual_lost_count,
                "lidar_fallback_count": self.lidar_fallback_count,
                "fused_none_duration": self.fused_none_duration,
                "tracking_state": self.tracking_state,
                "observation_age": self.observation_age,
                "detector_state": self.detector_state,
                "candidate_count": self.candidate_count,
                "range_valid_cluster_count": self.range_valid_cluster_count,
                "selected_cluster_id": self.selected_cluster_id,
                "selected_cluster_point_count": self.selected_cluster_point_count,
                "selected_cluster_score": self.selected_cluster_score,
                "second_best_score": self.second_best_score,
                "association_margin": self.association_margin,
                "hit_count": self.hit_count,
                "miss_count": self.miss_count,
                "reacquire_count": self.reacquire_count,
                "time_since_last_detection": self.time_since_last_detection,
                "gate_radius": self.gate_radius,
                "use_estimator_gating": int(self.use_estimator_gating),
                "selected_distance_to_gate": self.selected_distance_to_gate,
                "target_lost": int(self.tracking_state == "LOST"),
                "predict_only": int(self.tracking_state == "PREDICT_ONLY"),
                "detection_error_3d": detection_error_3d,
                "detection_error_xy": detection_error_xy,
                "detection_error_z": detection_error_z,
            }
        )
        self.csv_file.flush()

    def run(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            self.write_row()
            rate.sleep()
        self.csv_file.close()


def main():
    rospy.init_node("chase_evaluator_node")
    ChaseEvaluator().run()


if __name__ == "__main__":
    main()
