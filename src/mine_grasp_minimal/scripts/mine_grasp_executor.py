#!/usr/bin/env python3
"""Deterministic, independently implemented phase-1 mine grasp executor.

The perception target is the only source of a grasp position.  Gazebo state is
used after the final visual TCP gate to validate lift/hold and, in simulation,
to identify the one task-associated model for an explicit transport lock.  It
never generates or corrects a motion target.  Motion planning is performed
through the public MoveIt actions/services, so this node has no dependency on
moveit_commander or on the repository's legacy grasp wrapper.
"""

import copy
import json
import math
import os
import re
import threading
import time

import actionlib
import numpy as np
import rospy
import tf.transformations as tft
import tf2_geometry_msgs  # noqa: F401 - registers geometry messages with tf2
import tf2_ros
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState,
    JointTolerance,
)
from actionlib_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gazebo_grasp_plugin_ros.msg import GazeboGraspEvent
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from mobile_manipulator.msg import (
    MineGraspAction,
    MineGraspFeedback,
    MineGraspResult,
)
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    ExecuteTrajectoryAction,
    ExecuteTrajectoryGoal,
    JointConstraint,
    MoveGroupAction,
    MoveGroupGoal,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    ApplyPlanningSceneRequest,
    GetCartesianPath,
    GetCartesianPathRequest,
    GetPlanningScene,
    GetPlanningSceneRequest,
    GetPositionFK,
    GetPositionFKRequest,
    GetPositionIK,
    GetPositionIKRequest,
    GetStateValidity,
    GetStateValidityRequest,
)
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker, MarkerArray

from mine_grasp_minimal.motion_stability import (
    JointSpanWindow,
    MotionExecutionResult,
    classify_stability,
    maximum_absolute,
)


FAILURE_CODES = {
    "NO_DETECTION",
    "NO_DETONATOR",
    "INVALID_DEPTH",
    "TF_FAILED",
    "TARGET_UNSTABLE",
    "IK_FAILED",
    "COLLISION_FAILED",
    "PLANNING_FAILED",
    "CONTROLLER_UNSETTLED",
    "TRUE_POSITION_ERROR",
    "TCP_NOT_REACHED",
    "MOTION_TIMEOUT",
    "INTERNAL_ERROR",
    "TARGET_SHIFT_EXCESSIVE",
    "TARGET_OUT_OF_WORKSPACE",
    "PREGRASP_FAILED",
    "APPROACH_FAILED",
    "GRIPPER_FAILED",
    "EMPTY_GRASP",
    "GAZEBO_ATTACH_FAILED",
    "MOVEIT_ATTACH_FAILED",
    "LIFT_FAILED",
    "OBJECT_DROPPED",
    "BASE_UNSTABLE",
    "TRANSPORT_FAILED",
    "PLACE_FAILED",
    "CANCELLED",
}


class GraspFailure(RuntimeError):
    def __init__(self, code, detail):
        if code not in FAILURE_CODES:
            raise ValueError("unknown grasp failure code: {}".format(code))
        super().__init__("{}: {}".format(code, detail))
        self.code = code
        self.detail = detail


def _pose_matrix(pose):
    matrix = tft.quaternion_matrix([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ])
    matrix[0:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def _rotation_angle(matrix):
    value = max(-1.0, min(1.0, (float(np.trace(matrix[0:3, 0:3])) - 1.0) * 0.5))
    return math.acos(value)


def _wrap_pi(value):
    return math.atan2(math.sin(value), math.cos(value))


class MineGraspExecutor:
    def __init__(self):
        self.lock = threading.RLock()
        self.execution_lock = threading.Lock()
        self.target_pose = None
        self.target_observation = None
        self.target_valid = False
        self.target_confidence = 0.0
        self.target_std = [float("inf")] * 3
        self.localizer_status = "WAITING_FOR_CAMERA"
        self.joint_state = None
        self.arm_state = None
        self.gripper_state = None
        self.imu = None
        self.odom = None
        self.gazebo_attached = False
        self.gazebo_attached_object = ""
        self.attached_model_name = ""
        self.forced_lock_active = False
        self.forced_lock_object = ""
        self.pending_forced_lock_object = ""
        self.simulated_lock_visual_gate_pose = None
        self.retained = False
        # After a verified PICK the integrated mission may remove the carried
        # Gazebo entity entirely.  This avoids presenting the mine fixed below
        # the wrist as a permanent near-field obstacle to move_base.  The
        # transaction retains the exact model name/SDF and recreates it only at
        # the verified disposal pose during PLACE.
        self.virtual_transport_active = False
        self.virtual_model_name = ""
        self.virtual_source_pose = None
        self.virtual_model_present = False
        self.secure_gripper_position = None
        self.transport_relative_reference = None
        self.releasing = False
        # Once PLACE has proved an initial detach and removed the planning-
        # scene attachment, any new grasp-fix attach must stop the retreat.
        # Otherwise the arm could unknowingly lift the mine again while
        # MoveIt models the gripper as empty.
        self.release_motion_guard = False
        self.action_active = False
        self.last_grasp_event_wall = None
        self.model_states = None
        self.last_report = {}
        self.metrics = {}
        self.retreat_joints = None
        self.validation_model_baseline = None
        self.last_cartesian_error = ""
        self.active_source_prior = None
        self.active_source_mine_id = None
        self.active_observation_not_before = rospy.Time(0)
        self.attempt_id = "idle"
        self.clock = None
        self.last_arm_fjt_goal = None
        self.last_arm_fjt_goal_wall = 0.0
        self.last_arm_fjt_result = None
        self.last_arm_fjt_result_wall = 0.0
        self.last_gripper_fjt_goal = None
        self.last_gripper_fjt_goal_wall = 0.0
        self.last_gripper_fjt_result = None
        self.last_gripper_fjt_result_wall = 0.0
        self.last_motion_measurements = {}
        self.last_moveit_error_code = None
        self.last_cartesian_fraction = None
        self.trace_path = Path()
        self.trace_path.header.frame_id = self.gravity_frame if hasattr(self, "gravity_frame") else "odom"
        self.debug_poses = {}
        self.observation_base_reference = None

        self.action_name = rospy.get_param("~action_name", "/mine_grasp").rstrip("/")
        if not self.action_name:
            self.action_name = "/mine_grasp"
        # ``<action>/status`` is reserved by actionlib for
        # actionlib_msgs/GoalStatusArray.  Publishing our JSON diagnostics on
        # that same name silently prevents SimpleActionClient.wait_for_server
        # from ever succeeding because the TCPROS message types differ.
        self.executor_status_topic = rospy.get_param(
            "~executor_status_topic", self.action_name + "/executor_status"
        )
        if rospy.resolve_name(self.executor_status_topic) == rospy.resolve_name(
            self.action_name + "/status"
        ):
            raise rospy.ROSInitException(
                "executor_status_topic must not use actionlib-reserved {}".format(
                    self.action_name + "/status"
                )
            )
        self.attempt_output_directory = os.path.abspath(os.path.expanduser(
            rospy.get_param("~attempt_output_directory", "mine_grasp_results")
        ))

        self.arm_group = rospy.get_param("~arm_group", "ur5_arm")
        self.gripper_group = rospy.get_param("~gripper_group", "hand_e_gripper")
        self.planning_frame = rospy.get_param("~planning_frame", "base_link")
        self.arm_base_frame = rospy.get_param("~arm_base_frame", "ur5_base_link")
        self.gravity_frame = rospy.get_param("~gravity_frame", "odom")
        self.trace_path.header.frame_id = self.gravity_frame
        # gazebo_msgs/ModelStates is expressed in the Gazebo world frame.  In
        # this project that frame is aligned with ROS ``map``; it is not the
        # UGV-local ``odom`` frame after the robot has moved away from spawn.
        # This frame is validation/lock-identity-only and never feeds grasp
        # generation.
        self.gazebo_validation_frame = rospy.get_param(
            "~gazebo_validation_frame", "map"
        )
        self.tcp_link = rospy.get_param("~tcp_link", "grasp_tcp")
        self.arm_joint_names = list(rospy.get_param("~arm_joint_names"))
        self.gripper_joint = rospy.get_param("~gripper_joint_name", "finger_joint")

        self.look_joints = list(rospy.get_param("~look_joint_positions"))
        self.stow_joints = list(rospy.get_param("~stow_joint_positions"))
        self.look_duration = float(rospy.get_param("~look_motion_duration", 15.0))
        self.reset_duration = float(rospy.get_param("~reset_motion_duration", 15.0))
        # This bounds MoveIt's sampled goal region.  Keep it distinct from the
        # deliberately wider controller/FJT tolerance: the measured TCP gate
        # cannot correct a plan whose legal endpoint is already centimetres
        # away from the requested grasp pose.
        self.joint_tolerance = float(
            rospy.get_param("~joint_goal_tolerance", 0.002)
        )
        self.hold_tolerance = float(rospy.get_param("~joint_hold_tolerance", 0.025))

        self.velocity_scale = float(rospy.get_param("~arm_velocity_scale", 0.08))
        self.acceleration_scale = float(
            rospy.get_param("~arm_acceleration_scale", 0.04)
        )
        self.approach_speed = float(rospy.get_param("~approach_speed", 0.02))
        self.lift_speed = float(rospy.get_param("~lift_speed", 0.03))
        self.cartesian_step = float(rospy.get_param("~cartesian_step", 0.005))
        self.min_cartesian_fraction = float(
            rospy.get_param("~minimum_cartesian_fraction", 0.995)
        )
        self.planning_attempts = int(rospy.get_param("~planning_attempts", 5))
        self.planning_time = float(rospy.get_param("~planning_time", 8.0))
        self.ik_timeout = float(rospy.get_param("~ik_timeout", 0.75))

        self.pregrasp_distance = float(rospy.get_param("~pregrasp_distance", 0.12))
        self.approach_distance = float(rospy.get_param("~approach_distance", 0.08))
        self.lift_distance = float(rospy.get_param("~lift_distance", 0.08))
        self.yaws = [math.radians(float(value)) for value in
                      rospy.get_param("~candidate_yaws_deg", [0.0, 90.0])]
        self.tilts = [math.radians(float(value)) for value in
                       rospy.get_param("~candidate_tilts_deg", [0.0, 10.0, -10.0])]
        self.minimum_elbow_bend = float(
            rospy.get_param("~minimum_elbow_bend", 0.35)
        )
        self.joint_limit_margin = float(rospy.get_param("~joint_limit_margin", 0.12))
        self.maximum_planar_reach = float(
            rospy.get_param("~maximum_target_planar_reach", 0.58)
        )
        self.source_prior_xy_tolerance = float(
            rospy.get_param("~source_prior_xy_tolerance", 0.15)
        )
        if (
            not math.isfinite(self.source_prior_xy_tolerance)
            or self.source_prior_xy_tolerance <= 0.0
        ):
            raise rospy.ROSInitException(
                "source_prior_xy_tolerance must be finite and positive"
            )

        # A small positive opening avoids exciting the 2F-140 linkage at its
        # exact lower hard stop while retaining ample clearance for the fuse.
        self.gripper_open = float(rospy.get_param("~gripper_open_position", 0.080))
        self.gripper_closed = float(rospy.get_param("~gripper_closed_position", 0.500))
        self.gripper_close_positions = [
            float(value) for value in rospy.get_param(
                "~gripper_close_positions",
                [0.480, 0.490, 0.495, 0.498, 0.499, 0.500],
            )
        ]
        self.gripper_contact_squeeze = float(
            rospy.get_param("~gripper_contact_squeeze_delta", 0.003)
        )
        self.gripper_secure_speed = float(
            rospy.get_param("~gripper_secure_joint_speed", 0.01)
        )
        self.gripper_secure_hold = float(
            rospy.get_param("~gripper_secure_hold_duration", 1.0)
        )
        self.gripper_secure_timeout = float(
            rospy.get_param("~gripper_secure_timeout", 2.5)
        )
        self.gripper_close_speed = float(
            rospy.get_param("~gripper_close_joint_speed", 0.06)
        )
        self.gripper_close_min_duration = float(
            rospy.get_param("~gripper_close_min_stage_duration", 0.60)
        )
        self.gripper_contact_settle = float(
            rospy.get_param("~gripper_contact_settle_duration", 0.35)
        )
        self.gripper_duration = float(rospy.get_param("~gripper_motion_duration", 4.0))
        self.gripper_tolerance = float(rospy.get_param("~gripper_goal_tolerance", 0.04))
        self.gripper_terminal_position = float(
            rospy.get_param("~gripper_terminal_position_tolerance", 0.01)
        )
        self.gripper_terminal_velocity = float(
            rospy.get_param("~gripper_terminal_velocity_tolerance", 0.02)
        )
        self.gripper_contact_raw_velocity = float(
            rospy.get_param("~gripper_contact_raw_velocity_tolerance", 0.05)
        )
        self.empty_tolerance = float(rospy.get_param("~empty_position_tolerance", 0.025))
        self.gripper_reset_attempts = int(
            rospy.get_param("~gripper_reset_open_attempts", 3)
        )
        self.gripper_reset_open_tolerance = float(
            rospy.get_param("~gripper_reset_open_tolerance", 0.030)
        )
        self.gripper_reset_retry_delay = float(
            rospy.get_param("~gripper_reset_retry_delay", 0.30)
        )

        self.mine_model = rospy.get_param("~mine_model_name", "landmine_test")
        self.mine_model_prefix = rospy.get_param(
            "~mine_model_prefix", "landmine"
        )
        self.moveit_object_id = rospy.get_param(
            "~moveit_object_id", "detected_landmine"
        )
        self.disc_radius = float(rospy.get_param("~mine_disc_radius", 0.068))
        self.disc_height = float(rospy.get_param("~mine_disc_height", 0.025))
        self.detonator_size = [float(value) for value in
                               rospy.get_param("~mine_detonator_size",
                                               [0.04, 0.04, 0.06])]
        self.detonator_center_z = float(
            rospy.get_param("~mine_detonator_center_z", 0.055)
        )
        self.grasp_height_bias = float(
            rospy.get_param("~grasp_contact_height_bias", 0.008)
        )
        self.touch_links = list(rospy.get_param("~touch_links", []))
        # Gazebo-only retention fallback.  The command is not available until
        # the visually generated final grasp pose has passed its strict TCP
        # component gate.  Model truth selects only the fixed-joint identity;
        # it is prohibited from changing the requested grasp pose.
        self.simulated_lock_fallback_enabled = bool(rospy.get_param(
            "~simulated_lock_fallback_enabled", True
        ))
        self.force_attach_topic = rospy.get_param(
            "~force_attach_topic", "/mine_grasp/force_attach"
        )
        self.force_detach_topic = rospy.get_param(
            "~force_detach_topic", "/mine_grasp/force_detach"
        )
        self.force_lock_status_topic = rospy.get_param(
            "~force_lock_status_topic", "/mine_grasp/force_lock_status"
        )
        self.simulated_lock_collision_suffix = str(rospy.get_param(
            "~simulated_lock_collision_suffix", "body::detonator_collision"
        )).strip(":")
        self.simulated_lock_tcp_xy = float(rospy.get_param(
            "~simulated_lock_tcp_xy_tolerance", 0.075
        ))
        self.simulated_lock_vertical = float(rospy.get_param(
            "~simulated_lock_vertical_tolerance", 0.025
        ))
        self.simulated_lock_source_xy = float(rospy.get_param(
            "~simulated_lock_source_xy_tolerance", 0.15
        ))
        self.simulated_lock_min_separation = float(rospy.get_param(
            "~simulated_lock_min_candidate_separation", 0.25
        ))
        self.simulated_lock_request_timeout = float(rospy.get_param(
            "~simulated_lock_request_timeout", 3.0
        ))
        self.simulated_lock_trigger_stage = int(rospy.get_param(
            "~simulated_lock_trigger_stage", 0
        ))
        self.simulated_lock_before_close = bool(rospy.get_param(
            "~simulated_lock_before_close", True
        ))
        self.simulated_lock_close_speed = float(rospy.get_param(
            "~simulated_lock_close_joint_speed", 0.10
        ))
        self.virtual_transport_enabled = bool(rospy.get_param(
            "~virtual_transport_enabled", False
        ))
        self.landmine_sdf_path = os.path.abspath(os.path.expanduser(
            rospy.get_param(
                "~landmine_sdf_path",
                "src/mobile_manipulator/gazebo_models/landmine/model.sdf",
            )
        ))
        self.gazebo_delete_model_service_name = rospy.get_param(
            "~gazebo_delete_model_service", "/gazebo/delete_model"
        )
        self.gazebo_spawn_model_service_name = rospy.get_param(
            "~gazebo_spawn_model_service", "/gazebo/spawn_sdf_model"
        )
        self.gazebo_spawn_reference_frame = rospy.get_param(
            "~gazebo_spawn_reference_frame", "world"
        )
        self.virtual_model_service_timeout = float(rospy.get_param(
            "~virtual_model_service_timeout", 5.0
        ))
        self.virtual_release_settle = float(rospy.get_param(
            "~virtual_release_settle_duration", 0.10
        ))
        # In virtual transport mode the verified source model is sealed before
        # the arm rises.  Keeping the fixed mine in contact with the terrain
        # during the old physical lift formed a ground--mine--wrist constraint
        # which pitched the UGV forward.  This short hold proves the lock while
        # the chassis is still stationary, and the clearance motion happens
        # only after the source entity has been removed.
        self.capture_preseal_hold = float(rospy.get_param(
            "~capture_preseal_hold_duration", 0.15
        ))
        self.virtual_post_capture_clearance = float(rospy.get_param(
            "~virtual_post_capture_clearance", 0.05
        ))
        self.landmine_sdf_xml = ""
        if self.virtual_transport_enabled:
            try:
                with open(self.landmine_sdf_path, "r", encoding="utf-8") as stream:
                    self.landmine_sdf_xml = stream.read()
            except (OSError, UnicodeError) as exc:
                raise rospy.ROSInitException(
                    "virtual transport cannot read landmine SDF {}: {}".format(
                        self.landmine_sdf_path, exc
                    )
                )
            if "<model" not in self.landmine_sdf_xml:
                raise rospy.ROSInitException(
                    "landmine SDF does not contain a model: {}".format(
                        self.landmine_sdf_path
                    )
                )

        self.target_wait_timeout = float(rospy.get_param("~target_wait_timeout", 12.0))
        self.target_max_age = float(rospy.get_param("~target_max_age", 1.0))
        self.target_update_wait = float(rospy.get_param("~target_update_wait", 15.0))
        self.stationary_linear = float(
            rospy.get_param("~base_stationary_linear", 0.01)
        )
        self.stationary_angular = float(
            rospy.get_param("~base_stationary_angular", 0.02)
        )
        self.stationary_duration = float(
            rospy.get_param("~base_stationary_duration", 1.0)
        )
        self.max_roll = math.radians(float(rospy.get_param("~max_base_roll_deg", 5.0)))
        self.max_pitch = math.radians(float(rospy.get_param("~max_base_pitch_deg", 5.0)))
        self.hold_duration = float(rospy.get_param("~hold_duration", 3.0))
        self.grasp_event_timeout = float(rospy.get_param("~grasp_event_timeout", 4.0))
        self.grasp_release_timeout = float(
            rospy.get_param("~grasp_release_timeout", 12.0)
        )
        self.grasp_release_settle = float(
            rospy.get_param("~grasp_release_settle_duration", 1.0)
        )
        self.relative_translation_tolerance = float(
            rospy.get_param("~relative_pose_translation_tolerance", 0.015)
        )
        self.relative_rotation_tolerance = math.radians(float(
            rospy.get_param("~relative_pose_rotation_tolerance_deg", 8.0)
        ))
        self.minimum_lift_height = float(
            rospy.get_param("~minimum_lift_height", 0.045)
        )
        self.timeout_margin = float(rospy.get_param("~motion_timeout_margin", 8.0))
        self.action_wall_timeout_scale = float(
            rospy.get_param("~action_wall_timeout_scale", 3.0)
        )
        self.action_result_grace = float(
            rospy.get_param("~action_result_grace", 1.0)
        )
        self.cartesian_reached_position_tolerance = float(
            rospy.get_param("~cartesian_reached_position_tolerance", 0.012)
        )
        self.cartesian_reached_orientation_tolerance = math.radians(float(
            rospy.get_param("~cartesian_reached_orientation_tolerance_deg", 5.0)
        ))
        self.cartesian_timeout_settle = float(
            rospy.get_param("~cartesian_timeout_settle", 0.75)
        )
        self.motion_settle_timeout = float(
            rospy.get_param("~motion_settle_timeout", 3.0)
        )
        self.terminal_joint_error = float(
            rospy.get_param("~terminal_joint_error_tolerance", 0.01)
        )
        self.terminal_joint_velocity = float(
            rospy.get_param("~terminal_joint_velocity_tolerance", 0.05)
        )
        # gazebo_ros_control can report a non-zero instantaneous velocity field
        # while the sampled joint positions are stationary.  This bound is not
        # a motion tolerance: it only limits when the position-window derivative
        # may replace a demonstrably inconsistent controller velocity sample.
        self.terminal_velocity_field_disagreement_limit = float(
            rospy.get_param(
                "~terminal_velocity_field_disagreement_limit", 0.08
            )
        )
        self.terminal_joint_span = float(
            rospy.get_param("~terminal_joint_span_tolerance", 0.002)
        )
        self.terminal_stability_duration = float(
            rospy.get_param("~terminal_stability_duration", 0.5)
        )
        self.terminal_tcp_position = float(
            rospy.get_param("~terminal_tcp_position_tolerance", 0.008)
        )
        self.terminal_tcp_orientation = math.radians(float(
            rospy.get_param("~terminal_tcp_orientation_tolerance_deg", 3.0)
        ))
        self.final_tcp_lateral = float(
            rospy.get_param("~final_tcp_lateral_tolerance", 0.008)
        )
        self.final_tcp_vertical = float(
            rospy.get_param("~final_tcp_vertical_tolerance", 0.004)
        )
        self.explicit_goal_tolerance = float(
            rospy.get_param("~explicit_fjt_goal_tolerance", 0.01)
        )
        self.minimum_explicit_goal_tolerance = float(
            rospy.get_param("~minimum_explicit_fjt_goal_tolerance", 0.005)
        )
        self.relocalize_small = float(
            rospy.get_param("~relocalize_small_update", 0.020)
        )
        self.relocalize_maximum = float(
            rospy.get_param("~relocalize_maximum_update", 0.050)
        )
        self.maximum_coarse_replans = int(
            rospy.get_param("~maximum_coarse_replans", 1)
        )
        self.maximum_pregrasp_replans = int(
            rospy.get_param("~maximum_pregrasp_replans", 1)
        )
        # A collision-free analytic endpoint does not guarantee that MoveIt's
        # Cartesian interpolator can keep the same IK branch for the complete
        # final descent.  If one orientation returns a partial path, retry a
        # bounded number of the *other* collision-checked candidates in the
        # same PICK instead of retracting and deferring the whole mine.
        self.maximum_final_candidate_retries = int(
            rospy.get_param("~maximum_final_candidate_retries", 2)
        )
        self.near_field_relocalization_enabled = bool(
            rospy.get_param("~near_field_relocalization_enabled", False)
        )
        self.maximum_base_translation_after_observation = float(
            rospy.get_param(
                "~maximum_base_translation_after_observation", 0.005
            )
        )
        self.maximum_base_yaw_after_observation = math.radians(float(
            rospy.get_param(
                "~maximum_base_yaw_after_observation_deg", 1.0
            )
        ))
        self.physical_enabled = bool(
            rospy.get_param("~physical_grasp_enabled", False)
        )
        self.dry_clearance = float(rospy.get_param("~dry_run_clearance", 0.10))
        self.transport_xyz = [float(value) for value in rospy.get_param(
            "~transport_pose_xyz", [0.36, 0.0, 0.05]
        )]
        raw_transport_candidates = rospy.get_param(
            "~transport_pose_candidates",
            [self.transport_xyz, [0.40, 0.0, 0.00], [0.40, 0.0, 0.05],
             [0.44, 0.0, 0.00]],
        )
        self.transport_candidates = [
            [float(value) for value in candidate]
            for candidate in raw_transport_candidates
        ]
        if self.transport_xyz not in self.transport_candidates:
            self.transport_candidates.insert(0, list(self.transport_xyz))
        self.transport_rpy = [float(value) for value in rospy.get_param(
            "~transport_pose_rpy", [math.pi, 0.0, math.pi / 2.0]
        )]
        self.transport_hold_duration = float(
            rospy.get_param("~transport_hold_duration", 1.0)
        )
        self.place_speed = float(rospy.get_param("~place_speed", 0.02))
        self.place_retreat_distance = float(
            rospy.get_param("~place_retreat_distance", 0.12)
        )
        self.place_validation_xy = float(
            rospy.get_param("~place_validation_xy_tolerance", 0.12)
        )
        self.place_validation_z = float(
            rospy.get_param("~place_validation_z_tolerance", 0.08)
        )
        self.place_settle_motion = float(
            rospy.get_param("~place_settle_motion_tolerance", 0.01)
        )
        self.transport_gripper_drift = float(
            rospy.get_param("~transport_gripper_drift_tolerance", 0.020)
        )
        self.transport_hold_refresh = float(
            rospy.get_param("~transport_hold_refresh", 1.0)
        )

        if len(self.arm_joint_names) != 6 or len(self.look_joints) != 6:
            raise rospy.ROSInitException("UR5 arm joint and look arrays must contain six values")
        if not 0.05 <= self.pregrasp_distance <= 0.25:
            raise rospy.ROSInitException("pregrasp_distance outside phase-1 safety range")
        if not 0.05 <= self.lift_distance <= 0.08:
            raise rospy.ROSInitException("lift_distance must remain in the 5-8 cm gate")
        if not 0.05 <= self.capture_preseal_hold <= 0.50:
            raise rospy.ROSInitException(
                "capture_preseal_hold_duration must remain in the 0.05-0.50 s gate"
            )
        if not 0.03 <= self.virtual_post_capture_clearance <= self.lift_distance:
            raise rospy.ROSInitException(
                "virtual_post_capture_clearance must be 0.03 m through lift_distance"
            )
        if self.approach_distance >= self.pregrasp_distance:
            raise rospy.ROSInitException("approach_distance must be below pregrasp_distance")
        if not 0.0 < self.velocity_scale <= 0.10:
            raise rospy.ROSInitException("arm_velocity_scale exceeds phase-1 limit")
        if not 0.0 < self.acceleration_scale <= 0.05:
            raise rospy.ROSInitException("arm_acceleration_scale exceeds phase-1 limit")
        if (not self.gripper_close_positions or
                any(value <= self.gripper_open or value > 0.725
                    for value in self.gripper_close_positions) or
                any(b <= a for a, b in zip(
                    self.gripper_close_positions,
                    self.gripper_close_positions[1:],
                ))):
            raise rospy.ROSInitException(
                "gripper_close_positions must be strictly increasing within joint limits"
            )
        if abs(self.gripper_close_positions[-1] - self.gripper_closed) > 1e-6:
            raise rospy.ROSInitException(
                "gripper_closed_position must equal the final staged-close position"
            )
        if not 0.001 <= self.gripper_contact_squeeze <= 0.006:
            raise rospy.ROSInitException(
                "gripper_contact_squeeze_delta outside the 0.001-0.006 safe range"
            )
        if not 0.0 < self.gripper_secure_speed <= 0.02:
            raise rospy.ROSInitException("gripper secure speed exceeds 0.02 rad/s")
        if not 0.0 < self.gripper_close_speed <= 0.10:
            raise rospy.ROSInitException("gripper close speed exceeds phase-1 limit")
        if not 0.0 < self.gripper_terminal_position <= 0.01:
            raise rospy.ROSInitException("gripper terminal position must be <= 0.01 rad")
        if not 0.0 < self.gripper_terminal_velocity <= 0.02:
            raise rospy.ROSInitException("gripper terminal velocity must be <= 0.02 rad/s")
        if not (self.gripper_terminal_velocity
                <= self.gripper_contact_raw_velocity <= 0.05):
            raise rospy.ROSInitException(
                "gripper contact raw velocity must be between the terminal "
                "velocity and 0.05 rad/s"
            )
        if not 0.0 <= self.grasp_height_bias <= 0.015:
            raise rospy.ROSInitException(
                "grasp_contact_height_bias outside geometry-safe range"
            )
        if not self.simulated_lock_collision_suffix:
            raise rospy.ROSInitException(
                "simulated_lock_collision_suffix must not be empty"
            )
        if not 0.02 <= self.simulated_lock_tcp_xy <= 0.10:
            raise rospy.ROSInitException(
                "simulated lock TCP XY tolerance outside 0.02-0.10 m"
            )
        if not 0.005 <= self.simulated_lock_vertical <= 0.04:
            raise rospy.ROSInitException(
                "simulated lock vertical tolerance outside 0.005-0.04 m"
            )
        if not 0.02 <= self.simulated_lock_source_xy <= self.source_prior_xy_tolerance:
            raise rospy.ROSInitException(
                "simulated lock source tolerance must be within source-prior gate"
            )
        if not 0.15 <= self.simulated_lock_min_separation <= 1.0:
            raise rospy.ROSInitException(
                "simulated lock candidate separation outside 0.15-1.0 m"
            )
        if not 0.5 <= self.simulated_lock_request_timeout <= 3.0:
            raise rospy.ROSInitException(
                "simulated lock request timeout outside 0.5-3.0 s"
            )
        if not 0 <= self.simulated_lock_trigger_stage < len(self.gripper_close_positions):
            raise rospy.ROSInitException(
                "simulated lock trigger stage is outside staged-close positions"
            )
        if not 0.0 < self.simulated_lock_close_speed <= 0.10:
            raise rospy.ROSInitException(
                "simulated lock close speed must be within 0-0.10 rad/s"
            )
        if not 1.0 <= self.action_wall_timeout_scale <= 5.0:
            raise rospy.ROSInitException("action_wall_timeout_scale outside safe range")
        if not 0.1 <= self.action_result_grace <= 2.0:
            raise rospy.ROSInitException("action_result_grace outside 0.1-2.0 s")
        if not 0.003 <= self.cartesian_reached_position_tolerance <= 0.02:
            raise rospy.ROSInitException(
                "cartesian_reached_position_tolerance outside 3-20 mm"
            )
        if not 0.0 <= self.cartesian_timeout_settle <= 2.0:
            raise rospy.ROSInitException("cartesian_timeout_settle outside 0-2 s")
        if not 0.5 <= self.motion_settle_timeout <= 3.0:
            raise rospy.ROSInitException("motion_settle_timeout outside 0.5-3.0 s")
        if not 0.0 < self.terminal_joint_error <= 0.01:
            raise rospy.ROSInitException("terminal joint error must be <= 0.01 rad")
        if not 0.0 < self.terminal_joint_velocity <= 0.05:
            raise rospy.ROSInitException("terminal joint velocity must be <= 0.05 rad/s")
        if not (self.terminal_joint_velocity
                <= self.terminal_velocity_field_disagreement_limit <= 0.10):
            raise rospy.ROSInitException(
                "terminal velocity field disagreement limit must be between "
                "the terminal velocity tolerance and 0.10 rad/s"
            )
        if not 0.0 < self.terminal_joint_span <= 0.002:
            raise rospy.ROSInitException("terminal joint span must be <= 0.002 rad")
        if not 0.5 <= self.terminal_stability_duration <= 1.0:
            raise rospy.ROSInitException("terminal stability duration outside 0.5-1.0 s")
        if not 0.0 < self.terminal_tcp_position <= 0.008:
            raise rospy.ROSInitException("terminal TCP position must be <= 8 mm")
        if not 0.0 < self.terminal_tcp_orientation <= math.radians(3.0):
            raise rospy.ROSInitException("terminal TCP orientation must be <= 3 deg")
        if not 0.0 < self.final_tcp_lateral <= 0.008:
            raise rospy.ROSInitException("final TCP lateral tolerance must be <= 8 mm")
        if not 0.0 < self.final_tcp_vertical <= 0.004:
            raise rospy.ROSInitException("final TCP vertical tolerance must be <= 4 mm")
        if not 0.0 < self.maximum_base_translation_after_observation <= 0.01:
            raise rospy.ROSInitException(
                "post-observation base translation limit must be <= 10 mm"
            )
        if not 0.0 < self.maximum_base_yaw_after_observation <= math.radians(2.0):
            raise rospy.ROSInitException(
                "post-observation base yaw limit must be <= 2 deg"
            )
        if not 0.0 < self.relocalize_small < self.relocalize_maximum <= 0.05:
            raise rospy.ROSInitException("relocalization thresholds must satisfy 0 < small < maximum <= 0.05")
        if not 0 <= self.maximum_pregrasp_replans <= 3:
            raise rospy.ROSInitException(
                "maximum_pregrasp_replans must be between 0 and 3"
            )
        if not 0 <= self.maximum_final_candidate_retries <= 5:
            raise rospy.ROSInitException(
                "maximum_final_candidate_retries must be between 0 and 5"
            )
        if len(self.transport_xyz) != 3 or len(self.transport_rpy) != 3:
            raise rospy.ROSInitException("transport pose xyz/rpy must each have three values")
        if (not self.transport_candidates
                or any(len(candidate) != 3 for candidate in self.transport_candidates)):
            raise rospy.ROSInitException(
                "transport_pose_candidates must contain xyz triples"
            )
        if not 0.005 <= self.place_speed <= 0.03:
            raise rospy.ROSInitException("place_speed outside 0.005-0.03 m/s")

        self.robot = URDF.from_parameter_server("/robot_description")
        if self.tcp_link not in self.robot.link_map:
            raise rospy.ROSInitException(
                "robot_description does not contain geometry-derived {}".format(
                    self.tcp_link
                )
            )
        self.joint_limits = self._load_joint_limits()
        gripper_description = self.robot.joint_map.get(self.gripper_joint)
        if gripper_description is None or gripper_description.limit is None:
            raise rospy.ROSInitException(
                "gripper joint limit missing from robot_description: {}".format(
                    self.gripper_joint
                )
            )
        self.gripper_joint_lower = float(gripper_description.limit.lower)
        self.gripper_joint_upper = float(gripper_description.limit.upper)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.status_pub = rospy.Publisher(
            self.executor_status_topic, String, queue_size=10, latch=True
        )
        self.report_pub = rospy.Publisher(
            "/mine_grasp/report", String, queue_size=1, latch=True
        )
        self.retained_pub = rospy.Publisher(
            "/mine_grasp/retained", Bool, queue_size=1, latch=True
        )
        self.motion_diag_pub = rospy.Publisher(
            "/mine_grasp/motion_diagnostics", DiagnosticArray,
            queue_size=5, latch=True,
        )
        self.debug_markers_pub = rospy.Publisher(
            "/mine_grasp/debug_markers", MarkerArray, queue_size=2, latch=True
        )
        self.tcp_trace_pub = rospy.Publisher(
            "/mine_grasp/tcp_trace", Path, queue_size=2, latch=True
        )
        self.attempt_pub = rospy.Publisher(
            "/mine_grasp/attempt_id", String, queue_size=1, latch=True
        )
        self.expected_source_pub = rospy.Publisher(
            "/mine_grasp/expected_source_pose", PoseStamped, queue_size=1, latch=True
        )
        self.force_attach_pub = rospy.Publisher(
            self.force_attach_topic, String, queue_size=5
        )
        self.force_detach_pub = rospy.Publisher(
            self.force_detach_topic, String, queue_size=5
        )
        self.pose_pubs = {
            "grasp": rospy.Publisher("/mine_grasp/grasp_pose", PoseStamped,
                                     queue_size=1, latch=True),
            "pregrasp": rospy.Publisher("/mine_grasp/pregrasp_pose", PoseStamped,
                                        queue_size=1, latch=True),
            "approach": rospy.Publisher("/mine_grasp/approach_pose", PoseStamped,
                                         queue_size=1, latch=True),
            "lift": rospy.Publisher("/mine_grasp/lift_pose", PoseStamped,
                                    queue_size=1, latch=True),
        }
        self.cmd_publishers = [
            rospy.Publisher(topic, Twist, queue_size=1)
            for topic in rospy.get_param(
                "~cmd_vel_topics", ["/husky_velocity_controller/cmd_vel", "/cmd_vel"]
            )
        ]

        rospy.Subscriber("/mine_grasp/target_pose", PoseStamped,
                         self._target_cb, queue_size=1)
        rospy.Subscriber("/mine_grasp/target_observation",
                         PoseWithCovarianceStamped,
                         self._target_observation_cb, queue_size=1)
        rospy.Subscriber("/mine_grasp/target_valid", Bool,
                         self._valid_cb, queue_size=1)
        rospy.Subscriber("/mine_grasp/confidence", Float32,
                         self._confidence_cb, queue_size=1)
        rospy.Subscriber("/mine_grasp/target_std", Float32MultiArray,
                         self._std_cb, queue_size=1)
        rospy.Subscriber("/mine_grasp/localizer_status", String,
                         self._localizer_status_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~joint_states_topic", "/joint_states"),
                         JointState, self._joint_state_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~arm_state_topic",
                                        "/ur5_arm_controller/state"),
                         JointTrajectoryControllerState, self._arm_state_cb,
                         queue_size=1)
        rospy.Subscriber(rospy.get_param("~gripper_state_topic",
                                        "/gripper_controller/state"),
                         JointTrajectoryControllerState, self._gripper_state_cb,
                         queue_size=1)
        rospy.Subscriber(rospy.get_param("~imu_topic", "/imu/data"), Imu,
                         self._imu_cb, queue_size=1)
        for topic in rospy.get_param(
                "~odom_topics", ["/odometry/filtered", "/husky_velocity_controller/odom"]):
            rospy.Subscriber(topic, Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param(
            "~grasp_event_topic", "/mine_grasp_event_republisher/grasp_events"),
            GazeboGraspEvent, self._grasp_event_cb, queue_size=5)
        self.force_lock_status_sub = rospy.Subscriber(
            self.force_lock_status_topic, String,
            self._force_lock_status_cb, queue_size=10,
        )
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self._model_states_cb, queue_size=1)
        rospy.Subscriber("/clock", Clock, self._clock_cb, queue_size=1)
        self.arm_action_topic = rospy.get_param(
            "~arm_action", "/ur5_arm_controller/follow_joint_trajectory"
        )
        self.gripper_action_topic = rospy.get_param(
            "~gripper_action", "/gripper_controller/follow_joint_trajectory"
        )
        rospy.Subscriber(
            self.arm_action_topic + "/goal",
            FollowJointTrajectoryActionGoal,
            self._arm_fjt_goal_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            self.arm_action_topic + "/result",
            FollowJointTrajectoryActionResult,
            self._arm_fjt_result_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            self.gripper_action_topic + "/goal",
            FollowJointTrajectoryActionGoal,
            self._gripper_fjt_goal_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            self.gripper_action_topic + "/result",
            FollowJointTrajectoryActionResult,
            self._gripper_fjt_result_cb,
            queue_size=10,
        )

        self.arm_client = actionlib.SimpleActionClient(
            self.arm_action_topic,
            FollowJointTrajectoryAction,
        )
        self.gripper_client = actionlib.SimpleActionClient(
            self.gripper_action_topic,
            FollowJointTrajectoryAction,
        )
        self.move_group_client = actionlib.SimpleActionClient(
            rospy.get_param("~move_group_action", "/move_group"), MoveGroupAction
        )
        self.execute_client = actionlib.SimpleActionClient(
            rospy.get_param("~execute_trajectory_action", "/execute_trajectory"),
            ExecuteTrajectoryAction,
        )
        self.ik_service = rospy.ServiceProxy(
            rospy.get_param("~compute_ik_service", "/compute_ik"), GetPositionIK
        )
        self.fk_service = rospy.ServiceProxy(
            rospy.get_param("~compute_fk_service", "/compute_fk"), GetPositionFK
        )
        self.cartesian_service = rospy.ServiceProxy(
            rospy.get_param("~cartesian_path_service", "/compute_cartesian_path"),
            GetCartesianPath,
        )
        self.apply_scene_service = rospy.ServiceProxy(
            rospy.get_param("~apply_scene_service", "/apply_planning_scene"),
            ApplyPlanningScene,
        )
        self.get_scene_service = rospy.ServiceProxy(
            rospy.get_param("~get_scene_service", "/get_planning_scene"),
            GetPlanningScene,
        )
        self.state_validity_service = rospy.ServiceProxy(
            rospy.get_param("~state_validity_service", "/check_state_validity"),
            GetStateValidity,
        )
        self.clear_octomap_service = rospy.ServiceProxy(
            rospy.get_param("~clear_octomap_service", "/clear_octomap"), Empty
        )
        self.reset_target_confirmation = rospy.ServiceProxy(
            rospy.get_param(
                "~reset_confirmation_service",
                "/mine_grasp/reset_target_confirmation",
            ),
            Trigger,
        )
        self.list_controllers = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers
        )
        self.delete_model_service = rospy.ServiceProxy(
            self.gazebo_delete_model_service_name, DeleteModel
        )
        self.spawn_model_service = rospy.ServiceProxy(
            self.gazebo_spawn_model_service_name, SpawnModel
        )

        self.execute_service = rospy.Service(
            "/mine_grasp/execute", Trigger, self._execute_cb
        )
        self.reset_service = rospy.Service(
            "/mine_grasp/reset_executor", Trigger, self._reset_cb
        )
        self.transport_service = rospy.Service(
            "/mine_grasp/prepare_transport", Trigger, self._transport_cb
        )
        self.drop_now_service = rospy.Service(
            "/mine_grasp/drop_now", Trigger, self._drop_now_cb
        )
        self.action_server = actionlib.SimpleActionServer(
            self.action_name, MineGraspAction,
            execute_cb=self._action_cb, auto_start=False,
        )
        self.action_server.start()
        self.retention_timer = rospy.Timer(
            rospy.Duration(max(0.2, self.transport_hold_refresh)),
            self._retention_watchdog,
        )
        self.trace_timer = rospy.Timer(rospy.Duration(0.1), self._trace_tcp_cb)
        self._set_retained(False)
        self._publish_status("READY", "READY", {
            "physical_grasp_enabled": self.physical_enabled,
            "simulated_lock_fallback_enabled": self.simulated_lock_fallback_enabled,
            "virtual_transport_enabled": self.virtual_transport_enabled,
            "tcp_link": self.tcp_link,
        })

    def _load_joint_limits(self):
        limits = {}
        for name in self.arm_joint_names:
            joint = self.robot.joint_map.get(name)
            if joint is None:
                raise rospy.ROSInitException("joint missing from URDF: {}".format(name))
            if joint.type == "continuous" or joint.limit is None:
                limits[name] = None
            else:
                limits[name] = (float(joint.limit.lower), float(joint.limit.upper))
        return limits

    def _set_retained(self, retained):
        relative_reference = self._relative_model_tcp() if retained else None
        with self.lock:
            self.retained = bool(retained)
            if retained:
                self.secure_gripper_position = self._gripper_actual_position()
                self.transport_relative_reference = relative_reference
            else:
                self.secure_gripper_position = None
                self.transport_relative_reference = None
        self.retained_pub.publish(Bool(data=bool(retained)))

    def _retention_watchdog(self, _event):
        """Monitor transport retention and expose any physical unlock.

        gazebo_grasp_fix is the primary rigid transport lock.  This watchdog
        keeps an ordinary physical grasp at its measured contact position.  An
        explicit simulator lock is already rigid and follows the palm in the
        Gazebo world-update thread, so only the plugin's attach/detach state is
        authoritative for that path.  `/gazebo/model_states` and TF are
        separate asynchronous streams; comparing them while the base moves can
        manufacture centimetres of apparent slip even though the kinematic
        lock has not moved.  Neither path fabricates an attached state when the
        Gazebo lock is absent.
        """
        with self.lock:
            retained = self.retained
            attached = self.gazebo_attached
            forced = self.forced_lock_active
            forced_object = str(self.forced_lock_object)
            attached_object = str(self.gazebo_attached_object)
            releasing = self.releasing
            virtual = self.virtual_transport_active
            virtual_present = self.virtual_model_present
            virtual_name = self.virtual_model_name
            secure = self.secure_gripper_position
            reference = None if self.transport_relative_reference is None else np.array(
                self.transport_relative_reference, copy=True
            )
        if not retained or releasing:
            return
        if virtual:
            # The entity is intentionally absent while driving.  Retention is
            # represented by the PICK/PLACE transaction, not by a Gazebo joint
            # or a continuously squeezed finger controller.
            if virtual_present or self._gazebo_model_exists(virtual_name):
                self._set_retained(False)
                self._publish_status(
                    "TRANSPORT_LOCK_LOST", "TRANSPORT_FAILED",
                    {
                        "detail": "virtual carried model unexpectedly exists in Gazebo",
                        "model_name": virtual_name,
                    },
                )
            return

        # A plugin-confirmed forced lock is a kinematic world-update lock, not
        # a friction grasp.  It cannot slip relative to the palm and the plugin
        # deliberately never auto-detaches it.  Do not revoke that transaction
        # from a cross-topic pose sample while the chassis translates/turns;
        # only an authoritative detach or an object-identity mismatch may stop
        # loaded driving.  PLACE sets `releasing` before requesting that detach.
        if forced:
            identity_ok = bool(
                attached
                and forced_object
                and attached_object
                and forced_object == attached_object
            )
            if not identity_ok:
                self._set_retained(False)
                self._publish_status(
                    "TRANSPORT_LOCK_LOST", "OBJECT_DROPPED",
                    {
                        "detail": "plugin-confirmed transport lock became incoherent",
                        "gazebo_attached": attached,
                        "forced_object": forced_object,
                        "attached_object": attached_object,
                    },
                )
            return

        actual = self._gripper_actual_position()
        if not attached or actual is None or secure is None:
            self._set_retained(False)
            self._publish_status(
                "TRANSPORT_LOCK_LOST", "OBJECT_DROPPED",
                {
                    "gazebo_attached": attached,
                    "forced_lock_active": forced,
                    "gripper_position": actual,
                },
            )
            return
        try:
            self._verify_attached_scene()
            current_relative = self._relative_model_tcp()
            if reference is not None and current_relative is not None:
                delta = np.linalg.inv(reference).dot(current_relative)
                translation = float(np.linalg.norm(delta[0:3, 3]))
                rotation = _rotation_angle(delta)
                if (translation > self.relative_translation_tolerance
                        or rotation > self.relative_rotation_tolerance):
                    raise GraspFailure(
                        "OBJECT_DROPPED",
                        "transport slip {:.3f} m / {:.1f} deg".format(
                            translation, math.degrees(rotation)
                        ),
                    )
        except Exception as exc:
            self._set_retained(False)
            self._publish_status(
                "TRANSPORT_LOCK_LOST", "OBJECT_DROPPED", {"detail": str(exc)}
            )
            return
        if abs(actual - secure) <= self.transport_gripper_drift:
            return
        # Reassert the measured contact position at a deliberately low speed.
        # The physical grasp-fix joint must still be attached for this path.
        try:
            duration = max(0.5, abs(actual - secure) / 0.01)
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.05)
            goal.trajectory.joint_names = [self.gripper_joint]
            point = JointTrajectoryPoint()
            point.positions = [float(secure)]
            point.velocities = [0.0]
            point.time_from_start = rospy.Duration(duration)
            goal.trajectory.points = [point]
            self._set_goal_tolerances(goal, [self.gripper_joint])
            self.gripper_client.send_goal(goal)
            rospy.logwarn(
                "[MineGrasp] transport jaw drift %.4f rad; reasserting lock",
                abs(actual - secure),
            )
        except Exception as exc:
            self._set_retained(False)
            rospy.logerr("[MineGrasp] cannot maintain transport jaw lock: %s", exc)

    def _target_cb(self, msg):
        with self.lock:
            self.target_pose = msg

    def _target_observation_cb(self, msg):
        diagonal = [msg.pose.covariance[index] for index in (0, 7, 14)]
        position = msg.pose.pose.position
        values = [position.x, position.y, position.z] + list(diagonal)
        if (not msg.header.frame_id or msg.header.stamp == rospy.Time()
                or not all(math.isfinite(float(value)) for value in values)
                or any(float(value) < 0.0 for value in diagonal)):
            rospy.logwarn_throttle(
                1.0, "[MineGrasp] rejecting malformed target_observation"
            )
            return
        with self.lock:
            self.target_observation = copy.deepcopy(msg)
            self.target_std = [math.sqrt(max(float(value), 0.0))
                               for value in diagonal]

    def _valid_cb(self, msg):
        with self.lock:
            self.target_valid = bool(msg.data)

    def _confidence_cb(self, msg):
        with self.lock:
            self.target_confidence = float(msg.data)

    def _std_cb(self, msg):
        if len(msg.data) >= 3:
            with self.lock:
                self.target_std = [float(value) for value in msg.data[:3]]

    def _localizer_status_cb(self, msg):
        with self.lock:
            self.localizer_status = msg.data

    def _joint_state_cb(self, msg):
        with self.lock:
            self.joint_state = msg

    def _arm_state_cb(self, msg):
        with self.lock:
            self.arm_state = msg
            if msg.error.positions:
                error = max(abs(value) for value in msg.error.positions)
                self.metrics["arm_sag_max_rad"] = max(
                    self.metrics.get("arm_sag_max_rad", 0.0), error
                )

    def _gripper_state_cb(self, msg):
        with self.lock:
            self.gripper_state = msg
            if msg.actual.positions and self.metrics.get("attempt_id"):
                position = float(msg.actual.positions[0])
                velocity = (
                    float(msg.actual.velocities[0])
                    if msg.actual.velocities else None
                )
                count = int(self.metrics.get("gripper_state_samples", 0)) + 1
                self.metrics["gripper_state_samples"] = count
                previous_min = self.metrics.get("gripper_position_min_rad")
                previous_max = self.metrics.get("gripper_position_max_rad")
                self.metrics["gripper_position_min_rad"] = (
                    position if previous_min is None
                    else min(position, previous_min)
                )
                self.metrics["gripper_position_max_rad"] = (
                    position if previous_max is None
                    else max(position, previous_max)
                )
                if velocity is not None:
                    self.metrics["gripper_velocity_peak_rad_s"] = max(
                        abs(velocity),
                        self.metrics.get("gripper_velocity_peak_rad_s", 0.0),
                    )
                    self.metrics["_gripper_velocity_square_sum"] = (
                        self.metrics.get("_gripper_velocity_square_sum", 0.0)
                        + velocity * velocity
                    )
                    self.metrics["_gripper_velocity_samples"] = (
                        int(self.metrics.get("_gripper_velocity_samples", 0)) + 1
                    )

    def _imu_cb(self, msg):
        with self.lock:
            self.imu = msg
        self._sample_attitude()

    def _odom_cb(self, msg):
        with self.lock:
            self.odom = msg

    def _grasp_event_cb(self, msg):
        object_model = str(msg.object).split("::", 1)[0]
        if not object_model.startswith(self.mine_model_prefix):
            return
        with self.lock:
            previous_attached = bool(self.gazebo_attached)
            self.gazebo_attached = bool(msg.attached)
            self.gazebo_attached_object = msg.object
            if msg.attached:
                self.attached_model_name = object_model
            elif self.forced_lock_object == msg.object:
                # The Gazebo event is the authoritative physical-joint state.
                # The separate force-lock acknowledgement identifies whether
                # this was an explicit lock or an ordinary contact grasp.
                self.forced_lock_active = False
                self.forced_lock_object = ""
            self.last_grasp_event_wall = time.monotonic()
            retained = self.retained
            releasing = self.releasing
            if previous_attached != bool(msg.attached):
                key = (
                    "grasp_attach_edges" if msg.attached
                    else "grasp_detach_edges"
                )
                self.metrics[key] = int(self.metrics.get(key, 0)) + 1
        with self.lock:
            virtual = self.virtual_transport_active
        if retained and not msg.attached and not releasing and not virtual:
            self._set_retained(False)
            self._publish_status(
                "TRANSPORT_LOCK_LOST", "OBJECT_DROPPED",
                {"object": msg.object},
            )

    def _force_lock_status_cb(self, msg):
        """Track acknowledgements emitted after Gazebo changed the joint."""
        parts = str(msg.data).split("|", 1)
        if len(parts) != 2:
            return
        state, object_name = parts[0].strip().upper(), parts[1].strip()
        object_model = object_name.split("::", 1)[0]
        if not object_model.startswith(self.mine_model_prefix):
            return
        with self.lock:
            if state == "ATTACHED":
                self.forced_lock_active = True
                self.forced_lock_object = object_name
                self.attached_model_name = object_model
                self.metrics["simulated_lock_acknowledged"] = True
                self.metrics["simulated_lock_object"] = object_name
            elif state == "DETACHED":
                if (not self.forced_lock_object
                        or self.forced_lock_object == object_name):
                    self.forced_lock_active = False
                    self.forced_lock_object = ""
                self.metrics["simulated_lock_release_acknowledged"] = True

    def _model_states_cb(self, msg):
        with self.lock:
            self.model_states = msg

    def _clock_cb(self, msg):
        with self.lock:
            self.clock = copy.deepcopy(msg)

    def _arm_fjt_goal_cb(self, msg):
        goal = msg.goal
        explicit = {
            item.name: float(item.position)
            for item in list(goal.goal_tolerance)
        }
        too_small = {
            name: value for name, value in explicit.items()
            if 0.0 < value < self.minimum_explicit_goal_tolerance
        }
        snapshot = {
            "wall_time": time.time(),
            "ros_stamp": msg.header.stamp.to_sec(),
            "goal_id": msg.goal_id.id,
            "joint_names": list(goal.trajectory.joint_names),
            "point_count": len(goal.trajectory.points),
            "final_positions": (
                [float(value) for value in goal.trajectory.points[-1].positions]
                if goal.trajectory.points else []
            ),
            "final_velocities": (
                [float(value) for value in goal.trajectory.points[-1].velocities]
                if goal.trajectory.points else []
            ),
            "goal_tolerance": explicit,
            "path_tolerance": {
                item.name: float(item.position)
                for item in list(goal.path_tolerance)
            },
            "goal_time_tolerance": goal.goal_time_tolerance.to_sec(),
        }
        with self.lock:
            self.last_arm_fjt_goal = copy.deepcopy(msg)
            self.last_arm_fjt_goal_wall = time.monotonic()
            self.metrics.setdefault("fjt_goals", []).append(snapshot)
            if len(self.metrics["fjt_goals"]) > 32:
                self.metrics["fjt_goals"] = self.metrics["fjt_goals"][-32:]
        if too_small:
            rospy.logerr(
                "[MineGrasp] observed unsafe explicit FJT goal tolerance(s) %s; "
                "executor-generated goals use %.3f rad",
                too_small,
                self.explicit_goal_tolerance,
            )

    def _arm_fjt_result_cb(self, msg):
        with self.lock:
            self.last_arm_fjt_result = copy.deepcopy(msg)
            self.last_arm_fjt_result_wall = time.monotonic()
            self.metrics.setdefault("fjt_results", []).append({
                "wall_time": time.time(),
                "ros_stamp": msg.header.stamp.to_sec(),
                "goal_id": msg.status.goal_id.id,
                "action_status": int(msg.status.status),
                "error_code": int(msg.result.error_code),
                "error_string": str(msg.result.error_string),
            })
            if len(self.metrics["fjt_results"]) > 32:
                self.metrics["fjt_results"] = self.metrics["fjt_results"][-32:]

    def _gripper_fjt_goal_cb(self, msg):
        goal = msg.goal
        snapshot = {
            "wall_time": time.time(),
            "ros_stamp": msg.header.stamp.to_sec(),
            "goal_id": msg.goal_id.id,
            "joint_names": list(goal.trajectory.joint_names),
            "point_count": len(goal.trajectory.points),
            "final_positions": (
                [float(value) for value in goal.trajectory.points[-1].positions]
                if goal.trajectory.points else []
            ),
            "final_velocities": (
                [float(value) for value in goal.trajectory.points[-1].velocities]
                if goal.trajectory.points else []
            ),
            "goal_tolerance": {
                item.name: float(item.position)
                for item in list(goal.goal_tolerance)
            },
            "goal_time_tolerance": goal.goal_time_tolerance.to_sec(),
        }
        with self.lock:
            self.last_gripper_fjt_goal = copy.deepcopy(msg)
            self.last_gripper_fjt_goal_wall = time.monotonic()
            self.metrics.setdefault("gripper_fjt_goals", []).append(snapshot)
            if len(self.metrics["gripper_fjt_goals"]) > 32:
                self.metrics["gripper_fjt_goals"] = (
                    self.metrics["gripper_fjt_goals"][-32:]
                )

    def _gripper_fjt_result_cb(self, msg):
        with self.lock:
            self.last_gripper_fjt_result = copy.deepcopy(msg)
            self.last_gripper_fjt_result_wall = time.monotonic()
            self.metrics.setdefault("gripper_fjt_results", []).append({
                "wall_time": time.time(),
                "ros_stamp": msg.header.stamp.to_sec(),
                "goal_id": msg.status.goal_id.id,
                "action_status": int(msg.status.status),
                "error_code": int(msg.result.error_code),
                "error_string": str(msg.result.error_string),
            })
            if len(self.metrics["gripper_fjt_results"]) > 32:
                self.metrics["gripper_fjt_results"] = (
                    self.metrics["gripper_fjt_results"][-32:]
                )

    def _trace_tcp_cb(self, _event):
        if not self.action_active and not self.execution_lock.locked():
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.gravity_frame, self.tcp_link, rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception:
            return
        pose = PoseStamped()
        pose.header = copy.deepcopy(transform.header)
        pose.header.frame_id = self.gravity_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        with self.lock:
            self.trace_path.header.stamp = pose.header.stamp
            self.trace_path.header.frame_id = self.gravity_frame
            self.trace_path.poses.append(copy.deepcopy(pose))
            if len(self.trace_path.poses) > 3000:
                self.trace_path.poses = self.trace_path.poses[-3000:]
            path = copy.deepcopy(self.trace_path)
            self.debug_poses["actual_tcp"] = copy.deepcopy(pose)
        self.tcp_trace_pub.publish(path)
        self._publish_debug_markers()

    def _publish_status(self, stage, code="RUNNING", extra=None):
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "stage": stage,
            "code": code,
        }
        if extra:
            payload.update(extra)
        text = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.status_pub.publish(String(data=text))
        if self.action_active and self.action_server.is_active():
            feedback = MineGraspFeedback()
            if stage in ("CHECK_INTERFACES", "OPEN_GRIPPER"):
                feedback.stage, feedback.progress = feedback.WAITING, 0.05
            elif stage in ("MOVE_TO_LOOK", "WAIT_TARGET", "REFINE_TARGET"):
                feedback.stage, feedback.progress = feedback.LOCALIZING, 0.20
            elif stage in ("SELECT_GRASP", "MOVE_PREGRASP"):
                feedback.stage, feedback.progress = feedback.PLANNING, 0.40
            elif stage in (
                    "COARSE_APPROACH", "FINAL_APPROACH", "CLOSE_GRIPPER",
                    "SECURE_SIM_LOCK"):
                feedback.stage, feedback.progress = feedback.GRASPING, 0.65
            elif stage in ("MOVEIT_ATTACH", "LIFT", "HOLD"):
                feedback.stage, feedback.progress = feedback.VERIFYING, 0.80
            elif stage.startswith("TRANSPORT"):
                feedback.stage, feedback.progress = feedback.TRANSPORTING, 0.90
            elif stage.startswith("PLACE"):
                feedback.stage, feedback.progress = feedback.PLACING, 0.65
            elif stage.startswith("RELEASE"):
                feedback.stage, feedback.progress = feedback.RELEASING, 0.85
            else:
                feedback.stage, feedback.progress = feedback.WAITING, 0.0
            detail = ""
            if extra:
                detail = str(extra.get("detail") or extra.get("localizer_status") or "")
            feedback.detail = (stage + ": " + code + ("; " + detail if detail else ""))[:512]
            self.action_server.publish_feedback(feedback)
        self._publish_motion_diagnostics(stage, code, extra or {})
        self._publish_debug_markers()
        rospy.loginfo("[MineGrasp] %s", text)

    @staticmethod
    def _diag_value(key, value):
        if isinstance(value, (dict, list, tuple)):
            value = json.dumps(value, ensure_ascii=False, sort_keys=True)
        return KeyValue(key=str(key), value=str(value))

    def _publish_motion_diagnostics(self, stage, code, extra):
        with self.lock:
            arm = copy.deepcopy(self.arm_state)
            gripper = copy.deepcopy(self.gripper_state)
            odom = copy.deepcopy(self.odom)
            clock = copy.deepcopy(self.clock)
            raw_result = copy.deepcopy(self.last_arm_fjt_result)
            measurements = copy.deepcopy(self.last_motion_measurements)
            contacts = copy.deepcopy(self.metrics.get("collision_contacts", []))
        values = {
            "attempt_id": self.attempt_id,
            "stage": stage,
            "code": code,
            "sim_clock": None if clock is None else clock.clock.to_sec(),
            "move_group_action_state": self.move_group_client.get_state(),
            "execute_trajectory_action_state": self.execute_client.get_state(),
            "arm_action_state": self.arm_client.get_state(),
            "gripper_action_state": self.gripper_client.get_state(),
            "moveit_error_code": self.last_moveit_error_code,
            "cartesian_fraction": self.last_cartesian_fraction,
            "collision_pairs": [
                "{}<->{}".format(item.get("body_1", "?"), item.get("body_2", "?"))
                for item in contacts[-12:]
            ],
        }
        if arm is not None:
            values.update({
                "arm_joint_names": list(arm.joint_names),
                "arm_desired_positions": list(arm.desired.positions),
                "arm_actual_positions": list(arm.actual.positions),
                "arm_position_errors": list(arm.error.positions),
                "arm_actual_velocities": list(arm.actual.velocities),
                "arm_max_position_error": maximum_absolute(arm.error.positions),
                "arm_max_velocity": maximum_absolute(arm.actual.velocities),
            })
        if gripper is not None:
            values.update({
                "gripper_desired_position": (
                    gripper.desired.positions[0] if gripper.desired.positions else None
                ),
                "gripper_actual_position": (
                    gripper.actual.positions[0] if gripper.actual.positions else None
                ),
                "gripper_actual_velocity": (
                    gripper.actual.velocities[0] if gripper.actual.velocities else None
                ),
            })
        if odom is not None:
            values.update({
                "base_position": [
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                ],
                "base_linear_speed": math.hypot(
                    odom.twist.twist.linear.x, odom.twist.twist.linear.y
                ),
                "base_angular_speed": abs(odom.twist.twist.angular.z),
            })
        if raw_result is not None:
            values.update({
                "last_fjt_action_status": int(raw_result.status.status),
                "last_fjt_error_code": int(raw_result.result.error_code),
                "last_fjt_error_string": str(raw_result.result.error_string),
            })
        values.update(measurements)
        values.update(extra)
        status = DiagnosticStatus()
        status.name = "mine_grasp/motion"
        status.hardware_id = "husky_ur5"
        if code in FAILURE_CODES or stage.endswith("FAILED"):
            status.level = DiagnosticStatus.ERROR
        elif code in ("CONTROLLER_UNSETTLED", "RUNNING"):
            status.level = DiagnosticStatus.WARN if code != "RUNNING" else DiagnosticStatus.OK
        else:
            status.level = DiagnosticStatus.OK
        status.message = "{}: {}".format(stage, code)
        status.values = [
            self._diag_value(key, value) for key, value in sorted(values.items())
        ]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self.motion_diag_pub.publish(message)

    def _pose_in_gravity_for_marker(self, pose):
        if pose is None or not pose.header.frame_id:
            return None
        command = copy.deepcopy(pose)
        command.header.stamp = rospy.Time(0)
        try:
            return self.tf_buffer.transform(
                command, self.gravity_frame, rospy.Duration(0.05)
            )
        except Exception:
            return None

    def _publish_debug_markers(self):
        if not hasattr(self, "debug_markers_pub"):
            return
        with self.lock:
            poses = copy.deepcopy(self.debug_poses)
        output = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        output.markers.append(clear)
        stamp = rospy.Time.now()
        marker_id = 1
        colors = {
            "target_center": (1.0, 0.85, 0.0),
            "target_top": (1.0, 0.45, 0.0),
            "pregrasp": (0.1, 0.4, 1.0),
            "approach": (0.0, 0.9, 0.9),
            "grasp": (0.9, 0.1, 0.1),
            "lift": (0.1, 1.0, 0.2),
            "commanded_tcp": (1.0, 0.0, 1.0),
            "actual_tcp": (0.1, 1.0, 1.0),
        }
        transformed = {}
        for name in (
            "target_center", "target_top", "pregrasp", "approach",
            "grasp", "lift", "commanded_tcp", "actual_tcp",
        ):
            pose = self._pose_in_gravity_for_marker(poses.get(name))
            if pose is None:
                continue
            transformed[name] = pose
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.gravity_frame
            marker.ns = "mine_grasp_" + name
            marker.id = marker_id
            marker_id += 1
            marker.action = Marker.ADD
            marker.pose = pose.pose
            if name.startswith("target_") or name == "actual_tcp":
                marker.type = Marker.SPHERE
                marker.scale.x = marker.scale.y = marker.scale.z = 0.018
            else:
                marker.type = Marker.ARROW
                marker.scale.x, marker.scale.y, marker.scale.z = 0.08, 0.012, 0.018
            marker.color.a = 0.95
            marker.color.r, marker.color.g, marker.color.b = colors[name]
            output.markers.append(marker)
        actual = transformed.get("actual_tcp")
        commanded = transformed.get("commanded_tcp")
        if actual is not None and commanded is not None:
            arrow = Marker()
            arrow.header.stamp = stamp
            arrow.header.frame_id = self.gravity_frame
            arrow.ns = "mine_grasp_tcp_error"
            arrow.id = marker_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(
                    x=actual.pose.position.x,
                    y=actual.pose.position.y,
                    z=actual.pose.position.z,
                ),
                Point(
                    x=commanded.pose.position.x,
                    y=commanded.pose.position.y,
                    z=commanded.pose.position.z,
                ),
            ]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.006, 0.012, 0.015
            arrow.color.a, arrow.color.r, arrow.color.g = 1.0, 1.0, 0.2
            output.markers.append(arrow)
        self.debug_markers_pub.publish(output)

    def _begin_metrics(self):
        self.metrics = {
            "attempt_id": self.attempt_id,
            "success": False,
            "physical_grasp_enabled": self.physical_enabled,
            "detection_success": False,
            "target_confidence": 0.0,
            "target_std_x": None,
            "target_std_y": None,
            "target_std_z": None,
            "ik_success": False,
            "collision_checked": False,
            "collision_contacts": [],
            "pregrasp_success": False,
            "approach_success": False,
            "gripper_closed": False,
            "nonempty_grasp": False,
            "gazebo_attached": False,
            "moveit_attached": False,
            "lifted": False,
            "dropped": False,
            "max_roll_deg": 0.0,
            "max_pitch_deg": 0.0,
            "arm_sag_max_rad": 0.0,
            "failure_reason": "",
            "failure_detail": "",
            "candidate_yaw_deg": None,
            "candidate_tilt_deg": None,
            "candidate_cartesian_prechecks": [],
            "final_approach_candidates": [],
            "cartesian_error": "",
            "cartesian_fraction": None,
            "source_prior_xy_error": None,
            "late_controller_success": False,
            "recovery_attempted": False,
            "recovery_success": False,
            "recovery_suppressed_for_attitude": False,
            "total_duration": 0.0,
            "motion_events": [],
            "fjt_goals": [],
            "fjt_results": [],
            "gripper_fjt_goals": [],
            "gripper_fjt_results": [],
            "gripper_state_samples": 0,
            "gripper_position_min_rad": None,
            "gripper_position_max_rad": None,
            "gripper_velocity_peak_rad_s": 0.0,
            "grasp_attach_edges": 0,
            "grasp_detach_edges": 0,
            "simulated_lock_enabled": self.simulated_lock_fallback_enabled,
            "simulated_lock_attempted": False,
            "simulated_lock_acknowledged": False,
            "simulated_lock_release_acknowledged": False,
            "simulated_lock_used": False,
            "simulated_lock_pre_close": False,
            "simulated_lock_skipped_contact_squeeze": False,
            "simulated_lock_object": "",
            "simulated_lock_rejection": "",
            "target_updates": [],
            # Gazebo model truth is recorded only to diagnose physical contact
            # after a pose has already been produced by wrist perception.  It
            # is never read by IK, candidate generation, or motion planning.
            "validation_snapshots": {},
        }
        self.validation_model_baseline = None

    def _finish_report(self, start_wall, success, reason="", detail=""):
        self.metrics["success"] = bool(success)
        self.metrics["failure_reason"] = reason
        self.metrics["failure_detail"] = detail
        self.metrics["total_duration"] = round(time.monotonic() - start_wall, 3)
        velocity_samples = int(
            self.metrics.pop("_gripper_velocity_samples", 0)
        )
        velocity_square_sum = float(
            self.metrics.pop("_gripper_velocity_square_sum", 0.0)
        )
        self.metrics["gripper_velocity_rms_rad_s"] = (
            math.sqrt(velocity_square_sum / velocity_samples)
            if velocity_samples else None
        )
        with self.lock:
            clock = copy.deepcopy(self.clock)
            odom = copy.deepcopy(self.odom)
            joint_state = copy.deepcopy(self.joint_state)
            arm_state = copy.deepcopy(self.arm_state)
            gripper_state = copy.deepcopy(self.gripper_state)
            retained = bool(self.retained)
            attached = bool(self.gazebo_attached)
            attached_object = str(self.gazebo_attached_object)
            forced_lock = bool(self.forced_lock_active)
            forced_lock_object = str(self.forced_lock_object)
        self.metrics["final_sim_clock"] = (
            None if clock is None else clock.clock.to_sec()
        )
        self.metrics["retained"] = retained
        self.metrics["gazebo_attached_final"] = attached
        self.metrics["gazebo_attached_object_final"] = attached_object
        self.metrics["simulated_lock_active_final"] = forced_lock
        self.metrics["simulated_lock_object_final"] = forced_lock_object
        if joint_state is not None:
            self.metrics["final_joint_state"] = {
                "stamp": joint_state.header.stamp.to_sec(),
                "names": list(joint_state.name),
                "positions": [float(value) for value in joint_state.position],
                "velocities": [float(value) for value in joint_state.velocity],
            }
        if arm_state is not None:
            self.metrics["final_arm_controller_state"] = {
                "stamp": arm_state.header.stamp.to_sec(),
                "joint_names": list(arm_state.joint_names),
                "desired_positions": [
                    float(value) for value in arm_state.desired.positions
                ],
                "actual_positions": [
                    float(value) for value in arm_state.actual.positions
                ],
                "actual_velocities": [
                    float(value) for value in arm_state.actual.velocities
                ],
                "position_errors": [
                    float(value) for value in arm_state.error.positions
                ],
            }
        if gripper_state is not None:
            self.metrics["final_gripper_controller_state"] = {
                "stamp": gripper_state.header.stamp.to_sec(),
                "joint_names": list(gripper_state.joint_names),
                "desired_positions": [
                    float(value) for value in gripper_state.desired.positions
                ],
                "actual_positions": [
                    float(value) for value in gripper_state.actual.positions
                ],
                "actual_velocities": [
                    float(value) for value in gripper_state.actual.velocities
                ],
                "position_errors": [
                    float(value) for value in gripper_state.error.positions
                ],
            }
        if odom is not None:
            self.metrics["final_base_pose"] = {
                "frame_id": odom.header.frame_id,
                "x": float(odom.pose.pose.position.x),
                "y": float(odom.pose.pose.position.y),
                "z": float(odom.pose.pose.position.z),
                "qx": float(odom.pose.pose.orientation.x),
                "qy": float(odom.pose.pose.orientation.y),
                "qz": float(odom.pose.pose.orientation.z),
                "qw": float(odom.pose.pose.orientation.w),
            }
        self.last_report = copy.deepcopy(self.metrics)
        text = json.dumps(self.last_report, ensure_ascii=False, sort_keys=True)
        self.report_pub.publish(String(data=text))
        try:
            directory = os.path.join(
                self.attempt_output_directory, self.attempt_id
            )
            os.makedirs(directory, exist_ok=True)
            path = os.path.join(directory, "executor_report.json")
            temporary = path + ".writing"
            with open(temporary, "w", encoding="utf-8") as stream:
                json.dump(
                    self.last_report, stream, ensure_ascii=False,
                    indent=2, sort_keys=True,
                )
                stream.write("\n")
            os.replace(temporary, path)
        except Exception as exc:
            rospy.logerr("[MineGrasp] cannot save attempt report: %s", exc)
        return text

    def _action_cb(self, goal):
        result = MineGraspResult()
        self.action_active = True
        try:
            if not self.physical_enabled:
                result.outcome = result.UNSAFE
                result.success = False
                result.message = "physical_grasp_enabled=false; mission action is safety-blocked"
                self.action_server.set_aborted(result, result.message)
                return
            if goal.operation == goal.PICK:
                if not goal.mine_pose.header.frame_id:
                    success = False
                    message = (
                        "TARGET_UNSTABLE: PICK goal has no map-frame mine_pose prior"
                    )
                    response = None
                else:
                    with self.lock:
                        self.active_source_prior = copy.deepcopy(goal.mine_pose)
                        self.active_source_mine_id = int(goal.mine_id)
                    self.expected_source_pub.publish(goal.mine_pose)
                    response = self._execute_cb(None)
                if response is not None and response.success:
                    response = self._transport_cb(None)
                if response is not None:
                    success = bool(response.success)
                    message = response.message
            elif goal.operation == goal.PLACE:
                success, message = self._place_goal(goal)
            else:
                success = False
                message = "unsupported manipulation operation {}".format(goal.operation)

            if success:
                result.outcome = result.SUCCESS
                result.success = True
                result.message = message
                self.action_server.set_succeeded(result, message)
                return
            code = self._failure_code_from_message(message)
            result.success = False
            result.message = message
            if code in ("NO_DETECTION", "NO_DETONATOR", "INVALID_DEPTH",
                        "TARGET_UNSTABLE", "TF_FAILED"):
                result.outcome = result.TARGET_NOT_FOUND
            elif code in ("BASE_UNSTABLE", "TRANSPORT_FAILED"):
                result.outcome = result.UNSAFE
            elif code == "CANCELLED":
                result.outcome = result.CANCELLED
                self.action_server.set_preempted(result, message)
                return
            else:
                result.outcome = result.RETRYABLE_FAILURE
            self.action_server.set_aborted(result, message)
        except Exception as exc:
            result.outcome = result.RETRYABLE_FAILURE
            result.success = False
            result.message = "manipulation action exception: {}".format(exc)
            if self.action_server.is_active():
                self.action_server.set_aborted(result, result.message)
        finally:
            with self.lock:
                self.active_source_prior = None
                self.active_source_mine_id = None
            # Clear the latched diagnostic/association prior.  Otherwise a
            # later standalone run or RViz subscriber would see a stale mine.
            self.expected_source_pub.publish(PoseStamped())
            self.action_active = False

    @staticmethod
    def _failure_code_from_message(message):
        try:
            parsed = json.loads(message)
            return str(parsed.get("failure_reason", ""))
        except (TypeError, ValueError):
            for code in FAILURE_CODES:
                if code in str(message):
                    return code
        return ""

    def _transport_cb(self, _request):
        if not self.execution_lock.acquire(False):
            return TriggerResponse(success=False, message="executor is already busy")
        try:
            if not self.physical_enabled:
                return TriggerResponse(success=False, message="TRANSPORT_FAILED: physical mode disabled")
            self._wait_interfaces(require_moveit=True)
            self._check_controllers()
            self._wait_base_stationary()
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                retained = self.retained
                virtualized = bool(self.virtual_transport_active)
                virtual_name = str(self.virtual_model_name)

            if virtualized:
                # The normal integrated PICK path seals the source model before
                # any lift.  A virtual transaction deliberately has no Gazebo
                # joint or MoveIt attachment; requiring those again here would
                # reject the safe state and strand the UGV beside the mine.
                if not retained or not virtual_name:
                    raise GraspFailure(
                        "OBJECT_DROPPED",
                        "pre-lift virtual transport transaction is incomplete",
                    )
                if attached or forced:
                    raise GraspFailure(
                        "TRANSPORT_FAILED",
                        "virtual transport still has a stale physical lock",
                    )
                if self._gazebo_model_exists(virtual_name):
                    raise GraspFailure(
                        "TRANSPORT_FAILED",
                        "virtual carried model unexpectedly exists before arm stow",
                    )
                self._publish_status(
                    "TRANSPORT_VIRTUAL_LOCK_REUSED",
                    "TRANSPORT_SEALED",
                    {"model_name": virtual_name, "ready_to_drive": False},
                )
            else:
                if not attached or not retained:
                    raise GraspFailure(
                        "OBJECT_DROPPED",
                        "physical lock was lost before transport sealing",
                    )
                if self.virtual_transport_enabled and not forced:
                    raise GraspFailure(
                        "GAZEBO_ATTACH_FAILED",
                        "virtual transport requires the acknowledged fixed lock",
                    )
                self._verify_attached_scene()

                # Backward-compatible service calls may still arrive with a
                # physical lock.  Seal that lock before folding; the integrated
                # action normally completed this step before its clearance.
                if self.virtual_transport_enabled:
                    self._publish_status("TRANSPORT_PRESEAL_VERIFY")
                    self._verify_transport_lock(self.transport_hold_duration)
                    virtualized = self._virtualize_carried_model()

            quaternion = tft.quaternion_from_euler(*self.transport_rpy)
            current = self._current_arm_positions()
            choices = []
            failure_kinds = []
            self._publish_status(
                "TRANSPORT_PLAN", extra={
                    "candidate_count": len(self.transport_candidates),
                }
            )
            for index, xyz in enumerate(self.transport_candidates):
                pose = PoseStamped()
                pose.header.frame_id = self.arm_base_frame
                pose.header.stamp = rospy.Time(0)
                (pose.pose.position.x,
                 pose.pose.position.y,
                 pose.pose.position.z) = xyz
                pose.pose.orientation.x = quaternion[0]
                pose.pose.orientation.y = quaternion[1]
                pose.pose.orientation.z = quaternion[2]
                pose.pose.orientation.w = quaternion[3]
                solution, failure_kind = self._solve_ik(
                    pose, "transport_{}".format(index)
                )
                if solution is None:
                    failure_kinds.append(failure_kind or "ik")
                    continue
                joints = self._extract_arm_solution(solution)
                if not self._safe_joint_configuration(joints):
                    failure_kinds.append("joint_safety")
                    continue
                # Prefer the configured ordering, then the smallest joint
                # displacement.  Virtual transport has already removed the
                # physical disc, so this is an unloaded collision-checked fold.
                joint_distance = sum(
                    abs(_wrap_pi(a - b)) for a, b in zip(joints, current)
                )
                choices.append((index, joint_distance, pose, joints))
            if not choices:
                raise GraspFailure(
                    "TRANSPORT_FAILED",
                    "all transport candidates failed ({})".format(
                        ",".join(sorted(set(failure_kinds))) or "ik"
                    ),
                )
            _, _, pose, joints = min(choices, key=lambda item: (item[0], item[1]))
            selected_xyz = [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ]
            self._publish_status(
                "TRANSPORT_MOVE", extra={"selected_xyz": selected_xyz}
            )
            self._motion_or_raise(
                self._move_group_joints(
                    joints, target_pose=pose, stage="TRANSPORT_MOVE"
                ),
                "TRANSPORT_FAILED",
            )
            if virtualized:
                with self.lock:
                    virtual_active = bool(self.virtual_transport_active)
                    retained = bool(self.retained)
                    virtual_name = str(self.virtual_model_name)
                if (not virtual_active or not retained
                        or self._gazebo_model_exists(virtual_name)):
                    raise GraspFailure(
                        "TRANSPORT_FAILED",
                        "virtual transport lock changed during unloaded arm fold",
                    )
                self._publish_status(
                    "VIRTUAL_TRANSPORT_STOWED",
                    "READY_TO_DRIVE",
                    {"model_name": virtual_name, "gripper_command": "HOLD_CLOSED"},
                )
            else:
                self._verify_transport_lock(self.transport_hold_duration)
                # Non-virtual fallback retains its physical/MoveIt attachment.
                self._remove_world_object("mine_ground_guard")
                if not self._wait_world_object_absent("mine_ground_guard"):
                    raise GraspFailure(
                        "TRANSPORT_FAILED",
                        "source ground guard remained in the mobile planning scene",
                    )
                self._set_retained(True)
                self._publish_status("TRANSPORT_LOCKED", "READY_TO_DRIVE")
            return TriggerResponse(
                success=True,
                message=(
                    "PICK verified; source Gazebo model removed and virtual "
                    "transport transaction locked"
                    if virtualized
                    else "PICK verified; grasp-fix + MoveIt + jaw hold locked for transport"
                ),
            )
        except GraspFailure as exc:
            self._cancel_all_motion()
            detail = exc.detail
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                virtual = self.virtual_transport_active
            if attached or forced or virtual:
                # Planning can fail after the mine is already physically held.
                # Propagate retention so the mission manager cannot unlock the
                # chassis or retry alignment with an extended loaded arm.
                self._set_retained(True)
                detail += "; physical lock retained, base must remain locked"
            self._publish_status("TRANSPORT_FAILED", exc.code, {"detail": detail})
            return TriggerResponse(
                success=False, message="{}: {}".format(exc.code, detail)
            )
        except Exception as exc:
            self._cancel_all_motion()
            detail = str(exc)
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                virtual = self.virtual_transport_active
            if attached or forced or virtual:
                self._set_retained(True)
                detail += "; physical lock retained, base must remain locked"
            return TriggerResponse(
                success=False, message="TRANSPORT_FAILED: {}".format(detail)
            )
        finally:
            self.execution_lock.release()

    def _drop_now_cb(self, _request):
        """Release a retained physical mine without a Cartesian place move.

        This is an idempotent last-resort depot operation.  The mission only
        calls it after obstacle-aware return has finished, the base lock is
        asserted, and the ordinary PLACE sequence failed.  It deliberately
        leaves the arm in its drive-safe transport pose; the next PICK begins
        by moving from that pose to LOOK.
        """
        if not self.execution_lock.acquire(False):
            return TriggerResponse(
                success=False, message="DROP_NOW_FAILED: executor is already busy"
            )
        self.releasing = True
        self.release_motion_guard = False
        try:
            self._wait_interfaces(require_moveit=True)
            self._check_controllers()
            self._cancel_all_motion()
            self._wait_base_stationary()
            with self.lock:
                retained = bool(self.retained)
                attached = bool(self.gazebo_attached)
                forced = bool(self.forced_lock_active)
                virtual = bool(self.virtual_transport_active)

            if virtual:
                raise GraspFailure(
                    "DROP_NOW_FAILED",
                    "direct drop is unavailable for virtual transport",
                )

            # PLACE can fail during post-release validation or retreat.  In
            # that case the physical transaction is already complete, so make
            # this service idempotent and only clean stale planning objects.
            already_released = not retained and not attached and not forced
            if not already_released:
                if not retained or (not attached and not forced):
                    raise GraspFailure(
                        "OBJECT_DROPPED",
                        "retention state and simulator lock disagree",
                    )
                self._publish_status(
                    "DROP_NOW_OPEN", "RUNNING", {
                        "detail": "normal PLACE failed; releasing at current depot pose"
                    },
                )
                self._open_gripper_for_release()
                self._release_forced_lock(enforce_safety=False)
                self._wait_gazebo_released(
                    enforce_safety=False, settle_duration=0.15
                )

            # From here a late contact event must not recreate software
            # retention while the MoveIt representation is being removed.
            self.release_motion_guard = True
            if not self._detach_moveit_object():
                rospy.logwarn(
                    "[MineGrasp] DROP_NOW could not confirm MoveIt detach; "
                    "removing both transient world objects anyway"
                )
            self._remove_world_object(self.moveit_object_id)
            self._remove_world_object("mine_ground_guard")
            try:
                self._verify_transient_scene_cleared()
            except GraspFailure as exc:
                # A stale planning marker must not resurrect a released hazard
                # or strand all remaining mission tasks.  The next PICK also
                # performs its normal scene reset before planning.
                rospy.logwarn("[MineGrasp] DROP_NOW scene cleanup: %s", exc.detail)
            self._clear_octomap_best_effort()
            self._set_retained(False)
            with self.lock:
                self.attached_model_name = ""
            self._publish_status(
                "DROP_NOW_COMPLETE", "PLACE_SUCCESS", {
                    "already_released": already_released,
                    "arm_pose": "transport",
                },
            )
            return TriggerResponse(
                success=True,
                message=(
                    "physical lock was already released; stale scene cleared"
                    if already_released
                    else "gripper opened and physical lock released at current depot pose"
                ),
            )
        except GraspFailure as exc:
            with self.lock:
                locked = bool(
                    self.gazebo_attached
                    or self.forced_lock_active
                    or self.virtual_transport_active
                )
            self._set_retained(locked)
            self._publish_status(
                "DROP_NOW_FAILED", exc.code, {"detail": exc.detail}
            )
            return TriggerResponse(
                success=False,
                message="{}: {}".format(exc.code, exc.detail),
            )
        except Exception as exc:
            with self.lock:
                locked = bool(
                    self.gazebo_attached
                    or self.forced_lock_active
                    or self.virtual_transport_active
                )
            self._set_retained(locked)
            return TriggerResponse(
                success=False, message="DROP_NOW_FAILED: {}".format(exc)
            )
        finally:
            self.release_motion_guard = False
            self.releasing = False
            self.execution_lock.release()

    def _place_goal(self, goal):
        if not self.execution_lock.acquire(False):
            return False, "PLACE_FAILED: executor is already busy"
        self.releasing = False
        self.release_motion_guard = False
        try:
            self._wait_interfaces(require_moveit=True)
            self._check_controllers()
            self._wait_base_stationary()
            with self.lock:
                retained = self.retained
                attached = self.gazebo_attached
                virtual = self.virtual_transport_active
            if not retained or (not attached and not virtual):
                raise GraspFailure(
                    "OBJECT_DROPPED",
                    "PLACE requested without a physical or virtual transport lock",
                )
            if not goal.drop_pose.header.frame_id:
                raise GraspFailure("PLACE_FAILED", "drop_pose frame is empty")
            if virtual:
                with self.lock:
                    virtual_name = str(self.virtual_model_name)
                    virtual_present = bool(self.virtual_model_present)
                if virtual_present or self._gazebo_model_exists(virtual_name):
                    raise GraspFailure(
                        "PLACE_FAILED",
                        "virtual carried model unexpectedly exists before PLACE",
                    )
            else:
                self._verify_attached_scene()
            self._publish_status("PLACE_PLAN")
            candidate = self._select_place_candidate(goal.drop_pose)
            self._motion_or_raise(
                self._move_group_joints(
                    candidate["pregrasp_joints"],
                    target_pose=candidate["pregrasp"],
                    stage="PLACE_PREGRASP",
                ),
                "PLACE_FAILED",
            )
            if not virtual:
                self._verify_transport_lock(0.25)
            self._publish_status("PLACE_LOWER")
            self._motion_or_raise(
                self._cartesian_to(
                    candidate["grasp"], self.place_speed, stage="PLACE_LOWER"
                ),
                "PLACE_FAILED",
            )
            if not virtual:
                self._verify_transport_lock(0.25)

            # The only unlock point in the mission: open the physical jaws,
            # wait for gazebo_grasp_fix to report detached, then update MoveIt.
            self.releasing = True
            self._publish_status("RELEASE_OPEN")
            self._motion_or_raise(
                self._command_gripper(
                    self.gripper_open, self.gripper_duration,
                    enforce_safety=True,
                    stage="RELEASE_OPEN",
                ),
                "GRIPPER_FAILED",
            )
            if not virtual:
                self._release_forced_lock(enforce_safety=True)
                self._wait_gazebo_released(enforce_safety=True)
                # From this point onward the physical lock must remain open.
                # The guard prevents a late grasp-fix reattach after MoveIt has
                # detached its collision object.
                self.release_motion_guard = True
                self._publish_status("RELEASE_MOVEIT_DETACH")
                if not self._detach_moveit_object():
                    raise GraspFailure("PLACE_FAILED", "MoveIt detach request failed")
                self._remove_world_object(self.moveit_object_id)
                self._remove_world_object("mine_ground_guard")
                self._verify_transient_scene_cleared()
                self._verify_place_validation(goal.drop_pose)

            retreat = self._world_vertical_offset(
                candidate["grasp"], self.place_retreat_distance
            )
            self._publish_status("PLACE_RETREAT")
            self._motion_or_raise(
                self._cartesian_to(
                    retreat, self.place_speed, stage="PLACE_RETREAT"
                ),
                "PLACE_FAILED",
            )
            self._motion_or_raise(
                self._command_arm(
                    self.look_joints, self.reset_duration,
                    enforce_safety=True,
                    stage="PLACE_LOOK",
                ),
                "PLACE_FAILED",
            )
            if virtual:
                # Keep the world empty until the arm is safely back at look.
                # The exact source model is then recreated at the assigned
                # depot slot and scored using the same post-place validation.
                self._restore_virtual_model_at_drop(goal.drop_pose)
            else:
                # Re-check the physical lock after all arm motion.  The first
                # validation alone cannot rule out a detach/reattach cycle.
                self._wait_gazebo_released(enforce_safety=True)
            self._verify_place_validation(goal.drop_pose)
            if virtual:
                self._clear_virtual_transport()
            self._set_retained(False)
            self.release_motion_guard = False
            self.releasing = False
            self._publish_status("PLACE_COMPLETE", "PLACE_SUCCESS")
            return True, (
                "PLACE verified; source model recreated at disposal slot and arm retreated"
                if virtual
                else "PLACE verified; physical lock released and arm retreated"
            )
        except GraspFailure as exc:
            self._cancel_all_motion()
            self._rollback_virtual_place_spawn()
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                virtual = self.virtual_transport_active
            self._set_retained(attached or forced or virtual)
            self._publish_status("PLACE_FAILED", exc.code, {"detail": exc.detail})
            return False, "{}: {}".format(exc.code, exc.detail)
        except Exception as exc:
            self._cancel_all_motion()
            self._rollback_virtual_place_spawn()
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                virtual = self.virtual_transport_active
            self._set_retained(attached or forced or virtual)
            return False, "PLACE_FAILED: {}".format(exc)
        finally:
            self.release_motion_guard = False
            self.releasing = False
            self.execution_lock.release()

    def _verify_place_validation(self, drop_pose):
        """Use Gazebo truth only to score the already executed PLACE."""
        expected = self._target_in_frame(
            drop_pose, self.gazebo_validation_frame, "TF_FAILED"
        )
        first = self._model_pose(validation_only=True)
        if first is None:
            raise GraspFailure("PLACE_FAILED", "Gazebo place validation pose unavailable")
        settle_start = time.monotonic()
        while time.monotonic() - settle_start < 0.5 and not rospy.is_shutdown():
            self._safety_check()
            time.sleep(0.05)
        second = self._model_pose(validation_only=True)
        if second is None:
            raise GraspFailure("PLACE_FAILED", "placed model disappeared")
        xy_error = math.hypot(
            second.position.x - expected.pose.position.x,
            second.position.y - expected.pose.position.y,
        )
        expected_origin_z = expected.pose.position.z - self.detonator_center_z
        z_error = abs(second.position.z - expected_origin_z)
        settle_motion = math.sqrt(
            (second.position.x - first.position.x) ** 2
            + (second.position.y - first.position.y) ** 2
            + (second.position.z - first.position.z) ** 2
        )
        if xy_error > self.place_validation_xy:
            raise GraspFailure(
                "PLACE_FAILED", "placed mine xy error {:.3f} m".format(xy_error)
            )
        if z_error > self.place_validation_z:
            raise GraspFailure(
                "PLACE_FAILED", "placed mine z error {:.3f} m".format(z_error)
            )
        if settle_motion > self.place_settle_motion:
            raise GraspFailure(
                "PLACE_FAILED", "placed mine still moving {:.3f} m".format(settle_motion)
            )

    def _execute_cb(self, _request):
        if not self.execution_lock.acquire(False):
            return TriggerResponse(success=False, message="executor is already busy")
        start_wall = time.monotonic()
        started_ros = rospy.Time.now()
        with self.lock:
            mine_id = int(self.active_source_mine_id or 0)
            self.active_observation_not_before = started_ros
            self.target_observation = None
            self.attempt_id = "M{:03d}_{}_{:03d}".format(
                mine_id,
                time.strftime("%Y%m%d_%H%M%S", time.localtime()),
                int((time.time() % 1.0) * 1000.0),
            )
            self.trace_path = Path()
            self.trace_path.header.frame_id = self.gravity_frame
            self.trace_path.header.stamp = started_ros
            self.debug_poses = {}
            self.last_motion_measurements = {}
            self.observation_base_reference = None
            self.simulated_lock_visual_gate_pose = None
            if not self.gazebo_attached and not self.forced_lock_active:
                self.forced_lock_active = False
                self.forced_lock_object = ""
            self.pending_forced_lock_object = ""
        self.attempt_pub.publish(String(data=self.attempt_id))
        self.tcp_trace_pub.publish(copy.deepcopy(self.trace_path))
        self._begin_metrics()
        try:
            self._run_pipeline()
            code = "GRASP_SUCCESS" if self.physical_enabled else "DRY_RUN_SUCCESS"
            self._publish_status("COMPLETE", code)
            report = self._finish_report(start_wall, True)
            return TriggerResponse(success=True, message=report)
        except GraspFailure as exc:
            self._cancel_all_motion()
            detail = exc.detail
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                virtual = self.virtual_transport_active
            if attached or forced or virtual:
                # A physical grasp-fix joint is more authoritative than the
                # software stage that happened to fail.  Advertise retention
                # so the mission manager keeps the base locked for recovery.
                self._set_retained(True)
                detail += "; physical grasp lock detected, base must remain locked"
            elif exc.code == "TARGET_OUT_OF_WORKSPACE":
                # This gate is evaluated immediately after MOVE_TO_LOOK was
                # measured stable and before OPEN_GRIPPER, IK or any descent.
                # No recovery trajectory is needed: advertise the already
                # verified empty LOOK state so the mission can reverse to the
                # outer ring, try another direction, and keep later mines live.
                self.metrics["recovery_attempted"] = True
                self.metrics["recovery_success"] = True
                self.metrics["recovery_mode"] = "already_verified_at_look"
                detail += "; empty arm remains verified at look"
            elif exc.code == "BASE_UNSTABLE":
                # The 2026-07-14 M001 report crossed 5.06 deg during close,
                # then the old recovery trajectory drove pitch to 29.4 deg.
                # Once the attitude gate trips, cancelling motion is the only
                # automatic action permitted; never add another arm impulse.
                self.metrics["recovery_attempted"] = False
                self.metrics["recovery_suppressed_for_attitude"] = True
                detail += "; emergency hold applied; automatic arm recovery suppressed"
            elif exc.code != "CANCELLED":
                self.metrics["recovery_attempted"] = True
                recovered = self._best_effort_slow_retreat()
                self.metrics["recovery_success"] = recovered
                detail += (
                    "; arm recovered to look pose"
                    if recovered else "; automatic arm recovery failed"
                )
            self._publish_status("FAILED", exc.code, {"detail": detail})
            report = self._finish_report(start_wall, False, exc.code, detail)
            return TriggerResponse(success=False, message=report)
        except Exception as exc:  # Convert unexpected faults to a diagnostic response.
            self._cancel_all_motion()
            detail = "unexpected executor error: {}".format(exc)
            rospy.logerr("[MineGrasp] %s", detail)
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                virtual = self.virtual_transport_active
            if attached or forced or virtual:
                self._set_retained(True)
                detail += "; physical grasp lock detected, base must remain locked"
            else:
                self.metrics["recovery_attempted"] = True
                recovered = self._best_effort_slow_retreat()
                self.metrics["recovery_success"] = recovered
                detail += (
                    "; arm recovered to look pose"
                    if recovered else "; automatic arm recovery failed"
                )
            self._publish_status("FAILED", "INTERNAL_ERROR", {"detail": detail})
            report = self._finish_report(
                start_wall, False, "INTERNAL_ERROR", detail
            )
            return TriggerResponse(success=False, message=report)
        finally:
            self.execution_lock.release()

    def _reset_cb(self, _request):
        if not self.execution_lock.acquire(False):
            return TriggerResponse(success=False, message="executor is already busy")
        try:
            self._wait_interfaces(require_moveit=True)
            self._cancel_all_motion()
            self.releasing = True
            self._open_gripper_for_release()
            with self.lock:
                virtual = bool(self.virtual_transport_active)
                virtual_name = str(self.virtual_model_name)
                virtual_source_pose = copy.deepcopy(self.virtual_source_pose)
            if virtual:
                if not self._gazebo_model_exists(virtual_name):
                    if virtual_source_pose is None:
                        raise GraspFailure(
                            "PLACE_FAILED",
                            "virtual reset has no saved source model pose",
                        )
                    self._spawn_gazebo_model_origin(
                        virtual_name,
                        virtual_source_pose,
                        enforce_safety=False,
                    )
                self._clear_virtual_transport()
            else:
                self._release_forced_lock(enforce_safety=False)
                self._wait_gazebo_released(enforce_safety=False)
            self._detach_moveit_object()
            # Removing an AttachedCollisionObject may restore it to the world
            # in some MoveIt versions.  Explicitly remove both representations
            # so a previous trial cannot invalidate every IK request.
            self._remove_world_object(self.moveit_object_id)
            self._remove_world_object("mine_ground_guard")
            self._clear_octomap_best_effort()
            self._verify_transient_scene_cleared()
            self._set_retained(False)
            with self.lock:
                self.attached_model_name = ""
            self.releasing = False
            self._motion_or_raise(
                self._command_arm(
                    self.look_joints, self.reset_duration,
                    enforce_safety=True,
                    stage="RESET_LOOK",
                ),
                "PREGRASP_FAILED",
            )
            self._publish_status("RESET", "READY")
            return TriggerResponse(success=True, message="executor reset to look pose")
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))
        finally:
            self.releasing = False
            self.execution_lock.release()

    def _run_pipeline(self):
        self._publish_status("CHECK_INTERFACES")
        self._wait_interfaces(require_moveit=True)
        self._check_controllers()
        self._wait_base_stationary()
        with self.lock:
            attached = self.gazebo_attached
            retained = self.retained
            forced = self.forced_lock_active
            virtual = self.virtual_transport_active
            if not attached:
                self.attached_model_name = ""
        if attached or retained or forced or virtual:
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "PICK rejected because an existing physical/forced lock must be placed or reset first",
            )

        # Reassert the positive safe-open hold before inspecting idle health.
        # A controller initialized at the exact zero-radian hard stop can
        # rebound across the lower limit during the long drive from HOME.  It
        # is recoverable by the measured 0.08-rad open command, so rejecting it
        # before issuing that command would permanently strand an otherwise
        # healthy full mission at CHECK_INTERFACES.
        self._publish_status("STABILIZE_GRIPPER")
        self._motion_or_raise(
            self._command_gripper(
                self.gripper_open, self.gripper_duration,
                enforce_safety=True,
                stage="STABILIZE_GRIPPER",
            ),
            "GRIPPER_FAILED",
        )

        # Prove on fresh samples that the repaired hold is inside the URDF
        # range and stationary before allowing the heavier arm motion.
        self._motion_or_raise(
            self._wait_idle_gripper_safe(),
            "GRIPPER_FAILED",
        )

        self._publish_status("MOVE_TO_LOOK")
        self._motion_or_raise(
            self._command_arm(
                self.look_joints, self.look_duration, enforce_safety=True,
                stage="MOVE_TO_LOOK",
            ),
            "PREGRASP_FAILED",
        )
        self._verify_arm_holding(1.0)

        self._publish_status("WAIT_TARGET")
        target = self._wait_for_target(timeout=self.target_wait_timeout)
        target_base = target
        if target.header.frame_id != self.arm_base_frame:
            try:
                target_base = self.tf_buffer.transform(
                    target, self.arm_base_frame, rospy.Duration(0.5)
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
                raise GraspFailure("TF_FAILED", str(exc))
        planar_reach = math.hypot(
            target_base.pose.position.x, target_base.pose.position.y
        )
        self.metrics["target_planar_reach"] = planar_reach
        if planar_reach > self.maximum_planar_reach:
            raise GraspFailure(
                "TARGET_OUT_OF_WORKSPACE",
                "target planar reach {:.3f} m exceeds stable workspace {:.3f} m; "
                "reposition the stopped UGV closer".format(
                    planar_reach, self.maximum_planar_reach
                ),
            )
        self.metrics["detection_success"] = True
        with self.lock:
            self.metrics["target_confidence"] = self.target_confidence
            self.metrics["target_std_x"] = self.target_std[0]
            self.metrics["target_std_y"] = self.target_std[1]
            self.metrics["target_std_z"] = self.target_std[2]
        self._record_perception_target("initial", target_base)
        self._record_validation_snapshot("target_acquired")
        self._capture_observation_base_reference(target)
        self.metrics["observation_strategy"] = (
            "near_field_relocalization"
            if self.near_field_relocalization_enabled
            else "frozen_downward_look"
        )

        self._publish_status("OPEN_GRIPPER")
        self._motion_or_raise(
            self._command_gripper(
                self.gripper_open, self.gripper_duration,
                enforce_safety=True,
                stage="OPEN_GRIPPER",
            ),
            "GRIPPER_FAILED",
        )
        self._wait_gazebo_released(enforce_safety=True)
        self._record_validation_snapshot("gripper_open")
        self._add_ground_guard(target)

        self._publish_status("SELECT_GRASP")
        candidate = self._select_candidate(target)
        self.metrics["ik_success"] = True
        self.metrics["collision_checked"] = True
        self.metrics["candidate_yaw_deg"] = math.degrees(candidate["yaw"])
        self.metrics["candidate_tilt_deg"] = math.degrees(candidate["tilt"])
        self._publish_candidate_poses(candidate)

        self._publish_status("MOVE_PREGRASP")
        self._motion_or_raise(
            self._move_group_joints(
                candidate["pregrasp_joints"],
                target_pose=candidate["pregrasp"],
                stage="MOVE_PREGRASP",
            ),
            "PREGRASP_FAILED",
        )
        self.metrics["pregrasp_success"] = True
        self._record_validation_snapshot("pregrasp")

        if self.near_field_relocalization_enabled:
            refined, refined_candidate = self._refine_from_pregrasp(
                target, candidate
            )
        else:
            # The current camera looks sideways in a top-down grasp pose.  The
            # fresh, low-covariance atomic LOOK observation is therefore the
            # only geometric target; map/Gazebo data never replace it.
            refined = target
            refined_candidate = candidate
            self._publish_status(
                "TARGET_FROZEN", "FROZEN_DOWNWARD_LOOK",
                {"near_field_relocalization_enabled": False},
            )
            self._verify_observation_base_drift("PREGRASP")

        if not self.physical_enabled:
            self._run_dry_trajectory(
                refined, refined_candidate["yaw"], refined_candidate["tilt"]
            )
            return

        self._verify_observation_base_drift("COARSE_APPROACH_START")
        self._publish_status("COARSE_APPROACH")
        self._motion_or_raise(
            self._cartesian_to(
                refined_candidate["approach"], self.approach_speed,
                stage="COARSE_APPROACH",
            ),
            "APPROACH_FAILED",
        )
        self._record_validation_snapshot("coarse_approach")
        if self.near_field_relocalization_enabled:
            refined, refined_candidate = self._refine_from_coarse(
                refined, refined_candidate
            )
        else:
            self._verify_observation_base_drift("COARSE_APPROACH")

        self._verify_observation_base_drift("FINAL_APPROACH_START")
        self._publish_status("FINAL_APPROACH")
        final_result, refined_candidate = self._final_approach_with_fallback(
            refined, refined_candidate
        )
        self._motion_or_raise(
            final_result,
            "APPROACH_FAILED",
        )
        self._verify_observation_base_drift("FINAL_APPROACH_END")
        self._verify_final_tcp_components(refined_candidate["grasp"])
        with self.lock:
            self.simulated_lock_visual_gate_pose = copy.deepcopy(
                refined_candidate["grasp"]
            )
        self.metrics["approach_success"] = True
        self._record_validation_snapshot("final_approach")

        self._publish_status("CLOSE_GRIPPER")
        if not self._close_gripper_staged():
            raise GraspFailure("GRIPPER_FAILED", "gripper trajectory did not complete")
        self.metrics["gripper_closed"] = True
        self._record_validation_snapshot("gripper_closed")
        self._verify_nonempty_physical_grasp()
        initial_model_pose = self._model_pose(validation_only=True)
        self.metrics["nonempty_grasp"] = True
        self.metrics["gazebo_attached"] = True

        self._publish_status("MOVEIT_ATTACH")
        if not self._attach_moveit_object():
            raise GraspFailure("MOVEIT_ATTACH_FAILED", "attached object not visible in scene")
        self.metrics["moveit_attached"] = True

        if self.virtual_transport_enabled:
            # The fixed joint has already proved the visually selected object
            # identity and the closed-jaw grasp.  Do not lift that rigidly
            # attached model while its disc is still supported by terrain: it
            # creates a closed kinematic chain through the ground and can tip
            # the complete UGV.  Seal/delete it at the contact pose, then move
            # the real arm without payload or ground constraint.  Gazebo truth
            # remains validation/identity-only and never changes the target.
            self._set_retained(True)
            self._publish_status(
                "CAPTURE_PRESEAL_VERIFY",
                "RUNNING",
                {"hold_duration_s": self.capture_preseal_hold},
            )
            self._verify_transport_lock(self.capture_preseal_hold)
            self._record_validation_snapshot("capture_preseal")
            if not self._virtualize_carried_model():
                raise GraspFailure(
                    "TRANSPORT_FAILED",
                    "verified capture could not be sealed before lift",
                )

            clearance_pose = self._world_vertical_offset(
                refined_candidate["grasp"],
                self.virtual_post_capture_clearance,
            )
            self.pose_pubs["lift"].publish(clearance_pose)
            with self.lock:
                self.debug_poses["lift"] = copy.deepcopy(clearance_pose)
            self._publish_status(
                "POST_CAPTURE_CLEARANCE",
                "RUNNING",
                {
                    "distance_m": self.virtual_post_capture_clearance,
                    "payload_mode": "VIRTUAL_UNLOADED",
                },
            )
            self._motion_or_raise(
                self._cartesian_to(
                    clearance_pose,
                    min(self.lift_speed, self.approach_speed, 0.02),
                    stage="POST_CAPTURE_CLEARANCE",
                ),
                "LIFT_FAILED",
            )
            with self.lock:
                virtual_active = bool(self.virtual_transport_active)
                retained = bool(self.retained)
                virtual_name = str(self.virtual_model_name)
            if (not virtual_active or not retained or not virtual_name
                    or self._gazebo_model_exists(virtual_name)):
                raise GraspFailure(
                    "TRANSPORT_FAILED",
                    "virtual capture changed during unloaded clearance",
                )
            self.metrics["lifted"] = True
            self.metrics["lift_mode"] = "VIRTUAL_PRE_LIFT_SEAL"
            self.metrics["physical_payload_lift_skipped"] = True
            self.metrics["post_capture_clearance_m"] = (
                self.virtual_post_capture_clearance
            )
            self._publish_status(
                "VIRTUAL_CAPTURE_CLEAR",
                "TRANSPORT_SEALED",
                {
                    "model_name": virtual_name,
                    "ready_to_drive": False,
                    "next_stage": "TRANSPORT_MOVE",
                },
            )
            return

        lift_pose = self._world_vertical_offset(
            refined_candidate["grasp"], self.lift_distance
        )
        self.pose_pubs["lift"].publish(lift_pose)
        with self.lock:
            self.debug_poses["lift"] = copy.deepcopy(lift_pose)
        self._publish_status("LIFT")
        self._motion_or_raise(
            self._cartesian_to(lift_pose, self.lift_speed, stage="LIFT"),
            "LIFT_FAILED",
        )
        self._record_validation_snapshot("lift_complete")
        self._verify_lift(initial_model_pose)
        self.metrics["lifted"] = True

        self._publish_status("HOLD")
        self._verify_hold()
        self._set_retained(True)

    def _run_dry_trajectory(self, target, yaw, tilt):
        dry_target = self._world_vertical_offset(target, self.dry_clearance)
        dry_candidate = self._candidate_from_angles(dry_target, yaw, tilt)
        self._publish_status("DRY_APPROACH")
        self._motion_or_raise(
            self._cartesian_to(
                dry_candidate["approach"], self.approach_speed,
                stage="DRY_APPROACH_START",
            ),
            "APPROACH_FAILED",
        )
        self._motion_or_raise(
            self._cartesian_to(
                dry_candidate["grasp"], self.approach_speed,
                stage="DRY_APPROACH_FINAL",
            ),
            "APPROACH_FAILED",
        )
        self.metrics["approach_success"] = True
        lift_pose = self._world_vertical_offset(
            dry_candidate["grasp"], self.lift_distance
        )
        self.pose_pubs["lift"].publish(lift_pose)
        self._publish_status("DRY_LIFT")
        self._motion_or_raise(
            self._cartesian_to(lift_pose, self.lift_speed, stage="DRY_LIFT"),
            "LIFT_FAILED",
        )
        self.metrics["lifted"] = True
        self._verify_arm_holding(self.hold_duration)

    def _wait_interfaces(self, require_moveit):
        clients = [self.arm_client, self.gripper_client]
        if require_moveit:
            clients.extend([self.move_group_client, self.execute_client])
        names = ["arm controller", "gripper controller", "move_group", "execute_trajectory"]
        for index, client in enumerate(clients):
            if not self._wait_for_server_wall(client, 6.0):
                raise GraspFailure(
                    "MOTION_TIMEOUT", "{} action unavailable".format(names[index])
                )
        services = [
            self.list_controllers.resolved_name,
            self.ik_service.resolved_name,
            self.fk_service.resolved_name,
            self.cartesian_service.resolved_name,
            self.apply_scene_service.resolved_name,
            self.get_scene_service.resolved_name,
            self.state_validity_service.resolved_name,
        ]
        if self.near_field_relocalization_enabled:
            services.append(self.reset_target_confirmation.resolved_name)
        if self.virtual_transport_enabled:
            services.extend([
                self.delete_model_service.resolved_name,
                self.spawn_model_service.resolved_name,
            ])
        for name in services:
            try:
                rospy.wait_for_service(name, timeout=6.0)
            except rospy.ROSException:
                raise GraspFailure("MOTION_TIMEOUT", "service unavailable: {}".format(name))
        deadline = time.monotonic() + 5.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                ready = self.joint_state is not None and self.arm_state is not None
                ready = ready and self.gripper_state is not None and self.imu is not None
            if self.simulated_lock_fallback_enabled:
                ready = bool(
                    ready
                    and self.force_attach_pub.get_num_connections() > 0
                    and self.force_detach_pub.get_num_connections() > 0
                    and self.force_lock_status_sub.get_num_connections() > 0
                )
            if ready:
                return
            time.sleep(0.05)
        if self.simulated_lock_fallback_enabled:
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "joint/controller/IMU state or simulator force-lock plugin interface is incomplete",
            )
        raise GraspFailure(
            "CONTROLLER_UNSETTLED", "joint/controller/IMU state is incomplete"
        )

    def _check_controllers(self):
        response = self.list_controllers()
        states = {entry.name: entry.state for entry in response.controller}
        required = ["ur5_arm_controller", "gripper_controller"]
        bad = [name for name in required if states.get(name) != "running"]
        if bad:
            raise GraspFailure(
                "CONTROLLER_UNSETTLED", "controllers not running: {}".format(", ".join(bad))
            )

    def _publish_zero(self):
        message = Twist()
        for publisher in self.cmd_publishers:
            publisher.publish(message)

    def _wait_base_stationary(self):
        stable_since = None
        deadline = time.monotonic() + 8.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._publish_zero()
            self._safety_check()
            with self.lock:
                odom = self.odom
            if odom is None:
                time.sleep(0.05)
                continue
            linear = math.sqrt(
                odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2
            )
            angular = abs(odom.twist.twist.angular.z)
            if linear <= self.stationary_linear and angular <= self.stationary_angular:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= self.stationary_duration:
                    return
            else:
                stable_since = None
            time.sleep(0.05)
        raise GraspFailure("BASE_UNSTABLE", "base did not remain stationary")

    def _base_pose_for_observation_gate(self, stamp=None):
        lookup_stamp = stamp
        if lookup_stamp is None or lookup_stamp == rospy.Time():
            lookup_stamp = rospy.Time(0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.gravity_frame, self.planning_frame, lookup_stamp,
                rospy.Duration(0.25),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            raise GraspFailure(
                "TF_FAILED",
                "cannot measure base drift after wrist observation: {}".format(exc),
            )
        rotation = transform.transform.rotation
        yaw = tft.euler_from_quaternion([
            rotation.x, rotation.y, rotation.z, rotation.w,
        ])[2]
        translation = transform.transform.translation
        return {
            "frame_id": self.gravity_frame,
            "x": float(translation.x),
            "y": float(translation.y),
            "yaw": float(yaw),
            "stamp": transform.header.stamp.to_sec(),
            "requested_stamp": lookup_stamp.to_sec(),
        }

    def _make_observation_base_reference(self, observation=None, stage="INITIAL"):
        observation_stamp = (
            observation.header.stamp
            if observation is not None else rospy.Time(0)
        )
        reference = self._base_pose_for_observation_gate(observation_stamp)
        reference["observation_stamp"] = observation_stamp.to_sec()
        reference["stage"] = str(stage)
        return reference

    def _commit_observation_base_reference(self, reference):
        with self.lock:
            self.observation_base_reference = copy.deepcopy(reference)
            self.metrics["observation_base_reference"] = copy.deepcopy(reference)
            self.metrics.setdefault(
                "observation_base_reference_history", []
            ).append(copy.deepcopy(reference))

    def _capture_observation_base_reference(self, observation=None, stage="INITIAL"):
        reference = self._make_observation_base_reference(observation, stage)
        self._commit_observation_base_reference(reference)
        return reference

    def _verify_observation_base_drift(self, stage, reference=None, enforce=True):
        if reference is None:
            with self.lock:
                reference = copy.deepcopy(self.observation_base_reference)
        else:
            reference = copy.deepcopy(reference)
        if reference is None:
            raise GraspFailure(
                "INTERNAL_ERROR", "wrist observation has no base-pose reference"
            )
        current = self._base_pose_for_observation_gate()
        translation = math.hypot(
            current["x"] - reference["x"],
            current["y"] - reference["y"],
        )
        yaw = abs(_wrap_pi(current["yaw"] - reference["yaw"]))
        evidence = {
            "stage": str(stage),
            "translation_m": translation,
            "yaw_error_rad": yaw,
            "yaw_error_deg": math.degrees(yaw),
            "translation_limit_m": self.maximum_base_translation_after_observation,
            "yaw_limit_deg": math.degrees(
                self.maximum_base_yaw_after_observation
            ),
            "current_stamp": current["stamp"],
        }
        with self.lock:
            self.metrics.setdefault("observation_base_drift_checks", []).append(
                evidence
            )
        if enforce and (
                translation > self.maximum_base_translation_after_observation
                or yaw > self.maximum_base_yaw_after_observation):
            raise GraspFailure(
                "BASE_UNSTABLE",
                "{} base moved {:.4f} m / {:.2f} deg after the active wrist "
                "observation (limits {:.4f} m / {:.2f} deg)".format(
                    stage, translation, math.degrees(yaw),
                    self.maximum_base_translation_after_observation,
                    math.degrees(self.maximum_base_yaw_after_observation),
                ),
            )
        return evidence

    def _classify_relocalization_base_motion(self, stage):
        """Classify planned arm-reaction drift without weakening the strict gate."""
        evidence = self._verify_observation_base_drift(
            stage + "_OLD_REFERENCE", enforce=False
        )
        translation = evidence["translation_m"]
        yaw = evidence["yaw_error_rad"]
        if (translation > self.relocalize_maximum
                or yaw > self.maximum_base_yaw_after_observation):
            raise GraspFailure(
                "BASE_UNSTABLE",
                "{} base reaction {:.4f} m / {:.2f} deg exceeds the hard "
                "relocalization bound {:.4f} m / {:.2f} deg".format(
                    stage, translation, math.degrees(yaw),
                    self.relocalize_maximum,
                    math.degrees(self.maximum_base_yaw_after_observation),
                ),
            )
        if translation > self.relocalize_small:
            decision = "RETREAT_AND_REPLAN"
        elif translation > self.maximum_base_translation_after_observation:
            decision = "RELOCALIZE"
        else:
            decision = "WITHIN_STRICT_GATE"
        classified = dict(evidence, decision=decision)
        with self.lock:
            self.metrics.setdefault(
                "pre_relocalization_base_motion", []
            ).append(copy.deepcopy(classified))
        self._publish_status(stage, decision, classified)
        return decision, classified

    def _sample_attitude(self):
        with self.lock:
            imu = self.imu
        if imu is None:
            return 0.0, 0.0
        quaternion = [imu.orientation.x, imu.orientation.y,
                      imu.orientation.z, imu.orientation.w]
        roll, pitch, _ = tft.euler_from_quaternion(quaternion)
        with self.lock:
            self.metrics["max_roll_deg"] = max(
                self.metrics.get("max_roll_deg", 0.0), abs(math.degrees(roll))
            )
            self.metrics["max_pitch_deg"] = max(
                self.metrics.get("max_pitch_deg", 0.0), abs(math.degrees(pitch))
            )
        return roll, pitch

    def _safety_check(self):
        self._publish_zero()
        if (self.action_active and self.action_server.is_active()
                and self.action_server.is_preempt_requested()):
            self._cancel_all_motion()
            raise GraspFailure("CANCELLED", "manipulation action preempted")
        with self.lock:
            release_reattached = bool(
                self.release_motion_guard and self.gazebo_attached
            )
            partial_forced_lock = bool(
                self.forced_lock_active
                and not self.gazebo_attached
                and not self.pending_forced_lock_object
                and not self.releasing
            )
        if release_reattached:
            self._cancel_all_motion()
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "grasp-fix reattached after PLACE release; retreat stopped",
            )
        if partial_forced_lock:
            self._cancel_all_motion()
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "force-lock acknowledgement exists without the matching grasp "
                "event; motion is interlocked until reset",
            )
        roll, pitch = self._sample_attitude()
        if abs(roll) > self.max_roll or abs(pitch) > self.max_pitch:
            self._cancel_all_motion()
            raise GraspFailure(
                "BASE_UNSTABLE",
                "roll={:.2f} deg pitch={:.2f} deg".format(
                    math.degrees(roll), math.degrees(pitch)
                ),
            )

    def _wait_action(self, client, timeout, enforce_safety=True):
        # Gazebo may run below real time while CPU YOLO inference and MoveIt are
        # active.  Trajectory durations use /clock, while this safety loop uses
        # wall time, so retain a bounded wall-time scale without weakening any
        # roll/pitch checks performed on every iteration.
        deadline = time.monotonic() + timeout * self.action_wall_timeout_scale
        terminal_states = {
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        }
        terminal_since = None
        terminal_state = None
        while not rospy.is_shutdown():
            now = time.monotonic()
            # SimpleActionClient.wait_for_result uses simulated ROS time even
            # for its short timeout.  If Gazebo /clock freezes, that one call
            # never returns and defeats this wall-clock safety deadline.
            state = client.get_state()
            if state in terminal_states:
                # actionlib publishes status and result independently.  Under
                # load, SimpleActionClient can observe SUCCEEDED several
                # milliseconds before its result callback runs.  Every caller
                # reads get_result() immediately after this helper, so wait for
                # that payload on the same wall-clock loop.  LOST is the only
                # terminal state for which no result is expected.
                if state == GoalStatus.LOST or client.get_result() is not None:
                    return True
                if terminal_since is None or state != terminal_state:
                    terminal_since = now
                    terminal_state = state
                if now - terminal_since >= self.action_result_grace:
                    rospy.logwarn(
                        "[MineGrasp] action reached terminal state %s but no "
                        "result arrived within %.2f wall seconds",
                        state, self.action_result_grace,
                    )
                    return True
            elif now >= deadline:
                client.cancel_goal()
                return False
            elif terminal_since is not None:
                # A transient terminal observation must not shorten the main
                # action deadline if the client's tracked state changes.
                terminal_since = None
                terminal_state = None
            self._publish_zero()
            if enforce_safety:
                self._safety_check()
            time.sleep(0.05)
        client.cancel_goal()
        return False

    @staticmethod
    def _missing_result_allows_measured_settle(action_state):
        """Limit result-loss recovery to controller terminal outcomes.

        actionlib can deliver a terminal status before (or, under heavy ROS
        load, without) the matching result payload.  A SUCCEEDED/ABORTED
        controller goal may therefore be classified from fresh measured state;
        protocol/rejection states must never be converted into success.
        """
        return action_state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED)

    @staticmethod
    def _action_server_connected(client):
        """Non-blocking equivalent of actionlib's ROS-time wait_for_server."""
        try:
            action_client = client.action_client
            status = action_client.last_status_msg
            if status is None:
                return False
            server_id = status._connection_header.get("callerid")
            if not server_id:
                return False
            if not action_client.pub_goal.impl.has_connection(server_id):
                return False
            if not action_client.pub_cancel.impl.has_connection(server_id):
                return False
            result_ok = any(
                connection.callerid_pub == server_id
                for connection in action_client.result_sub.impl.connections
            )
            feedback_ok = any(
                connection.callerid_pub == server_id
                for connection in action_client.feedback_sub.impl.connections
            )
            return result_ok and feedback_ok
        except (AttributeError, KeyError, TypeError):
            return False

    @classmethod
    def _wait_for_server_wall(cls, client, timeout):
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if cls._action_server_connected(client):
                return True
            time.sleep(0.02)
        return cls._action_server_connected(client)

    def _record_motion_result(self, stage, result):
        event = {
            "stage": str(stage),
            "ok": bool(result.ok),
            "code": str(result.code),
            "detail": str(result.detail),
            "measurements": copy.deepcopy(result.measurements),
            "wall_time": time.time(),
            "sim_time": rospy.Time.now().to_sec(),
        }
        with self.lock:
            self.last_motion_measurements = copy.deepcopy(result.measurements)
            self.metrics.setdefault("motion_events", []).append(event)
        self._publish_motion_diagnostics(stage, result.code, result.measurements)
        return result

    def _amend_latest_motion_result(self, stage, result):
        """Keep the recorded event consistent with late controller evidence."""
        with self.lock:
            events = self.metrics.setdefault("motion_events", [])
            if events and events[-1].get("stage") == str(stage):
                events[-1]["ok"] = bool(result.ok)
                events[-1]["code"] = str(result.code)
                events[-1]["detail"] = str(result.detail)
                events[-1]["measurements"] = copy.deepcopy(
                    result.measurements
                )
            self.last_motion_measurements = copy.deepcopy(result.measurements)
        self._publish_motion_diagnostics(
            stage, result.code, result.measurements
        )
        return result

    def _motion_or_raise(self, result, fallback_code):
        if result:
            return result
        code = result.code if result.code in FAILURE_CODES else fallback_code
        raise GraspFailure(code, result.detail or "motion failed")

    def _arm_measurement(self):
        with self.lock:
            state = copy.deepcopy(self.arm_state)
            joint_state = copy.deepcopy(self.joint_state)
        if state is not None and state.actual.positions:
            names = list(state.joint_names)
            positions = dict(zip(names, state.actual.positions))
            velocities = dict(zip(names, state.actual.velocities))
            if all(name in positions for name in self.arm_joint_names):
                return (
                    [float(positions[name]) for name in self.arm_joint_names],
                    ([float(velocities[name]) for name in self.arm_joint_names]
                     if all(name in velocities for name in self.arm_joint_names)
                     else None),
                    state.header.stamp.to_sec(),
                )
        if joint_state is None:
            return None, None, None
        positions = dict(zip(joint_state.name, joint_state.position))
        velocities = dict(zip(joint_state.name, joint_state.velocity))
        if not all(name in positions for name in self.arm_joint_names):
            return None, None, None
        return (
            [float(positions[name]) for name in self.arm_joint_names],
            ([float(velocities[name]) for name in self.arm_joint_names]
             if all(name in velocities for name in self.arm_joint_names)
             else None),
            joint_state.header.stamp.to_sec(),
        )

    def _tcp_pose_errors(self, target_pose):
        if target_pose is None:
            return None
        target = self._pose_in_gravity_for_marker(target_pose)
        if target is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.gravity_frame, self.tcp_link, rospy.Time(0),
                rospy.Duration(0.1),
            )
        except Exception:
            return None
        dx = float(transform.transform.translation.x - target.pose.position.x)
        dy = float(transform.transform.translation.y - target.pose.position.y)
        dz = float(transform.transform.translation.z - target.pose.position.z)
        current_rotation = tft.quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])
        target_rotation = tft.quaternion_matrix([
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ])
        orientation = _rotation_angle(
            np.linalg.inv(target_rotation).dot(current_rotation)
        )
        return {
            "tcp_position_error_m": math.sqrt(dx * dx + dy * dy + dz * dz),
            "tcp_lateral_error_m": math.hypot(dx, dy),
            "tcp_vertical_error_m": abs(dz),
            "tcp_vertical_error_signed_m": dz,
            "tcp_orientation_error_rad": orientation,
            "tcp_orientation_error_deg": math.degrees(orientation),
        }

    def _verify_final_tcp_components(self, target_pose):
        errors = self._tcp_pose_errors(target_pose)
        if errors is None:
            raise GraspFailure(
                "TCP_NOT_REACHED", "final TCP component measurement unavailable"
            )
        with self.lock:
            self.metrics["final_tcp_component_errors"] = copy.deepcopy(errors)
        if (errors["tcp_lateral_error_m"] > self.final_tcp_lateral
                or errors["tcp_vertical_error_m"] > self.final_tcp_vertical
                or errors["tcp_orientation_error_rad"]
                > self.terminal_tcp_orientation):
            raise GraspFailure(
                "TCP_NOT_REACHED",
                "final TCP lateral/vertical/orientation error "
                "{:.4f} m / {:.4f} m / {:.2f} deg exceeds "
                "{:.4f} m / {:.4f} m / {:.2f} deg".format(
                    errors["tcp_lateral_error_m"],
                    errors["tcp_vertical_error_m"],
                    errors["tcp_orientation_error_deg"],
                    self.final_tcp_lateral,
                    self.final_tcp_vertical,
                    math.degrees(self.terminal_tcp_orientation),
                ),
            )
        return errors

    def _wait_motion_stable(self, stage, desired_joints, target_pose=None):
        deadline = time.monotonic() + self.motion_settle_timeout
        window = JointSpanWindow(self.terminal_stability_duration)
        previous = None
        last = {
            "max_joint_error_rad": None,
            "max_joint_velocity_rad_s": None,
            "max_joint_endpoint_velocity_rad_s": None,
            "max_joint_sample_velocity_rad_s": None,
            "max_joint_velocity_for_gate_rad_s": None,
            "joint_velocity_gate_source": "unavailable",
            "controller_velocity_field_disagreement": False,
            "max_joint_span_rad": None,
            "stability_window_s": 0.0,
        }
        if target_pose is not None:
            with self.lock:
                self.debug_poses["commanded_tcp"] = copy.deepcopy(target_pose)
        while not rospy.is_shutdown() and time.monotonic() <= deadline:
            self._safety_check()
            now = time.monotonic()
            positions, velocities, sample_stamp = self._arm_measurement()
            if positions is None:
                time.sleep(0.05)
                continue
            if (previous is not None
                    and sample_stamp <= previous[0] + 1e-9):
                time.sleep(0.02)
                continue
            if velocities is None and previous is not None:
                elapsed = max(sample_stamp - previous[0], 1e-6)
                velocities = [
                    _wrap_pi(value - old) / elapsed
                    for value, old in zip(positions, previous[1])
                ]
            previous = (sample_stamp, list(positions))
            window.add(sample_stamp, positions)
            max_error = max(
                abs(_wrap_pi(actual - desired))
                for actual, desired in zip(positions, desired_joints)
            )
            max_velocity = (
                None if velocities is None else maximum_absolute(velocities)
            )
            max_span = window.maximum_span() if window.samples else None
            endpoint_velocity = (
                window.maximum_endpoint_velocity() if window.ready else None
            )
            sample_velocity = (
                window.maximum_sample_velocity() if window.ready else None
            )
            position_velocity = (
                None if endpoint_velocity is None or sample_velocity is None
                else max(endpoint_velocity, sample_velocity)
            )
            gate_velocity = max_velocity
            velocity_source = "controller_state"
            velocity_disagreement = False
            # The controller's velocity field is accepted directly whenever it
            # is inside the real 0.05 rad/s limit.  If it is just outside that
            # limit, use the independently measured position-window derivative
            # only when the complete 0.5 s window proves that the arm has not
            # moved.  The bounded raw field, span and endpoint checks prevent a
            # genuinely moving arm from being converted into success.
            if (
                max_velocity is not None
                and max_velocity > self.terminal_joint_velocity
                and max_velocity
                <= self.terminal_velocity_field_disagreement_limit
                and window.ready
                and max_span is not None
                and max_span <= self.terminal_joint_span
                and endpoint_velocity is not None
                and endpoint_velocity <= self.terminal_joint_velocity
                and sample_velocity is not None
                and sample_velocity <= self.terminal_joint_velocity
            ):
                gate_velocity = position_velocity
                velocity_source = "position_window_derivative"
                velocity_disagreement = True
            tcp = self._tcp_pose_errors(target_pose)
            roll, pitch = self._sample_attitude()
            last = {
                "joint_names": list(self.arm_joint_names),
                "actual_joint_positions_rad": [float(value) for value in positions],
                "desired_joint_positions_rad": [
                    float(value) for value in desired_joints
                ],
                "joint_position_errors_rad": [
                    float(_wrap_pi(actual - desired))
                    for actual, desired in zip(positions, desired_joints)
                ],
                "actual_joint_velocities_rad_s": (
                    None if velocities is None
                    else [float(value) for value in velocities]
                ),
                "max_joint_error_rad": max_error,
                "max_joint_velocity_rad_s": max_velocity,
                "max_joint_endpoint_velocity_rad_s": endpoint_velocity,
                "max_joint_sample_velocity_rad_s": sample_velocity,
                "max_joint_velocity_for_gate_rad_s": gate_velocity,
                "joint_velocity_gate_source": velocity_source,
                "controller_velocity_field_disagreement": (
                    velocity_disagreement
                ),
                "terminal_velocity_field_disagreement_limit_rad_s": (
                    self.terminal_velocity_field_disagreement_limit
                ),
                "max_joint_span_rad": max_span,
                "stability_window_s": window.coverage,
                "roll_deg": math.degrees(roll),
                "pitch_deg": math.degrees(pitch),
            }
            if tcp:
                last.update(tcp)
            stable, code, detail = classify_stability(
                max_joint_error=max_error,
                max_joint_velocity=gate_velocity,
                max_joint_span=max_span,
                window_ready=window.ready,
                tcp_position_error=(None if tcp is None else tcp["tcp_position_error_m"]),
                tcp_orientation_error=(None if tcp is None else tcp["tcp_orientation_error_rad"]),
                joint_error_tolerance=self.terminal_joint_error,
                joint_velocity_tolerance=self.terminal_joint_velocity,
                joint_span_tolerance=self.terminal_joint_span,
                tcp_position_tolerance=self.terminal_tcp_position,
                tcp_orientation_tolerance=self.terminal_tcp_orientation,
                require_tcp=target_pose is not None,
            )
            with self.lock:
                self.last_motion_measurements = copy.deepcopy(last)
            if stable:
                if velocity_disagreement:
                    detail += (
                        "; controller velocity field {:.6f} rad/s disagreed "
                        "with {:.6f} rad/s endpoint / {:.6f} rad/s maximum "
                        "sample derivative and {:.6f} rad span; bounded "
                        "position evidence used"
                    ).format(
                        max_velocity, endpoint_velocity, sample_velocity,
                        max_span,
                    )
                return self._record_motion_result(
                    stage,
                    MotionExecutionResult(True, "SUCCESS", detail, last),
                )
            time.sleep(0.05)
        stable, code, detail = classify_stability(
            max_joint_error=last.get("max_joint_error_rad"),
            max_joint_velocity=last.get("max_joint_velocity_for_gate_rad_s"),
            max_joint_span=last.get("max_joint_span_rad"),
            window_ready=last.get("stability_window_s", 0.0)
            >= self.terminal_stability_duration,
            tcp_position_error=last.get("tcp_position_error_m"),
            tcp_orientation_error=last.get("tcp_orientation_error_rad"),
            joint_error_tolerance=self.terminal_joint_error,
            joint_velocity_tolerance=self.terminal_joint_velocity,
            joint_span_tolerance=self.terminal_joint_span,
            tcp_position_tolerance=self.terminal_tcp_position,
            tcp_orientation_tolerance=self.terminal_tcp_orientation,
            require_tcp=target_pose is not None,
        )
        if stable and last.get("controller_velocity_field_disagreement", False):
            detail += (
                "; controller velocity field {:.6f} rad/s disagreed with "
                "{:.6f} rad/s endpoint / {:.6f} rad/s maximum sample "
                "derivative and {:.6f} rad span; bounded position evidence "
                "used"
            ).format(
                last["max_joint_velocity_rad_s"],
                last["max_joint_endpoint_velocity_rad_s"],
                last["max_joint_sample_velocity_rad_s"],
                last["max_joint_span_rad"],
            )
        return self._record_motion_result(
            stage, MotionExecutionResult(stable, code, detail, last)
        )

    def _fk_pose_for_joints(self, positions):
        request = GetPositionFKRequest()
        request.header.frame_id = self.arm_base_frame
        request.header.stamp = rospy.Time(0)
        request.fk_link_names = [self.tcp_link]
        request.robot_state = self._current_robot_state()
        names = list(request.robot_state.joint_state.name)
        values = list(request.robot_state.joint_state.position)
        index_by_name = {name: index for index, name in enumerate(names)}
        for name, value in zip(self.arm_joint_names, positions):
            if name in index_by_name:
                values[index_by_name[name]] = float(value)
            else:
                names.append(name)
                values.append(float(value))
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = values
        try:
            response = self.fk_service(request)
        except Exception as exc:
            rospy.logerr("[MineGrasp] FK for terminal gate failed: %s", exc)
            return None
        if (response.error_code.val != MoveItErrorCodes.SUCCESS
                or not response.pose_stamped):
            return None
        return response.pose_stamped[0]

    def _set_goal_tolerances(self, goal, joint_names):
        goal.goal_tolerance = []
        for name in joint_names:
            tolerance = JointTolerance()
            tolerance.name = str(name)
            tolerance.position = self.explicit_goal_tolerance
            tolerance.velocity = 0.0
            tolerance.acceleration = 0.0
            goal.goal_tolerance.append(tolerance)

    def _command_arm(self, positions, duration, enforce_safety,
                     stage="DIRECT_ARM"):
        target_pose = self._fk_pose_for_joints(positions)
        if target_pose is None:
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "TCP_NOT_REACHED",
                    "cannot compute commanded TCP pose for direct joint trajectory",
                ),
            )
        if self._joint_error(self.arm_joint_names, positions) <= self.terminal_joint_error:
            return self._wait_motion_stable(
                stage + "_ALREADY_REACHED", positions, target_pose
            )
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        goal.trajectory.joint_names = list(self.arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = rospy.Duration(2.0)
        self._set_goal_tolerances(goal, self.arm_joint_names)
        started = time.monotonic()
        self.arm_client.send_goal(goal)
        if not self._wait_action(
                self.arm_client, duration + self.timeout_margin, enforce_safety):
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "MOTION_TIMEOUT", "arm controller action timed out",
                    {"action_state": self.arm_client.get_state()},
                ),
            )
        result = self.arm_client.get_result()
        if result is None:
            action_state = self.arm_client.get_state()
            if self._missing_result_allows_measured_settle(action_state):
                settled = self._wait_motion_stable(
                    stage, positions, target_pose
                )
                settled.detail = (
                    "arm action result payload missing in terminal state {}; "
                    "classified from fresh measured joint/TCP stability; "
                    "{}".format(action_state, settled.detail)
                )
                settled.measurements.update({
                    "action_state": int(action_state),
                    "action_result_payload_missing": True,
                })
                if settled:
                    self.metrics["late_controller_success"] = True
                return self._amend_latest_motion_result(
                    stage, settled
                )
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "CONTROLLER_UNSETTLED",
                    "arm controller returned no result in action state {}"
                    .format(action_state),
                    {
                        "action_state": int(action_state),
                        "action_result_payload_missing": True,
                    },
                ),
            )
        if result.error_code not in (
                result.SUCCESSFUL, result.GOAL_TOLERANCE_VIOLATED):
            code = (
                "PLANNING_FAILED"
                if result.error_code in (
                    result.INVALID_GOAL, result.INVALID_JOINTS,
                    result.OLD_HEADER_TIMESTAMP,
                ) else "TRUE_POSITION_ERROR"
            )
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, code,
                    "arm controller error {}: {}".format(
                        result.error_code, result.error_string
                    ),
                    {"fjt_error_code": int(result.error_code)},
                ),
            )
        settled = self._wait_motion_stable(stage, positions, target_pose)
        if result.error_code == result.GOAL_TOLERANCE_VIOLATED and settled:
            settled.detail = (
                "GOAL_TOLERANCE_VIOLATED accepted only after measured stability; "
                + settled.detail
            )
            settled.measurements["accepted_goal_tolerance_violation"] = True
            self.metrics["late_controller_success"] = True
            self._amend_latest_motion_result(stage, settled)
        return settled

    def _gripper_measurement(self):
        with self.lock:
            state = copy.deepcopy(self.gripper_state)
            joint_state = copy.deepcopy(self.joint_state)
        if state is not None and state.actual.positions:
            velocity = (
                float(state.actual.velocities[0])
                if state.actual.velocities else None
            )
            return float(state.actual.positions[0]), velocity
        if joint_state is None or self.gripper_joint not in joint_state.name:
            return None, None
        index = list(joint_state.name).index(self.gripper_joint)
        velocity = (
            float(joint_state.velocity[index])
            if index < len(joint_state.velocity) else None
        )
        return float(joint_state.position[index]), velocity

    def _wait_gripper_stable(self, target, allow_contact, require_contact,
                             contact_not_before_wall):
        deadline = time.monotonic() + self.motion_settle_timeout
        window = JointSpanWindow(self.terminal_stability_duration)
        previous = None
        last_sample_stamp = None
        last = {}
        while not rospy.is_shutdown() and time.monotonic() <= deadline:
            self._safety_check()
            now = time.monotonic()
            actual, velocity = self._gripper_measurement()
            if actual is None:
                time.sleep(0.05)
                continue
            if velocity is None and previous is not None:
                velocity = (actual - previous[1]) / max(now - previous[0], 1e-6)
            previous = (now, actual)
            with self.lock:
                state = copy.deepcopy(self.gripper_state)
            sample_stamp = (
                state.header.stamp.to_sec()
                if state is not None else now
            )
            if (last_sample_stamp is None
                    or sample_stamp > last_sample_stamp + 1e-9):
                window.add(sample_stamp, [actual])
                last_sample_stamp = sample_stamp
            with self.lock:
                attached = bool(self.gazebo_attached)
                event_wall = self.last_grasp_event_wall
            fresh_contact = bool(
                attached and event_wall is not None
                and event_wall >= contact_not_before_wall - 0.1
            )
            position_error = abs(actual - float(target))
            contact_substitute = bool(allow_contact and fresh_contact)
            position_ok = (
                position_error <= self.gripper_terminal_position
                or contact_substitute
            )
            raw_velocity_ok = (
                velocity is not None
                and abs(velocity) <= self.gripper_terminal_velocity
            )
            span = window.maximum_span()
            endpoint_velocity = window.maximum_endpoint_velocity()
            # Under a fresh opposing-contact lock, require both a bounded raw
            # sample and negligible measured displacement over the full
            # stability window.  The wider raw bound only covers Gazebo's
            # contact impulse; it is never used for an idle/open gripper.
            contact_velocity_ok = bool(
                contact_substitute
                and velocity is not None
                and abs(velocity) <= self.gripper_contact_raw_velocity
                and endpoint_velocity <= self.gripper_terminal_velocity
            )
            velocity_ok = raw_velocity_ok or contact_velocity_ok
            last = {
                "gripper_target_rad": float(target),
                "gripper_actual_rad": actual,
                "gripper_position_error_rad": position_error,
                "gripper_velocity_rad_s": velocity,
                "gripper_window_velocity_rad_s": endpoint_velocity,
                "gripper_raw_velocity_limit_rad_s": (
                    self.gripper_contact_raw_velocity
                    if contact_substitute else self.gripper_terminal_velocity
                ),
                "gripper_span_rad": span,
                "gripper_stability_window_s": window.coverage,
                "grasp_fix_attached": attached,
                "fresh_grasp_fix_contact": fresh_contact,
            }
            if (position_ok and velocity_ok and window.ready
                    and span <= self.terminal_joint_span
                    and (not require_contact or fresh_contact)):
                detail = "measured gripper state is stable"
                if contact_velocity_ok and not raw_velocity_ok:
                    detail += (
                        "; fresh opposing contact is stationary over the "
                        "full position window despite a bounded solver impulse"
                    )
                return MotionExecutionResult(True, "SUCCESS", detail, last)
            time.sleep(0.05)
        if require_contact and not last.get("fresh_grasp_fix_contact", False):
            detail = "grasp-fix did not report a fresh opposing-contact attachment"
        elif last.get("gripper_position_error_rad", float("inf")) > self.gripper_terminal_position:
            detail = "measured gripper position did not reach target and no contact justified the stop"
        else:
            detail = "gripper velocity/span did not settle"
        return MotionExecutionResult(False, "GRIPPER_FAILED", detail, last)

    def _wait_idle_gripper_safe(self):
        """Reject an oscillating or out-of-limit jaw before moving the arm."""
        deadline = time.monotonic() + self.motion_settle_timeout
        window = JointSpanWindow(self.terminal_stability_duration)
        last_sample_stamp = None
        last = {}
        margin = 0.005
        while not rospy.is_shutdown() and time.monotonic() <= deadline:
            self._safety_check()
            with self.lock:
                state = copy.deepcopy(self.gripper_state)
            if state is None or not state.actual.positions:
                time.sleep(0.05)
                continue
            sample_stamp = state.header.stamp.to_sec()
            if (last_sample_stamp is not None
                    and sample_stamp <= last_sample_stamp + 1e-9):
                time.sleep(0.02)
                continue
            last_sample_stamp = sample_stamp
            position = float(state.actual.positions[0])
            velocity = (
                float(state.actual.velocities[0])
                if state.actual.velocities else None
            )
            window.add(sample_stamp, [position])
            within_limits = (
                self.gripper_joint_lower - margin <= position
                <= self.gripper_joint_upper + margin
            )
            last = {
                "gripper_actual_rad": position,
                "gripper_velocity_rad_s": velocity,
                "gripper_span_rad": window.maximum_span(),
                "gripper_stability_window_s": window.coverage,
                "gripper_lower_limit_rad": self.gripper_joint_lower,
                "gripper_upper_limit_rad": self.gripper_joint_upper,
                "fresh_controller_stamp": sample_stamp,
                "gripper_command_count": len(
                    self.metrics.get("gripper_fjt_goals", [])
                ),
            }
            if not within_limits:
                return self._record_motion_result(
                    "CHECK_GRIPPER_IDLE",
                    MotionExecutionResult(
                        False, "GRIPPER_FAILED",
                        "idle gripper is outside its URDF joint limits; "
                        "mechanical/mimic oscillation detected",
                        last,
                    ),
                )
            if (velocity is not None
                    and abs(velocity) <= self.gripper_terminal_velocity
                    and window.ready
                    and window.maximum_span() <= self.terminal_joint_span):
                return self._record_motion_result(
                    "CHECK_GRIPPER_IDLE",
                    MotionExecutionResult(
                        True, "SUCCESS",
                        "idle gripper is within limits and stable on fresh samples",
                        last,
                    ),
                )
            time.sleep(0.02)
        return self._record_motion_result(
            "CHECK_GRIPPER_IDLE",
            MotionExecutionResult(
                False, "GRIPPER_FAILED",
                "idle gripper velocity/span did not settle on fresh samples",
                last,
            ),
        )

    def _command_gripper(self, position, duration, enforce_safety,
                         allow_contact=False, require_contact=False,
                         stage="GRIPPER", contact_not_before_wall=None):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        goal.trajectory.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.velocities = [0.0]
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = rospy.Duration(2.0)
        self._set_goal_tolerances(goal, [self.gripper_joint])
        started = time.monotonic()
        self.gripper_client.send_goal(goal)
        if not self._wait_action(
                self.gripper_client, duration + self.timeout_margin, enforce_safety):
            return self._record_motion_result(
                stage,
                MotionExecutionResult(False, "MOTION_TIMEOUT", "gripper action timed out"),
            )
        result = self.gripper_client.get_result()
        if result is None:
            action_state = self.gripper_client.get_state()
            if self._missing_result_allows_measured_settle(action_state):
                settled = self._wait_gripper_stable(
                    position,
                    allow_contact,
                    require_contact,
                    started if contact_not_before_wall is None
                    else float(contact_not_before_wall),
                )
                settled.detail = (
                    "gripper action result payload missing in terminal state "
                    "{}; classified from fresh measured finger/contact state; "
                    "{}".format(action_state, settled.detail)
                )
                settled.measurements.update({
                    "action_state": int(action_state),
                    "action_result_payload_missing": True,
                })
                return self._record_motion_result(stage, settled)
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "GRIPPER_FAILED",
                    "gripper controller returned no result in action state {}"
                    .format(action_state),
                    {
                        "action_state": int(action_state),
                        "action_result_payload_missing": True,
                    },
                ),
            )
        if result.error_code not in (
                result.SUCCESSFUL, result.GOAL_TOLERANCE_VIOLATED):
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "GRIPPER_FAILED",
                    "gripper controller error {}: {}".format(
                        None if result is None else result.error_code,
                        "" if result is None else result.error_string,
                    ),
                ),
            )
        settled = self._wait_gripper_stable(
            position,
            allow_contact,
            require_contact,
            started if contact_not_before_wall is None
            else float(contact_not_before_wall),
        )
        if (result.error_code == result.GOAL_TOLERANCE_VIOLATED
                and settled.ok):
            settled.detail = (
                "GOAL_TOLERANCE_VIOLATED accepted from measured finger "
                "position/velocity and grasp-fix state; " + settled.detail
            )
            settled.measurements["accepted_goal_tolerance_violation"] = True
        return self._record_motion_result(stage, settled)

    def _open_gripper_for_release(self):
        """Open and verify the measured joint before waiting for detach.

        A late actionlib transition from the preceding close trajectory can
        occasionally preempt the first reset goal.  Merely sending an open
        goal is therefore insufficient: moving the fixture while the fingers
        remain closed would leave gazebo_grasp_fix's internal joint alive.
        Retry a bounded number of times and only proceed once `/joint_states`
        proves that the gripper is physically open.
        """
        last_actual = self._gripper_actual_position()
        for attempt in range(1, self.gripper_reset_attempts + 1):
            self.gripper_client.cancel_all_goals()
            time.sleep(self.gripper_reset_retry_delay)
            action_ok = self._command_gripper(
                self.gripper_open, self.gripper_duration,
                enforce_safety=False,
                stage="RESET_OPEN_{}".format(attempt),
            )
            verify_deadline = time.monotonic() + 1.0
            while not rospy.is_shutdown() and time.monotonic() < verify_deadline:
                self._publish_zero()
                last_actual = self._gripper_actual_position()
                if (last_actual is not None and
                        abs(last_actual - self.gripper_open) <=
                        self.gripper_reset_open_tolerance):
                    return
                time.sleep(0.02)
            self._publish_status(
                "RESET_OPEN_RETRY", "RUNNING", {
                    "attempt": attempt,
                    "action_ok": bool(action_ok),
                    "actual_position": last_actual,
                },
            )
        raise GraspFailure(
            "GRIPPER_FAILED",
            "reset could not open gripper after {} attempts; actual={}".format(
                self.gripper_reset_attempts, last_actual
            ),
        )

    def _close_gripper_staged(self):
        """Close gently through geometry-calibrated gap setpoints.

        A single command to q=0.513 corresponds to a 36.996 mm inner gap and
        squeezes the 40 mm detonator by 3 mm.  In position-controlled Gazebo
        that can eject the freely resting mine before opposing contacts exist.
        The square detonator's projected width changes with its yaw, so no
        absolute q value can represent contact for every trial.  Once the
        physical plugin reports opposing contact, close only a small delta
        relative to the *measured contact joint position* and require that
        attachment to remain continuous before lifting.  In the integrated
        simulator mission, establish the guarded fixed joint while the jaws
        are still open.  That prevents the long first close from pushing the
        mine/ground and applying an overturning moment to the stopped UGV.
        """
        actual = self._gripper_actual_position()
        previous = self.gripper_open if actual is None else actual
        close_started_wall = time.monotonic()
        prelocked = False
        if (self.simulated_lock_fallback_enabled
                and self.simulated_lock_before_close):
            if actual is None:
                raise GraspFailure(
                    "GRIPPER_FAILED",
                    "pre-close simulator lock requires a measured open jaw",
                )
            self._publish_status(
                "SIM_LOCK_PRE_CLOSE", "RUNNING", {
                    "joint_position": float(actual),
                },
            )
            if not self._try_simulated_lock(
                    joint_position=actual, allow_open=True):
                raise GraspFailure(
                    "GAZEBO_ATTACH_FAILED",
                    "pre-close simulator lock failed; refusing to let an "
                    "unlocked jaw push the mine or ground: {}".format(
                        self.metrics.get("simulated_lock_rejection", "unknown")
                    ),
                )
            prelocked = True
            self.metrics["simulated_lock_pre_close"] = True
        for index, position in enumerate(self.gripper_close_positions):
            close_speed = (
                self.simulated_lock_close_speed
                if prelocked else self.gripper_close_speed
            )
            duration = max(
                self.gripper_close_min_duration,
                abs(position - previous) / close_speed,
            )
            self._publish_status(
                "CLOSE_GRIPPER_STAGE",
                extra={
                    "stage_index": index,
                    "joint_target": position,
                    "duration": duration,
                    "simulator_prelocked": prelocked,
                },
            )
            if not self._command_gripper(
                    position, duration, enforce_safety=True,
                    allow_contact=not prelocked,
                    stage="CLOSE_GRIPPER_STAGE_{}".format(index),
                    contact_not_before_wall=close_started_wall):
                if prelocked:
                    # The fixed joint is already authoritative.  A strict jaw
                    # action result must not discard a safe lock when the
                    # measured jaw nevertheless entered the visual close
                    # window; _secure_forced_lock performs that measurement.
                    return self._secure_forced_lock(position)
                return False
            previous = self._gripper_actual_position()
            if previous is None:
                previous = position
            self._record_validation_snapshot(
                "gripper_close_{:03d}".format(int(round(position * 1000.0)))
            )
            deadline = time.monotonic() + self.gripper_contact_settle
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                self._safety_check()
                with self.lock:
                    attached = self.gazebo_attached
                    attached_object = self.gazebo_attached_object
                if attached:
                    self.metrics["gripper_first_contact_command"] = float(position)
                    self.metrics["gripper_first_contact_position"] = float(previous)
                    # Upgrade even a contact-created joint to the explicit
                    # transport lock.  This removes the attach->detach race
                    # measured on M001 while retaining all association gates.
                    if self.simulated_lock_fallback_enabled:
                        forced = self._try_simulated_lock(
                            expected_object=attached_object,
                            joint_position=previous,
                        )
                        if forced:
                            # Acknowledged fixed-joint retention is already
                            # stronger than finger preload.  Do not issue the
                            # legacy +1 mrad squeeze: the measured M001 run
                            # showed that it made the position-controlled jaw
                            # fight the fixed object (0.27 rad/s oscillation),
                            # aborting an otherwise successful simulator lock.
                            return self._secure_forced_lock(previous)
                    return self._secure_contact_grasp(
                        previous, close_started_wall
                    )
                time.sleep(0.02)
            if (self.simulated_lock_fallback_enabled
                    and index >= self.simulated_lock_trigger_stage
                    and not self.metrics.get("simulated_lock_attempted", False)):
                # Give ordinary opposing contact the complete settle window
                # first.  If it is absent, lock the uniquely associated model
                # before later squeeze stages can push the freely resting mine.
                if self._try_simulated_lock(joint_position=previous):
                    self.metrics["gripper_first_contact_command"] = float(position)
                    self.metrics["gripper_first_contact_position"] = float(previous)
                    return self._secure_forced_lock(previous)
        return True

    def _try_simulated_lock(self, joint_position, expected_object="",
                            allow_open=False):
        """Request a pose-preserving Gazebo joint after all visual gates.

        Failure is non-destructive: the staged physical contact search may
        continue.  Success requires both the plugin's explicit acknowledgement
        and the ordinary grasp event proving that the physical joint exists.
        """
        if not self.simulated_lock_fallback_enabled:
            return False
        self.metrics["simulated_lock_attempted"] = True
        if allow_open:
            minimum_close = self.gripper_open - self.gripper_reset_open_tolerance
            maximum_close = self.gripper_open + self.gripper_reset_open_tolerance
            lock_phase = "PRE_CLOSE"
        else:
            minimum_close = (
                self.gripper_close_positions[self.simulated_lock_trigger_stage]
                - self.gripper_terminal_position
            )
            maximum_close = self.gripper_closed + self.gripper_terminal_position
            lock_phase = "CLOSED_WINDOW"
        if (not math.isfinite(float(joint_position))
                or float(joint_position) < minimum_close
                or float(joint_position) > maximum_close):
            reason = (
                "measured gripper position {:.4f} rad is outside the lock "
                "window {:.4f}..{:.4f} rad".format(
                    float(joint_position), minimum_close, maximum_close,
                )
            )
            self.metrics["simulated_lock_rejection"] = reason
            self._publish_status(
                "SIM_LOCK_REJECTED", "GRIPPER_FAILED", {"detail": reason}
            )
            return False
        with self.lock:
            gated_pose = copy.deepcopy(self.simulated_lock_visual_gate_pose)
        if gated_pose is None:
            reason = "final visual TCP gate was not established for this attempt"
            self.metrics["simulated_lock_rejection"] = reason
            self._publish_status(
                "SIM_LOCK_REJECTED", "TCP_NOT_REACHED", {"detail": reason}
            )
            return False
        try:
            # Re-check after the first jaw motion so a displaced/unstable arm
            # cannot gain a fixed joint from an earlier valid measurement.
            lock_errors = self._verify_final_tcp_components(gated_pose)
            self.metrics["simulated_lock_tcp_gate"] = copy.deepcopy(lock_errors)
        except GraspFailure as exc:
            self.metrics["simulated_lock_rejection"] = exc.detail
            self._publish_status(
                "SIM_LOCK_REJECTED", exc.code, {"detail": exc.detail}
            )
            return False
        try:
            candidate = self._select_simulated_lock_candidate()
        except GraspFailure as exc:
            reason = "{}: {}".format(exc.code, exc.detail)
            self.metrics["simulated_lock_rejection"] = reason
            self._publish_status(
                "SIM_LOCK_REJECTED", exc.code, {"detail": exc.detail}
            )
            return False

        object_name = candidate["collision_name"]
        if expected_object and expected_object != object_name:
            reason = (
                "current grasp event names {}, but unique task candidate is {}"
                .format(expected_object, object_name)
            )
            self.metrics["simulated_lock_rejection"] = reason
            self._publish_status(
                "SIM_LOCK_REJECTED", "GAZEBO_ATTACH_FAILED",
                {"detail": reason},
            )
            return False

        with self.lock:
            if (self.forced_lock_active
                    and self.forced_lock_object == object_name
                    and self.gazebo_attached
                    and self.gazebo_attached_object == object_name):
                self.metrics["simulated_lock_used"] = True
                return True
            self.pending_forced_lock_object = object_name
        self.metrics["simulated_lock_candidate"] = copy.deepcopy(candidate)
        self._publish_status(
            "SIM_LOCK_REQUEST", "RUNNING", {
                "object": object_name,
                "joint_position": float(joint_position),
                "lock_phase": lock_phase,
                "tcp_xy_error_m": candidate["tcp_xy_error_m"],
                "source_xy_error_m": candidate["source_xy_error_m"],
                "vertical_error_m": candidate["vertical_error_m"],
            },
        )

        deadline = time.monotonic() + self.simulated_lock_request_timeout
        next_publish = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._safety_check()
            now = time.monotonic()
            if now >= next_publish:
                self.force_attach_pub.publish(String(data=object_name))
                next_publish = now + 0.15
            with self.lock:
                acknowledged = bool(
                    self.forced_lock_active
                    and self.forced_lock_object == object_name
                )
                physically_attached = bool(
                    self.gazebo_attached
                    and self.gazebo_attached_object == object_name
                )
            if acknowledged and physically_attached:
                with self.lock:
                    self.pending_forced_lock_object = ""
                self.metrics["simulated_lock_used"] = True
                self.metrics["simulated_lock_object"] = object_name
                self.metrics["simulated_lock_joint_position"] = float(
                    joint_position
                )
                self._publish_status(
                    "SIM_LOCKED", "GAZEBO_ATTACHED",
                    {"object": object_name},
                )
                return True
            time.sleep(0.02)

        with self.lock:
            if self.pending_forced_lock_object == object_name:
                self.pending_forced_lock_object = ""
            partial_ack = bool(
                self.forced_lock_active
                and self.forced_lock_object == object_name
            )
            partial_event = bool(
                self.gazebo_attached
                and self.gazebo_attached_object == object_name
            )
        if partial_ack or partial_event:
            # Either signal means a joint may exist.  Do not continue closing or
            # recover the arm until an operator/reset obtains a clean release.
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "force lock reached a partial state (ack={}, grasp_event={}) "
                "for {}; base/arm motion is interlocked".format(
                    partial_ack, partial_event, object_name
                ),
            )
        reason = (
            "force-lock plugin did not acknowledge both fixed-joint and grasp "
            "event within {:.2f} s".format(self.simulated_lock_request_timeout)
        )
        self.metrics["simulated_lock_rejection"] = reason
        self._publish_status(
            "SIM_LOCK_TIMEOUT", "GAZEBO_ATTACH_FAILED",
            {"detail": reason, "object": object_name},
        )
        return False

    def _secure_forced_lock(self, joint_position):
        """Verify an acknowledged simulator lock without squeezing again.

        The first close stage has already reached a measured, bounded jaw
        position before the force-lock request is allowed.  Once Gazebo has
        acknowledged the fixed joint and emitted its ordinary grasp event,
        additional finger motion cannot improve retention; it only creates a
        controller/constraint fight.  Keep the jaw command unchanged and
        require the two lock signals to remain coherent for a short window.
        """
        object_name = ""
        stable_since = time.monotonic()
        deadline = stable_since + min(0.50, self.gripper_secure_hold)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._safety_check()
            with self.lock:
                forced = bool(self.forced_lock_active)
                object_name = str(self.forced_lock_object)
                attached = bool(self.gazebo_attached)
                attached_object = str(self.gazebo_attached_object)
            if (not forced or not attached or not object_name
                    or attached_object != object_name):
                raise GraspFailure(
                    "GAZEBO_ATTACH_FAILED",
                    "simulator lock became incoherent before lift "
                    "(forced={}, attached={}, forced_object={}, "
                    "grasp_object={})".format(
                        forced, attached, object_name, attached_object
                    ),
                )
            time.sleep(0.02)
        actual = self._gripper_actual_position()
        if actual is None:
            raise GraspFailure(
                "GRIPPER_FAILED",
                "gripper measurement disappeared after simulator lock",
            )
        minimum_close = (
            self.gripper_close_positions[self.simulated_lock_trigger_stage]
            - self.gripper_terminal_position
        )
        if (actual < minimum_close
                or actual > self.gripper_closed + self.gripper_terminal_position):
            raise GraspFailure(
                "GRIPPER_FAILED",
                "jaw left simulator-lock window after attach: {:.4f} rad"
                .format(actual),
            )
        self.metrics["simulated_lock_skipped_contact_squeeze"] = True
        self.metrics["gripper_contact_command"] = float(joint_position)
        self.metrics["gripper_contact_position"] = float(actual)
        self.metrics["gripper_secure_hold_duration"] = float(
            min(0.50, self.gripper_secure_hold)
        )
        self._record_validation_snapshot("gripper_secure_sim_lock")
        self._publish_status(
            "SECURE_SIM_LOCK", "GAZEBO_ATTACHED", {
                "object": object_name,
                "joint_position": float(actual),
                "extra_squeeze_rad": 0.0,
            },
        )
        return True

    def _secure_contact_grasp(self, contact_position,
                              contact_not_before_wall):
        target = min(
            self.gripper_closed,
            float(contact_position) + self.gripper_contact_squeeze,
        )
        duration = max(
            self.gripper_close_min_duration,
            abs(target - contact_position) / self.gripper_secure_speed,
        )
        self._publish_status(
            "SECURE_GRIPPER_CONTACT",
            extra={
                "contact_position": float(contact_position),
                "joint_target": target,
                "squeeze_delta": target - float(contact_position),
                "duration": duration,
            },
        )
        if not self._command_gripper(
                target, duration, enforce_safety=True,
                allow_contact=True, require_contact=True,
                stage="SECURE_GRIPPER_CONTACT",
                contact_not_before_wall=contact_not_before_wall):
            return False
        actual = self._gripper_actual_position()
        self._record_validation_snapshot("gripper_secure_contact")

        stable_since = None
        deadline = time.monotonic() + self.gripper_secure_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._safety_check()
            with self.lock:
                attached = self.gazebo_attached
            now = time.monotonic()
            if attached:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.gripper_secure_hold:
                    self.metrics["gripper_contact_command"] = float(target)
                    self.metrics["gripper_contact_position"] = (
                        None if actual is None else float(actual)
                    )
                    self.metrics["gripper_secure_hold_duration"] = float(
                        self.gripper_secure_hold
                    )
                    return True
            else:
                stable_since = None
            time.sleep(0.02)
        return False

    def _joint_error(self, names, desired):
        with self.lock:
            state = self.joint_state
        if state is None:
            return float("inf")
        values = dict(zip(state.name, state.position))
        if any(name not in values for name in names):
            return float("inf")
        return max(abs(_wrap_pi(values[name] - goal))
                   for name, goal in zip(names, desired))

    def _verify_arm_holding(self, duration):
        start = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic() - start < duration:
            self._safety_check()
            with self.lock:
                state = self.arm_state
            if state is None or not state.error.positions:
                raise GraspFailure("CONTROLLER_UNSETTLED", "arm controller state unavailable")
            if max(abs(value) for value in state.error.positions) > self.hold_tolerance:
                raise GraspFailure(
                    "TRUE_POSITION_ERROR", "arm does not hold commanded joint state"
                )
            time.sleep(0.05)

    def _wait_for_target(self, timeout, not_before=None, wait_stage="WAIT_TARGET"):
        deadline = time.monotonic() + timeout
        prior_detail = ""
        last_status_publish = 0.0
        with self.lock:
            status = self.localizer_status
            required_stamp = self.active_observation_not_before
        if not_before is not None and not_before > required_stamp:
            required_stamp = not_before
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._safety_check()
            with self.lock:
                observation = copy.deepcopy(self.target_observation)
                status = self.localizer_status
                std = list(self.target_std)
            pose = None
            age = None
            observation_stamp = None
            if observation is not None:
                pose = PoseStamped()
                pose.header = copy.deepcopy(observation.header)
                pose.pose = copy.deepcopy(observation.pose.pose)
                age = (rospy.Time.now() - pose.header.stamp).to_sec()
                observation_stamp = pose.header.stamp.to_sec()
                if (0.0 <= age <= self.target_max_age
                        and pose.header.stamp > required_stamp):
                    matches, prior_detail = self._matches_source_prior(pose)
                    if matches:
                        return pose
                    rospy.logwarn_throttle(
                        1.0,
                        "[MineGrasp] rejecting wrist target not associated with "
                        "the selected UAV mine: %s",
                        prior_detail,
                    )
            now_wall = time.monotonic()
            if now_wall - last_status_publish >= 0.5:
                match = re.search(r"\bn=(\d+)\b", status or "")
                prefix = (
                    (status or "TARGET_UNSTABLE").split(":", 1)[0].split()[0]
                )
                self._publish_status(
                    wait_stage,
                    "WAITING_FOR_FRESH_OBSERVATION",
                    {
                        "confirmed_frames": int(match.group(1)) if match else 0,
                        "target_age_s": age,
                        "target_std_m": std,
                        "observation_stamp": observation_stamp,
                        "required_after_stamp": required_stamp.to_sec(),
                        "last_failure_reason": prefix,
                        "localizer_status": status,
                        "source_match": prior_detail,
                    },
                )
                last_status_publish = now_wall
            time.sleep(0.05)
        if prior_detail:
            code = "TF_FAILED" if prior_detail.startswith("TF_FAILED:") else "TARGET_UNSTABLE"
            raise GraspFailure(
                code,
                "stable wrist target does not match selected mine; {}".format(
                    prior_detail
                ),
            )
        prefix = status.split(":", 1)[0].split()[0] if status else "TARGET_UNSTABLE"
        if prefix not in {"NO_DETECTION", "NO_DETONATOR", "INVALID_DEPTH", "TF_FAILED"}:
            prefix = "TARGET_UNSTABLE"
        raise GraspFailure(prefix, "stable target timed out; localizer={}".format(status))

    def _matches_source_prior(self, target):
        """Associate wrist RGB-D with the selected map mine without targeting truth.

        The UAV pose is only a coarse identity prior.  The returned grasp point
        remains the wrist depth measurement; the prior can only reject a yellow
        object belonging to another scene feature/mine.
        """
        with self.lock:
            prior = copy.deepcopy(self.active_source_prior)
            mine_id = self.active_source_mine_id
        if prior is None:
            return True, "standalone execution has no source prior"
        if target.header.stamp == rospy.Time():
            return False, "wrist target has no image timestamp"
        if not prior.header.frame_id:
            return False, "source prior has no frame"
        target_xyz = (
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )
        prior_xyz = (
            prior.pose.position.x,
            prior.pose.position.y,
            prior.pose.position.z,
        )
        if not all(math.isfinite(value) for value in target_xyz + prior_xyz):
            return False, "target/source prior contains NaN or Inf"
        try:
            target_map = target
            if target.header.frame_id != prior.header.frame_id:
                target_map = self.tf_buffer.transform(
                    target, prior.header.frame_id, rospy.Duration(0.5)
                )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            return False, "TF_FAILED: source-prior TF failed: {}".format(exc)
        dx = target_map.pose.position.x - prior.pose.position.x
        dy = target_map.pose.position.y - prior.pose.position.y
        error = math.hypot(dx, dy)
        self.metrics["source_prior_xy_error"] = error
        detail = "M{:03d} prior_xy_error={:.3f} m (limit {:.3f} m)".format(
            int(mine_id or 0), error, self.source_prior_xy_tolerance
        )
        return error <= self.source_prior_xy_tolerance, detail

    def _target_in_gravity(self, target):
        return self._target_in_frame(target, self.gravity_frame, "TF_FAILED")

    def _target_in_frame(self, target, frame, failure_code="TF_FAILED"):
        try:
            return self.tf_buffer.transform(
                target, frame, rospy.Duration(0.5)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            raise GraspFailure(failure_code, str(exc))

    def _arm_hold_goal(self):
        """Snapshot the current controller hold target in canonical joint order."""
        with self.lock:
            state = copy.deepcopy(self.arm_state)
        if state is not None and state.desired.positions:
            desired = dict(zip(state.joint_names, state.desired.positions))
            if all(name in desired for name in self.arm_joint_names):
                return [float(desired[name]) for name in self.arm_joint_names]
        positions, _velocities, _stamp = self._arm_measurement()
        if positions is None:
            raise GraspFailure(
                "CONTROLLER_UNSETTLED",
                "arm state unavailable before near-field observation",
            )
        return list(positions)

    def _fresh_stage_observation(self, stage):
        """Return an atomic target made solely from post-settle RGB-D frames.

        Moving the arm can cause a few millimetres of suspension/chassis
        reaction even while the mission's priority base lock is asserted.  It
        is wrong to compare that planned reaction with the old LOOK-frame base
        reference.  Instead, first prove both base and arm stationary, then
        synchronously reset the localizer's three-frame confirmation epoch.
        The new observation becomes the only geometry source and the new
        strict 5 mm/1 degree base-drift reference.
        """
        self._publish_status(stage + "_SETTLE", "WAITING_FOR_LOCKED_SETTLE")
        self._wait_base_stationary()
        self._motion_or_raise(
            self._wait_motion_stable(
                stage + "_ARM_STABLE", self._arm_hold_goal(), None
            ),
            "CONTROLLER_UNSETTLED",
        )
        # Recheck the base after the measured 0.5 s joint span/velocity gate;
        # the two independent stability conditions therefore overlap rather
        # than relying on the old observation's pose.
        self._wait_base_stationary()
        base_motion_decision, base_motion = (
            self._classify_relocalization_base_motion(stage)
        )
        try:
            response = self.reset_target_confirmation()
        except (rospy.ServiceException, rospy.ROSException) as exc:
            raise GraspFailure(
                "TARGET_UNSTABLE",
                "{} could not reset localizer confirmation epoch: {}".format(
                    stage, exc
                ),
            )
        if not response.success:
            raise GraspFailure(
                "TARGET_UNSTABLE",
                "{} localizer confirmation reset rejected: {}".format(
                    stage, response.message
                ),
            )
        # Record the executor-side boundary only after the synchronous reset
        # returns.  Clearing this subscriber cache is defence in depth; the
        # localizer service is what clears its deque and rejects queued frames.
        with self.lock:
            self.target_observation = None
            self.target_valid = False
        not_before = rospy.Time.now()
        with self.lock:
            self.metrics.setdefault("observation_epochs", []).append({
                "stage": str(stage),
                "executor_not_before": not_before.to_sec(),
                "localizer_epoch": str(response.message),
            })
        self._publish_status(
            stage, "WAITING_FOR_POST_EPOCH_OBSERVATION",
            {
                "required_after_stamp": not_before.to_sec(),
                "localizer_epoch": str(response.message),
            },
        )
        observation = self._wait_for_target(
            timeout=self.target_update_wait,
            not_before=not_before,
            wait_stage=stage,
        )
        # This reference remains provisional until source association (inside
        # _wait_for_target), target-delta classification, ground-guard update,
        # and collision-aware IK have all passed in the caller.
        provisional_reference = self._make_observation_base_reference(
            observation, stage=stage
        )
        self._verify_observation_base_drift(
            stage + "_PROVISIONAL_REFERENCE",
            reference=provisional_reference,
        )
        return (
            observation, provisional_reference,
            base_motion_decision, base_motion,
        )

    def _target_update(self, previous, current, stage):
        old = self._target_in_gravity(previous)
        new = self._target_in_gravity(current)
        dx = float(new.pose.position.x - old.pose.position.x)
        dy = float(new.pose.position.y - old.pose.position.y)
        dz = float(new.pose.position.z - old.pose.position.z)
        lateral = math.hypot(dx, dy)
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        decision = "REGENERATE"
        if distance > self.relocalize_maximum:
            decision = "REJECT"
        elif distance > self.relocalize_small:
            decision = "RETREAT_AND_REPLAN"
        update = {
            "stage": stage,
            "dx_m": dx,
            "dy_m": dy,
            "dz_m": dz,
            "lateral_m": lateral,
            "distance_m": distance,
            "decision": decision,
        }
        self.metrics.setdefault("target_updates", []).append(update)
        self._publish_status(stage, decision, update)
        if decision == "REJECT":
            raise GraspFailure(
                "TARGET_SHIFT_EXCESSIVE",
                "{} target update {:.3f} m (lateral {:.3f}, vertical {:.3f}) "
                "exceeds {:.3f} m; localization/parking must be corrected".format(
                    stage, distance, lateral, abs(dz), self.relocalize_maximum
                ),
            )
        return decision, update

    def _refine_from_pregrasp(self, target, candidate):
        """Relocalize after pregrasp reaction and converge with bounded replans."""
        previous = target
        previous_candidate = candidate
        replans = 0
        while True:
            (refined, provisional_reference,
             base_motion_decision, _base_motion) = (
                self._fresh_stage_observation("REFINE_TARGET")
            )
            decision, update = self._target_update(
                previous, refined, "REFINE_TARGET"
            )
            # The local collision plane follows the accepted RGB-D target; it
            # must be updated before candidate IK/collision checks.
            self._add_ground_guard(refined)
            refined_candidate = self._select_candidate(refined)
            self._publish_candidate_poses(refined_candidate)
            refined_base = (
                refined if refined.header.frame_id == self.arm_base_frame
                else self._target_in_frame(refined, self.arm_base_frame)
            )
            self._record_perception_target(
                "refined_{}".format(replans), refined_base
            )
            self._record_validation_snapshot("target_refined")
            # Candidate generation includes collision-aware IK at pregrasp,
            # approach and grasp.  Guard against the base moving while those
            # service calls were running before accepting the regenerated path.
            self._verify_observation_base_drift(
                "REFINE_TARGET_SELECTION", reference=provisional_reference
            )
            self._commit_observation_base_reference(provisional_reference)
            candidate_changed = (
                abs(refined_candidate["yaw"] - previous_candidate["yaw"]) > 1e-6
                or abs(refined_candidate["tilt"] - previous_candidate["tilt"])
                > 1e-6
            )
            needs_replan = (
                decision == "RETREAT_AND_REPLAN"
                or base_motion_decision == "RETREAT_AND_REPLAN"
                or candidate_changed
            )
            if not needs_replan:
                self.metrics["pregrasp_replans"] = replans
                self._publish_status(
                    "REFINE_TARGET", "TARGET_ACCEPTED",
                    dict(update, pregrasp_replans=replans),
                )
                return refined, refined_candidate
            if replans >= self.maximum_pregrasp_replans:
                raise GraspFailure(
                    "TARGET_UNSTABLE",
                    "pregrasp target still requires replan after {} bounded "
                    "correction(s)".format(replans),
                )
            self._publish_status(
                "REPLAN_PREGRASP", "RETREAT_AND_REPLAN",
                dict(update, candidate_changed=bool(candidate_changed)),
            )
            self._motion_or_raise(
                self._move_group_joints(
                    refined_candidate["pregrasp_joints"],
                    target_pose=refined_candidate["pregrasp"],
                    stage="REPLAN_PREGRASP",
                ),
                "PREGRASP_FAILED",
            )
            # This planned correction may itself react against the suspension.
            # Do not weaken the 5 mm gate; start another fully fresh epoch once
            # the corrected pregrasp is stationary.
            previous = refined
            previous_candidate = refined_candidate
            replans += 1

    def _refine_from_coarse(self, refined, refined_candidate):
        """Relocalize at coarse approach without mixing pregrasp frames."""
        coarse_replans = 0
        while True:
            (coarse_target, provisional_reference,
             base_motion_decision, _base_motion) = (
                self._fresh_stage_observation("REFINE_COARSE")
            )
            decision, update = self._target_update(
                refined, coarse_target, "REFINE_COARSE"
            )
            self._add_ground_guard(coarse_target)
            coarse_candidate = self._select_candidate(coarse_target)
            candidate_changed = (
                abs(coarse_candidate["yaw"] - refined_candidate["yaw"]) > 1e-6
                or abs(coarse_candidate["tilt"] - refined_candidate["tilt"]) > 1e-6
            )
            self._publish_candidate_poses(coarse_candidate)
            coarse_base = (
                coarse_target
                if coarse_target.header.frame_id == self.arm_base_frame
                else self._target_in_frame(coarse_target, self.arm_base_frame)
            )
            self._record_perception_target(
                "coarse_refined_{}".format(coarse_replans), coarse_base
            )
            self._verify_observation_base_drift(
                "REFINE_COARSE_SELECTION", reference=provisional_reference
            )
            self._commit_observation_base_reference(provisional_reference)
            if (decision == "RETREAT_AND_REPLAN"
                    or base_motion_decision == "RETREAT_AND_REPLAN"
                    or candidate_changed):
                if coarse_replans >= self.maximum_coarse_replans:
                    raise GraspFailure(
                        "TARGET_UNSTABLE",
                        "coarse-approach target still requires retreat/replan "
                        "after {} bounded correction(s)".format(coarse_replans),
                    )
                self._publish_status("RETREAT_TO_PREGRASP")
                self._motion_or_raise(
                    self._cartesian_to(
                        refined_candidate["pregrasp"], self.approach_speed,
                        stage="RETREAT_TO_PREGRASP",
                    ),
                    "APPROACH_FAILED",
                )
                self._motion_or_raise(
                    self._move_group_joints(
                        coarse_candidate["pregrasp_joints"],
                        target_pose=coarse_candidate["pregrasp"],
                        stage="COARSE_REPLAN_PREGRASP",
                    ),
                    "PREGRASP_FAILED",
                )
                self._motion_or_raise(
                    self._cartesian_to(
                        coarse_candidate["approach"], self.approach_speed,
                        stage="COARSE_REAPPROACH",
                    ),
                    "APPROACH_FAILED",
                )
                refined = coarse_target
                refined_candidate = coarse_candidate
                coarse_replans += 1
                continue
            # A <=20 mm update only regenerates and collision-checks the
            # approach/grasp targets.  The final Cartesian service will plan
            # from the measured current TCP and still enforce fraction>=0.995;
            # avoiding an unnecessary intermediate correction also avoids
            # creating another unobserved suspension reaction.
            self._publish_status(
                "REFINE_COARSE", "TARGET_ACCEPTED",
                dict(update, coarse_replans=coarse_replans),
            )
            return coarse_target, coarse_candidate

    def _candidate_from_angles(self, target, yaw, tilt):
        gravity_target = self._target_in_gravity(target)
        # The 60 mm pad is as tall as the 60 mm detonator.  Centring it on the
        # block makes its lower edge exactly coplanar with the wider mine disc;
        # even sub-millimetre simulation error then contacts the disc first.
        # Raise the contact band by a geometry-configured clearance while still
        # retaining ample vertical overlap with the yellow block.
        gravity_target.pose.position.z += self.grasp_height_bias
        quaternion = tft.quaternion_from_euler(math.pi, tilt, yaw, axes="sxyz")
        rotation = tft.quaternion_matrix(quaternion)
        approach_axis = rotation[0:3, 2]

        def make_pose(offset):
            result = PoseStamped()
            result.header.frame_id = self.gravity_frame
            result.header.stamp = rospy.Time(0)
            point = np.asarray([
                gravity_target.pose.position.x,
                gravity_target.pose.position.y,
                gravity_target.pose.position.z,
            ]) - approach_axis * offset
            result.pose.position.x = float(point[0])
            result.pose.position.y = float(point[1])
            result.pose.position.z = float(point[2])
            result.pose.orientation.x = float(quaternion[0])
            result.pose.orientation.y = float(quaternion[1])
            result.pose.orientation.z = float(quaternion[2])
            result.pose.orientation.w = float(quaternion[3])
            try:
                converted = self.tf_buffer.transform(
                    result, self.arm_base_frame, rospy.Duration(0.5)
                )
                converted.header.stamp = rospy.Time.now()
                return converted
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
                raise GraspFailure("TF_FAILED", str(exc))

        return {
            "yaw": yaw,
            "tilt": tilt,
            "grasp": make_pose(0.0),
            "approach": make_pose(self.approach_distance),
            "pregrasp": make_pose(self.pregrasp_distance),
        }

    def _publish_candidate_poses(self, candidate):
        for name in ("pregrasp", "approach", "grasp"):
            pose = copy.deepcopy(candidate[name])
            self.pose_pubs[name].publish(pose)
            with self.lock:
                self.debug_poses[name] = copy.deepcopy(pose)
        self._publish_debug_markers()

    @staticmethod
    def _candidate_key(candidate):
        return (
            round(float(candidate["yaw"]), 9),
            round(float(candidate["tilt"]), 9),
        )

    def _candidate_descent_precheck(self, approach_state, grasp_pose, label):
        """Plan the complete approach->grasp line from an analytic IK state."""
        request = GetCartesianPathRequest()
        request.header.frame_id = grasp_pose.header.frame_id
        request.header.stamp = rospy.Time(0)
        request.start_state = copy.deepcopy(approach_state)
        request.start_state.is_diff = False
        request.group_name = self.arm_group
        request.link_name = self.tcp_link
        request.waypoints = [copy.deepcopy(grasp_pose.pose)]
        request.max_step = self.cartesian_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        try:
            response = self.cartesian_service(request)
            error_code = int(response.error_code.val)
            fraction = float(response.fraction)
            detail = ""
        except (rospy.ServiceException, rospy.ROSException) as exc:
            error_code = None
            fraction = 0.0
            detail = str(exc)
        record = {
            "candidate": str(label),
            "cartesian_fraction": fraction,
            "moveit_error_code": error_code,
            "accepted": bool(
                error_code == MoveItErrorCodes.SUCCESS
                and fraction >= self.min_cartesian_fraction
            ),
        }
        if detail:
            record["detail"] = detail
        self.metrics.setdefault("candidate_cartesian_prechecks", []).append(record)
        return record["accepted"], fraction, error_code

    def _select_candidate(self, target, excluded=None):
        current = self._current_arm_positions()
        choices = []
        collision_only_failures = 0
        ik_failures = 0
        descent_failures = []
        excluded = set(excluded or ())
        for tilt in self.tilts:  # top-down (0) is always tried first.
            for yaw in self.yaws:
                candidate = self._candidate_from_angles(target, yaw, tilt)
                if self._candidate_key(candidate) in excluded:
                    continue
                solutions = {}
                failed = False
                for key in ("pregrasp", "approach", "grasp"):
                    solution, failure_kind = self._solve_ik(
                        candidate[key],
                        "yaw={:.1f},tilt={:.1f},{}".format(
                            math.degrees(yaw), math.degrees(tilt), key
                        ),
                    )
                    if solution is None:
                        failed = True
                        if failure_kind == "collision":
                            collision_only_failures += 1
                        else:
                            ik_failures += 1
                        break
                    solutions[key] = solution
                if failed:
                    continue
                pregrasp = self._extract_arm_solution(solutions["pregrasp"])
                grasp = self._extract_arm_solution(solutions["grasp"])
                if not self._safe_joint_configuration(grasp):
                    ik_failures += 1
                    continue
                label = "yaw={:.1f},tilt={:.1f}".format(
                    math.degrees(yaw), math.degrees(tilt)
                )
                descent_ok, fraction, error_code = (
                    self._candidate_descent_precheck(
                        solutions["approach"], candidate["grasp"], label
                    )
                )
                if not descent_ok:
                    descent_failures.append((label, fraction, error_code))
                    continue
                score = sum(abs(_wrap_pi(a - b)) for a, b in zip(pregrasp, current))
                score += abs(tilt) * 3.0 + abs(yaw) * 0.05
                candidate["pregrasp_joints"] = pregrasp
                candidate["grasp_joints"] = grasp
                choices.append((score, candidate))
        if not choices:
            if descent_failures and not collision_only_failures and not ik_failures:
                detail = ", ".join(
                    "{} fraction={:.3f} error={}".format(label, fraction, code)
                    for label, fraction, code in descent_failures
                )
                raise GraspFailure(
                    "PLANNING_FAILED",
                    "no analytic candidate has a complete collision-free "
                    "approach-to-grasp path: " + detail,
                )
            if collision_only_failures and not ik_failures:
                pairs = sorted({
                    "{}<->{}".format(item["body_1"], item["body_2"])
                    for item in self.metrics.get("collision_contacts", [])
                })
                detail = "all analytic candidates collide"
                if pairs:
                    detail += "; contacts=" + ", ".join(pairs)
                raise GraspFailure("COLLISION_FAILED", detail)
            raise GraspFailure("IK_FAILED", "no safe analytic top-down/tilted IK candidate")
        return min(choices, key=lambda item: item[0])[1]

    def _final_approach_with_fallback(self, target, candidate):
        """Execute final descent, changing safe orientation when needed.

        The failed M002 run reached coarse approach normally, but the selected
        yaw=90/tilt=0 Cartesian descent stopped at fraction 0.1875.  Its final
        IK state was valid; this is an interpolation/IK-branch failure, not a
        reason to abandon the physical target.  Every fallback below still
        performs collision-aware IK at pregrasp/approach/grasp and still
        requires the configured >=0.995 Cartesian fraction.
        """
        attempted = set()
        current = candidate
        retries = 0
        last_result = None

        while not rospy.is_shutdown():
            key = self._candidate_key(current)
            attempted.add(key)
            yaw_deg = math.degrees(current["yaw"])
            tilt_deg = math.degrees(current["tilt"])
            stage = (
                "FINAL_APPROACH" if retries == 0
                else "FINAL_APPROACH_RETRY_{}".format(retries)
            )
            result = self._cartesian_to(
                current["grasp"], self.approach_speed, stage=stage
            )
            record = {
                "retry": retries,
                "yaw_deg": yaw_deg,
                "tilt_deg": tilt_deg,
                "cartesian_fraction": self.last_cartesian_fraction,
                "result_code": result.code,
                "result_detail": result.detail,
            }
            self.metrics.setdefault("final_approach_candidates", []).append(record)
            if result:
                self.metrics["candidate_yaw_deg"] = yaw_deg
                self.metrics["candidate_tilt_deg"] = tilt_deg
                return result, current
            last_result = result

            # Execution/controller/attitude failures are not alternative-IK
            # failures.  Stop immediately rather than hiding a real fault.
            if (result.code != "PLANNING_FAILED"
                    or retries >= self.maximum_final_candidate_retries):
                return result, current

            try:
                alternate = self._select_candidate(target, excluded=attempted)
            except GraspFailure:
                break

            next_retry = retries + 1
            self._publish_status(
                "FINAL_APPROACH_REPLAN",
                "ALTERNATE_CANDIDATE",
                {
                    "failed_yaw_deg": yaw_deg,
                    "failed_tilt_deg": tilt_deg,
                    "failed_fraction": self.last_cartesian_fraction,
                    "retry": next_retry,
                    "next_yaw_deg": math.degrees(alternate["yaw"]),
                    "next_tilt_deg": math.degrees(alternate["tilt"]),
                },
            )
            self._publish_candidate_poses(alternate)

            # Reorient only in the high-clearance pregrasp pose.  MoveIt owns
            # obstacle avoidance for this transition; both following straight
            # segments must independently satisfy the strict Cartesian gate.
            pre_result = self._move_group_joints(
                alternate["pregrasp_joints"],
                target_pose=alternate["pregrasp"],
                stage="FINAL_RETRY_{}_PREGRASP".format(next_retry),
            )
            if not pre_result:
                attempted.add(self._candidate_key(alternate))
                self.metrics.setdefault("final_approach_candidates", []).append({
                    "retry": next_retry,
                    "yaw_deg": math.degrees(alternate["yaw"]),
                    "tilt_deg": math.degrees(alternate["tilt"]),
                    "phase": "pregrasp",
                    "result_code": pre_result.code,
                    "result_detail": pre_result.detail,
                })
                # A failed high-clearance transition leaves the measured arm
                # state as the only trustworthy start state.  Do not chain a
                # second fallback from an assumed pose.
                return pre_result, current

            approach_result = self._cartesian_to(
                alternate["approach"],
                self.approach_speed,
                stage="FINAL_RETRY_{}_COARSE".format(next_retry),
            )
            if not approach_result:
                attempted.add(self._candidate_key(alternate))
                self.metrics.setdefault("final_approach_candidates", []).append({
                    "retry": next_retry,
                    "yaw_deg": math.degrees(alternate["yaw"]),
                    "tilt_deg": math.degrees(alternate["tilt"]),
                    "phase": "coarse",
                    "cartesian_fraction": self.last_cartesian_fraction,
                    "result_code": approach_result.code,
                    "result_detail": approach_result.detail,
                })
                return approach_result, current

            self._verify_observation_base_drift(
                "FINAL_RETRY_{}_COARSE".format(next_retry)
            )
            current = alternate
            retries = next_retry

        if last_result is None:
            last_result = MotionExecutionResult(
                False, "PLANNING_FAILED",
                "no final-approach candidate was executable",
            )
        detail = (
            "{}; exhausted {} distinct collision-checked final-approach "
            "candidate(s)"
        ).format(last_result.detail, len(attempted))
        return self._record_motion_result(
            "FINAL_APPROACH_CANDIDATES_EXHAUSTED",
            MotionExecutionResult(
                False,
                "PLANNING_FAILED",
                detail,
                {"candidate_count": len(attempted)},
            ),
        ), current

    def _select_place_candidate(self, target):
        current = self._current_arm_positions()
        choices = []
        for tilt in self.tilts:
            for yaw in self.yaws:
                candidate = self._candidate_from_angles(target, yaw, tilt)
                pre_solution, _ = self._solve_ik(
                    candidate["pregrasp"], "place_pre"
                )
                if pre_solution is None:
                    continue
                place_solution, _ = self._solve_ik(
                    candidate["grasp"], "place_contact"
                )
                if place_solution is None:
                    continue
                pre_joints = self._extract_arm_solution(pre_solution)
                place_joints = self._extract_arm_solution(place_solution)
                if not self._safe_joint_configuration(place_joints):
                    continue
                score = sum(
                    abs(_wrap_pi(a - b)) for a, b in zip(pre_joints, current)
                )
                score += abs(tilt) * 3.0 + abs(yaw - math.pi / 2.0) * 0.05
                candidate["pregrasp_joints"] = pre_joints
                choices.append((score, candidate))
        if not choices:
            raise GraspFailure(
                "PLACE_FAILED", "no collision-free analytic place candidate"
            )
        return min(choices, key=lambda item: item[0])[1]

    def _solve_ik(self, pose, candidate_label):
        collision_response = self._call_ik(pose, avoid_collisions=True)
        if collision_response.error_code.val == MoveItErrorCodes.SUCCESS:
            return collision_response.solution, ""
        free_response = self._call_ik(pose, avoid_collisions=False)
        if free_response.error_code.val == MoveItErrorCodes.SUCCESS:
            self._record_collision_contacts(
                free_response.solution, candidate_label
            )
            return None, "collision"
        return None, "ik"

    def _record_collision_contacts(self, robot_state, candidate_label):
        """Ask MoveIt for the exact invalid-state contact pairs.

        Collision-free IK and unconstrained IK deliberately remain separate:
        the latter is used only for diagnosis and is never executed.
        """
        request = GetStateValidityRequest()
        request.robot_state = copy.deepcopy(robot_state)
        request.group_name = self.arm_group
        try:
            response = self.state_validity_service(request)
        except rospy.ServiceException as exc:
            rospy.logwarn("state-validity diagnosis failed: %s", exc)
            return
        if response.valid:
            # The collision-aware IK call can also reject a solution for a
            # scene update race.  Preserve that fact rather than inventing a
            # collision pair.
            self.metrics.setdefault("collision_diagnostic_notes", []).append(
                "{}: free IK state reported valid".format(candidate_label)
            )
            return
        contacts = self.metrics.setdefault("collision_contacts", [])
        existing = {
            (item["candidate"], item["body_1"], item["body_2"])
            for item in contacts
        }
        for contact in response.contacts:
            body_1, body_2 = sorted([
                str(contact.contact_body_1), str(contact.contact_body_2)
            ])
            key = (candidate_label, body_1, body_2)
            if key in existing:
                continue
            contacts.append({
                "candidate": candidate_label,
                "body_1": body_1,
                "body_2": body_2,
                "depth": float(contact.depth),
            })
            existing.add(key)
            # Bound report size while retaining contacts from multiple poses.
            if len(contacts) >= 48:
                break

    def _call_ik(self, pose, avoid_collisions):
        request = GetPositionIKRequest()
        request.ik_request.group_name = self.arm_group
        request.ik_request.ik_link_name = self.tcp_link
        request.ik_request.pose_stamped = copy.deepcopy(pose)
        request.ik_request.pose_stamped.header.stamp = rospy.Time(0)
        request.ik_request.timeout = rospy.Duration(self.ik_timeout)
        request.ik_request.avoid_collisions = bool(avoid_collisions)
        request.ik_request.robot_state = self._current_robot_state()
        return self.ik_service(request)

    def _current_robot_state(self):
        with self.lock:
            state = copy.deepcopy(self.joint_state)
        result = RobotState()
        if state is not None:
            result.joint_state = state
        result.is_diff = True
        return result

    def _current_arm_positions(self):
        with self.lock:
            state = self.joint_state
        if state is None:
            raise GraspFailure("IK_FAILED", "joint state unavailable")
        values = dict(zip(state.name, state.position))
        try:
            return [float(values[name]) for name in self.arm_joint_names]
        except KeyError as exc:
            raise GraspFailure("IK_FAILED", "joint state missing {}".format(exc))

    def _extract_arm_solution(self, robot_state):
        values = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
        return [float(values[name]) for name in self.arm_joint_names]

    def _safe_joint_configuration(self, values):
        elbow_index = self.arm_joint_names.index("ur5_elbow_joint")
        elbow = abs(_wrap_pi(values[elbow_index]))
        if elbow < self.minimum_elbow_bend:
            return False
        for name, value in zip(self.arm_joint_names, values):
            bounds = self.joint_limits[name]
            if bounds is None:
                continue
            if value <= bounds[0] + self.joint_limit_margin:
                return False
            if value >= bounds[1] - self.joint_limit_margin:
                return False
        return True

    def _recent_arm_controller_result(self, started_wall, wait=0.25):
        deadline = time.monotonic() + max(float(wait), 0.0)
        while not rospy.is_shutdown():
            with self.lock:
                result = copy.deepcopy(self.last_arm_fjt_result)
                result_wall = self.last_arm_fjt_result_wall
            if result is not None and result_wall >= started_wall:
                return result
            if time.monotonic() >= deadline:
                return None
            self._safety_check()
            time.sleep(0.02)
        return None

    @staticmethod
    def _moveit_planning_failure(error_code):
        return error_code in {
            MoveItErrorCodes.PLANNING_FAILED,
            MoveItErrorCodes.INVALID_MOTION_PLAN,
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,
            MoveItErrorCodes.START_STATE_IN_COLLISION,
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS,
            MoveItErrorCodes.GOAL_IN_COLLISION,
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS,
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
            MoveItErrorCodes.INVALID_GROUP_NAME,
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
            MoveItErrorCodes.INVALID_ROBOT_STATE,
            MoveItErrorCodes.NO_IK_SOLUTION,
        }

    @classmethod
    def _moveit_failure_allows_measured_settle(cls, error_code, action_state):
        """Return whether a MoveIt terminal failure may use measured state.

        Planning, collision, cancellation and protocol failures must never be
        hidden by an arm that happens to be near the requested endpoint.  The
        fallback is deliberately limited to MoveIt's execution-time timeout /
        controller-failure codes, plus the already-supported actionlib case in
        which a SUCCEEDED/ABORTED terminal status lost its result payload.
        """
        if not cls._missing_result_allows_measured_settle(action_state):
            return False
        return error_code is None or error_code in (
            MoveItErrorCodes.CONTROL_FAILED,
            MoveItErrorCodes.TIMED_OUT,
        )

    def _moveit_terminal_failure_settled(
            self, error_code, action_state, raw_result, started_wall, stage,
            desired_joints, target_pose):
        if not self._moveit_failure_allows_measured_settle(
                error_code, action_state):
            return None
        # Every integrated call supplies the commanded TCP.  Refuse a future
        # caller that cannot prove the Cartesian endpoint instead of silently
        # degrading this to a joint-only acceptance gate.
        if target_pose is None:
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "TCP_NOT_REACHED",
                    "MoveIt terminal-failure settle requires a commanded TCP",
                    {
                        "moveit_error_code": error_code,
                        "moveit_action_state": int(action_state),
                    },
                ),
            )
        settled = self._wait_motion_stable(stage, desired_joints, target_pose)
        description = (
            "missing result payload" if error_code is None
            else "MoveIt execution error {}".format(error_code)
        )
        settled.detail = (
            "{} in terminal action state {}; classified only from strict "
            "measured joint/TCP/attitude stability; {}".format(
                description, action_state, settled.detail
            )
        )
        settled.measurements.update({
            "moveit_error_code": error_code,
            "moveit_action_state": int(action_state),
            "moveit_result_payload_missing": error_code is None,
            "accepted_moveit_terminal_failure": bool(settled),
        })
        if raw_result is not None:
            settled.measurements.update({
                "fjt_error_code": int(raw_result.result.error_code),
                "fjt_error_string": str(raw_result.result.error_string),
                "fjt_result_age_s": max(
                    0.0, self.last_arm_fjt_result_wall - started_wall
                ),
            })
        if settled:
            self.metrics["late_controller_success"] = True
        return self._amend_latest_motion_result(stage, settled)

    def _move_group_joints(self, positions, target_pose=None,
                           stage="MOVE_GROUP_JOINTS"):
        goal = MoveGroupGoal()
        goal.request.group_name = self.arm_group
        goal.request.num_planning_attempts = self.planning_attempts
        goal.request.allowed_planning_time = self.planning_time
        goal.request.max_velocity_scaling_factor = self.velocity_scale
        goal.request.max_acceleration_scaling_factor = self.acceleration_scale
        goal.request.start_state.is_diff = True
        constraints = Constraints(name="mine_pregrasp")
        for name, position in zip(self.arm_joint_names, positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = self.joint_tolerance
            constraint.tolerance_below = self.joint_tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        started = time.monotonic()
        self.move_group_client.send_goal(goal)
        timeout = self.planning_time + 40.0
        if not self._wait_action(self.move_group_client, timeout, enforce_safety=True):
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "MOTION_TIMEOUT",
                    "MoveIt joint plan/execution exceeded wall timeout",
                    {"move_group_action_state": self.move_group_client.get_state()},
                ),
            )
        result = self.move_group_client.get_result()
        action_state = self.move_group_client.get_state()
        error_code = None if result is None else int(result.error_code.val)
        self.last_moveit_error_code = error_code
        if (result is not None
                and error_code == MoveItErrorCodes.SUCCESS
                and action_state == GoalStatus.SUCCEEDED):
            return self._wait_motion_stable(stage, positions, target_pose)
        raw = self._recent_arm_controller_result(started)
        settled = self._moveit_terminal_failure_settled(
            error_code, action_state, raw, started, stage, positions,
            target_pose
        )
        if settled is not None:
            return settled
        code = (
            "PLANNING_FAILED"
            if result is None or self._moveit_planning_failure(error_code)
            else "CONTROLLER_UNSETTLED"
        )
        return self._record_motion_result(
            stage,
            MotionExecutionResult(
                False, code,
                "MoveIt joint motion failed with error {} (action state {})".format(
                    error_code, self.move_group_client.get_state()
                ),
                {
                    "moveit_error_code": error_code,
                    "fjt_error_code": (
                        None if raw is None else int(raw.result.error_code)
                    ),
                },
            ),
        )

    def _trajectory_arm_endpoint(self, trajectory):
        names = list(trajectory.joint_trajectory.joint_names)
        points = list(trajectory.joint_trajectory.points)
        if not points:
            return None
        endpoint = dict(zip(names, points[-1].positions))
        current = self._current_arm_positions()
        return [
            float(endpoint.get(name, value))
            for name, value in zip(self.arm_joint_names, current)
        ]

    def _cartesian_to(self, pose, speed, stage="CARTESIAN"):
        self.last_cartesian_error = ""
        self.last_cartesian_fraction = None
        request = GetCartesianPathRequest()
        request.header.frame_id = pose.header.frame_id
        request.header.stamp = rospy.Time(0)
        request.start_state.is_diff = True
        request.group_name = self.arm_group
        request.link_name = self.tcp_link
        request.waypoints = [copy.deepcopy(pose.pose)]
        request.max_step = self.cartesian_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        if hasattr(request, "cartesian_speed_limited_link"):
            request.cartesian_speed_limited_link = self.tcp_link
            request.max_cartesian_speed = speed
        try:
            response = self.cartesian_service(request)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            self.last_cartesian_error = "compute_cartesian_path service failed: {}".format(
                exc
            )
            self.metrics["cartesian_error"] = self.last_cartesian_error
            rospy.logerr("[MineGrasp] %s", self.last_cartesian_error)
            return self._record_motion_result(
                stage,
                MotionExecutionResult(False, "PLANNING_FAILED", self.last_cartesian_error),
            )
        self.last_moveit_error_code = int(response.error_code.val)
        self.last_cartesian_fraction = float(response.fraction)
        self.metrics["cartesian_fraction"] = float(response.fraction)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.last_cartesian_error = "MoveIt Cartesian error code {}".format(
                response.error_code.val
            )
            self.metrics["cartesian_error"] = self.last_cartesian_error
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "PLANNING_FAILED", self.last_cartesian_error,
                    {"moveit_error_code": int(response.error_code.val)},
                ),
            )
        if response.fraction < self.min_cartesian_fraction:
            self.last_cartesian_error = "Cartesian path fraction {:.3f} below {:.3f}".format(
                response.fraction, self.min_cartesian_fraction
            )
            self.metrics["cartesian_error"] = self.last_cartesian_error
            rospy.logwarn("[MineGrasp] %s", self.last_cartesian_error)
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "PLANNING_FAILED", self.last_cartesian_error,
                    {"cartesian_fraction": float(response.fraction)},
                ),
            )
        trajectory = response.solution
        if not trajectory.joint_trajectory.points:
            self.last_cartesian_error = "MoveIt returned an empty Cartesian trajectory"
            self.metrics["cartesian_error"] = self.last_cartesian_error
            return self._record_motion_result(
                stage,
                MotionExecutionResult(False, "PLANNING_FAILED", self.last_cartesian_error),
            )
        self._ensure_slow_trajectory(trajectory, pose, speed)
        desired_joints = self._trajectory_arm_endpoint(trajectory)
        if desired_joints is None:
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "PLANNING_FAILED",
                    "Cartesian trajectory has no measurable arm endpoint",
                ),
            )
        goal = ExecuteTrajectoryGoal(trajectory=trajectory)
        started = time.monotonic()
        self.execute_client.send_goal(goal)
        duration = 0.0
        if trajectory.joint_trajectory.points:
            duration = trajectory.joint_trajectory.points[-1].time_from_start.to_sec()
        if not self._wait_action(
                self.execute_client, duration + self.timeout_margin, enforce_safety=True):
            self.last_cartesian_error = (
                "execute_trajectory action exceeded executor wall timeout"
            )
            self.metrics["cartesian_error"] = self.last_cartesian_error
            return self._record_motion_result(
                stage,
                MotionExecutionResult(
                    False, "MOTION_TIMEOUT", self.last_cartesian_error,
                    {"execute_action_state": self.execute_client.get_state()},
                ),
            )
        result = self.execute_client.get_result()
        action_state = self.execute_client.get_state()
        if (result is not None
                and result.error_code.val == MoveItErrorCodes.SUCCESS
                and action_state == GoalStatus.SUCCEEDED):
            self.last_moveit_error_code = int(result.error_code.val)
            return self._wait_motion_stable(stage, desired_joints, pose)
        error_value = None if result is None else result.error_code.val
        self.last_moveit_error_code = error_value
        raw = self._recent_arm_controller_result(started)
        settled = self._moveit_terminal_failure_settled(
            error_value, action_state, raw, started, stage, desired_joints,
            pose
        )
        if settled is not None:
            return settled
        self.last_cartesian_error = (
            "execute_trajectory finished with MoveIt error {} (action state {})"
        ).format(error_value, self.execute_client.get_state())
        self.metrics["cartesian_error"] = self.last_cartesian_error
        code = (
            "PLANNING_FAILED"
            if error_value is None or self._moveit_planning_failure(error_value)
            else "CONTROLLER_UNSETTLED"
        )
        return self._record_motion_result(
            stage,
            MotionExecutionResult(
                False, code, self.last_cartesian_error,
                {
                    "moveit_error_code": error_value,
                    "fjt_error_code": (
                        None if raw is None else int(raw.result.error_code)
                    ),
                    "cartesian_fraction": float(response.fraction),
                },
            ),
        )

    def _ensure_slow_trajectory(self, trajectory, target_pose, speed):
        points = trajectory.joint_trajectory.points
        if not points:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                target_pose.header.frame_id, self.tcp_link,
                rospy.Time(0), rospy.Duration(0.5)
            )
            current = np.asarray([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ])
            target = np.asarray([
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
            ])
            minimum_duration = float(np.linalg.norm(target - current)) / max(speed, 1e-3)
        except Exception:
            minimum_duration = 1.0
        existing = points[-1].time_from_start.to_sec()
        if existing <= 1e-6:
            duration = max(minimum_duration, 1.0)
            count = len(points)
            for index, point in enumerate(points):
                point.time_from_start = rospy.Duration(
                    duration * float(index + 1) / float(count)
                )
                point.velocities = [0.0] * len(point.positions)
                point.accelerations = [0.0] * len(point.positions)
        else:
            scale = max(1.0, minimum_duration / existing)
            if scale > 1.0:
                for point in points:
                    point.time_from_start = rospy.Duration(
                        point.time_from_start.to_sec() * scale
                    )
                    if point.velocities:
                        point.velocities = [value / scale for value in point.velocities]
                    if point.accelerations:
                        point.accelerations = [value / (scale * scale)
                                               for value in point.accelerations]
        # A Cartesian path returned by MoveIt can retain a small non-zero final
        # velocity.  Explicitly command a stopped endpoint; measured velocity
        # still has to pass the independent 0.05 rad/s terminal gate.
        points[-1].velocities = [0.0] * len(points[-1].positions)
        points[-1].accelerations = [0.0] * len(points[-1].positions)

    def _world_vertical_offset(self, pose, distance):
        command = copy.deepcopy(pose)
        command.header.stamp = rospy.Time(0)
        try:
            gravity = self.tf_buffer.transform(
                command, self.gravity_frame, rospy.Duration(0.5)
            )
            gravity.pose.position.z += float(distance)
            gravity.header.stamp = rospy.Time(0)
            result = self.tf_buffer.transform(
                gravity, pose.header.frame_id, rospy.Duration(0.5)
            )
            result.header.stamp = rospy.Time.now()
            return result
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            raise GraspFailure("TF_FAILED", str(exc))

    def _apply_scene(self, scene):
        request = ApplyPlanningSceneRequest(scene=scene)
        try:
            return bool(self.apply_scene_service(request).success)
        except rospy.ServiceException as exc:
            rospy.logerr("apply planning scene failed: %s", exc)
            return False

    def _add_ground_guard(self, target):
        gravity = self._target_in_gravity(target)
        collision = CollisionObject()
        collision.header.frame_id = self.gravity_frame
        collision.id = "mine_ground_guard"
        primitive = SolidPrimitive(type=SolidPrimitive.BOX,
                                   dimensions=[0.28, 0.28, 0.02])
        pose = Pose()
        pose.position.x = gravity.pose.position.x
        pose.position.y = gravity.pose.position.y
        pose.position.z = gravity.pose.position.z - self.detonator_center_z - 0.01
        pose.orientation.w = 1.0
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision]
        if not self._apply_scene(scene):
            raise GraspFailure("COLLISION_FAILED", "could not add local ground guard")

    def _remove_world_object(self, object_id):
        collision = CollisionObject()
        collision.header.frame_id = self.planning_frame
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision]
        self._apply_scene(scene)

    def _wait_world_object_absent(self, object_id, timeout=2.0):
        request = GetPlanningSceneRequest()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            response = self.get_scene_service(request)
            ids = {entry.id for entry in response.scene.world.collision_objects}
            if object_id not in ids:
                return True
            self._remove_world_object(object_id)
            time.sleep(0.05)
        return False

    def _clear_octomap_best_effort(self):
        try:
            rospy.wait_for_service(
                self.clear_octomap_service.resolved_name, timeout=0.75
            )
            self.clear_octomap_service()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            # The current scene normally has no octomap, but clearing it here
            # makes repeated tests deterministic when a sensor updater is
            # enabled by another launch file.
            rospy.logwarn("clear_octomap unavailable during reset: %s", exc)

    def _verify_transient_scene_cleared(self):
        request = GetPlanningSceneRequest()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        deadline = time.monotonic() + 2.0
        remaining = set()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            response = self.get_scene_service(request)
            world_ids = {
                entry.id for entry in response.scene.world.collision_objects
            }
            attached_ids = {
                entry.object.id for entry in
                response.scene.robot_state.attached_collision_objects
            }
            transient = {self.moveit_object_id, "mine_ground_guard"}
            remaining = (world_ids | attached_ids) & transient
            if not remaining:
                return
            for object_id in remaining:
                self._remove_world_object(object_id)
            time.sleep(0.05)
        raise GraspFailure(
            "COLLISION_FAILED",
            "planning-scene reset retained transient object(s): {}".format(
                ", ".join(sorted(remaining))
            ),
        )

    def _attach_moveit_object(self):
        collision = CollisionObject()
        collision.header.frame_id = self.tcp_link
        collision.id = self.moveit_object_id
        collision.pose.orientation.w = 1.0

        detonator = SolidPrimitive(
            type=SolidPrimitive.BOX, dimensions=list(self.detonator_size)
        )
        detonator_pose = Pose()
        # At top-down grasp, TCP +Z points downward.  Since the contact band is
        # above the detonator centre, the object centre lies +bias along TCP Z.
        detonator_pose.position.z = self.grasp_height_bias
        detonator_pose.orientation.w = 1.0

        disc = SolidPrimitive(
            type=SolidPrimitive.CYLINDER,
            dimensions=[self.disc_height, self.disc_radius],
        )
        disc_pose = Pose()
        disc_pose.position.z = (
            self.grasp_height_bias + self.detonator_center_z
            - self.disc_height * 0.5
        )
        disc_pose.orientation.w = 1.0
        collision.primitives = [detonator, disc]
        collision.primitive_poses = [detonator_pose, disc_pose]
        collision.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = self.tcp_link
        attached.object = collision
        attached.touch_links = list(self.touch_links)
        attached.weight = 0.30
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        if not self._apply_scene(scene):
            return False
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            request = GetPlanningSceneRequest()
            request.components.components = (
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            )
            response = self.get_scene_service(request)
            ids = [entry.object.id for entry in
                   response.scene.robot_state.attached_collision_objects]
            if self.moveit_object_id in ids:
                return True
            time.sleep(0.05)
        return False

    def _detach_moveit_object(self):
        collision = CollisionObject()
        collision.id = self.moveit_object_id
        collision.operation = CollisionObject.REMOVE
        attached = AttachedCollisionObject()
        attached.link_name = self.tcp_link
        attached.object = collision
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        return self._apply_scene(scene)

    def _verify_nonempty_physical_grasp(self):
        deadline = time.monotonic() + self.grasp_event_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._safety_check()
            with self.lock:
                attached = self.gazebo_attached
            if attached:
                return
            time.sleep(0.05)
        actual = self._gripper_actual_position()
        if actual is not None and abs(actual - self.gripper_closed) <= self.empty_tolerance:
            raise GraspFailure(
                "EMPTY_GRASP", "fingers reached empty-close target without a grasp event"
            )
        raise GraspFailure(
            "GAZEBO_ATTACH_FAILED", "opposing-contact grasp event was not established"
        )

    def _release_forced_lock(self, enforce_safety):
        """Release only a lock explicitly acknowledged by the plugin."""
        with self.lock:
            forced = bool(self.forced_lock_active)
            object_name = str(self.forced_lock_object)
            attached = bool(self.gazebo_attached)
            attached_object = str(self.gazebo_attached_object)
        if not forced:
            return
        if not object_name:
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "forced lock is active but its collision identity is empty",
            )
        if attached and attached_object != object_name:
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "refusing to release mismatched object {} (forced {})".format(
                    attached_object, object_name
                ),
            )

        self._publish_status(
            "SIM_LOCK_RELEASE", "RUNNING", {"object": object_name}
        )
        deadline = time.monotonic() + self.simulated_lock_request_timeout
        next_publish = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if enforce_safety:
                self._safety_check()
            else:
                self._publish_zero()
            now = time.monotonic()
            if now >= next_publish:
                self.force_detach_pub.publish(String(data=object_name))
                next_publish = now + 0.15
            with self.lock:
                forced_now = bool(self.forced_lock_active)
                attached_now = bool(self.gazebo_attached)
            if not forced_now and not attached_now:
                self.metrics["simulated_lock_release_acknowledged"] = True
                self._publish_status(
                    "SIM_LOCK_RELEASED", "GAZEBO_DETACHED",
                    {"object": object_name},
                )
                return
            time.sleep(0.02)
        raise GraspFailure(
            "GAZEBO_ATTACH_FAILED",
            "forced lock release was not acknowledged for {} within {:.2f} s"
            .format(object_name, self.simulated_lock_request_timeout),
        )

    def _wait_gazebo_released(self, enforce_safety, settle_duration=None):
        """Require a stable physical detach before scene/model reset.

        gazebo_grasp_fix owns a Gazebo joint internally.  Deleting or moving a
        model while that joint still references it can crash gzserver, so a
        local boolean must never be force-cleared as a substitute for the
        plugin's actual `attached=false` event.
        """
        required_settle = (
            self.grasp_release_settle
            if settle_duration is None
            else max(0.0, float(settle_duration))
        )
        deadline = time.monotonic() + self.grasp_release_timeout
        stable_since = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if enforce_safety:
                self._safety_check()
            else:
                self._publish_zero()
            now = time.monotonic()
            with self.lock:
                attached = self.gazebo_attached
                last_event = self.last_grasp_event_wall
            if attached:
                stable_since = None
            else:
                if stable_since is None:
                    stable_since = now
                quiet_since = stable_since
                if last_event is not None:
                    quiet_since = max(quiet_since, last_event)
                if now - quiet_since >= required_settle:
                    return
            time.sleep(0.02)
        raise GraspFailure(
            "GAZEBO_ATTACH_FAILED",
            "gazebo_grasp_fix did not confirm a stable release",
        )

    def _restore_forced_lock_identity(self, object_name):
        """Rollback a failed virtual-transport delete without moving the jaw."""
        if not object_name or not self._gazebo_model_exists(
                object_name.split("::", 1)[0]):
            raise GraspFailure(
                "TRANSPORT_FAILED",
                "cannot restore transport lock because its Gazebo model is absent",
            )
        with self.lock:
            self.pending_forced_lock_object = object_name
        self._publish_status(
            "VIRTUAL_TRANSPORT_ROLLBACK_LOCK", "RUNNING", {"object": object_name}
        )
        deadline = time.monotonic() + self.simulated_lock_request_timeout
        next_publish = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            # Do not call _safety_check here: the release guard correctly treats
            # any re-attachment as an error during normal forward progress, but
            # re-attachment is exactly the intended rollback operation.
            self._publish_zero()
            now = time.monotonic()
            if now >= next_publish:
                self.force_attach_pub.publish(String(data=object_name))
                next_publish = now + 0.15
            with self.lock:
                restored = bool(
                    self.forced_lock_active
                    and self.forced_lock_object == object_name
                    and self.gazebo_attached
                    and self.gazebo_attached_object == object_name
                )
            if restored:
                with self.lock:
                    self.pending_forced_lock_object = ""
                self._publish_status(
                    "VIRTUAL_TRANSPORT_ROLLBACK_LOCKED",
                    "GAZEBO_ATTACHED",
                    {"object": object_name},
                )
                return
            time.sleep(0.02)
        with self.lock:
            if self.pending_forced_lock_object == object_name:
                self.pending_forced_lock_object = ""
        raise GraspFailure(
            "TRANSPORT_FAILED",
            "failed to restore fixed transport lock for {} after delete rollback"
            .format(object_name),
        )

    def _gazebo_model_exists(self, model_name):
        if not model_name:
            return False
        with self.lock:
            states = self.model_states
            return states is not None and model_name in states.name

    def _wait_gazebo_model_presence(
        self, model_name, expected, timeout=None, enforce_safety=True
    ):
        deadline = time.monotonic() + (
            self.virtual_model_service_timeout if timeout is None else float(timeout)
        )
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._gazebo_model_exists(model_name) == bool(expected):
                return True
            if enforce_safety:
                self._safety_check()
            else:
                self._publish_zero()
            time.sleep(0.02)
        return self._gazebo_model_exists(model_name) == bool(expected)

    def _delete_gazebo_model(self, model_name, enforce_safety=True):
        if not model_name:
            raise GraspFailure("TRANSPORT_FAILED", "carried Gazebo model name is empty")
        if not self._gazebo_model_exists(model_name):
            return
        try:
            response = self.delete_model_service(model_name)
        except rospy.ServiceException as exc:
            raise GraspFailure(
                "TRANSPORT_FAILED",
                "Gazebo delete_model failed for {}: {}".format(model_name, exc),
            )
        if not response.success and self._gazebo_model_exists(model_name):
            raise GraspFailure(
                "TRANSPORT_FAILED",
                "Gazebo refused to delete {}: {}".format(
                    model_name, response.status_message
                ),
            )
        if not self._wait_gazebo_model_presence(
            model_name, False, enforce_safety=enforce_safety
        ):
            raise GraspFailure(
                "TRANSPORT_FAILED",
                "Gazebo model {} remained after delete_model".format(model_name),
            )

    def _spawn_gazebo_model_origin(self, model_name, origin_pose, enforce_safety=True):
        if not model_name:
            raise GraspFailure("PLACE_FAILED", "virtual carried model name is empty")
        if self._gazebo_model_exists(model_name):
            raise GraspFailure(
                "PLACE_FAILED",
                "refusing to spawn duplicate Gazebo model {}".format(model_name),
            )
        try:
            response = self.spawn_model_service(
                model_name,
                self.landmine_sdf_xml,
                "",
                copy.deepcopy(origin_pose),
                self.gazebo_spawn_reference_frame,
            )
        except rospy.ServiceException as exc:
            raise GraspFailure(
                "PLACE_FAILED",
                "Gazebo spawn_sdf_model failed for {}: {}".format(model_name, exc),
            )
        if not response.success:
            raise GraspFailure(
                "PLACE_FAILED",
                "Gazebo refused to spawn {}: {}".format(
                    model_name, response.status_message
                ),
            )
        if not self._wait_gazebo_model_presence(
            model_name, True, enforce_safety=enforce_safety
        ):
            raise GraspFailure(
                "PLACE_FAILED",
                "spawned Gazebo model {} did not appear in model_states".format(
                    model_name
                ),
            )

    def _virtualize_carried_model(self):
        """Remove the verified source entity before physical payload lift.

        The visible gripper remains closed for the complete loaded journey.
        Only the internal Gazebo fixed joint is released, under an explicit
        plugin quarantine, immediately before the source model is deleted.
        The exact model identity and original model-origin pose are retained
        for PLACE/reset; no Gazebo pose is used to generate a grasp target.
        """
        if not self.virtual_transport_enabled:
            return False
        with self.lock:
            model_name = str(self.attached_model_name)
            attached = bool(self.gazebo_attached)
            forced = bool(self.forced_lock_active)
            locked_object = str(
                self.forced_lock_object or self.gazebo_attached_object
            )
        source_pose = self._model_pose(validation_only=True)
        if (not model_name or source_pose is None or not attached or not forced
                or not locked_object
                or locked_object.split("::", 1)[0] != model_name):
            raise GraspFailure(
                "TRANSPORT_FAILED",
                "cannot virtualize transport without matching physical and forced locks",
            )

        self.releasing = True
        release_requested = False
        model_deleted = False
        try:
            self._publish_status(
                "VIRTUAL_TRANSPORT_SEALED",
                "RUNNING",
                {"model_name": model_name, "gripper_command": "HOLD_CLOSED"},
            )
            release_requested = True
            self._release_forced_lock(enforce_safety=True)
            self._wait_gazebo_released(
                enforce_safety=True,
                settle_duration=self.virtual_release_settle,
            )
            self.release_motion_guard = True
            self._publish_status(
                "VIRTUAL_TRANSPORT_DELETE", "RUNNING", {"model_name": model_name}
            )
            self._delete_gazebo_model(model_name, enforce_safety=True)
            model_deleted = True
            # Establish the virtual identity immediately after the physical
            # entity disappears.  Later planning-scene cleanup may fail, but it
            # must never lose the fact that the mine is still logically held.
            with self.lock:
                self.virtual_transport_active = True
                self.virtual_model_name = model_name
                self.virtual_source_pose = copy.deepcopy(source_pose)
                self.virtual_model_present = False
                self.debug_poses = {}
            if not self._detach_moveit_object():
                raise GraspFailure(
                    "TRANSPORT_FAILED",
                    "MoveIt detach failed after virtual transport was sealed",
                )
            self._remove_world_object(self.moveit_object_id)
            self._remove_world_object("mine_ground_guard")
            self._verify_transient_scene_cleared()
            # Grasp target markers are latched; clear them in the same PICK
            # transaction so RViz cannot continue drawing the removed source.
            self._publish_debug_markers()
            self.metrics["virtual_transport_used"] = True
            self.metrics["virtual_transport_model"] = model_name
            self._set_retained(True)
            self._publish_status(
                "VIRTUAL_TRANSPORT_LOCKED",
                "TRANSPORT_SEALED",
                {
                    "model_name": model_name,
                    "gazebo_model_present": False,
                    "gripper_command": "HOLD_CLOSED",
                    "ready_to_drive": False,
                },
            )
            return True
        except Exception:
            # If delete_model failed while the source still exists, restore the
            # same verified fixed joint with the still-closed jaws.  ForceAttach
            # also clears the plugin's explicit-detach quarantine.
            if (release_requested and not model_deleted
                    and self._gazebo_model_exists(model_name)):
                self.release_motion_guard = False
                self._restore_forced_lock_identity(locked_object)
            raise
        finally:
            self.release_motion_guard = False
            self.releasing = False

    def _restore_virtual_model_at_drop(self, drop_pose):
        with self.lock:
            active = bool(self.virtual_transport_active)
            model_name = str(self.virtual_model_name)
            model_present = bool(self.virtual_model_present)
        if not active:
            raise GraspFailure("PLACE_FAILED", "virtual transport transaction is inactive")
        if model_present or self._gazebo_model_exists(model_name):
            raise GraspFailure(
                "PLACE_FAILED",
                "virtual model {} already exists before PLACE restore".format(model_name),
            )
        expected = self._target_in_frame(
            drop_pose, self.gazebo_validation_frame, "TF_FAILED"
        )
        origin = copy.deepcopy(expected.pose)
        origin.position.z -= self.detonator_center_z
        self._publish_status(
            "PLACE_RESTORE_MODEL", "RUNNING", {"model_name": model_name}
        )
        self._spawn_gazebo_model_origin(model_name, origin, enforce_safety=True)
        with self.lock:
            self.virtual_model_present = True
        self.metrics["virtual_place_model_restored"] = True

    def _clear_virtual_transport(self):
        with self.lock:
            self.virtual_transport_active = False
            self.virtual_model_name = ""
            self.virtual_source_pose = None
            self.virtual_model_present = False

    def _rollback_virtual_place_spawn(self):
        """Best-effort rollback keeps a failed PLACE retryable and stationary."""
        with self.lock:
            active = bool(self.virtual_transport_active)
            model_name = str(self.virtual_model_name)
        if not active or not model_name or not self._gazebo_model_exists(model_name):
            return
        try:
            self._delete_gazebo_model(model_name, enforce_safety=False)
            with self.lock:
                self.virtual_model_present = False
        except Exception as exc:
            rospy.logerr(
                "[MineGrasp] could not roll back failed virtual PLACE for %s: %s",
                model_name,
                exc,
            )

    def _gripper_actual_position(self):
        with self.lock:
            state = self.gripper_state
        if state is None or not state.actual.positions:
            return None
        return float(state.actual.positions[0])

    def _record_perception_target(self, label, pose):
        """Store the already-computed visual target for post-run diagnostics."""
        self.metrics["{}_target_x".format(label)] = float(pose.pose.position.x)
        self.metrics["{}_target_y".format(label)] = float(pose.pose.position.y)
        self.metrics["{}_target_z".format(label)] = float(pose.pose.position.z)
        gravity = self._target_in_gravity(pose)
        self.metrics["{}_target_{}_x".format(label, self.gravity_frame)] = float(
            gravity.pose.position.x
        )
        self.metrics["{}_target_{}_y".format(label, self.gravity_frame)] = float(
            gravity.pose.position.y
        )
        self.metrics["{}_target_{}_z".format(label, self.gravity_frame)] = float(
            gravity.pose.position.z
        )
        top = copy.deepcopy(gravity)
        top.pose.position.z += 0.030
        top.header.stamp = rospy.Time.now()
        with self.lock:
            self.debug_poses["target_center"] = copy.deepcopy(gravity)
            self.debug_poses["target_top"] = top
        self._publish_debug_markers()

    def _select_simulated_lock_candidate(self):
        """Identify one task mine without feeding truth into target generation.

        This method is called only after FINAL_APPROACH and the strict measured
        TCP gate.  It may name the collision for a pose-preserving fixed joint;
        it never returns a pose used by IK, Cartesian motion, or perception.
        """
        with self.lock:
            states = copy.deepcopy(self.model_states)
            prior = copy.deepcopy(self.active_source_prior)
            mine_id = self.active_source_mine_id
        if prior is None or not prior.header.frame_id:
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "simulated lock requires the frozen mission mine prior",
            )
        if states is None or not states.name:
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED", "Gazebo model states are unavailable"
            )
        try:
            if prior.header.frame_id == self.gazebo_validation_frame:
                prior_world = prior
            else:
                # The frozen prior is a map location, not a moving sensor
                # sample.  Use the latest frame transform only for identity
                # association after motion is already complete.
                prior.header.stamp = rospy.Time(0)
                prior_world = self.tf_buffer.transform(
                    prior, self.gazebo_validation_frame, rospy.Duration(0.5)
                )
            tcp = self.tf_buffer.lookup_transform(
                self.gazebo_validation_frame, self.tcp_link,
                rospy.Time(0), rospy.Duration(0.5),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException) as exc:
            raise GraspFailure(
                "TF_FAILED", "simulated-lock association TF failed: {}".format(exc)
            )

        tcp_x = float(tcp.transform.translation.x)
        tcp_y = float(tcp.transform.translation.y)
        tcp_z = float(tcp.transform.translation.z)
        prior_x = float(prior_world.pose.position.x)
        prior_y = float(prior_world.pose.position.y)
        expected_vertical = self.detonator_center_z + self.grasp_height_bias
        prefix = self.mine_model_prefix.rstrip("_")
        evaluated = []
        passing = []
        for model_name, model_pose in zip(states.name, states.pose):
            if not (model_name == prefix or model_name.startswith(prefix + "_")):
                continue
            source_xy = math.hypot(
                float(model_pose.position.x) - prior_x,
                float(model_pose.position.y) - prior_y,
            )
            tcp_xy = math.hypot(
                float(model_pose.position.x) - tcp_x,
                float(model_pose.position.y) - tcp_y,
            )
            vertical_delta = (
                tcp_z - float(model_pose.position.z) - expected_vertical
            )
            vertical_error = abs(vertical_delta)
            entry = {
                "model_name": str(model_name),
                "source_xy_error_m": float(source_xy),
                "tcp_xy_error_m": float(tcp_xy),
                "vertical_error_m": float(vertical_error),
                "vertical_error_signed_m": float(vertical_delta),
                "model_x": float(model_pose.position.x),
                "model_y": float(model_pose.position.y),
                "model_z": float(model_pose.position.z),
            }
            entry["within_gates"] = bool(
                source_xy <= self.simulated_lock_source_xy
                and tcp_xy <= self.simulated_lock_tcp_xy
                and vertical_error <= self.simulated_lock_vertical
            )
            entry["score"] = float(
                tcp_xy + 0.25 * source_xy + vertical_error
            )
            evaluated.append(entry)
            if entry["within_gates"]:
                passing.append(entry)

        self.metrics["simulated_lock_association"] = {
            "mine_id": None if mine_id is None else int(mine_id),
            "validation_frame": self.gazebo_validation_frame,
            "expected_tcp_above_model_m": float(expected_vertical),
            "tcp_xy_limit_m": float(self.simulated_lock_tcp_xy),
            "source_xy_limit_m": float(self.simulated_lock_source_xy),
            "vertical_limit_m": float(self.simulated_lock_vertical),
            "evaluated": copy.deepcopy(evaluated),
        }
        if not passing:
            nearest = min(
                evaluated, key=lambda item: item["score"]
            ) if evaluated else None
            raise GraspFailure(
                "GAZEBO_ATTACH_FAILED",
                "no landmine model passed task/TCP/height association gates; "
                "nearest={}".format(nearest),
            )
        passing.sort(key=lambda item: item["score"])
        if len(passing) > 1:
            first, second = passing[0], passing[1]
            separation = math.hypot(
                first["model_x"] - second["model_x"],
                first["model_y"] - second["model_y"],
            )
            if separation < self.simulated_lock_min_separation:
                raise GraspFailure(
                    "GAZEBO_ATTACH_FAILED",
                    "ambiguous lock candidates {} and {} are only {:.3f} m "
                    "apart (required {:.3f} m)".format(
                        first["model_name"], second["model_name"], separation,
                        self.simulated_lock_min_separation,
                    ),
                )
        selected = copy.deepcopy(passing[0])
        selected["collision_name"] = "{}::{}".format(
            selected["model_name"], self.simulated_lock_collision_suffix
        )
        return selected

    def _record_validation_snapshot(self, label):
        """Record truth displacement without exposing it to grasp generation.

        Only displacement relative to the first snapshot is useful here: it
        reveals whether pregrasp, approach, or finger closing physically pushed
        the mine.  Missing Gazebo state never changes execution behaviour.
        """
        pose = self._model_pose(validation_only=True)
        if pose is None:
            return
        position = np.asarray([
            pose.position.x,
            pose.position.y,
            pose.position.z,
        ], dtype=np.float64)
        if self.validation_model_baseline is None:
            self.validation_model_baseline = position.copy()
        delta = position - self.validation_model_baseline
        roll, pitch = self._sample_attitude()
        snapshot = {
            "model_x": float(position[0]),
            "model_y": float(position[1]),
            "model_z": float(position[2]),
            "displacement_x": float(delta[0]),
            "displacement_y": float(delta[1]),
            "displacement_z": float(delta[2]),
            "displacement_norm": float(np.linalg.norm(delta)),
            "gripper_joint": self._gripper_actual_position(),
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
        }
        try:
            tcp = self.tf_buffer.lookup_transform(
                self.gazebo_validation_frame, self.tcp_link,
                rospy.Time(0), rospy.Duration(0.25),
            )
            snapshot.update({
                "tcp_frame": self.gazebo_validation_frame,
                "tcp_x": float(tcp.transform.translation.x),
                "tcp_y": float(tcp.transform.translation.y),
                "tcp_z": float(tcp.transform.translation.z),
            })
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException):
            # Diagnostics must never alter grasp execution.
            pass
        self.metrics.setdefault("validation_snapshots", {})[label] = snapshot
        rospy.loginfo(
            "[MineGraspValidation] %s mine displacement=(%.4f, %.4f, %.4f) m",
            label, delta[0], delta[1], delta[2],
        )

    def _model_pose(self, validation_only=False):
        # This is deliberately never called by pose generation or IK.  It is
        # permitted only for post-contact success scoring.
        if not validation_only:
            raise RuntimeError("Gazebo model pose is validation-only")
        with self.lock:
            states = copy.deepcopy(self.model_states)
            model_name = self.attached_model_name or self.mine_model
        if states is None or model_name not in states.name:
            return None
        return states.pose[states.name.index(model_name)]

    def _verify_lift(self, initial_pose):
        final_pose = self._model_pose(validation_only=True)
        with self.lock:
            attached = self.gazebo_attached
        if not attached:
            self.metrics["dropped"] = True
            raise GraspFailure("OBJECT_DROPPED", "Gazebo grasp event detached during lift")
        if initial_pose is None or final_pose is None:
            raise GraspFailure("LIFT_FAILED", "Gazebo validation pose unavailable")
        rise = final_pose.position.z - initial_pose.position.z
        self.metrics["measured_lift_height"] = float(rise)
        if rise < self.minimum_lift_height:
            raise GraspFailure(
                "LIFT_FAILED", "mine rose only {:.3f} m".format(rise)
            )

    def _relative_model_tcp(self):
        model = self._model_pose(validation_only=True)
        if model is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.gazebo_validation_frame, self.tcp_link,
                rospy.Time(0), rospy.Duration(0.5)
            )
        except Exception:
            return None
        tcp_pose = Pose()
        tcp_pose.position.x = transform.transform.translation.x
        tcp_pose.position.y = transform.transform.translation.y
        tcp_pose.position.z = transform.transform.translation.z
        tcp_pose.orientation = transform.transform.rotation
        return np.linalg.inv(_pose_matrix(tcp_pose)).dot(_pose_matrix(model))

    def _verify_hold(self):
        reference_pose = self._model_pose(validation_only=True)
        reference_relative = self._relative_model_tcp()
        if reference_pose is None or reference_relative is None:
            raise GraspFailure("LIFT_FAILED", "hold validation pose unavailable")
        minimum_z = reference_pose.position.z
        start = time.monotonic()
        while not rospy.is_shutdown() and time.monotonic() - start < self.hold_duration:
            self._safety_check()
            with self.lock:
                attached = self.gazebo_attached
            current_pose = self._model_pose(validation_only=True)
            current_relative = self._relative_model_tcp()
            if not attached or current_pose is None or current_relative is None:
                self.metrics["dropped"] = True
                raise GraspFailure("OBJECT_DROPPED", "object detached during 3 s hold")
            if current_pose.position.z < minimum_z - 0.02:
                self.metrics["dropped"] = True
                raise GraspFailure("OBJECT_DROPPED", "object lost lift height")
            delta = np.linalg.inv(reference_relative).dot(current_relative)
            translation = float(np.linalg.norm(delta[0:3, 3]))
            rotation = _rotation_angle(delta)
            self.metrics["relative_translation_max"] = max(
                self.metrics.get("relative_translation_max", 0.0), translation
            )
            self.metrics["relative_rotation_max_deg"] = max(
                self.metrics.get("relative_rotation_max_deg", 0.0),
                math.degrees(rotation),
            )
            if translation > self.relative_translation_tolerance:
                self.metrics["dropped"] = True
                raise GraspFailure("OBJECT_DROPPED", "mine slipped relative to TCP")
            if rotation > self.relative_rotation_tolerance:
                self.metrics["dropped"] = True
                raise GraspFailure("OBJECT_DROPPED", "mine rotated relative to TCP")
            self._verify_attached_scene()
            time.sleep(0.05)

    def _verify_transport_lock(self, duration):
        deadline = time.monotonic() + max(0.0, duration)
        with self.lock:
            reference = (
                None if self.transport_relative_reference is None
                else np.array(self.transport_relative_reference, copy=True)
            )
        if reference is None:
            reference = self._relative_model_tcp()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._safety_check()
            with self.lock:
                attached = self.gazebo_attached
                forced = self.forced_lock_active
                forced_object = str(self.forced_lock_object)
                attached_object = str(self.gazebo_attached_object)
                retained = self.retained
            if not retained:
                raise GraspFailure(
                    "OBJECT_DROPPED",
                    "transport watchdog rejected the physical lock",
                )
            if not attached:
                self._set_retained(False)
                raise GraspFailure(
                    "OBJECT_DROPPED", "gazebo_grasp_fix unlocked during transport"
                )

            # See `_retention_watchdog`: once the plugin has acknowledged its
            # kinematic lock, its state and collision identity are the lock
            # proof.  Sampling model_states and TF during arm/base motion is
            # not a synchronous measurement and must not manufacture a drop.
            if forced:
                if (not forced_object
                        or not attached_object
                        or forced_object != attached_object):
                    self._set_retained(False)
                    raise GraspFailure(
                        "OBJECT_DROPPED",
                        "plugin transport lock object identity changed "
                        "(forced={}, attached={})".format(
                            forced_object, attached_object,
                        ),
                    )
                time.sleep(0.05)
                continue

            self._verify_attached_scene()
            actual = self._gripper_actual_position()
            if actual is None:
                raise GraspFailure(
                    "GRIPPER_FAILED",
                    "gripper state lost during transport",
                )
            if (self.secure_gripper_position is not None
                    and abs(actual - self.secure_gripper_position)
                    > self.transport_gripper_drift):
                raise GraspFailure(
                    "GRIPPER_FAILED",
                    "jaw drift exceeded transport tolerance",
                )
            current = self._relative_model_tcp()
            if reference is not None and current is not None:
                delta = np.linalg.inv(reference).dot(current)
                if float(np.linalg.norm(delta[0:3, 3])) > self.relative_translation_tolerance:
                    raise GraspFailure("OBJECT_DROPPED", "mine slipped in transport lock")
                if _rotation_angle(delta) > self.relative_rotation_tolerance:
                    raise GraspFailure("OBJECT_DROPPED", "mine rotated in transport lock")
            time.sleep(0.05)

    def _verify_attached_scene(self):
        request = GetPlanningSceneRequest()
        request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        response = self.get_scene_service(request)
        ids = [entry.object.id for entry in
               response.scene.robot_state.attached_collision_objects]
        if self.moveit_object_id not in ids:
            raise GraspFailure("MOVEIT_ATTACH_FAILED", "attached object disappeared")

    def _cancel_all_motion(self):
        for client in (getattr(self, "arm_client", None),
                       getattr(self, "gripper_client", None),
                       getattr(self, "move_group_client", None),
                       getattr(self, "execute_client", None)):
            if client is not None:
                client.cancel_all_goals()
        self._publish_zero()

    def _best_effort_slow_retreat(self):
        roll, pitch = self._sample_attitude()
        if abs(roll) >= self.max_roll:
            rospy.logerr("base reached retreat roll limit; keeping arm stopped")
            return False
        if abs(pitch) >= self.max_pitch:
            rospy.logerr("base reached retreat pitch limit; keeping arm stopped")
            return False
        try:
            # Return toward the previously validated low-overhang look posture,
            # never toward the failed pregrasp.  If the body has reached either
            # configured attitude limit, motion remains cancelled.
            return bool(self._command_arm(
                self.look_joints, 20.0, enforce_safety=False,
                stage="RECOVERY_TO_LOOK",
            ))
        except Exception as exc:
            rospy.logerr("slow safety retreat failed: %s", exc)
            return False


def main():
    rospy.init_node("mine_grasp_executor")
    MineGraspExecutor()
    rospy.spin()


if __name__ == "__main__":
    main()
