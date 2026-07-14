#!/usr/bin/env python3
"""Dispatch a UGV to UAV-confirmed landmines and hand them to an arm action.

Safety invariants:
  * A confirmed mine remains active until PICK, loaded return, and PLACE all
    return explicit, physically verified success.
  * The base is locked while paused, at standoff, or while the arm is active.
  * A timed-out/failed arm attempt never clears the mine; if a physical grasp
    already exists, the base remains locked for manual recovery.
"""

import copy
import json
import math
import os
import tempfile
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

import actionlib
import rospy
import tf2_ros
import yaml
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty, Header, String, UInt32MultiArray
from std_srvs.srv import Empty as EmptyService
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

from mobile_manipulator.msg import (
    MineGraspAction,
    MineGraspGoal,
    MineGraspResult,
    MineMission,
    MineMissionEntry,
)
from uav_truth_tracker.msg import MineMap, MineMapEntry


DEFAULT_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR = 0.35
MAX_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR = 0.35


STATE_NAMES = {
    MineMissionEntry.PENDING: "PENDING",
    MineMissionEntry.WAITING_FOR_MAP: "WAITING_FOR_MAP",
    MineMissionEntry.NAVIGATING: "NAVIGATING",
    MineMissionEntry.AT_STANDOFF: "AT_STANDOFF",
    MineMissionEntry.WAITING_ARM: "WAITING_ARM",
    MineMissionEntry.CLEARED: "CLEARED",
    MineMissionEntry.DEFERRED: "DEFERRED",
    MineMissionEntry.UNREACHABLE: "UNREACHABLE",
    MineMissionEntry.MANUAL_REQUIRED: "MANUAL_REQUIRED",
    MineMissionEntry.CARRYING: "CARRYING",
    MineMissionEntry.RETURNING_HOME: "RETURNING_HOME",
    MineMissionEntry.AT_DROPOFF: "AT_DROPOFF",
    MineMissionEntry.PLACING: "PLACING",
}

GOAL_STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING",
    GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING",
    GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED",
    GoalStatus.LOST: "LOST",
}


def goal_status_name(state: int) -> str:
    return GOAL_STATUS_NAMES.get(state, str(state))

TERMINAL_STATES = {
    MineMissionEntry.CLEARED,
    MineMissionEntry.UNREACHABLE,
    MineMissionEntry.MANUAL_REQUIRED,
}


def base_safety_failure_state(paused: bool) -> int:
    """A live failure blocks automatic candidates unless the user paused."""
    return MineMissionEntry.PENDING if paused else MineMissionEntry.MANUAL_REQUIRED


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def path_length(path: Path) -> float:
    poses = path.poses
    if len(poses) < 2:
        return 0.0
    return sum(
        math.hypot(
            b.pose.position.x - a.pose.position.x,
            b.pose.position.y - a.pose.position.y,
        )
        for a, b in zip(poses[:-1], poses[1:])
    )


def base_motion_window_span(
    samples: Sequence[Tuple[float, float, float, float]],
    minimum_duration: float,
    minimum_samples: int = 3,
) -> Optional[Tuple[float, float, float, int]]:
    """Measure worst planar/yaw motion in the latest complete time window.

    Samples are ``(monotonic_time, x, y, yaw)`` in one continuous odometry
    frame.  Pairwise spans are
    used rather than first-to-last displacement so that a short oscillation
    which returns to its starting pose cannot be mistaken for a stopped base.
    Yaw differences are normalized across the +/-pi wrap.
    """
    if minimum_duration <= 0.0:
        raise ValueError("minimum_duration must be positive")
    minimum_samples = max(2, int(minimum_samples))
    if len(samples) < minimum_samples:
        return None
    if any(
        not all(math.isfinite(value) for value in sample)
        for sample in samples
    ):
        raise ValueError("pose samples must be finite")
    if any(b[0] <= a[0] for a, b in zip(samples[:-1], samples[1:])):
        raise ValueError("pose sample times must be strictly increasing")

    cutoff = samples[-1][0] - minimum_duration
    start_index = None
    for index, sample in enumerate(samples):
        if sample[0] <= cutoff:
            start_index = index
        else:
            break
    if start_index is None:
        return None
    window = samples[start_index:]
    duration = window[-1][0] - window[0][0]
    if duration + 1.0e-9 < minimum_duration or len(window) < minimum_samples:
        return None

    translation_span = 0.0
    yaw_span = 0.0
    for index, first in enumerate(window[:-1]):
        for second in window[index + 1:]:
            translation_span = max(
                translation_span,
                math.hypot(second[1] - first[1], second[2] - first[2]),
            )
            yaw_span = max(
                yaw_span,
                abs(normalize_angle(second[3] - first[3])),
            )
    return duration, translation_span, yaw_span, len(window)


def tf_stamp_is_post_cancel_fresh(
    tf_stamp: float,
    ros_now: float,
    cancel_ros_stamp: float,
    maximum_age: float,
) -> Tuple[bool, str]:
    """Pure timestamp gate for a post-cancel sensor sample."""
    values = (tf_stamp, ros_now, cancel_ros_stamp, maximum_age)
    if not all(math.isfinite(value) for value in values):
        return False, "non-finite sample/clock timestamp"
    if tf_stamp <= 0.0 or maximum_age <= 0.0:
        return False, "invalid sample timestamp or maximum age"
    if cancel_ros_stamp > 0.0 and tf_stamp <= cancel_ros_stamp:
        return False, "sample does not postdate navigation cancel"
    if ros_now > 0.0:
        age = ros_now - tf_stamp
        if age < -0.05:
            return False, "sample is in the future by {:.3f} s".format(-age)
        if age > maximum_age:
            return False, "sample is stale by {:.3f} s".format(age)
    return True, ""


def bounded_command(
    error: float, gain: float, maximum: float, minimum: float
) -> float:
    value = max(-maximum, min(maximum, gain * error))
    if abs(value) < minimum:
        value = math.copysign(minimum, error)
    return value


def empty_pick_retreat_step(
    distance: float,
    yaw_error: float,
    target_distance: float,
    distance_tolerance: float,
    yaw_tolerance: float,
    max_linear: float,
    max_angular: float,
    max_heading_error: float = math.radians(6.0),
) -> Tuple[str, float, float]:
    """Pure control decision for a straight, reverse-only mine retreat."""
    values = (
        distance,
        yaw_error,
        target_distance,
        distance_tolerance,
        yaw_tolerance,
        max_linear,
        max_angular,
        max_heading_error,
    )
    if not all(math.isfinite(value) for value in values):
        return "invalid", 0.0, 0.0
    distance_error = distance - target_distance
    if distance_error > distance_tolerance:
        # Never drive forward toward the mine to correct an outward overshoot.
        return "overshot", 0.0, 0.0
    if abs(yaw_error) > max_heading_error:
        # At the 0.67 m inner pose an in-place turn is not part of this bounded
        # recovery.  Stop and retain the safety lock instead.
        return "heading_rejected", 0.0, 0.0
    distance_ok = abs(distance_error) <= distance_tolerance
    yaw_ok = abs(yaw_error) <= yaw_tolerance
    if distance_ok and yaw_ok:
        return "reached", 0.0, 0.0

    angular = 0.0
    if not yaw_ok:
        angular = bounded_command(yaw_error, 1.2, max_angular, 0.04)
    linear = 0.0
    if distance < target_distance - distance_tolerance:
        linear = bounded_command(distance_error, 0.8, max_linear, 0.035)
        # This recovery is reverse-only by construction.
        linear = min(0.0, linear)
    return "moving", linear, angular


def centered_lateral_index(within_row: int) -> int:
    """Return the depot order 0, +1, -1, +2, -2, ... for a row."""
    if within_row <= 0:
        return 0
    magnitude = (within_row + 1) // 2
    return magnitude if within_row % 2 == 1 else -magnitude


def make_approach_pose(
    mine_pose: PoseStamped, angle: float, distance: float, frame_id: str
) -> PoseStamped:
    """Place the base around the mine and point its +X axis at the mine."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    # The make_plan call stamps both start and goal together. Keeping this
    # helper independent of a running ROS clock also makes its geometry testable.
    pose.header.stamp = mine_pose.header.stamp
    pose.pose.position.x = mine_pose.pose.position.x + distance * math.cos(angle)
    pose.pose.position.y = mine_pose.pose.position.y + distance * math.sin(angle)
    pose.pose.position.z = 0.0
    yaw = normalize_angle(angle + math.pi)
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
    pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
    return pose


@dataclass
class Task:
    mine_id: int
    mine_pose: PoseStamped
    confidence: float
    observation_count: int
    state: int = MineMissionEntry.PENDING
    approach_pose: Optional[PoseStamped] = None
    navigation_attempts: int = 0
    grasp_attempts: int = 0
    disposal_slot: int = -1
    dropoff_pose: Optional[PoseStamped] = None
    drop_pose: Optional[PoseStamped] = None
    retry_round: int = 0
    detail: str = "confirmed by UAV"
    updated_at: Optional[rospy.Time] = None
    waiting_since_wall: Optional[float] = None
    eligible_after_wall: float = 0.0
    latest_mine_pose: Optional[PoseStamped] = None
    map_revision: int = 0
    frozen_mine_pose: Optional[PoseStamped] = None
    frozen_map_revision: int = 0
    excluded_approach_directions: Set[int] = field(default_factory=set)

    def touch(self) -> None:
        self.updated_at = rospy.Time.now()


@dataclass
class ArmAttemptResult:
    success: bool
    detail: str
    code: str = ""
    recovered_to_look: bool = False
    retained: bool = False
    action_outcome: int = -1


class FineHandoffFailure(RuntimeError):
    """A locked navigation-to-fine handoff could not prove base safety."""


RECOVERABLE_OBSERVATION_FAILURES = frozenset({
    "OBSERVATION_INVALIDATED",
    "TARGET_FRAME_MOVED",
    "TARGET_OUT_OF_WORKSPACE",
})


def decode_executor_report(message: str) -> dict:
    """Extract the executor JSON report even when an action prefix surrounds it."""
    text = str(message or "")
    candidates = [text]
    opening = text.find("{")
    if opening >= 0:
        candidates.append(text[opening:])
    for candidate in candidates:
        try:
            value = json.loads(candidate)
        except (TypeError, ValueError):
            continue
        if isinstance(value, dict):
            return value
    return {}


def normalized_executor_failure_code(report: dict) -> str:
    """Disambiguate the legacy overloaded ``BASE_UNSTABLE`` result.

    Older grasp executors used ``BASE_UNSTABLE`` both for a true chassis
    safety fault (including excessive roll/pitch or failure to stop) and for a
    much narrower event: a frozen wrist observation becoming invalid after a
    measured, threshold-exceeding base-frame translation.  The latter still
    aborts the grasp, but an empty arm that subsequently verifies LOOK is safe
    to retreat and defer; it must not globally inhibit every later mine.

    Only structured reports containing both the legacy detail phrase and the
    recorded limit violation are translated.  Bare/ambiguous
    ``BASE_UNSTABLE`` results remain safety faults.
    """
    if not isinstance(report, dict):
        return ""
    code = str(report.get("failure_reason") or "")
    if code != "BASE_UNSTABLE":
        return code
    detail = str(report.get("failure_detail") or "")
    if "after the frozen wrist observation" not in detail:
        return code
    checks = report.get("observation_base_drift_checks")
    if not isinstance(checks, list):
        return code
    for check in checks:
        if not isinstance(check, dict):
            continue
        try:
            translation = float(check["translation_m"])
            translation_limit = float(check["translation_limit_m"])
            yaw = abs(float(check["yaw_error_rad"]))
            yaw_limit = math.radians(float(check["yaw_limit_deg"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
        if not all(math.isfinite(value) for value in (
                translation, translation_limit, yaw, yaw_limit)):
            continue
        if translation > translation_limit or yaw > yaw_limit:
            return "OBSERVATION_INVALIDATED"
    return code


class MineMissionManager:
    def __init__(self) -> None:
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.ground_frame = rospy.get_param("~ground_frame", "base_footprint")
        self.mine_map_topic = rospy.get_param(
            "~mine_map_topic", "/mine_detection/map"
        )
        self.terrain_map_topic = rospy.get_param("~terrain_map_topic", "/terrain_map")
        self.move_base_name = rospy.get_param("~move_base_action", "/move_base")
        self.make_plan_name = rospy.get_param(
            "~make_plan_service", "/move_base/make_plan"
        )
        self.clear_costmaps_name = rospy.get_param(
            "~clear_costmaps_service", "/move_base/clear_costmaps"
        )
        self.grasp_action_name = rospy.get_param("~grasp_action", "/mine_grasp")
        self.survey_status_topic = rospy.get_param(
            "~survey_status_topic", "/mine_survey/status"
        )
        self.require_survey_complete = bool(
            rospy.get_param("~require_survey_complete", False)
        )
        self.wait_for_survey_before_dispatch = bool(
            rospy.get_param(
                "~wait_for_survey_before_dispatch",
                self.require_survey_complete,
            )
        )

        # ``approach_distance`` is the final camera/manipulation standoff.  The
        # Husky base_link is 0.1812 m behind ur5_base_link.  The 0.67 m stop
        # produced a measured 0.527 m arm-base reach and the chassis crept from
        # 0.43 to 5.06 deg pitch during jaw closure.  A later M004 run measured
        # 0.519 m at the old 0.64 m stop because of the bounded map-to-wrist
        # correction, just 9 mm outside the verified 0.510 m stability region.
        # Stopping at 0.62 m absorbs that correction without widening the arm
        # overhang gate or weakening collision/attitude protection.
        self.approach_distance = float(rospy.get_param("~approach_distance", 0.62))
        # move_base stops on an outer collision-safe ring.  Once position is
        # reached it is cancelled and the dedicated low-speed controller turns
        # toward the mine and advances radially to ``approach_distance``.
        self.navigation_approach_distance = max(
            self.approach_distance,
            float(rospy.get_param("~navigation_approach_distance", 0.84)),
        )
        self.candidate_count = max(4, int(rospy.get_param("~candidate_count", 16)))
        self.candidate_limit = max(1, int(rospy.get_param("~candidate_limit", 3)))
        self.plan_tolerance = float(rospy.get_param("~plan_tolerance", 0.12))
        self.plan_endpoint_tolerance = float(
            rospy.get_param("~plan_endpoint_tolerance", 0.35)
        )
        self.other_mine_clearance = float(
            rospy.get_param("~other_mine_clearance", 0.75)
        )
        self.arrival_distance_tolerance = float(
            rospy.get_param("~arrival_distance_tolerance", 0.020)
        )
        self.arrival_yaw_tolerance = math.radians(
            float(rospy.get_param("~arrival_yaw_tolerance_deg", 3.0))
        )
        self.coarse_arrival_distance_tolerance = min(
            max(
                float(rospy.get_param(
                    "~coarse_arrival_distance_tolerance", 0.07
                )),
                self.arrival_distance_tolerance,
            ),
            0.15,
        )
        self.coarse_arrival_yaw_tolerance = math.radians(
            min(
                max(
                    float(rospy.get_param(
                        "~coarse_arrival_yaw_tolerance_deg", 180.0
                    )),
                    math.degrees(self.arrival_yaw_tolerance),
                ),
                180.0,
            )
        )
        # move_base is used for obstacle-aware long travel.  At the last few
        # centimetres it can declare success inside its own tolerances while the
        # wrist camera is already looking past the mine.  A bounded, low-speed
        # controller therefore finishes only the radial/yaw parking error after
        # the navigation goal has been cancelled.
        self.fine_alignment_timeout = max(
            2.0, float(rospy.get_param("~fine_alignment_timeout", 60.0))
        )
        self.fine_alignment_hold = max(
            0.1, float(rospy.get_param("~fine_alignment_hold", 0.6))
        )
        self.fine_alignment_max_linear = min(
            max(float(rospy.get_param("~fine_alignment_max_linear", 0.12)), 0.03),
            0.15,
        )
        self.fine_alignment_max_angular = min(
            max(float(rospy.get_param("~fine_alignment_max_angular", 0.25)), 0.08),
            0.35,
        )
        self.fine_alignment_max_initial_distance_error = min(
            max(
                float(rospy.get_param(
                    "~fine_alignment_max_initial_distance_error",
                    DEFAULT_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR,
                )),
                0.05,
            ),
            MAX_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR,
        )
        # Long-range move_base owns obstacle avoidance only.  At the depot it
        # hands off on position before it can circle while chasing final yaw.
        # The legacy fine-alignment bound remains for configuration
        # compatibility; disposal now regenerates PLACE at the actual base pose.
        self.dropoff_fine_alignment_max_initial_distance_error = min(
            max(float(rospy.get_param(
                "~dropoff_fine_alignment_max_initial_distance_error", 0.45
            )), 0.10),
            0.60,
        )
        self.loaded_dropoff_navigation_handoff_distance = min(
            self.dropoff_fine_alignment_max_initial_distance_error,
            max(float(rospy.get_param(
                "~loaded_dropoff_navigation_handoff_distance", 0.45
            )), 0.10),
        )
        # Disposal is an area transaction, not another grasp-quality parking
        # problem.  Once obstacle-aware navigation has entered this radius and
        # the chassis is stopped, PLACE is regenerated relative to the actual
        # base pose.  The arm therefore sees the same reachable 0.65 m geometry
        # even when the vehicle arrives with arbitrary heading.
        self.dropoff_relaxed_acceptance_radius = min(
            max(float(rospy.get_param(
                "~dropoff_relaxed_acceptance_radius", 0.75
            )), self.loaded_dropoff_navigation_handoff_distance),
            0.80,
        )
        self.drop_now_service_name = rospy.get_param(
            "~drop_now_service", "/mine_grasp/drop_now"
        )
        self.fine_cmd_topic = rospy.get_param(
            "~fine_alignment_cmd_vel_topic", "/mine_mission/fine_cmd_vel"
        )
        self.base_odom_topic = rospy.get_param(
            "~base_odom_topic", "/husky_velocity_controller/odom"
        )
        # A move_base cancel is asynchronous.  Keep the high-priority mux lock
        # asserted until fresh base-controller odometry proves that residual
        # DWA motion has stopped; only then may the bounded controller take over.
        self.fine_handoff_timeout = min(
            max(float(rospy.get_param(
                "~fine_alignment_handoff_timeout", 2.0
            )), 1.0),
            5.0,
        )
        self.fine_handoff_window = min(
            max(float(rospy.get_param(
                "~fine_alignment_handoff_window", 0.5
            )), 0.25),
            self.fine_handoff_timeout - 0.1,
        )
        self.fine_handoff_translation_tolerance = min(
            max(float(rospy.get_param(
                "~fine_alignment_handoff_translation_tolerance", 0.005
            )), 0.001),
            0.010,
        )
        self.fine_handoff_yaw_tolerance = math.radians(min(
            max(float(rospy.get_param(
                "~fine_alignment_handoff_yaw_tolerance_deg", 0.5
            )), 0.1),
            1.0,
        ))
        self.fine_handoff_min_samples = min(
            max(int(rospy.get_param(
                "~fine_alignment_handoff_min_samples", 5
            )), 3),
            20,
        )
        self.fine_handoff_odom_max_age = min(
            max(float(rospy.get_param(
                "~fine_handoff_odom_max_age", 0.75
            )), 0.20),
            2.0,
        )
        self.fine_handoff_linear_velocity_tolerance = min(
            max(float(rospy.get_param(
                "~fine_handoff_linear_velocity_tolerance", 0.03
            )), 0.005),
            0.10,
        )
        self.fine_handoff_angular_velocity_tolerance = min(
            max(float(rospy.get_param(
                "~fine_handoff_angular_velocity_tolerance", 0.05
            )), 0.01),
            0.20,
        )
        self.empty_pick_retreat_timeout = min(
            max(float(rospy.get_param(
                "~empty_pick_retreat_timeout", 8.0
            )), 2.0),
            15.0,
        )
        # After release the new disposal hazard is directly in front of the
        # chassis.  Back out on the already verified clear arrival corridor so
        # the next move_base plan does not start inside its inflation field.
        self.post_place_retreat_distance = min(
            max(float(rospy.get_param(
                "~post_place_retreat_distance", 0.80
            )), 0.30),
            1.50,
        )
        self.post_place_retreat_speed = min(
            max(float(rospy.get_param(
                "~post_place_retreat_speed", 0.55
            )), 0.10),
            0.80,
        )
        self.post_place_retreat_timeout = min(
            max(float(rospy.get_param(
                "~post_place_retreat_timeout", 10.0
            )), 3.0),
            20.0,
        )
        # A loaded return may recover from a transient local-costmap failure,
        # but every retry still requires the physical lock and attitude gates.
        # This is a bounded total-attempt count, never an endless retry loop.
        self.loaded_return_attempts = min(
            max(int(rospy.get_param("~loaded_return_attempts", 3)), 1),
            5,
        )
        self.loaded_return_retry_delay = min(
            max(float(rospy.get_param(
                "~loaded_return_retry_delay", 0.5
            )), 0.0),
            3.0,
        )
        self.loaded_return_costmap_settle = min(
            max(float(rospy.get_param(
                "~loaded_return_costmap_settle", 0.30
            )), 0.0),
            2.0,
        )
        self.loaded_departure_turn_timeout = min(
            max(float(rospy.get_param(
                "~loaded_departure_turn_timeout", 20.0
            )), 5.0),
            30.0,
        )
        self.loaded_departure_turn_tolerance = math.radians(min(
            max(float(rospy.get_param(
                "~loaded_departure_turn_tolerance_deg", 6.0
            )), 2.0),
            12.0,
        ))
        self.loaded_departure_turn_hold = min(
            max(float(rospy.get_param(
                "~loaded_departure_turn_hold", 0.30
            )), 0.10),
            1.0,
        )
        self.navigation_timeout = float(
            rospy.get_param("~navigation_timeout", 600.0)
        )
        self.navigation_stall_timeout = max(
            5.0, float(rospy.get_param("~navigation_stall_timeout", 45.0))
        )
        # A plugin-locked mine is a no-reaction kinematic follower, so loaded
        # driving no longer needs a long "wait and see" window.  If DWA has
        # made no measurable progress, cancel/clear/replan promptly instead of
        # leaving the UGV apparently frozen for the ordinary 45 s watchdog.
        self.loaded_navigation_stall_timeout = min(
            self.navigation_stall_timeout,
            max(5.0, float(rospy.get_param(
                "~loaded_navigation_stall_timeout", 8.0
            ))),
        )
        self.navigation_progress_epsilon = max(
            0.01, float(rospy.get_param("~navigation_progress_epsilon", 0.05))
        )
        self.arrival_hold_time = max(
            0.0, float(rospy.get_param("~arrival_hold_time", 0.30))
        )
        self.arm_server_wait = float(rospy.get_param("~arm_server_wait", 5.0))
        # Wall-clock guard for the complete PICK Action: wrist refinement,
        # slow close, lift and collision-checked loaded transport pose.
        self.arm_timeout = float(rospy.get_param("~arm_timeout", 360.0))
        self.place_timeout = float(rospy.get_param("~place_timeout", 90.0))
        self.action_result_grace = min(
            max(float(rospy.get_param("~action_result_grace", 1.0)), 0.1),
            2.0,
        )
        self.map_wait_timeout = float(rospy.get_param("~map_wait_timeout", 120.0))
        self.map_settle_time = float(rospy.get_param("~map_settle_time", 3.0))
        self.retry_delay = float(rospy.get_param("~retry_delay", 10.0))
        self.retry_limit = max(0, int(rospy.get_param("~retry_limit", 1)))
        self.nominal_diameter = float(rospy.get_param("~nominal_diameter", 0.136))
        self.drop_standoff = float(rospy.get_param("~drop_standoff", 0.65))
        self.drop_center_height = float(
            rospy.get_param("~drop_center_height", 0.055)
        )
        self.depot_forward = float(rospy.get_param("~depot_forward", 1.8))
        self.depot_lateral = float(rospy.get_param("~depot_lateral", 0.0))
        self.depot_columns = max(1, int(rospy.get_param("~depot_columns", 5)))
        self.depot_column_spacing = float(
            rospy.get_param("~depot_column_spacing", 1.20)
        )
        self.depot_row_spacing = float(
            rospy.get_param("~depot_row_spacing", 2.00)
        )
        self.disposed_exclusion_radius = float(
            rospy.get_param("~disposed_exclusion_radius", 0.35)
        )
        self.hazard_transition_settle = float(
            rospy.get_param("~hazard_transition_settle", 0.25)
        )
        self.depot_mine_clearance = float(
            rospy.get_param("~depot_mine_clearance", 0.75)
        )
        self.depot_parking_clearance = float(
            rospy.get_param("~depot_parking_clearance", 0.75)
        )
        self.carried_detection_exclusion_radius = float(
            rospy.get_param("~carried_detection_exclusion_radius", 1.0)
        )
        # Fusion owns the primary identity, but the mission must never dispatch
        # two physical tasks for short-lived confirmed fragments of the same
        # mine. This must match fusion's identity quarantine: the latest
        # integrated run produced 0.30--0.88 m depth/TF fragments around one
        # physical mine, while distinct field mines remain more than 2 m apart.
        self.task_alias_radius = min(
            max(float(rospy.get_param("~task_alias_radius", 1.00)), 0.10),
            1.00,
        )
        self.task_alias_height_tolerance = min(
            max(
                float(rospy.get_param("~task_alias_height_tolerance", 0.20)),
                0.05,
            ),
            0.50,
        )
        self.carry_speed = float(rospy.get_param("~carry_max_vel_x", 3.40))
        self.carry_turn_speed = float(
            rospy.get_param("~carry_max_vel_theta", 1.20)
        )
        self.carry_accel = float(rospy.get_param("~carry_acc_lim_x", 3.00))
        self.carry_turn_accel = float(
            rospy.get_param("~carry_acc_lim_theta", 1.80)
        )
        # Do not offer DWA near-zero "moving" trajectories while carrying.
        # The restored 3.0 m/s^2 acceleration makes 0.25 m/s reachable in the
        # first 10 Hz control window, so this floor starts decisively without
        # creating the empty dynamic window that the former low acceleration
        # required us to work around with a 0.01 m/s crawl sample.
        self.carry_min_vel_trans = min(
            self.carry_speed,
            max(0.0, float(rospy.get_param("~carry_min_vel_trans", 0.25))),
        )
        self.carry_min_vel_theta = min(
            self.carry_turn_speed,
            max(0.0, float(rospy.get_param("~carry_min_vel_theta", 0.15))),
        )
        self.carry_max_roll = math.radians(
            float(rospy.get_param("~carry_max_roll_deg", 5.0))
        )
        self.carry_max_pitch = math.radians(
            float(rospy.get_param("~carry_max_pitch_deg", 5.0))
        )
        self.require_carry_speed_limit = bool(
            rospy.get_param("~require_carry_speed_limit", True)
        )
        self.dwa_reconfigure_name = rospy.get_param(
            "~dwa_reconfigure_name", "/move_base/DWAPlannerROS"
        )
        self.resume_from_file = bool(rospy.get_param("~resume_from_file", False))
        self.resume_match_radius = float(rospy.get_param("~resume_match_radius", 0.5))
        self.persistence_file = os.path.expanduser(
            rospy.get_param(
                "~persistence_file", "~/.ros/mine_mission_state.yaml"
            )
        )

        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.tasks: Dict[int, Task] = {}
        self.last_mine_map: Optional[MineMap] = None
        self.resume_records: List[dict] = []
        self.running = bool(rospy.get_param("~auto_start", True))
        self.paused = False
        self.map_ready = False
        self.map_ready_wall: Optional[float] = None
        self.current_mine_id: Optional[int] = None
        self.revision = 0
        self.epoch = 1
        self.shutdown = False
        self._plan_transport_error = False
        self.arm_preflight_blocked = False
        self.home_pose: Optional[PoseStamped] = None
        self.home_ground_z: Optional[float] = None
        self.disposed_poses: Dict[int, Pose] = {}
        # Detection IDs whose physical source entity has entered the verified
        # PICK transaction.  They remain suppressed after CLEARED so RViz and
        # the persistent detection outputs cannot keep showing a mine at its
        # old source coordinate.
        self.suppressed_detection_ids: Set[int] = set()
        self.next_disposal_slot = 0
        self.mine_retained = False
        self.survey_complete = not (
            self.require_survey_complete or self.wait_for_survey_before_dispatch
        )
        self.survey_wait_announced = False
        self.imu: Optional[Imu] = None
        self.base_odom: Optional[Odometry] = None
        self.carry_speed_active = False
        self.dwa_client = None
        self.normal_dwa_config = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.move_client = actionlib.SimpleActionClient(
            self.move_base_name, MoveBaseAction
        )
        self.grasp_client = actionlib.SimpleActionClient(
            self.grasp_action_name, MineGraspAction
        )
        self.make_plan = rospy.ServiceProxy(self.make_plan_name, GetPlan)
        self.clear_costmaps = rospy.ServiceProxy(
            self.clear_costmaps_name, EmptyService
        )
        self.drop_now = rospy.ServiceProxy(
            self.drop_now_service_name, Trigger
        )

        self.status_pub = rospy.Publisher(
            "/mine_mission/status", MineMission, queue_size=1, latch=True
        )
        self.current_pub = rospy.Publisher(
            "/mine_mission/current_task",
            MineMissionEntry,
            queue_size=1,
            latch=True,
        )
        self.markers_pub = rospy.Publisher(
            "/mine_mission/markers", MarkerArray, queue_size=1, latch=True
        )
        self.active_hazards_pub = rospy.Publisher(
            "/mine_hazard/active", PoseArray, queue_size=1, latch=True
        )
        self.suppressed_ids_pub = rospy.Publisher(
            "/mine_mission/suppressed_detection_ids",
            UInt32MultiArray,
            queue_size=1,
            latch=True,
        )
        self.base_lock_pub = rospy.Publisher(
            "/mine_mission/base_lock", Bool, queue_size=1, latch=True
        )
        self.hazard_reset_pub = rospy.Publisher(
            "/mine_hazard/reset", Empty, queue_size=1
        )
        self.disposed_pub = rospy.Publisher(
            "/mine_disposal/poses", PoseArray, queue_size=1, latch=True
        )
        self.carrying_pub = rospy.Publisher(
            "/mine_mission/carrying", Bool, queue_size=1, latch=True
        )
        self.all_cleared_pub = rospy.Publisher(
            "/mine_mission/all_known_cleared", Bool, queue_size=1, latch=True
        )
        self.fine_cmd_pub = rospy.Publisher(
            self.fine_cmd_topic, Twist, queue_size=1
        )

        rospy.Subscriber(
            self.mine_map_topic, MineMap, self._mine_map_cb, queue_size=5
        )
        rospy.Subscriber(
            self.terrain_map_topic, OccupancyGrid, self._terrain_map_cb, queue_size=1
        )
        rospy.Subscriber("/mine_grasp/retained", Bool,
                         self._retained_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~imu_topic", "/imu/data"), Imu,
                         self._imu_cb, queue_size=1)
        rospy.Subscriber(
            self.base_odom_topic, Odometry, self._base_odom_cb, queue_size=1
        )
        if self.require_survey_complete or self.wait_for_survey_before_dispatch:
            rospy.Subscriber(
                self.survey_status_topic, String, self._survey_status_cb,
                queue_size=1,
            )
        rospy.Service("/mine_mission/start", Trigger, self._start_cb)
        rospy.Service("/mine_mission/pause", Trigger, self._pause_cb)
        rospy.Service("/mine_mission/resume", Trigger, self._resume_cb)
        rospy.Service("/mine_mission/reset", Trigger, self._reset_cb)

        if self.resume_from_file:
            self._load_resume_records()

        self._set_base_lock(not self.running)
        self._publish_all(persist=False)
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[MineMission] input=%s nav_ring=%.2fm final_standoff=%.2fm candidates=%d "
            "nav_timeout=%.0fs arm=%s timeout=%.0fs retry=%d",
            self.mine_map_topic,
            self.navigation_approach_distance,
            self.approach_distance,
            self.candidate_count,
            self.navigation_timeout,
            self.grasp_action_name,
            self.arm_timeout,
            self.retry_limit,
        )

    # ------------------------------------------------------------------ inputs
    def _retained_cb(self, msg: Bool) -> None:
        with self.condition:
            self.mine_retained = bool(msg.data)
            self.condition.notify_all()

    def _imu_cb(self, msg: Imu) -> None:
        with self.lock:
            self.imu = msg

    def _base_odom_cb(self, msg: Odometry) -> None:
        with self.lock:
            self.base_odom = msg

    def _survey_status_cb(self, msg: String) -> None:
        if msg.data.strip().upper() != "COMPLETE":
            return
        with self.condition:
            changed = not self.survey_complete
            self.survey_complete = True
            self.survey_wait_announced = False
            for task in self.tasks.values():
                if (
                    task.state in (MineMissionEntry.PENDING, MineMissionEntry.DEFERRED)
                    and task.detail == "waiting for UAV survey COMPLETE"
                ):
                    task.detail = "UAV survey complete; ready for UGV dispatch"
                    task.touch()
            self.condition.notify_all()
        if changed:
            rospy.logwarn(
                "[MineMission] UAV coverage COMPLETE; UGV dispatch and final "
                "all-clear are now eligible"
            )
            self._publish_all()

    def _terrain_map_cb(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0:
            return
        changed = False
        with self.condition:
            if not self.map_ready:
                self.map_ready = True
                self.map_ready_wall = time.monotonic()
                changed = True
                for task in self.tasks.values():
                    if task.state == MineMissionEntry.WAITING_FOR_MAP:
                        task.state = MineMissionEntry.PENDING
                        task.detail = "terrain map ready"
                        task.waiting_since_wall = None
                        task.touch()
                self.condition.notify_all()
        if changed:
            rospy.loginfo(
                "[MineMission] terrain map ready: %dx%d @ %.3fm",
                msg.info.width,
                msg.info.height,
                msg.info.resolution,
            )
            self._publish_all()

    def _mine_map_cb(self, msg: MineMap) -> None:
        changed = False
        with self.condition:
            self.last_mine_map = copy.deepcopy(msg)
            changed = self._ingest_map_locked(msg)
            if changed:
                self.condition.notify_all()
        if changed:
            self._publish_all()

    def _task_alias_id_locked(self, mine: MineMapEntry) -> Optional[int]:
        """Return an existing physical task represented by a new map ID.

        UAV fusion IDs are diagnostic identities, not permission to create two
        UGV jobs within one mine footprint.  Exact IDs are handled normally by
        the caller; this guard only applies when a previously unseen ID lands
        inside the small spatial/height alias gate.
        """
        nearest_id = None
        nearest_distance = float("inf")
        for other_id, task in self.tasks.items():
            if other_id == mine.id:
                continue
            reference = task.frozen_mine_pose or task.mine_pose
            dx = mine.position.x - reference.pose.position.x
            dy = mine.position.y - reference.pose.position.y
            distance = math.hypot(dx, dy)
            if (
                distance <= self.task_alias_radius
                and abs(mine.position.z - reference.pose.position.z)
                <= self.task_alias_height_tolerance
                and distance < nearest_distance
            ):
                nearest_id = other_id
                nearest_distance = distance
        return nearest_id

    def _ingest_map_locked(self, msg: MineMap) -> bool:
        changed = False
        frame = msg.header.frame_id or self.map_frame
        carried_base = self._frame_pose(self.base_frame) if self.mine_retained else None
        for mine in msg.mines:
            if not mine.confirmed:
                continue
            if (carried_base is not None
                    and math.hypot(
                        mine.position.x - carried_base.pose.position.x,
                        mine.position.y - carried_base.pose.position.y,
                    ) <= self.carried_detection_exclusion_radius):
                # A UAV observation of the mine currently fixed to the wrist
                # must not become a second navigation task. The persistent map
                # will be ingested normally again after PLACE.
                continue
            if any(
                math.hypot(
                    mine.position.x - pose.position.x,
                    mine.position.y - pose.position.y,
                ) <= self.disposed_exclusion_radius
                for pose in self.disposed_poses.values()
            ):
                rospy.loginfo_throttle(
                    5.0,
                    "[MineMission] ignoring detection inside verified disposal depot",
                )
                continue
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = msg.header.stamp
            pose.pose.position = copy.deepcopy(mine.position)
            pose.pose.orientation.w = 1.0
            task = self.tasks.get(mine.id)
            if task is None:
                alias_id = self._task_alias_id_locked(mine)
                if alias_id is not None:
                    rospy.logwarn_throttle(
                        5.0,
                        "[MineMission] ignoring confirmed map ID M%03d as a "
                        "spatial alias of existing physical task M%03d",
                        mine.id,
                        alias_id,
                    )
                    continue
                task = Task(
                    mine_id=mine.id,
                    mine_pose=pose,
                    confidence=mine.confidence,
                    observation_count=mine.observation_count,
                    latest_mine_pose=copy.deepcopy(pose),
                    map_revision=int(msg.revision),
                )
                task.touch()
                self._apply_resume_record_locked(task)
                self.tasks[mine.id] = task
                changed = True
                rospy.logwarn(
                    "[MineMission] NEW confirmed mine M%03d at map=(%.2f, %.2f)",
                    mine.id,
                    mine.position.x,
                    mine.position.y,
                )
            else:
                moved = math.hypot(
                    task.mine_pose.pose.position.x - mine.position.x,
                    task.mine_pose.pose.position.y - mine.position.y,
                )
                active = (
                    task.mine_id == self.current_mine_id
                    or task.state in (
                        MineMissionEntry.NAVIGATING,
                        MineMissionEntry.AT_STANDOFF,
                        MineMissionEntry.WAITING_ARM,
                        MineMissionEntry.CARRYING,
                        MineMissionEntry.RETURNING_HOME,
                        MineMissionEntry.AT_DROPOFF,
                        MineMissionEntry.PLACING,
                    )
                )
                task.latest_mine_pose = copy.deepcopy(pose)
                task.map_revision = int(msg.revision)
                # Freeze the dispatched ID/revision/position. New UAV fusion
                # remains available for a later deferred attempt, but it can
                # never move the target under an active navigation or grasp.
                if not active:
                    task.mine_pose = copy.deepcopy(pose)
                task.confidence = max(task.confidence, mine.confidence)
                task.observation_count = max(
                    task.observation_count, mine.observation_count
                )
                if moved > 0.03 and task.state not in TERMINAL_STATES:
                    task.detail = (
                        "UAV update buffered for next dispatch"
                        if active else "UAV refined pending mine position"
                    )
                    task.touch()
                    changed = True
        return changed

    # -------------------------------------------------------------- ROS services
    def _start_cb(self, _req) -> TriggerResponse:
        with self.condition:
            self.running = True
            self.paused = False
            self.condition.notify_all()
        with self.lock:
            carrying = self._has_carrying_task_locked()
        self._set_base_lock(carrying)
        self._publish_all()
        return TriggerResponse(success=True, message="mine mission started")

    def _pause_cb(self, _req) -> TriggerResponse:
        with self.condition:
            self.paused = True
            self.condition.notify_all()
        self._cancel_navigation_goal_if_active()
        self.grasp_client.cancel_goal()
        self._set_base_lock(True)
        self._publish_all()
        return TriggerResponse(success=True, message="mine mission paused; base locked")

    def _resume_cb(self, _req) -> TriggerResponse:
        with self.condition:
            self.running = True
            self.paused = False
            self.condition.notify_all()
        with self.lock:
            carrying = self._has_carrying_task_locked()
        self._set_base_lock(carrying)
        self._publish_all()
        return TriggerResponse(success=True, message="mine mission resumed")

    def _has_carrying_task_locked(self) -> bool:
        return any(
            task.state in (
                MineMissionEntry.CARRYING,
                MineMissionEntry.RETURNING_HOME,
                MineMissionEntry.AT_DROPOFF,
                MineMissionEntry.PLACING,
            )
            for task in self.tasks.values()
        )

    def _reset_cb(self, _req) -> TriggerResponse:
        with self.lock:
            if self.mine_retained:
                return TriggerResponse(
                    success=False,
                    message="cannot reset while a mine is physically retained; place it or handle manually",
                )
        self._cancel_navigation_goal_if_active()
        self.grasp_client.cancel_goal()
        with self.condition:
            self.current_mine_id = None
            self.epoch += 1
            for task in self.tasks.values():
                task.state = MineMissionEntry.PENDING
                task.approach_pose = None
                task.navigation_attempts = 0
                task.grasp_attempts = 0
                task.dropoff_pose = None
                task.drop_pose = None
                task.disposal_slot = -1
                task.retry_round = 0
                task.eligible_after_wall = 0.0
                task.frozen_mine_pose = None
                task.frozen_map_revision = 0
                task.excluded_approach_directions.clear()
                if task.latest_mine_pose is not None:
                    task.mine_pose = copy.deepcopy(task.latest_mine_pose)
                task.detail = "mission reset; confirmed mine restored"
                task.touch()
            if self.last_mine_map is not None:
                self._ingest_map_locked(self.last_mine_map)
            self.condition.notify_all()
        self.hazard_reset_pub.publish(Empty())
        self._set_carry_speed(False)
        self._set_base_lock(self.paused or not self.running)
        self._publish_all()
        return TriggerResponse(
            success=True,
            message="mission reset; current confirmed mines restored as pending",
        )

    # ---------------------------------------------------------- task scheduling
    def _worker_loop(self) -> None:
        while not rospy.is_shutdown():
            with self.lock:
                capture_home = self.running and not self.paused and self.home_pose is None
            if capture_home and not self._capture_home_pose():
                time.sleep(0.2)
                continue

            should_publish = False
            delivery_id = None
            manual_block_id = None
            manual_block_detail = ""
            with self.condition:
                if self.shutdown:
                    return
                if not self.running or self.paused or not self.tasks:
                    self.condition.wait(timeout=0.5)
                    continue

                carrying = [
                    task.mine_id for task in self.tasks.values()
                    if task.state in (
                        MineMissionEntry.CARRYING,
                        MineMissionEntry.RETURNING_HOME,
                        MineMissionEntry.AT_DROPOFF,
                        MineMissionEntry.PLACING,
                    )
                ]
                if carrying:
                    delivery_id = carrying[0]
                    candidates = []
                elif any(
                    task.state == MineMissionEntry.MANUAL_REQUIRED
                    for task in self.tasks.values()
                ):
                    # Do not abandon a hazardous source after every local PICK
                    # retry has failed.  A manual/reset decision is required
                    # before the vehicle can drive toward another mine.
                    manual_task = next(
                        task for task in self.tasks.values()
                        if task.state == MineMissionEntry.MANUAL_REQUIRED
                    )
                    manual_block_id = manual_task.mine_id
                    manual_block_detail = manual_task.detail[:500]
                    candidates = []
                elif not self.map_ready:
                    now = time.monotonic()
                    for task in self.tasks.values():
                        if task.state in (
                            MineMissionEntry.PENDING,
                            MineMissionEntry.DEFERRED,
                        ):
                            task.state = MineMissionEntry.WAITING_FOR_MAP
                            task.detail = "waiting for /terrain_map"
                            task.waiting_since_wall = now
                            task.touch()
                            should_publish = True
                        elif (
                            task.state == MineMissionEntry.WAITING_FOR_MAP
                            and task.waiting_since_wall is not None
                            and now - task.waiting_since_wall > self.map_wait_timeout
                        ):
                            task.detail = "terrain map wait timed out; still retained"
                            task.touch()
                            task.waiting_since_wall = now
                            should_publish = True
                    self.condition.wait(timeout=0.5)
                    candidates = []
                else:
                    for task in self.tasks.values():
                        if task.state == MineMissionEntry.WAITING_FOR_MAP:
                            task.state = MineMissionEntry.PENDING
                            task.detail = "terrain map ready"
                            task.waiting_since_wall = None
                            task.touch()
                            should_publish = True
                    if (
                        self.wait_for_survey_before_dispatch
                        and not self.survey_complete
                    ):
                        candidates = []
                        if not self.survey_wait_announced:
                            for task in self.tasks.values():
                                if task.state in (
                                    MineMissionEntry.PENDING,
                                    MineMissionEntry.DEFERRED,
                                ):
                                    task.detail = "waiting for UAV survey COMPLETE"
                                    task.touch()
                            self.survey_wait_announced = True
                            should_publish = True
                            rospy.loginfo(
                                "[MineMission] retaining confirmed mines while UAV "
                                "finishes coverage; UGV dispatch is gated"
                            )
                    elif (
                        self.map_ready_wall is not None
                        and time.monotonic() - self.map_ready_wall
                        < self.map_settle_time
                    ):
                        candidates = []
                    else:
                        candidates = self._eligible_task_ids_locked()

            if should_publish:
                self._publish_all()
            if delivery_id is not None:
                self._complete_delivery(delivery_id)
                continue
            if manual_block_id is not None:
                self._set_base_lock(True)
                rospy.logerr_throttle(
                    10.0,
                    "[MineMission] M%03d requires recovery; base locked and "
                    "later mine tasks are inhibited; reason=%s",
                    manual_block_id,
                    manual_block_detail,
                )
                time.sleep(0.1)
                continue
            if not candidates:
                time.sleep(0.1)
                continue

            # Never drive toward a mine unless the manipulation Action is
            # actually connected.  Previously this check happened only after
            # arrival; a broken server therefore made the UGV skip the mine
            # and immediately drive to the next one.
            if not self._wait_for_action_server(self.grasp_client, 0.5):
                self._set_base_lock(True)
                if not self.arm_preflight_blocked:
                    self.arm_preflight_blocked = True
                    with self.lock:
                        for candidate_id in candidates:
                            task = self.tasks.get(candidate_id)
                            if task is not None:
                                task.detail = (
                                    "waiting for manipulation Action server before navigation"
                                )
                                task.touch()
                    self._publish_all()
                rospy.logerr_throttle(
                    3.0,
                    "[MineMission] /mine_grasp Action unavailable; base locked "
                    "and mine navigation is inhibited",
                )
                time.sleep(0.25)
                continue
            if self.arm_preflight_blocked:
                self.arm_preflight_blocked = False
                rospy.logwarn(
                    "[MineMission] manipulation Action connected; mine navigation enabled"
                )

            selection, unreachable = self._select_nearest_reachable(candidates)
            for mine_id in unreachable:
                self._navigation_round_failed(
                    mine_id, "no reachable {:.2f} m approach pose".format(
                        self.navigation_approach_distance
                    )
                )
            if selection is None:
                time.sleep(0.25)
                continue

            mine_id, approaches = selection
            self._execute_task(mine_id, approaches)

    def _eligible_task_ids_locked(self) -> List[int]:
        active = [
            task
            for task in self.tasks.values()
            if task.state in (MineMissionEntry.PENDING, MineMissionEntry.DEFERRED)
            and time.monotonic() >= task.eligible_after_wall
        ]
        if not active:
            return []
        # Finish every first-pass target before retrying a deferred one. A mine
        # discovered later by the still-flying UAV joins that first pass.
        first_pass = [task for task in active if task.retry_round == 0]
        pool = first_pass if first_pass else active
        return [task.mine_id for task in pool]

    def _capture_home_pose(self) -> bool:
        base = self._frame_pose(self.base_frame)
        ground = self._frame_pose(self.ground_frame)
        if base is None or ground is None:
            rospy.logwarn_throttle(
                2.0, "[MineMission] waiting to capture initial base/ground pose"
            )
            return False
        with self.lock:
            if self.home_pose is not None:
                return True
            self.home_pose = base
            self.home_ground_z = float(ground.pose.position.z)
        rospy.logwarn(
            "[MineMission] HOME captured at map=(%.2f, %.2f, yaw fixed); depot %.2f m ahead",
            base.pose.position.x, base.pose.position.y, self.depot_forward,
        )
        self._publish_all()
        return True

    def _assign_drop_slot_locked(self, task: Task) -> None:
        if task.drop_pose is not None and task.dropoff_pose is not None:
            return
        if self.home_pose is None or self.home_ground_z is None:
            raise RuntimeError("home pose has not been captured")
        q = self.home_pose.pose.orientation
        home_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        cos_yaw, sin_yaw = math.cos(home_yaw), math.sin(home_yaw)

        def local_to_map(x_value: float, y_value: float) -> Tuple[float, float]:
            return (
                self.home_pose.pose.position.x + cos_yaw * x_value - sin_yaw * y_value,
                self.home_pose.pose.position.y + sin_yaw * x_value + cos_yaw * y_value,
            )

        # Enumerate each row from its centre outwards: 0, +1, -1, +2, -2.
        # The first five mines stay on one open line; a later row's 0.65 m
        # parking pose therefore cannot land on a mine in the previous row.
        for _ in range(10000):
            slot = self.next_disposal_slot
            self.next_disposal_slot += 1
            row = slot // self.depot_columns
            within_row = slot % self.depot_columns
            lateral_index = centered_lateral_index(within_row)
            local_x = self.depot_forward + row * self.depot_row_spacing
            local_y = (
                self.depot_lateral
                + float(lateral_index) * self.depot_column_spacing
            )
            drop_x, drop_y = local_to_map(local_x, local_y)
            park_x, park_y = local_to_map(
                local_x - self.drop_standoff, local_y
            )
            source_conflict = any(
                other.mine_id != task.mine_id
                and other.state != MineMissionEntry.CLEARED
                and (
                    math.hypot(
                        drop_x - other.mine_pose.pose.position.x,
                        drop_y - other.mine_pose.pose.position.y,
                    ) < self.depot_mine_clearance
                    or math.hypot(
                        park_x - other.mine_pose.pose.position.x,
                        park_y - other.mine_pose.pose.position.y,
                    ) < self.depot_parking_clearance
                )
                for other in self.tasks.values()
            )
            disposed_conflict = any(
                math.hypot(drop_x - pose.position.x, drop_y - pose.position.y)
                < self.depot_mine_clearance
                or math.hypot(park_x - pose.position.x, park_y - pose.position.y)
                < self.depot_parking_clearance
                for pose in self.disposed_poses.values()
            )
            if not source_conflict and not disposed_conflict:
                break
            rospy.logwarn(
                "[MineMission] disposal slot %d conflicts with a source, "
                "disposed mine, or parking footprint; skipping",
                slot,
            )
        else:
            raise RuntimeError("no conflict-free slot in the first 10000 candidates")
        drop = PoseStamped()
        drop.header.frame_id = self.map_frame
        drop.pose.position.x = drop_x
        drop.pose.position.y = drop_y
        drop.pose.position.z = self.home_ground_z + self.drop_center_height
        drop.pose.orientation.w = 1.0
        parking = PoseStamped()
        parking.header.frame_id = self.map_frame
        parking.pose.position.x = park_x
        parking.pose.position.y = park_y
        parking.pose.position.z = 0.0
        orientation = quaternion_from_euler(0.0, 0.0, home_yaw)
        parking.pose.orientation.x, parking.pose.orientation.y = orientation[0], orientation[1]
        parking.pose.orientation.z, parking.pose.orientation.w = orientation[2], orientation[3]
        task.disposal_slot = slot
        task.drop_pose = drop
        task.dropoff_pose = parking

    def _select_nearest_reachable(
        self, task_ids: Sequence[int]
    ) -> Tuple[Optional[Tuple[int, List[Tuple[float, PoseStamped]]]], List[int]]:
        start = self._robot_pose()
        if start is None:
            rospy.logwarn_throttle(2.0, "[MineMission] waiting for map -> base_link TF")
            return None, []
        if not self._wait_for_plan_service(2.0):
            rospy.logwarn_throttle(2.0, "[MineMission] waiting for make_plan service")
            return None, []

        with self.lock:
            for mine_id in task_ids:
                task = self.tasks.get(mine_id)
                if task is not None and task.latest_mine_pose is not None:
                    task.mine_pose = copy.deepcopy(task.latest_mine_pose)
            snapshots = {
                mine_id: copy.deepcopy(self.tasks[mine_id].mine_pose)
                for mine_id in task_ids
                if mine_id in self.tasks
            }
            active_mines = {
                task.mine_id: copy.deepcopy(task.mine_pose)
                for task in self.tasks.values()
                if task.state != MineMissionEntry.CLEARED
            }
            excluded_directions = {
                mine_id: set(self.tasks[mine_id].excluded_approach_directions)
                for mine_id in task_ids if mine_id in self.tasks
            }

        best_for_task: Dict[int, List[Tuple[float, PoseStamped]]] = {}
        unreachable: List[int] = []
        self._plan_transport_error = False
        for mine_id, mine_pose in snapshots.items():
            approaches: List[Tuple[float, PoseStamped]] = []
            for index in range(self.candidate_count):
                if index in excluded_directions.get(mine_id, set()):
                    continue
                angle = 2.0 * math.pi * index / self.candidate_count
                goal = make_approach_pose(
                    mine_pose,
                    angle,
                    self.navigation_approach_distance,
                    self.map_frame,
                )
                if self._too_close_to_other_mine(goal, mine_id, active_mines):
                    continue
                plan = self._request_plan(start, goal)
                if plan is None:
                    continue
                approaches.append((path_length(plan), goal))
            approaches.sort(key=lambda item: item[0])
            if approaches:
                best_for_task[mine_id] = approaches[: self.candidate_limit]
            else:
                unreachable.append(mine_id)

        if self._plan_transport_error:
            rospy.logwarn_throttle(
                2.0, "[MineMission] make_plan transport error; deferring decisions"
            )
            return None, []
        if not best_for_task:
            return None, unreachable
        selected_id = min(best_for_task, key=lambda key: best_for_task[key][0][0])
        rospy.loginfo(
            "[MineMission] nearest reachable is M%03d (planned %.2fm, %d approaches)",
            selected_id,
            best_for_task[selected_id][0][0],
            len(best_for_task[selected_id]),
        )
        return (selected_id, best_for_task[selected_id]), unreachable

    def _too_close_to_other_mine(
        self,
        goal: PoseStamped,
        target_id: int,
        active_mines: Dict[int, PoseStamped],
    ) -> bool:
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        return any(
            other_id != target_id
            and math.hypot(
                gx - pose.pose.position.x, gy - pose.pose.position.y
            )
            < self.other_mine_clearance
            for other_id, pose in active_mines.items()
        )

    def _request_plan(
        self, start: PoseStamped, goal: PoseStamped
    ) -> Optional[Path]:
        try:
            start = copy.deepcopy(start)
            goal = copy.deepcopy(goal)
            start.header.stamp = rospy.Time.now()
            goal.header.stamp = start.header.stamp
            response = self.make_plan(
                start=start, goal=goal, tolerance=self.plan_tolerance
            )
        except (rospy.ServiceException, rospy.ROSException) as exc:
            self._plan_transport_error = True
            rospy.logwarn_throttle(2.0, "[MineMission] make_plan failed: %s", exc)
            return None
        if not response.plan.poses:
            return None
        end = response.plan.poses[-1].pose.position
        distance_to_goal = math.hypot(
            end.x - goal.pose.position.x, end.y - goal.pose.position.y
        )
        if distance_to_goal > self.plan_endpoint_tolerance:
            return None
        return response.plan

    def _approach_direction_index(
        self, mine_pose: PoseStamped, approach: PoseStamped
    ) -> int:
        angle = math.atan2(
            approach.pose.position.y - mine_pose.pose.position.y,
            approach.pose.position.x - mine_pose.pose.position.x,
        )
        return int(round(
            (angle % (2.0 * math.pi)) * self.candidate_count /
            (2.0 * math.pi)
        )) % self.candidate_count

    @staticmethod
    def _arm_failure_requires_manual(result: ArmAttemptResult) -> bool:
        """Classify failures that make any further base motion unsafe."""
        if (
            result.code in RECOVERABLE_OBSERVATION_FAILURES
            and result.recovered_to_look
            and not result.retained
        ):
            # The target reference was invalidated before descent.  The
            # executor has already proved an empty arm at LOOK, so the bounded
            # reverse-only outer-ring retreat is the safe continuation even
            # when a legacy action server labelled the event UNSAFE.
            return False
        return bool(
            result.retained
            or not result.recovered_to_look
            or result.action_outcome == MineGraspResult.UNSAFE
            or result.code in {
                "BASE_UNSTABLE",
                "OBJECT_DROPPED",
                "TRANSPORT_FAILED",
                "TRANSPORT_LOCK_LOST",
            }
        )

    def _execute_task(
        self, mine_id: int, approaches: Sequence[Tuple[float, PoseStamped]]
    ) -> None:
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None or task.state in TERMINAL_STATES:
                return
            task.frozen_mine_pose = copy.deepcopy(task.mine_pose)
            task.frozen_map_revision = int(task.map_revision)
            task.detail = "dispatched from frozen map revision {}".format(
                task.frozen_map_revision
            )
            task.touch()
        for number, (length, approach) in enumerate(approaches, start=1):
            with self.lock:
                task = self.tasks.get(mine_id)
                if task is None or task.state in TERMINAL_STATES:
                    return
                self.current_mine_id = mine_id
                task.state = MineMissionEntry.NAVIGATING
                task.approach_pose = copy.deepcopy(approach)
                direction = self._approach_direction_index(
                    task.frozen_mine_pose or task.mine_pose, approach
                )
                task.excluded_approach_directions.add(direction)
                task.navigation_attempts += 1
                task.detail = (
                    "approach {}/{}, direction {}, planned {:.2f} m, frozen "
                    "map revision {}".format(
                        number, len(approaches), direction, length,
                        task.frozen_map_revision,
                    )
                )
                task.touch()
            self._set_base_lock(False)
            self._publish_all()

            outcome = self._navigate_to(approach, mine_id=mine_id)
            if outcome == "paused":
                with self.lock:
                    task = self.tasks.get(mine_id)
                    if task and task.state not in TERMINAL_STATES:
                        task.state = MineMissionEntry.PENDING
                        task.detail = "navigation cancelled by pause"
                        task.touch()
                    self.current_mine_id = None
                self._publish_all()
                return
            if outcome != "success":
                rospy.logwarn(
                    "[MineMission] M%03d approach %d failed: %s",
                    mine_id,
                    number,
                    outcome,
                )
                continue

            verified, detail = self._verify_arrival(mine_id)
            if not verified:
                rospy.loginfo(
                    "[MineMission] M%03d coarse arrival needs fine alignment: %s",
                    mine_id,
                    detail,
                )

            try:
                aligned, alignment_detail = self._fine_align_to_mine(mine_id)
            except FineHandoffFailure as exc:
                with self.lock:
                    paused = self.paused or not self.running
                    task = self.tasks.get(mine_id)
                    if task is not None and task.state not in TERMINAL_STATES:
                        task.state = base_safety_failure_state(paused)
                        task.detail = str(exc) + "; source hazard remains active"
                        task.touch()
                    self.current_mine_id = None
                self._set_base_lock(True)
                self._publish_all()
                return
            if not aligned:
                with self.lock:
                    paused = self.paused or not self.running
                    task = self.tasks.get(mine_id)
                    if paused and task is not None and task.state not in TERMINAL_STATES:
                        task.state = MineMissionEntry.PENDING
                        task.detail = alignment_detail + "; base remains locked"
                        task.touch()
                        self.current_mine_id = None
                if paused:
                    self._set_base_lock(True)
                    self._publish_all()
                    return
                rospy.logwarn(
                    "[MineMission] M%03d fine alignment failed: %s",
                    mine_id,
                    alignment_detail,
                )
                continue

            verified, detail = self._verify_arrival(mine_id)
            if not verified:
                rospy.logwarn(
                    "[MineMission] M%03d post-alignment geometry rejected: %s",
                    mine_id,
                    detail,
                )
                continue
            detail = detail + "; " + alignment_detail

            actual_approach = self._robot_pose()
            if actual_approach is None:
                rospy.logwarn(
                    "[MineMission] M%03d actual fine-aligned base pose unavailable",
                    mine_id,
                )
                continue

            self._set_base_lock(True)
            with self.lock:
                task = self.tasks.get(mine_id)
                if task is None:
                    return
                # The manipulation contract records where the chassis actually
                # stopped after precision alignment, never the outer 0.84 m
                # move_base ring goal.
                task.approach_pose = copy.deepcopy(actual_approach)
                task.state = MineMissionEntry.AT_STANDOFF
                task.detail = detail
                task.touch()
            self._publish_all()

            arm_result = self._run_arm_action(mine_id)
            with self.lock:
                paused = self.paused or not self.running
                retained_after_action = bool(
                    arm_result.retained or self.mine_retained
                )
            if paused:
                # The grasp-fix callback and action terminal result can arrive
                # in either order. Preserve a lock established in that narrow
                # cancellation window instead of relabelling it PENDING.
                if (
                    not retained_after_action
                    and self._wait_retained(True, self.action_result_grace)
                ):
                    retained_after_action = True
                with self.lock:
                    task = self.tasks.get(mine_id)
                    if task is not None and task.state not in TERMINAL_STATES:
                        if retained_after_action:
                            try:
                                self._assign_drop_slot_locked(task)
                            except RuntimeError as exc:
                                task.state = MineMissionEntry.MANUAL_REQUIRED
                                task.detail = (
                                    "PICK paused with physical lock but no safe "
                                    "disposal slot: {}"
                                ).format(exc)
                                self.current_mine_id = None
                            else:
                                task.state = MineMissionEntry.CARRYING
                                task.detail = (
                                    "PICK paused after physical lock; carrying "
                                    "state preserved for resume"
                                )
                                self.current_mine_id = mine_id
                        elif arm_result.recovered_to_look:
                            task.state = MineMissionEntry.PENDING
                            task.detail = (
                                "PICK paused before physical lock; arm verified "
                                "at look and task remains pending"
                            )
                            self.current_mine_id = None
                        else:
                            task.state = MineMissionEntry.MANUAL_REQUIRED
                            task.detail = (
                                "PICK cancelled by pause without a physical "
                                "lock, but recovery to look was not verified; "
                                "base remains locked"
                            )
                            self.current_mine_id = None
                        task.touch()
                self._set_base_lock(True)
                self._publish_all()
                return

            if arm_result.success:
                slot_error = None
                with self.lock:
                    task = self.tasks.get(mine_id)
                    if task is None:
                        return
                    try:
                        self._assign_drop_slot_locked(task)
                    except RuntimeError as exc:
                        slot_error = str(exc)
                        task.state = MineMissionEntry.MANUAL_REQUIRED
                        task.detail = (
                            "PICK is physically locked but no safe disposal slot: "
                            + slot_error
                        )
                        self.current_mine_id = None
                    else:
                        task.state = MineMissionEntry.CARRYING
                        task.detail = (
                            arm_result.detail
                            + "; physically locked, awaiting return"
                        )
                    task.touch()
                if slot_error is not None:
                    self._set_base_lock(True)
                    self._publish_all()
                    return
                self._publish_all()
                self._complete_delivery(mine_id)
                return

            if self._arm_failure_requires_manual(arm_result):
                self._arm_round_failed(mine_id, arm_result.detail)
                self._set_base_lock(True)
                return

            # Empty grasp + verified recovery is safe to defer.  The selected
            # direction was already recorded as excluded, so a later attempt
            # must use a different parking candidate and cannot replay the
            # exact same MoveIt/FJT path immediately.
            retreat_ok, retreat_detail = self._retreat_after_empty_pick(
                mine_id, arm_result.recovered_to_look
            )
            if not retreat_ok:
                with self.lock:
                    task = self.tasks.get(mine_id)
                    paused = self.paused or not self.running
                    if task is not None and task.state not in TERMINAL_STATES:
                        task.state = base_safety_failure_state(paused)
                        task.detail = (
                            "{}: {}; empty-pick retreat failed: {}; source "
                            "hazard remains active; base remains locked"
                        ).format(
                            arm_result.code or "PICK_FAILED",
                            arm_result.detail,
                            retreat_detail,
                        )
                        task.touch()
                    self.current_mine_id = None
                self._set_base_lock(True)
                self._publish_all()
                return
            self._defer_empty_pick(
                mine_id,
                "{}: {}; {}".format(
                    arm_result.code or "PICK_FAILED",
                    arm_result.detail,
                    retreat_detail,
                ),
            )
            self._set_base_lock(False)
            return

        self._navigation_round_failed(mine_id, "all approach poses failed")
        self._set_base_lock(False)

    def _turn_loaded_toward(self, target: PoseStamped) -> Tuple[bool, str]:
        """Rotate in place after PICK, before handing the return to move_base."""
        self._cancel_navigation_goal_if_active()
        self._set_base_lock(False)
        started = time.monotonic()
        stable_since = None
        last_detail = "waiting for map->base pose"
        try:
            while (
                not rospy.is_shutdown()
                and time.monotonic() - started < self.loaded_departure_turn_timeout
            ):
                with self.lock:
                    if self.paused or not self.running:
                        return False, "loaded departure turn cancelled by pause"
                    retained = bool(self.mine_retained)
                if not retained:
                    return False, "OBJECT_DROPPED: lock lost during loaded departure turn"
                attitude_ok, attitude_detail = self._carry_attitude_ok()
                if not attitude_ok:
                    return False, attitude_detail
                robot = self._robot_pose()
                if robot is None:
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                q = robot.pose.orientation
                yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                dx = target.pose.position.x - robot.pose.position.x
                dy = target.pose.position.y - robot.pose.position.y
                if math.hypot(dx, dy) <= 0.05:
                    return True, "already at loaded return target"
                desired = math.atan2(dy, dx)
                error = normalize_angle(desired - yaw)
                last_detail = "departure yaw error {:.1f} deg".format(
                    math.degrees(error)
                )
                if abs(error) <= self.loaded_departure_turn_tolerance:
                    self.fine_cmd_pub.publish(Twist())
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= self.loaded_departure_turn_hold:
                        return True, last_detail + "; stable"
                else:
                    stable_since = None
                    command = Twist()
                    speed = min(
                        self.carry_turn_speed,
                        max(0.08, 0.8 * abs(error)),
                    )
                    command.angular.z = math.copysign(speed, error)
                    self.fine_cmd_pub.publish(command)
                time.sleep(0.05)
            return False, (
                "loaded departure turn timeout after {:.1f} s: {}"
            ).format(time.monotonic() - started, last_detail)
        finally:
            self.fine_cmd_pub.publish(Twist())
            self._set_base_lock(True)

    def _navigate_to(
        self,
        approach: PoseStamped,
        require_retained: bool = False,
        mine_id: Optional[int] = None,
    ) -> str:
        if not self._wait_for_action_server(self.move_client, 5.0):
            return "move_base server unavailable"
        prior_state = self.move_client.get_state()
        if prior_state in (
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
        ):
            # GoalStatus can become terminal just before SimpleActionClient has
            # consumed the final transition callback.  Sending the retry in
            # that window makes actionlib drop the old handle and emit "goal
            # handle that we're not tracking".  Drain the bounded terminal
            # transition before replacing the goal.
            self.move_client.wait_for_result(
                rospy.Duration(self.action_result_grace)
            )
        goal = MoveBaseGoal()
        goal.target_pose = copy.deepcopy(approach)
        goal.target_pose.header.stamp = rospy.Time.now()
        self.move_client.send_goal(goal)
        started = time.monotonic()
        deadline = started + self.navigation_timeout
        stall_timeout = (
            self.loaded_navigation_stall_timeout
            if require_retained
            else self.navigation_stall_timeout
        )
        last_progress = started
        best_distance = float("inf")
        best_yaw_error = float("inf")
        arrival_since = None
        last_arrival_detail = "coarse arrival not evaluated"
        while not rospy.is_shutdown():
            with self.lock:
                if self.paused or not self.running:
                    self._cancel_navigation_and_brake()
                    return "paused"
                retained = self.mine_retained
            if require_retained:
                if not retained:
                    self._cancel_navigation_and_brake()
                    return "OBJECT_DROPPED: physical grasp lock lost"
                attitude_ok, attitude_detail = self._carry_attitude_ok()
                if not attitude_ok:
                    self._cancel_navigation_and_brake()
                    return attitude_detail

            now = time.monotonic()
            if mine_id is None and require_retained:
                # PLACE is regenerated from the actual stopped base pose.
                # Requiring DWA to reach a precise depot yaw caused it to pass
                # the slot, rotate/overshoot, and appear stuck.  Entering this
                # small pre-cleared region is the complete navigation contract.
                dropoff_errors = self._pose_goal_errors(approach)
                if (dropoff_errors is not None
                        and dropoff_errors[0]
                        <= self.loaded_dropoff_navigation_handoff_distance):
                    self._cancel_navigation_and_brake()
                    rospy.loginfo(
                        "[MineMission] loaded return reached dropoff handoff: "
                        "goal_error=%.3f m, yaw_error=%.1f deg; switching "
                        "directly to relaxed disposal",
                        dropoff_errors[0],
                        math.degrees(dropoff_errors[1]),
                    )
                    return "success"
            if mine_id is not None:
                # Long-range navigation only has to enter the bounded fine-
                # alignment neighbourhood.  Requiring the DWA controller to
                # achieve the final 25 mm / 3 deg on rough terrain can make it
                # oscillate indefinitely; the dedicated low-speed controller
                # below owns those final centimetres.
                arrived, arrival_detail = self._verify_coarse_arrival(mine_id)
            else:
                arrived, arrival_detail = self._verify_pose_arrival(approach)
            last_arrival_detail = arrival_detail
            if arrived:
                if arrival_since is None:
                    arrival_since = now
                elif now - arrival_since >= self.arrival_hold_time:
                    self._cancel_navigation_and_brake()
                    rospy.loginfo(
                        "[MineMission] accepted stable geometric arrival before "
                        "move_base terminal state: %s",
                        arrival_detail,
                    )
                    return "success"
            else:
                arrival_since = None

            errors = self._pose_goal_errors(approach)
            if errors is not None:
                distance, yaw_error = errors
                progressed = False
                if distance <= best_distance - self.navigation_progress_epsilon:
                    best_distance = distance
                    progressed = True
                # Once near the parking pose, correct final heading also counts
                # as progress. Arbitrary rotate-recovery circles do not.
                if (
                    distance <= 0.35
                    and yaw_error <= best_yaw_error - math.radians(3.0)
                ):
                    best_yaw_error = yaw_error
                    progressed = True
                if progressed:
                    last_progress = now

            state = self.move_client.get_state()
            if state == GoalStatus.SUCCEEDED and mine_id is not None:
                # A move_base terminal flag is not a safe handoff by itself:
                # its XY tolerance can stop the footprint too close to the
                # mine for an in-place turn.  Require the measured outer-ring
                # geometry (including its short stability hold) before handing
                # control to the fine alignment controller.
                if not arrived:
                    self._cancel_navigation_and_brake()
                    return (
                        "move_base SUCCEEDED outside collision-safe handoff: "
                        + arrival_detail
                    )
                time.sleep(0.05)
                continue
            if state in (
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.PREEMPTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                if state != GoalStatus.LOST:
                    self.move_client.wait_for_result(
                        rospy.Duration(self.action_result_grace)
                    )
                self._set_base_lock(True)
                return (
                    "success"
                    if state == GoalStatus.SUCCEEDED
                    else goal_status_name(state)
                )
            if now - last_progress >= stall_timeout:
                self._cancel_navigation_and_brake()
                return (
                    "navigation stalled for {:.0f} s (best goal distance {:.2f} m; {})"
                ).format(
                    stall_timeout,
                    best_distance,
                    last_arrival_detail,
                )
            if now >= deadline:
                self._cancel_navigation_and_brake()
                return "navigation timeout"
            time.sleep(0.1)
        return "shutdown"

    def _cancel_navigation_goal_if_active(self) -> bool:
        """Cancel only a live move_base goal.

        actionlib logs a protocol error when cancel_goal() is sent after an
        ABORTED goal has already entered DONE.  Avoid that misleading error
        while preserving cancellation for PENDING/ACTIVE transitions.
        """
        state = self.move_client.get_state()
        if state in (
            GoalStatus.PREEMPTED,
            GoalStatus.SUCCEEDED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ):
            return False
        self.move_client.cancel_goal()
        return True

    def _cancel_navigation_and_brake(self) -> None:
        self._cancel_navigation_goal_if_active()
        # The lock input has priority in twist_mux and suppresses stale
        # recovery-rotation commands while the next state is being selected.
        self._set_base_lock(True)
        # Clear any fine-controller command which could otherwise become live
        # when the lock is released during a later handoff.
        self.fine_cmd_pub.publish(Twist())
        time.sleep(0.15)

    def _clear_navigation_costmaps(self) -> str:
        """Clear transient observations between bounded loaded-return retries.

        The custom mine layer repaints every active confirmed mine on the next
        update.  The one physically retained source is intentionally omitted
        while CARRYING/RETURNING/PLACING because it is no longer at that map
        coordinate.  Failure to reach the service is reported but does not
        discard an otherwise safe retry.
        """
        self._set_base_lock(True)
        self.fine_cmd_pub.publish(Twist())
        try:
            rospy.wait_for_service(self.clear_costmaps_name, timeout=1.0)
            self.clear_costmaps()
            if self.loaded_return_costmap_settle > 0.0:
                time.sleep(self.loaded_return_costmap_settle)
            return "costmaps cleared and active hazards repainted"
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return "costmap clear unavailable: {}".format(exc)

    def _wait_for_fine_alignment_handoff(
        self,
        label: str,
        require_retained: bool = False,
        reject_retained: bool = False,
    ) -> Tuple[bool, str]:
        """Cancel navigation and prove the locked chassis is stationary.

        The base lock remains asserted on every failure path.  Only a caller
        receiving ``True`` is allowed to release it and publish fine commands.
        Fresh base-controller odometry proves both real velocity and physical
        pose span.  The composed map->base transform is deliberately excluded
        because global-EKF corrections can move it by centimetres while the
        wheel controller correctly reports a stationary chassis.  The fine
        controller itself waits for map->base before publishing nonzero motion.
        """
        self._cancel_navigation_goal_if_active()
        self._set_base_lock(True)
        self.fine_cmd_pub.publish(Twist())
        try:
            cancel_ros_stamp = float(rospy.Time.now().to_sec())
        except (AttributeError, TypeError, ValueError):
            cancel_ros_stamp = 0.0

        started = time.monotonic()
        deadline = started + self.fine_handoff_timeout
        def wait_cycle() -> None:
            remaining = deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(0.05, remaining))

        samples: List[Tuple[float, float, float, float]] = []
        last_detail = "no fresh post-cancel base odometry/pose sample"
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                # Reassert both channels every cycle.  A concurrent resume
                # service must not unlock the base halfway through this proof.
                self._set_base_lock(True)
                self.fine_cmd_pub.publish(Twist())
                with self.lock:
                    if self.shutdown:
                        return False, label + " handoff cancelled by shutdown; base remains locked"
                    if self.paused or not self.running:
                        return False, label + " handoff cancelled by pause; base remains locked"
                    retained = bool(self.mine_retained)
                    base_odom = copy.deepcopy(self.base_odom)
                if reject_retained and retained:
                    return False, (
                        label + " handoff rejected unexpected grasp lock; "
                        "base remains locked"
                    )
                if require_retained:
                    if not retained:
                        return False, (
                            "OBJECT_DROPPED: lock lost during " + label
                            + " braking handoff; base remains locked"
                        )
                    attitude_ok, attitude_detail = self._carry_attitude_ok()
                    if not attitude_ok:
                        return False, attitude_detail + "; base remains locked"

                now = time.monotonic()
                if base_odom is None:
                    samples = []
                    last_detail = "base controller odometry unavailable"
                    wait_cycle()
                    continue
                try:
                    odom_stamp = float(base_odom.header.stamp.to_sec())
                    ros_now = float(rospy.Time.now().to_sec())
                except (AttributeError, TypeError, ValueError):
                    odom_stamp = 0.0
                    ros_now = 0.0
                stamp_ok, stamp_detail = tf_stamp_is_post_cancel_fresh(
                    odom_stamp,
                    ros_now,
                    cancel_ros_stamp,
                    self.fine_handoff_odom_max_age,
                )
                if not stamp_ok:
                    samples = []
                    last_detail = "base odometry " + stamp_detail
                    wait_cycle()
                    continue
                twist = base_odom.twist.twist
                velocities = (
                    twist.linear.x, twist.linear.y, twist.angular.z,
                )
                if not all(math.isfinite(value) for value in velocities):
                    samples = []
                    last_detail = "base odometry contains non-finite velocity"
                    wait_cycle()
                    continue
                linear_speed = math.hypot(twist.linear.x, twist.linear.y)
                angular_speed = abs(twist.angular.z)
                if (linear_speed > self.fine_handoff_linear_velocity_tolerance
                        or angular_speed
                        > self.fine_handoff_angular_velocity_tolerance):
                    samples = []
                    last_detail = (
                        "base still braking: linear={:.3f} m/s angular={:.3f} rad/s"
                    ).format(linear_speed, angular_speed)
                    wait_cycle()
                    continue

                # `/husky_velocity_controller/odom` is continuous and comes
                # directly from the controller.  Its pose span and twist must
                # independently agree that the physical base is stopped.  In
                # the 2026-07-14 M003 run these values were exactly stationary
                # while the global EKF shifted map->base by 11.9 mm in 0.5 s.
                point = base_odom.pose.pose.position
                quaternion = base_odom.pose.pose.orientation
                components = (
                    point.x,
                    point.y,
                    quaternion.x,
                    quaternion.y,
                    quaternion.z,
                    quaternion.w,
                )
                quaternion_norm = math.sqrt(sum(
                    value * value for value in components[2:]
                )) if all(math.isfinite(value) for value in components) else 0.0
                if quaternion_norm <= 1.0e-6:
                    samples = []
                    last_detail = (
                        "base controller odometry contains "
                        "non-finite/invalid pose"
                    )
                    wait_cycle()
                    continue
                yaw = euler_from_quaternion([
                    quaternion.x,
                    quaternion.y,
                    quaternion.z,
                    quaternion.w,
                ])[2]
                if not math.isfinite(yaw):
                    samples = []
                    last_detail = "base controller odometry yaw is non-finite"
                    wait_cycle()
                    continue
                samples.append((now, point.x, point.y, yaw))
                metrics = base_motion_window_span(
                    samples,
                    self.fine_handoff_window,
                    self.fine_handoff_min_samples,
                )
                if metrics is None:
                    elapsed = samples[-1][0] - samples[0][0]
                    last_detail = (
                        "collecting pose window {:.3f}/{:.3f} s ({} samples)"
                    ).format(
                        elapsed,
                        self.fine_handoff_window,
                        len(samples),
                    )
                else:
                    duration, translation_span, yaw_span, sample_count = metrics
                    last_detail = (
                        "controller_odom_window={:.3f} s "
                        "translation_span={:.4f} m "
                        "yaw_span={:.3f} deg samples={} velocity="
                        "{:.3f}m/s/{:.3f}rad/s"
                    ).format(
                        duration,
                        translation_span,
                        math.degrees(yaw_span),
                        sample_count,
                        linear_speed,
                        angular_speed,
                    )
                    if (
                        translation_span
                        <= self.fine_handoff_translation_tolerance + 1.0e-9
                        and yaw_span <= self.fine_handoff_yaw_tolerance + 1.0e-9
                    ):
                        return True, label + " brake stable: " + last_detail
                wait_cycle()

            if rospy.is_shutdown():
                return False, label + " handoff cancelled by ROS shutdown; base remains locked"
            return False, (
                "{} braking handoff timeout after {:.2f} s: {}; base remains locked"
            ).format(label, time.monotonic() - started, last_detail)
        finally:
            self.fine_cmd_pub.publish(Twist())
            self._set_base_lock(True)

    def _pose_goal_errors(
        self, target: PoseStamped
    ) -> Optional[Tuple[float, float]]:
        robot = self._robot_pose()
        if robot is None:
            return None
        rp, gp = robot.pose.position, target.pose.position
        distance = math.hypot(gp.x - rp.x, gp.y - rp.y)
        robot_q = robot.pose.orientation
        goal_q = target.pose.orientation
        robot_yaw = euler_from_quaternion(
            [robot_q.x, robot_q.y, robot_q.z, robot_q.w]
        )[2]
        goal_yaw = euler_from_quaternion(
            [goal_q.x, goal_q.y, goal_q.z, goal_q.w]
        )[2]
        return distance, abs(normalize_angle(goal_yaw - robot_yaw))

    def _verify_pose_arrival(self, target: PoseStamped) -> Tuple[bool, str]:
        errors = self._pose_goal_errors(target)
        if errors is None:
            return False, "arrival TF unavailable"
        distance, yaw_error = errors
        detail = "goal_error={:.2f} m, yaw_error={:.1f} deg".format(
            distance, math.degrees(yaw_error)
        )
        return (
            distance <= self.arrival_distance_tolerance
            and yaw_error <= self.arrival_yaw_tolerance,
            detail,
        )

    def _verify_arrival(self, mine_id: int) -> Tuple[bool, str]:
        errors = self._mine_alignment_errors(mine_id)
        if errors is None:
            return False, "arrival TF unavailable"
        distance, yaw_error = errors
        distance_ok = abs(distance - self.approach_distance) <= self.arrival_distance_tolerance
        yaw_ok = abs(yaw_error) <= self.arrival_yaw_tolerance
        detail = (
            f"standoff={distance:.2f} m, yaw_error={math.degrees(abs(yaw_error)):.1f} deg"
        )
        return distance_ok and yaw_ok, detail

    def _verify_coarse_arrival(self, mine_id: int) -> Tuple[bool, str]:
        errors = self._mine_alignment_errors(mine_id)
        if errors is None:
            return False, "coarse arrival TF unavailable"
        distance, yaw_error = errors
        distance_error = abs(distance - self.navigation_approach_distance)
        detail = (
            "coarse standoff={:.3f} m, ring_error={:.3f} m, "
            "yaw_error={:.1f} deg"
        ).format(
            distance,
            distance_error,
            math.degrees(abs(yaw_error)),
        )
        return (
            distance_error <= self.coarse_arrival_distance_tolerance + 1.0e-9
            and abs(yaw_error) <= self.coarse_arrival_yaw_tolerance,
            detail,
        )

    def _mine_alignment_errors(
        self, mine_id: int, pose_timeout: Optional[float] = None
    ) -> Optional[Tuple[float, float]]:
        robot = (
            self._robot_pose()
            if pose_timeout is None
            else self._frame_pose(self.base_frame, timeout=pose_timeout)
        )
        with self.lock:
            task = self.tasks.get(mine_id)
            mine = (
                copy.deepcopy(task.frozen_mine_pose or task.mine_pose)
                if task else None
            )
        if robot is None or mine is None:
            return None
        rp = robot.pose.position
        mp = mine.pose.position
        q = robot.pose.orientation
        components = (rp.x, rp.y, mp.x, mp.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in components):
            return None
        if math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) <= 1.0e-6:
            return None
        distance = math.hypot(mp.x - rp.x, mp.y - rp.y)
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        desired_yaw = math.atan2(mp.y - rp.y, mp.x - rp.x)
        if not all(math.isfinite(value) for value in (distance, yaw, desired_yaw)):
            return None
        return distance, normalize_angle(desired_yaw - yaw)

    @staticmethod
    def _bounded_command(error: float, gain: float, maximum: float,
                         minimum: float) -> float:
        return bounded_command(error, gain, maximum, minimum)

    def _fine_align_to_mine(self, mine_id: int) -> Tuple[bool, str]:
        """Finish the final centimetres/degrees only after cancelling move_base.

        The controller is deliberately bounded to a small neighbourhood of an
        already planned, collision-checked approach.  It never drives toward a
        distant target and always reasserts the independent base lock on exit.
        """
        handed_off, handoff_detail = self._wait_for_fine_alignment_handoff(
            "mine fine alignment", reject_retained=True
        )
        if not handed_off:
            raise FineHandoffFailure(handoff_detail)
        self._set_base_lock(False)
        deadline = time.monotonic() + self.fine_alignment_timeout
        stable_since = None
        last_detail = "alignment has not received map->base TF"
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with self.lock:
                    if self.paused or not self.running:
                        return False, "fine alignment cancelled by mission interlock"
                    if self.mine_retained:
                        raise FineHandoffFailure(
                            "unexpected grasp lock during mine fine alignment; "
                            "base remains locked"
                        )
                errors = self._mine_alignment_errors(mine_id, pose_timeout=0.04)
                if errors is None:
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                distance, yaw_error = errors
                distance_error = distance - self.approach_distance
                last_detail = (
                    "distance_error={:.3f} m yaw_error={:.2f} deg"
                ).format(distance_error, math.degrees(abs(yaw_error)))
                if abs(distance_error) > self.fine_alignment_max_initial_distance_error:
                    return False, last_detail + "; outside bounded fine-alignment region"

                distance_ok = abs(distance_error) <= self.arrival_distance_tolerance
                yaw_ok = abs(yaw_error) <= self.arrival_yaw_tolerance
                if distance_ok and yaw_ok:
                    self.fine_cmd_pub.publish(Twist())
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= self.fine_alignment_hold:
                        return True, handoff_detail + "; fine alignment stable: " + last_detail
                    time.sleep(0.05)
                    continue
                stable_since = None

                command = Twist()
                if not yaw_ok:
                    command.angular.z = self._bounded_command(
                        yaw_error, 1.2, self.fine_alignment_max_angular, 0.05
                    )
                # Correct range only while approximately facing the mine.  This
                # prevents an arc from moving the detonator sideways out of view.
                if abs(yaw_error) <= math.radians(6.0) and not distance_ok:
                    command.linear.x = self._bounded_command(
                        distance_error, 0.8, self.fine_alignment_max_linear, 0.035
                    )
                self.fine_cmd_pub.publish(command)
                time.sleep(0.05)
            return False, "fine alignment timeout: " + last_detail
        finally:
            for _ in range(3):
                self.fine_cmd_pub.publish(Twist())
                time.sleep(0.03)
            self._set_base_lock(True)

    def _fine_align_to_pose(
        self, target: PoseStamped, require_retained: bool = False
    ) -> Tuple[bool, str]:
        """Close the small pose-tolerance gap left by move_base.

        This is used at the disposal parking pose, where DWA may legitimately
        report success around 3 cm / 3.5 deg while PLACE requires 2 cm / 3 deg.
        It is bounded to the same small neighbourhood as mine alignment and,
        when loaded, continuously checks the grasp lock and chassis attitude.
        """
        handed_off, handoff_detail = self._wait_for_fine_alignment_handoff(
            "dropoff fine alignment", require_retained=require_retained
        )
        if not handed_off:
            return False, handoff_detail
        self._set_base_lock(False)
        deadline = time.monotonic() + self.fine_alignment_timeout
        stable_since = None
        last_detail = "dropoff alignment has not received map->base TF"
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with self.lock:
                    if self.paused or not self.running:
                        return False, "paused"
                    retained = bool(self.mine_retained)
                if require_retained:
                    if not retained:
                        return False, "OBJECT_DROPPED: lock lost during dropoff alignment"
                    attitude_ok, attitude_detail = self._carry_attitude_ok()
                    if not attitude_ok:
                        return False, attitude_detail

                robot = self._frame_pose(self.base_frame, timeout=0.04)
                if robot is None:
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue

                rp, gp = robot.pose.position, target.pose.position
                dx, dy = gp.x - rp.x, gp.y - rp.y
                robot_q = robot.pose.orientation
                goal_q = target.pose.orientation
                components = (
                    dx, dy,
                    robot_q.x, robot_q.y, robot_q.z, robot_q.w,
                    goal_q.x, goal_q.y, goal_q.z, goal_q.w,
                )
                if not all(math.isfinite(value) for value in components):
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                distance = math.hypot(dx, dy)
                robot_yaw = euler_from_quaternion(
                    [robot_q.x, robot_q.y, robot_q.z, robot_q.w]
                )[2]
                goal_yaw = euler_from_quaternion(
                    [goal_q.x, goal_q.y, goal_q.z, goal_q.w]
                )[2]
                final_yaw_error = normalize_angle(goal_yaw - robot_yaw)
                if not all(math.isfinite(value) for value in (
                    distance, robot_yaw, goal_yaw, final_yaw_error
                )):
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                last_detail = (
                    "goal_error={:.3f} m yaw_error={:.2f} deg"
                ).format(distance, math.degrees(abs(final_yaw_error)))
                if (distance
                        > self.dropoff_fine_alignment_max_initial_distance_error):
                    return False, last_detail + "; outside bounded alignment region"

                position_ok = distance <= self.arrival_distance_tolerance
                yaw_ok = abs(final_yaw_error) <= self.arrival_yaw_tolerance
                if position_ok and yaw_ok:
                    self.fine_cmd_pub.publish(Twist())
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= self.fine_alignment_hold:
                        return True, handoff_detail + "; dropoff fine alignment stable: " + last_detail
                    time.sleep(0.05)
                    continue
                stable_since = None

                command = Twist()
                if not position_ok:
                    travel_yaw = math.atan2(dy, dx)
                    direction = 1.0
                    heading_error = normalize_angle(travel_yaw - robot_yaw)
                    # The last centimetres may lie behind the footprint after a
                    # DWA overshoot. Reverse instead of making a loaded U-turn.
                    if abs(heading_error) > math.pi / 2.0:
                        direction = -1.0
                        heading_error = normalize_angle(
                            travel_yaw + math.pi - robot_yaw
                        )
                    if abs(heading_error) > math.radians(1.0):
                        command.angular.z = self._bounded_command(
                            heading_error,
                            1.2,
                            self.fine_alignment_max_angular,
                            0.04,
                        )
                    if abs(heading_error) <= math.radians(6.0):
                        maximum = self.fine_alignment_max_linear
                        if require_retained:
                            maximum = min(maximum, self.carry_speed)
                        command.linear.x = direction * self._bounded_command(
                            distance, 0.8, maximum, 0.025
                        )
                else:
                    command.angular.z = self._bounded_command(
                        final_yaw_error,
                        1.2,
                        self.fine_alignment_max_angular,
                        0.04,
                    )
                self.fine_cmd_pub.publish(command)
                time.sleep(0.05)
            return False, "dropoff fine alignment timeout: " + last_detail
        finally:
            for _ in range(3):
                self.fine_cmd_pub.publish(Twist())
                time.sleep(0.03)
            self._set_base_lock(True)

    def _retreat_after_empty_pick(
        self, mine_id: int, recovered_to_look: bool
    ) -> Tuple[bool, str]:
        """Leave the inner grasp ring after a verified empty-arm recovery."""
        self.fine_cmd_pub.publish(Twist())
        self._set_base_lock(True)
        if not recovered_to_look:
            return False, "arm recovery to LOOK was not verified"
        return self._retreat_from_grasp_pose(
            mine_id,
            require_retained=False,
            timeout=self.empty_pick_retreat_timeout,
            label="empty-pick",
            target_distance=self.navigation_approach_distance,
        )

    def _retreat_from_grasp_pose(
        self,
        mine_id: int,
        require_retained: bool,
        timeout: float,
        label: str,
        target_distance: float,
    ) -> Tuple[bool, str]:
        """Reverse-only transition from the inner grasp pose to the outer ring.

        The source hazard stays active throughout.  Empty and loaded exits use
        identical geometry, but opposite grasp-lock predicates; loaded motion
        also enforces the carry attitude and speed gates every cycle.
        """
        self.fine_cmd_pub.publish(Twist())
        self._set_base_lock(True)
        with self.lock:
            retained = bool(self.mine_retained)
            if require_retained and not retained:
                return False, "OBJECT_DROPPED: physical grasp lock missing before retreat"
            if not require_retained and retained:
                return False, "unexpected physical grasp lock before retreat"
            task = self.tasks.get(mine_id)
            if task is None or task.frozen_mine_pose is None:
                return False, "frozen mine target unavailable for retreat"
            if self.paused or not self.running:
                return False, "retreat cancelled by pause"
            if require_retained:
                task.detail = (
                    "PICK retained; backing straight out to {:.2f} m outer "
                    "ring before loaded return"
                ).format(target_distance)
            else:
                task.detail = (
                    "empty PICK recovered to LOOK; preparing bounded reverse "
                    "retreat to {:.2f} m outer ring"
                ).format(target_distance)
            task.touch()
        if require_retained:
            attitude_ok, attitude_detail = self._carry_attitude_ok()
            if not attitude_ok:
                return False, attitude_detail + "; base remains locked"
        self._publish_all()

        handed_off, start_brake_detail = self._wait_for_fine_alignment_handoff(
            label + " retreat",
            require_retained=require_retained,
            reject_retained=not require_retained,
        )
        if not handed_off:
            return False, start_brake_detail

        maximum_linear = self.fine_alignment_max_linear
        maximum_angular = self.fine_alignment_max_angular
        if require_retained:
            maximum_linear = min(maximum_linear, self.carry_speed)
            maximum_angular = min(maximum_angular, self.carry_turn_speed)

        self.fine_cmd_pub.publish(Twist())
        self._set_base_lock(False)
        deadline = time.monotonic() + timeout
        stable_since = None
        reached_detail = None
        last_detail = "retreat has not received map->base TF"
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with self.lock:
                    if self.shutdown:
                        return False, "retreat cancelled by shutdown"
                    if self.paused or not self.running:
                        return False, "retreat cancelled by pause"
                    retained = bool(self.mine_retained)
                    if require_retained and not retained:
                        return False, "OBJECT_DROPPED: physical grasp lock lost during retreat"
                    if not require_retained and retained:
                        return False, "unexpected physical grasp lock during retreat"
                if require_retained:
                    attitude_ok, attitude_detail = self._carry_attitude_ok()
                    if not attitude_ok:
                        return False, attitude_detail

                errors = self._mine_alignment_errors(
                    mine_id, pose_timeout=0.04
                )
                if errors is None:
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                distance, yaw_error = errors
                distance_error = distance - target_distance
                last_detail = (
                    "standoff={:.3f} m ring_error={:.3f} m "
                    "yaw_error={:.2f} deg"
                ).format(
                    distance,
                    distance_error,
                    math.degrees(abs(yaw_error)),
                )
                if abs(distance_error) > self.fine_alignment_max_initial_distance_error:
                    return False, last_detail + "; outside bounded retreat region"

                state, linear, angular = empty_pick_retreat_step(
                    distance,
                    yaw_error,
                    target_distance,
                    self.arrival_distance_tolerance,
                    self.arrival_yaw_tolerance,
                    maximum_linear,
                    maximum_angular,
                )
                if state == "invalid":
                    return False, last_detail + "; non-finite retreat geometry"
                if state == "overshot":
                    return False, last_detail + "; outer ring overshot; forward correction forbidden"
                if state == "heading_rejected":
                    return False, last_detail + "; heading exceeds straight-retreat gate"
                if state == "reached":
                    self.fine_cmd_pub.publish(Twist())
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= self.fine_alignment_hold:
                        reached_detail = last_detail
                        break
                    time.sleep(0.05)
                    continue
                stable_since = None
                command = Twist()
                command.linear.x = linear
                command.angular.z = angular
                self.fine_cmd_pub.publish(command)
                time.sleep(0.05)
        finally:
            for _ in range(3):
                self.fine_cmd_pub.publish(Twist())
                time.sleep(0.03)
            self._set_base_lock(True)

        if reached_detail is None:
            return False, label + " retreat timeout: " + last_detail
        stopped, final_brake_detail = self._wait_for_fine_alignment_handoff(
            label + " outer-ring stop",
            require_retained=require_retained,
            reject_retained=not require_retained,
        )
        if not stopped:
            return False, final_brake_detail
        final_errors = self._mine_alignment_errors(
            mine_id, pose_timeout=0.04
        )
        if final_errors is None:
            return False, "outer-ring verification TF unavailable; base remains locked"
        final_distance, final_yaw_error = final_errors
        final_ring_error = abs(
            final_distance - target_distance
        )
        if (
            final_ring_error > self.arrival_distance_tolerance + 1.0e-9
            or abs(final_yaw_error) > self.arrival_yaw_tolerance
        ):
            return False, (
                "post-brake outer-ring verification failed: standoff={:.3f} m "
                "ring_error={:.3f} m yaw_error={:.2f} deg; base remains locked"
            ).format(
                final_distance,
                final_ring_error,
                math.degrees(abs(final_yaw_error)),
            )
        return True, (
            "{} retreat complete: {}; {}; {}"
        ).format(label, start_brake_detail, reached_detail, final_brake_detail)

    def _wait_for_manipulation_server(
        self, mine_id: int, operation: str
    ) -> Tuple[bool, str]:
        """Wait safely at the parking pose instead of skipping the mine."""
        last_status_update = 0.0
        while not rospy.is_shutdown():
            with self.lock:
                if self.shutdown:
                    return False, "shutdown"
                if self.paused or not self.running:
                    return False, "{} cancelled by pause".format(operation)
                if self.tasks.get(mine_id) is None:
                    return False, "task disappeared"
            if self._wait_for_action_server(
                self.grasp_client, min(max(self.arm_server_wait, 0.2), 1.0)
            ):
                return True, ""

            self._set_base_lock(True)
            now = time.monotonic()
            if now - last_status_update >= 2.0:
                with self.lock:
                    task = self.tasks.get(mine_id)
                    if task is not None:
                        task.detail = (
                            "{} waiting for /mine_grasp; base remains locked"
                        ).format(operation)
                        task.touch()
                self._publish_all()
                last_status_update = now
            rospy.logerr_throttle(
                3.0,
                "[MineMission] %s server unavailable at M%03d; holding position "
                "with base locked",
                operation,
                mine_id,
            )
        return False, "shutdown"

    def _run_arm_action(self, mine_id: int) -> ArmAttemptResult:
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None:
                return ArmAttemptResult(False, "task disappeared")
            task.state = MineMissionEntry.WAITING_ARM
            task.detail = "waiting for arm action server"
            task.touch()
        self._set_base_lock(True)
        self._publish_all()

        ready, detail = self._wait_for_manipulation_server(mine_id, "PICK")
        if not ready:
            return ArmAttemptResult(False, detail)

        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None:
                return ArmAttemptResult(False, "task disappeared")
            task.grasp_attempts += 1
            task.detail = "PICK Action connected; starting wrist localization"
            task.touch()
            goal = MineGraspGoal()
            goal.operation = MineGraspGoal.PICK
            goal.mine_id = mine_id
            goal.source_detection_id = mine_id
            goal.mine_pose = copy.deepcopy(task.frozen_mine_pose or task.mine_pose)
            goal.approach_pose = copy.deepcopy(task.approach_pose)
            goal.nominal_diameter_m = self.nominal_diameter
        self._publish_all()
        self.grasp_client.send_goal(goal, feedback_cb=self._arm_feedback_cb)
        deadline = time.monotonic() + self.arm_timeout
        terminal_since = None
        while not rospy.is_shutdown():
            with self.lock:
                if self.paused or not self.running:
                    self.grasp_client.cancel_goal()
                    return ArmAttemptResult(False, "arm cancelled by pause")
            state = self.grasp_client.get_state()
            if state in (
                GoalStatus.SUCCEEDED,
                GoalStatus.ABORTED,
                GoalStatus.REJECTED,
                GoalStatus.PREEMPTED,
                GoalStatus.RECALLED,
                GoalStatus.LOST,
            ):
                result = self.grasp_client.get_result()
                if result is None and state != GoalStatus.LOST:
                    if terminal_since is None:
                        terminal_since = time.monotonic()
                    if time.monotonic() - terminal_since < self.action_result_grace:
                        time.sleep(0.02)
                        continue
                with self.lock:
                    retained = bool(self.mine_retained)
                if (
                    state == GoalStatus.SUCCEEDED
                    and result is not None
                    and result.success
                    and result.outcome == MineGraspResult.SUCCESS
                ):
                    if not retained and self._wait_retained(True, 2.0):
                        retained = True
                    if retained:
                        return ArmAttemptResult(
                            True,
                            result.message or "arm reported verified SUCCESS",
                            retained=True,
                            action_outcome=int(result.outcome),
                        )
                    return ArmAttemptResult(
                        False,
                        "PICK action reported SUCCESS but /mine_grasp/retained "
                        "did not become true",
                        code="OBJECT_DROPPED",
                        recovered_to_look=False,
                        retained=False,
                        action_outcome=int(result.outcome),
                    )
                if result is None:
                    return ArmAttemptResult(
                        False,
                        f"arm ended {goal_status_name(state)} without result",
                        retained=retained,
                    )
                if result.outcome == MineGraspResult.CANCELLED:
                    return ArmAttemptResult(
                        False,
                        "manipulation was cancelled while the arm pose is "
                        "unverified; reset before navigation: "
                        + (result.message or goal_status_name(state)),
                        code="CANCELLED",
                        recovered_to_look=False,
                        retained=retained,
                        action_outcome=int(result.outcome),
                    )
                report = decode_executor_report(result.message)
                code = normalized_executor_failure_code(report)
                recovered = bool(
                    report.get("recovery_attempted", False)
                    and report.get("recovery_success", False)
                    and not retained
                )
                detail_text = (
                    f"arm outcome={result.outcome} state={goal_status_name(state)}: "
                    f"{result.message}"
                )
                return ArmAttemptResult(
                    False,
                    detail_text,
                    code=code,
                    recovered_to_look=recovered,
                    retained=retained,
                    action_outcome=int(result.outcome),
                )
            if time.monotonic() >= deadline:
                self.grasp_client.cancel_goal()
                # Wait for the action protocol to acknowledge preemption, but
                # never infer a safe arm pose from that acknowledgement.
                cancel_deadline = time.monotonic() + 30.0
                while (not rospy.is_shutdown()
                       and time.monotonic() < cancel_deadline
                       and self.grasp_client.get_state() not in (
                           GoalStatus.PREEMPTED,
                           GoalStatus.ABORTED,
                           GoalStatus.REJECTED,
                           GoalStatus.RECALLED,
                           GoalStatus.LOST,
                           GoalStatus.SUCCEEDED,
                       )):
                    self._set_base_lock(True)
                    time.sleep(0.1)
                with self.lock:
                    retained = bool(self.mine_retained)
                return ArmAttemptResult(False, (
                    "arm action timed out after {:.1f} s; "
                    "goal cancelled but arm pose is unverified, reset the "
                    "executor manually before navigation"
                ).format(self.arm_timeout), code="MOTION_TIMEOUT",
                    recovered_to_look=False, retained=retained)
            time.sleep(0.1)
        return ArmAttemptResult(False, "shutdown")

    def _complete_delivery(self, mine_id: int) -> None:
        """Return a physically locked mine to its reserved slot and release it."""
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None or task.state in TERMINAL_STATES:
                return
            self.current_mine_id = mine_id
            self._assign_drop_slot_locked(task)
            dropoff = copy.deepcopy(task.dropoff_pose)
        if not self._wait_retained(True, 2.0):
            self._delivery_failed(mine_id, "grasp lock was not retained after PICK")
            return
        if not self._set_carry_speed(True):
            self._delivery_failed(
                mine_id, "cannot apply locked-return DWA profile"
            )
            return

        with self.lock:
            task = self.tasks[mine_id]
            task.state = MineMissionEntry.RETURNING_HOME
            task.detail = (
                "preparing loaded egress for disposal slot {} at <= {:.2f} m/s".format(
                    task.disposal_slot, self.carry_speed
                )
            )
            task.touch()
        self._publish_all()

        # PICK has proved the simulator transport lock.  The same mine remains
        # visible in Gazebo as a no-reaction kinematic follower, while
        # RETURNING_HOME suppresses only its stored source coordinate in the
        # fusion map, both hazard layers and RViz. Clear stale inflated cells,
        # perform a bounded in-place turn, then hand translation to move_base.
        if self.hazard_transition_settle > 0.0:
            time.sleep(self.hazard_transition_settle)
        source_clear_detail = self._clear_navigation_costmaps()
        if not self._wait_retained(True, 0.25):
            self._delivery_failed(
                mine_id,
                "physical lock lost while clearing the picked source hazard",
            )
            return
        attitude_ok, attitude_detail = self._carry_attitude_ok()
        if not attitude_ok:
            self._delivery_failed(mine_id, attitude_detail)
            return
        retreat_detail = (
            "retained source M{:03d} removed from navigation map; {}"
        ).format(mine_id, source_clear_detail)
        rospy.logwarn(
            "[MineMission] M%03d PICK lock confirmed; source hazard removed; "
            "turning directly toward disposal slot",
            mine_id,
        )
        turned, turn_detail = self._turn_loaded_toward(dropoff)
        if not turned and (
            turn_detail.startswith("OBJECT_DROPPED")
            or turn_detail.startswith("BASE_UNSTABLE")
        ):
            self._delivery_failed(mine_id, turn_detail)
            return
        if not turned and "pause" in turn_detail:
            with self.lock:
                task = self.tasks.get(mine_id)
                if task and task.state not in TERMINAL_STATES:
                    task.state = MineMissionEntry.CARRYING
                    task.detail = turn_detail + "; virtual/physical lock retained"
                    task.touch()
            self._publish_all()
            return
        if turned:
            rospy.loginfo(
                "[MineMission] M%03d loaded departure turn complete: %s",
                mine_id,
                turn_detail,
            )
        else:
            # A transient TF starvation must not replace obstacle-aware return;
            # move_base can still rotate once its transforms recover.
            rospy.logwarn(
                "[MineMission] M%03d direct loaded turn incomplete (%s); "
                "continuing with move_base",
                mine_id,
                turn_detail,
            )
        retreat_detail += "; " + turn_detail

        outcome = "loaded return not started"
        for attempt in range(1, self.loaded_return_attempts + 1):
            with self.lock:
                task = self.tasks.get(mine_id)
                if task and task.state not in TERMINAL_STATES:
                    task.state = MineMissionEntry.RETURNING_HOME
                    task.detail = (
                        "{}; loaded return attempt {}/{} to slot {} at <= "
                        "{:.2f} m/s"
                    ).format(
                        retreat_detail,
                        attempt,
                        self.loaded_return_attempts,
                        task.disposal_slot,
                        self.carry_speed,
                    )
                    task.touch()
            self._publish_all()

            self._set_base_lock(False)
            outcome = self._navigate_to(dropoff, require_retained=True)
            if outcome == "success":
                break
            if outcome == "paused":
                with self.lock:
                    task = self.tasks.get(mine_id)
                    if task and task.state not in TERMINAL_STATES:
                        task.state = MineMissionEntry.CARRYING
                        task.detail = (
                            "paused while carrying; physical lock retained"
                        )
                        task.touch()
                self._set_base_lock(True)
                self._publish_all()
                return
            if (
                outcome.startswith("OBJECT_DROPPED")
                or outcome.startswith("BASE_UNSTABLE")
                or outcome == "shutdown"
            ):
                self._delivery_failed(
                    mine_id, "loaded return safety stop: " + outcome
                )
                return
            if attempt >= self.loaded_return_attempts:
                self._delivery_failed(
                    mine_id,
                    "loaded return failed after {} attempts: {}".format(
                        self.loaded_return_attempts, outcome
                    ),
                )
                return

            stopped, stop_detail = self._wait_for_fine_alignment_handoff(
                "loaded-return retry", require_retained=True
            )
            if not stopped:
                self._delivery_failed(
                    mine_id,
                    "loaded return retry could not prove stopped base: "
                    + stop_detail,
                )
                return
            clear_detail = self._clear_navigation_costmaps()
            with self.lock:
                task = self.tasks.get(mine_id)
                if task and task.state not in TERMINAL_STATES:
                    task.detail = (
                        "loaded return attempt {}/{} ended {}; {}; {}; retrying"
                    ).format(
                        attempt,
                        self.loaded_return_attempts,
                        outcome,
                        stop_detail,
                        clear_detail,
                    )
                    task.touch()
            self._publish_all()
            rospy.logwarn(
                "[MineMission] M%03d loaded return attempt %d/%d failed: %s; %s",
                mine_id,
                attempt,
                self.loaded_return_attempts,
                outcome,
                clear_detail,
            )
            if self.loaded_return_retry_delay > 0.0:
                time.sleep(self.loaded_return_retry_delay)

        # Never spend another minute chasing an exact depot XY/yaw.  The
        # move_base goal has already entered a clear disposal neighbourhood;
        # all that remains is to prove the chassis stopped, then preserve the
        # reserved base->drop transform at the *actual* base pose.  This makes
        # PLACE reachable without moving the base by another millimetre.
        stopped, stop_detail = self._wait_for_fine_alignment_handoff(
            "dropoff release", require_retained=True
        )
        if not stopped:
            with self.lock:
                paused = self.paused or not self.running
                retained = bool(self.mine_retained)
            if paused and retained:
                with self.lock:
                    task = self.tasks.get(mine_id)
                    if task and task.state not in TERMINAL_STATES:
                        task.state = MineMissionEntry.CARRYING
                        task.detail = (
                            "paused at disposal area; physical lock retained"
                        )
                        task.touch()
                self._publish_all()
                return
            if (not retained
                    or stop_detail.startswith("OBJECT_DROPPED")
                    or stop_detail.startswith("BASE_UNSTABLE")):
                self._delivery_failed(mine_id, stop_detail)
                return
            # The executor has its own controller-odometry stationary gate
            # (8 s).  Do not strand a retained mine merely because this shorter
            # 2 s handoff window missed enough samples; keep the priority base
            # lock asserted and let PLACE own the final stop proof.
            rospy.logwarn(
                "[MineMission] M%03d short dropoff brake proof unavailable "
                "(%s); continuing with executor stationary gate",
                mine_id,
                stop_detail,
            )
            stop_detail += "; executor stationary gate required"
        accepted, detail = self._relocate_drop_to_actual_base(
            mine_id, dropoff
        )
        if not accepted:
            self._delivery_failed(mine_id, detail)
            return
        if not self._wait_retained(True, 0.5):
            self._delivery_failed(mine_id, "mine lock lost at dropoff")
            return

        with self.lock:
            task = self.tasks[mine_id]
            task.state = MineMissionEntry.AT_DROPOFF
            task.detail = detail + "; " + stop_detail + "; base locked before release"
            task.touch()
        self._publish_all()
        rospy.logwarn(
            "[MineMission] M%03d inside relaxed disposal area; exact XY/yaw "
            "alignment skipped; starting PLACE at the current base pose",
            mine_id,
        )
        placed, place_detail = self._run_place_action(mine_id)
        if not placed:
            with self.lock:
                paused = self.paused
                retained = self.mine_retained
            if paused and retained:
                with self.lock:
                    task = self.tasks.get(mine_id)
                    if task:
                        task.state = MineMissionEntry.CARRYING
                        task.detail = "place cancelled by pause; lock retained"
                        task.touch()
                self._publish_all()
                return
            # The normal arm sequence is preferred because it lowers the mine
            # onto the ground.  If its IK/controller/validation still fails,
            # explicitly open and release the simulator lock at the current
            # disposal-area pose instead of globally blocking later mines.
            dropped, drop_detail = self._run_drop_now_fallback(
                mine_id, place_detail
            )
            if not dropped:
                self._delivery_failed(mine_id, drop_detail)
                return
            placed = True
            place_detail = drop_detail
        if not self._wait_retained(False, 2.0):
            self._delivery_failed(mine_id, "PLACE returned success but physical lock remains")
            return

        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None:
                return
            self.disposed_poses[mine_id] = copy.deepcopy(task.drop_pose.pose)
            task.state = MineMissionEntry.CLEARED
            task.detail = (
                place_detail
                + "; deposited near HOME; preparing post-place reverse"
            )
            task.touch()
        # Publish the new disposal hazard before CLEARED removes the source
        # hazard, with the base locked throughout the transition.
        self._publish_all()
        # Give both local and global costmap callbacks at least two 10 Hz
        # cycles to paint the new depot hazard before navigation is unlocked.
        time.sleep(max(0.0, self.hazard_transition_settle))
        self._set_carry_speed(False)
        retreat_ok, retreat_detail = self._retreat_after_place(mine_id)
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is not None:
                task.detail = (
                    place_detail + "; deposited near HOME; " + retreat_detail
                )
                task.touch()
            self.current_mine_id = None
        self._publish_all()
        if not retreat_ok:
            # The mine is already released and the arm is empty.  A failed
            # optional egress must not convert a completed task into a global
            # MANUAL_REQUIRED lock or inhibit every remaining mine.
            rospy.logwarn(
                "[MineMission] M%03d post-place reverse incomplete: %s; "
                "continuing remaining tasks",
                mine_id,
                retreat_detail,
            )
        self._set_base_lock(False)
        rospy.loginfo(
            "[MineMission] M%03d PICKED, RETURNED, PLACED and CLEARED; %s",
            mine_id,
            retreat_detail,
        )

    def _retreat_after_place(self, mine_id: int) -> Tuple[bool, str]:
        """Reverse a measured distance after releasing a disposal-slot mine.

        This is intentionally distance-only: no depot XY or yaw alignment is
        reintroduced.  Controller odometry measures progress, and failure is
        non-fatal because PLACE has already completed with no retained load.
        """
        handed_off, start_detail = self._wait_for_fine_alignment_handoff(
            "post-place retreat", reject_retained=True
        )
        if not handed_off:
            return False, start_detail

        with self.lock:
            start_odom = copy.deepcopy(self.base_odom)
            task = self.tasks.get(mine_id)
            if task is not None:
                task.detail = (
                    "mine released; reversing {:.2f} m before next task"
                ).format(self.post_place_retreat_distance)
                task.touch()
        if start_odom is None:
            return False, "post-place retreat has no controller odometry"
        start = start_odom.pose.pose.position
        if not all(math.isfinite(value) for value in (start.x, start.y)):
            return False, "post-place retreat start odometry is non-finite"
        self._publish_all()

        deadline = time.monotonic() + self.post_place_retreat_timeout
        reached = False
        last_progress = 0.0
        self.fine_cmd_pub.publish(Twist())
        self._set_base_lock(False)
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                with self.lock:
                    if self.shutdown:
                        return False, "post-place retreat cancelled by shutdown"
                    if self.paused or not self.running:
                        return False, "post-place retreat cancelled by pause"
                    if self.mine_retained:
                        return False, (
                            "post-place retreat rejected unexpected grasp lock"
                        )
                    odom = copy.deepcopy(self.base_odom)
                if odom is None:
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                point = odom.pose.pose.position
                if not all(math.isfinite(value) for value in (point.x, point.y)):
                    self.fine_cmd_pub.publish(Twist())
                    time.sleep(0.05)
                    continue
                last_progress = math.hypot(point.x - start.x, point.y - start.y)
                remaining = self.post_place_retreat_distance - last_progress
                if remaining <= 0.0:
                    reached = True
                    break
                command = Twist()
                speed = min(
                    self.post_place_retreat_speed,
                    max(0.16, 1.2 * remaining),
                )
                command.linear.x = -speed
                self.fine_cmd_pub.publish(command)
                time.sleep(0.05)
        finally:
            for _ in range(3):
                self.fine_cmd_pub.publish(Twist())
                time.sleep(0.03)
            self._set_base_lock(True)

        if not reached:
            return False, (
                "post-place reverse timed out at {:.2f}/{:.2f} m"
            ).format(last_progress, self.post_place_retreat_distance)
        stopped, final_detail = self._wait_for_fine_alignment_handoff(
            "post-place retreat stop", reject_retained=True
        )
        if not stopped:
            return False, final_detail
        return True, (
            "post-place reverse complete ({:.2f} m); {}; {}"
        ).format(last_progress, start_detail, final_detail)

    def _relocate_drop_to_actual_base(
        self, mine_id: int, reserved_dropoff: PoseStamped
    ) -> Tuple[bool, str]:
        """Keep the reserved arm-relative drop geometry at actual arrival.

        Only the depot-area membership is checked.  Yaw is deliberately not an
        acceptance criterion: rotating the reserved parking->drop vector by
        the measured base yaw produces the same arm-relative target from any
        arrival heading.
        """
        robot = self._robot_pose()
        if robot is None:
            return False, "relaxed dropoff rejected: map->base TF unavailable"
        with self.lock:
            task = self.tasks.get(mine_id)
            reserved_drop = (
                copy.deepcopy(task.drop_pose)
                if task is not None and task.drop_pose is not None
                else None
            )
        if reserved_drop is None:
            return False, "relaxed dropoff rejected: reserved drop pose unavailable"

        rp = robot.pose.position
        pp = reserved_dropoff.pose.position
        distance = math.hypot(rp.x - pp.x, rp.y - pp.y)
        if distance > self.dropoff_relaxed_acceptance_radius:
            return False, (
                "relaxed dropoff rejected: {:.3f} m from reserved parking "
                "exceeds {:.3f} m disposal radius"
            ).format(distance, self.dropoff_relaxed_acceptance_radius)

        parking_q = reserved_dropoff.pose.orientation
        parking_yaw = euler_from_quaternion([
            parking_q.x, parking_q.y, parking_q.z, parking_q.w,
        ])[2]
        robot_q = robot.pose.orientation
        robot_yaw = euler_from_quaternion([
            robot_q.x, robot_q.y, robot_q.z, robot_q.w,
        ])[2]
        world_dx = reserved_drop.pose.position.x - pp.x
        world_dy = reserved_drop.pose.position.y - pp.y
        relative_x = (
            math.cos(parking_yaw) * world_dx
            + math.sin(parking_yaw) * world_dy
        )
        relative_y = (
            -math.sin(parking_yaw) * world_dx
            + math.cos(parking_yaw) * world_dy
        )

        actual_drop = copy.deepcopy(reserved_drop)
        actual_drop.header.frame_id = self.map_frame
        actual_drop.header.stamp = rospy.Time.now()
        actual_drop.pose.position.x = (
            rp.x
            + math.cos(robot_yaw) * relative_x
            - math.sin(robot_yaw) * relative_y
        )
        actual_drop.pose.position.y = (
            rp.y
            + math.sin(robot_yaw) * relative_x
            + math.cos(robot_yaw) * relative_y
        )

        actual_parking = copy.deepcopy(robot)
        actual_parking.header.frame_id = self.map_frame
        actual_parking.header.stamp = actual_drop.header.stamp
        actual_parking.pose.position.z = 0.0
        yaw_only = quaternion_from_euler(0.0, 0.0, robot_yaw)
        actual_parking.pose.orientation.x = yaw_only[0]
        actual_parking.pose.orientation.y = yaw_only[1]
        actual_parking.pose.orientation.z = yaw_only[2]
        actual_parking.pose.orientation.w = yaw_only[3]

        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None or task.state in TERMINAL_STATES:
                return False, "relaxed dropoff rejected: task disappeared"
            task.dropoff_pose = actual_parking
            task.drop_pose = actual_drop
            task.touch()
        return True, (
            "relaxed dropoff accepted at {:.3f} m with no yaw requirement; "
            "drop target regenerated {:.3f} m forward / {:.3f} m lateral "
            "from actual base"
        ).format(distance, relative_x, relative_y)

    def _run_drop_now_fallback(
        self, mine_id: int, normal_place_failure: str
    ) -> Tuple[bool, str]:
        """Request the executor's idempotent depot-area direct unlock."""
        self._set_base_lock(True)
        try:
            rospy.wait_for_service(self.drop_now_service_name, timeout=2.0)
            response = None
            deadline = time.monotonic() + 8.0
            while not rospy.is_shutdown():
                response = self.drop_now()
                if (response.success
                        or "executor is already busy" not in response.message
                        or time.monotonic() >= deadline):
                    break
                # A timed-out action may need one callback cycle to release
                # the executor lock after cancellation.  Retry only this
                # explicit transient state; every physical failure remains
                # immediate and visible.
                time.sleep(0.20)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, (
                "PLACE failed ({}); direct drop service unavailable: {}"
            ).format(normal_place_failure, exc)
        if response is None:
            return False, "PLACE failed; direct drop cancelled by ROS shutdown"
        if not response.success:
            return False, (
                "PLACE failed ({}); direct drop also failed: {}"
            ).format(normal_place_failure, response.message)
        if not self._wait_retained(False, 3.0):
            return False, (
                "PLACE failed ({}); direct drop returned success but lock remains"
            ).format(normal_place_failure)
        rospy.logwarn(
            "[MineMission] M%03d normal PLACE failed; direct depot-area "
            "unlock completed so the remaining mines can continue: %s",
            mine_id,
            normal_place_failure,
        )
        return True, (
            "best-effort depot drop completed after normal PLACE failure: {}; {}"
        ).format(normal_place_failure, response.message)

    def _run_place_action(self, mine_id: int) -> Tuple[bool, str]:
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None or task.drop_pose is None:
                return False, "drop target unavailable"
            task.state = MineMissionEntry.PLACING
            task.detail = "waiting for controlled unlock at disposal slot"
            task.touch()
            goal = MineGraspGoal()
            goal.operation = MineGraspGoal.PLACE
            goal.mine_id = mine_id
            goal.source_detection_id = mine_id
            goal.mine_pose = copy.deepcopy(task.frozen_mine_pose or task.mine_pose)
            goal.approach_pose = copy.deepcopy(task.dropoff_pose)
            goal.drop_pose = copy.deepcopy(task.drop_pose)
            goal.nominal_diameter_m = self.nominal_diameter
        self._publish_all()
        ready, detail = self._wait_for_manipulation_server(mine_id, "PLACE")
        if not ready:
            return False, detail
        self.grasp_client.send_goal(goal, feedback_cb=self._arm_feedback_cb)
        deadline = time.monotonic() + self.place_timeout
        terminal_since = None
        while not rospy.is_shutdown():
            with self.lock:
                if self.paused or not self.running:
                    self.grasp_client.cancel_goal()
                    return False, "PLACE cancelled by pause"
            state = self.grasp_client.get_state()
            if state in (
                GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST,
            ):
                result = self.grasp_client.get_result()
                if result is None and state != GoalStatus.LOST:
                    if terminal_since is None:
                        terminal_since = time.monotonic()
                    if time.monotonic() - terminal_since < self.action_result_grace:
                        time.sleep(0.02)
                        continue
                if (state == GoalStatus.SUCCEEDED and result is not None
                        and result.success
                        and result.outcome == MineGraspResult.SUCCESS):
                    return True, result.message or "controlled PLACE succeeded"
                if result is None:
                    return False, "PLACE ended {} without result".format(
                        goal_status_name(state)
                    )
                return False, "PLACE outcome={} state={}: {}".format(
                    result.outcome, goal_status_name(state), result.message
                )
            if time.monotonic() >= deadline:
                self.grasp_client.cancel_goal()
                return False, "PLACE timed out after {:.1f} s".format(
                    self.place_timeout
                )
            time.sleep(0.1)
        return False, "shutdown"

    def _wait_retained(self, expected: bool, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                if self.mine_retained == expected:
                    return True
            time.sleep(0.02)
        return False

    def _delivery_failed(self, mine_id: int, reason: str) -> None:
        self._cancel_navigation_goal_if_active()
        self._set_base_lock(True)
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is not None:
                task.state = MineMissionEntry.MANUAL_REQUIRED
                task.detail = reason + "; base locked for manual recovery"
                task.touch()
            self.current_mine_id = None
        self._publish_all()
        rospy.logerr("[MineMission] M%03d delivery stopped: %s", mine_id, reason)

    def _set_carry_speed(self, enabled: bool) -> bool:
        if enabled == self.carry_speed_active:
            return True
        try:
            if self.dwa_client is None:
                self.dwa_client = DynamicReconfigureClient(
                    self.dwa_reconfigure_name, timeout=2.0
                )
            if enabled:
                current = self.dwa_client.get_configuration(timeout=2.0)
                keys = (
                    "max_vel_x", "max_vel_trans", "max_vel_theta",
                    "min_vel_trans", "min_vel_theta",
                    "acc_lim_x", "acc_lim_theta",
                )
                self.normal_dwa_config = {
                    key: current[key] for key in keys if key in current
                }
                updated = self.dwa_client.update_configuration({
                    "max_vel_x": self.carry_speed,
                    "max_vel_trans": self.carry_speed,
                    "max_vel_theta": self.carry_turn_speed,
                    "min_vel_trans": self.carry_min_vel_trans,
                    "min_vel_theta": self.carry_min_vel_theta,
                    "acc_lim_x": self.carry_accel,
                    "acc_lim_theta": self.carry_turn_accel,
                })
                if (float(updated.get("max_vel_x", 1e9)) > self.carry_speed + 1e-3
                        or float(updated.get("max_vel_theta", 1e9))
                        > self.carry_turn_speed + 1e-3
                        or float(updated.get("min_vel_trans", 1e9))
                        > self.carry_min_vel_trans + 1e-3
                        or float(updated.get("min_vel_theta", 1e9))
                        > self.carry_min_vel_theta + 1e-3):
                    raise RuntimeError("DWA rejected carrying limits")
                rospy.logwarn(
                    "[MineMission] loaded DWA profile active: vx=%.2f, "
                    "wz=%.2f, min_trans=%.2f, min_theta=%.2f, "
                    "acc_x=%.2f, acc_theta=%.2f",
                    float(updated.get("max_vel_x", self.carry_speed)),
                    float(updated.get("max_vel_theta", self.carry_turn_speed)),
                    float(updated.get("min_vel_trans", self.carry_min_vel_trans)),
                    float(updated.get("min_vel_theta", self.carry_min_vel_theta)),
                    float(updated.get("acc_lim_x", self.carry_accel)),
                    float(updated.get("acc_lim_theta", self.carry_turn_accel)),
                )
                self.carry_speed_active = True
            else:
                if self.normal_dwa_config:
                    self.dwa_client.update_configuration(self.normal_dwa_config)
                self.carry_speed_active = False
            return True
        except Exception as exc:
            rospy.logerr("[MineMission] DWA carrying limit update failed: %s", exc)
            if enabled and self.require_carry_speed_limit:
                return False
            self.carry_speed_active = bool(enabled)
            return True

    def _carry_attitude_ok(self) -> Tuple[bool, str]:
        with self.lock:
            imu = copy.deepcopy(self.imu)
        if imu is None:
            return False, "IMU unavailable while carrying"
        q = imu.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        if abs(roll) > self.carry_max_roll or abs(pitch) > self.carry_max_pitch:
            return False, "BASE_UNSTABLE roll={:.2f} pitch={:.2f} deg".format(
                math.degrees(roll), math.degrees(pitch)
            )
        return True, ""

    def _arm_feedback_cb(self, feedback) -> None:
        with self.lock:
            if self.current_mine_id is None:
                return
            task = self.tasks.get(self.current_mine_id)
            if task is None or task.state not in (
                MineMissionEntry.WAITING_ARM, MineMissionEntry.PLACING
            ):
                return
            task.detail = (
                f"arm stage={feedback.stage} {feedback.progress * 100.0:.0f}% "
                f"{feedback.detail}"
            ).strip()
            task.touch()
        self._publish_all(persist=False)

    def _navigation_round_failed(self, mine_id: int, reason: str) -> None:
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None or task.state in TERMINAL_STATES:
                return
            if task.retry_round < self.retry_limit:
                task.retry_round += 1
                task.state = MineMissionEntry.DEFERRED
                task.eligible_after_wall = time.monotonic() + self.retry_delay
                task.detail = reason + "; retry after first pass"
            else:
                task.state = MineMissionEntry.UNREACHABLE
                task.detail = reason + "; manual route required"
            task.touch()
            if self.current_mine_id == mine_id:
                self.current_mine_id = None
        self._publish_all()

    def _defer_empty_pick(self, mine_id: int, reason: str) -> None:
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None or task.state == MineMissionEntry.CLEARED:
                return
            task.retry_round += 1
            task.state = MineMissionEntry.DEFERRED
            task.eligible_after_wall = time.monotonic() + self.retry_delay
            task.detail = (
                reason
                + "; no physical grasp was established, arm verified at look; "
                "deferred while later mines run, next attempt must use a new "
                "approach direction"
            )
            task.frozen_mine_pose = None
            task.touch()
            self.current_mine_id = None
        self._publish_all()

    def _arm_round_failed(self, mine_id: int, reason: str) -> None:
        with self.lock:
            task = self.tasks.get(mine_id)
            if task is None or task.state == MineMissionEntry.CLEARED:
                return
            # This path is only for retained objects or unverified/failed arm
            # recovery. Empty failures that reached look are handled by
            # _defer_empty_pick and never globally inhibit later mines.
            if self.mine_retained:
                task.state = MineMissionEntry.MANUAL_REQUIRED
                task.detail = (
                    reason
                    + "; physical grasp lock still active; base locked for recovery"
                )
            else:
                task.state = MineMissionEntry.MANUAL_REQUIRED
                task.detail = (
                    reason
                    + "; automatic recovery did not prove the arm is at look; "
                    "source mine remains hazardous and base is locked"
                )
            task.touch()
            self.current_mine_id = None
        self._publish_all()

    # --------------------------------------------------------------- ROS helpers
    def _robot_pose(self) -> Optional[PoseStamped]:
        return self._frame_pose(self.base_frame)

    def _frame_pose(
        self, frame: str, timeout: float = 0.5
    ) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame,
                rospy.Time(0),
                rospy.Duration(max(0.0, float(timeout))),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "[MineMission] TF %s unavailable: %s", frame, exc
            )
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _wait_for_action_server(client, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                action_client = client.action_client
                status = action_client.last_status_msg
                if status is not None:
                    server_id = status._connection_header.get("callerid")
                    if server_id:
                        goal_ok = action_client.pub_goal.impl.has_connection(server_id)
                        cancel_ok = action_client.pub_cancel.impl.has_connection(server_id)
                        result_ok = any(
                            connection.callerid_pub == server_id
                            for connection in action_client.result_sub.impl.connections
                        )
                        feedback_ok = any(
                            connection.callerid_pub == server_id
                            for connection in action_client.feedback_sub.impl.connections
                        )
                        if goal_ok and cancel_ok and result_ok and feedback_ok:
                            return True
            except (AttributeError, KeyError, TypeError):
                pass
            time.sleep(0.02)
        return False

    def _wait_for_plan_service(self, timeout: float) -> bool:
        try:
            rospy.wait_for_service(self.make_plan_name, timeout=timeout)
            return True
        except rospy.ROSException:
            return False

    def _set_base_lock(self, locked: bool) -> None:
        self.base_lock_pub.publish(Bool(data=bool(locked)))

    # ---------------------------------------------------------- status / markers
    def _entry_from_task(self, task: Task) -> MineMissionEntry:
        entry = MineMissionEntry()
        entry.id = task.mine_id
        entry.map_revision = int(task.map_revision)
        entry.frozen_map_revision = int(task.frozen_map_revision)
        entry.mine_pose = copy.deepcopy(task.mine_pose)
        if task.approach_pose is not None:
            entry.approach_pose = copy.deepcopy(task.approach_pose)
        else:
            entry.approach_pose.header.frame_id = self.map_frame
            entry.approach_pose.pose.orientation.w = 1.0
        if task.dropoff_pose is not None:
            entry.dropoff_pose = copy.deepcopy(task.dropoff_pose)
        else:
            entry.dropoff_pose.header.frame_id = self.map_frame
            entry.dropoff_pose.pose.orientation.w = 1.0
        if task.drop_pose is not None:
            entry.drop_pose = copy.deepcopy(task.drop_pose)
        else:
            entry.drop_pose.header.frame_id = self.map_frame
            entry.drop_pose.pose.orientation.w = 1.0
        entry.disposal_slot = max(0, task.disposal_slot)
        entry.state = task.state
        entry.navigation_attempts = task.navigation_attempts
        entry.grasp_attempts = task.grasp_attempts
        entry.detail = task.detail
        entry.updated_at = task.updated_at or rospy.Time.now()
        return entry

    def _publish_all(self, persist: bool = True) -> None:
        with self.lock:
            self.revision += 1
            stamp = rospy.Time.now()
            status = MineMission()
            status.header = Header(stamp=stamp, frame_id=self.map_frame)
            status.revision = self.revision
            status.epoch = self.epoch
            status.current_mine_id = self.current_mine_id or 0
            status.running = self.running
            status.paused = self.paused
            status.survey_complete = self.survey_complete
            if self.home_pose is not None:
                status.home_pose = copy.deepcopy(self.home_pose)
            else:
                status.home_pose.header.frame_id = self.map_frame
                status.home_pose.pose.orientation.w = 1.0
            status.disposed_mines.header = status.header
            status.disposed_mines.poses = [
                copy.deepcopy(self.disposed_poses[mine_id])
                for mine_id in sorted(self.disposed_poses)
            ]
            status.entries = [
                self._entry_from_task(self.tasks[mine_id])
                for mine_id in sorted(self.tasks)
            ]

            if self.current_mine_id in self.tasks:
                current = self._entry_from_task(self.tasks[self.current_mine_id])
            else:
                current = MineMissionEntry()
                current.mine_pose.header.frame_id = self.map_frame
                current.approach_pose.header.frame_id = self.map_frame
                current.detail = "no active task"

            active = PoseArray()
            active.header = status.header
            active.poses = [
                copy.deepcopy(task.mine_pose.pose)
                for task in self.tasks.values()
                if task.state not in (
                    MineMissionEntry.CLEARED,
                    MineMissionEntry.CARRYING,
                    MineMissionEntry.RETURNING_HOME,
                    MineMissionEntry.AT_DROPOFF,
                    MineMissionEntry.PLACING,
                )
            ]
            for mine_id, task in self.tasks.items():
                if task.state in (
                    MineMissionEntry.CARRYING,
                    MineMissionEntry.RETURNING_HOME,
                    MineMissionEntry.AT_DROPOFF,
                    MineMissionEntry.PLACING,
                    MineMissionEntry.CLEARED,
                ):
                    self.suppressed_detection_ids.add(int(mine_id))
            suppressed_ids = UInt32MultiArray(
                data=sorted(self.suppressed_detection_ids)
            )
            markers = self._build_markers_locked(stamp)
            mine_map_received = self.last_mine_map is not None
            has_unconfirmed_candidates = bool(
                self.last_mine_map is not None
                and any(not mine.confirmed for mine in self.last_mine_map.mines)
            )
            all_confirmed_cleared = bool(
                status.survey_complete
                and status.entries
                and all(
                    entry.state == MineMissionEntry.CLEARED
                    for entry in status.entries
                )
                and mine_map_received
            )

        # Publish the destination hazard before the CLEARED status removes the
        # source hazard.  Cross-topic delivery is not transactional, so the
        # base remains locked through a short settling window after PLACE.
        self.disposed_pub.publish(status.disposed_mines)
        self.suppressed_ids_pub.publish(suppressed_ids)
        self.status_pub.publish(status)
        self.current_pub.publish(current)
        self.active_hazards_pub.publish(active)
        self.carrying_pub.publish(Bool(data=any(
            entry.state in (
                MineMissionEntry.CARRYING,
                MineMissionEntry.RETURNING_HOME,
                MineMissionEntry.AT_DROPOFF,
                MineMissionEntry.PLACING,
            ) for entry in status.entries
        )))
        # Candidates have not passed the confirmation contract and therefore
        # are not dispatchable "known mines". Keep reporting them in the map,
        # but never let a persistent yellow fragment permanently veto completion
        # after every confirmed physical task has been placed.
        if all_confirmed_cleared and has_unconfirmed_candidates:
            rospy.logwarn_throttle(
                10.0,
                "[MineMission] all confirmed tasks cleared; residual "
                "unconfirmed detections remain diagnostic-only",
            )
        self.all_cleared_pub.publish(Bool(data=all_confirmed_cleared))
        self.markers_pub.publish(markers)
        if persist:
            self._persist(status)

    def _build_markers_locked(self, stamp: rospy.Time) -> MarkerArray:
        result = MarkerArray()
        clear = Marker()
        clear.header = Header(stamp=stamp, frame_id=self.map_frame)
        clear.action = Marker.DELETEALL
        result.markers.append(clear)
        marker_id = 1
        for mine_id in sorted(self.tasks):
            task = self.tasks[mine_id]
            if mine_id in self.suppressed_detection_ids:
                continue
            sphere = Marker()
            sphere.header = Header(stamp=stamp, frame_id=self.map_frame)
            sphere.ns = "mine_mission"
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.CYLINDER
            sphere.action = Marker.ADD
            sphere.pose = copy.deepcopy(task.mine_pose.pose)
            sphere.pose.position.z = max(0.04, sphere.pose.position.z)
            sphere.scale.x = sphere.scale.y = 0.50
            sphere.scale.z = 0.08
            sphere.color.a = 0.8
            if task.state == MineMissionEntry.CLEARED:
                sphere.color.g = 0.9
            elif task.state in (
                MineMissionEntry.MANUAL_REQUIRED,
                MineMissionEntry.UNREACHABLE,
            ):
                sphere.color.r, sphere.color.b = 1.0, 0.8
            elif mine_id == self.current_mine_id:
                sphere.color.r, sphere.color.g = 1.0, 0.65
            else:
                sphere.color.r = 1.0
            result.markers.append(sphere)

            text = Marker()
            text.header = sphere.header
            text.ns = "mine_mission_text"
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = copy.deepcopy(task.mine_pose.pose)
            text.pose.position.z += 0.45
            text.scale.z = 0.28
            text.color.a = 1.0
            text.color.r = text.color.g = text.color.b = 1.0
            text.text = f"M{mine_id:03d} {STATE_NAMES.get(task.state, task.state)}"
            result.markers.append(text)

            if task.approach_pose is not None and mine_id == self.current_mine_id:
                arrow = Marker()
                arrow.header = sphere.header
                arrow.ns = "mine_approach"
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose = copy.deepcopy(task.approach_pose.pose)
                arrow.pose.position.z = 0.12
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.55, 0.10, 0.10
                arrow.color.a, arrow.color.g, arrow.color.b = 1.0, 0.8, 1.0
                result.markers.append(arrow)
        if self.home_pose is not None:
            home = Marker()
            home.header = Header(stamp=stamp, frame_id=self.map_frame)
            home.ns = "mine_home"
            home.id = marker_id
            marker_id += 1
            home.type = Marker.ARROW
            home.action = Marker.ADD
            home.pose = copy.deepcopy(self.home_pose.pose)
            home.pose.position.z += 0.15
            home.scale.x, home.scale.y, home.scale.z = 0.8, 0.16, 0.16
            home.color.a, home.color.g, home.color.b = 1.0, 1.0, 1.0
            result.markers.append(home)
        for mine_id in sorted(self.disposed_poses):
            marker = Marker()
            marker.header = Header(stamp=stamp, frame_id=self.map_frame)
            marker.ns = "mine_disposal"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = copy.deepcopy(self.disposed_poses[mine_id])
            marker.pose.position.z = max(0.04, marker.pose.position.z)
            marker.scale.x = marker.scale.y = 0.22
            marker.scale.z = 0.08
            marker.color.a, marker.color.r, marker.color.g = 0.9, 0.7, 0.2
            result.markers.append(marker)
        return result

    # --------------------------------------------------------------- persistence
    def _persist(self, status: MineMission) -> None:
        data = {
            "schema": 2,
            "frame_id": self.map_frame,
            "epoch": status.epoch,
            "revision": status.revision,
            "running": status.running,
            "paused": status.paused,
            "home": None if self.home_pose is None else {
                "x": float(self.home_pose.pose.position.x),
                "y": float(self.home_pose.pose.position.y),
                "z": float(self.home_pose.pose.position.z),
                "ground_z": self.home_ground_z,
                "qx": float(self.home_pose.pose.orientation.x),
                "qy": float(self.home_pose.pose.orientation.y),
                "qz": float(self.home_pose.pose.orientation.z),
                "qw": float(self.home_pose.pose.orientation.w),
            },
            "tasks": [],
        }
        with self.lock:
            for task in self.tasks.values():
                data["tasks"].append(
                    {
                        "id": task.mine_id,
                        "map_revision": task.map_revision,
                        "frozen_map_revision": task.frozen_map_revision,
                        "x": float(task.mine_pose.pose.position.x),
                        "y": float(task.mine_pose.pose.position.y),
                        "z": float(task.mine_pose.pose.position.z),
                        "state": STATE_NAMES.get(task.state, str(task.state)),
                        "state_code": int(task.state),
                        "navigation_attempts": task.navigation_attempts,
                        "grasp_attempts": task.grasp_attempts,
                        "disposal_slot": task.disposal_slot,
                        "drop_x": None if task.drop_pose is None else float(
                            task.drop_pose.pose.position.x
                        ),
                        "drop_y": None if task.drop_pose is None else float(
                            task.drop_pose.pose.position.y
                        ),
                        "drop_z": None if task.drop_pose is None else float(
                            task.drop_pose.pose.position.z
                        ),
                        "retry_round": task.retry_round,
                        "excluded_approach_directions": sorted(
                            task.excluded_approach_directions
                        ),
                        "detail": task.detail,
                    }
                )
        try:
            directory = os.path.dirname(self.persistence_file) or "."
            os.makedirs(directory, exist_ok=True)
            fd, temp_path = tempfile.mkstemp(prefix=".mine_mission_", dir=directory)
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
            os.replace(temp_path, self.persistence_file)
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0, "[MineMission] cannot save %s: %s", self.persistence_file, exc
            )

    def _load_resume_records(self) -> None:
        try:
            with open(self.persistence_file, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
            self.resume_records = list(data.get("tasks", []))
            home = data.get("home")
            if home:
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.pose.position.x = float(home.get("x", 0.0))
                pose.pose.position.y = float(home.get("y", 0.0))
                pose.pose.position.z = float(home.get("z", 0.0))
                pose.pose.orientation.x = float(home.get("qx", 0.0))
                pose.pose.orientation.y = float(home.get("qy", 0.0))
                pose.pose.orientation.z = float(home.get("qz", 0.0))
                pose.pose.orientation.w = float(home.get("qw", 1.0))
                self.home_pose = pose
                self.home_ground_z = float(home.get("ground_z", 0.0))
            rospy.logwarn(
                "[MineMission] real-world resume enabled: loaded %d task records from %s",
                len(self.resume_records),
                self.persistence_file,
            )
        except FileNotFoundError:
            rospy.loginfo("[MineMission] no resume file at %s", self.persistence_file)
        except Exception as exc:
            rospy.logerr("[MineMission] failed to read resume file: %s", exc)

    def _apply_resume_record_locked(self, task: Task) -> None:
        if not self.resume_records:
            return
        p = task.mine_pose.pose.position
        matches = [
            record
            for record in self.resume_records
            if int(record.get("id", -1)) == task.mine_id
            or math.hypot(
                float(record.get("x", 1e9)) - p.x,
                float(record.get("y", 1e9)) - p.y,
            )
            <= self.resume_match_radius
        ]
        if not matches:
            return
        record = min(
            matches,
            key=lambda item: math.hypot(
                float(item.get("x", 1e9)) - p.x,
                float(item.get("y", 1e9)) - p.y,
            ),
        )
        state = int(record.get("state_code", MineMissionEntry.PENDING))
        # In-progress states cannot safely resume mid-action. Retry from pending.
        if state in (
            MineMissionEntry.NAVIGATING,
            MineMissionEntry.AT_STANDOFF,
            MineMissionEntry.WAITING_ARM,
            MineMissionEntry.WAITING_FOR_MAP,
        ):
            state = MineMissionEntry.PENDING
        if state in (
            MineMissionEntry.CARRYING,
            MineMissionEntry.RETURNING_HOME,
            MineMissionEntry.AT_DROPOFF,
            MineMissionEntry.PLACING,
        ):
            # A process restart cannot prove that the physical grasp-fix joint
            # still exists. Never resume motion carrying an unverified load.
            state = MineMissionEntry.MANUAL_REQUIRED
        task.state = state
        task.navigation_attempts = int(record.get("navigation_attempts", 0))
        task.grasp_attempts = int(record.get("grasp_attempts", 0))
        task.disposal_slot = int(record.get("disposal_slot", -1))
        if state == MineMissionEntry.CLEARED and record.get("drop_x") is not None:
            drop = PoseStamped()
            drop.header.frame_id = self.map_frame
            drop.pose.position.x = float(record["drop_x"])
            drop.pose.position.y = float(record["drop_y"])
            drop.pose.position.z = float(record.get("drop_z") or 0.055)
            drop.pose.orientation.w = 1.0
            task.drop_pose = drop
            self.disposed_poses[task.mine_id] = copy.deepcopy(drop.pose)
            self.next_disposal_slot = max(
                self.next_disposal_slot, task.disposal_slot + 1
            )
        task.retry_round = int(record.get("retry_round", 0))
        task.map_revision = int(record.get("map_revision", task.map_revision))
        task.frozen_map_revision = int(
            record.get("frozen_map_revision", 0)
        )
        task.excluded_approach_directions = {
            int(value) for value in record.get(
                "excluded_approach_directions", []
            )
            if 0 <= int(value) < self.candidate_count
        }
        task.detail = "resumed: " + str(record.get("detail", ""))
        task.touch()

    def _on_shutdown(self) -> None:
        with self.condition:
            self.shutdown = True
            self.condition.notify_all()
        self._cancel_navigation_goal_if_active()
        self.grasp_client.cancel_goal()
        self._set_base_lock(True)


def main() -> None:
    rospy.init_node("mine_mission_manager")
    MineMissionManager()
    rospy.spin()


if __name__ == "__main__":
    main()
