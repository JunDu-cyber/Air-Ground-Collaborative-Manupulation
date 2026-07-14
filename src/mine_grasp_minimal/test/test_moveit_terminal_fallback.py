#!/usr/bin/env python3
"""Regression coverage for bounded MoveIt execution-failure recovery."""

import importlib.util
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mine_grasp_executor.py"
SPEC = importlib.util.spec_from_file_location("mine_grasp_executor_moveit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
MineGraspExecutor = MODULE.MineGraspExecutor
MotionExecutionResult = MODULE.MotionExecutionResult


class TerminalClient:
    def __init__(self, state, error_code):
        self.state = int(state)
        self.result = (
            None if error_code is None
            else SimpleNamespace(
                error_code=SimpleNamespace(val=int(error_code))
            )
        )
        self.goal = None

    def send_goal(self, goal):
        self.goal = goal

    def get_state(self):
        return self.state

    def get_result(self):
        return self.result


class MoveItTerminalFallbackTest(unittest.TestCase):
    def setUp(self):
        MODULE.rospy.rostime.set_rostime_initialized(True)
        self.executor = MineGraspExecutor.__new__(MineGraspExecutor)

    def test_only_execution_failures_and_missing_payload_are_eligible(self):
        for code in (
            MoveItErrorCodes.CONTROL_FAILED,
            MoveItErrorCodes.TIMED_OUT,
            None,
        ):
            for state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED):
                self.assertTrue(
                    self.executor._moveit_failure_allows_measured_settle(
                        code, state
                    )
                )

        for code in (
            MoveItErrorCodes.PLANNING_FAILED,
            MoveItErrorCodes.GOAL_IN_COLLISION,
            MoveItErrorCodes.INVALID_MOTION_PLAN,
            MoveItErrorCodes.SUCCESS,
        ):
            self.assertFalse(
                self.executor._moveit_failure_allows_measured_settle(
                    code, GoalStatus.ABORTED
                )
            )

        for state in (
            GoalStatus.REJECTED,
            GoalStatus.PREEMPTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        ):
            self.assertFalse(
                self.executor._moveit_failure_allows_measured_settle(
                    MoveItErrorCodes.CONTROL_FAILED, state
                )
            )

    def test_eligible_failure_invokes_strict_measured_gate(self):
        measured = MotionExecutionResult(
            True, "SUCCESS", "measured terminal state is stable", {}
        )
        calls = []
        self.executor.metrics = {}
        self.executor.last_arm_fjt_result_wall = time.monotonic()
        self.executor._wait_motion_stable = (
            lambda stage, joints, tcp: (
                calls.append((stage, joints, tcp)) or measured
            )
        )
        self.executor._amend_latest_motion_result = (
            lambda _stage, result: result
        )
        target = PoseStamped()
        target.header.frame_id = "base_link"

        result = self.executor._moveit_terminal_failure_settled(
            MoveItErrorCodes.TIMED_OUT,
            GoalStatus.ABORTED,
            None,
            time.monotonic(),
            "PREGRASP_TEST",
            [0.2],
            target,
        )

        self.assertTrue(result)
        self.assertEqual(len(calls), 1)
        self.assertTrue(
            result.measurements["accepted_moveit_terminal_failure"]
        )
        self.assertEqual(
            result.measurements["moveit_error_code"],
            MoveItErrorCodes.TIMED_OUT,
        )

    def test_planning_or_preempt_failure_never_invokes_measured_gate(self):
        calls = []
        self.executor._wait_motion_stable = (
            lambda *_args: calls.append(True)
        )
        target = PoseStamped()
        for code, state in (
            (MoveItErrorCodes.PLANNING_FAILED, GoalStatus.ABORTED),
            (MoveItErrorCodes.GOAL_IN_COLLISION, GoalStatus.ABORTED),
            (MoveItErrorCodes.CONTROL_FAILED, GoalStatus.PREEMPTED),
            (MoveItErrorCodes.TIMED_OUT, GoalStatus.LOST),
        ):
            self.assertIsNone(
                self.executor._moveit_terminal_failure_settled(
                    code, state, None, time.monotonic(), "DENIED",
                    [0.2], target,
                )
            )
        self.assertEqual(calls, [])

    def test_move_group_joint_path_routes_control_failure_to_gate(self):
        client = TerminalClient(
            GoalStatus.ABORTED, MoveItErrorCodes.CONTROL_FAILED
        )
        executor = self.executor
        executor.move_group_client = client
        executor.arm_group = "ur5_arm"
        executor.planning_attempts = 1
        executor.planning_time = 1.0
        executor.velocity_scale = 0.1
        executor.acceleration_scale = 0.1
        executor.arm_joint_names = ["joint"]
        executor.joint_tolerance = 0.002
        executor.last_moveit_error_code = None
        executor._wait_action = lambda *_args, **_kwargs: True
        executor._recent_arm_controller_result = lambda _started: None
        routed = []
        expected = MotionExecutionResult(True, "SUCCESS", "accepted", {})
        executor._moveit_terminal_failure_settled = (
            lambda *args: routed.append(args) or expected
        )
        target = PoseStamped()
        target.header.frame_id = "base_link"

        result = executor._move_group_joints(
            [0.2], target_pose=target, stage="MOVE_PREGRASP_TEST"
        )

        self.assertIs(result, expected)
        self.assertEqual(len(routed), 1)
        self.assertEqual(routed[0][0], MoveItErrorCodes.CONTROL_FAILED)
        self.assertEqual(routed[0][1], GoalStatus.ABORTED)

    def test_cartesian_path_routes_timeout_to_gate_after_full_plan(self):
        client = TerminalClient(
            GoalStatus.ABORTED, MoveItErrorCodes.TIMED_OUT
        )
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = ["joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.2]
        point.time_from_start = MODULE.rospy.Duration(1.0)
        trajectory.joint_trajectory.points = [point]
        response = SimpleNamespace(
            error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            fraction=1.0,
            solution=trajectory,
        )
        executor = self.executor
        executor.last_cartesian_error = ""
        executor.last_cartesian_fraction = None
        executor.last_moveit_error_code = None
        executor.metrics = {}
        executor.arm_group = "ur5_arm"
        executor.tcp_link = "grasp_tcp"
        executor.cartesian_step = 0.005
        executor.min_cartesian_fraction = 0.995
        executor.timeout_margin = 1.0
        executor.cartesian_service = lambda _request: response
        executor.execute_client = client
        executor._ensure_slow_trajectory = lambda *_args: None
        executor._trajectory_arm_endpoint = lambda _trajectory: [0.2]
        executor._wait_action = lambda *_args, **_kwargs: True
        executor._recent_arm_controller_result = lambda _started: None
        routed = []
        expected = MotionExecutionResult(True, "SUCCESS", "accepted", {})
        executor._moveit_terminal_failure_settled = (
            lambda *args: routed.append(args) or expected
        )
        target = PoseStamped()
        target.header.frame_id = "base_link"
        target.pose.orientation.w = 1.0

        result = executor._cartesian_to(
            target, 0.02, stage="FINAL_APPROACH_TEST"
        )

        self.assertIs(result, expected)
        self.assertEqual(len(routed), 1)
        self.assertEqual(routed[0][0], MoveItErrorCodes.TIMED_OUT)
        self.assertEqual(routed[0][1], GoalStatus.ABORTED)
        self.assertEqual(executor.last_cartesian_fraction, 1.0)

    def test_moveit_goal_duration_margin_is_persisted(self):
        moveit_root = Path(__file__).resolve().parents[2] / (
            "husky_ur5_moveit_config/launch"
        )
        for name in ("move_group.launch", "trajectory_execution.launch.xml"):
            text = (moveit_root / name).read_text(encoding="utf-8")
            self.assertIn(
                'name="allowed_goal_duration_margin" default="4.0"', text
            )


if __name__ == "__main__":
    unittest.main()
