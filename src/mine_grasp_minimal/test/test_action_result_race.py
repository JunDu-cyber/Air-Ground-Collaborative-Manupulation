#!/usr/bin/env python3
"""Regression tests for status-before-result actionlib delivery."""

import importlib.util
import time
import unittest
from pathlib import Path

from actionlib_msgs.msg import GoalStatus


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mine_grasp_executor.py"
SPEC = importlib.util.spec_from_file_location("mine_grasp_executor", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
MineGraspExecutor = MODULE.MineGraspExecutor
MotionExecutionResult = MODULE.MotionExecutionResult


class DelayedResultClient:
    def __init__(self, delay):
        self.started = time.monotonic()
        self.delay = float(delay)
        self.cancelled = False

    def get_state(self):
        return GoalStatus.SUCCEEDED

    def get_result(self):
        return object() if time.monotonic() - self.started >= self.delay else None

    def cancel_goal(self):
        self.cancelled = True


class NonTerminalClient:
    def __init__(self):
        self.cancelled = False

    def get_state(self):
        return GoalStatus.ACTIVE

    def get_result(self):
        return None

    def cancel_goal(self):
        self.cancelled = True


class MissingResultClient:
    def __init__(self, state):
        self.state = int(state)
        self.goal = None

    def send_goal(self, goal):
        self.goal = goal

    def get_state(self):
        return self.state

    def get_result(self):
        return None


class ActionResultRaceTest(unittest.TestCase):
    def setUp(self):
        # Message construction in the command-path tests stamps trajectories;
        # enable rospy's wall-clock mode without starting a ROS master/node.
        MODULE.rospy.rostime.set_rostime_initialized(True)
        self.executor = MineGraspExecutor.__new__(MineGraspExecutor)
        self.executor.action_wall_timeout_scale = 1.0
        self.executor.action_result_grace = 0.5
        self.executor._publish_zero = lambda: None
        self.executor._safety_check = lambda: None

    def test_terminal_status_waits_for_delayed_result_payload(self):
        client = DelayedResultClient(0.15)
        started = time.monotonic()

        self.assertTrue(
            self.executor._wait_action(
                client, timeout=0.05, enforce_safety=False
            )
        )

        self.assertGreaterEqual(time.monotonic() - started, 0.14)
        self.assertIsNotNone(client.get_result())
        self.assertFalse(client.cancelled)

    def test_nonterminal_action_still_obeys_wall_timeout(self):
        client = NonTerminalClient()

        self.assertFalse(
            self.executor._wait_action(
                client, timeout=0.05, enforce_safety=False
            )
        )

        self.assertTrue(client.cancelled)

    def test_missing_result_fallback_is_limited_to_controller_outcomes(self):
        self.assertTrue(
            self.executor._missing_result_allows_measured_settle(
                GoalStatus.SUCCEEDED
            )
        )
        self.assertTrue(
            self.executor._missing_result_allows_measured_settle(
                GoalStatus.ABORTED
            )
        )
        for state in (
            GoalStatus.REJECTED, GoalStatus.PREEMPTED,
            GoalStatus.RECALLED, GoalStatus.LOST,
        ):
            self.assertFalse(
                self.executor._missing_result_allows_measured_settle(state)
            )

    def test_direct_arm_missing_result_uses_measured_stability_gate(self):
        client = MissingResultClient(GoalStatus.SUCCEEDED)
        self.executor.arm_client = client
        self.executor.arm_joint_names = ["joint"]
        self.executor.terminal_joint_error = 0.01
        self.executor.timeout_margin = 1.0
        self.executor.explicit_goal_tolerance = 0.01
        self.executor.metrics = {}
        self.executor._fk_pose_for_joints = lambda _positions: object()
        self.executor._joint_error = lambda _names, _positions: 1.0
        self.executor._wait_action = lambda *_args, **_kwargs: True
        self.executor._record_motion_result = lambda _stage, result: result
        self.executor._amend_latest_motion_result = lambda _stage, result: result
        measured = MotionExecutionResult(
            True, "SUCCESS", "measured terminal state is stable", {}
        )
        self.executor._wait_motion_stable = lambda *_args: measured

        result = self.executor._command_arm(
            [0.2], 1.0, enforce_safety=False, stage="ARM_TEST"
        )

        self.assertTrue(result)
        self.assertTrue(result.measurements["action_result_payload_missing"])
        self.assertIsNotNone(client.goal)

    def test_gripper_missing_result_uses_measured_stability_gate(self):
        client = MissingResultClient(GoalStatus.ABORTED)
        self.executor.gripper_client = client
        self.executor.gripper_joint = "finger_joint"
        self.executor.timeout_margin = 1.0
        self.executor.explicit_goal_tolerance = 0.01
        self.executor._wait_action = lambda *_args, **_kwargs: True
        self.executor._record_motion_result = lambda _stage, result: result
        measured = MotionExecutionResult(
            True, "SUCCESS", "measured gripper state is stable", {}
        )
        self.executor._wait_gripper_stable = lambda *_args: measured

        result = self.executor._command_gripper(
            0.08, 1.0, enforce_safety=False, stage="GRIPPER_TEST"
        )

        self.assertTrue(result)
        self.assertTrue(result.measurements["action_result_payload_missing"])
        self.assertIsNotNone(client.goal)


if __name__ == "__main__":
    unittest.main()
