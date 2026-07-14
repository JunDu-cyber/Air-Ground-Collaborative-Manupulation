#!/usr/bin/env python3
"""Regression coverage for the physical-release half of PLACE."""

import importlib.util
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "mine_grasp_executor.py"
SPEC = importlib.util.spec_from_file_location("mine_grasp_executor_place", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
GraspFailure = MODULE.GraspFailure
MineGraspExecutor = MODULE.MineGraspExecutor
MotionExecutionResult = MODULE.MotionExecutionResult


class PlaceReleaseInterlockTest(unittest.TestCase):
    def _executor(self):
        executor = MineGraspExecutor.__new__(MineGraspExecutor)
        executor.lock = threading.RLock()
        executor.execution_lock = threading.Lock()
        executor.releasing = False
        executor.release_motion_guard = False
        executor.retained = True
        executor.gazebo_attached = True
        executor.gripper_open = 0.08
        executor.gripper_duration = 4.0
        executor.place_speed = 0.02
        executor.place_retreat_distance = 0.12
        executor.moveit_object_id = "detected_landmine"
        executor.look_joints = [0.0] * 6
        executor.reset_duration = 15.0
        executor.action_active = False

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.orientation.w = 1.0
        executor._wait_interfaces = lambda **_kwargs: None
        executor._check_controllers = lambda: None
        executor._wait_base_stationary = lambda: None
        executor._verify_attached_scene = lambda: None
        executor._select_place_candidate = lambda _target: {
            "pregrasp_joints": [0.0] * 6,
            "pregrasp": pose,
            "grasp": pose,
        }
        success = MotionExecutionResult(True, "SUCCESS", "ok", {})
        executor._move_group_joints = lambda *_args, **_kwargs: success
        executor._cartesian_to = lambda *_args, **_kwargs: success
        executor._command_gripper = lambda *_args, **_kwargs: success
        executor._command_arm = lambda *_args, **_kwargs: success
        executor._verify_transport_lock = lambda _duration: None
        executor._detach_moveit_object = lambda: True
        executor._remove_world_object = lambda _object_id: None
        executor._verify_transient_scene_cleared = lambda: None
        executor._world_vertical_offset = lambda command, _distance: command
        executor._cancel_all_motion = lambda: None

        events = []
        executor._publish_status = (
            lambda stage, code="RUNNING", extra=None:
            events.append(("status", stage, code))
        )
        executor._set_retained = lambda value: (
            events.append(("retained", bool(value))),
            setattr(executor, "retained", bool(value)),
        )[-1]
        executor._verify_place_validation = lambda _target: events.append(
            ("validation", executor.retained, executor.gazebo_attached)
        )

        goal = SimpleNamespace(drop_pose=PoseStamped())
        goal.drop_pose.header.frame_id = "map"
        goal.drop_pose.pose.orientation.w = 1.0
        return executor, goal, events

    def test_retained_clears_only_after_final_detach_and_validation(self):
        executor, goal, events = self._executor()
        release_checks = []

        def wait_released(enforce_safety):
            del enforce_safety
            executor.gazebo_attached = False
            release_checks.append(executor.retained)
            events.append(("released", executor.retained))

        executor._wait_gazebo_released = wait_released

        success, _detail = executor._place_goal(goal)

        self.assertTrue(success)
        self.assertEqual(release_checks, [True, True])
        validations = [event for event in events if event[0] == "validation"]
        self.assertEqual(validations, [
            ("validation", True, False),
            ("validation", True, False),
        ])
        retained_false = events.index(("retained", False))
        second_validation = max(
            index for index, event in enumerate(events)
            if event[0] == "validation"
        )
        complete = events.index(("status", "PLACE_COMPLETE", "PLACE_SUCCESS"))
        self.assertGreater(retained_false, second_validation)
        self.assertGreater(complete, retained_false)
        self.assertFalse(executor.retained)
        self.assertFalse(executor.release_motion_guard)

    def test_reattach_during_retreat_aborts_and_restores_retention(self):
        executor, goal, events = self._executor()
        executor._wait_gazebo_released = (
            lambda enforce_safety: (
                enforce_safety,
                setattr(executor, "gazebo_attached", False),
            )[-1]
        )

        cartesian_calls = []

        def cartesian(_pose, _speed, stage):
            cartesian_calls.append(stage)
            if stage == "PLACE_RETREAT":
                executor.gazebo_attached = True
                executor._safety_check()
            return MotionExecutionResult(True, "SUCCESS", "ok", {})

        executor._cartesian_to = cartesian
        executor._publish_zero = lambda: None
        executor._sample_attitude = lambda: (0.0, 0.0)

        success, detail = executor._place_goal(goal)

        self.assertFalse(success)
        self.assertIn("GAZEBO_ATTACH_FAILED", detail)
        self.assertIn("PLACE_RETREAT", cartesian_calls)
        self.assertTrue(executor.retained)
        self.assertIn(("retained", True), events)
        self.assertNotIn(
            ("status", "PLACE_COMPLETE", "PLACE_SUCCESS"), events
        )

    def test_release_guard_stops_motion_on_physical_reattach(self):
        executor, _goal, _events = self._executor()
        executor.release_motion_guard = True
        executor.gazebo_attached = True
        cancelled = []
        executor._publish_zero = lambda: None
        executor._cancel_all_motion = lambda: cancelled.append(True)
        executor._sample_attitude = lambda: self.fail(
            "attitude gate must not run after a release reattach"
        )

        with self.assertRaises(GraspFailure) as raised:
            executor._safety_check()

        self.assertEqual(raised.exception.code, "GAZEBO_ATTACH_FAILED")
        self.assertEqual(cancelled, [True])


if __name__ == "__main__":
    unittest.main()
