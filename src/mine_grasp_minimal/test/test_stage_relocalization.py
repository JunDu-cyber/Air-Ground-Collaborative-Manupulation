#!/usr/bin/env python3
"""Regression coverage for stage-atomic wrist relocalization."""

import collections
import importlib.util
import math
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped


ROOT = Path(__file__).resolve().parents[1]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EXECUTOR_MODULE = _load(
    "mine_grasp_executor_stage_relocalization",
    ROOT / "scripts" / "mine_grasp_executor.py",
)
LOCALIZER_MODULE = _load(
    "wrist_mine_localizer_stage_relocalization",
    ROOT / "scripts" / "wrist_mine_localizer.py",
)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _pose(stamp=1.0, x=0.0):
    result = PoseStamped()
    result.header.frame_id = "ur5_base_link"
    result.header.stamp = EXECUTOR_MODULE.rospy.Time.from_sec(stamp)
    result.pose.position.x = float(x)
    result.pose.orientation.w = 1.0
    return result


def _candidate(yaw=0.0, tilt=0.0):
    return {
        "yaw": float(yaw),
        "tilt": float(tilt),
        "pregrasp_joints": [0.0] * 6,
        "pregrasp": _pose(),
        "approach": _pose(),
        "grasp": _pose(),
    }


class StageRelocalizationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        EXECUTOR_MODULE.rospy.rostime.set_rostime_initialized(True)
        LOCALIZER_MODULE.rospy.rostime.set_rostime_initialized(True)

    def test_localizer_epoch_reset_is_synchronous_and_invalidates(self):
        localizer = LOCALIZER_MODULE.WristMineLocalizer.__new__(
            LOCALIZER_MODULE.WristMineLocalizer
        )
        localizer.processing = threading.Lock()
        localizer.lock = threading.Lock()
        localizer.history = collections.deque([(1.0, object(), 0.5)])
        localizer.minimum_frames = 3
        localizer.confirmation_epoch = LOCALIZER_MODULE.rospy.Time(0)
        localizer.valid_pub = Publisher()
        localizer.status_pub = Publisher()

        response = localizer._reset_confirmation_cb(None)

        self.assertTrue(response.success)
        self.assertEqual(len(localizer.history), 0)
        self.assertGreater(localizer.confirmation_epoch.to_sec(), 0.0)
        self.assertFalse(localizer.valid_pub.messages[-1].data)
        self.assertIn("post-epoch RGB-D frames", localizer.status_pub.messages[-1].data)

    def test_localizer_drops_a_queued_pre_epoch_pair(self):
        localizer = LOCALIZER_MODULE.WristMineLocalizer.__new__(
            LOCALIZER_MODULE.WristMineLocalizer
        )
        localizer.processing = threading.Lock()
        localizer.lock = threading.Lock()
        stamp = LOCALIZER_MODULE.rospy.Time.from_sec(10.0)
        localizer.latest_pair = (
            SimpleNamespace(header=SimpleNamespace(stamp=stamp)),
            SimpleNamespace(header=SimpleNamespace(stamp=stamp)),
        )
        localizer.camera_info = object()
        localizer.last_processed_stamp = LOCALIZER_MODULE.rospy.Time(0)
        localizer.confirmation_epoch = LOCALIZER_MODULE.rospy.Time.from_sec(11.0)
        processed = []
        localizer._process = lambda *args: processed.append(args)
        localizer._invalidate = lambda *args, **kwargs: self.fail(
            "a queued pre-epoch pair must be dropped, not processed"
        )

        localizer._tick(None)

        self.assertEqual(processed, [])
        self.assertEqual(localizer.last_processed_stamp, stamp)

    def _executor(self):
        executor = EXECUTOR_MODULE.MineGraspExecutor.__new__(
            EXECUTOR_MODULE.MineGraspExecutor
        )
        executor.lock = threading.RLock()
        executor.metrics = {}
        executor.arm_base_frame = "ur5_base_link"
        executor.maximum_pregrasp_replans = 1
        executor.maximum_coarse_replans = 1
        executor.approach_speed = 0.02
        executor._publish_status = lambda *args, **kwargs: None
        executor._publish_candidate_poses = lambda _candidate: None
        executor._record_perception_target = lambda *args: None
        executor._record_validation_snapshot = lambda *args: None
        executor._target_in_frame = lambda target, _frame: target
        executor._verify_observation_base_drift = lambda *args, **kwargs: {}
        return executor

    def test_small_pregrasp_update_rebuilds_scene_and_ik_before_commit(self):
        executor = self._executor()
        previous = _pose(1.0)
        refined = _pose(2.0, x=0.010)
        original_candidate = _candidate()
        refined_candidate = _candidate()
        reference = {"observation_stamp": 2.0}
        order = []
        executor._fresh_stage_observation = lambda _stage: (
            refined, reference, "RELOCALIZE", {}
        )
        executor._target_update = lambda *args: (
            "REGENERATE", {"distance_m": 0.010, "decision": "REGENERATE"}
        )
        executor._add_ground_guard = lambda target: order.append(("guard", target))
        executor._select_candidate = lambda target: (
            order.append(("ik", target)) or refined_candidate
        )
        executor._commit_observation_base_reference = lambda value: order.append(
            ("commit", value)
        )
        executor._move_group_joints = lambda *args, **kwargs: self.fail(
            "a <=20 mm update must not add an unnecessary pregrasp motion"
        )

        actual_target, actual_candidate = executor._refine_from_pregrasp(
            previous, original_candidate
        )

        self.assertIs(actual_target, refined)
        self.assertIs(actual_candidate, refined_candidate)
        self.assertEqual([entry[0] for entry in order], ["guard", "ik", "commit"])

    def test_20_to_50_mm_base_reaction_replans_then_reobserves(self):
        executor = self._executor()
        first = _pose(2.0, x=0.010)
        second = _pose(3.0, x=0.011)
        observations = iter((
            (first, {"observation_stamp": 2.0}, "RETREAT_AND_REPLAN", {}),
            (second, {"observation_stamp": 3.0}, "WITHIN_STRICT_GATE", {}),
        ))
        executor._fresh_stage_observation = lambda _stage: next(observations)
        executor._target_update = lambda *args: (
            "REGENERATE", {"distance_m": 0.001, "decision": "REGENERATE"}
        )
        executor._add_ground_guard = lambda _target: None
        executor._select_candidate = lambda _target: _candidate()
        executor._commit_observation_base_reference = lambda _value: None
        moves = []
        executor._move_group_joints = lambda *args, **kwargs: (
            moves.append((args, kwargs))
            or EXECUTOR_MODULE.MotionExecutionResult(True, "SUCCESS", "ok", {})
        )

        target, _result_candidate = executor._refine_from_pregrasp(
            _pose(1.0), _candidate()
        )

        self.assertIs(target, second)
        self.assertEqual(len(moves), 1)
        self.assertEqual(executor.metrics["pregrasp_replans"], 1)

    def test_base_reaction_classification_keeps_5mm_and_50mm_distinct(self):
        executor = self._executor()
        executor.maximum_base_translation_after_observation = 0.005
        executor.maximum_base_yaw_after_observation = math.radians(1.0)
        executor.relocalize_small = 0.020
        executor.relocalize_maximum = 0.050

        def classify(translation, yaw=0.0):
            executor._verify_observation_base_drift = lambda *args, **kwargs: {
                "stage": "test",
                "translation_m": translation,
                "yaw_error_rad": yaw,
                "yaw_error_deg": math.degrees(yaw),
            }
            return executor._classify_relocalization_base_motion("TEST")[0]

        self.assertEqual(classify(0.005), "WITHIN_STRICT_GATE")
        self.assertEqual(classify(0.0052), "RELOCALIZE")
        self.assertEqual(classify(0.030), "RETREAT_AND_REPLAN")
        with self.assertRaises(EXECUTOR_MODULE.GraspFailure):
            classify(0.051)
        with self.assertRaises(EXECUTOR_MODULE.GraspFailure):
            classify(0.001, math.radians(1.1))

    def test_config_enables_verified_near_field_epoch_path(self):
        text = (ROOT / "config" / "grasp.yaml").read_text(encoding="utf-8")
        self.assertIn("near_field_relocalization_enabled: true", text)
        self.assertIn(
            "reset_confirmation_service: /mine_grasp/reset_target_confirmation",
            text,
        )


if __name__ == "__main__":
    unittest.main()
