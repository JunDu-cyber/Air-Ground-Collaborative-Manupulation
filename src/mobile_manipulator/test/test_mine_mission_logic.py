#!/usr/bin/env python3

import math
import os
import sys
import threading
import unittest
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPT_DIR)

from mine_mission_manager import (  # noqa: E402
    ArmAttemptResult,
    DEFAULT_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR,
    MAX_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR,
    MineMissionManager,
    TERMINAL_STATES,
    Task,
    base_motion_window_span,
    base_safety_failure_state,
    centered_lateral_index,
    decode_executor_report,
    empty_pick_retreat_step,
    make_approach_pose,
    normalized_executor_failure_code,
    path_length,
    tf_stamp_is_post_cancel_fresh,
)
from mobile_manipulator.msg import MineGraspResult, MineMissionEntry  # noqa: E402
from uav_truth_tracker.msg import MineMapEntry  # noqa: E402


class MineMissionLogicTest(unittest.TestCase):
    def test_handoff_timeout_blocks_automatic_candidate_retry(self):
        state = base_safety_failure_state(paused=False)
        self.assertEqual(MineMissionEntry.MANUAL_REQUIRED, state)
        self.assertIn(state, TERMINAL_STATES)
        paused_state = base_safety_failure_state(paused=True)
        self.assertEqual(MineMissionEntry.PENDING, paused_state)
        self.assertNotIn(paused_state, TERMINAL_STATES)

    def test_navigation_to_fine_handoff_requires_complete_stopped_window(self):
        stable = [
            (0.0, 1.0000, -2.0000, math.radians(179.9)),
            (0.1, 1.0008, -2.0005, math.radians(-179.9)),
            (0.2, 0.9997, -1.9998, math.radians(179.8)),
            (0.3, 1.0002, -2.0001, math.radians(-179.8)),
            (0.4, 1.0001, -2.0002, math.radians(179.9)),
            (0.5, 1.0000, -2.0000, math.radians(-179.9)),
        ]
        self.assertIsNone(base_motion_window_span(stable[:-1], 0.5, 5))
        duration, translation, yaw, count = base_motion_window_span(
            stable, 0.5, 5
        )
        self.assertGreaterEqual(duration, 0.5)
        self.assertLessEqual(translation, 0.005)
        self.assertLessEqual(yaw, math.radians(0.5))
        self.assertGreaterEqual(count, 5)

    def test_handoff_tf_must_be_fresh_and_postdate_cancel(self):
        self.assertFalse(tf_stamp_is_post_cancel_fresh(
            99.9, 100.1, 100.0, 0.5
        )[0])
        self.assertFalse(tf_stamp_is_post_cancel_fresh(
            100.0, 100.1, 100.0, 0.5
        )[0])
        self.assertTrue(tf_stamp_is_post_cancel_fresh(
            100.05, 100.10, 100.0, 0.5
        )[0])
        self.assertFalse(tf_stamp_is_post_cancel_fresh(
            100.05, 100.70, 100.0, 0.5
        )[0])
        self.assertFalse(tf_stamp_is_post_cancel_fresh(
            100.20, 100.10, 100.0, 0.5
        )[0])
        self.assertFalse(tf_stamp_is_post_cancel_fresh(
            float("nan"), 100.1, 100.0, 0.5
        )[0])

    def test_navigation_to_fine_handoff_catches_zero_net_oscillation(self):
        # First and last poses match, but the middle excursion exceeds the
        # 5 mm safety limit and therefore must not be classified as stopped.
        oscillating = [
            (0.0, 0.000, 0.0, 0.0),
            (0.1, 0.002, 0.0, math.radians(0.2)),
            (0.2, 0.007, 0.0, math.radians(0.7)),
            (0.3, 0.003, 0.0, math.radians(0.3)),
            (0.4, 0.001, 0.0, math.radians(0.1)),
            (0.5, 0.000, 0.0, 0.0),
        ]
        _, translation, yaw, _ = base_motion_window_span(
            oscillating, 0.5, 5
        )
        self.assertGreater(translation, 0.005)
        self.assertGreater(yaw, math.radians(0.5))

        invalid = list(oscillating)
        invalid[2] = (0.2, float("nan"), 0.0, 0.0)
        with self.assertRaises(ValueError):
            base_motion_window_span(invalid, 0.5, 5)

    def test_empty_pick_retreat_is_reverse_only_and_bounded(self):
        common = (
            0.84,
            0.020,
            math.radians(3.0),
            0.12,
            0.25,
        )
        state, linear, angular = empty_pick_retreat_step(
            0.67, math.radians(2.0), *common
        )
        self.assertEqual("moving", state)
        self.assertLess(linear, 0.0)
        self.assertLessEqual(abs(linear), 0.12)
        self.assertEqual(0.0, angular)

        state, linear, angular = empty_pick_retreat_step(
            0.70, math.radians(4.0), *common
        )
        self.assertEqual("moving", state)
        self.assertLess(linear, 0.0)
        self.assertGreater(angular, 0.0)

        self.assertEqual(
            "reached",
            empty_pick_retreat_step(0.84, math.radians(2.0), *common)[0],
        )
        state, linear, _ = empty_pick_retreat_step(
            0.861, 0.0, *common
        )
        self.assertEqual("overshot", state)
        self.assertEqual(0.0, linear)
        state, linear, angular = empty_pick_retreat_step(
            0.67, math.radians(6.1), *common
        )
        self.assertEqual("heading_rejected", state)
        self.assertEqual((0.0, 0.0), (linear, angular))

    def test_full_chain_braking_handoff_defaults_are_strict(self):
        launch_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "launch", "mine_mission.launch"
        ))
        root = ET.parse(launch_path).getroot()
        params = {
            node.attrib.get("name"): node.attrib.get("value")
            for node in root.findall(".//param")
        }
        self.assertEqual("2.0", params["fine_alignment_handoff_timeout"])
        self.assertEqual("0.5", params["fine_alignment_handoff_window"])
        self.assertEqual(
            "0.005", params["fine_alignment_handoff_translation_tolerance"]
        )
        self.assertEqual(
            "0.5", params["fine_alignment_handoff_yaw_tolerance_deg"]
        )
        self.assertEqual("5", params["fine_alignment_handoff_min_samples"])
        self.assertEqual("8.0", params["empty_pick_retreat_timeout"])
        # The braking fix must not weaken the existing bounded controller.
        self.assertEqual(
            "0.35", params["fine_alignment_max_initial_distance_error"]
        )
        self.assertEqual("0.12", params["fine_alignment_max_linear"])

    def test_approach_ring_is_configured_distance_and_faces_mine(self):
        mine = PoseStamped()
        mine.header.frame_id = "map"
        mine.pose.position.x = 4.2
        mine.pose.position.y = -1.7
        mine.pose.orientation.w = 1.0

        positions = set()
        for index in range(16):
            goal = make_approach_pose(mine, index * math.pi / 8.0, 0.84, "map")
            dx = mine.pose.position.x - goal.pose.position.x
            dy = mine.pose.position.y - goal.pose.position.y
            self.assertAlmostEqual(math.hypot(dx, dy), 0.84, places=6)
            q = goal.pose.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.assertAlmostEqual(
                math.cos(yaw) * dx / 0.84 + math.sin(yaw) * dy / 0.84,
                1.0,
                places=6,
            )
            positions.add((round(goal.pose.position.x, 5), round(goal.pose.position.y, 5)))
        self.assertEqual(len(positions), 16)

    def test_path_length_uses_actual_polyline(self):
        path = Path()
        for x, y in [(0.0, 0.0), (3.0, 4.0), (3.0, 6.0)]:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            path.poses.append(pose)
        self.assertAlmostEqual(path_length(path), 7.0)

    def test_coarse_navigation_gate_hands_off_before_precision_gate(self):
        manager = MineMissionManager.__new__(MineMissionManager)
        manager.approach_distance = 0.67
        manager.navigation_approach_distance = 0.84
        manager.arrival_distance_tolerance = 0.020
        manager.arrival_yaw_tolerance = math.radians(3.0)
        manager.coarse_arrival_distance_tolerance = 0.07
        manager.coarse_arrival_yaw_tolerance = math.radians(180.0)

        manager._mine_alignment_errors = lambda _mine_id: (
            0.84,
            math.radians(125.0),
        )
        coarse_ok, _ = manager._verify_coarse_arrival(1)
        precise_ok, _ = manager._verify_arrival(1)
        self.assertTrue(coarse_ok)
        self.assertFalse(precise_ok)

        manager._mine_alignment_errors = lambda _mine_id: (
            0.67,
            math.radians(2.0),
        )
        precise_ok, _ = manager._verify_arrival(1)
        self.assertTrue(precise_ok)

        for boundary in (0.77, 0.91):
            manager._mine_alignment_errors = lambda _mine_id, value=boundary: (
                value,
                math.radians(179.0),
            )
            coarse_ok, _ = manager._verify_coarse_arrival(1)
            self.assertTrue(coarse_ok)
        manager._mine_alignment_errors = lambda _mine_id: (0.92, 0.0)
        coarse_ok, _ = manager._verify_coarse_arrival(1)
        self.assertFalse(coarse_ok)

    def test_final_standoff_keeps_husky_outside_mine_hazard(self):
        final_standoff = 0.67
        base_front = 0.50
        footprint_padding = 0.01
        mine_lethal_radius = 0.10
        self.assertGreaterEqual(
            final_standoff - base_front - footprint_padding - mine_lethal_radius,
            0.06,
        )

    def test_full_chain_fine_alignment_entry_covers_measured_handoff_drift(self):
        launch_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "launch", "mine_mission.launch"
        ))
        root = ET.parse(launch_path).getroot()
        configured = [
            node.attrib["value"]
            for node in root.findall(".//param")
            if node.attrib.get("name")
            == "fine_alignment_max_initial_distance_error"
        ]
        self.assertEqual(configured, ["0.35"])
        entry_limit = float(configured[0])
        self.assertEqual(
            entry_limit, DEFAULT_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR
        )
        self.assertEqual(
            entry_limit, MAX_FINE_ALIGNMENT_MAX_INITIAL_DISTANCE_ERROR
        )
        # Both values were measured after a valid 0.84 m coarse-ring handoff
        # in the second run_air_ground.sh chain.
        self.assertLessEqual(0.281, entry_limit)
        self.assertLessEqual(0.285, entry_limit)
        self.assertGreater(0.351, entry_limit)

        # Expanding the entry neighbourhood must not alter the physical grasp
        # geometry or its final acceptance gates.
        launch_args = {
            node.attrib.get("name"): node.attrib.get("default")
            for node in root.findall("./arg")
        }
        launch_params = {
            node.attrib.get("name"): node.attrib.get("value")
            for node in root.findall(".//param")
        }
        self.assertEqual(launch_args["approach_distance"], "0.67")
        self.assertEqual(launch_params["arrival_distance_tolerance"], "0.020")
        self.assertEqual(launch_params["arrival_yaw_tolerance_deg"], "3.0")

    def test_only_explicit_success_code_is_clearable(self):
        self.assertEqual(MineGraspResult.SUCCESS, 0)
        unsafe_outcomes = {
            MineGraspResult.RETRYABLE_FAILURE,
            MineGraspResult.FATAL_FAILURE,
            MineGraspResult.TARGET_NOT_FOUND,
            MineGraspResult.UNSAFE,
            MineGraspResult.CANCELLED,
        }
        self.assertNotIn(MineGraspResult.SUCCESS, unsafe_outcomes)
        self.assertEqual(len(unsafe_outcomes), 5)

    def test_executor_report_preserves_recovery_and_retention_evidence(self):
        report = decode_executor_report(
            'PICK failed: {"failure_reason":"CONTROLLER_UNSETTLED",'
            '"recovery_attempted":true,"recovery_success":true,'
            '"retained":false}'
        )
        self.assertEqual(report["failure_reason"], "CONTROLLER_UNSETTLED")
        self.assertTrue(report["recovery_attempted"])
        self.assertTrue(report["recovery_success"])
        self.assertFalse(report["retained"])
        self.assertEqual(decode_executor_report("legacy failure text"), {})

    def test_observation_drift_legacy_code_is_narrowly_disambiguated(self):
        report = {
            "failure_reason": "BASE_UNSTABLE",
            "failure_detail": (
                "PREGRASP base moved 0.0052 m / 0.00 deg after the frozen "
                "wrist observation (limits 0.0050 m / 1.00 deg)"
            ),
            "observation_base_drift_checks": [{
                "stage": "PREGRASP",
                "translation_m": 0.0052038,
                "translation_limit_m": 0.005,
                "yaw_error_rad": math.radians(0.0011),
                "yaw_limit_deg": 1.0,
            }],
        }
        self.assertEqual(
            "OBSERVATION_INVALIDATED",
            normalized_executor_failure_code(report),
        )

        # Text alone cannot downgrade a safety classification, and a true
        # attitude failure remains BASE_UNSTABLE even if recovery reached LOOK.
        del report["observation_base_drift_checks"]
        self.assertEqual(
            "BASE_UNSTABLE", normalized_executor_failure_code(report)
        )
        report["failure_detail"] = "roll=6.10 deg pitch=0.30 deg"
        report["observation_base_drift_checks"] = []
        self.assertEqual(
            "BASE_UNSTABLE", normalized_executor_failure_code(report)
        )

    def test_approach_direction_index_distinguishes_retries(self):
        manager = MineMissionManager.__new__(MineMissionManager)
        manager.candidate_count = 16
        mine = PoseStamped()
        mine.pose.position.x = 2.0
        mine.pose.position.y = -1.0
        directions = set()
        for index in range(16):
            approach = make_approach_pose(
                mine, index * math.pi / 8.0, 0.67, "map"
            )
            directions.add(manager._approach_direction_index(mine, approach))
        self.assertEqual(directions, set(range(16)))

    def test_dispatched_alignment_uses_frozen_mine_pose(self):
        manager = MineMissionManager.__new__(MineMissionManager)
        manager.lock = threading.RLock()
        live = PoseStamped()
        live.pose.position.x = 9.0
        live.pose.orientation.w = 1.0
        frozen = PoseStamped()
        frozen.pose.position.x = 2.0
        frozen.pose.orientation.w = 1.0
        robot = PoseStamped()
        robot.pose.orientation.w = 1.0
        manager.tasks = {
            7: Task(
                mine_id=7,
                mine_pose=live,
                frozen_mine_pose=frozen,
                confidence=1.0,
                observation_count=3,
            )
        }
        manager._robot_pose = lambda: robot
        distance, yaw_error = manager._mine_alignment_errors(7)
        self.assertAlmostEqual(distance, 2.0)
        self.assertAlmostEqual(yaw_error, 0.0)

    def test_new_confirmed_id_near_existing_task_is_alias(self):
        manager = MineMissionManager.__new__(MineMissionManager)
        existing = PoseStamped()
        existing.pose.position.x = 2.30
        existing.pose.position.y = -1.08
        existing.pose.position.z = 0.03
        existing.pose.orientation.w = 1.0
        manager.tasks = {
            2: Task(
                mine_id=2,
                mine_pose=existing,
                confidence=0.9,
                observation_count=5,
            )
        }
        manager.task_alias_radius = 0.30
        manager.task_alias_height_tolerance = 0.20
        fragment = MineMapEntry()
        fragment.id = 99
        fragment.position.x = 2.39
        fragment.position.y = -1.15
        fragment.position.z = 0.04
        self.assertEqual(manager._task_alias_id_locked(fragment), 2)
        fragment.position.x = 2.75
        self.assertIsNone(manager._task_alias_id_locked(fragment))

    def test_unsafe_pick_never_deferred_even_after_look_recovery(self):
        unsafe = ArmAttemptResult(
            False,
            "base exceeded attitude gate",
            code="BASE_UNSTABLE",
            recovered_to_look=True,
            retained=False,
            action_outcome=MineGraspResult.UNSAFE,
        )
        self.assertTrue(MineMissionManager._arm_failure_requires_manual(unsafe))
        retryable = ArmAttemptResult(
            False,
            "empty grasp",
            code="TARGET_NOT_FOUND",
            recovered_to_look=True,
            retained=False,
            action_outcome=MineGraspResult.TARGET_NOT_FOUND,
        )
        self.assertFalse(
            MineMissionManager._arm_failure_requires_manual(retryable)
        )

    def test_recovered_observation_invalidation_defers_but_faults_lock(self):
        invalidated = ArmAttemptResult(
            False,
            "frozen observation moved beyond its recorded gate",
            code="OBSERVATION_INVALIDATED",
            recovered_to_look=True,
            retained=False,
            # Compatibility with the currently deployed executor, which maps
            # its overloaded BASE_UNSTABLE code to UNSAFE.
            action_outcome=MineGraspResult.UNSAFE,
        )
        self.assertFalse(
            MineMissionManager._arm_failure_requires_manual(invalidated)
        )
        invalidated.recovered_to_look = False
        self.assertTrue(
            MineMissionManager._arm_failure_requires_manual(invalidated)
        )
        invalidated.recovered_to_look = True
        invalidated.retained = True
        self.assertTrue(
            MineMissionManager._arm_failure_requires_manual(invalidated)
        )

        true_base_fault = ArmAttemptResult(
            False,
            "roll=6.10 deg pitch=0.30 deg",
            code="BASE_UNSTABLE",
            recovered_to_look=True,
            retained=False,
            action_outcome=MineGraspResult.UNSAFE,
        )
        self.assertTrue(
            MineMissionManager._arm_failure_requires_manual(true_base_fault)
        )

    def test_depot_slots_keep_every_later_parking_pose_clear(self):
        columns = 5
        lateral_spacing = 1.20
        row_spacing = 2.00
        drop_standoff = 0.65
        self.assertEqual(
            [centered_lateral_index(i) for i in range(columns)],
            [0, 1, -1, 2, -2],
        )
        drops = []
        for slot in range(10):
            row = slot // columns
            lateral = centered_lateral_index(slot % columns)
            drop = (1.8 + row * row_spacing, lateral * lateral_spacing)
            parking = (drop[0] - drop_standoff, drop[1])
            for prior in drops:
                self.assertGreaterEqual(
                    math.hypot(drop[0] - prior[0], drop[1] - prior[1]),
                    0.75,
                )
                self.assertGreaterEqual(
                    math.hypot(parking[0] - prior[0], parking[1] - prior[1]),
                    0.75,
                )
            drops.append(drop)


if __name__ == "__main__":
    unittest.main()
