#!/usr/bin/env python3
"""Unit tests for the measured terminal-motion acceptance gate."""

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, PACKAGE_SRC)

from mine_grasp_minimal.motion_stability import (  # noqa: E402
    JointSpanWindow,
    classify_stability,
)


def classify(**updates):
    values = {
        "max_joint_error": 0.001,
        "max_joint_velocity": 0.004,
        "max_joint_span": 0.0004,
        "window_ready": True,
        "tcp_position_error": 0.002,
        "tcp_orientation_error": 0.01,
        "joint_error_tolerance": 0.01,
        "joint_velocity_tolerance": 0.05,
        "joint_span_tolerance": 0.002,
        "tcp_position_tolerance": 0.008,
        "tcp_orientation_tolerance": 0.0523598776,
        "require_tcp": True,
    }
    values.update(updates)
    return classify_stability(**values)


class MotionStabilityTest(unittest.TestCase):
    def test_joint_window_requires_full_half_second(self):
        window = JointSpanWindow(0.5)
        window.add(1.0, [1.0, 2.0])
        window.add(1.2, [1.0003, 1.9998])
        self.assertFalse(window.ready)
        window.add(1.5, [1.0001, 2.0002])
        self.assertTrue(window.ready)
        self.assertAlmostEqual(window.maximum_span(), 0.0004, places=7)

    def test_window_velocity_rejects_motion_but_ignores_instantaneous_field(self):
        window = JointSpanWindow(0.5)
        window.add(2.0, [0.49000])
        window.add(2.25, [0.49004])
        window.add(2.5, [0.49005])
        self.assertTrue(window.ready)
        self.assertLess(window.maximum_endpoint_velocity(), 0.001)
        self.assertLess(window.maximum_sample_velocity(), 0.001)

        moving = JointSpanWindow(0.5)
        moving.add(2.0, [0.490])
        moving.add(2.5, [0.505])
        self.assertGreater(moving.maximum_endpoint_velocity(), 0.02)
        self.assertGreater(moving.maximum_sample_velocity(), 0.02)

        oscillating = JointSpanWindow(0.5)
        oscillating.add(2.0, [0.490])
        oscillating.add(2.25, [0.505])
        oscillating.add(2.5, [0.490])
        self.assertEqual(oscillating.maximum_endpoint_velocity(), 0.0)
        self.assertGreater(oscillating.maximum_sample_velocity(), 0.05)

    def test_all_measured_limits_accept(self):
        stable, code, _detail = classify()
        self.assertTrue(stable)
        self.assertEqual(code, "SUCCESS")

    def test_true_position_error_is_not_terminal_settle(self):
        stable, code, detail = classify(max_joint_error=0.0101)
        self.assertFalse(stable)
        self.assertEqual(code, "TRUE_POSITION_ERROR")
        self.assertIn("joint error", detail)

    def test_residual_speed_and_span_are_controller_unsettled(self):
        for update in (
            {"max_joint_velocity": 0.0501},
            {"max_joint_span": 0.0021},
            {"window_ready": False},
        ):
            with self.subTest(update=update):
                stable, code, _detail = classify(**update)
                self.assertFalse(stable)
                self.assertEqual(code, "CONTROLLER_UNSETTLED")

    def test_tcp_position_and_orientation_fail_separately(self):
        for update in (
            {"tcp_position_error": 0.0081},
            {"tcp_orientation_error": 0.0524},
            {"tcp_position_error": None},
        ):
            with self.subTest(update=update):
                stable, code, _detail = classify(**update)
                self.assertFalse(stable)
                self.assertEqual(code, "TCP_NOT_REACHED")

    def test_joint_only_gate_does_not_invent_tcp_requirement(self):
        stable, code, _detail = classify(
            require_tcp=False,
            tcp_position_error=None,
            tcp_orientation_error=None,
        )
        self.assertTrue(stable)
        self.assertEqual(code, "SUCCESS")


if __name__ == "__main__":
    unittest.main()
