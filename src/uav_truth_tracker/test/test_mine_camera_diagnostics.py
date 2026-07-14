#!/usr/bin/env python3
"""Wall-clock camera timestamp-health regression checks."""

import importlib.util
import unittest
from pathlib import Path

import rospy
from sensor_msgs.msg import Image


PACKAGE = Path(__file__).resolve().parents[1]


def load_diagnostics_module():
    path = PACKAGE / "scripts" / "mine_camera_diagnostics.py"
    spec = importlib.util.spec_from_file_location(
        "mine_camera_diagnostics", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


StreamState = load_diagnostics_module().StreamState


def image_at(stamp):
    message = Image()
    message.header.stamp = rospy.Time.from_sec(float(stamp))
    return message


class MineCameraDiagnosticsTest(unittest.TestCase):
    def test_advancement_must_be_recent(self):
        state = StreamState()
        state.update(image_at(1.0), 10.0, 5.0)
        self.assertFalse(state.stamp_is_advancing(10.0, 2.0))
        state.update(image_at(1.1), 10.1, 5.0)
        self.assertTrue(state.stamp_is_advancing(12.0, 2.0))
        self.assertFalse(state.stamp_is_advancing(12.2, 2.0))

    def test_equal_and_regressed_stamps_are_reported(self):
        state = StreamState()
        state.update(image_at(2.0), 20.0, 5.0)
        state.update(image_at(2.0), 20.1, 5.0)
        state.update(image_at(1.9), 20.2, 5.0)
        self.assertEqual(state.stamp_repeats, 1)
        self.assertEqual(state.regressions, 1)
        self.assertFalse(state.stamp_is_advancing(20.2, 2.0))


if __name__ == "__main__":
    unittest.main()
