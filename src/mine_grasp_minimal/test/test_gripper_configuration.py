#!/usr/bin/env python3
"""Regression checks for the physical 2F-140 operating range."""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


class GripperConfigurationTest(unittest.TestCase):
    def _config(self):
        config_path = Path(__file__).resolve().parents[1] / "config" / "grasp.yaml"
        with config_path.open("r", encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def test_open_target_stays_off_lower_hard_stop(self):
        config = self._config()
        open_position = float(config["gripper_open_position"])
        close_positions = [
            float(value) for value in config["gripper_close_positions"]
        ]
        self.assertGreaterEqual(open_position, 0.05)
        self.assertLess(open_position, 0.20)
        self.assertTrue(all(value > open_position for value in close_positions))

    def test_air_ground_spawn_uses_the_same_safe_open_position(self):
        config = self._config()
        launch_path = (
            Path(__file__).resolve().parents[2]
            / "mobile_manipulator" / "launch" / "air_ground_world.launch"
        )
        root = ET.parse(str(launch_path)).getroot()
        arguments = {
            node.attrib.get("name"): node.attrib.get("default")
            for node in root.findall("arg")
        }
        self.assertIn("gripper_initial_position", arguments)
        self.assertAlmostEqual(
            float(arguments["gripper_initial_position"]),
            float(config["gripper_open_position"]),
            places=6,
        )
        spawn = next(
            node for node in root.findall("node")
            if node.attrib.get("name") == "spawn_urdf"
        )
        self.assertIn(
            "-J finger_joint $(arg gripper_initial_position)",
            spawn.attrib.get("args", ""),
        )


if __name__ == "__main__":
    unittest.main()
