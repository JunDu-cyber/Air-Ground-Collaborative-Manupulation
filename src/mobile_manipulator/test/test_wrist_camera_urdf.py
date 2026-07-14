#!/usr/bin/env python3
"""Static checks for the project-owned wrist RGB-D generated URDF."""

import math
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro


PACKAGE = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE / "urdf" / "husky_ur5.urdf.xacro"


def generated_sensor(horizontal_fov_rad=None):
    mappings = None
    if horizontal_fov_rad is not None:
        mappings = {
            "wrist_camera_horizontal_fov_rad": str(horizontal_fov_rad),
        }
    document = xacro.process_file(str(XACRO_PATH), mappings=mappings)
    root = ET.fromstring(document.toxml())
    matches = []
    for gazebo in root.findall("gazebo"):
        if gazebo.attrib.get("reference") != "realsense_camera_link":
            continue
        for sensor in gazebo.findall("sensor"):
            if sensor.attrib.get("name") == "realsense_depth":
                matches.append(sensor)
    if len(matches) != 1:
        raise AssertionError(
            "generated URDF has {} realsense_depth sensors".format(len(matches))
        )
    return matches[0]


class WristCameraUrdfTest(unittest.TestCase):
    def test_default_is_seventy_five_degrees_with_rgbd_contract_unchanged(self):
        sensor = generated_sensor()
        camera = sensor.find("camera")
        image = camera.find("image")
        clip = camera.find("clip")
        plugin = sensor.find("plugin")

        self.assertEqual(sensor.attrib.get("type"), "depth")
        self.assertAlmostEqual(float(sensor.findtext("update_rate")), 30.0)
        self.assertAlmostEqual(
            float(camera.findtext("horizontal_fov")), math.radians(75.0), places=8
        )
        self.assertEqual(int(image.findtext("width")), 640)
        self.assertEqual(int(image.findtext("height")), 480)
        self.assertEqual(image.findtext("format"), "R8G8B8")
        self.assertAlmostEqual(float(clip.findtext("near")), 0.05)
        self.assertAlmostEqual(float(clip.findtext("far")), 3.0)
        self.assertEqual(plugin.findtext("frameName"), "realsense_camera_optical_frame")
        self.assertEqual(plugin.findtext("imageTopicName"), "/camera/color/image_raw")
        self.assertEqual(plugin.findtext("depthImageTopicName"), "/camera/depth/image_raw")

    def test_xacro_fov_parameter_is_overridable(self):
        sensor = generated_sensor(math.radians(80.0))
        value = float(sensor.find("camera").findtext("horizontal_fov"))
        self.assertAlmostEqual(value, math.radians(80.0), places=9)

    def test_measured_edge_ray_moves_clear_of_bottom_crossbar(self):
        width = 640.0
        centre_v = 240.0
        focal_60 = width / (2.0 * math.tan(math.radians(60.0) / 2.0))
        focal_75 = width / (2.0 * math.tan(math.radians(75.0) / 2.0))

        # M004's fixed-LOOK evidence placed the ray at about v=453 with the
        # former 60-degree intrinsics. Reproject the same optical ray only;
        # no map or Gazebo truth coordinate participates in this calculation.
        ray_tangent = (453.0 - centre_v) / focal_60
        reprojected_v = centre_v + focal_75 * ray_tangent
        self.assertGreater(reprojected_v, 395.0)
        self.assertLess(reprojected_v, 405.0)
        self.assertLess(reprojected_v, 420.0)  # measured crossbar/occlusion band

        # A 1500-pixel M002 yellow face scales conservatively with f^2 and
        # remains orders of magnitude above the unchanged 18-pixel gate.
        conservative_area = 1500.0 * (focal_75 / focal_60) ** 2
        self.assertGreater(conservative_area, 800.0)
        self.assertGreater(conservative_area, 18.0 * 40.0)


if __name__ == "__main__":
    unittest.main()
