#!/usr/bin/env python3
"""Static regressions for the project-owned UR5 damping injection path."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import xacro


PACKAGE_DIR = Path(__file__).resolve().parents[1]
GENERATOR = PACKAGE_DIR / "scripts" / "generate_husky_ur5_urdf.py"
ROBOT_XACRO = PACKAGE_DIR / "urdf" / "husky_ur5.urdf.xacro"
RUN_SCRIPT = PACKAGE_DIR.parents[1] / "run_air_ground.sh"

SPEC = importlib.util.spec_from_file_location("ur5_damping_generator", GENERATOR)
GENERATOR_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR_MODULE)


def _top_level_joints(root):
    return {joint.attrib["name"]: joint for joint in root.findall("./joint")}


def _dynamics_attributes(root):
    result = {}
    for name, joint in _top_level_joints(root).items():
        dynamics = joint.findall("./dynamics")
        if dynamics:
            result[name] = dict(dynamics[0].attrib)
    return result


class Ur5JointDampingTest(unittest.TestCase):
    def generated_root(self, damping=1.5, overrides=None):
        document = GENERATOR_MODULE.generate_urdf_document(
            str(ROBOT_XACRO), damping, overrides
        )
        return ET.fromstring(document.toxml())

    def test_default_sets_shoulder_twenty_and_other_five_common(self):
        baseline = ET.fromstring(xacro.process_file(str(ROBOT_XACRO)).toxml())
        generated = self.generated_root()
        generated_joints = _top_level_joints(generated)

        self.assertEqual(6, len(GENERATOR_MODULE.ARM_JOINTS))
        for name in GENERATOR_MODULE.ARM_JOINTS:
            dynamics = generated_joints[name].find("./dynamics")
            self.assertIsNotNone(dynamics)
            expected = (
                20.0
                if name == GENERATOR_MODULE.SHOULDER_LIFT_JOINT else 1.5
            )
            self.assertAlmostEqual(expected, float(dynamics.attrib["damping"]))
            # The post-processor must not manufacture Coulomb friction.
            self.assertEqual("0", dynamics.attrib["friction"])

        baseline_dynamics = _dynamics_attributes(baseline)
        generated_dynamics = _dynamics_attributes(generated)
        for name, attributes in baseline_dynamics.items():
            if name not in GENERATOR_MODULE.ARM_JOINTS:
                self.assertEqual(attributes, generated_dynamics[name])

    def test_single_joint_override_does_not_change_common_value(self):
        generated = self.generated_root(
            1.5, {"ur5_shoulder_lift_joint": 8.0}
        )
        joints = _top_level_joints(generated)
        for name in GENERATOR_MODULE.ARM_JOINTS:
            expected = 8.0 if name == "ur5_shoulder_lift_joint" else 1.5
            self.assertAlmostEqual(
                expected, float(joints[name].find("./dynamics").attrib["damping"])
            )

    def test_invalid_damping_is_rejected(self):
        with self.assertRaises(ValueError):
            self.generated_root(-0.1)
        with self.assertRaises(ValueError):
            self.generated_root(float("nan"))
        with self.assertRaises(ValueError):
            self.generated_root(
                1.5, {GENERATOR_MODULE.SHOULDER_LIFT_JOINT: -0.1}
            )
        with self.assertRaises(ValueError):
            self.generated_root(
                1.5, {GENERATOR_MODULE.SHOULDER_LIFT_JOINT: float("inf")}
            )
        with self.assertRaises(Exception):
            GENERATOR_MODULE.nonnegative_finite("nan")

    def test_cli_defaults_expose_independent_shoulder_parameter(self):
        arguments = GENERATOR_MODULE._parser().parse_args(
            ["--xacro", str(ROBOT_XACRO)]
        )
        self.assertAlmostEqual(1.5, arguments.arm_joint_damping)
        self.assertAlmostEqual(
            20.0, arguments.ur5_shoulder_lift_joint_damping
        )

    def test_full_chain_launches_share_parameterized_generator(self):
        mobile_launch_names = (
            "air_ground_world.launch",
            "ugv_terrain_nav.launch",
            "spawn_robot.launch",
            "spawn_outdoor_city.launch",
        )
        for launch_name in mobile_launch_names:
            root = ET.parse(PACKAGE_DIR / "launch" / launch_name).getroot()
            common_args = [
                arg
                for arg in root.findall("./arg")
                if arg.attrib.get("name") == "ur5_joint_damping"
            ]
            shoulder_args = [
                arg
                for arg in root.findall("./arg")
                if arg.attrib.get("name") == "ur5_shoulder_lift_damping"
            ]
            self.assertEqual(1, len(common_args), launch_name)
            self.assertEqual("1.5", common_args[0].attrib.get("default"))
            self.assertEqual(1, len(shoulder_args), launch_name)
            self.assertEqual("20.0", shoulder_args[0].attrib.get("default"))
            robot_description = root.find("./param[@name='robot_description']")
            command = robot_description.attrib.get("command", "")
            self.assertIn("generate_husky_ur5_urdf.py", command)
            self.assertIn(
                "--arm-joint-damping $(arg ur5_joint_damping)", command
            )
            self.assertIn(
                "--shoulder-lift-joint-damping "
                "$(arg ur5_shoulder_lift_damping)",
                command,
            )

        moveit_launch = PACKAGE_DIR.parent / (
            "husky_ur5_moveit_config/launch"
        )
        planning_context = ET.parse(
            moveit_launch / "planning_context.launch"
        ).getroot()
        planning_args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in planning_context.findall("./arg")
        }
        self.assertEqual("1.5", planning_args.get("ur5_joint_damping"))
        self.assertEqual(
            "20.0", planning_args.get("ur5_shoulder_lift_damping")
        )
        planning_command = planning_context.find(
            "./param[@name='$(arg robot_description)']"
        ).attrib["command"]
        self.assertIn("--arm-joint-damping $(arg ur5_joint_damping)", planning_command)
        self.assertIn(
            "--shoulder-lift-joint-damping "
            "$(arg ur5_shoulder_lift_damping)",
            planning_command,
        )

        run_script = RUN_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("UR5_JOINT_DAMPING=${UR5_JOINT_DAMPING:-1.5}", run_script)
        self.assertIn(
            "UR5_SHOULDER_LIFT_DAMPING="
            "${UR5_SHOULDER_LIFT_DAMPING:-20.0}",
            run_script,
        )
        self.assertEqual(
            2,
            run_script.count("ur5_joint_damping:='$UR5_JOINT_DAMPING'"),
        )
        self.assertEqual(
            2,
            run_script.count(
                "ur5_shoulder_lift_damping:="
                "'$UR5_SHOULDER_LIFT_DAMPING'"
            ),
        )

        air_ground = ET.parse(
            PACKAGE_DIR / "launch" / "air_ground_world.launch"
        ).getroot()
        nested_nav_args = [
            arg
            for include in air_ground.findall(".//include")
            if include.attrib.get("file", "").endswith("ugv_terrain_nav.launch")
            for arg in include.findall("./arg")
            if arg.attrib.get("name") == "ur5_joint_damping"
        ]
        self.assertEqual(1, len(nested_nav_args))
        self.assertEqual(
            "$(arg ur5_joint_damping)", nested_nav_args[0].attrib.get("value")
        )
        nested_shoulder_args = [
            arg
            for include in air_ground.findall(".//include")
            if include.attrib.get("file", "").endswith("ugv_terrain_nav.launch")
            for arg in include.findall("./arg")
            if arg.attrib.get("name") == "ur5_shoulder_lift_damping"
        ]
        self.assertEqual(1, len(nested_shoulder_args))
        self.assertEqual(
            "$(arg ur5_shoulder_lift_damping)",
            nested_shoulder_args[0].attrib.get("value"),
        )

        move_group = ET.parse(moveit_launch / "move_group.launch").getroot()
        move_group_args = {
            arg.attrib.get("name"): arg.attrib.get("default")
            for arg in move_group.findall("./arg")
        }
        self.assertEqual("1.5", move_group_args.get("ur5_joint_damping"))
        self.assertEqual(
            "20.0", move_group_args.get("ur5_shoulder_lift_damping")
        )
        planning_includes = [
            include
            for include in move_group.findall("./include")
            if include.attrib.get("file", "").endswith("planning_context.launch")
        ]
        self.assertEqual(1, len(planning_includes))
        passed = {
            arg.attrib.get("name"): arg.attrib.get("value")
            for arg in planning_includes[0].findall("./arg")
        }
        self.assertEqual(
            "$(arg ur5_joint_damping)", passed.get("ur5_joint_damping")
        )
        self.assertEqual(
            "$(arg ur5_shoulder_lift_damping)",
            passed.get("ur5_shoulder_lift_damping"),
        )


if __name__ == "__main__":
    unittest.main()
