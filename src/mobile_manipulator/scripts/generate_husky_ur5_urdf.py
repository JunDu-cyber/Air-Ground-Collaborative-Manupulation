#!/usr/bin/env python3
"""Generate the project Husky/UR5 URDF with explicit arm joint damping.

The upstream Noetic ``ur_description`` macro hard-codes zero damping for all
six arm joints and does not expose a xacro argument for it.  This small,
project-owned post-processor keeps using that upstream model, then changes only
the six generated ``<dynamics damping=...>`` attributes before the XML is put
on ``robot_description``.  It avoids modifying /opt, a generated URDF snapshot,
or Gazebo's transient set_joint_properties service.
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, Mapping, Optional

import xacro


ARM_JOINTS = (
    "ur5_shoulder_pan_joint",
    "ur5_shoulder_lift_joint",
    "ur5_elbow_joint",
    "ur5_wrist_1_joint",
    "ur5_wrist_2_joint",
    "ur5_wrist_3_joint",
)

DEFAULT_ARM_JOINT_DAMPING = 1.5
SHOULDER_LIFT_JOINT = "ur5_shoulder_lift_joint"
# Integrated run_air_ground measurements isolate the residual stopped velocity
# to shoulder_lift; the other five joints remain stable with the common 1.5.
DEFAULT_SHOULDER_LIFT_JOINT_DAMPING = 20.0


def nonnegative_finite(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError(
            "joint damping must be finite and non-negative"
        )
    return parsed


def _direct_children(element, tag_name):
    return [
        child
        for child in element.childNodes
        if child.nodeType == child.ELEMENT_NODE and child.tagName == tag_name
    ]


def generate_urdf_document(
    xacro_file: str,
    common_damping: float = DEFAULT_ARM_JOINT_DAMPING,
    overrides: Optional[Mapping[str, float]] = None,
):
    """Expand *xacro_file* and return a DOM with measured arm damping.

    The operation is deliberately strict: a missing/duplicate arm joint or a
    missing/duplicate dynamics element fails generation instead of silently
    spawning a partly undamped model.
    """
    common_damping = float(common_damping)
    if not math.isfinite(common_damping) or common_damping < 0.0:
        raise ValueError("common_damping must be finite and non-negative")

    damping_by_joint: Dict[str, float] = {
        name: common_damping for name in ARM_JOINTS
    }
    damping_by_joint[SHOULDER_LIFT_JOINT] = (
        DEFAULT_SHOULDER_LIFT_JOINT_DAMPING
    )
    for name, value in dict(overrides or {}).items():
        if name not in damping_by_joint:
            raise ValueError("unknown UR5 damping override: {}".format(name))
        value = float(value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(
                "damping for {} must be finite and non-negative".format(name)
            )
        damping_by_joint[name] = value

    path = Path(xacro_file).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError("URDF xacro not found: {}".format(path))
    document = xacro.process_file(str(path))

    joints_by_name = {}
    # Transmission elements also contain nested <joint name="..."> tags.
    # Only top-level robot joints own URDF dynamics.
    for joint in _direct_children(document.documentElement, "joint"):
        name = joint.getAttribute("name")
        if name in damping_by_joint:
            if name in joints_by_name:
                raise RuntimeError("duplicate UR5 joint in generated URDF: {}".format(name))
            joints_by_name[name] = joint

    missing = sorted(set(ARM_JOINTS) - set(joints_by_name))
    if missing:
        raise RuntimeError(
            "generated URDF is missing UR5 joint(s): {}".format(
                ", ".join(missing)
            )
        )

    for name in ARM_JOINTS:
        dynamics = _direct_children(joints_by_name[name], "dynamics")
        if len(dynamics) != 1:
            raise RuntimeError(
                "UR5 joint {} has {} dynamics elements; expected exactly one".format(
                    name, len(dynamics)
                )
            )
        dynamics[0].setAttribute("damping", format(damping_by_joint[name], ".12g"))

    return document


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro", required=True, help="Husky/UR5 xacro input")
    parser.add_argument(
        "--arm-joint-damping",
        type=nonnegative_finite,
        default=DEFAULT_ARM_JOINT_DAMPING,
        help="common damping for all six UR5 joints in N*m*s/rad",
    )
    for joint in ARM_JOINTS:
        option = "--{}-damping".format(joint.replace("ur5_", "").replace("_", "-"))
        default = (
            DEFAULT_SHOULDER_LIFT_JOINT_DAMPING
            if joint == SHOULDER_LIFT_JOINT else None
        )
        parser.add_argument(
            option,
            dest="{}_damping".format(joint),
            type=nonnegative_finite,
            default=default,
            help="optional per-joint override in N*m*s/rad",
        )
    return parser


def main() -> int:
    args = _parser().parse_args()
    overrides = {
        joint: getattr(args, "{}_damping".format(joint))
        for joint in ARM_JOINTS
        if getattr(args, "{}_damping".format(joint)) is not None
    }
    try:
        document = generate_urdf_document(
            args.xacro, args.arm_joint_damping, overrides
        )
    except Exception as exc:
        print("URDF damping generation failed: {}".format(exc), file=sys.stderr)
        return 2
    sys.stdout.write(document.toxml())
    return 0


if __name__ == "__main__":
    sys.exit(main())
