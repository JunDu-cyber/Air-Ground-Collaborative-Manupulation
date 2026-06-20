#!/usr/bin/env python3
"""Helpers for locating PX4-Autopilot assets without hardcoding a user path.

The PX4 root is resolved from the ``PX4_AUTOPILOT_PATH`` environment variable,
falling back to ``~/PX4-Autopilot``. Every consumer also exposes the result as
a ROS param, so it can still be overridden per-launch.
"""

import os

# Relative path of the iris mesh inside a PX4-Autopilot checkout.
_IRIS_MESH_REL = (
    "Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    "/models/iris/meshes/iris.stl"
)


def px4_root():
    """Return the PX4-Autopilot root directory."""
    return os.environ.get(
        "PX4_AUTOPILOT_PATH", os.path.expanduser("~/PX4-Autopilot")
    )


def default_iris_mesh_path():
    """Return the filesystem path to the iris STL mesh (no URI scheme)."""
    return os.path.join(px4_root(), _IRIS_MESH_REL)


def default_iris_mesh_resource():
    """Return a ``file://`` URI to the iris STL mesh in the PX4 checkout."""
    return "file://" + default_iris_mesh_path()
