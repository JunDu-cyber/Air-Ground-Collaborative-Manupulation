#!/usr/bin/env python3
"""Python package installation for mine_grasp_minimal helpers."""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["mine_grasp_minimal"],
    package_dir={"": "src"},
)

setup(**setup_args)
