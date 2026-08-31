#!/usr/bin/env python3
"""Catkin Python-package definition for ROS-free GroundGrid helpers."""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup_args = generate_distutils_setup(
    packages=["groundgrid"],
    package_dir={"": "src"},
)

setup(**setup_args)
