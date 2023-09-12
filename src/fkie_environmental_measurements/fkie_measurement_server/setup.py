#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['nodes/measurement_server_node.py'],
    packages=['fkie_measurement_server'],
    package_dir={'': 'src'}
)

setup(**d)
