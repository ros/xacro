#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['xacro'],
    package_dir={'': 'src'},
    scripts=['scripts/xacro']
)

setup(**d)
