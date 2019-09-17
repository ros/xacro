#!/usr/bin/env python3

from setuptools import find_packages
from setuptools import setup

package_name = 'xacro'

setup(
    name=package_name,
    version='1.13.4',
    python_requires='>=3',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Robert Haschke',
    author_email='rhaschke@techfak.uni-bielefeld.de',
    maintainer='Robert Haschke',
    maintainer_email='rhaschke@techfak.uni-bielefeld.de',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Xacro is an XML macro language. With xacro,'
    'you can construct shorter and more readable XML files'
    'by using macros that expand to larger XML expressions.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    test_suite='test',
    entry_points={
        'console_scripts': [
            'xacro = xacro:main'
        ],
    },
)
