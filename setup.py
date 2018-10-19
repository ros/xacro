#!/usr/bin/env python
from setuptools import find_packages
from setuptools import setup

package_name = 'xacro'

setup(
    name=package_name,
    version='1.13.4',
    packages=find_packages(exclude=['test']),
    py_modules=['xacro'],
    #data_files=[
    #    ('share/ament_index/resource_index/packages',
    #        ['resource/' + package_name]),
    #    ('share/' + package_name, ['package.xml']),
    #],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Robert Haschke',
    author_email='rhaschke@techfak.uni-bielefeld.de',
    maintainer='Robert Haschke',
    maintainer_email='rhaschke@techfak.uni-bielefeld.de',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Xacro is an XML macro language. With xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions.',
    license='',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'xacro = xacro:main'
        ],
    },
)
