#!/usr/bin/python3.5

from setuptools import find_packages
from setuptools import setup

package_name = 'xacro'

setup(
    name=package_name,
    version='1.13.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name, [
            'resource/completion.bash'
        ]),
    ],
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
    description='Xacro is an XML macro language. With xacro, you can construct shorter and more readable XML files by using macros that expand to larger XML expressions.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'xacro.xacro': [
            'cli = xacro.xacro.cli:ColoredOptionParser',
        ],
        'console_scripts': [
            'xacro = resource.xacro:main'
        ],
    },
)
