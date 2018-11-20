from setuptools import find_packages
from setuptools import setup

package_name = 'xacro'

setup(
    name=package_name,
    python_requires='>=3',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/environment', ['scripts/completion.bash']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robert Haschke',
    maintainer_email='rhaschke@techfak.uni-bielefeld.de',
    url='https://github.com/ros/xacro',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Xacro is an XML macro language.\n'
                'With xacro, you can construct shorter and more readable XML files '
                'by using macros that expand to larger XML expressions.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xacro = xacro:main'
        ],
    },
)
