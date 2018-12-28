This file describes the work done and steps to perform test on the xacro package.

"https://github.com/vandanamandlik/xacro/tree/ros2"


ROS2 Migration changes

	The basic concept and design are same as ROS1.
	Work Done by referring ROS1 indigo-devel branch of xacro package.
	All changes for migration have been done as per Migration guide.
		Migrated CMakeLists.txt and package.xml in ROS2 style.
		Python 2.7 to Python 3 migration done. 
		directory structure changes as per ros2 python pacakges.

Build proccedure and testing on Bouncy

1. Get pacakge at local system
	1.1 # mkdir -p xacro_ws/src

	1.2 # cd xacro_ws/src

	1.3 # git clone git@github.com:vandanamandlik/xacro.git -b ros2

	1.4 # source /opt/ros/bouncy/setup.sh



2. Build the package
	2.1 # cd ../
	2.2 # colcon build
