cmake_minimum_required(VERSION 2.8.3)
project(xacro)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roslaunch
  CFG_EXTRAS xacro-extras.cmake
)

install(FILES xacro.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Add folders to be run by python nosetests
catkin_add_nosetests(test)
