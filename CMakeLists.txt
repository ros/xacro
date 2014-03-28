cmake_minimum_required(VERSION 2.8.3)
project(xacro)

find_package(catkin REQUIRED roslint)

catkin_python_setup()

catkin_package(
  CFG_EXTRAS xacro-extras.cmake
)

# this script is just for backwards compatibility. the roslaunch $(find ...) 
# expects the script to be in the package directory, and in catkin
# the package direcory is the share directory
install(PROGRAMS xacro.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/xacro DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  ## Add folders to be run by python nosetests
  catkin_add_nosetests(test)
endif()

roslint_python()
