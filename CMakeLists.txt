cmake_minimum_required(VERSION 2.8.3)
project(xacro)

find_package(catkin REQUIRED COMPONENTS roslint)

catkin_package(
  CFG_EXTRAS xacro-extras.cmake
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_install_python(PROGRAMS scripts/xacro
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## For backwards compatibility we also install as xacro.py
#  The roslaunch $(find ...) expects the script to be in the package directory, 
#  and in catkin the package direcory is /share.
install(PROGRAMS xacro.py DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  ## Add folders to be run by python nosetests
  catkin_add_nosetests(test)
  ## add tests for xacro's cmake functions
  add_subdirectory(test)
endif()

roslint_python()
