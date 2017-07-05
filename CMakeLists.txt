cmake_minimum_required(VERSION 3.5)
project(xacro)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/xacro DESTINATION bin)

## For backwards compatibility we also install as xacro.py
#  The roslaunch $(find ...) expects the script to be in the package directory, 
#  and in catkin the package direcory is /share.
install(PROGRAMS xacro.py DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
    
  ## Add folders to be run by python nosetests
  ament_add_nosetest(${PROJECT_NAME}_nose_tests test)
  ## add tests for xacro's cmake functions
  add_subdirectory(test)
endif()


ament_package(
  CONFIG_EXTRAS cmake/xacro-extras.cmake.in
)
