cmake_minimum_required(VERSION 3.5)
project(xacro)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(FILES scripts/completion.bash DESTINATION share/${PROJECT_NAME}/environment)

## The following manually installs the python package and the entry-point script
## Using ament_cmake, we cannot actually use setup.py to install these...

# install xacro python package w/ console_script entrypoint
ament_python_install_package(${PROJECT_NAME}
    SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# copy generated script executables to bin dir so it can be run without ros2 run
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/scripts/
    DESTINATION bin/)

if(BUILD_TESTING)
  ## run linters as defined in package.xml
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ## add tests for xacro's cmake functions
  add_subdirectory(test)
endif()

ament_package(
  CONFIG_EXTRAS cmake/xacro-extras.cmake
)
