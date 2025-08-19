cmake_minimum_required(VERSION 3.12)
project(xacro)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(FILES scripts/completion.bash DESTINATION share/${PROJECT_NAME}/environment)

# install xacro python package and it's console_script entry-point script
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# install entry-point script(s) in bin as well
install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ament_cmake_python/${PROJECT_NAME}/scripts/
    DESTINATION bin/
    USE_SOURCE_PERMISSIONS)

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
