# Run pytest
find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(pytest . WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_test(NAME xacro_cmake
  # run test-cmake.sh for subdir test-xacro-cmake
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test-cmake.sh ${CMAKE_CURRENT_SOURCE_DIR}/test-xacro-cmake
  )
