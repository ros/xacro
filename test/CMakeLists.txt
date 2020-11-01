# Create dummy ament_index for pytest, such that $(find xacro)/test will point to the current folder
set(TEST_AMENT_INDEX "${CMAKE_CURRENT_BINARY_DIR}/test_ament_index")
file(GENERATE OUTPUT "${TEST_AMENT_INDEX}/share/ament_index/resource_index/packages/${PROJECT_NAME}" CONTENT "")
file(MAKE_DIRECTORY "${TEST_AMENT_INDEX}/share/${PROJECT_NAME}")
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
  "${CMAKE_CURRENT_SOURCE_DIR}" "${TEST_AMENT_INDEX}/share/${PROJECT_NAME}/test")

# Finally, run pytest
find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(pytest .
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  ENV AMENT_PREFIX_PATH=${CMAKE_CURRENT_BINARY_DIR}/test_ament_index:$ENV{AMENT_PREFIX_PATH}
  )

add_test(NAME xacro_cmake
  # run test-cmake.sh for subdir test-xacro-cmake
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test-cmake.sh ${CMAKE_CURRENT_SOURCE_DIR}/test-xacro-cmake
  )
