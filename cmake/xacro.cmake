rosbuild_find_ros_package(xacro)
set(_xacro_py ${xacro_PACKAGE_PATH}/xacro.py)

macro(xacro_add_xacro_file input output)
  # Call out to xacro to get dependencies
  execute_process(COMMAND ${_xacro_py} ${input}
    ERROR_VARIABLE _xacro_err
    OUTPUT_FILE ${output}
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if (_xacro_err)
    message(SEND_ERROR "Error occured while trying to execute xacro: ${_xacro_err}")
  else (_xacro_err)
    #message(STATUS " - Xacro processing results: ${_xacro_deps_result}")
    separate_arguments(_xacro_deps_result)

    add_custom_command(OUTPUT ${output}
      COMMAND ${_xacro_py}
      ARGS ${input} > ${output}
      DEPENDS ${input} ${_xacro_deps_result})
  endif (_xacro_err)
endmacro(xacro_add_xacro_file)
