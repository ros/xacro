@[if DEVELSPACE]@
set(_xacro_py
  @(CMAKE_CURRENT_SOURCE_DIR)/scripts/xacro
)
@[else]@
set(_xacro_py
  ${xacro_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/xacro
)
@[end if]@

macro(xacro_add_xacro_file input output)
  # Call out to xacro to get dependencies
  execute_process(COMMAND ${_xacro_py} --deps ${input}
    ERROR_VARIABLE _xacro_err_ignore
    OUTPUT_VARIABLE _xacro_deps_result
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  separate_arguments(_xacro_deps_result)

  add_custom_command(OUTPUT ${output}
    COMMAND ${_xacro_py}
    ARGS ${input} > ${output}
    DEPENDS ${input} ${_xacro_deps_result})
endmacro(xacro_add_xacro_file)
