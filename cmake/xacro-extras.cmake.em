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
  set(options OPTIONAL INORDER)
  set(multiValueArgs REMAP)
  cmake_parse_arguments(_XACRO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  if(_XACRO_INORDER)
    set(_XACRO_INORDER "--inorder")
  else()
    unset(_XACRO_INORDER)
  endif()

  # create absolute input filename (if not yet absolute)
  get_filename_component(input_abs ${input} ABSOLUTE)

  # Call out to xacro to get dependencies
  execute_process(COMMAND ${_xacro_py} --deps ${input_abs} ${_XACRO_REMAP}
    ERROR_VARIABLE _xacro_err_ignore
    OUTPUT_VARIABLE _xacro_deps_result
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  separate_arguments(_xacro_deps_result)

  # stringify _XACRO_REMAP: turning list into a space-separated string
  foreach(remap ${_XACRO_REMAP})
    set(_xacro_remap_args "${_xacro_remap_args} ${remap}")
  endforeach(remap)

  add_custom_command(OUTPUT ${output}
    COMMAND echo ${_xacro_py} ${_XACRO_INORDER} -o ${output} ${input_abs} ${_XACRO_REMAP}
    COMMAND ${CATKIN_ENV} ${_xacro_py} ${_XACRO_INORDER} -o ${output} ${input_abs} ${_XACRO_REMAP}
    DEPENDS ${input_abs} ${_xacro_deps_result})
endmacro(xacro_add_xacro_file)
