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
  message(STATUS "xacro: determining deps for: " ${input} " ...")
  execute_process(COMMAND ${_xacro_py} --deps ${input_abs} ${_XACRO_REMAP}
    ERROR_VARIABLE _xacro_err
    OUTPUT_VARIABLE _xacro_deps_result
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_xacro_err)
    message(WARNING "failed to determine deps for: ${input}
${_xacro_err}")
  endif(_xacro_err)

  separate_arguments(_xacro_deps_result)

  add_custom_command(OUTPUT ${output}
    COMMAND echo ${_xacro_py} ${_XACRO_INORDER} -o ${output} ${input_abs} ${_XACRO_REMAP}
    COMMAND ${CATKIN_ENV} ${_xacro_py} ${_XACRO_INORDER} -o ${output} ${input_abs} ${_XACRO_REMAP}
    DEPENDS ${input_abs} ${_xacro_deps_result})
endmacro(xacro_add_xacro_file)

macro(xacro_add_target input output)
  xacro_add_xacro_file(${input} ${output} ${ARGN})
  # create target name from ${output} (which might contain directory separators)
  file(TO_CMAKE_PATH ${output} _target_name)
  string(REPLACE "/" "_" _target_name _auto_gen_${_target_name})
  add_custom_target(${_target_name} ALL DEPENDS ${output})
endmacro(xacro_add_target)
