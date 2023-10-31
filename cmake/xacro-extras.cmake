## xacro_add_xacro_file(<input> [<output>] [REMAP <arg> <arg> ...]
##                      [OUTPUT <variable>] DEPENDS <arg> <arg>)
##
## Creates a command to run xacro on <input> like so:
## xacro -o <output> <input> [<remap args>]
##
## If <output> was not specified, it is determined from <input> removing the suffix .xacro
## The absolute output file name is returned in variable <output>, which defaults to
## XACRO_OUTPUT_FILE.
##
## In order to actually build and install, you need to provide a custom target:
## foreach(xacro_file ${MY_XACRO_FILES})
##   xacro_add_xacro_file(${xacro_file} REMAP bar:=foo foo:=bar)
##   list(APPEND xacro_outputs ${XACRO_OUTPUT_FILE})
## endforeach()
## xacro_install(xacro_target ${xacro_outputs} DESTINATION xml)
##
## Alternatively to xacro_install, you can call
## install(xacro_target ${xacro_outputs} DESTINATION share/${PROJECT_NAME}/xml)
##
## For conveniency, you might want to use xacro_add_files(), which does the same:
## xacro_add_files(${MY_XACRO_FILES} REMAP bar:=foo foo:=bar
##                 TARGET xacro_target INSTALL DESTINATION xml)
function(xacro_add_xacro_file input)
  # parse arguments
  set(options)
  set(oneValueArgs OUTPUT)
  set(multiValueArgs REMAP DEPENDS)
  cmake_parse_arguments(_XACRO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  ## process arguments
  # retrieve output file name
  if(_XACRO_UNPARSED_ARGUMENTS)
    # output file explicitly specified
    list(GET _XACRO_UNPARSED_ARGUMENTS 0 output)
    list(REMOVE_AT _XACRO_UNPARSED_ARGUMENTS 0)
    # any remaining unparsed args?
    if(_XACRO_UNPARSED_ARGUMENTS)
      message(WARNING "unknown arguments: ${_XACRO_UNPARSED_ARGUMENTS}")
    endif(_XACRO_UNPARSED_ARGUMENTS)
  else()
    # implicitly determine output file from input
    if(${input} MATCHES "(.*)[.]xacro$")
      set(output ${CMAKE_MATCH_1})
    else()
      message(FATAL_ERROR "no <output> specified for: " ${input})
    endif()
  endif()

  ## determine absolute output target location
  if(IS_ABSOLUTE ${output})
    set(abs_output ${output})
  else()
    set(abs_output ${CMAKE_CURRENT_BINARY_DIR}/${output})
  endif()

  ## export abs_output to parent scope in variable ${_XACRO_OUTPUT}
  if(NOT _XACRO_OUTPUT)
    set(_XACRO_OUTPUT XACRO_OUTPUT_FILE)
  endif()
  set(${_XACRO_OUTPUT} ${abs_output} PARENT_SCOPE)

  ## Call out to xacro to determine dependencies
  message(STATUS "xacro: determining deps for: " ${input} " ...")
  execute_process(COMMAND xacro --deps ${input} ${_XACRO_REMAP}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE _xacro_result
    ERROR_VARIABLE _xacro_err
    OUTPUT_VARIABLE _xacro_deps_result
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_xacro_result)
    message(WARNING "failed to determine deps for: ${input}
${_xacro_err}")
  endif(_xacro_result)

  separate_arguments(_xacro_deps_result)

  ## HACK: ament package resolution doesn't work at build time yet
  # - Augment AMENT_PREFIX_PATH to include ${PROJECT_BINARY_DIR}/ament_cmake_index
  # - Create a symlink from there to the actual source directory to find source files
  set(PROJECT_BUILD_INDEX "${PROJECT_BINARY_DIR}/ament_cmake_index")
  execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory "${PROJECT_BUILD_INDEX}/share")
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink "${PROJECT_SOURCE_DIR}" "${PROJECT_BUILD_INDEX}/share/${PROJECT_NAME}")

  ## command to actually call xacro
  list(JOIN AMENT_PREFIX_PATH ":" AMENT_PREFIX_PATH_ENV) # format as colon-separated list
  add_custom_command(OUTPUT ${abs_output}
    COMMAND ${CMAKE_COMMAND} -E env AMENT_PREFIX_PATH="${PROJECT_BUILD_INDEX}:${AMENT_PREFIX_PATH_ENV}" xacro -o ${abs_output} ${input} ${_XACRO_REMAP}
    DEPENDS ${input} ${_xacro_deps_result} ${_XACRO_DEPENDS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "xacro: generating ${output} from ${input}"
    )
endfunction(xacro_add_xacro_file)


## xacro_install(<target> <output> [<output> ...] DESTINATION <path>)
##
## installs xacro-generated files into share/<package>/<path>
function(xacro_install target)
  # parse arguments
  set(oneValueArgs DESTINATION)
  cmake_parse_arguments(_XACRO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  set(outputs ${_XACRO_UNPARSED_ARGUMENTS})

  ## normal install
  install(FILES ${outputs} DESTINATION share/${PROJECT_NAME}/${_XACRO_DESTINATION})
endfunction(xacro_install)


## xacro_add_files(<file> [<file> ...] [REMAP <arg> <arg> ...] [DEPENDS <arg> <arg>]
##                 [TARGET <target>] [INSTALL [DESTINATION <path>]])
##
## create make <target> to generate xacro files and optionally install to share/<package>/<path>
function(xacro_add_files)
  # parse arguments
  set(options INSTALL)
  set(oneValueArgs OUTPUT TARGET DESTINATION)
  set(multiValueArgs REMAP DEPENDS)
  cmake_parse_arguments(_XACRO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  ## process arguments
  # prepare REMAP args (prepending REMAP)
  if(_XACRO_REMAP)
    set(_XACRO_REMAP REMAP ${_XACRO_REMAP})
  endif()
  # prepare DEPENDS args (prepending DEPENDS)
  if(_XACRO_DEPENDS)
    set(_XACRO_DEPENDS DEPENDS ${_XACRO_DEPENDS})
  endif()

  # have INSTALL option, but no TARGET: fallback to default target
  if(_XACRO_INSTALL AND NOT _XACRO_TARGET)
    set(_XACRO_TARGET _xacro_auto_generate)
  endif()

  foreach(input ${_XACRO_UNPARSED_ARGUMENTS})
    # call to main function
    xacro_add_xacro_file(${input} ${_XACRO_OUTPUT} ${_XACRO_REMAP} ${_XACRO_DEPENDS})
    list(APPEND outputs ${XACRO_OUTPUT_FILE})
  endforeach()

  if(outputs)
    # link to target
    add_custom_target(${PROJECT_NAME}_${_XACRO_TARGET} ALL DEPENDS ${outputs})

    # install?
    if(_XACRO_INSTALL)
      xacro_install(${_XACRO_TARGET} ${outputs} DESTINATION ${_XACRO_DESTINATION})
    endif(_XACRO_INSTALL)
  endif()
endfunction(xacro_add_files)
