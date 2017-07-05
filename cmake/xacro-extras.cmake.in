@[if DEVELSPACE]@
set(_xacro_py
  @(CMAKE_CURRENT_SOURCE_DIR)/scripts/xacro
  )
@[else]@
set(_xacro_py
  ${xacro_DIR}/../../../@(CATKIN_PACKAGE_BIN_DESTINATION)/xacro
  )
@[end if]@

add_custom_target(${PROJECT_NAME}_xacro_generated_to_devel_space_ ALL)


## xacro_add_xacro_file(<input> [<output>] [INORDER] [REMAP <arg> <arg> ...]
##                      [OUTPUT <variable>])
##
## Creates a command run xacro on <input> like so:
## xacro [--inorder] -o <output> <input> [<remap args>]
##
## If <output> was not specified, it is determined from <input> removing the suffix .xacro
## The absolute output file name is returned in variable <output>, which defaults to
## XACRO_OUTPUT_FILE.
##
## In order to actually build and install, you need to provide a custom target:
## foreach(xacro_file ${MY_XACRO_FILES})
##   xacro_add_xacro_file(${xacro_file} INORDER REMAP bar:=foo foo:=bar)
##   list(APPEND xacro_outputs ${XACRO_OUTPUT_FILE})
## endforeach()
## xacro_install(xacro_target ${xacro_outputs} DESTINATION xml)
##
## Be aware, that xacro_install() is required to install into both, install and devel space.
## Normal install() only installs into install space!
##
## For conveniency, you might want to use xacro_add_files(), which does the same:
## xacro_add_files(${MY_XACRO_FILES} INORDER REMAP bar:=foo foo:=bar
##                 TARGET xacro_target INSTALL DESTINATION xml)
function(xacro_add_xacro_file input)
  # parse arguments
  set(options INORDER)
  set(oneValueArgs OUTPUT)
  set(multiValueArgs REMAP)
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
  # message(STATUS "output: ${output}")

  # transform _XACRO_INORDER's BOOL value
  if(_XACRO_INORDER)
    set(_XACRO_INORDER "--inorder")
  else()
    unset(_XACRO_INORDER)
  endif()

  ## determine absolute output target location
  if(IS_ABSOLUTE ${output})
    set(abs_output ${output})
  else()
    set(abs_output ${CMAKE_CURRENT_BINARY_DIR}/${output})
  endif()
  # message(STATUS "abs_output: ${abs_output}")

  ## export abs_output to parent scope in variable ${_XACRO_OUTPUT}
  if(NOT _XACRO_OUTPUT)
    set(_XACRO_OUTPUT XACRO_OUTPUT_FILE)
  endif()
  set(${_XACRO_OUTPUT} ${abs_output} PARENT_SCOPE)

  ## Call out to xacro to determine dependencies
  message(STATUS "xacro: determining deps for: " ${input} " ...")
  execute_process(COMMAND ${CATKIN_ENV} ${_xacro_py} ${_XACRO_INORDER} --deps ${input} ${_XACRO_REMAP}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ERROR_VARIABLE _xacro_err
    OUTPUT_VARIABLE _xacro_deps_result
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(_xacro_err)
    message(WARNING "failed to determine deps for: ${input}
${_xacro_err}")
  endif(_xacro_err)

  separate_arguments(_xacro_deps_result)

  ## command to actually call xacro
  add_custom_command(OUTPUT ${output}
    COMMAND ${CATKIN_ENV} ${_xacro_py} ${_XACRO_INORDER} -o ${abs_output} ${input} ${_XACRO_REMAP}
    DEPENDS ${input} ${_xacro_deps_result}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "xacro: generating ${output} from ${input}"
    )
endfunction(xacro_add_xacro_file)


## xacro_install(<target> <output> [<output> ...] DESTINATION <path>)
##
## installs xacro-generated files both to devel and input space
## into ${CATKIN_PACKAGE_SHARE_DESTINATION}/<path>
function(xacro_install target)
  # parse arguments
  set(oneValueArgs DESTINATION)
  cmake_parse_arguments(_XACRO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
  set(outputs ${_XACRO_UNPARSED_ARGUMENTS})

  ## rule to create target dir
  file(TO_CMAKE_PATH ${CATKIN_PACKAGE_SHARE_DESTINATION}/${_XACRO_DESTINATION} dest)
  file(TO_CMAKE_PATH ${CATKIN_DEVEL_PREFIX}/${dest} TARGET_DIR)
  add_custom_command(OUTPUT ${TARGET_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${TARGET_DIR}
    COMMENT "creating dir ${TARGET_DIR}")

  ## process list of outputs
  foreach(output ${outputs})
    get_filename_component(tgt ${output} NAME)
    file(TO_CMAKE_PATH ${TARGET_DIR}/${tgt} tgt)
    list(APPEND tgts ${tgt})

    # rule to create devel space tgt
    add_custom_command(OUTPUT ${tgt}
      COMMAND ${CMAKE_COMMAND} -E copy_if_different ${output} ${tgt}
      DEPENDS ${TARGET_DIR} ${output}
      COMMENT "Copying to devel space: ${tgt}"
      )
  endforeach()

  ## rule to create devel space tgts
  add_custom_target(${PROJECT_NAME}_${target}_to_devel_space_ DEPENDS ${tgts})
  # add to main target _xacro_generated_to_devel_space_
  add_dependencies(${PROJECT_NAME}_xacro_generated_to_devel_space_
                   ${PROJECT_NAME}_${target}_to_devel_space_)


  ## normal install
  install(FILES ${outputs} DESTINATION ${dest})
endfunction(xacro_install)


## xacro_add_files(<file> [<file> ...] [INORDER] [REMAP <arg> <arg> ...]
##                 [TARGET <target>] [INSTALL [DESTINATION <path>]])
##
## create make <target> to generate xacro files
## and optionally install to ${CATKIN_PACKAGE_SHARE_DESTINATION}/<path>
## in devel and install space.
function(xacro_add_files)
  # parse arguments
  set(options INORDER INSTALL)
  set(oneValueArgs OUTPUT TARGET DESTINATION)
  set(multiValueArgs REMAP)
  cmake_parse_arguments(_XACRO "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  ## process arguments
  # transform _XACRO_INORDER's BOOL value
  if(_XACRO_INORDER)
    set(_XACRO_INORDER "INORDER")
  else()
    unset(_XACRO_INORDER)
  endif()

  # prepare REMAP args (prepending REMAP)
  if(_XACRO_REMAP)
    set(_XACRO_REMAP REMAP ${_XACRO_REMAP})
  endif()

  # have INSTALL option, but no TARGET: fallback to default target
  if(_XACRO_INSTALL AND NOT _XACRO_TARGET)
    # message(STATUS "xacro: no TARGET specified, using default")
    set(_XACRO_TARGET _xacro_auto_generate)
  endif()

  foreach(input ${_XACRO_UNPARSED_ARGUMENTS})
    # call to main function
    xacro_add_xacro_file(${input} ${_XACRO_OUTPUT} ${_XACRO_INORDER} ${_XACRO_REMAP})
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
