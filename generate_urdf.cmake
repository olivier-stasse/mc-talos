# This macro generates URDF files statically from a ROS 2 package
# named ${robot_description}.
# The macro uses xacro_src as the files to take into account
# based on the ${robot_description}_DIR variable provided
# by the package robot_description.
#
# The generated URDF will be installed in
# ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
#
# The xacro command which is used to generate the URDF file
# takes xacro_args in argument and parse the file
# ${${robot_description}_PREFIX}/${robot_root_xacro}

function(dump_cmake_variables)
    get_cmake_property(_variableNames VARIABLES)
    list (SORT _variableNames)
    foreach (_variableName ${_variableNames})
        if ((NOT DEFINED ARGV0) OR _variableName MATCHES ${ARGV0})
            message(STATUS "${_variableName}=${${_variableName}}")
        endif()
    endforeach()
endfunction()

macro(generate_mc_rtc_urdf )
  set(lrobot_description ${ARGV0})
  set(lxacro_src ${ARGV1})
  set(lrobot_name ${ARGV2})
  set(lrobot_root_xacro ${ARGV3})
  set(lxacro_args ${ARGV4})

  message(STATUS "lrobot_description: ${lrobot_description}")
  message(STATUS "lxacro_src: ${lxacro_src}")
  message(STATUS "lrobot_name: ${lrobot_name}")
  message(STATUS "lrobot_root_xacro ${lrobot_root_xacro}")
  message(STATUS "lxacro_args ${lxacro_args}")    
  
  find_package(ament_cmake REQUIRED)
  find_package(${lrobot_description} REQUIRED)

  dump_cmake_variables(${lrobot_description})

  message(STATUS "${lrobot_description} path ${${lrobot_description}_DIR}")

  # Checks that robot_description is correct
  if("${${lrobot_description}_INSTALL_PREFIX}" STREQUAL "")
    if("${${lrobot_description}_SOURCE_PREFIX}" STREQUAL "")
      if("${${lrobot_description}_DIR}" STREQUAL "")
        message(
          FATAL_ERROR
          "${lrobot_description} does not provide SOURCE_PREFIX or INSTALL_PREFIX")
      else()
        set(${lrobot_description}_PREFIX "${${lrobot_description}_DIR}/..")
      endif()
    else()
      set(${lrobot_description}_PREFIX "${${lrobot_description}_SOURCE_PREFIX}")
    endif()
  else()
    set(${lrobot_description}_PREFIX
      "${${lrobot_description}_INSTALL_PREFIX}/share/${lrobot_description}")
  endif()

  # Prepare the target directory
  set(DATA_INSTALL_FOLDER "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

  # Search for xacro
  find_program(XACRO xacro REQUIRED)

  # Build input and output arguments to call xacro
  set(urdf_OUT "${CMAKE_CURRENT_BINARY_DIR}/urdf/${lrobot_name}.urdf")

  set(xacro_IN ${${lrobot_description}_PREFIX}/${lrobot_root_xacro})
  set(xacro_SRC ${lxacro_src} ${xacro_IN})

  message(STATUS "urdf_out: ${urdf_OUT}")
  message(STATUS "XACRO:${XACRO}")
  message(STATUS "xacro_in: ${xacro_IN}")
  message(STATUS "xacro_SRC: ${xacro_SRC}")  
  message(STATUS "xacro_args: ${lxacro_args}")
  message(STATUS "urdf_OUT: ${urdf_OUT}")  
  
  # Call xacro
  add_custom_command(
    OUTPUT ${urdf_OUT}
    COMMAND ${XACRO} ${xacro_SRC} ${xacro_args} -o ${urdf_OUT}
    DEPENDS ${xacro_SRC}
    COMMENT "Generate ${urdf_OUT}")
  add_custom_target(generate_urdf ALL DEPENDS ${urdf_OUT})

  # Install
  install(FILES "${urdf_OUT}" DESTINATION ${DATA_INSTALL_FOLDER}/urdf/)

endmacro()
