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


macro(generate_mc_rtc_urdf robot_description xacro_src robot_name robot_root_xacro xacro_args)
  find_package(ament_cmake REQUIRED)
  find_package(${robot_description} REQUIRED)

  set(
  message(STATUS "${robot_description} path ${${robot_description}_DIR}")

  # Checks that robot_description is correct
  if("${${robot_description}_INSTALL_PREFIX}" STREQUAL "")
    if("${${robot_description}_SOURCE_PREFIX}" STREQUAL "")
      if("${${robot_description}_DIR}" STREQUAL "")
        message(
          FATAL_ERROR
          "${robot_description} does not provide SOURCE_PREFIX or INSTALL_PREFIX")
      else()
        set(${robot_description}_PREFIX "${${robot_description}_DIR}/..")
      endif()
    else()
      set(${robot_description}_PREFIX "${${robot_description}_SOURCE_PREFIX}")
    endif()
  else()
    set(${robot_description}_PREFIX
      "${${robot_description}_INSTALL_PREFIX}/share/${robot_description}")
  endif()

  # Prepare the target directory
  set(DATA_INSTALL_FOLDER "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

  # Search for xacro
  find_program(XACRO xacro REQUIRED)

  # Build input and output arguments to call xacro
  set(urdf_OUT "${CMAKE_CURRENT_BUILD_DIR}/urdf/${robot_name}.urdf")
  
  set(xacro_IN ${${robot_description}_PREFIX}/urdf/ur.urdf.xacro)
  set(xacro_SRC ${xacro_SRC} ${xacro_IN}
                ${${robot_description}_PREFIX}/${robot_root_xacro})

  # Call xacro
  add_custom_command(
    OUTPUT ${urdf_OUT}
    COMMAND ${XACRO} ${xacro_IN} ${xacro_args} -o ${urdf_OUT}
    DEPENDS ${xacro_SRC}
    COMMENT "Generate ${urdf_OUT}")

  # Install
  install(FILES "${urdf_OUT}" DESTINATION ${DATA_INSTALL_FOLDER}/urdf/)
  
endmacro()
