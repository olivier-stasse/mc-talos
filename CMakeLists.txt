cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

project(mc_talos CXX)

include(generate_urdf.cmake)

find_package(mc_rtc REQUIRED)
find_package(talos_description REQUIRED)

# Generate URDF
set(robot_name "talos_full_v2")
set(xacro_src "")
set(xacro_args "--used-fixed_base:=false")
set(xacro_args "${xacro_args} --use_sim:=False")
set(xacro_args "${xacro_args} --test:=False")
set(xacro_args "${xacro_args} --robot_model:=full_v2")
set(xacro_args "${xacro_args} --enable_crane:=False")
set(xacro_args "${xacro_args} --foot_collision:=thinbox")
set(xacro_args "${xacro_args} --disable_gazebo_camera:=true")
set(xacro_args "${xacro_args} --head_type:=default")
set(xacro_args "${xacro_args} --flexibility:=False")
set(xacro_args "${xacro_args} --default_configuration_type:=False")

set(robot_root_xacro robots/talos_full_v2.urdf.xacro )

generate_mc_rtc_urdf(talos_description xacro_src robot_name robot_root_xacro xacro_args)

if("${talos_description_DIR}" STREQUAL "")
  message(FATAL_ERROR "Your talos_description package does not define where to find the data")
endif()
set(TALOS_DESCRIPTION_PATH "${talos_description_DIR}/share/talos_description/robots")

set(TALOS_URDF_PATH "${TALOS_DESCRIPTION_PATH}/talos_full_v2.urdf")
set(TALOS_RSDF_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/rsdf")
install(DIRECTORY rsdf DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
set(TALOS_CONVEX_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/convex")
install(DIRECTORY convex DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

message("-- TALOS_DESCRIPTION_PATH: ${TALOS_DESCRIPTION_PATH}")
message("-- TALOS_URDF_PATH: ${TALOS_URDF_PATH}")

configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

add_robot(talos src/talos.cpp src/talos.h)
target_include_directories(talos PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
