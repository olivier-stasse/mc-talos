cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

project(mc_talos CXX)

include(generate_urdf.cmake)

find_package(mc_rtc REQUIRED)
find_package(talos_description REQUIRED)

# Generate URDF
set(robot_name "talos_full_v2")
set(xacro_src "")
set(xacro_args
  "used-fixed_base:=false"
  "use_sim:=False"
  "test:=False"
  "robot_model:=full_v2"
  "enable_crane:=False"
  "foot_collision:=thinbox"
  "disable_gazebo_camera:=true"
  "head_type:=default"
  "flexibility:=False"
  "default_configuration_type:=False"
)

set(robot_root_xacro "robots/talos_full_v2.urdf.xacro")

generate_mc_rtc_urdf(talos_description "${xacro_src}" "${robot_name}" "${robot_root_xacro}" "${xacro_args}")

if("${talos_description_DIR}" STREQUAL "")
  message(FATAL_ERROR "Your talos_description package does not define where to find the data")
endif()

# Set variables for generated URDF file.
set(MC_RTC_TALOS_DESCRIPTION_PATH "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
set(MC_RTC_TALOS_URDF_PATH "${MC_RTC_TALOS_DESCRIPTION_PATH}/urdf/${robot_name}.urdf")
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/urdf DESTINATION ${MC_RTC_TALOS_DESCRIPTION_PATH})

set(MC_RTC_TALOS_RSDF_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/rsdf")
install(DIRECTORY rsdf DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
set(MC_RTC_TALOS_CONVEX_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/convex")
install(DIRECTORY convex DESTINATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

message("-- TALOS_DESCRIPTION_PATH: ${MC_RTC_TALOS_DESCRIPTION_PATH}")
message("-- TALOS_URDF_PATH: ${MC_RTC_TALOS_URDF_PATH}")

configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

# Generate robot module
add_robot(talos src/talos.cpp src/talos.h)
target_include_directories(talos PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")

# Install LIPMWalking file for talos.
install(FILES etc/mc_controllers/LIPMWalking/talos.yaml
  DESTINATION ${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/LIPMWalking/)

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
