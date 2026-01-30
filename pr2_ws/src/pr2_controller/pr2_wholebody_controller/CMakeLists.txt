cmake_minimum_required(VERSION 3.8)
project(pr2_wholebody_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  angles
  std_srvs
  std_msgs
  pr2_wholebody_interface
  ocs2_ros_interfaces
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_self_collision_visualization
  ocs2_mobile_manipulator
  geometry_msgs
  Boost
  tf2
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add pr2_wholebody_controller library related compile commands
generate_parameter_library(pr2_wholebody_controller_parameters
  src/pr2_wholebody_controller.yaml
  include/pr2_wholebody_controller/validate_pr2_wholebody_controller_parameters.hpp
)
add_library(
  pr2_wholebody_controller
  SHARED
  src/pr2_wholebody_controller.cpp
)
target_include_directories(pr2_wholebody_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(pr2_wholebody_controller pr2_wholebody_controller_parameters)
ament_target_dependencies(pr2_wholebody_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(pr2_wholebody_controller PRIVATE "MMCO_DUAL_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface pr2_wholebody_controller.xml)

install(
  TARGETS
  pr2_wholebody_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  pr2_wholebody_controller
)
ament_package()
