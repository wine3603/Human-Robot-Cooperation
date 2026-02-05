cmake_minimum_required(VERSION 3.8)
project(mmco_dual_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
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
  mmco_dual_interface
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

# Add mmco_dual_controller library related compile commands
generate_parameter_library(mmco_dual_controller_parameters
  src/mmco_dual_controller.yaml
  include/mmco_dual_controller/validate_mmco_dual_controller_parameters.hpp
)
add_library(
  mmco_dual_controller
  SHARED
  src/mmco_dual_controller.cpp
)
target_include_directories(mmco_dual_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mmco_dual_controller mmco_dual_controller_parameters)
ament_target_dependencies(mmco_dual_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(mmco_dual_controller PRIVATE "MMCO_DUAL_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface mmco_dual_controller.xml)

install(
  TARGETS
  mmco_dual_controller
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
  mmco_dual_controller
)

ament_package()
