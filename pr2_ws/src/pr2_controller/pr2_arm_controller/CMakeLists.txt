cmake_minimum_required(VERSION 3.8)
project(pr2_arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  baichuan_arm_dynamics_models
  std_srvs
  std_msgs
  pinocchio
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pinocchio REQUIRED) 
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Add pr2_arm_velocity_controller library related compile commands
generate_parameter_library(pr2_arm_velocity_controller_parameters
  src/pr2_arm_velocity_controller.yaml
  include/pr2_arm_controller/validate_pr2_arm_velocity_controller_parameters.hpp
)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

add_library(
  pr2_arm_velocity_controller
  SHARED
  src/pr2_arm_velocity_controller.cpp
)
target_include_directories(pr2_arm_velocity_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(pr2_arm_velocity_controller pr2_arm_velocity_controller_parameters)
ament_target_dependencies(pr2_arm_velocity_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(pr2_arm_velocity_controller PRIVATE "VELOCITY_INTERFACE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface pr2_arm_velocity_controller.xml)

install(
  TARGETS
  pr2_arm_velocity_controller
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
  pr2_arm_velocity_controller
)

ament_package()


