cmake_minimum_required(VERSION 3.8)
project(moying_mcr_arm_controller)

# if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
#   add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
# endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  moying_mcr_arm_dynamics_models
  std_srvs
  std_msgs
  pinocchio
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pinocchio REQUIRED) 
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Add velocity_interface_controller library related compile commands
generate_parameter_library(velocity_interface_controller_parameters
  src/velocity_interface_controller.yaml
  include/velocity_interface_controller/validate_velocity_interface_controller_parameters.hpp
)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/velocity_interface_controller.cpp
  src/gravity_compensation.cpp
  src/pinocchio_gravity_compensation.cpp
  src/KalmanFilter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} velocity_interface_controller_parameters)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "VELOCITY_INTERFACE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface velocity_interface_controller.xml)

install(
  TARGETS
  ${PROJECT_NAME}
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
  ${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "cmake/pinocchio_config.cmake")

