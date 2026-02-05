cmake_minimum_required(VERSION 3.8)
project(moying_mcr_hardware)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(elfin_ethercat_driver_plant REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moying_mcr_arm_dynamics_models REQUIRED)

add_library(
  moying_mcr_hardware
  SHARED
  src/arm_hardware_interface.cpp
  src/arm_ethercat_driver.cpp
)
target_include_directories(
  moying_mcr_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  moying_mcr_hardware

  ament_index_cpp
  elfin_ethercat_driver_plant
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  moying_mcr_arm_dynamics_models
  yaml-cpp
)
# prevent pluginlib from using boost
target_compile_definitions(moying_mcr_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface moying_mcr_hardware.xml)

install(
  TARGETS
  moying_mcr_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  moying_mcr_hardware
)
ament_export_dependencies(
  ament_index_cpp
  elfin_ethercat_driver_plant
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  moying_mcr_arm_dynamics_models
  yaml-cpp
)

ament_package()
