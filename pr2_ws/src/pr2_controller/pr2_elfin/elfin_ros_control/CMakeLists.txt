cmake_minimum_required(VERSION 3.5)
project(elfin_ros_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclpy
  std_msgs
  std_srvs
  controller_manager
  controller_manager_msgs
  elfin_robot_msgs
  trajectory_msgs
  sensor_msgs
  tf2
  hardware_interface
  controller_interface
  elfin_ethercat_driver
  pluginlib
  rosidl_default_generators
  soem
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(yaml-cpp REQUIRED)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(
  ${PROJECT_NAME}
  SHARED
  src/elfin_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})


pluginlib_export_plugin_description_file(hardware_interface elfin_hardware_interface.xml)


install(
  
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS 
  ${PROJECT_NAME}
  EXPORT export_elfin_ros_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_elfin_ros_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} elfin_ethercat_driver)
ament_package()
