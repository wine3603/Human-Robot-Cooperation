cmake_minimum_required(VERSION 3.8)
project(robotiq_ft_sensor_hardware_zombie)

# find dependencies
find_package(ament_cmake REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
    geometry_msgs
    rclcpp hardware_interface pluginlib
    realtime_tools
    robotiq_ft_sensor_interfaces
    std_srvs
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

# === Private library
add_library(robotiq_ft_sensor src/rq_sensor_com.cpp src/rq_sensor_state.cpp)
ament_target_dependencies(robotiq_ft_sensor ${dependencies})

add_library(
  robotiq_ft_sensor_hardware_zombie
  SHARED
  src/robotiq_ft_sensor_hardware_zombie.cpp
)
target_include_directories(
  robotiq_ft_sensor_hardware_zombie
  PUBLIC
  include
)
ament_target_dependencies(
  robotiq_ft_sensor_hardware_zombie
   ${THIS_PACKAGE_INCLUDE_DEPENDS}

)
target_link_libraries(robotiq_ft_sensor_hardware_zombie robotiq_ft_sensor)
# prevent pluginlib from using boost
target_compile_definitions(robotiq_ft_sensor_hardware_zombie PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface robotiq_ft_sensor_hardware_zombie.xml)

install(
  TARGETS
  robotiq_ft_sensor_hardware_zombie
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
  robotiq_ft_sensor_hardware_zombie
)
ament_export_dependencies(
${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
