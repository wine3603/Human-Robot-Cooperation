
cmake_minimum_required(VERSION 3.8)
project(elfin_ethercat_driver_plant)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(elfin_robot_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(soem3 REQUIRED)
find_package(boost_thread REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
add_library(elfin_ethercat_driver_plant 
  "src/elfin_ethercat_driver.cpp"
  "src/elfin_ethercat_client.cpp"
  "src/elfin_ethercat_io_client.cpp"
  "src/elfin_ethercat_manager.cpp"
  )

target_link_libraries(elfin_ethercat_driver_plant PRIVATE boost_thread yaml-cpp)

target_compile_options(elfin_ethercat_driver_plant PRIVATE -fPIC)
target_include_directories(elfin_ethercat_driver_plant
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(elfin_ethercat_driver_plant
PUBLIC
  rclcpp
  rclpy
  std_msgs
  soem3
  elfin_robot_msgs
  trajectory_msgs
  sensor_msgs
  std_srvs
  tf2
  hardware_interface
  rosidl_default_generators
  ament_cmake
  boost_thread
)
ament_export_targets(elfin_ethercat_driver_plantTargets HAS_LIBRARY_TARGET) 
ament_export_dependencies(
  Eigen3
  rclcpp
  rclpy
  std_msgs
  elfin_robot_msgs
  soem3
  trajectory_msgs
  sensor_msgs
  std_srvs
  tf2
  hardware_interface
  rosidl_default_generators
  ament_cmake
  boost_thread
)

install(
  
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS elfin_ethercat_driver_plant 
  EXPORT elfin_ethercat_driver_plantTargets
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
