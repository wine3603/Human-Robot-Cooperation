cmake_minimum_required(VERSION 3.8)
project(baichuan_fieldbus_ethercat)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )

if(WIN32)
  set(OS "win32")
else()
  set(OS "linux")
endif()

# explicitely find pthread to be able to explicitely export the pthread dependency via catkin_package
# see https://github.com/ros/ros_comm/issues/135
find_package(soem REQUIRED)
find_package(Threads)


add_library(${PROJECT_NAME} src/ethercat_master.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  rclcpp
  soem
)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET) 
ament_export_dependencies(soem)

add_executable(${PROJECT_NAME}_test_node
    test/baichuan_fieldbus_ethercat_master_test_node.cpp
)

target_link_libraries(${PROJECT_NAME}_test_node ${PROJECT_NAME} )
ament_target_dependencies(${PROJECT_NAME}_test_node
    rclcpp
)
install(
    TARGETS ${PROJECT_NAME}_test_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS 
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_include_directories(include)


ament_package()
