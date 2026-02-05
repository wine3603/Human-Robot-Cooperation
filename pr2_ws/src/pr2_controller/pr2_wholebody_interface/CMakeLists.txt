cmake_minimum_required(VERSION 3.8)
project(pr2_wholebody_interface)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(dependencies
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_robotic_assets
  ocs2_pinocchio_interface
  ocs2_self_collision
  ocs2_ros_interfaces
  ocs2_mobile_manipulator
  geometry_msgs
  Boost
  angles
  tf2_ros
  realtime_tools
)
find_package(ament_cmake REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ocs2_ddp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ocs2_mobile_manipulator REQUIRED)

find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_pinocchio_interface REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_robotic_assets REQUIRED)
find_package(ocs2_self_collision REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

find_package(PkgConfig REQUIRED)
find_package(angles REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
pkg_check_modules(hpp-fcl REQUIRED hpp-fcl)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/wholebody_pinocchio_mapping.cpp
  src/wholebody_wb_pinocchio_mapping.cpp
  src/wholebody_precomputation.cpp
  src/factory_functions.cpp
  src/dynamics/wholebody_dynamics.cpp
  src/constraint/end_effort_constraint.cpp
  src/constraint/base_constraint.cpp
  src/command/target_trajectories_interactive_marker.cpp
  src/command/joy_target_trajectories.cpp
  src/wholebody_interface.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies} 
)
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
endif()

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
