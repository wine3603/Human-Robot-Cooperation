cmake_minimum_required(VERSION 3.8)
project(pr2_mobile_controller)

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
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Parameters library (rename it to match package scope)
generate_parameter_library(pr2_mobile_controller_parameters
  src/pr2_omni_controller.yaml
  include/pr2_mobile_controller/validate_pr2_mobile_controller_parameters.hpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Controller library
add_library(pr2_omni_controller SHARED
  src/pr2_omni_controller.cpp
)

target_include_directories(pr2_omni_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(pr2_omni_controller
  pr2_mobile_controller_parameters
)

ament_target_dependencies(pr2_omni_controller
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# For visibility_control.h on Windows/macros
target_compile_definitions(pr2_omni_controller PRIVATE "PR2_MOBILE_CONTROLLER_BUILDING_DLL")

# pluginlib XML (file name must exist in the package root)
pluginlib_export_plugin_description_file(
  controller_interface pr2_omni_controller.xml
)

install(
  TARGETS
    pr2_omni_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES pr2_omni_controller.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_libraries(
  pr2_omni_controller
)

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
