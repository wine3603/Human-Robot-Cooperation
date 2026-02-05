cmake_minimum_required(VERSION 3.16)
project(moying_mcr_mujoco_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
  soem
  nlohmann_json
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_sdk2 REQUIRED)
find_package(yaml-cpp REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  moying_mcr_mujoco_hardware
  SHARED
  src/moying_mcr_mujoco_hardware.cpp
)
target_compile_features(moying_mcr_mujoco_hardware PUBLIC cxx_std_17)
target_include_directories(moying_mcr_mujoco_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/moying_mcr_mujoco_hardware>
)
ament_target_dependencies(
  moying_mcr_mujoco_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
# target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp )
# target_link_libraries(${PROJECT_NAME} unitree_sdk2)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    yaml-cpp
    unitree_sdk2
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "MOYING_MCR_MUJOCO_HARDWARE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface moying_mcr_mujoco_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/moying_mcr_mujoco_hardware
)

install(TARGETS moying_mcr_mujoco_hardware
  EXPORT export_moying_mcr_mujoco_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## EXPORTS
ament_export_targets(export_moying_mcr_mujoco_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
