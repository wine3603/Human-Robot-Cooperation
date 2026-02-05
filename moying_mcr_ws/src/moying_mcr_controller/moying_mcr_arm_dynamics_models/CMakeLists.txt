cmake_minimum_required(VERSION 3.8)
project(moying_mcr_arm_dynamics_models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

file(GLOB ELFIN_SRC src/elfin/*/*.cpp)
add_library(${PROJECT_NAME} SHARED
    ${ELFIN_SRC}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    # $<INSTALL_INTERFACE:include/moying_mcr_arm_dynamics_models>
    $<INSTALL_INTERFACE:include>

    ${Eigen3_INCLUDE_DIR}
)
ament_target_dependencies(${PROJECT_NAME}
    Eigen3
)

install(
    DIRECTORY include/
    DESTINATION include

    # DESTINATION include/moying_mcr_arm_dynamics_models
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_dyn_models
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

# add_executable(test_elfin test/test_dynamics.cpp)
# target_link_libraries(test_elfin
#     ${PROJECT_NAME}
# )
# install(TARGETS test_elfin
#     DESTINATION lib/${PROJECT_NAME}
# )

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

ament_export_include_directories(
    include
)

ament_export_libraries(
    ${PROJECT_NAME}
)
ament_export_targets(export_dyn_models HAS_LIBRARY_TARGET)

ament_export_dependencies(
    eigen3_cmake_module
    Eigen3
)

ament_package()
