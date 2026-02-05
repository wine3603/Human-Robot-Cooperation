cmake_minimum_required(VERSION 3.8)
project(moying_mcr_mujoco_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

# 构建可执行程序
add_executable(moying_mcr_mujoco_tf_node
  src/moying_mcr_mujoco_tf.cpp
)

# add_executable(pr2_mujoco_tf_node
#   src/pr2_mujoco_tf.cpp
# )

# 链接依赖库
target_link_libraries(moying_mcr_mujoco_tf_node
  unitree_sdk2
)
# target_link_libraries(pr2_mujoco_tf_node
#   unitree_sdk2
# )
# 设置依赖项
ament_target_dependencies(moying_mcr_mujoco_tf_node
  rclcpp
  tf2_ros
  geometry_msgs
)
# ament_target_dependencies(pr2_mujoco_tf_node
#   rclcpp
#   tf2_ros
#   geometry_msgs
# )
# 安装节点
install(TARGETS
  moying_mcr_mujoco_tf_node
  DESTINATION lib/${PROJECT_NAME}
)
# install(TARGETS
#   pr2_mujoco_tf_node
#   DESTINATION lib/${PROJECT_NAME}
# )
# 测试依赖（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
