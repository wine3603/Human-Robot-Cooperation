import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction,DeclareLaunchArgument,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

def generate_launch_description():
    # Initialize Arguments
    mode = LaunchConfiguration("mode")
    log_level = LaunchConfiguration("log_level")

    # Declare arguments
    declared_arguments = [DeclareLaunchArgument('mode', default_value='mujoco'),
                          DeclareLaunchArgument(name='log_level', default_value='info')]
    # Get URDF via xacro
    simulation_controller_settings = PathJoinSubstitution(
      [
        FindPackageShare("moying_mcr_bringup"),
        "config",
        "simulation_controller.yaml"
      ]
    )
    elfin_drivers_yaml = os.path.join(get_package_share_directory("moying_mcr_bringup"),
        "config","elfin_drivers.yaml")
    robot_description_content = Command(
      [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
          [
            FindPackageShare("moying_mcr_description"),
            "urdf",
            "moying_mcr.urdf.xacro",
          ]
        ),
        ' mode:=',mode,
        ' settings:=', simulation_controller_settings,
      ]
    )

    robot_description = {"robot_description": robot_description_content}

    real_controller_settings = PathJoinSubstitution(
      [
        FindPackageShare("moying_mcr_bringup"),
        "config",
        "moying_mcr_controllers.yaml"
      ]
    )

    spawn_entity = Node(
          package="controller_manager",
          executable="ros2_control_node",
          namespace="/mcr",
          parameters=[robot_description,real_controller_settings],
          output={
              "stdout":"screen",
              "stderr":"screen",
          },
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="/mcr",
        parameters=[robot_description, {"frame_prefix": "/mcr/"}],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/mcr",
        arguments=["joint_state_broadcaster", "--controller-manager", "/mcr/controller_manager"],
    )

    load_chassis_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_controller", "--controller-manager", "/mcr/controller_manager"],
        namespace="/mcr",
        output="log",
    )
    load_arm_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "--controller-manager", "/controller_manager",
                   '--ros-args', '--log-level', log_level],
        output="log",
    )
    load_left_arm_velcoity_interface_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_velocity_interface_controller", "--controller-manager", "/mcr/controller_manager",
                   '--ros-args', '--log-level', log_level],
        namespace="/mcr",
        output="log",
    )
    joy_entity = Node(
          package="joy",
          executable="joy_node",
          # namespace="/mcr/robotiq_controller",
          namespace="/mcr",
          output={
              "stdout":"screen",
              "stderr":"screen",
          },
    )

    # joy_entity = Node(
    #       package="joy",
    #       executable="joy_node",
    #       # name='mcr_gripper_joy_node',  # 1. 给它起个独一无二的名字，防止被系统里的其他节点干扰
    #       namespace="/mcr/robotiq_controller",
    #       parameters=[{
    #           "device_id": 0,           # 2. 显式指定读取 js0 (如果 js0 不行就试 js1)
    #           "deadzone": 0.05,         # 设置死区，防止漂移
    #           "autorepeat_rate": 20.0,  # 设置刷新率
    #           "sticky_buttons": False,
    #           "coalesce_interval": 0.001
    #       }],
    #       output="screen",              # 3. 开启屏幕输出，如果有错能立刻看到
    # )

    load_right_arm_velcoity_interface_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_velocity_interface_controller", "--controller-manager", "/mcr/controller_manager",
                   '--ros-args', '--log-level', log_level],
        namespace="/mcr",
        output="log",
    )

    nodes = [
        spawn_entity,
        robot_state_publisher_node,
        load_joint_state_controller,
        joy_entity,
        # load_velcoity_interface_controller,
        load_chassis_controller,
        # load_arm_trajectory_controller,

    ]

    return LaunchDescription(declared_arguments + nodes)