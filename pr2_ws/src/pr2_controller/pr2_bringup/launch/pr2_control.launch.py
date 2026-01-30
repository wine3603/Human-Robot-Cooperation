import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction,DeclareLaunchArgument,ExecuteProcess,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    
    controllers_yaml = PathJoinSubstitution(
      [
        FindPackageShare("pr2_bringup"),
        "config",
        "pr2_controllers.yaml"
      ]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("pr2_description"),
                "urdf",
                "pr2.urdf.xacro",
            ]),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="/pr2",
        parameters=[robot_description,controllers_yaml],
        output={
            "stdout":"screen",
            "stderr":"screen",
        },
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace="/pr2",
        parameters=[robot_description, {"frame_prefix": "/pr2/"}],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/pr2",
        arguments=["joint_state_broadcaster", "--controller-manager", "/pr2/controller_manager"],
    )

    left_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/pr2",
        arguments=["left_gripper_controller", "--controller-manager", "/pr2/controller_manager"],
    )

    right_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/pr2",
        arguments=["right_gripper_controller", "--controller-manager", "/pr2/controller_manager"],
    )

    pr2_omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="/pr2",
        arguments=["pr2_omni_controller", "--controller-manager", "/pr2/controller_manager"],
    )
    return LaunchDescription([
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        left_gripper_spawner,
        right_gripper_spawner,
        pr2_omni_controller_spawner,
    ])
