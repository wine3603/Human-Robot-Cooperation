from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    pkg_share = FindPackageShare("pr2_description")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "pr2.urdf.xacro"])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro", " ", xacro_file]),
            value_type=str
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )
    
    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "pr2.rviz"])],
        output="screen",
    )

    return LaunchDescription([
        rsp, 
        jsp_gui,
        rviz
        ])
