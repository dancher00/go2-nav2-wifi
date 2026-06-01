"""go2_description: robot_state_publisher + joint states from robot /lf/lowstate."""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_path = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "xacro", "robot.xacro"]
    )
    xacro_exe = os.path.join(get_package_prefix("xacro"), "bin", "xacro")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "lowstate_topic",
                default_value="/lf/lowstate",
                description="Unitree LowState for live leg joints",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command([xacro_exe, " ", xacro_path]),
                            value_type=str,
                        ),
                    }
                ],
            ),
            Node(
                package="go2_nav2",
                executable="go2_joint_state_bridge",
                name="go2_joint_state_bridge",
                output="screen",
                parameters=[{"lowstate_topic": LaunchConfiguration("lowstate_topic")}],
            ),
        ]
    )
