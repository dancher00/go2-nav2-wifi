"""RViz bringup view: robot mesh + utlidar rainbow cloud + Go2 front camera (if relay running)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_nav2")
    default_rviz = os.path.join(pkg, "rviz", "bringup.rviz")
    sensors_launch = os.path.join(pkg, "launch", "sensors.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("start_sensors", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sensors_launch),
                condition=IfCondition(LaunchConfiguration("start_sensors")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
