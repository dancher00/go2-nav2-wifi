"""Localization + Nav2 + loop patrol through waypoints YAML."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory("go2_nav2")
    default_map = "/ws/maps/my_room.yaml"
    default_wp = os.path.join(pkg, "config", "patrol_example.yaml")

    map_arg = DeclareLaunchArgument("map", default_value=default_map)
    wp_arg = DeclareLaunchArgument("waypoints", default_value=default_wp)

    nav2_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "nav2_slam_loc.launch.py")),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "goal_nav": "false",
        }.items(),
    )

    patrol_node = Node(
        package="go2_nav2",
        executable="go2_patrol",
        name="go2_patrol",
        output="screen",
        parameters=[
            {"waypoints_file": LaunchConfiguration("waypoints")},
        ],
    )

    return LaunchDescription([map_arg, wp_arg, nav2_loc, patrol_node])
