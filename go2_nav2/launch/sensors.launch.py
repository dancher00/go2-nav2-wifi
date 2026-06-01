"""Lidar pipeline: odom TF, go2_description TF tree, pointcloud_to_laserscan."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("go2_nav2")
    pcl_config = os.path.join(pkg, "config", "pointcloud_to_laserscan.yaml")
    robot_desc_launch = os.path.join(pkg, "launch", "robot_description.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_deskewed"),
            DeclareLaunchArgument(
                "cloud_topic_sync",
                default_value="/utlidar/cloud_deskewed_sync",
                description="Stamp-synced cloud for pointcloud_to_laserscan",
            ),
            DeclareLaunchArgument("odom_topic", default_value="/utlidar/robot_odom"),
            DeclareLaunchArgument(
                "odom_tf_use_current_stamp",
                default_value="true",
                description="false for Nav2 — TF stamps match /scan (avoid costmap drops)",
            ),
            DeclareLaunchArgument(
                "odom_source",
                default_value="utlidar",
                description="utlidar (default) or sport (/sportmodestate via bridge)",
            ),
            DeclareLaunchArgument(
                "use_go2_description",
                default_value="true",
                description="Use go2_description URDF (meshes). If false, static box + go2_vis.",
            ),
            DeclareLaunchArgument("lidar_x", default_value="0.171"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.09"),
            DeclareLaunchArgument("lidar_roll", default_value="0.0"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            DeclareLaunchArgument("body_z", default_value="0.30"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_desc_launch),
                condition=IfCondition(LaunchConfiguration("use_go2_description")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="footprint_to_base",
                condition=UnlessCondition(LaunchConfiguration("use_go2_description")),
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    LaunchConfiguration("body_z"),
                    "--frame-id",
                    "base_footprint",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_lidar",
                condition=UnlessCondition(LaunchConfiguration("use_go2_description")),
                arguments=[
                    "--x",
                    LaunchConfiguration("lidar_x"),
                    "--y",
                    LaunchConfiguration("lidar_y"),
                    "--z",
                    LaunchConfiguration("lidar_z"),
                    "--roll",
                    LaunchConfiguration("lidar_roll"),
                    "--pitch",
                    LaunchConfiguration("lidar_pitch"),
                    "--yaw",
                    LaunchConfiguration("lidar_yaw"),
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "utlidar_lidar",
                ],
            ),
            Node(
                package="go2_nav2",
                executable="go2_sport_state_odom",
                name="go2_sport_state_odom",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("odom_source"), "' == 'sport'"]
                    )
                ),
                parameters=[
                    {
                        "state_topic": "/sportmodestate",
                        "state_type": "unitree_go/msg/SportModeState",
                        "odom_out_topic": "/sport_state/odom",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "use_current_stamp": True,
                    }
                ],
            ),
            Node(
                package="go2_nav2",
                executable="go2_odom_tf",
                name="go2_odom_tf",
                output="screen",
                parameters=[
                    os.path.join(pkg, "config", "odom_tf.yaml"),
                    {
                        "odom_topic": PythonExpression(
                            [
                                "'/sport_state/odom' if '",
                                LaunchConfiguration("odom_source"),
                                "' == 'sport' else '",
                                LaunchConfiguration("odom_topic"),
                                "'",
                            ]
                        )
                    },
                    {
                        "base_frame": "base_link",
                        "use_current_stamp": LaunchConfiguration(
                            "odom_tf_use_current_stamp"
                        ),
                    },
                ],
            ),
            Node(
                package="go2_nav2",
                executable="go2_cloud_stamp_sync",
                name="go2_cloud_stamp_sync",
                output="screen",
                parameters=[
                    {
                        "cloud_in": LaunchConfiguration("cloud_topic"),
                        "cloud_out": LaunchConfiguration("cloud_topic_sync"),
                    }
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[("cloud_in", LaunchConfiguration("cloud_topic_sync"))],
                parameters=[pcl_config],
            ),
        ]
    )
