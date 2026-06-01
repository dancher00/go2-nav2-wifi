"""slam_toolbox localization on saved map + planner + controller."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _localization_launch(context, *args, **kwargs):
    go2_pkg = get_package_share_directory("go2_nav2")
    map_yaml = LaunchConfiguration("map").perform(context)
    map_stem, _ext = os.path.splitext(map_yaml)
    if not os.path.isfile(map_yaml):
        raise RuntimeError(f"Map not found: {map_yaml}")

    params_path = os.path.join(go2_pkg, "config", "slam_toolbox_localization.yaml")
    with open(params_path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    data["slam_toolbox"]["ros__parameters"]["map_file_name"] = map_stem

    if not os.path.isfile(f"{map_stem}.posegraph"):
        print(
            f"WARN: {map_stem}.posegraph missing — run save-map.sh while SLAM is active "
            "(serialize_map). Localization may fail."
        )

    fd, temp_path = tempfile.mkstemp(suffix=".yaml", prefix="go2_loc_")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False)

    slam_share = get_package_share_directory("slam_toolbox")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, "launch", "localization_launch.py")
            ),
            launch_arguments={"slam_params_file": temp_path}.items(),
        )
    ]


def _nav2_params_file(context, *args, **kwargs) -> str:
    """Bake map path into map_server.ros__parameters (flat override does not work)."""
    go2_pkg = get_package_share_directory("go2_nav2")
    map_yaml = LaunchConfiguration("map").perform(context)
    nav2_params = os.path.join(go2_pkg, "config", "go2_nav2_minimal.yaml")
    with open(nav2_params, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    data["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml
    fd, temp_path = tempfile.mkstemp(suffix=".yaml", prefix="go2_nav2_")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False)
    return temp_path


def _nav2_servers(context, *args, **kwargs):
    params = _nav2_params_file(context)
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params],
        ),
    ]


def _nav2_lifecycle(context, *args, **kwargs):
    return [
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "bond_timeout": 10.0,
                    "attempt_respawn_reconnection": True,
                    "node_names": [
                        "map_server",
                        "planner_server",
                        "controller_server",
                    ],
                },
            ],
        ),
    ]


def generate_launch_description():
    go2_pkg = get_package_share_directory("go2_nav2")
    default_map = "/ws/maps/my_room.yaml"

    goal_nav = LaunchConfiguration("goal_nav")

    nodes = [
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument(
            "goal_nav",
            default_value="true",
            description="Launch go2_goal_pose_nav (RViz 2D Goal). Set false for patrol.launch.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_pkg, "launch", "sensors.launch.py")
            ),
        ),
        OpaqueFunction(function=_localization_launch),
        Node(
            package="go2_nav2",
            executable="go2_map_odom_relay",
            name="go2_map_odom_relay",
            output="screen",
        ),
        # Nav2: servers first, lifecycle manager after they are up (avoid activate-before-configure).
        TimerAction(period=5.0, actions=[OpaqueFunction(function=_nav2_servers)]),
        TimerAction(period=8.0, actions=[OpaqueFunction(function=_nav2_lifecycle)]),
    ]

    nodes.append(
        Node(
            package="go2_nav2",
            executable="go2_goal_pose_nav",
            name="go2_goal_pose_nav",
            output="screen",
            condition=IfCondition(goal_nav),
        )
    )

    return LaunchDescription(nodes)
