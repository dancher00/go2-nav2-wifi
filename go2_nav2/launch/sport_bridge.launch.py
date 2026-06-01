from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("go2_nav2")
    config = os.path.join(pkg, "config", "sport_bridge.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=config,
                description="sport_bridge parameters",
            ),
            LogInfo(
                msg=(
                    "Wi-Fi: sport_bridge on laptop does NOT move the robot. "
                    "Use robot relay + /ws/scripts/teleop-slam.sh"
                ),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            EnvironmentVariable("GO2_NET", default_value="wifi"),
                            "' == 'wifi'",
                        ]
                    )
                ),
            ),
            Node(
                package="go2_nav2",
                executable="sport_bridge",
                name="sport_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
