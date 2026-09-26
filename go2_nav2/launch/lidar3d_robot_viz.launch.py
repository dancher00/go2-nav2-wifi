"""Laptop-only robot model: existing TF and joint bridges, no motion nodes."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('go2_nav2')
    return LaunchDescription([
        Node(package='go2_nav2', executable='go2_urdf_self_filter', output='screen', respawn=True, respawn_delay=1.0,
             parameters=[{'sensor_frame': 'utlidar_lidar' if os.environ.get('GO2_LIDAR3D_BACKEND') == 'legkilo' else 'lidar3d_sensor'}]),
        Node(package='go2_nav2', executable='go2_odom_tf', name='go2_odom_tf',
             parameters=[pkg + '/config/lidar3d_robot_viz.yaml',
                {'sensor_from_base': [0.,0.,0.,0.,0.,0.,1.]} if os.environ.get('GO2_LIDAR3D_BACKEND') == 'legkilo' else {'sensor_frame': 'lidar3d_sensor'}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(pkg + '/launch/robot_description.launch.py'),
                                 launch_arguments={'joint_stamp_odom_topic': '/lidar3d/odom'}.items()),
    ])
