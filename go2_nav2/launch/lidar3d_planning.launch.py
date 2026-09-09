"""Nav2 online plan preview from current Point-LIO scans; no motion nodes."""
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    pkg = get_package_share_directory('go2_nav2')
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    with open(pkg + '/config/go2_nav2_minimal.yaml') as stream:
        planner = yaml.safe_load(stream)['planner_server']
    planner['ros__parameters']['use_sim_time'] = sim
    planner['ros__parameters']['GridBased']['allow_unknown'] = False
    with open(pkg + '/config/lidar3d_planner.yaml') as stream:
        data = yaml.safe_load(stream)
    data['global_costmap']['global_costmap']['ros__parameters']['use_sim_time'] = sim
    data['planner_server'] = planner
    fd, path = tempfile.mkstemp(prefix='go2-lidar3d-planner-', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump(data, stream)
    def cleanup(_context):
        os.unlink(path)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[path]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_lidar3d_planning', output='screen', parameters=[{
                 'use_sim_time': sim, 'autostart': True, 'node_names': ['planner_server'],
                 'bond_timeout': 10.0}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('use_sim_time', default_value='false'),
                              OpaqueFunction(function=configure)])
