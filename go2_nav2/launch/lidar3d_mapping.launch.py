"""Raw L1 + gyro -> Point-LIO; local mapping without loop closure.

Factory odometry supplies clock reference stamps only, never pose or TF.
The default backend model does not integrate the raw IMU acceleration.
"""
import json
import os
from pathlib import Path
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    config = LaunchConfiguration('config').perform(context)
    result = Path(LaunchConfiguration('result_dir').perform(context))
    bag = LaunchConfiguration('bag').perform(context)
    recording = not bag and os.environ.get('GO2_RECORD', '0') == '1'
    parameters = yaml.safe_load(Path(config).read_text())['/**']['ros__parameters']
    settings = parameters['mapping']
    replay_rate = os.environ.get('GO2_REPLAY_RATE', '0.5')
    shutil.copyfile(config, result / 'pointlio.yaml')
    (result / 'input.json').write_text(json.dumps({
        'replay_bag': bag or None, 'recording': recording,
        'compute': os.environ.get('GO2_LIDAR3D_COMPUTE', 'laptop'),
        'bag_time': 'translator callback wall time; headers retain shared clock translation',
        'raw_measurements': 'point coordinates, relative point times, IMU vectors unchanged',
        'imu_enabled': settings.get('imu_en', True),
        'imu_as_input': parameters.get('use_imu_as_input', True),
        'replay_rate': float(replay_rate) if bag else None,
        'imu_acceleration_used': settings.get('use_acceleration', True),
    }, indent=2) + '\n')
    clock = None if bag else Node(
        package='go2_nav2', executable='go2_cloud_stamp_sync',
        name='go2_cloud_stamp_sync', output='screen', parameters=[{
            'cloud_in': '/utlidar/cloud', 'cloud_out': '/lidar3d/cloud_sync',
            'odom_in': '/utlidar/robot_odom',
            'odom_out': '/lidar3d/factory_odom_clock_reference',
            'imu_in': '/utlidar/imu', 'imu_out': '/lidar3d/imu_sync',
            'trace_path': str(result / 'sensor-clock.jsonl'),
            'bag_path': str(result / 'sensors') if recording else '',
        }])
    lio = Node(
        package='point_lio_unilidar', executable='pointlio_mapping',
        name='lidar3d_lio', output='screen', parameters=[str(result / 'pointlio.yaml'), {'use_sim_time': bool(bag)}],
        additional_env={'GO2_LIDAR3D_RESULT_DIR': str(result)},
        remappings=[('/cloud_registered', '/lidar3d/registered'),
                    ('/Laser_map', '/lidar3d/initial_map'),
                    ('/aft_mapped_to_init', '/lidar3d/odom'),
                    ('/path', '/lidar3d/path')])
    nodes = [clock, lio] if clock is not None else [lio]
    exits = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='3D mapping process exited'))]))
        for node in nodes]
    if bag:
        playback = ExecuteProcess(cmd=[
            'ros2', 'bag', 'play', bag, '--clock', '100', '--rate', replay_rate,
            '--delay', '2', '--disable-keyboard-controls', '--topics',
            '/lidar3d/cloud_sync', '/lidar3d/imu_sync'], output='screen')

        def finish_replay(event, _context):
            if event.returncode != 0:
                raise RuntimeError(f'rosbag playback failed: {event.returncode}')
            # Let the backend consume its last buffered measurements before SIGINT/save.
            return [TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown(reason='Replay complete'))])]

        exits.append(RegisterEventHandler(OnProcessExit(target_action=playback, on_exit=finish_replay)))
        nodes.append(playback)
    return [*exits, *nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('result_dir', description='New session output directory'),
        DeclareLaunchArgument('bag', default_value='', description='Recorded inputs; empty for live Wi-Fi'),
        DeclareLaunchArgument('config', default_value=get_package_share_directory('go2_nav2') + '/config/pointlio_go2.yaml'),
        OpaqueFunction(function=configure)])
