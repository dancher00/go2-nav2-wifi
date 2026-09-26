"""Leg-KILO: L1 + body IMU + body-frame foot kinematics; no factory pose fusion."""
import json
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def configure(context):
    result = Path(LaunchConfiguration('result_dir').perform(context))
    config = LaunchConfiguration('config').perform(context)
    bag = LaunchConfiguration('bag').perform(context)
    settings = yaml.safe_load(Path(config).read_text())
    settings['temp_result_save_folder'] = str(result / 'native')
    (result / 'legkilo.yaml').write_text(yaml.safe_dump(settings, sort_keys=False))
    recording = not bag and os.environ.get('GO2_RECORD', '0') == '1'
    rate = os.environ.get('GO2_REPLAY_RATE', '0.5')
    (result / 'input.json').write_text(json.dumps({
        'backend': 'legkilo', 'compute': os.environ.get('GO2_LIDAR3D_COMPUTE', 'laptop'),
        'replay_bag': bag or None, 'recording': recording, 'replay_rate': float(rate) if bag else None,
        'imu_enabled': True, 'imu_acceleration_used': True, 'kinematics_used': True,
        'factory_body_pose_used': False, 'point_time_bucket_s': 0.002, 'loop_closure': bool(settings['loop_closure_enable']),
        'kinematics_source': 'LowState q/dq/foot_force paired with SportModeState stamp by identical six-axis IMU sample',
        'contact_thresholds_provisional': True,
        'live_visualization': 'Optimized backend map and trajectory at ~1 Hz; body pose uses latest backend correction',
        'registered_cloud_semantics': 'complete backend map snapshot; replace previous cloud',
    }, indent=2) + '\n')
    clock = Node(package='go2_nav2', executable='go2_cloud_stamp_sync', name='go2_cloud_stamp_sync',
        parameters=[{'cloud_in': '/utlidar/cloud', 'cloud_out': '/lidar3d/cloud_sync',
            'odom_in': '/utlidar/robot_odom', 'odom_out': '/lidar3d/factory_odom_clock_reference',
            'lowstate_record_in': '/lowstate' if recording else '', 'kinematic_in': '/sportmodestate', 'kinematic_out': '/lidar3d/kinematic_sync',
            'imu_in': '/utlidar/imu' if recording else '', 'imu_out': '/lidar3d/imu_sync' if recording else '',
            'trace_path': str(result / 'sensor-clock.jsonl'),
            'bag_path': str(result / 'sensors') if recording else ''}], output='screen')
    backend = Node(package='legkilo', executable='legkilo_node', name='legkilo',
        arguments=['--config_file=' + str(result / 'legkilo.yaml')],
        additional_env={'GO2_LIDAR3D_RESULT_DIR': str(result)}, cwd=str(result / 'Log'), output='screen',
        remappings=[('/cloud_registered','/lidar3d/registered'),('/Odometry','/lidar3d/odom'),('/path','/lidar3d/path')])
    nodes = [backend] if bag else [clock, backend]
    handlers = [RegisterEventHandler(OnProcessExit(target_action=n,
        on_exit=[EmitEvent(event=Shutdown(reason='Leg-KILO session process exited'))])) for n in nodes]
    if bag:
        player = ExecuteProcess(cmd=['ros2','bag','play',bag,'--rate',rate,'--delay','2','--disable-keyboard-controls',
            '--topics','/lidar3d/cloud_sync','/lidar3d/kinematic_sync','/lowstate'], output='screen')
        def done(event, context):
            if event.returncode: raise RuntimeError('Leg-KILO bag playback failed')
            return [TimerAction(period=2., actions=[EmitEvent(event=Shutdown(reason='Replay complete'))])]
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=player,on_exit=done)))
        nodes.append(player)
    return [*handlers,*nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('result_dir'), DeclareLaunchArgument('bag',default_value=''),
        DeclareLaunchArgument('config',default_value=get_package_share_directory('go2_nav2')+'/config/legkilo_go2.yaml'),
        OpaqueFunction(function=configure)])
