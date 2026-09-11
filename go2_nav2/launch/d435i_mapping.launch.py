"""Onboard RGB-D mapping, with a configurable provisional Go2 camera mount."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    visual = PythonExpression(["'", LaunchConfiguration('odom_source'), "' == 'rgbd'"])
    inputs = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        ('imu', '/d435i/imu'),
        ('odom', '/d435i/odom'),
        ('odom_info', '/d435i/odom_info'),
    ]
    common = {
        'frame_id': 'camera_link',
        'odom_frame_id': 'd435i_odom',
        'wait_imu_to_init': True,
        'approx_sync': False,
        # Aligned RGB/depth/info have identical hardware timestamps.
        'topic_queue_size': 10,
        'sync_queue_size': 10,
        'qos': 1,
        'qos_image': 1,
        'qos_camera_info': 1,
        'qos_odom': 1,
        'qos_imu': 2,
    }
    return LaunchDescription([
        DeclareLaunchArgument('odom_source', default_value='go2', choices=['go2', 'rgbd']),
        # Initial mount translation from the reference Go2 demo, not a measured calibration.
        DeclareLaunchArgument('camera_x', default_value='0.2'),
        DeclareLaunchArgument('camera_y', default_value='0.0'),
        DeclareLaunchArgument('camera_z', default_value='0.15'),
        DeclareLaunchArgument('database_path', default_value='/data/rtabmap.db'),
        DeclareLaunchArgument('unite_imu_method', default_value='2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
            launch_arguments={
                'camera_namespace': '', 'camera_name': 'camera',
                'enable_color': 'true', 'enable_depth': 'true',
                'enable_infra1': 'false', 'enable_infra2': 'false',
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
                'config_file': str(Path(__file__).resolve().parents[1] / 'config' / 'd435i_camera.yaml'),
                'enable_gyro': 'true', 'enable_accel': 'true',
                'gyro_fps': '200', 'accel_fps': '100',
                'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                'align_depth.enable': 'true', 'enable_sync': 'true',
                'pointcloud.enable': 'false',
            }.items()),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='d435i_imu_filter', output='screen',
             parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
             remappings=[('imu/data_raw', '/camera/imu'), ('imu/data', '/d435i/imu')]),
        Node(package='rtabmap_odom', executable='rgbd_odometry',
             name='rgbd_odometry', namespace='d435i', output='screen',
             condition=IfCondition(visual),
             parameters=[common, {'publish_tf': True,
                                  'Odom/Strategy': '0', 'Vis/MaxFeatures': '600',
                                  'OdomF2M/MaxSize': '2000'}], remappings=inputs),
        ExecuteProcess(cmd=['python3', '/work/ws/scripts/d435i_go2_odom.py'],
                       condition=UnlessCondition(visual), output='screen'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='d435i_mount', condition=UnlessCondition(visual),
             arguments=['--x', LaunchConfiguration('camera_x'),
                        '--y', LaunchConfiguration('camera_y'), '--z', LaunchConfiguration('camera_z'),
                        '--roll', '0', '--pitch', '0', '--yaw', '0',
                        '--frame-id', 'd435i_base', '--child-frame-id', 'camera_link']),
        Node(package='rtabmap_slam', executable='rtabmap',
             name='rtabmap', namespace='d435i', output='screen',
             parameters=[common, {
                 'database_path': LaunchConfiguration('database_path'),
                 'map_frame_id': 'd435i_map', 'publish_tf': True,
                 'odom_frame_id': 'd435i_odom',
                 'wait_for_transform': 0.3,
                 'subscribe_depth': True, 'subscribe_odom_info': False,
                 'Rtabmap/DetectionRate': '2.0',
                 'RGBD/LinearUpdate': '0.05', 'RGBD/AngularUpdate': '0.05',
                 'Grid/FromDepth': 'true', 'Grid/3D': 'true',
                 'Grid/RangeMin': '0.4', 'Grid/RangeMax': '4.0', 'Grid/CellSize': '0.04',
                 'Grid/DepthDecimation': '2',
                 'RGBD/CreateOccupancyGrid': 'true',
             }], remappings=inputs),
    ])
