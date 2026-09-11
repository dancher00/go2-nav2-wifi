"""Supervised level-floor controller, added to an existing 3D planning session.

Robot transport is started separately after validation.
"""
import copy
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def configure(context):
    pkg = get_package_share_directory('go2_nav2')
    with open(pkg + '/config/go2_nav2_minimal.yaml') as stream:
        controller = yaml.safe_load(stream)['controller_server']
    params = controller['ros__parameters']
    params.update(odom_topic='/lidar3d/body_odom', controller_frequency=10.0,
                  transform_tolerance=0.3, failure_tolerance=0.0,
                  min_x_velocity_threshold=0.01, min_y_velocity_threshold=0.01,
                  min_theta_velocity_threshold=0.02)
    params['progress_checker']['movement_time_allowance'] = 35.0
    params['general_goal_checker']['xy_goal_tolerance'] = 0.25
    params['FollowPath'].update(min_vel_x=0.0, max_vel_x=0.30, min_vel_y=0.0,
        max_vel_y=0.0, max_vel_theta=0.35, max_speed_xy=0.30,
        acc_lim_x=0.3, decel_lim_x=-0.3, acc_lim_y=0.3, decel_lim_y=-0.3,
        acc_lim_theta=0.5, decel_lim_theta=-0.5, vy_samples=1,
        transform_tolerance=0.3, xy_goal_tolerance=0.25,
        critics=['Oscillation', 'ObstacleFootprint', 'PathAlign', 'GoalAlign', 'PathDist', 'GoalDist'])
    params['FollowPath'].update(
        plugin='nav2_rotation_shim_controller::RotationShimController',
        primary_controller='dwb_core::DWBLocalPlanner',
        angular_dist_threshold=0.5, angular_disengage_threshold=0.25,
        forward_sampling_distance=0.5, rotate_to_heading_angular_vel=0.35,
        max_angular_accel=0.5, simulate_ahead_time=2.0,
        rotate_to_goal_heading=False, closed_loop=False)
    # Heading critics from the community Go2 DWB profile: turning toward a path
    # must improve its score even when the body centre has not translated yet.
    params['FollowPath'].update({'PathAlign.scale': 32.0,
        'PathAlign.forward_point_distance': 0.1,
        'GoalAlign.scale': 24.0, 'GoalAlign.forward_point_distance': 0.1})
    with open(pkg + '/config/lidar3d_planner.yaml') as stream:
        local = copy.deepcopy(yaml.safe_load(stream)['global_costmap']['global_costmap'])
    cost = local['ros__parameters']
    cost.update(width=6, height=6, resolution=0.05, update_frequency=5.0,
                track_unknown_space=False,
                transform_tolerance=0.3)
    with open(pkg + '/config/go2_footprint.yaml') as stream:
        cost.update(yaml.safe_load(stream))
    cost.pop('robot_radius', None)
    cost['obstacle_layer']['lidar']['expected_update_rate'] = 0.8
    cost['obstacle_layer']['lidar']['topic'] = '/lidar3d/obstacle_points'
    cost['inflation_layer']['inflation_radius'] = 0.55
    fd, path = tempfile.mkstemp(prefix='go2-lidar3d-controller-', suffix='.yaml')
    with os.fdopen(fd, 'w') as stream:
        yaml.safe_dump({'controller_server': controller, 'local_costmap': {'local_costmap': local}}, stream)

    def cleanup(_context):
        os.unlink(path)
        return []

    with open(pkg + '/config/lidar3d_robot_viz.yaml') as stream:
        odom = yaml.safe_load(stream)['go2_odom_tf']['ros__parameters']
    odom.update(publish_odom=True, derive_planar_twist=True, publish_tf=False,
                odom_out_topic='/lidar3d/body_odom')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package='go2_nav2', executable='go2_odom_tf', name='lidar3d_body_odom', parameters=[odom]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[path]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_lidar3d_controller', parameters=[{
                 'autostart': True, 'node_names': ['controller_server'], 'bond_timeout': 4.0}]),
        Node(package='go2_nav2', executable='go2_goal_pose_nav', name='go2_goal_pose_nav',
             output='screen', parameters=[{'map_frame': 'lidar3d_map', 'pre_rotate_enable': False}]),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=configure)])
