#!/usr/bin/env python3
"""Read-only ROS graph and raw-sensor progression check before managed startup."""

import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

STACK_NODES = {'slam_toolbox', 'go2_odom_tf', 'go2_cloud_stamp_sync', 'map_server',
               'controller_server', 'planner_server', 'lidar3d_lio'}
MOTION_NODES = {'go2_cmd_vel_tcp_client', 'teleop_twist_keyboard', 'go2_goal_pose_nav', 'go2_patrol'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('mapping', 'lidar3d', 'navigation', 'teleop', 'transport'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('go2_session_preflight')
    samples = {'cloud': [], 'odom': []}

    def callback(key):
        def receive(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rows = samples[key]
            if stamp > 0 and (not rows or stamp > rows[-1][0]):
                rows.append((stamp, time.monotonic()))
        return receive

    node.create_subscription(Odometry, '/utlidar/robot_odom', callback('odom'), qos_profile_sensor_data)
    cloud_topic = '/utlidar/cloud' if args.mode == 'lidar3d' else '/utlidar/cloud_deskewed'
    node.create_subscription(PointCloud2, cloud_topic, callback('cloud'), qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        names = set(node.get_node_names())
        forbidden = set()
        if args.mode in ('mapping', 'lidar3d', 'navigation'):
            forbidden |= STACK_NODES
        # Mapping may coexist with this package's managed keyboard teleop.
        if args.mode not in ('mapping', 'lidar3d'):
            forbidden |= MOTION_NODES
        conflicts = sorted(names & forbidden)
        now = time.monotonic()
        fresh = all(len(rows) >= 5 and now - rows[-1][1] < .5 and
                    abs((rows[-1][0] - rows[0][0]) - (rows[-1][1] - rows[0][1])) < .5
                    for rows in samples.values())
        print(json.dumps({'raw_sensors_progressing': fresh, 'conflicting_nodes': conflicts,
                          'note': 'Startup check only; raw sensor clock epoch may differ.'}), flush=True)
        if conflicts:
            print('Stop the existing launch in its own terminal. It will NOT be killed automatically.', flush=True)
        return 0 if fresh and not conflicts else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
