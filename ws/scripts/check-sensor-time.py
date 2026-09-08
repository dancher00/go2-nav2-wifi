#!/usr/bin/env python3
"""Read-only check of the native LiDAR shared-clock pipeline (no motion commands)."""

import argparse
import json
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Int64
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0)
    args = parser.parse_args()
    if not 0 < args.duration <= 300:
        parser.error('duration must be between 0 and 300 seconds')
    rclpy.init()
    node = rclpy.create_node('go2_read_only_sensor_time_check')
    samples = {}
    offsets = set()
    measuring = False
    observer_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)

    def receive(name, msg):
        if measuring:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            samples[name].append((stamp, (node.get_clock().now().nanoseconds - stamp) / 1e6))

    def callback(name):
        def accept(msg):
            receive(name, msg)
        return accept

    for name, topic, kind in (
        ('raw_cloud', '/utlidar/cloud_deskewed', PointCloud2),
        ('raw_odom', '/utlidar/robot_odom', Odometry),
        ('sync_cloud', '/utlidar/cloud_deskewed_sync', PointCloud2),
        ('sync_odom', '/utlidar/robot_odom_sync', Odometry),
        ('odom', '/odom', Odometry),
        ('scan', '/scan', LaserScan),
    ):
        samples[name] = []
        node.create_subscription(kind, topic, callback(name),
                                 observer_qos)
    samples['odom_tf'] = []

    def receive_tf(msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                receive('odom_tf', transform)

    node.create_subscription(TFMessage, '/tf', receive_tf, observer_qos)
    node.create_subscription(Int64, '/go2/sensor_time_offset_ns',
                             lambda msg: offsets.add(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        measuring = True
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = time.monotonic() - start
        report = {'duration_sec': round(elapsed, 3), 'offsets_ns': sorted(offsets), 'streams': {}}
        stamps = {}
        for name, rows in samples.items():
            stamps[name] = {stamp for stamp, _ in rows}
            ages = [age for _, age in rows]
            report['streams'][name] = {
                'messages': len(rows), 'hz': round(len(rows) / elapsed, 2),
                'duplicates': len(rows) - len(stamps[name]),
                'median_age_ms': round(statistics.median(ages), 3) if ages else None,
                'max_age_ms': round(max(ages), 3) if ages else None,
            }
        checks = {}
        if len(offsets) == 1:
            offset = next(iter(offsets))
            for source, target, shift in (
                ('raw_cloud', 'sync_cloud', offset), ('raw_odom', 'sync_odom', offset),
                ('sync_odom', 'odom', 0), ('sync_odom', 'odom_tf', 0),
                ('sync_cloud', 'scan', 0),
            ):
                expected = {stamp + shift for stamp in stamps[source]}
                observed = stamps[target]
                # Exclude measurement-window edges, but not missing interior packets.
                lower, upper = (min(expected), max(expected)) if expected else (1, 0)
                interior = {stamp for stamp in observed if lower <= stamp <= upper}
                matched = len(interior & expected)
                checks[source + ' -> ' + target] = {
                    'matched_stamps': matched,
                    'unmatched_stamps': len(interior - expected),
                    'passed': matched >= 5 and matched / max(len(interior), 1) >= 0.95,
                }
        # BEST_EFFORT monitoring can itself lose packets: allow up to 5% unmatched
        # observations, reporting their exact count rather than hiding the loss.
        passed = len(checks) == 5 and all(check['passed'] for check in checks.values())
        passed = passed and all(rows and len(rows) == len(stamps[name])
                                for name, rows in samples.items())
        report.update(checks=checks, passed=bool(passed))
        print(json.dumps(report, indent=2), flush=True)
        return 0 if passed else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
