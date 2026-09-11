#!/usr/bin/env python3
"""Bounded, read-only live check. Run on Jetson to keep raw images off Wi-Fi."""
import argparse
import json
import math
import time
from collections import defaultdict

import rclpy
import numpy as np
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import Image, Imu, PointCloud2
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('d435i_health_check')
    counts = defaultdict(int)
    stamps = defaultdict(list)
    details = {}
    edges = set()
    positions = []
    loops = set()
    depth_coverage = []
    infrared_saturation = []
    best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(key, msg):
        counts[key] += 1
        if hasattr(msg, 'header'):
            stamps[key].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if isinstance(msg, Image):
            details[key] = {'width': msg.width, 'height': msg.height, 'encoding': msg.encoding}
            if counts[key] % 30 == 1 and msg.encoding in ('16UC1', 'mono8'):
                dtype = np.dtype('>u2' if msg.is_bigendian else '<u2') if msg.encoding == '16UC1' else np.dtype('u1')
                pixels = np.ndarray((msg.height, msg.width), dtype=dtype,
                                    buffer=bytes(msg.data), strides=(msg.step, dtype.itemsize))[::4, ::4]
                if key == 'depth':
                    depth_coverage.append(float(np.mean((pixels > 400) & (pixels < 4000))))
                elif key.startswith('infra'):
                    infrared_saturation.append(float(np.mean(pixels >= 254)))
        elif isinstance(msg, PointCloud2):
            details[key] = {'points': msg.width * msg.height, 'frame': msg.header.frame_id}
        elif isinstance(msg, Odometry):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            valid = all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            valid = valid and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1) < 0.05
            valid = valid and 0 <= msg.pose.covariance[0] < 9999
            counts['valid_odom'] += int(valid)
            if valid:
                positions.append((p.x, p.y, p.z))
            details[key] = {'position': [p.x, p.y, p.z], 'covariance_x': msg.pose.covariance[0],
                            'frame': msg.header.frame_id, 'child': msg.child_frame_id}
        elif isinstance(msg, Info):
            details[key] = {'reference_id': msg.ref_id, 'working_memory_nodes': len(msg.wm_state)}
            if msg.loop_closure_id > 0:
                loops.add((msg.ref_id, msg.loop_closure_id))
        elif isinstance(msg, TFMessage):
            for transform in msg.transforms:
                edges.add((transform.header.frame_id, transform.child_frame_id))

    subscriptions = []
    for key, msg_type, topic, qos in [
        ('rgb', Image, '/camera/color/image_raw', best_effort),
        ('depth', Image, '/camera/aligned_depth_to_color/image_raw', best_effort),
        # Optional diagnostics; infra streams need not be enabled for mapping.
        ('infra1', Image, '/camera/infra1/image_rect_raw', best_effort),
        ('infra2', Image, '/camera/infra2/image_rect_raw', best_effort),
        ('imu_raw', Imu, '/camera/imu', best_effort),
        ('imu', Imu, '/d435i/imu', best_effort),
        ('odom', Odometry, '/d435i/odom', best_effort),
        ('cloud', PointCloud2, '/d435i/cloud_map', latched),
        ('mapping', Info, '/d435i/info', best_effort),
        ('tf', TFMessage, '/tf', best_effort),
        ('tf_static', TFMessage, '/tf_static', QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]:
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, key=key: receive(key, msg), qos))
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
        rates = {key: (len(values)-1)/(values[-1]-values[0])
                 for key, values in stamps.items() if len(values)>1 and values[-1]>values[0]}
        required = ['rgb', 'depth', 'imu_raw', 'imu', 'odom', 'valid_odom', 'cloud', 'mapping']
        failures = [key + ': no data' for key in required if counts[key] == 0]
        if counts['valid_odom'] != counts['odom']:
            failures.append('invalid/lost odometry samples')
        if details.get('cloud', {}).get('points', 0) == 0:
            failures.append('empty cloud map')
        body = details.get('odom', {}).get('child', 'camera_link')
        required_edges = [('d435i_map', 'd435i_odom'), ('d435i_odom', body)]
        if body != 'camera_link':
            required_edges.append((body, 'camera_link'))
        for edge in required_edges:
            if edge not in edges:
                failures.append('missing TF ' + ' -> '.join(edge))
        warnings = []
        if depth_coverage and np.median(depth_coverage) < 0.3:
            warnings.append('Less than 30% usable depth at 0.4–4 m; inspect scene and stereo exposure before mapping.')
        if infrared_saturation and np.median(infrared_saturation) > 0.2:
            warnings.append('More than 20% saturated IR pixels; stereo exposure is unsuitable.')
        print(json.dumps({'ok': not failures, 'scope': 'transport and sampled input quality; not map accuracy',
                          'warnings': warnings,
                          'depth_usable_fraction_median': float(np.median(depth_coverage)) if depth_coverage else None,
                          'infra_saturated_fraction_median': float(np.median(infrared_saturation)) if infrared_saturation else None,
                          'seconds': args.seconds, 'counts': dict(counts),
                          'stamp_rate_hz': rates, 'details': details, 'tf_edges': sorted(edges),
                          'position_span_m': [max(p[i] for p in positions)-min(p[i] for p in positions) for i in range(3)] if positions else [],
                          'loop_closures': sorted(loops), 'failures': failures}, indent=2))
        return bool(failures)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
