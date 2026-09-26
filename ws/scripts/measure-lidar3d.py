#!/usr/bin/env python3
"""Finite read-only sensor/backend observer. Source stamps never become wall-time latency."""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import struct
import time
import zlib

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


def distribution(values):
    if not values:
        return {}
    ordered = sorted(values)
    return {'min': ordered[0], 'median': statistics.median(ordered),
            'p95': ordered[min(len(ordered)-1, math.ceil(len(ordered)*.95)-1)],
            'max': ordered[-1]}


def cloud_schema(msg):
    result = {'point_step': msg.point_step, 'row_step': msg.row_step,
              'width': msg.width, 'height': msg.height, 'is_bigendian': msg.is_bigendian,
              'fields': [], 'ranges': {}}
    for field in msg.fields:
        result['fields'].append({'name': field.name, 'offset': field.offset,
                                 'datatype': field.datatype, 'count': field.count})
        fmt = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}.get(field.datatype)
        if fmt and field.count == 1:
            decoder = struct.Struct(('>' if msg.is_bigendian else '<') + fmt)
            values = [decoder.unpack_from(msg.data, row*msg.row_step+col*msg.point_step+field.offset)[0]
                      for row in range(msg.height) for col in range(msg.width)]
            values = [v for v in values if math.isfinite(v)]
            result['ranges'][field.name] = [min(values), max(values)] if values else None
    return result


def processes():
    """Only project/backend leaf processes; CPU is percent of one logical core."""
    names = {'robot_relay_wifi.py', 'go2_cloud_stamp_sync', 'icp_odometry', 'rtabmap', 'pointlio_mapping', 'legkilo_node', 'rviz2'}
    result = {}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            argv = (path / 'cmdline').read_bytes().split(b'\0')
            matched = next((os.path.basename(a.decode()) for a in argv[:2]
                            if os.path.basename(a.decode()) in names), None)
            if not matched:
                continue
            fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
            result[path.name] = {'name': matched, 'ticks': int(fields[11])+int(fields[12]),
                                 'rss_mib': int(fields[21])*os.sysconf('SC_PAGE_SIZE')/2**20}
        except (OSError, ValueError, UnicodeError):
            continue
    return result


def interface_bytes(name):
    if not name:
        return None
    base = Path('/sys/class/net') / name / 'statistics'
    return {key: int((base / (key + '_bytes')).read_text()) for key in ('rx', 'tx')}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('source', 'laptop', 'backend', 'pipeline', 'resources'), default='resources')
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--interface')
    parser.add_argument('--inspect-deskewed', action='store_true', help='Also inspect factory processed cloud on source; adds observer load')
    parser.add_argument('--output', required=True, help='New JSON file, including exact source stamp records')
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')
    # Reserve this file before starting; a previous measurement is never replaced.
    output = Path(args.output).open('x')
    topics = {
        '/utlidar/cloud': 'sensor_msgs/msg/PointCloud2',
        '/utlidar/imu': 'sensor_msgs/msg/Imu',
        '/utlidar/robot_odom': 'nav_msgs/msg/Odometry',
        '/cmd_vel': 'geometry_msgs/msg/Twist',
    }
    if args.side in ('backend', 'pipeline'):
        topics = {'/cmd_vel': 'geometry_msgs/msg/Twist'}
    if args.side == 'source' and args.inspect_deskewed:
        topics['/utlidar/cloud_deskewed'] = 'sensor_msgs/msg/PointCloud2'
    if args.side != 'source':
        topics.update({
            '/lidar3d/cloud_sync': 'sensor_msgs/msg/PointCloud2',
            '/lidar3d/imu_sync': 'sensor_msgs/msg/Imu',
            '/lidar3d/odom': 'nav_msgs/msg/Odometry',
            '/lidar3d/registered': 'sensor_msgs/msg/PointCloud2',
            '/lidar3d/path': 'nav_msgs/msg/Path',
            '/go2/sensor_time_offset_ns': 'std_msgs/msg/Int64',
        })
    if args.side == 'pipeline':
        topics['/lidar3d/factory_odom_clock_reference'] = 'nav_msgs/msg/Odometry'
    if args.side == 'resources':
        topics = {}
    node = None
    if topics:
        rclpy.init()
        node = rclpy.create_node('go2_lidar3d_measure')
    records = {topic: {'samples': []} for topic in topics}
    def callback(topic):
        def receive(msg):
            wall, mono = time.time_ns(), time.monotonic_ns()
            row = records[topic]
            stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec if hasattr(msg, 'header') else 0
            crc = zlib.crc32(msg.data) if hasattr(msg, 'point_step') else None
            row['samples'].append([stamp, wall, mono, len(serialize_message(msg)), crc])
            if hasattr(msg, 'header'):
                row['frame'] = msg.header.frame_id
            if hasattr(msg, 'point_step') and 'schema' not in row:
                row['schema'] = cloud_schema(msg)
            if hasattr(msg, 'child_frame_id'):
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                if all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:])-1) < .01:
                    row.setdefault('poses', []).append([stamp, *values])
                else:
                    row['invalid_poses'] = row.get('invalid_poses', 0) + 1
                row['child_frame'] = msg.child_frame_id
            if hasattr(msg, 'linear_acceleration'):
                a, g = msg.linear_acceleration, msg.angular_velocity
                row.setdefault('imu', []).append([a.x, a.y, a.z, g.x, g.y, g.z])
                row['orientation_covariance'] = list(msg.orientation_covariance)
                row['acceleration_covariance'] = list(msg.linear_acceleration_covariance)
            if hasattr(msg, 'loop_closure_id'):
                row.setdefault('closures', []).append([stamp, msg.ref_id, msg.loop_closure_id,
                                                       msg.proximity_detection_id])
            if hasattr(msg, 'poses'):
                row['path_pose_count'] = len(msg.poses)
            if topic == '/go2/sensor_time_offset_ns':
                row['offset_ns'] = msg.data
        return receive
    for topic, ros_type in topics.items():
        latched = topic in ('/lidar3d/cloud_map', '/go2/sensor_time_offset_ns')
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE if latched else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
        node.create_subscription(get_message(ros_type), topic, callback(topic), qos)
    wall_start, start = time.time_ns(), time.monotonic()
    net_start = interface_bytes(args.interface)
    proc_samples = {}
    next_sample = start
    try:
        while time.monotonic()-start < args.duration:
            if node is not None:
                rclpy.spin_once(node, timeout_sec=.02)
            else:
                time.sleep(.02)
            now = time.monotonic()
            if now >= next_sample:
                for pid, values in processes().items():
                    proc_samples.setdefault(pid, []).append((now, values))
                next_sample = now+.5
        duration = time.monotonic()-start
        proc_report = {}
        for pid, samples in proc_samples.items():
            first, last = samples[0], samples[-1]
            elapsed = last[0]-first[0]
            proc_report[pid] = {'name': last[1]['name'],
                                'cpu_percent_one_core': 100*(last[1]['ticks']-first[1]['ticks'])/os.sysconf('SC_CLK_TCK')/elapsed if elapsed else None,
                                'rss_peak_mib': max(v['rss_mib'] for _, v in samples)}
        for topic, row in records.items():
            samples = row['samples']
            row['count'] = len(samples)
            row['window_hz'] = len(samples)/duration
            if not samples:
                continue
            span = (samples[-1][2]-samples[0][2])*1e-9
            row['hz'] = (len(samples)-1)/span if span else None
            row['cdr_payload_mbit_s'] = sum(s[3] for s in samples)*8/duration/1e6
            stamps = [s[0] for s in samples if s[0]]
            row['duplicates'] = len(stamps)-len(set(stamps))
            row['reordered'] = sum(b<a for a,b in zip(stamps, stamps[1:]))
            row['arrival_interval_ms'] = distribution([(b[2]-a[2])/1e6 for a,b in zip(samples,samples[1:])])
            row['source_interval_ms'] = distribution([(b-a)/1e6 for a,b in zip(stamps,stamps[1:])])
            row['wall_minus_header_ms_NOT_latency'] = distribution([(s[1]-s[0])/1e6 for s in samples if s[0]])
            if 'imu' in row:
                row['imu_mean'] = [statistics.mean(v[i] for v in row['imu']) for i in range(6)]
                row['imu_std'] = [statistics.pstdev(v[i] for v in row['imu']) for i in range(6)]
            if 'poses' in row:
                first = row['poses'][0]
                row['displacement_from_first_m'] = distribution([math.sqrt(sum((p[i]-first[i])**2 for i in (1,2,3))) for p in row['poses']])
        net_end = interface_bytes(args.interface)
        report = {'side': args.side, 'duration_s': duration, 'wall_start_ns': wall_start,
                  'wall_end_ns': time.time_ns(), 'ros_domain': os.environ.get('ROS_DOMAIN_ID'),
                  'sample_columns': ['source_stamp_ns', 'observer_wall_ns', 'observer_monotonic_ns',
                                     'serialized_cdr_bytes', 'cloud_data_crc32'],
                  'interface_mbit_s_ALL_traffic': {k:(net_end[k]-net_start[k])*8/duration/1e6 for k in net_start} if net_start else None,
                  'processes': proc_report, 'topics': records}
        json.dump(report, output, indent=2)
        output.write('\n')
        summary = dict(report, topics={k:{a:b for a,b in v.items() if a not in ('samples','poses','imu','closures')}
                                       for k,v in records.items()})
        print(json.dumps(summary, indent=2))
    finally:
        output.close()
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
