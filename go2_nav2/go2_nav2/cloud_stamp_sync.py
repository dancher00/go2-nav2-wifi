#!/usr/bin/env python3
"""Translate native cloud, odometry and optional IMU with one fixed clock offset.

Both inputs must carry acquisition timestamps from the same source clock.
The historical executable name is retained for existing launch files.
"""

import signal
import json
import time
import zlib

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import Int64

from go2_nav2.sensor_time import ClockDiscontinuity, SharedSensorClock


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def set_stamp(stamp, value):
    stamp.sec, stamp.nanosec = divmod(value, 10**9)


def record_trace(node, stream, msg, source_stamp, corrected):
    trace = getattr(node, '_trace_file', None)
    if trace is not None:
        crc = zlib.crc32(msg.data) if hasattr(msg, 'point_step') else None
        trace.write(json.dumps([stream, source_stamp, time.time_ns(), time.monotonic_ns(),
                                corrected, crc]) + '\n')


def record_bag(node, stream, msg):
    writer = getattr(node, '_bag_writer', None)
    if writer is not None:
        from rclpy.serialization import serialize_message
        # Payload has the same immutable clock translation as the published input.
        # Bag time is callback wall time, NOT sensor acquisition or E2E latency.
        writer.write(node._bag_topics[stream], serialize_message(msg), time.time_ns())


class CloudStampSync(Node):
    def __init__(self):
        super().__init__("go2_cloud_stamp_sync")
        defaults = {
            'cloud_in': '/utlidar/cloud_deskewed',
            'cloud_out': '/utlidar/cloud_deskewed_sync',
            'odom_in': '/utlidar/robot_odom',
            'odom_out': '/utlidar/robot_odom_sync',
            'kinematic_in': '',
            'kinematic_out': '',
            'lowstate_record_in': '',
            'imu_in': '',
            'imu_out': '',
            'trace_path': '',
            'bag_path': '',
            'calibration_sec': 1.0,
            'time_margin_sec': 0.05,
            'max_age_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self._trace_file = open(value('trace_path'), 'x', buffering=65536) if value('trace_path') else None
        if value('cloud_in') == value('cloud_out') or value('odom_in') == value('odom_out'):
            raise ValueError('time synchronization input and output topics must differ')
        if bool(value('imu_in')) != bool(value('imu_out')) or (value('imu_in') and value('imu_in') == value('imu_out')):
            raise ValueError('optional IMU requires distinct input and output topics')
        self._bag_writer = None
        if value('bag_path'):
            # Opt-in debugging only; no recorder node or additional DDS reader.
            import rosbag2_py
            self._bag_writer = rosbag2_py.SequentialWriter()
            self._bag_writer.open(
                rosbag2_py.StorageOptions(uri=value('bag_path'), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
            self._bag_topics = {'cloud': value('cloud_out'), 'odom': value('odom_out')}
            types = {'cloud': 'sensor_msgs/msg/PointCloud2', 'odom': 'nav_msgs/msg/Odometry'}
            if value('imu_out'):
                self._bag_topics['imu'] = value('imu_out')
                types['imu'] = 'sensor_msgs/msg/Imu'
            for stream, topic in self._bag_topics.items():
                self._bag_writer.create_topic(rosbag2_py.TopicMetadata(
                    name=topic, type=types[stream], serialization_format='cdr'))
            if value('kinematic_out'):
                self._bag_topics['kinematic'] = value('kinematic_out')
                self._bag_writer.create_topic(rosbag2_py.TopicMetadata(name=value('kinematic_out'), type='unitree_go/msg/SportModeState', serialization_format='cdr'))
            self.get_logger().info(f'Recording backend inputs to {value("bag_path")}')
            if value('lowstate_record_in'):
                self._bag_topics['lowstate'] = value('lowstate_record_in')
                self._bag_writer.create_topic(rosbag2_py.TopicMetadata(name=value('lowstate_record_in'), type='unitree_go/msg/LowState', serialization_format='cdr'))
        if self._bag_writer is not None and value('lowstate_record_in'):
            from unitree_go.msg import LowState
            # Record native CDR directly: avoid decoding 500 Hz motor packets in Python.
            self.create_subscription(LowState, value('lowstate_record_in'), self._on_lowstate,
                                     QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)
        self._sensor_clock = SharedSensorClock(value('calibration_sec'), value('time_margin_sec'),
                                               value('max_age_sec'))
        self._failed = False
        self._reported_offset = False
        self._first_odom_stamp = None
        self._last_odom_mono = None
        self._pub = self.create_publisher(PointCloud2, value('cloud_out'), qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(Odometry, value('odom_out'), 10)
        self._offset_pub = self.create_publisher(
            Int64, '/go2/sensor_time_offset_ns',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, value('odom_in'), self._on_odom, 50)
        self.create_subscription(PointCloud2, value('cloud_in'), self._on_cloud, qos_profile_sensor_data)
        if value('imu_in'):
            imu_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
            self._imu_pub = self.create_publisher(Imu, value('imu_out'), imu_qos)
            self.create_subscription(Imu, value('imu_in'), self._on_imu, imu_qos)
        if value('kinematic_in'):
            from unitree_go.msg import SportModeState
            self._kinematic_pub = self.create_publisher(SportModeState, value('kinematic_out'), qos_profile_sensor_data)
            self.create_subscription(SportModeState, value('kinematic_in'), self._on_kinematic, QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info('Calibrating ONE source clock for sensors + odometry; no per-message restamping')

    def _clock_fault(self, exc):
        if not self._failed:
            self.get_logger().error(f'SENSOR CLOCK FAULT: {exc}; forwarding stopped, no automatic rebase')
        self._failed = True

    def _on_odom(self, msg):
        if self._failed:
            return
        mono = time.monotonic_ns()
        try:
            corrected = self._sensor_clock.observe_odom(
                stamp_ns(msg.header.stamp), self.get_clock().now().nanoseconds, mono)
        except ClockDiscontinuity as exc:
            self._clock_fault(exc)
            return
        if corrected is None:
            return
        if not self._reported_offset:
            self._offset_pub.publish(Int64(data=self._sensor_clock.offset_ns))
            self.get_logger().info(f'Shared sensor offset locked: {self._sensor_clock.offset_ns} ns')
            self._reported_offset = True
        if self._first_odom_stamp is None:
            self._first_odom_stamp = corrected
        self._last_odom_mono = mono
        record_trace(self, 'odom', msg, stamp_ns(msg.header.stamp), corrected)
        set_stamp(msg.header.stamp, corrected)
        self._odom_pub.publish(msg)
        record_bag(self, 'odom', msg)

    def _on_cloud(self, msg):
        self._forward_sensor('cloud', msg, self._pub)

    def _on_imu(self, msg):
        self._forward_sensor('imu', msg, self._imu_pub)

    def _forward_sensor(self, stream, msg, publisher):
        if self._failed or self._first_odom_stamp is None:
            return
        mono = time.monotonic_ns()
        # Do not keep outputting scans when the pose stream is unavailable.
        if mono - self._last_odom_mono > self._sensor_clock.max_age_ns:
            self._sensor_clock.dropped['no_fresh_odom'] += 1
            return
        try:
            corrected = self._sensor_clock.map_stamp(
                stream, stamp_ns(msg.stamp if stream == "kinematic" else msg.header.stamp), self.get_clock().now().nanoseconds, mono)
        except ClockDiscontinuity as exc:
            self._clock_fault(exc)
            return
        if corrected is None or corrected < self._first_odom_stamp:
            return  # TF has no history before the first published odometry.
        message_stamp = msg.stamp if stream == "kinematic" else msg.header.stamp
        record_trace(self, stream, msg, stamp_ns(message_stamp), corrected)
        set_stamp(message_stamp, corrected)
        publisher.publish(msg)
        record_bag(self, stream, msg)

    def _on_lowstate(self, cdr):
        self._bag_writer.write(self._bag_topics['lowstate'], cdr, time.time_ns())

    def _on_kinematic(self, msg):
        self._forward_sensor("kinematic", msg, self._kinematic_pub)

    def destroy_node(self):
        if self._trace_file is not None:
            self._trace_file.close()
        # Humble's writer finalizes metadata in its destructor.
        self._bag_writer = None
        return super().destroy_node()


def main():
    rclpy.init()
    node = CloudStampSync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can interrupt take_message() while SIGINT closes the context.
        # A runtime failure during normal operation must still propagate.
        if rclpy.ok():
            raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
