#!/usr/bin/env python3
"""Translate native LiDAR cloud and odometry with the same fixed clock offset.

Both inputs must carry acquisition timestamps from the same source clock.
The historical executable name is retained for existing launch files.
"""

import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int64

from go2_nav2.sensor_time import ClockDiscontinuity, SharedSensorClock


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def set_stamp(stamp, value):
    stamp.sec, stamp.nanosec = divmod(value, 10**9)


class CloudStampSync(Node):
    def __init__(self):
        super().__init__("go2_cloud_stamp_sync")
        defaults = {
            'cloud_in': '/utlidar/cloud_deskewed',
            'cloud_out': '/utlidar/cloud_deskewed_sync',
            'odom_in': '/utlidar/robot_odom',
            'odom_out': '/utlidar/robot_odom_sync',
            'calibration_sec': 1.0,
            'time_margin_sec': 0.05,
            'max_age_sec': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        if value('cloud_in') == value('cloud_out') or value('odom_in') == value('odom_out'):
            raise ValueError('time synchronization input and output topics must differ')
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
        self.get_logger().info('Calibrating ONE source clock for cloud + odometry; no per-message restamping')

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
        set_stamp(msg.header.stamp, corrected)
        self._odom_pub.publish(msg)

    def _on_cloud(self, msg):
        if self._failed or self._first_odom_stamp is None:
            return
        mono = time.monotonic_ns()
        # Do not keep outputting scans when the pose stream is unavailable.
        if mono - self._last_odom_mono > self._sensor_clock.max_age_ns:
            self._sensor_clock.dropped['no_fresh_odom'] += 1
            return
        try:
            corrected = self._sensor_clock.map_stamp(
                'cloud', stamp_ns(msg.header.stamp), self.get_clock().now().nanoseconds, mono)
        except ClockDiscontinuity as exc:
            self._clock_fault(exc)
            return
        if corrected is None or corrected < self._first_odom_stamp:
            return  # TF has no history before the first published odometry.
        set_stamp(msg.header.stamp, corrected)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = CloudStampSync()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
