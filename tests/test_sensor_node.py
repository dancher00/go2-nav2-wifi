"""Construct the real node to catch collisions with rclpy's internal state.

Run in an isolated ROS domain / network namespace; never publishes motion.
"""

from pathlib import Path
import sys
import unittest

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_nav2'))
from go2_nav2.cloud_stamp_sync import CloudStampSync
from go2_nav2.sensor_time import SharedSensorClock


class SensorNodeTests(unittest.TestCase):
    def test_sensor_clock_does_not_replace_ros_node_clock(self):
        rclpy.init()
        node = None
        try:
            node = CloudStampSync()
            self.assertIsInstance(node._sensor_clock, SharedSensorClock)
            self.assertGreater(node.get_clock().now().nanoseconds, 0)
            self.assertIsNot(node.get_clock(), node._sensor_clock)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
