#!/usr/bin/env python3
"""Refresh PointCloud2 header stamps to the local ROS clock.

Wi-Fi relay preserves robot timestamps; go2_odom_tf publishes odom->base_link
with use_current_stamp. pointcloud_to_laserscan then cannot transform clouds in
frame odom (typical for cloud_deskewed) and drops /scan.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class CloudStampSync(Node):
    def __init__(self) -> None:
        super().__init__("go2_cloud_stamp_sync")
        self.declare_parameter("cloud_in", "/utlidar/cloud_deskewed")
        self.declare_parameter("cloud_out", "/utlidar/cloud_deskewed_sync")
        cloud_in = self.get_parameter("cloud_in").value
        cloud_out = self.get_parameter("cloud_out").value
        self._pub = self.create_publisher(
            PointCloud2, cloud_out, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, cloud_in, self._on_cloud, qos_profile_sensor_data
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CloudStampSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
