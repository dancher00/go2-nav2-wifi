#!/usr/bin/env python3
"""odom -> base_link TF from /utlidar/robot_odom (same as unitree_go2_nav odomTf.cpp).

Optional smoothing only for RViz; keep smooth_alpha:=1.0 for SLAM/Nav2.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self) -> None:
        super().__init__("go2_odom_tf")
        self.declare_parameter("odom_topic", "/utlidar/robot_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # 1.0 = passthrough like ref; 0.35 = extra smoothing for RViz only
        self.declare_parameter("smooth_alpha", 1.0)
        self.declare_parameter("max_jump_xy", 0.5)
        self.declare_parameter("max_jump_yaw", 0.8)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_out_topic", "/odom")
        # Fresh TF stamps (robot_odom header can lag vs controller clock).
        self.declare_parameter("use_current_stamp", True)

        topic = self.get_parameter("odom_topic").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._alpha = float(self.get_parameter("smooth_alpha").value)
        self._max_jump_xy = float(self.get_parameter("max_jump_xy").value)
        self._max_jump_yaw = float(self.get_parameter("max_jump_yaw").value)
        self._use_current_stamp = bool(
            self.get_parameter("use_current_stamp").value
        )
        self._publish_odom = bool(self.get_parameter("publish_odom").value)
        odom_out = self.get_parameter("odom_out_topic").value

        self._br = TransformBroadcaster(self)
        self._odom_pub = (
            self.create_publisher(Odometry, odom_out, 10) if self._publish_odom else None
        )
        self._sub = self.create_subscription(Odometry, topic, self._on_odom, 50)
        self._have_state = False
        self._got_odom = False
        self._x = self._y = self._z = 0.0
        self._qx = self._qy = self._qz = self._qw = 1.0
        self.create_timer(1.0 / 30.0, self._publish_tf)

    def _publish_tf(self) -> None:
        if not self._got_odom:
            self._x = self._y = self._z = 0.0
            self._qx = self._qy = self._qz = 0.0
            self._qw = 1.0
        stamp = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = self._z
        t.transform.rotation.x = self._qx
        t.transform.rotation.y = self._qy
        t.transform.rotation.z = self._qz
        t.transform.rotation.w = self._qw
        self._br.sendTransform(t)

    def _on_odom(self, msg: Odometry) -> None:
        if not self._got_odom:
            self._got_odom = True
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        if self._alpha >= 1.0:
            self._x, self._y, self._z = x, y, z
            self._qx, self._qy, self._qz, self._qw = qx, qy, qz, qw
        else:
            if not self._have_state:
                self._x, self._y, self._z = x, y, z
                self._qx, self._qy, self._qz, self._qw = qx, qy, qz, qw
                self._have_state = True
            else:
                dx, dy = x - self._x, y - self._y
                siny = 2.0 * (qw * qz + qx * qy)
                cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
                yaw = math.atan2(siny, cosy)
                py = 2.0 * (self._qw * self._qz + self._qx * self._qy)
                pc = 1.0 - 2.0 * (self._qy * self._qy + self._qz * self._qz)
                prev_yaw = math.atan2(py, pc)
                dyaw = math.atan2(math.sin(yaw - prev_yaw), math.cos(yaw - prev_yaw))
                if math.hypot(dx, dy) > self._max_jump_xy or abs(dyaw) > self._max_jump_yaw:
                    self.get_logger().warn(
                        f"robot_odom jump dx={dx:.2f} dy={dy:.2f} dyaw={dyaw:.2f}",
                        throttle_duration_sec=2.0,
                    )
                a = self._alpha
                self._x = (1.0 - a) * self._x + a * x
                self._y = (1.0 - a) * self._y + a * y
                self._z = (1.0 - a) * self._z + a * z
                self._qx = (1.0 - a) * self._qx + a * qx
                self._qy = (1.0 - a) * self._qy + a * qy
                self._qz = (1.0 - a) * self._qz + a * qz
                self._qw = (1.0 - a) * self._qw + a * qw
                n = math.sqrt(
                    self._qx * self._qx
                    + self._qy * self._qy
                    + self._qz * self._qz
                    + self._qw * self._qw
                )
                if n > 1e-9:
                    self._qx, self._qy, self._qz, self._qw = (
                        self._qx / n,
                        self._qy / n,
                        self._qz / n,
                        self._qw / n,
                    )

        self._publish_tf()
        if self._odom_pub is not None:
            stamp = (
                self.get_clock().now().to_msg()
                if self._use_current_stamp
                else msg.header.stamp
            )
            out = Odometry()
            out.header.stamp = stamp
            out.header.frame_id = self._odom_frame
            out.child_frame_id = self._base_frame
            out.pose.pose.position.x = self._x
            out.pose.pose.position.y = self._y
            out.pose.pose.position.z = self._z
            out.pose.pose.orientation.x = self._qx
            out.pose.pose.orientation.y = self._qy
            out.pose.pose.orientation.z = self._qz
            out.pose.pose.orientation.w = self._qw
            out.twist = msg.twist
            self._odom_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
