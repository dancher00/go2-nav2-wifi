#!/usr/bin/env python3
"""odom -> base_link TF from shared-clock /utlidar/robot_odom_sync.

Optional smoothing only for RViz; keep smooth_alpha:=1.0 for SLAM/Nav2.
"""

from __future__ import annotations

import math

import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def compose_pose(position, quaternion, offset):
    """T_world_sensor * T_sensor_body; quaternion order xyzw."""
    x, y, z, w = quaternion
    a, b, c = offset[:3]
    # Rotate translation using q * v * inverse(q).
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    p = (position[0]+a+w*tx+y*tz-z*ty,
         position[1]+b+w*ty+z*tx-x*tz,
         position[2]+c+w*tz+x*ty-y*tx)
    i, j, k, l = offset[3:]
    return p, (w*i+x*l+y*k-z*j, w*j-x*k+y*l+z*i,
               w*k+x*j-y*i+z*l, w*l-x*i-y*j-z*k)
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    def __init__(self) -> None:
        super().__init__("go2_odom_tf")
        self.declare_parameter("sensor_from_base", [0., 0., 0., 0., 0., 0., 1.])
        self.declare_parameter("best_effort", False)
        self._sensor_from_base = self.get_parameter("sensor_from_base").value
        self.declare_parameter("odom_topic", "/utlidar/robot_odom_sync")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # 1.0 = passthrough like ref; 0.35 = extra smoothing for RViz only
        self.declare_parameter("smooth_alpha", 1.0)
        self.declare_parameter("max_jump_xy", 0.5)
        self.declare_parameter("max_jump_yaw", 0.8)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("odom_out_topic", "/odom")
        # Acquisition time has already been translated by the shared clock node.
        self.declare_parameter("use_current_stamp", False)

        topic = self.get_parameter("odom_topic").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._alpha = float(self.get_parameter("smooth_alpha").value)
        self._max_jump_xy = float(self.get_parameter("max_jump_xy").value)
        self._max_jump_yaw = float(self.get_parameter("max_jump_yaw").value)
        self._use_current_stamp = bool(
            self.get_parameter("use_current_stamp").value
        )
        if self._use_current_stamp:
            raise ValueError('use_current_stamp=true breaks cloud/pose alignment; use the shared clock node')
        self._stamp = None
        self._last_stamp_ns = 0
        self._publish_odom = bool(self.get_parameter("publish_odom").value)
        odom_out = self.get_parameter("odom_out_topic").value

        self._br = TransformBroadcaster(self)
        self._odom_pub = (
            self.create_publisher(Odometry, odom_out, 10) if self._publish_odom else None
        )
        self._sub = self.create_subscription(Odometry, topic, self._on_odom,
            qos_profile_sensor_data if self.get_parameter("best_effort").value else 50)
        self._have_state = False
        self._got_odom = False
        self._x = self._y = self._z = 0.0
        self._qx = self._qy = self._qz = self._qw = 1.0
        # No timer: old poses must never masquerade as fresh measurements.

    def _publish_tf(self) -> None:
        if self._stamp is None:
            return
        t = TransformStamped()
        t.header.stamp = self._stamp
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
        incoming_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if incoming_ns <= self._last_stamp_ns:
            return
        self._last_stamp_ns = incoming_ns
        self._stamp = msg.header.stamp
        if not self._got_odom:
            self._got_odom = True
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        offset = getattr(self, "_sensor_from_base", (0., 0., 0., 0., 0., 0., 1.))
        (x, y, z), (qx, qy, qz, qw) = compose_pose((x, y, z), (qx, qy, qz, qw), offset)

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
            out = Odometry()
            out.header.stamp = self._stamp
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
            out.pose.covariance = msg.pose.covariance
            self._odom_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launchers can forward a second signal while cleanup is in progress.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
