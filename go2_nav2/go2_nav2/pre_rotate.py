"""Turn in place toward goal before Nav2 plans (goal behind the robot)."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def _yaw_from_tf(t) -> float:
    q = t.transform.rotation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def maybe_rotate_toward_goal(
    node: Node,
    goal: PoseStamped,
    map_frame: str,
    base_frame: str = "base_link",
    *,
    start_threshold_rad: float = 1.0,
    stop_threshold_rad: float = 0.45,
    max_wz: float = 0.85,
    timeout_sec: float = 12.0,
) -> None:
    """Publish /cmd_vel yaw only when goal is mostly behind/side (>~57 deg)."""
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    time.sleep(0.15)

    gx = goal.pose.position.x
    gy = goal.pose.position.y
    deadline = time.monotonic() + timeout_sec
    started = False

    while time.monotonic() < deadline and rclpy_ok(node):
        try:
            tf = buffer.lookup_transform(
                map_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            time.sleep(0.05)
            continue

        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        yaw = _yaw_from_tf(tf)
        bearing = math.atan2(gy - ry, gx - rx)
        err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))

        if abs(err) < stop_threshold_rad:
            if started:
                node.get_logger().info(
                    f"Pre-rotate done (bearing err {math.degrees(err):.0f} deg)"
                )
            break

        if abs(err) < start_threshold_rad:
            break

        if not started:
            node.get_logger().info(
                f"Pre-rotate toward goal ({math.degrees(err):.0f} deg) — goal behind/side"
            )
            started = True

        twist = Twist()
        twist.angular.z = max(-max_wz, min(max_wz, err * 1.2))
        cmd_pub.publish(twist)
        time.sleep(0.05)

    stop = Twist()
    cmd_pub.publish(stop)
    time.sleep(0.1)


def rclpy_ok(node: Node) -> bool:
    return rclpy.ok()
