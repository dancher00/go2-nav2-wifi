#!/usr/bin/env python3
"""Convert Unitree sport state to nav_msgs/Odometry.

Used as optional odom source for SLAM when topic `/sportmodestate` is available.
The message layout varies between Unitree stacks, so this bridge uses
best-effort field extraction.
"""

from __future__ import annotations

import math
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


def _get_attr(obj: Any, path: str) -> Any:
    cur = obj
    for part in path.split("."):
        if not hasattr(cur, part):
            return None
        cur = getattr(cur, part)
    return cur


def _float_at(obj: Any, idx: int) -> float | None:
    if obj is None:
        return None
    try:
        return float(obj[idx])
    except Exception:
        return None


def _first_attr(obj: Any, paths: list[str]) -> Any:
    for path in paths:
        val = _get_attr(obj, path)
        if val is not None:
            return val
    return None


def _first_float(msg: Any, paths: list[str], default: float = 0.0) -> float:
    for path in paths:
        val = _get_attr(msg, path)
        if val is None:
            continue
        try:
            return float(val)
        except Exception:
            continue
    return default


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class SportStateOdom(Node):
    def __init__(self) -> None:
        super().__init__("go2_sport_state_odom")
        self.declare_parameter("state_topic", "/sportmodestate")
        self.declare_parameter("state_type", "unitree_go/msg/SportModeState")
        self.declare_parameter("odom_out_topic", "/sport_state/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_current_stamp", True)

        state_topic = str(self.get_parameter("state_topic").value)
        state_type = str(self.get_parameter("state_type").value)
        odom_out_topic = str(self.get_parameter("odom_out_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._use_current_stamp = bool(self.get_parameter("use_current_stamp").value)

        msg_type = get_message(state_type)
        self._pub = self.create_publisher(Odometry, odom_out_topic, 10)
        self._sub = self.create_subscription(msg_type, state_topic, self._on_state, 30)

    def _on_state(self, msg: Any) -> None:
        x = _first_float(msg, ["position.x", "pose.position.x"], 0.0)
        y = _first_float(msg, ["position.y", "pose.position.y"], 0.0)
        z = _first_float(msg, ["position.z", "pose.position.z"], 0.0)
        if x == 0.0 and y == 0.0:
            pos = _get_attr(msg, "position")
            x0 = _float_at(pos, 0)
            y0 = _float_at(pos, 1)
            z0 = _float_at(pos, 2)
            if x0 is not None:
                x = x0
            if y0 is not None:
                y = y0
            if z0 is not None:
                z = z0

        yaw = _first_float(
            msg,
            [
                "yaw",
            ],
            0.0,
        )
        if yaw == 0.0:
            rpy = _first_attr(msg, ["imu_state.rpy", "rpy"])
            y2 = _float_at(rpy, 2)
            if y2 is not None:
                yaw = y2

        vx = _first_float(msg, ["velocity.x", "vel.x", "body_vel.x"], 0.0)
        vy = _first_float(msg, ["velocity.y", "vel.y", "body_vel.y"], 0.0)
        vyaw = _first_float(msg, ["yaw_speed", "velocity.z", "vel.z"], 0.0)

        if vx == 0.0 and vy == 0.0:
            vel = _first_attr(msg, ["velocity", "vel", "body_vel"])
            x0 = _float_at(vel, 0)
            y0 = _float_at(vel, 1)
            if x0 is not None:
                vx = x0
            if y0 is not None:
                vy = y0

        if vyaw == 0.0:
            gyro = _first_attr(msg, ["imu_state.gyroscope", "gyro"])
            z2 = _float_at(gyro, 2)
            if z2 is not None:
                vyaw = z2

        qx, qy, qz, qw = _yaw_to_quat(yaw)
        odom = Odometry()
        odom.header.stamp = (
            self.get_clock().now().to_msg()
            if self._use_current_stamp
            else getattr(getattr(msg, "header", None), "stamp", self.get_clock().now().to_msg())
        )
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vyaw
        self._pub.publish(odom)


def main() -> None:
    rclpy.init()
    node = SportStateOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
