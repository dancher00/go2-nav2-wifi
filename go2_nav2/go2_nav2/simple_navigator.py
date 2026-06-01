#!/usr/bin/env python3
"""Send one Nav2 goal from CLI: pose:="x y yaw"."""

from __future__ import annotations

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from go2_nav2.nav2_goal_exec import Nav2GoalExecutor


class SimpleNavigator(Node):
    def __init__(self) -> None:
        super().__init__("go2_simple_navigator")
        self.declare_parameter("pose", "")
        pose_str = self.get_parameter("pose").get_parameter_value().string_value
        self._goal = self._parse_pose(pose_str)
        self._exec = Nav2GoalExecutor(self)

    @staticmethod
    def _parse_pose(pose_str: str) -> tuple[float, float, float]:
        parts = pose_str.split()
        if len(parts) not in (2, 3):
            raise ValueError(f"pose must be 'x y' or 'x y yaw', got: {pose_str!r}")
        x, y = float(parts[0]), float(parts[1])
        yaw = float(parts[2]) if len(parts) == 3 else 0.0
        return x, y, yaw

    @staticmethod
    def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
        h = yaw * 0.5
        return 0.0, 0.0, math.sin(h), math.cos(h)

    def run(self) -> bool:
        x, y, yaw = self._goal
        qx, qy, qz, qw = self._yaw_to_quat(yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return self._exec.navigate(pose)


def main() -> None:
    rclpy.init()
    node = SimpleNavigator()
    try:
        ok = node.run()
    except ValueError as e:
        node.get_logger().error(str(e))
        node.get_logger().info('Usage: --ros-args -p pose:="1.0 2.0 0.5"')
        ok = False
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
