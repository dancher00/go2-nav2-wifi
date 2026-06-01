#!/usr/bin/env python3
"""Loop through map-frame waypoints using Nav2 planner + controller."""

from __future__ import annotations

import math
from pathlib import Path
from typing import List

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from go2_nav2.nav2_goal_exec import Nav2GoalExecutor


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    h = yaw * 0.5
    return 0.0, 0.0, math.sin(h), math.cos(h)


def _load_waypoints(path: str) -> tuple[str, bool, float, List[dict]]:
    p = Path(path).expanduser()
    if not p.is_file():
        raise FileNotFoundError(f"Waypoints file not found: {path}")
    with p.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    frame = str(data.get("frame_id", "map"))
    loop = bool(data.get("loop", True))
    pause = float(data.get("pause_sec", 2.0))
    wps = data.get("waypoints") or []
    if not wps:
        raise ValueError(f"No waypoints in {path}")
    return frame, loop, pause, wps


def _pose_from_wp(node: Node, frame: str, wp: dict) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(wp["x"])
    pose.pose.position.y = float(wp["y"])
    yaw = float(wp.get("yaw", 0.0))
    qx, qy, qz, qw = _yaw_to_quat(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__("go2_patrol")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("start_index", 0)
        self.declare_parameter("retry_sec", 5.0)

        wp_file = self.get_parameter("waypoints_file").get_parameter_value().string_value
        if not wp_file:
            raise RuntimeError("Set waypoints_file parameter")

        self._frame, self._loop, self._pause_sec, self._waypoints = _load_waypoints(wp_file)
        self._index = int(self.get_parameter("start_index").value) % len(self._waypoints)
        self._retry_sec = float(self.get_parameter("retry_sec").value)
        self._cb = ReentrantCallbackGroup()
        self._exec = Nav2GoalExecutor(self, self._frame, callback_group=self._cb)
        self._wait_timer = self.create_timer(1.0, self._wait_for_nav2, callback_group=self._cb)
        self._pause_timer = None
        self.get_logger().info(
            f"Patrol: {len(self._waypoints)} waypoints, loop={self._loop}, "
            f"pause={self._pause_sec}s, file={wp_file}"
        )

    def _wait_for_nav2(self) -> None:
        if not self._exec.nav2_ready:
            self.get_logger().info("Waiting for Nav2 action servers...", throttle_duration_sec=5.0)
            return
        self._wait_timer.cancel()
        self.get_logger().info("Nav2 ready — starting patrol")
        self._go_to_current()

    def _go_to_current(self) -> None:
        if self._exec.busy:
            return
        wp = self._waypoints[self._index]
        pose = _pose_from_wp(self, self._frame, wp)
        name = wp.get("name", f"wp{self._index + 1}")
        p = pose.pose.position
        self.get_logger().info(
            f"Leg {self._index + 1}/{len(self._waypoints)} [{name}] "
            f"→ ({p.x:.2f}, {p.y:.2f})"
        )
        self._exec.start(pose, self._on_leg_done)

    def _on_leg_done(self, ok: bool) -> None:
        if not ok:
            self.get_logger().warn(
                f"Leg {self._index + 1} failed — retry in {self._retry_sec}s "
                "(check Pose Estimate / goal on map)"
            )
            self._pause_timer = self.create_timer(
                self._retry_sec, self._retry_same, callback_group=self._cb
            )
            return
        self._index += 1
        if self._index >= len(self._waypoints):
            if self._loop:
                self._index = 0
                self.get_logger().info("Patrol lap complete — next lap")
            else:
                self.get_logger().info("Patrol finished (loop=false)")
                return
        self._pause_timer = self.create_timer(
            self._pause_sec, self._after_pause, callback_group=self._cb
        )

    def _after_pause(self) -> None:
        if self._pause_timer:
            self._pause_timer.cancel()
            self._pause_timer = None
        self._go_to_current()

    def _retry_same(self) -> None:
        if self._pause_timer:
            self._pause_timer.cancel()
            self._pause_timer = None
        self._go_to_current()


def main() -> None:
    rclpy.init()
    node = PatrolNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._exec.cancel_active()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
