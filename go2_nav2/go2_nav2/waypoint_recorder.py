#!/usr/bin/env python3
"""Append RViz 2D Goal Pose clicks to a patrol waypoints YAML file."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def _quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class WaypointRecorder(Node):
    def __init__(self) -> None:
        super().__init__("go2_waypoint_recorder")
        self.declare_parameter("output", "/ws/maps/patrol.yaml")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("loop", True)
        self.declare_parameter("pause_sec", 2.0)

        out = self.get_parameter("output").get_parameter_value().string_value
        self._path = Path(out).expanduser()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._frame = self.get_parameter("frame_id").value
        self._loop = bool(self.get_parameter("loop").value)
        self._pause = float(self.get_parameter("pause_sec").value)

        if self._path.is_file():
            with self._path.open(encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            self._waypoints = list(data.get("waypoints") or [])
        else:
            self._waypoints = []

        topic = self.get_parameter("goal_topic").value
        self.create_subscription(PoseStamped, topic, self._on_goal, 10)
        self.get_logger().info(
            f"Recording waypoints → {self._path} ({len(self._waypoints)} existing). "
            f"Use RViz 2D Goal Pose on {topic}"
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        yaw = _quat_to_yaw(msg.pose.orientation)
        idx = len(self._waypoints) + 1
        wp = {
            "name": f"wp{idx}",
            "x": round(float(p.x), 3),
            "y": round(float(p.y), 3),
            "yaw": round(float(yaw), 3),
        }
        self._waypoints.append(wp)
        self._save()
        self.get_logger().info(
            f"Saved {wp['name']}: x={wp['x']} y={wp['y']} yaw={wp['yaw']} "
            f"(total {len(self._waypoints)})"
        )

    def _save(self) -> None:
        doc = {
            "frame_id": self._frame,
            "loop": self._loop,
            "pause_sec": self._pause,
            "waypoints": self._waypoints,
        }
        with self._path.open("w", encoding="utf-8") as f:
            yaml.dump(doc, f, default_flow_style=False, sort_keys=False, allow_unicode=True)


def main() -> None:
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
