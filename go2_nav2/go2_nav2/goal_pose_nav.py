#!/usr/bin/env python3
"""RViz 2D Goal Pose (/goal_pose) -> planner + controller (nav2_slam_loc)."""

from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from go2_nav2.nav2_goal_exec import Nav2GoalExecutor
from go2_nav2.pre_rotate import maybe_rotate_toward_goal


class GoalPoseNav(Node):
    def __init__(self) -> None:
        super().__init__("go2_goal_pose_nav")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("pre_rotate_enable", True)
        self.declare_parameter("pre_rotate_threshold_rad", 1.0)
        topic = self.get_parameter("goal_topic").value
        map_frame = self.get_parameter("map_frame").value
        self._map_frame = map_frame
        self._pre_rotate = bool(self.get_parameter("pre_rotate_enable").value)
        self._pre_rotate_thresh = float(
            self.get_parameter("pre_rotate_threshold_rad").value
        )
        self._cb = ReentrantCallbackGroup()
        self._exec = Nav2GoalExecutor(self, map_frame, callback_group=self._cb)
        self._lock = threading.Lock()
        self._pending: PoseStamped | None = None
        self._waiting_nav2: PoseStamped | None = None
        self._leg_done_timer = None
        self._ready_timer = self.create_timer(
            1.0, self._try_start_waiting, callback_group=self._cb
        )
        self._sub = self.create_subscription(
            PoseStamped,
            topic,
            self._on_goal,
            10,
            callback_group=self._cb,
        )

    def _on_goal(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.get_logger().info(f"Goal received ({p.x:.2f}, {p.y:.2f})")
        if not self._exec.nav2_ready:
            with self._lock:
                self._waiting_nav2 = msg
            self.get_logger().warn("goal queued: nav2 not ready")
            return
        with self._lock:
            if self._exec.busy:
                self._pending = msg
                self.get_logger().warn("goal queued: leg in progress")
                return
        self._begin(msg)

    def _try_start_waiting(self) -> None:
        if not self._exec.nav2_ready:
            return
        with self._lock:
            msg = self._waiting_nav2
            self._waiting_nav2 = None
            if msg is None or self._exec.busy:
                return
        self._begin(msg)

    def _begin(self, msg: PoseStamped) -> None:
        if self._pre_rotate:
            maybe_rotate_toward_goal(
                self,
                msg,
                self._map_frame,
                start_threshold_rad=self._pre_rotate_thresh,
            )
        self._exec.start(msg, self._on_leg_done)

    def _on_leg_done(self, ok: bool) -> None:
        with self._lock:
            nxt = self._pending
            self._pending = None
        if ok:
            pass
        elif nxt is None:
            self.get_logger().warn("leg failed")
        if nxt is not None:
            if self._leg_done_timer is not None:
                self._leg_done_timer.cancel()
            self._leg_done_timer = self.create_timer(
                2.0,
                lambda: self._start_queued(nxt),
                callback_group=self._cb,
            )

    def _start_queued(self, msg: PoseStamped) -> None:
        if self._leg_done_timer is not None:
            self._leg_done_timer.cancel()
            self._leg_done_timer = None
        if self._exec.busy:
            with self._lock:
                self._pending = msg
            return
        self._begin(msg)


def main() -> None:
    rclpy.init()
    node = GoalPoseNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._exec.cancel_active()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
