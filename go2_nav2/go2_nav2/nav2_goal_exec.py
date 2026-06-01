"""Planner + controller actions — async callbacks only (no spin_until_future_complete)."""

from __future__ import annotations

import math
import time
from typing import Callable, Optional

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

DoneCallback = Callable[[bool], None]


class Nav2GoalExecutor:
    def __init__(
        self,
        node: Node,
        map_frame: str = "map",
        callback_group: CallbackGroup | None = None,
    ) -> None:
        self._node = node
        self._map_frame = map_frame
        self._planner = ActionClient(
            node,
            ComputePathToPose,
            "compute_path_to_pose",
            callback_group=callback_group,
        )
        self._controller = ActionClient(
            node,
            FollowPath,
            "follow_path",
            callback_group=callback_group,
        )
        self._cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        self._plan_pub = node.create_publisher(Path, "/nav_plan", 10)
        self._cb_group = callback_group
        self._busy = False
        self._done_cb: DoneCallback | None = None
        self._follow_handle: ClientGoalHandle | None = None
        self._leg_t0 = 0.0
        self._leg_path_len = 0

    @property
    def nav2_ready(self) -> bool:
        return self._planner.server_is_ready() and self._controller.server_is_ready()

    @property
    def busy(self) -> bool:
        return self._busy

    def cancel_active(self) -> None:
        handle = self._follow_handle
        self._follow_handle = None
        if handle is not None:
            self._node.get_logger().info("Cancelling previous follow_path")
            handle.cancel_goal_async()
        self._publish_stop()

    def _publish_stop(self) -> None:
        if self._node.context.ok():
            self._cmd_pub.publish(Twist())

    def start(self, pose: PoseStamped, done_cb: DoneCallback) -> None:
        """Non-blocking: returns immediately; done_cb(ok) when leg finishes."""
        if self._busy:
            self._node.get_logger().warn("start() while busy")
            done_cb(False)
            return
        if not self._planner.server_is_ready() or not self._controller.server_is_ready():
            self._node.get_logger().error("Nav2 action servers not ready")
            done_cb(False)
            return

        self._busy = True
        self._done_cb = done_cb
        self._pose = pose
        self.cancel_active()
        self._send_plan(self._pose)

    def _send_plan(self, pose: PoseStamped) -> None:
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        if not goal.goal.header.frame_id:
            goal.goal.header.frame_id = self._map_frame
        goal.goal.header.stamp = self._node.get_clock().now().to_msg()
        goal.use_start = False
        goal.planner_id = "GridBased"

        p = pose.pose.position
        self._node.get_logger().info(
            f"Planning to ({p.x:.2f}, {p.y:.2f}) frame={goal.goal.header.frame_id}"
        )
        send_future = self._planner.send_goal_async(goal)
        send_future.add_done_callback(self._on_plan_sent)

    def _on_plan_sent(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(False, f"planner send failed: {exc}")
            return
        if not handle.accepted:
            self._finish(False, "planner rejected goal")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        try:
            path = future.result().result.path
        except Exception as exc:  # noqa: BLE001
            self._finish(False, f"planner result failed: {exc}")
            return
        if not path.poses:
            self._finish(False, "Empty path — goal in obstacle or bad TF?")
            return

        start = path.poses[0].pose.position
        end = path.poses[-1].pose.position
        dist = math.hypot(end.x - start.x, end.y - start.y)
        self._leg_path_len = len(path.poses)
        self._node.get_logger().info(
            f"Planned {len(path.poses)} poses, path length ~{dist:.2f} m"
        )
        if dist < 0.35:
            self._node.get_logger().warn(
                f"Path short ({dist:.2f} m) — pick a farther goal on the map"
            )
        self._publish_plan(path)
        self._send_follow(path)

    def _publish_plan(self, path: Path) -> None:
        out = Path()
        out.header.frame_id = path.header.frame_id or self._map_frame
        out.header.stamp = self._node.get_clock().now().to_msg()
        out.poses = path.poses
        self._plan_pub.publish(out)

    def _send_follow(self, path: Path) -> None:
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        self._leg_t0 = time.monotonic()
        send_future = self._controller.send_goal_async(goal)
        send_future.add_done_callback(self._on_follow_sent)

    def _on_follow_sent(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(False, f"controller send failed: {exc}")
            return
        if not handle.accepted:
            self._finish(False, "Controller rejected goal")
            return
        self._follow_handle = handle
        self._node.get_logger().info("follow_path accepted — robot should move")
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_follow_result)

    def _on_follow_result(self, future) -> None:
        self._follow_handle = None
        dt = time.monotonic() - self._leg_t0
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001
            self._finish(False, f"controller result failed: {exc}")
            return
        if status == 4:
            if dt < 2.0 and self._leg_path_len > 8:
                self._finish(
                    True,
                    f"Navigation finished OK ({dt:.1f}s) — check robot moved",
                )
            else:
                self._finish(True, f"Navigation finished OK ({dt:.1f}s)")
            return
        if status == 5:
            self._finish(False, f"follow_path canceled ({dt:.1f}s)")
            return
        self._finish(False, f"Navigation ended status={status} ({dt:.1f}s)")

    def _finish(self, ok: bool, msg: str) -> None:
        if ok:
            self._node.get_logger().info(msg)
        elif "canceled" in msg:
            self._node.get_logger().info(msg)
        else:
            self._node.get_logger().warn(msg)
        self._busy = False
        cb = self._done_cb
        self._done_cb = None
        if not ok:
            self._publish_stop()
        if cb is not None:
            cb(ok)

    def navigate(self, pose: PoseStamped) -> bool:
        """Blocking API for go2_simple_navigator CLI."""
        import threading

        done = threading.Event()
        result: list[bool] = [False]

        def _cb(ok: bool) -> None:
            result[0] = ok
            done.set()

        self.start(pose, _cb)
        done.wait(timeout=600.0)
        return result[0]
