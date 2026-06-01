#!/usr/bin/env python3
"""Single publisher of map->odom for Nav2 (no duplicate with slam_toolbox).

slam_toolbox: transform_publish_period=0 (no map->odom TF).
Fresh /pose: map->odom from SLAM pose + odom->base_link.
Stale /pose: propagate last SLAM pose with odom delta (dead reckoning until scan returns).
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _rotate2d(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


class MapOdomRelay(Node):
    def __init__(self) -> None:
        super().__init__("go2_map_odom_relay")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("pose_topic", "/pose")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("smooth_alpha", 1.0)
        self.declare_parameter("max_pose_age_sec", 5.0)
        self.declare_parameter("odom_propagate_when_stale", True)

        self._map = str(self.get_parameter("map_frame").value)
        self._odom = str(self.get_parameter("odom_frame").value)
        self._base = str(self.get_parameter("base_frame").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        rate = max(float(self.get_parameter("rate_hz").value), 1.0)
        self._odom_propagate = bool(
            self.get_parameter("odom_propagate_when_stale").value
        )

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._br = TransformBroadcaster(self)
        self._last_pose: PoseWithCovarianceStamped | None = None
        self._smooth_tx = 0.0
        self._smooth_ty = 0.0
        self._smooth_yaw = 0.0
        self._have_smooth = False
        self._warned_pose = False
        self._warned_stale = False
        self._max_pose_age = float(self.get_parameter("max_pose_age_sec").value)
        self._last_pose_rx = 0.0
        # Anchor for odom propagation when /pose is stale (uses /utlidar/robot_odom TF).
        self._anchor_px = self._anchor_py = self._anchor_yaw_map = 0.0
        self._anchor_ox = self._anchor_oy = self._anchor_yaw_odom = 0.0
        self._have_anchor = False

        self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._on_pose, 10
        )
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"map->odom @ {rate:.0f} Hz; stale -> "
            f"{'odom propagate' if self._odom_propagate else 'hold'}"
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._last_pose is None:
            self._have_smooth = False
            self.get_logger().debug("SLAM /pose received")
        self._last_pose = msg
        self._last_pose_rx = time.monotonic()
        self._warned_stale = False
        self._save_anchor(msg)

    def _save_anchor(self, pose: PoseWithCovarianceStamped) -> None:
        try:
            odom_base = self._buffer.lookup_transform(
                self._odom,
                self._base,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            return
        p = pose.pose.pose.position
        q = pose.pose.pose.orientation
        self._anchor_px = p.x
        self._anchor_py = p.y
        self._anchor_yaw_map = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._anchor_ox = odom_base.transform.translation.x
        self._anchor_oy = odom_base.transform.translation.y
        bo = odom_base.transform.rotation
        self._anchor_yaw_odom = _yaw_from_quat(bo.x, bo.y, bo.z, bo.w)
        self._have_anchor = True

    def _send(self, tx: float, ty: float, yaw: float) -> None:
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map
        t.child_frame_id = self._odom
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._br.sendTransform(t)

    def _map_pose_from_odom_propagate(
        self, bx: float, by: float, yaw_odom: float
    ) -> tuple[float, float, float]:
        """Dead-reckoning in map from last SLAM anchor + odom delta."""
        dx = bx - self._anchor_ox
        dy = by - self._anchor_oy
        mx, my = _rotate2d(dx, dy, self._anchor_yaw_map)
        px = self._anchor_px + mx
        py = self._anchor_py + my
        dyaw = yaw_odom - self._anchor_yaw_odom
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        yaw_map = self._anchor_yaw_map + dyaw
        return px, py, yaw_map

    def _compute_map_odom(
        self, px: float, py: float, yaw_map: float, bx: float, by: float, yaw_odom: float
    ) -> tuple[float, float, float]:
        yaw_mo = yaw_map - yaw_odom
        c, s = math.cos(yaw_mo), math.sin(yaw_mo)
        tx = px - (bx * c - by * s)
        ty = py - (bx * s + by * c)
        return tx, ty, yaw_mo

    def _tick(self) -> None:
        if self._last_pose is None:
            if not self._warned_pose:
                self.get_logger().warn("map->odom identity until /pose")
                self._warned_pose = True
            self._send(0.0, 0.0, 0.0)
            return

        try:
            odom_base = self._buffer.lookup_transform(
                self._odom,
                self._base,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception:
            if self._have_smooth:
                self._send(self._smooth_tx, self._smooth_ty, self._smooth_yaw)
            return

        bx = odom_base.transform.translation.x
        by = odom_base.transform.translation.y
        bo = odom_base.transform.rotation
        yaw_odom = _yaw_from_quat(bo.x, bo.y, bo.z, bo.w)

        pose_age = time.monotonic() - self._last_pose_rx
        pose_stale = self._last_pose_rx > 0.0 and pose_age > self._max_pose_age

        if pose_stale:
            if not self._warned_stale:
                self.get_logger().warn(
                    f"/pose stale ({pose_age:.1f}s) — "
                    + (
                        "map->odom from odom delta (/utlidar/robot_odom)"
                        if self._odom_propagate and self._have_anchor
                        else "holding map->odom; check /scan"
                    )
                )
                self._warned_stale = True
            if self._odom_propagate and self._have_anchor:
                px, py, yaw_map = self._map_pose_from_odom_propagate(
                    bx, by, yaw_odom
                )
            else:
                if self._have_smooth:
                    self._send(self._smooth_tx, self._smooth_ty, self._smooth_yaw)
                return
        else:
            self._warned_stale = False
            p = self._last_pose.pose.pose.position
            q = self._last_pose.pose.pose.orientation
            px, py = p.x, p.y
            yaw_map = _yaw_from_quat(q.x, q.y, q.z, q.w)

        tx, ty, yaw_mo = self._compute_map_odom(
            px, py, yaw_map, bx, by, yaw_odom
        )

        alpha = float(self.get_parameter("smooth_alpha").value)
        if not self._have_smooth:
            self._smooth_tx, self._smooth_ty, self._smooth_yaw = tx, ty, yaw_mo
            self._have_smooth = True
        else:
            self._smooth_tx = alpha * tx + (1.0 - alpha) * self._smooth_tx
            self._smooth_ty = alpha * ty + (1.0 - alpha) * self._smooth_ty
            dyaw = yaw_mo - self._smooth_yaw
            dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
            self._smooth_yaw += alpha * dyaw

        self._send(self._smooth_tx, self._smooth_ty, self._smooth_yaw)


def main() -> None:
    rclpy.init()
    node = MapOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
