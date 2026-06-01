#!/usr/bin/env python3
"""Sport bridge on robot (internal DDS): /cmd_vel -> /api/sport/request.

Wi-Fi relay forwards /cmd_vel from laptop; this node must run on the robot.
Env: GO2_MAX_LINEAR, GO2_MAX_ANGULAR (optional caps; unset = no clamp),
GO2_CMD_TIMEOUT, GO2_SPORT_WALK_MODE (free|classic).
"""

from __future__ import annotations

import json
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request

API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_RECOVERY_STAND = 1006
API_MOVE = 1008
API_SWITCH_JOYSTICK = 1027
API_FREE_WALK = 2045
API_CLASSIC_WALK = 2049

TOPIC_REQUEST = "/api/sport/request"


def make_request(api_id: int, parameter: str | None = None) -> Request:
    req = Request()
    req.header.identity.id = 0
    req.header.identity.api_id = api_id
    req.header.lease.id = 0
    req.header.policy.priority = 0
    req.header.policy.noreply = True
    if parameter is not None:
        req.parameter = parameter
    return req


def clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


class SportBridge(Node):
    def __init__(self) -> None:
        super().__init__("sport_bridge")

        max_lin_env = os.environ.get("GO2_MAX_LINEAR", "").strip()
        max_ang_env = os.environ.get("GO2_MAX_ANGULAR", "").strip()
        self._max_lin = float(max_lin_env) if max_lin_env else None
        self._max_ang = float(max_ang_env) if max_ang_env else None
        cmd_timeout = float(os.environ.get("GO2_CMD_TIMEOUT", "3.0"))
        rate_hz = float(
            os.environ.get(
                "GO2_SPORT_RATE_HZ",
                os.environ.get("GO2_CMD_VEL_HZ", "20"),
            )
        )
        rate_hz = max(1.0, min(100.0, rate_hz))
        stand_on_start = os.environ.get("GO2_SPORT_STAND_ON_START", "1") != "0"
        stand_mode = os.environ.get("GO2_SPORT_STAND_MODE", "balance")
        walk_mode = os.environ.get("GO2_SPORT_WALK_MODE", "free").lower()

        self._cmd_timeout = cmd_timeout
        self._cmd_count = 0
        self._last_cmd_log = time.monotonic()

        self._pub = self.create_publisher(Request, TOPIC_REQUEST, 10)
        self._sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._stopped = True
        self._timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._on_timer)

        if stand_on_start:
            self._init_sport_mode(stand_mode, walk_mode)

        cap = "off"
        if self._max_lin is not None or self._max_ang is not None:
            cap = (
                f"lin={self._max_lin:.2f}" if self._max_lin is not None else "lin=off"
            ) + (
                f" ang={self._max_ang:.2f}" if self._max_ang is not None else " ang=off"
            )
        self.get_logger().info(
            f"Robot sport bridge: /cmd_vel -> Move @ {rate_hz:.0f} Hz (cap {cap})"
        )

    def _send(self, api_id: int, parameter: str | None = None) -> None:
        self._pub.publish(make_request(api_id, parameter))

    def _init_sport_mode(self, stand_mode: str, walk_mode: str) -> None:
        """Release RC / app joystick and enable velocity walk (required after RC use)."""
        if stand_mode == "up":
            self._send(API_STAND_UP)
            self.get_logger().info("StandUp")
        else:
            self._send(API_BALANCE_STAND)
            self.get_logger().info("BalanceStand")
        time.sleep(0.4)
        self._send(API_RECOVERY_STAND)
        time.sleep(0.3)
        self._send(API_SWITCH_JOYSTICK, json.dumps({"data": False}))
        self.get_logger().info("SwitchJoystick(false) — put RC aside, do not touch sticks")
        time.sleep(0.3)
        if walk_mode == "classic":
            self._send(API_CLASSIC_WALK, json.dumps({"data": True}))
            self.get_logger().info("ClassicWalk(true)")
        else:
            self._send(API_FREE_WALK)
            self.get_logger().info("FreeWalk")
        time.sleep(0.4)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()
        self._cmd_count += 1

    def _on_timer(self) -> None:
        now = time.monotonic()
        if now - self._last_cmd_log >= 5.0:
            vx = float(self._last_cmd.linear.x)
            vyaw = float(self._last_cmd.angular.z)
            self.get_logger().info(
                f"cmd_vel (5s): n={self._cmd_count} last_vx={vx:.2f} vyaw={vyaw:.2f}"
            )
            self._cmd_count = 0
            self._last_cmd_log = now

        age = now - self._last_cmd_time
        stale = self._last_cmd_time == 0.0 or age > self._cmd_timeout
        if stale:
            if not self._stopped:
                self._send(API_STOP_MOVE)
                self._stopped = True
            return

        vx = float(self._last_cmd.linear.x)
        vy = float(self._last_cmd.linear.y)
        vyaw = float(self._last_cmd.angular.z)
        if self._max_lin is not None:
            vx = clamp(vx, self._max_lin)
            vy = clamp(vy, self._max_lin)
        if self._max_ang is not None:
            vyaw = clamp(vyaw, self._max_ang)
        self._send(API_MOVE, json.dumps({"x": vx, "y": vy, "z": vyaw}))
        self._stopped = False


def main() -> None:
    rclpy.init()
    node = SportBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("StopMove")
        node._send(API_STOP_MOVE)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
