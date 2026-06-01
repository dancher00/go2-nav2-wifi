#!/usr/bin/env python3
"""Bridge geometry_msgs/Twist (cmd_vel) to Unitree Sport API Move."""

from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from unitree_api.msg import Request

API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_MOVE = 1008

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

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("max_linear", 0.0)
        self.declare_parameter("max_angular", 0.0)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("stand_mode", "balance")
        self.declare_parameter("stand_on_start", True)

        topic = self.get_parameter("cmd_vel_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._max_lin = float(self.get_parameter("max_linear").value)
        self._max_ang = float(self.get_parameter("max_angular").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)

        self._pub = self.create_publisher(Request, TOPIC_REQUEST, 10)
        self._sub = self.create_subscription(Twist, topic, self._on_cmd_vel, 10)
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._stopped = True
        self._timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._on_timer)

        if self.get_parameter("stand_on_start").value:
            self._stand()

        self.get_logger().info(f"Sport bridge: sub {topic}, Move @ {rate_hz} Hz")

    def _stand(self) -> None:
        mode = self.get_parameter("stand_mode").value
        if mode == "balance":
            self._send(API_BALANCE_STAND)
            self.get_logger().info("BalanceStand")
        elif mode == "up":
            self._send(API_STAND_UP)
            self.get_logger().info("StandUp")

    def _send(self, api_id: int, parameter: str | None = None) -> None:
        self._pub.publish(make_request(api_id, parameter))

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()

    def _on_timer(self) -> None:
        age = time.monotonic() - self._last_cmd_time
        stale = self._last_cmd_time == 0.0 or age > self._cmd_timeout
        if stale:
            if not self._stopped:
                self._send(API_STOP_MOVE)
                self._stopped = True
            return

        vx = float(self._last_cmd.linear.x)
        vy = float(self._last_cmd.linear.y)
        vyaw = float(self._last_cmd.angular.z)
        if self._max_lin > 0.0:
            vx = clamp(vx, self._max_lin)
            vy = clamp(vy, self._max_lin)
        if self._max_ang > 0.0:
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
