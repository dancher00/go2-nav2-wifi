#!/usr/bin/env python3
"""Publish sensor_msgs/JointState from Unitree /lf/lowstate for go2_description."""

from __future__ import annotations

from typing import List

import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from unitree_go.msg import LowState

# Unitree Go2 motor index -> go2_description joint names (same order as SDK / URDF).
MOTOR_JOINTS: List[str] = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


class JointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("go2_joint_state_bridge")
        self.declare_parameter("lowstate_topic", "/lf/lowstate")
        self.declare_parameter("stamp_odom_topic", "")
        self._stamp_odom_topic = self.get_parameter("stamp_odom_topic").value
        self._latest_positions = None
        self._received_positions = 0.0
        topic = self.get_parameter("lowstate_topic").value
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._sub = self.create_subscription(LowState, topic, self._on_lowstate, 10)
        if self._stamp_odom_topic:
            self._odom_sub = self.create_subscription(Odometry, self._stamp_odom_topic,
                                                       self._on_odom, qos_profile_sensor_data)
        self.get_logger().info(f"JointState from {topic} -> /joint_states")

    def _on_lowstate(self, msg: LowState) -> None:
        positions = [float(msg.motor_state[i].q) for i in range(len(MOTOR_JOINTS))]
        if self._stamp_odom_topic:
            self._latest_positions = positions
            self._received_positions = time.monotonic()
            return
        self._publish(positions, self.get_clock().now().to_msg())

    def _on_odom(self, msg: Odometry) -> None:
        # LowState has no acquisition stamp. Associate only fresh received joint
        # measurements with this body pose; never combine laptop-clock joint TF
        # with robot-clock body TF. This is reception pairing, not hardware sync.
        if self._latest_positions is None or time.monotonic() - self._received_positions > 0.2:
            return
        self._publish(self._latest_positions, msg.header.stamp)

    def _publish(self, positions, stamp) -> None:
        js = JointState()
        js.header.stamp = stamp
        js.name = list(MOTOR_JOINTS)
        js.position = positions
        self._pub.publish(js)


def main() -> None:
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launchers can forward a second signal while cleanup is in progress.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
