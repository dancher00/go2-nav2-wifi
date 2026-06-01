#!/usr/bin/env python3
"""Publish sensor_msgs/JointState from Unitree /lf/lowstate for go2_description."""

from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        topic = self.get_parameter("lowstate_topic").value
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._sub = self.create_subscription(LowState, topic, self._on_lowstate, 10)
        self.get_logger().info(f"JointState from {topic} -> /joint_states")

    def _on_lowstate(self, msg: LowState) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(MOTOR_JOINTS)
        js.position = [float(msg.motor_state[i].q) for i in range(len(MOTOR_JOINTS))]
        self._pub.publish(js)


def main() -> None:
    rclpy.init()
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
