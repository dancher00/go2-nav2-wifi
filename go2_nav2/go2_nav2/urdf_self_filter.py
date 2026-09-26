"""Remove articulated URDF visual envelopes from navigation input only."""
import hashlib
import signal
import time
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from go2_nav2.urdf_filter_geometry import RobotGeometry


class UrdfSelfFilter(Node):
    def __init__(self):
        super().__init__('go2_urdf_self_filter')
        self.declare_parameter('padding', 0.015)
        self.declare_parameter('sensor_frame', 'lidar3d_sensor')
        self.declare_parameter('max_shadow_distance', 1.0)
        self.model = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.joints = deque(maxlen=100)
        self.pending = deque()
        self.filtered = self.removed = self.unfiltered = 0
        self.processing_ms = []
        self.latency_ms = []
        self.pub = self.create_publisher(PointCloud2, '/lidar3d/obstacle_points', qos_profile_sensor_data)
        self.self_pub = self.create_publisher(PointCloud2, '/lidar3d/self_points', qos_profile_sensor_data)
        self.shadow_pub = self.create_publisher(PointCloud2, '/lidar3d/shadow_points', qos_profile_sensor_data)
        self.create_subscription(String, '/robot_description', self.description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.joint_state, 10)
        self.create_subscription(PointCloud2, '/lidar3d/registered', self.enqueue, qos_profile_sensor_data)
        self.create_timer(.02, self.drain)
        self.create_timer(10., self.report)

    def description(self, msg):
        try:
            model = RobotGeometry(msg.data, get_package_share_directory, self.get_parameter('padding').value)
        except Exception as error:
            self.model = None
            self.get_logger().error(f'Cannot load URDF self-filter: {error}')
            return
        self.model = model
        self.get_logger().info(f'URDF loaded: {len(model.parts)} visuals, {len(model.required_joints)} moving joints, '
                               f'padding={model.padding:.3f} m, sha256={hashlib.sha256(msg.data.encode()).hexdigest()}')

    def joint_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(np.isfinite(v) for v in values.values()):
            self.joints.append((time.monotonic(), values))

    def enqueue(self, msg):
        if len(self.pending) >= 20:
            _, old = self.pending.popleft()
            self.pub.publish(old)
            self.unfiltered += 1
        self.pending.append((time.monotonic(), msg))

    def drain(self):
        while self.pending:
            received, msg = self.pending[0]
            if not self.process(msg, received):
                if time.monotonic() - received < .3:
                    return
                # Missing/stale geometry must never erase potential obstacles.
                self.pub.publish(msg)
                self.unfiltered += 1
            self.pending.popleft()

    def process(self, msg, received):
        if self.model is None or not self.joints:
            return False
        # LowState lacks a sensor timestamp. The 3D bridge associates fresh
        # received joints with body-pose stamps for coherent TF visualization.
        # This is not hardware synchronization: bound reception skew to 0.1 s.
        joint_time, joints = min(self.joints, key=lambda item: abs(item[0] - received))
        if abs(joint_time - received) > .1 or not self.model.required_joints <= joints.keys():
            return False
        try:
            tf = self.buffer.lookup_transform('base_link', msg.header.frame_id,
                                               rclpy.time.Time.from_msg(msg.header.stamp))
            sensor_tf = self.buffer.lookup_transform('base_link', self.get_parameter('sensor_frame').value,
                                                      rclpy.time.Time.from_msg(msg.header.stamp))
        except TransformException:
            return False
        started = time.perf_counter()
        points = np.asarray([[float(v) for v in p] for p in read_points(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)], dtype=np.float64).reshape(-1, 3)
        q, p = tf.transform.rotation, tf.transform.translation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        body_points = points @ rotation.T + np.array([p.x, p.y, p.z])
        inside = self.model.contains(body_points, joints)
        sensor = sensor_tf.transform.translation
        shadow = self.model.shadows(body_points, joints, np.array([sensor.x, sensor.y, sensor.z]),
                                    self.get_parameter('max_shadow_distance').value) & ~inside
        self.pub.publish(create_cloud_xyz32(msg.header, points[~(inside | shadow)].tolist()))
        self.self_pub.publish(create_cloud_xyz32(msg.header, points[inside].tolist()))
        self.shadow_pub.publish(create_cloud_xyz32(msg.header, points[shadow].tolist()))
        self.filtered += 1
        self.removed += int(np.count_nonzero(inside | shadow))
        self.processing_ms.append((time.perf_counter() - started) * 1000)
        self.latency_ms.append((time.monotonic() - received) * 1000)
        return True

    def report(self):
        message = f'URDF self-filter: {self.removed} self points removed / {self.filtered} scans; {self.unfiltered} scans passed unfiltered (missing URDF/joints/TF)'
        if self.processing_ms:
            message += (f'; processing p95={np.quantile(self.processing_ms, .95):.1f} ms'
                        f'; reception-to-output p95={np.quantile(self.latency_ms, .95):.1f} ms')
        # Humble caches severity per logging call site; keep distinct call sites.
        if self.unfiltered:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)
        self.filtered = self.removed = self.unfiltered = 0
        self.processing_ms.clear()
        self.latency_ms.clear()


def main():
    rclpy.init()
    node = UrdfSelfFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
