#!/usr/bin/env python3
"""Read native Go2 odometry in DDS domain 0; publish only in domain 65."""
import copy
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'go2_nav2'))
from go2_nav2.sensor_time import ClockDiscontinuity, SharedSensorClock


class RelativePose:
    """Use a gravity-aligned local origin; reject coordinate resets."""
    def __init__(self):
        self.origin = None
        self.previous = None

    def convert(self, pose, stamp):
        p, q = pose.position, pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        norm = sum(v*v for v in values[3:])
        if not all(math.isfinite(v) for v in values) or abs(norm-1.0) > 0.05:
            raise ValueError('invalid native pose')
        if self.previous is not None:
            last_p, last_stamp = self.previous
            distance = math.sqrt(sum((a-b)**2 for a, b in zip(values[:3], last_p)))
            if distance > 0.5 + 3.0 * max(0.0, (stamp-last_stamp)*1e-9):
                raise ValueError('native odometry coordinate jump; start a new session')
        self.previous = (values[:3], stamp)
        if self.origin is None:
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.origin = (values[:3], yaw)
        origin, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = p.x-origin[0], p.y-origin[1]
        result = copy.deepcopy(pose)
        result.position.x, result.position.y = c*dx+s*dy, -s*dx+c*dy
        result.position.z = p.z-origin[2]
        # q_z(-yaw) * q: preserve measured roll and pitch.
        a, b = math.cos(yaw/2), -math.sin(yaw/2)
        result.orientation.x = a*q.x-b*q.y
        result.orientation.y = a*q.y+b*q.x
        result.orientation.z = a*q.z+b*q.w
        result.orientation.w = a*q.w-b*q.z
        return result


def main():
    from rtabmap_msgs.msg import Info
    native, output = Context(), Context()
    rclpy.init(context=native, domain_id=0)
    rclpy.init(context=output, domain_id=65)
    rx = rclpy.create_node('d435i_native_odom_reader', context=native,
                           enable_rosout=False, start_parameter_services=False)
    tx = rclpy.create_node('d435i_go2_odom', context=output)
    pub = tx.create_publisher(Odometry, '/d435i/odom', 20)
    tf = TransformBroadcaster(tx)
    status_pub = tx.create_publisher(Marker, '/d435i/status', 1)
    clock = SharedSensorClock(margin_sec=0.0)
    poses = RelativePose()
    state = {'odom': 0.0, 'rgb': 0.0, 'map': 0.0, 'fault': '', 'offset_logged': False}

    def receive(msg):
        if state['fault']:
            return
        source = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
        try:
            stamp = clock.observe_odom(source, tx.get_clock().now().nanoseconds, time.monotonic_ns())
            if stamp is None:
                return
            pose = poses.convert(msg.pose.pose, source)
        except (ClockDiscontinuity, ValueError) as exc:
            state['fault'] = str(exc)
            tx.get_logger().error(state['fault'])
            return
        if not state['offset_logged']:
            tx.get_logger().info(f'Native odometry clock offset locked: {clock.offset_ns} ns')
            state['offset_logged'] = True
        out = copy.deepcopy(msg)
        out.header.frame_id = 'd435i_odom'
        out.child_frame_id = 'd435i_base'
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(stamp, 10**9)
        out.pose.pose = pose
        if not any(out.pose.covariance):
            for i in [0, 7, 14, 21, 28, 35]:
                out.pose.covariance[i] = 0.01
        pub.publish(out)
        transform = TransformStamped()
        transform.header = out.header
        transform.child_frame_id = out.child_frame_id
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        tf.sendTransform(transform)
        state['odom'] = time.monotonic()

    def camera(msg):
        state['rgb'] = time.monotonic()

    def mapping(msg):
        state['map'] = time.monotonic()

    def status():
        now = time.monotonic()
        okay = not state['fault'] and now-state['odom'] < 0.5 and now-state['rgb'] < 1.0 and now-state['map'] < 2.0
        marker = Marker()
        marker.header.frame_id = 'd435i_odom'
        marker.header.stamp = tx.get_clock().now().to_msg()
        marker.ns = 'tracking'; marker.id = 0
        # An old green marker must disappear if the Jetson reboots or DDS stops.
        marker.lifetime.sec = 1
        marker.type = Marker.TEXT_VIEW_FACING; marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0.8
        marker.scale.z = 0.12
        marker.color.a = 1.0
        marker.color.g = 1.0 if okay else 0.0
        marker.color.r = 0.1 if okay else 1.0
        marker.text = 'GO2 ODOM + RGB-D: LIVE' if okay else 'NO LIVE DATA: ' + (state['fault'] or 'waiting for odometry / camera / map')
        status_pub.publish(marker)

    sub = rx.create_subscription(Odometry, '/utlidar/robot_odom', receive, qos_profile_sensor_data)
    camera_sub = tx.create_subscription(CameraInfo, '/camera/color/camera_info', camera, qos_profile_sensor_data)
    mapping_sub = tx.create_subscription(Info, '/d435i/info', mapping, qos_profile_sensor_data)
    timer = tx.create_timer(0.25, status)
    executor = SingleThreadedExecutor(context=native)
    executor.add_node(rx)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    output_executor = SingleThreadedExecutor(context=output)
    output_executor.add_node(tx)
    try:
        output_executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        output_executor.shutdown(timeout_sec=2)
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        rx.destroy_node(); tx.destroy_node()
        for context in (native, output):
            if context.ok():
                context.shutdown()


if __name__ == '__main__':
    main()
