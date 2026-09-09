"""Body pose composition: translation must rotate with the sensor attitude."""
from pathlib import Path
import sys
import unittest
import math
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_nav2'))
from go2_nav2.odom_tf import compose_pose


class BodyPoseTests(unittest.TestCase):
    def test_rotates_lever_arm_and_body_axes(self):
        s = math.sqrt(.5)
        p, q = compose_pose((1., 2., 3.), (0., 0., s, s), (1., 0., 0., 1., 0., 0., 0.))
        for actual, expected in zip(p, (1., 3., 3.)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(q, (s, s, 0., 0.)):
            self.assertAlmostEqual(actual, expected)

    def test_default_is_identity_for_2d(self):
        pose = ((1., 2., 3.), (.1, .2, .3, .9))
        self.assertEqual(compose_pose(*pose, (0., 0., 0., 0., 0., 0., 1.)), pose)

    def test_sensor_tf_preserves_origin_and_stamp_before_body_offset(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from go2_nav2.odom_tf import OdomTf
        from unittest.mock import Mock
        rclpy.init()
        node = OdomTf()
        try:
            node._br = Mock()
            node._sensor_frame = 'lidar3d_sensor'
            node._sensor_from_base = (1., 0., 0., 0., 0., 0., 1.)
            msg = Odometry()
            msg.header.stamp.sec = 1234
            msg.header.stamp.nanosec = 5678
            msg.pose.pose.position.x = 2.
            msg.pose.pose.position.y = 3.
            msg.pose.pose.orientation.z = math.sqrt(.5)
            msg.pose.pose.orientation.w = math.sqrt(.5)
            node._on_odom(msg)
            sensor, body = [c.args[0] for c in node._br.sendTransform.call_args_list]
            self.assertEqual(sensor.child_frame_id, 'lidar3d_sensor')
            self.assertEqual(sensor.header.stamp, msg.header.stamp)
            self.assertEqual(body.header.stamp, msg.header.stamp)
            self.assertAlmostEqual(sensor.transform.translation.y, 3.)
            self.assertAlmostEqual(body.transform.translation.y, 4.)
        finally:
            node.destroy_node()
            rclpy.shutdown()
