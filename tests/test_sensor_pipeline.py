"""ROS message-level timing tests; no node, DDS graph or robot is started."""

from pathlib import Path
import sys
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_nav2'))
from go2_nav2 import cloud_stamp_sync, odom_tf, map_odom_relay
from go2_nav2.sensor_time import SharedSensorClock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

SECOND = 10**9
EPOCH = 10000 * SECOND
SOURCE = 9000 * SECOND


def stamp(msg, ns):
    cloud_stamp_sync.set_stamp(msg.header.stamp, ns)
    return msg


def odom_node():
    node = SimpleNamespace(_alpha=1.0, _got_odom=False, _stamp=None, _last_stamp_ns=0,
                           _odom_frame='odom', _base_frame='base_link',
                           _br=Mock(), _odom_pub=Mock(),
                           get_clock=Mock(side_effect=AssertionError('arrival time must not be read')))
    node._publish_tf = MethodType(odom_tf.OdomTf._publish_tf, node)
    return node


class OdomTimestampTests(unittest.TestCase):
    def test_tf_and_odom_keep_the_incoming_acquisition_stamp(self):
        node = odom_node()
        msg = stamp(Odometry(), EPOCH + 123456789)
        msg.pose.pose.position.x = 2.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = .25
        odom_tf.OdomTf._on_odom(node, msg)
        tf = node._br.sendTransform.call_args.args[0]
        out = node._odom_pub.publish.call_args.args[0]
        self.assertEqual(tf.header.stamp, msg.header.stamp)
        self.assertEqual(out.header.stamp, msg.header.stamp)
        self.assertEqual(out.pose.covariance[0], .25)
        self.assertEqual(tf.transform.translation.x, 2.0)

    def test_no_fake_identity_transform_before_first_measurement(self):
        node = odom_node()
        node._publish_tf()
        node._br.sendTransform.assert_not_called()

    def test_repeated_or_out_of_order_odom_does_not_refresh_tf(self):
        node = odom_node()
        for ns in (EPOCH, EPOCH, EPOCH-1):
            odom_tf.OdomTf._on_odom(node, stamp(Odometry(), ns))
        self.assertEqual(node._br.sendTransform.call_count, 1)


class PairedStreamTests(unittest.TestCase):
    def make_node(self):
        clock = SharedSensorClock(calibration_sec=.1)
        for i in range(11):
            t = i*10000000
            clock.observe_odom(SOURCE+t, EPOCH+t, t)
        node = SimpleNamespace(_sensor_clock=clock, _failed=False, _reported_offset=False,
                               _first_odom_stamp=None, _last_odom_mono=None,
                               _pub=Mock(), _imu_pub=Mock(), _odom_pub=Mock(), _offset_pub=Mock(),
                               get_logger=Mock(return_value=Mock()), get_clock=Mock(return_value=Mock()))
        node._clock_fault = MethodType(cloud_stamp_sync.CloudStampSync._clock_fault, node)
        node._forward_sensor = MethodType(cloud_stamp_sync.CloudStampSync._forward_sensor, node)
        return node

    def deliver(self, node, method, msg, mono):
        node.get_clock().now.return_value.nanoseconds = EPOCH+mono
        with patch.object(cloud_stamp_sync.time, 'monotonic_ns', return_value=mono):
            method(node, msg)

    def test_cloud_and_odom_share_translation_not_arrival_time(self):
        node = self.make_node()
        odom = stamp(Odometry(), SOURCE+SECOND)
        cloud = stamp(PointCloud2(), SOURCE+SECOND)
        cloud.header.frame_id = 'odom'
        cloud.data = [1,2,3,4]
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_odom, odom, SECOND)
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_cloud, cloud, SECOND+180000000)
        self.assertEqual(odom.header.stamp, cloud.header.stamp)
        self.assertEqual(cloud.header.frame_id, 'odom')
        self.assertEqual(bytes(cloud.data), bytes([1,2,3,4]))
        node._pub.publish.assert_called_once_with(cloud)
        self.assertEqual(node._offset_pub.publish.call_args.args[0].data, node._sensor_clock.offset_ns)

    def test_cloud_stops_when_odometry_stops(self):
        node = self.make_node()
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_odom,
                     stamp(Odometry(), SOURCE+SECOND), SECOND)
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_cloud,
                     stamp(PointCloud2(), SOURCE+2*SECOND), 2*SECOND)
        node._pub.publish.assert_not_called()

    def test_imu_uses_cloud_clock_without_changing_measurement_or_relative_time(self):
        node = self.make_node()
        odom = stamp(Odometry(), SOURCE+SECOND)
        imu = stamp(Imu(), SOURCE+SECOND+4000000)
        imu.header.frame_id = 'utlidar_imu'
        imu.linear_acceleration.z = 9.82
        imu.angular_velocity.x = .012
        imu.linear_acceleration_covariance[0] = .3
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_odom, odom, SECOND)
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_imu, imu, SECOND+180000000)
        self.assertEqual(cloud_stamp_sync.stamp_ns(imu.header.stamp)-cloud_stamp_sync.stamp_ns(odom.header.stamp), 4000000)
        self.assertEqual(imu.header.frame_id, 'utlidar_imu')
        self.assertEqual(imu.linear_acceleration.z, 9.82)
        self.assertEqual(imu.angular_velocity.x, .012)
        self.assertEqual(imu.linear_acceleration_covariance[0], .3)
        node._imu_pub.publish.assert_called_once_with(imu)

    def test_imu_obeys_same_stale_reference_and_clock_fault_as_cloud(self):
        node = self.make_node()
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_odom, stamp(Odometry(), SOURCE+SECOND), SECOND)
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_imu, stamp(Imu(), SOURCE+2*SECOND), 2*SECOND)
        node._imu_pub.publish.assert_not_called()
        node._failed = True
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_imu, stamp(Imu(), SOURCE+SECOND+1), SECOND+1)
        node._imu_pub.publish.assert_not_called()
        self.assertEqual(node._sensor_clock.dropped['no_fresh_odom'], 1)

    def test_clock_fault_does_not_resume_with_fresh_looking_messages(self):
        node = self.make_node()
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_odom,
                     stamp(Odometry(), SOURCE-SECOND), SECOND)
        self.assertTrue(node._failed)
        self.deliver(node, cloud_stamp_sync.CloudStampSync._on_odom,
                     stamp(Odometry(), SOURCE+2*SECOND), 2*SECOND)
        node._odom_pub.publish.assert_not_called()


class LocalizationTimestampTests(unittest.TestCase):
    def test_slam_anchor_uses_pose_timestamp_not_latest_tf(self):
        pose = stamp(PoseWithCovarianceStamped(), EPOCH)
        pose.pose.pose.position.x = 10.0
        pose.pose.pose.orientation.w = 1.0
        transform = TransformStamped()
        transform.transform.translation.x = 2.0
        transform.transform.rotation.w = 1.0
        node = SimpleNamespace(_buffer=Mock(), _odom='odom', _base='base_link')
        node._buffer.lookup_transform.return_value = transform
        self.assertTrue(map_odom_relay.MapOdomRelay._save_anchor(node, pose))
        self.assertEqual(node._buffer.lookup_transform.call_args.args[2].nanoseconds, EPOCH)
        self.assertEqual(node._anchor_px-node._anchor_ox, 8.0)

    def test_unavailable_historical_tf_does_not_fall_back_to_latest(self):
        node = SimpleNamespace(_buffer=Mock(), _odom='odom', _base='base_link', _have_anchor=False)
        node._buffer.lookup_transform.side_effect = RuntimeError('not yet in buffer')
        self.assertFalse(map_odom_relay.MapOdomRelay._save_anchor(node, stamp(PoseWithCovarianceStamped(), EPOCH)))
        self.assertFalse(node._have_anchor)
        self.assertEqual(node._buffer.lookup_transform.call_count, 1)

    def test_transform_holds_between_pose_updates_instead_of_chasing_current_odom(self):
        node = SimpleNamespace(_pending_pose=None, _have_anchor=True, _last_pose_rx=1.,
                               _max_pose_age=5., _warned_stale=False, _have_smooth=False,
                               _anchor_px=10., _anchor_py=0., _anchor_yaw_map=0.,
                               _anchor_ox=2., _anchor_oy=0., _anchor_yaw_odom=0.,
                               get_parameter=Mock(return_value=SimpleNamespace(value=1.)),
                               get_logger=Mock(return_value=Mock()), _send=Mock(), _buffer=Mock())
        node._compute_map_odom = MethodType(map_odom_relay.MapOdomRelay._compute_map_odom,node)
        for now in (2.,4.,8.):
            with patch.object(map_odom_relay.time,'monotonic',return_value=now):
                map_odom_relay.MapOdomRelay._tick(node)
            node._send.assert_called_with(8.,0.,0.)
        node._buffer.lookup_transform.assert_not_called()


if __name__ == '__main__':
    unittest.main()
