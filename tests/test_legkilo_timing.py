"""Stamped Unitree kinematics must preserve the existing common sensor clock."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_nav2'))
from go2_nav2.cloud_stamp_sync import CloudStampSync, stamp_ns


class KinematicClockTests(unittest.TestCase):
    def test_native_packet_stamp_uses_shared_translation_and_preserves_fields(self):
        stamp = SimpleNamespace(sec=100, nanosec=500)
        msg = SimpleNamespace(stamp=stamp, position=[11.,22.,33.], foot_position_body=[.2]*12)
        pub = Mock()
        clock = SimpleNamespace(max_age_ns=10**30, dropped={}, map_stamp=Mock(return_value=200_000_000_500))
        node = SimpleNamespace(_failed=False, _first_odom_stamp=1, _last_odom_mono=0,
            _sensor_clock=clock, _trace_file=None, _bag_writer=None,
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=201_000_000_000)))
        CloudStampSync._forward_sensor(node, 'kinematic', msg, pub)
        self.assertEqual(clock.map_stamp.call_args.args[:2], ('kinematic',100_000_000_500))
        self.assertEqual(stamp_ns(msg.stamp),200_000_000_500)
        self.assertEqual(msg.foot_position_body,[.2]*12)
        self.assertEqual(msg.position,[11.,22.,33.])
        pub.publish.assert_called_once_with(msg)
