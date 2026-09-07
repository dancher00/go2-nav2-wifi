"""Coordinate-frame regressions; no ROS node or robot is started."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "go2_nav2/go2_nav2/map_odom_relay.py"
SPEC = importlib.util.spec_from_file_location("map_odom_relay", SCRIPT)
relay_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(relay_module)
MapOdomRelay = relay_module.MapOdomRelay


def anchor(map_pose, odom_pose):
    return SimpleNamespace(
        _anchor_px=map_pose[0], _anchor_py=map_pose[1], _anchor_yaw_map=map_pose[2],
        _anchor_ox=odom_pose[0], _anchor_oy=odom_pose[1], _anchor_yaw_odom=odom_pose[2],
    )


class OdomPropagationTests(unittest.TestCase):
    def assert_pose(self, actual, expected):
        self.assertAlmostEqual(actual[0], expected[0])
        self.assertAlmostEqual(actual[1], expected[1])
        yaw_error = actual[2] - expected[2]
        self.assertAlmostEqual(math.atan2(math.sin(yaw_error), math.cos(yaw_error)), 0.0)

    def test_aligned_frames_do_not_rotate_displacement_by_robot_heading(self):
        state = anchor((10.0, 20.0, math.pi / 2), (2.0, 3.0, math.pi / 2))
        pose = MapOdomRelay._map_pose_from_odom_propagate(state, 2.0, 4.0, math.pi / 2)
        self.assert_pose(pose, (10.0, 21.0, math.pi / 2))

    def test_rotated_frames_apply_map_to_odom_heading_difference(self):
        state = anchor((10.0, 20.0, math.pi), (2.0, 3.0, math.pi / 2))
        pose = MapOdomRelay._map_pose_from_odom_propagate(state, 2.0, 5.0, math.pi / 2)
        self.assert_pose(pose, (8.0, 20.0, math.pi))

    def test_turn_across_pi_keeps_position_and_wraps_heading(self):
        state = anchor((10.0, 20.0, math.radians(30)), (2.0, 3.0, math.radians(179)))
        pose = MapOdomRelay._map_pose_from_odom_propagate(state, 2.0, 3.0, math.radians(-179))
        self.assert_pose(pose, (10.0, 20.0, math.radians(32)))

    def test_odometry_motion_preserves_map_to_odom_transform(self):
        map_pose = (4.0, -3.0, 1.3)
        odom_pose = (-2.0, 5.0, -0.7)
        state = anchor(map_pose, odom_pose)
        expected = MapOdomRelay._compute_map_odom(state, *map_pose, *odom_pose)
        for current in ((0.0, 7.0, 0.4), (-3.0, 2.0, -2.8), (1.0, -1.0, 3.0)):
            with self.subTest(current=current):
                propagated = MapOdomRelay._map_pose_from_odom_propagate(state, *current)
                actual = MapOdomRelay._compute_map_odom(state, *propagated, *current)
                self.assert_pose(actual, expected)


if __name__ == "__main__":
    unittest.main()
