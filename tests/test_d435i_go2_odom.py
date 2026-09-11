"""Coordinate anchoring must preserve gravity and reject odometry resets."""
import importlib.util
import math
from pathlib import Path
import unittest

from geometry_msgs.msg import Pose

spec = importlib.util.spec_from_file_location('d435i_odom', Path(__file__).resolve().parents[1] / 'ws/scripts/d435i_go2_odom.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def pose(x=0., y=0., z=0., yaw=0.):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    p.orientation.z, p.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
    return p


class RelativePoseTests(unittest.TestCase):
    def test_anchor_and_forward_motion(self):
        transform = module.RelativePose()
        initial = transform.convert(pose(10, 20, 1, math.pi/2), 10**9)
        moved = transform.convert(pose(10, 21, 1.2, math.pi/2), 2*10**9)
        self.assertAlmostEqual(initial.position.x, 0)
        self.assertAlmostEqual(initial.orientation.w, 1)
        self.assertAlmostEqual(moved.position.x, 1)
        self.assertAlmostEqual(moved.position.y, 0)
        self.assertAlmostEqual(moved.position.z, 0.2)

    def test_gravity_tilt_preserved(self):
        transform = module.RelativePose()
        p = pose()
        p.orientation.x, p.orientation.w = math.sin(.1), math.cos(.1)
        out = transform.convert(p, 10**9)
        self.assertAlmostEqual(out.orientation.x, p.orientation.x)

    def test_coordinate_reset_rejected(self):
        transform = module.RelativePose()
        transform.convert(pose(20, 10), 10**9)
        with self.assertRaisesRegex(ValueError, 'coordinate jump'):
            transform.convert(pose(), 10**9+10**7)

    def test_invalid_pose_rejected(self):
        for invalid in [pose(float('nan')), pose()]:
            if math.isfinite(invalid.position.x):
                invalid.orientation.w = 0.0
            with self.assertRaises(ValueError):
                module.RelativePose().convert(invalid, 10**9)
