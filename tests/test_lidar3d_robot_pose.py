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
