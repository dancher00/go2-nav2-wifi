"""The shared Go2 envelope must contain independently measured visual geometry."""
import json
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]

class Go2FootprintTests(unittest.TestCase):
    def test_shared_polygon_contains_measured_body_and_legs(self):
        footprint = yaml.safe_load((ROOT/'go2_nav2/config/go2_footprint.yaml').read_text())
        polygon = json.loads(footprint['footprint'])
        measured = json.loads((ROOT/'docs/measurements/go2-standing-footprint-2026-09-11.json').read_text())
        for point in measured['hull_xy']:
            cross = []
            for a, b in zip(polygon, polygon[1:] + polygon[:1]):
                cross.append((b[0]-a[0])*(point[1]-a[1])-(b[1]-a[1])*(point[0]-a[0]))
            self.assertTrue(all(v <= 0 for v in cross) or all(v >= 0 for v in cross), point)
        self.assertEqual(len(polygon), 4)
        self.assertLess(footprint['footprint_padding'], .1)
