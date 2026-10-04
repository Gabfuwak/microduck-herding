"""Run with python3 -m unittest discover -s Tools -p 'test_*.py'."""

import math
import unittest
import xml.etree.ElementTree as ET

from convert_microduck import origin


class MeshOrientationTests(unittest.TestCase):
    def test_singular_pitch_preserves_mesh_axes(self):
        # Both rotations occur on real MicroDuck visual meshes. Independent roll
        # and yaw extraction at gimbal lock previously reversed the X/Z axes.
        cases = (
            ((0, 0.707107, 0, 0.707107), ((0, 0, 1), (0, -1, 0), (1, 0, 0))),
            ((0.707107, 0, -0.707107, 0), ((0, 0, 1), (0, 1, 0), (-1, 0, 0))),
        )
        for quaternion, expected_axes in cases:
            with self.subTest(quaternion=quaternion):
                element = ET.fromstring(origin((0, 0, 0), quaternion))
                roll, pitch, yaw = map(float, element.attrib["rpy"].split())
                for axis, expected in zip(((1, 0, 0), (0, 1, 0), (0, 0, 1)), expected_axes):
                    x, y, z = axis
                    y, z = y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
                    x, z = x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)
                    x, y = x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)
                    for actual, wanted in zip((x, y, z), expected):
                        self.assertAlmostEqual(actual, wanted, places=10)


if __name__ == "__main__":
    unittest.main()
