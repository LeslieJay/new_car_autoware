#!/usr/bin/env python3
"""Unit tests for the read-only steering diagnosis helpers."""

import importlib.util
import math
import sys
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_avoidance_steering.py"


def load_analyzer():
    spec = importlib.util.spec_from_file_location("avoidance_steering", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Point:
    def __init__(self, x, y):
        self.pose = type("Pose", (), {})()
        self.pose.position = type("Position", (), {"x": x, "y": y})()


class AvoidanceSteeringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.analyzer = load_analyzer()

    def test_curvature_to_steering_uses_vehicle_wheelbase(self):
        angle = self.analyzer.curvature_to_steering_angle(1.0, 1.008)
        self.assertAlmostEqual(angle, math.atan(1.008), places=12)
        self.assertAlmostEqual(
            math.degrees(self.analyzer.curvature_to_steering_angle(-0.5, 1.008)),
            -math.degrees(math.atan(0.504)),
            places=12,
        )

    def test_maximum_path_curvature_returns_signed_peak(self):
        radius = 2.0
        points = [
            Point(radius * math.sin(angle), radius * (1.0 - math.cos(angle)))
            for angle in (-0.2, -0.1, 0.0, 0.1, 0.2)
        ]
        steering, curvature, index = self.analyzer.maximum_path_curvature(points, 1.008)
        self.assertIsNotNone(steering)
        self.assertIsNotNone(curvature)
        self.assertIsNotNone(index)
        self.assertAlmostEqual(abs(curvature), 0.5, places=3)
        self.assertAlmostEqual(abs(steering), math.atan(1.008 * 0.5), places=3)

    def test_default_windows_include_both_incidents(self):
        names = [window[0] for window in self.analyzer.DEFAULT_WINDOWS]
        self.assertIn("11:16:48-11:17:02", names)
        self.assertIn("11:12:02-11:12:04", names)


if __name__ == "__main__":
    unittest.main()
