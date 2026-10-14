#!/usr/bin/env python3
"""Behavior tests for the lateral control-gap analyzer."""

from __future__ import annotations

import importlib.util
import math
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("analyze_lateral_control_gap.py")


def load_analyzer():
    spec = importlib.util.spec_from_file_location("lateral_control_gap", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class LateralControlGapCoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.analyzer = load_analyzer()

    def test_delay_estimation_recovers_known_delay_and_bias(self):
        dt = 0.01
        delay_s = 0.12
        command = []
        actual = []
        for index in range(1200):
            timestamp = index * dt
            demand = 0.08 * math.sin(1.3 * timestamp) + 0.04 * math.sin(4.1 * timestamp)
            command.append((timestamp, demand))
            delayed_time = max(0.0, timestamp - delay_s)
            measured = (
                0.08 * math.sin(1.3 * delayed_time)
                + 0.04 * math.sin(4.1 * delayed_time)
                + 0.012
            )
            actual.append((timestamp, measured))

        result = self.analyzer.estimate_actuator_delay(command, actual, max_delay_s=0.5, step_s=dt)

        self.assertAlmostEqual(result.delay_s, delay_s, delta=dt)
        self.assertLess(result.rmse_rad, 0.014)

    def test_projection_uses_left_of_path_as_positive_lateral_error(self):
        trajectory = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]

        left = self.analyzer.project_pose_to_trajectory(2.0, 1.0, 0.0, trajectory)
        right = self.analyzer.project_pose_to_trajectory(2.0, -1.0, 0.0, trajectory)

        self.assertAlmostEqual(left.lateral_error_m, 1.0, places=6)
        self.assertAlmostEqual(right.lateral_error_m, -1.0, places=6)
        self.assertAlmostEqual(left.heading_error_rad, 0.0, places=6)

    def test_projection_wraps_heading_error(self):
        trajectory = [(0.0, 0.0, -math.pi + 0.02), (-10.0, 0.0, -math.pi + 0.02)]

        result = self.analyzer.project_pose_to_trajectory(-1.0, 0.0, math.pi - 0.01, trajectory)

        self.assertAlmostEqual(result.heading_error_rad, -0.01, places=6)

    def test_pp_selector_keeps_first_sample_per_control_cycle(self):
        command_times = [10.0, 10.02, 10.04]
        debug_rows = [
            (10.001, (1.0,)),
            (10.003, (2.0,)),
            (10.021, (3.0,)),
            (10.023, (4.0,)),
            (10.041, (5.0,)),
        ]

        selected, duplicate_count = self.analyzer.select_pp_control_samples(command_times, debug_rows)

        self.assertEqual([row[1][0] for row in selected], [1.0, 3.0, 5.0])
        self.assertEqual(duplicate_count, 2)

    def test_resampling_holds_commands_and_rejects_large_feedback_gaps(self):
        command = [(0.0, 0.0), (0.1, 1.0)]
        feedback = [(0.0, 0.0), (0.1, 1.0), (1.0, 3.0)]

        held = self.analyzer.zero_order_hold(command, [0.05, 0.1, 0.15])
        interpolated = self.analyzer.linear_interpolate_with_gap_limit(
            feedback, [0.05, 0.5], max_gap_s=0.2
        )

        self.assertEqual(held, [0.0, 1.0, 1.0])
        self.assertAlmostEqual(interpolated[0], 0.5, places=6)
        self.assertTrue(math.isnan(interpolated[1]))

    def test_bag_resolver_accepts_directory_without_metadata(self):
        with tempfile.TemporaryDirectory() as temporary:
            bag_dir = Path(temporary)
            db = bag_dir / "capture_0.db3"
            db.touch()

            self.assertEqual(self.analyzer.resolve_bag_database(bag_dir), db)


if __name__ == "__main__":
    unittest.main()
