#!/usr/bin/env python3
"""Offline regression for the articulated borrowing acceptance gate."""
import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "opposite_borrowing_metrics.py"
spec = importlib.util.spec_from_file_location("borrowing_metrics", SCRIPT)
metrics_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics_module)


class BorrowingMetricsTest(unittest.TestCase):
    def test_missing_samples_never_pass(self):
        errors = metrics_module.failures(metrics_module.new_metrics(), require_complete=True)
        self.assertIn("BORROWING_DIAGNOSTICS_MISSING", errors)
        self.assertIn("ARTICULATED_SAMPLES_MISSING", errors)
        self.assertIn("ARTICULATED_RETURN_NOT_COMPLETED", errors)

    def test_success_requires_last_body_return_and_complete_phase(self):
        metrics = metrics_module.new_metrics()
        metrics_module.observe(metrics,
            "[OPPOSITE_BORROWING] phase=returning actual_min_clearance_m=0.7 "
            "actual_outside_corridor=0 actual_returned=1 planning_ms=12.0")
        self.assertIn("ARTICULATED_RETURN_NOT_COMPLETED", metrics_module.failures(metrics, True))
        metrics_module.observe(metrics,
            "[OPPOSITE_BORROWING] phase=complete actual_min_clearance_m=0.9 "
            "actual_outside_corridor=0 actual_returned=1 planning_ms=8.0")
        self.assertEqual(metrics_module.failures(metrics, True), [])
        self.assertEqual(metrics["borrowing_min_body_clearance_m"], 0.7)
        self.assertEqual(metrics["borrowing_max_planning_ms"], 12.0)

    def test_collision_or_boundary_violation_is_not_hidden_by_later_success(self):
        metrics = metrics_module.new_metrics()
        metrics_module.observe(metrics,
            "[OPPOSITE_BORROWING] phase=stopping actual_min_clearance_m=0 "
            "actual_outside_corridor=1 actual_returned=0 planning_ms=20.0")
        metrics_module.observe(metrics,
            "[OPPOSITE_BORROWING] phase=complete actual_min_clearance_m=1 "
            "actual_outside_corridor=0 actual_returned=1 planning_ms=1.0")
        errors = metrics_module.failures(metrics, True)
        self.assertIn("ARTICULATED_BODY_COLLISION", errors)
        self.assertIn("ARTICULATED_BOUNDARY_VIOLATION", errors)

    def test_waiting_is_not_executed_footprint_evidence(self):
        metrics = metrics_module.new_metrics()
        metrics_module.observe(metrics,
            "[OPPOSITE_BORROWING] phase=waiting_approval actual_min_clearance_m=nan "
            "actual_outside_corridor=-1 actual_returned=-1 planning_ms=0.5")
        self.assertEqual(metrics_module.failures(metrics, False), [])
        self.assertIsNone(metrics["borrowing_min_body_clearance_m"])
        self.assertIn("ARTICULATED_SAMPLES_MISSING", metrics_module.failures(metrics, True))

    def test_other_module_or_lock_messages_are_ignored(self):
        metrics = metrics_module.new_metrics()
        metrics_module.observe(metrics, "[SIMPLE_LC_AVOIDANCE] phase=complete")
        metrics_module.observe(metrics, "[OPPOSITE_BORROWING] active target locked uuid=abc")
        self.assertEqual(metrics["borrowing_diagnostic_samples"], 0)


if __name__ == "__main__":
    unittest.main()
