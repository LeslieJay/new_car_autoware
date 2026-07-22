#!/usr/bin/env python3

import importlib.util
import io
from pathlib import Path
from contextlib import redirect_stderr
import unittest


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "check_autonomous_transition.py"
SPEC = importlib.util.spec_from_file_location("check_autonomous_transition", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class DiagnoseTest(unittest.TestCase):
    def test_recorded_failure_ranks_control_mode_before_zero_command(self):
        snapshot = MODULE.Snapshot(
            operation_mode=MODULE.OPERATION_MODE_AUTONOMOUS,
            autoware_control_enabled=False,
            vehicle_control_mode=4,
            trajectory_speed=2.0,
            follower_speed=0.0,
            gated_speed=0.0,
            service_available={
                "/system/operation_mode/change_operation_mode": True,
                "/system/operation_mode/change_autoware_control": True,
                "/control/control_mode_request": False,
            },
        )

        reasons = MODULE.diagnose(snapshot)

        self.assertIn("/control/control_mode_request is unavailable", reasons[0])
        self.assertLess(
            next(i for i, reason in enumerate(reasons) if "vehicle control mode" in reason),
            next(i for i, reason in enumerate(reasons) if "follower command" in reason),
        )

    def test_healthy_autonomous_state_has_no_failure_reason(self):
        snapshot = MODULE.Snapshot(
            operation_mode=MODULE.OPERATION_MODE_AUTONOMOUS,
            autoware_control_enabled=True,
            vehicle_control_mode=MODULE.VEHICLE_MODE_AUTONOMOUS,
            trajectory_speed=2.0,
            follower_speed=1.8,
            gated_speed=1.8,
            service_available={service: True for service in MODULE.REQUIRED_SERVICES},
        )

        self.assertEqual(MODULE.diagnose(snapshot), [])

    def test_missing_observations_are_not_reported_as_zero(self):
        snapshot = MODULE.Snapshot(
            operation_mode=None,
            autoware_control_enabled=None,
            vehicle_control_mode=None,
            trajectory_speed=None,
            follower_speed=None,
            gated_speed=None,
            service_available={service: True for service in MODULE.REQUIRED_SERVICES},
        )

        reasons = MODULE.diagnose(snapshot)

        self.assertTrue(any("unavailable or stale" in reason for reason in reasons))
        self.assertFalse(any("remains zero" in reason for reason in reasons))


class TransitionTrackerTest(unittest.TestCase):
    def test_reports_start_timeout_and_recovery_once(self):
        tracker = MODULE.TransitionTracker(timeout_sec=10.0)

        self.assertIsNone(tracker.observe(1, False, False, 0.0))
        self.assertEqual(tracker.observe(2, False, True, 1.0), "start")
        self.assertIsNone(tracker.tick(False, 10.9))
        self.assertEqual(tracker.tick(False, 11.0), "failure")
        self.assertIsNone(tracker.tick(False, 11.5))
        self.assertEqual(tracker.tick(True, 12.0), "recovered")
        self.assertIsNone(tracker.tick(True, 12.5))

    def test_new_transition_edge_restarts_after_failure(self):
        tracker = MODULE.TransitionTracker(timeout_sec=1.0)
        tracker.observe(1, False, False, 0.0)
        self.assertEqual(tracker.observe(2, False, True, 1.0), "start")
        self.assertEqual(tracker.tick(False, 2.0), "failure")

        self.assertIsNone(tracker.observe(2, False, False, 2.1))
        self.assertEqual(tracker.observe(2, False, True, 2.2), "start")

    def test_autonomous_mode_edge_starts_without_transition_flag(self):
        tracker = MODULE.TransitionTracker(timeout_sec=10.0)
        tracker.observe(1, False, False, 0.0)

        self.assertEqual(tracker.observe(2, False, False, 1.0), "start")


class CliTest(unittest.TestCase):
    def test_rejects_non_positive_timeout(self):
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                MODULE.parse_cli(["--timeout", "0"])

    def test_preserves_ros_arguments(self):
        args, ros_args = MODULE.parse_cli(
            ["--timeout", "3.5", "--ros-args", "-r", "__node:=auto_check"]
        )

        self.assertEqual(args.timeout, 3.5)
        self.assertEqual(ros_args, ["--ros-args", "-r", "__node:=auto_check"])


if __name__ == "__main__":
    unittest.main()
