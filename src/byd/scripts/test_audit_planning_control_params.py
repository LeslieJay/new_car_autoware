import tempfile
import unittest
import sys
import shutil
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from audit_planning_control_params import (
    ACTIVE,
    DISABLED,
    audit,
    load_yaml,
    range_order_violations,
)


WORKSPACE = Path(__file__).resolve().parents[3]
CONFIG_ROOT = WORKSPACE / "src/launcher/autoware_launch/autoware_launch/config"


class MinimalYamlChecksTest(unittest.TestCase):
    def test_duplicate_parameter_key_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "duplicate.yaml"
            path.write_text(
                "/**:\n  ros__parameters:\n    min_acc: -1.0\n    min_acc: -2.0\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate key"):
                load_yaml(path)

    def test_inverted_range_fixture_is_detected(self):
        fixture = {"limits": {"min_acc": 1.0, "max_acc": -1.0}}
        self.assertEqual(
            range_order_violations(fixture),
            [("limits.min_acc", 1.0, "limits.max_acc", -1.0)],
        )

    def test_valid_range_fixture_is_clean(self):
        self.assertEqual(
            range_order_violations({"limits": {"min_acc": -1.0, "max_acc": 1.0}}),
            [],
        )


class WorkspaceAuditRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.inventory, cls.findings, cls.counts = audit(WORKSPACE, CONFIG_ROOT)

    def test_all_planning_and_control_yaml_are_inventoried(self):
        self.assertEqual(self.counts["yaml_files"], 80)
        self.assertEqual(self.counts["planning_files"], 62)
        self.assertEqual(self.counts["control_files"], 18)
        self.assertEqual(len(self.inventory), 80)

    def test_source_and_install_configs_match(self):
        self.assertEqual(self.counts["install_compared"], 80)
        self.assertEqual(self.counts["install_mismatches"], 0)

    def test_current_active_planner_validator_envelope_is_clean(self):
        self.assertFalse(any(
            item.id == "PLANNER_VALIDATOR_MIN_ACC" and item.severity == "P1"
            for item in self.findings
        ))

    def test_old_validator_floor_fixture_still_goes_red(self):
        with tempfile.TemporaryDirectory() as directory:
            fixture_root = Path(directory) / "config"
            shutil.copytree(CONFIG_ROOT, fixture_root)
            validator = fixture_root / (
                "planning/scenario_planning/common/planning_validator/"
                "trajectory_checker.param.yaml"
            )
            text = validator.read_text(encoding="utf-8")
            self.assertIn("threshold: -3.0", text)
            validator.write_text(text.replace("threshold: -3.0", "threshold: -1.5", 1), encoding="utf-8")
            _, findings, _ = audit(WORKSPACE, fixture_root)
            self.assertTrue(any(
                item.id == "PLANNER_VALIDATOR_MIN_ACC"
                and item.file == "planning/scenario_planning/common/common.param.yaml"
                and item.severity == "P1"
                for item in findings
            ))

    def test_avoidance_arbitration_conflict_is_resolved(self):
        self.assertFalse(any(item.id == "AVOIDANCE_ARBITRATION" for item in self.findings))

    def test_default_enablement_is_classified(self):
        states = {item.file: item.status for item in self.inventory}
        self.assertEqual(
            states["planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/simple_lc_avoidance/simple_lc_avoidance.param.yaml"],
            DISABLED,
        )
        self.assertEqual(
            states["planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/stop_line.param.yaml"],
            DISABLED,
        )


if __name__ == "__main__":
    unittest.main()
