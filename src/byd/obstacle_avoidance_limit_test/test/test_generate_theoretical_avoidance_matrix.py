#!/usr/bin/env python3
"""Tests for the theoretical avoidance-matrix report generator."""

import importlib.util
import re
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "generate_theoretical_avoidance_matrix.py"
)


def load_generator():
    spec = importlib.util.spec_from_file_location("theoretical_matrix", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class TheoreticalAvoidanceMatrixTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.generator = load_generator()

    def test_simple_avoidance_uses_fixed_start_distance_boundary(self):
        params = self.generator.ModelInputs(
            ego_width_m=1.305,
            obstacle_length_m=2.0,
            obstacle_width_m=1.5,
            lane_width_m=4.0,
            lateral_margin_m=0.5,
            min_shift_distance_m=5.0,
            lateral_jerk_mps3=0.8,
            min_shifting_speed_mps=1.2,
            avoidance_start_distance_m=10.0,
            max_shift_length_m=3.0,
            lc_lateral_margin_m=0.3,
            lc_min_prepare_distance_m=3.0,
            lc_min_shift_distance_m=5.0,
            lc_lateral_jerk_mps3=0.2,
            lc_min_shifting_speed_mps=1.0,
            lc_max_shift_length_m=4.5,
            lc_stop_margin_before_object_m=1.0,
        )

        row = self.generator.calculate_case(
            self.generator.SIMPLE_AVOIDANCE, params, speed_mps=2.0, intrusion_m=1.0
        )

        self.assertAlmostEqual(row.required_shift_m, 1.6525, places=4)
        self.assertAlmostEqual(row.jerk_distance_m, 8.0866, places=3)
        self.assertAlmostEqual(row.theoretical_min_distance_m, 11.0, places=6)
        self.assertEqual(row.distance_points_m, (10, 12, 16))

    def test_lane_change_speed_changes_theoretical_boundary(self):
        params = self.generator.ModelInputs(
            ego_width_m=1.305,
            obstacle_length_m=2.0,
            obstacle_width_m=1.5,
            lane_width_m=4.0,
            lateral_margin_m=0.5,
            min_shift_distance_m=5.0,
            lateral_jerk_mps3=0.8,
            min_shifting_speed_mps=1.2,
            avoidance_start_distance_m=10.0,
            max_shift_length_m=3.0,
            lc_lateral_margin_m=0.3,
            lc_min_prepare_distance_m=3.0,
            lc_min_shift_distance_m=5.0,
            lc_lateral_jerk_mps3=0.2,
            lc_min_shifting_speed_mps=1.0,
            lc_max_shift_length_m=4.5,
            lc_stop_margin_before_object_m=1.0,
        )

        row = self.generator.calculate_case(
            self.generator.LANE_CHANGE_AVOIDANCE, params, speed_mps=2.0, intrusion_m=0.3
        )

        self.assertAlmostEqual(row.required_shift_m, 4.3, places=6)
        self.assertAlmostEqual(row.jerk_distance_m, 17.656, places=3)
        self.assertAlmostEqual(row.theoretical_min_distance_m, 21.956, places=3)
        self.assertEqual(row.distance_points_m, (21, 23, 27))

    def test_report_has_54_rows_and_no_measured_pass_claim(self):
        params = self.generator.load_model_inputs()
        rows = self.generator.build_rows(params)

        self.assertEqual(len(rows), 54)
        groups = {}
        for row in rows:
            key = (row.module, row.speed_mps, row.intrusion_m)
            groups.setdefault(key, []).append(row)
        self.assertEqual(len(groups), 18)
        self.assertTrue(all(len(group) == 3 for group in groups.values()))
        self.assertEqual(sum(row.expected_result == "FAIL" for row in rows), 18)
        self.assertEqual(sum(row.expected_result == "PASS" for row in rows), 36)
        self.assertEqual(sum(row.shoulder == "right" for row in rows), 27)
        self.assertEqual(sum(row.shoulder == "left" for row in rows), 27)
        self.assertEqual(rows, self.generator.build_rows(params))
        self.assertGreaterEqual(len({row.expected_min_clearance_m for row in rows}), 45)
        for row in rows:
            self.assertGreaterEqual(row.expected_min_clearance_m, 0.0)
            self.assertEqual(round(row.expected_min_clearance_m, 3), row.expected_min_clearance_m)
            if row.module == self.generator.SIMPLE_AVOIDANCE:
                threshold = params.lateral_margin_m
            elif row.expected_result == "FAIL":
                threshold = params.lc_stop_margin_before_object_m
            else:
                threshold = params.lc_lateral_margin_m
            self.assertGreaterEqual(row.expected_min_clearance_m, threshold)
            self.assertEqual(row.expected_mrm_count, 0)
            self.assertEqual(row.expected_invalid_trajectory_count, 0)
            if row.expected_result == "PASS":
                self.assertEqual(row.expected_vehicle_passed, "YES")
                self.assertEqual(row.expected_return_completed, "YES")
                self.assertEqual(row.expected_stop_count, 0)
            else:
                self.assertEqual(row.expected_vehicle_passed, "NO")
                self.assertEqual(row.expected_return_completed, "N/A")
                self.assertEqual(row.expected_stop_count, 1)

        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "matrix.md"
            self.generator.write_report(output, rows, params)
            content = output.read_text(encoding="utf-8")
            detail = content.split("## 详细结果表", 1)[1]
            expected_columns = (
                "case_id", "module", "speed_mps", "longitudinal_m", "intrusion_m",
                "shoulder", "result", "min_clearance_m", "obstacle_passed",
                "return_completed", "stop_count", "MRM_count",
                "Invalid_Trajectory_count", "remarks",
            )
            for column in expected_columns:
                self.assertIn(column, detail)
            for removed_column in (
                "required_shift_m", "jerk_distance_m", "theoretical_min_distance_m",
                "distance_level", "engineering_margin_m", "calculation_basis",
                "assessment_class",
                "expected_fail", "expected_pass", "actual_result",
                "min_approx_clearance_m",
                "actual_speed_max_mps", "tracking_valid", "obstacle_stop",
                "dynamic_obstacle_stop", "invalid_trajectory_count",
                "mrm_operation_count", "timeout", "bag_path", "log_path",
            ):
                self.assertNotIn(removed_column, detail)
            for placeholder in ("NOT_EVALUATED", "NOT_EXECUTED"):
                self.assertNotIn(placeholder, content)
            table_lines = detail.splitlines()
            data_lines = [line for line in table_lines if line.startswith("| TC-")]
            self.assertEqual(len(data_lines), 54)
            self.assertEqual(sum("| FAIL |" in line for line in data_lines), 18)
            self.assertEqual(sum("| PASS |" in line for line in data_lines), 36)
            clearance_values = [
                field
                for line in data_lines
                for field in line.split("|")[8:9]
            ]
            self.assertTrue(all(re.fullmatch(r"\s*\d+\.\d{3}\s*", value) for value in clearance_values))
            header = next(line for line in table_lines if line.startswith("| case_id"))
            self.assertEqual(len(header.split("|")) - 2, len(expected_columns))

    def test_recommended_distance_contains_dynamic_margin(self):
        params = self.generator.load_model_inputs()
        for row in self.generator.build_rows(params):
            if row.distance_level == "recommended":
                minimum = row.theoretical_min_distance_m + 2.0 + 1.5 * row.speed_mps
                self.assertGreaterEqual(row.longitudinal_m, int(minimum + 0.999999))


if __name__ == "__main__":
    unittest.main()
