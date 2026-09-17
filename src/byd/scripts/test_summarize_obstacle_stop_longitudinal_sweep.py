import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from summarize_obstacle_stop_longitudinal_sweep import (
    aggregate,
    expected_rows,
    load_rows,
    write_aggregate,
    write_cases,
    write_report,
)


class ObstacleStopSummaryTest(unittest.TestCase):
    def make_root(self):
        return tempfile.TemporaryDirectory()

    def write_result(self, root, name, **fields):
        case = Path(root) / name
        case.mkdir(parents=True, exist_ok=True)
        payload = {
            "case_id": name,
            "repetition": 1,
            "longitudinal_m": 12.0,
            "intrusion_m": 0.5,
            "shoulder": "right",
            "result": "PASS",
            "min_collision_clearance_m": 0.2,
            "time_to_obstacle_cross_sec": 10.0,
            "time_to_goal_sec": 20.0,
            **fields,
        }
        (case / "result.json").write_text(json.dumps(payload), encoding="utf-8")

    def test_missing_matrix_cases_are_setup_invalid(self):
        with self.make_root() as root:
            self.write_result(root, "one", repetition=1)
            rows = expected_rows(
                Path(root), load_rows(Path(root)), [12.0], [0.5, 1.0], "right", 2
            )
            self.assertEqual(len(rows), 4)
            missing = [row for row in rows if row["result"] == "SETUP_INVALID"]
            self.assertEqual(len(missing), 3)
            self.assertTrue(all(row["failure_category"] == "SETUP_INVALID" for row in missing))
            self.assertIn("d12m_i1m_right_r02", {row["case_id"] for row in missing})

    def test_aggregate_reports_stability_and_failure_counts(self):
        rows = [
            {
                "longitudinal_m": 12.0,
                "intrusion_m": 0.5,
                "shoulder": "right",
                "result": "PASS",
                "min_collision_clearance_m": 0.4,
                "time_to_obstacle_cross_sec": 8.0,
                "time_to_goal_sec": 18.0,
            },
            {
                "longitudinal_m": 12.0,
                "intrusion_m": 0.5,
                "shoulder": "right",
                "result": "STOPPED_BEFORE_OBSTACLE",
                "failure_reason": "NO_PROGRESS_BEFORE_OBSTACLE",
                "min_collision_clearance_m": 0.1,
            },
        ]
        result = aggregate(rows, expected_repetitions=2)
        self.assertEqual(result[0]["pass_count"], 1)
        self.assertEqual(result[0]["stable_result"], "UNSTABLE")
        self.assertEqual(result[0]["count_stopped_before_obstacle"], 1)
        self.assertEqual(result[0]["median_clearance_m"], 0.25)

    def test_setup_invalid_is_incomplete_not_a_behavior_failure(self):
        rows = [
            {"longitudinal_m": 12.0, "intrusion_m": 0.5, "shoulder": "right", "result": "PASS"},
            {"longitudinal_m": 12.0, "intrusion_m": 0.5, "shoulder": "right", "result": "SETUP_INVALID"},
        ]
        result = aggregate(rows, expected_repetitions=2)[0]
        self.assertEqual(result["stable_result"], "INCOMPLETE")
        self.assertEqual(result["setup_invalid_count"], 1)
        self.assertEqual(result["pass_rate"], 1.0)

    def test_outputs_include_case_and_aggregate_tables(self):
        with self.make_root() as root:
            self.write_result(root, "d12m_i0.5m_right_r01")
            rows = load_rows(Path(root))
            self.assertTrue(write_cases(Path(root), rows).exists())
            self.assertTrue(write_aggregate(Path(root), rows, 1).exists())
            report = write_report(Path(root), rows, [12.0], [0.5], "right", 1)
            self.assertIn("通过率矩阵", report.read_text(encoding="utf-8"))
            self.assertIn("cases.csv", {path.name for path in Path(root).iterdir()})


if __name__ == "__main__":
    unittest.main()
