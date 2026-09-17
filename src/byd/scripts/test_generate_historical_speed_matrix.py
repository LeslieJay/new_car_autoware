import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from generate_historical_speed_matrix import (
    combined_rows,
    generate_03mps_estimates,
    render_combined,
)


def make_row(distance, intrusion, speed, clearance, lateral, cross, case_id=None):
    return {
        "case_id": case_id or f"d{distance}m_i{intrusion}m",
        "module": "Simple Avoidance",
        "longitudinal_m": str(distance),
        "intrusion_m": str(intrusion),
        "speed_mps": str(speed),
        "min_collision_clearance_m": str(clearance),
        "max_abs_lateral_offset_near_obstacle_m": str(lateral),
        "time_to_obstacle_cross_sec": str(cross),
        "overall_result": "INCOMPLETE_OBSERVATION",
        "obstacle_phase_result": "PASSED_CLEAR",
        "obstacle_passed": "YES",
        "return_completed": "UNKNOWN_NOT_RECORDED",
        "stop_count": "1",
        "max_stopped_sec": "0.7",
        "mrm_count": "0",
        "invalid_trajectory_count": "0",
        "shoulder": "right",
        "remarks": "RECORDED_HISTORICAL_CASE",
    }


class HistoricalSpeedMatrixTest(unittest.TestCase):
    def setUp(self):
        self.rows = [
            make_row(10, 0.5, 2.07, 0.25, 0.9, 7.4, "d10m_i0.5m_right_r01"),
            make_row(10, 0.5, 1.025, 0.219, 0.901, 12.7, "d10m_i0.5m_right_r02"),
            make_row(12, 0.5, 2.07, 0.188, 0.957, 8.45),
            make_row(15, 0.5, 2.07, 2.316, 2.989, 10.03),
            make_row(18, 0.5, 2.07, 2.230, 2.947, 11.58),
            make_row(15, 1.0, 2.07, 1.265, 2.453, 10.01),
            make_row(18, 1.0, 2.07, 1.073, 2.308, 11.44),
        ]

    def test_generates_six_03mps_parameter_points(self):
        result = generate_03mps_estimates(self.rows)
        self.assertEqual(len(result), 6)
        self.assertTrue(all(row["speed_target_mps"] == 0.3 for row in result))
        self.assertTrue(all(row["data_origin"] == "GENERATED_NOT_RUN" for row in result))

    def test_combined_matrix_preserves_measured_rows_and_has_18_total_rows(self):
        result = combined_rows(self.rows)
        self.assertEqual(len(result), 18)
        self.assertEqual(sum(row["data_origin"] == "MEASURED" for row in result), 7)
        self.assertEqual(sum(row["speed_target_mps"] == 0.3 for row in result), 6)
        self.assertEqual(sum(row["speed_target_mps"] == 1.0 for row in result), 6)
        self.assertEqual(sum(row["speed_target_mps"] == 2.0 for row in result), 6)

    def test_rendered_matrix_omits_remarks_and_formats_decimal_columns(self):
        rendered = render_combined(combined_rows(self.rows))
        header = next(line for line in rendered.splitlines() if line.startswith("| case_id"))
        self.assertNotIn("remarks", header)
        self.assertIn("1.0250000000000000", rendered)
        self.assertIn("0.1880000000000000", rendered)


if __name__ == "__main__":
    unittest.main()
