import csv
import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from generate_historical_1mps_supplement import (
    SUPPLEMENT_FIELDS,
    format_table_value,
    generate_estimates,
    measured_1mps_row,
)


def row(distance, intrusion, speed, clearance, lateral, cross):
    return {
        "case_id": f"d{distance}m_i{intrusion}m",
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
    }


class HistoricalSupplementTest(unittest.TestCase):
    def test_output_schema_and_decimal_format(self):
        self.assertNotIn("remarks", SUPPLEMENT_FIELDS)
        self.assertEqual(
            format_table_value("speed_mps", 1.0251826736921847),
            "1.0251826736921847",
        )
        self.assertEqual(
            format_table_value("min_clearance_m", 0.188),
            "0.1880000000000000",
        )

    def test_generates_only_missing_1mps_parameter_points(self):
        rows = [
            row(10, 0.5, 1.025, 0.219, 0.901, 12.7),
            row(12, 0.5, 2.07, 0.188, 0.957, 8.45),
            row(15, 0.5, 2.07, 2.316, 2.989, 10.03),
            row(18, 0.5, 2.07, 2.230, 2.947, 11.58),
            row(15, 1.0, 2.07, 1.265, 2.453, 10.01),
            row(18, 1.0, 2.07, 1.073, 2.308, 11.44),
        ]
        measured = measured_1mps_row(rows)
        estimates = generate_estimates(rows)
        self.assertEqual(measured["data_origin"], "MEASURED")
        self.assertEqual(len(estimates), 5)
        self.assertTrue(all(item["data_origin"] == "GENERATED_NOT_RUN" for item in estimates))
        self.assertTrue(all(item["result"] == "ESTIMATED_PASS" for item in estimates))
        self.assertTrue(all(item["MRM_count"] == 0 for item in estimates))


if __name__ == "__main__":
    unittest.main()
