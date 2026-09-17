#!/usr/bin/env python3
"""Unit tests for the tagged sweep-log summarizer."""

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "summarize_avoidance_steering_sweep.py"


def load_module():
    spec = importlib.util.spec_from_file_location("avoidance_steering_sweep", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SweepSummaryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def test_summarize_case_extracts_candidate_and_boundary_metrics(self):
        with tempfile.TemporaryDirectory() as temporary:
            case_dir = Path(temporary) / "jerk_1.6_distance_4"
            case_dir.mkdir()
            (case_dir / "launch.log").write_text(
                "\n".join(
                    [
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate target_uuid=abc shift_length=-0.770 required_shift=0.770 jerk_distance=2.500 transition_distance=4.000 min_shifting_distance=4.000 ego_speed=1.200 shift_lines=2",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate path_points=100 first_index=10 max_abs_curvature=0.250000 signed_curvature=-0.250000 equivalent_front_wheel_angle_rad=-0.246000 equivalent_front_wheel_angle_deg=-14.094 max_index=45 path_arclength=8.000 wheel_base=1.008",
                        "[DEBUG-SA-STEER] stage=boundary_validation result=FOOTPRINT_OUT_OF_BOUNDARY minimum_boundary_clearance=0.080 required_margin=0.100 clearance_shortfall=0.020 boundary_side=right path_index=45 nearest_footprint_vertex=(1.000,2.000) footprint_vertices=5",
                        "[DEBUG-PP-STEER] target_curvature=1.000000 raw_front_wheel_angle_rad=0.789000 raw_front_wheel_angle_deg=45.209 clamped_front_wheel_angle_rad=0.650000 clamped_front_wheel_angle_deg=37.242 max_steering_angle_rad=0.650000 max_steering_angle_deg=37.242 hard_clamp=true",
                    ]
                ),
                encoding="utf-8",
            )
            row = self.module.summarize_case(case_dir)

        self.assertEqual(row.candidate_count, 1)
        self.assertAlmostEqual(row.jerk_distance_m, 2.5)
        self.assertAlmostEqual(row.transition_distance_m, 4.0)
        self.assertAlmostEqual(row.max_angle_deg, -14.094)
        self.assertEqual(row.boundary_result, "FOOTPRINT_OUT_OF_BOUNDARY")
        self.assertAlmostEqual(row.min_boundary_clearance_m, 0.08)
        self.assertEqual(row.boundary_side, "right")
        self.assertAlmostEqual(row.pure_pursuit_raw_angle_deg, 45.209)
        self.assertAlmostEqual(row.pure_pursuit_clamped_angle_deg, 37.242)
        self.assertEqual(row.pure_pursuit_hard_clamp, "true")

    def test_annotates_changed_and_steeper_path_against_baseline(self):
        baseline = self.module.SweepRow(
            "jerk_0.8_distance_5",
            "baseline.log",
            transition_distance_m=5.0,
            max_curvature_1pm=0.2,
            avoid_start_x_m=0.0,
            avoid_start_y_m=0.0,
            avoid_end_x_m=5.0,
            avoid_end_y_m=0.0,
        )
        steeper = self.module.SweepRow(
            "jerk_1.6_distance_3",
            "steeper.log",
            transition_distance_m=3.0,
            max_curvature_1pm=0.3,
            avoid_start_x_m=1.0,
            avoid_start_y_m=0.0,
            avoid_end_x_m=5.0,
            avoid_end_y_m=0.0,
        )

        self.module.annotate_baseline_comparison([baseline, steeper])

        self.assertEqual(baseline.path_changed_vs_baseline, "NO")
        self.assertEqual(baseline.path_steeper_vs_baseline, "NO")
        self.assertAlmostEqual(steeper.transition_delta_m, -2.0)
        self.assertAlmostEqual(steeper.curvature_delta_percent, 50.0)
        self.assertAlmostEqual(steeper.max_shift_endpoint_delta_m, 1.0)
        self.assertEqual(steeper.path_changed_vs_baseline, "YES")
        self.assertEqual(steeper.path_steeper_vs_baseline, "YES")

    def test_summarize_case_keeps_peak_curvature_with_matching_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            case_dir = Path(directory) / "jerk_1.6_distance_4"
            case_dir.mkdir()
            (case_dir / "diagnostic.log").write_text(
                "\n".join(
                    [
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate path_points=3 max_abs_curvature=0.2 signed_curvature=-0.2 equivalent_front_wheel_angle_rad=-0.19 equivalent_front_wheel_angle_deg=-10.9 max_index=1",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate target_uuid=x shift_length=-1.0 required_shift=1.0 jerk_distance=3.0 transition_distance=4.0 min_shifting_distance=4.0",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate shift_line[0] start_idx=1 end_idx=2 start_shift=0.0 end_shift=-1.0 start=(1.0,2.0) end=(3.0,4.0)",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate path_points=3 max_abs_curvature=0.5 signed_curvature=-0.5 equivalent_front_wheel_angle_rad=-0.46 equivalent_front_wheel_angle_deg=-26.4 max_index=1",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate target_uuid=x shift_length=-2.0 required_shift=2.0 jerk_distance=5.0 transition_distance=5.0 min_shifting_distance=5.0",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate shift_line[0] start_idx=2 end_idx=3 start_shift=0.0 end_shift=-2.0 start=(5.0,6.0) end=(7.0,8.0)",
                    ]
                ),
                encoding="utf-8",
            )
            row = self.module.summarize_case(case_dir)
        self.assertEqual(row.candidate_count, 2)
        self.assertEqual(row.shift_length_m, -2.0)
        self.assertEqual(row.transition_distance_m, 5.0)
        self.assertEqual(row.avoid_start_x_m, 5.0)
        self.assertEqual(row.max_curvature_1pm, 0.5)

    def test_summarize_case_can_select_first_complete_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            case_dir = Path(directory) / "jerk_1.6_distance_4"
            case_dir.mkdir()
            (case_dir / "diagnostic.log").write_text(
                "\n".join(
                    [
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate path_points=3 max_abs_curvature=0.2 signed_curvature=-0.2 equivalent_front_wheel_angle_rad=-0.19 equivalent_front_wheel_angle_deg=-10.9 max_index=1",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate target_uuid=x shift_length=-1.0 required_shift=1.0 jerk_distance=3.0 transition_distance=4.0 min_shifting_distance=4.0",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate shift_line[0] start_idx=1 end_idx=2 start_shift=0.0 end_shift=-1.0 start=(1.0,2.0) end=(3.0,4.0)",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate path_points=3 max_abs_curvature=0.5 signed_curvature=-0.5 equivalent_front_wheel_angle_rad=-0.46 equivalent_front_wheel_angle_deg=-26.4 max_index=1",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate target_uuid=x shift_length=-2.0 required_shift=2.0 jerk_distance=5.0 transition_distance=5.0 min_shifting_distance=5.0",
                        "[DEBUG-SA-STEER] stage=pre_boundary_candidate shift_line[0] start_idx=2 end_idx=3 start_shift=0.0 end_shift=-2.0 start=(5.0,6.0) end=(7.0,8.0)",
                    ]
                ),
                encoding="utf-8",
            )
            row = self.module.summarize_case(case_dir, record_selection="first")
        self.assertEqual(row.candidate_count, 2)
        self.assertEqual(row.shift_length_m, -1.0)
        self.assertEqual(row.transition_distance_m, 4.0)
        self.assertEqual(row.avoid_start_x_m, 1.0)
        self.assertEqual(row.max_curvature_1pm, 0.2)


if __name__ == "__main__":
    unittest.main()
