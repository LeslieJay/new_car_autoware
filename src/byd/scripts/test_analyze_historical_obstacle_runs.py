import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_historical_obstacle_runs import (
    BagRecord,
    LogRecord,
    classify_historical_result,
    max_stopped_duration,
    stopped_event_count,
    aggregate,
    pair_records,
    parse_case_parameters,
    parse_log_events,
)


class HistoricalPairingTest(unittest.TestCase):
    def test_parse_case_parameters_uses_tenths_for_intrusion_token(self):
        self.assertEqual(parse_case_parameters(Path("10_05.log")), (10.0, 0.5))
        self.assertEqual(parse_case_parameters(Path("15_10.log")), (15.0, 1.0))

    def test_pair_records_assigns_repetitions_by_parameter_point(self):
        bags = [
            BagRecord(Path("a"), 100.0, 40.0),
            BagRecord(Path("b"), 200.0, 40.0),
        ]
        logs = [
            LogRecord(Path("10_05.log"), 90.0),
            LogRecord(Path("repeat/10_05.log"), 190.0),
        ]
        pairs = pair_records(bags, logs, shoulder="right")
        self.assertEqual([pair.case_id for pair in pairs], [
            "d10m_i0.5m_right_r01",
            "d10m_i0.5m_right_r02",
        ])
        self.assertTrue(all(pair.pairing_status == "OK" for pair in pairs))


class HistoricalLogEvidenceTest(unittest.TestCase):
    def test_runtime_mrm_excludes_startup_operation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "run.log"
            path.write_text(
                "\n".join(
                    [
                        "[1.0] [system.mrm_handler]: EMERGENCY_STOP is operated.",
                        "[10.0] [adapi.node.autoware_state]: WaitingForEngage => Driving",
                        "[12.0] [system.mrm_handler]: COMFORTABLE_STOP is operated.",
                        "[13.0] [planning_validator]: Invalid Trajectory detected.",
                        "[20.0] [adapi.node.autoware_state]: Driving => ArrivedGoal",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            events = parse_log_events(path)
        self.assertEqual(events["driving_epoch"], 10.0)
        self.assertEqual(events["arrived_goal_epoch"], 20.0)
        self.assertEqual(events["runtime_mrm_count"], 1)
        self.assertEqual(events["invalid_trajectory_count"], 1)


class HistoricalClassificationTest(unittest.TestCase):
    def test_missing_arrival_is_incomplete_observation_after_clear(self):
        evidence = dict(
            setup_valid=True,
            runtime_mrm_count=0,
            invalid_trajectory_count=0,
            min_collision_clearance_m=0.2,
            obstacle_phase_result="PASSED_CLEAR",
            arrived_goal=False,
        )
        self.assertEqual(classify_historical_result(evidence), "INCOMPLETE_OBSERVATION")

    def test_runtime_failure_precedes_missing_arrival(self):
        evidence = dict(
            setup_valid=True,
            runtime_mrm_count=1,
            invalid_trajectory_count=0,
            min_collision_clearance_m=0.2,
            obstacle_phase_result="PASSED_CLEAR",
            arrived_goal=False,
        )
        self.assertEqual(classify_historical_result(evidence), "MRM")

    def test_max_stopped_duration_uses_contiguous_near_zero_samples(self):
        samples = [
            (0, 0.0, 0.0, 0.0, 0.0, 1.0),
            (1_000_000_000, 1.0, 0.0, 0.0, 0.0, 0.0),
            (3_000_000_000, 1.0, 0.0, 0.0, 0.0, 0.0),
            (4_000_000_000, 2.0, 0.0, 0.0, 0.0, 1.0),
        ]
        self.assertAlmostEqual(max_stopped_duration(samples, 0), 2.0)
        self.assertEqual(stopped_event_count(samples, 0), 1)

    def test_aggregate_uses_observed_repetition_count(self):
        rows = [
            {
                "longitudinal_m": 10.0,
                "intrusion_m": 0.5,
                "shoulder": "right",
                "overall_result": "INCOMPLETE_OBSERVATION",
                "obstacle_phase_result": "PASSED_CLEAR",
                "obstacle_crossed_epoch": 12.0,
                "driving_epoch": 2.0,
                "arrived_goal_epoch": None,
                "min_collision_clearance_m": 0.2,
            },
            {
                "longitudinal_m": 10.0,
                "intrusion_m": 0.5,
                "shoulder": "right",
                "overall_result": "MRM",
                "obstacle_phase_result": "PASSED_CLEAR",
                "obstacle_crossed_epoch": None,
                "driving_epoch": 20.0,
                "arrived_goal_epoch": None,
                "min_collision_clearance_m": 0.1,
            },
        ]
        result = aggregate(rows)[0]
        self.assertEqual(result["observed_repetitions"], 2)
        self.assertEqual(result["pass_rate"], 0.0)
        self.assertEqual(result["count_mrm"], 1)
        self.assertEqual(result["obstacle_clear_count"], 2)


if __name__ == "__main__":
    unittest.main()
