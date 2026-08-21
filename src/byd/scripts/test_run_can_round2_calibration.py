#!/usr/bin/env python3

import csv
from pathlib import Path
import subprocess
import sys


SCRIPT = Path(__file__).with_name("run_can_round2_calibration.py")


def test_analyze_result_ignores_initial_samples_and_reports_delays(tmp_path: Path) -> None:
    with (tmp_path / "events.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "timestamp", "test_name", "byte5", "byte6", "repetition", "event",
            "phase", "target_velocity", "command_acceleration", "measured_velocity", "result",
        ])
        writer.writerows([
            [1.0, "ramp_0p2", 3, 3, 1, "phase_start", "ramp_up", 0.0, 0.2, 0.0, "ramp_up"],
            [1.7, "ramp_0p2", 3, 3, 1, "phase_start", "hold", 0.5, 0.0, 0.1, "hold"],
            [3.0, "ramp_0p2", 3, 3, 1, "target_speed", "hold", 0.5, 0.0, 0.5, "stable"],
            [4.0, "ramp_0p2", 3, 3, 1, "phase_start", "ramp_down", 0.5, -0.2, 0.5, "ramp_down"],
            [4.7, "ramp_0p2", 3, 3, 1, "phase_start", "settle_zero", 0.0, -0.2, 0.4, "settle_zero"],
        ])
    path = tmp_path / "tracking_samples.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "timestamp", "test_name", "phase", "byte5", "byte6", "repetition",
            "command_velocity_mps", "command_acceleration_mps2", "measured_velocity_mps",
        ])
        writer.writerows([
            [0.0, "none", "initializing", 0, 0, 0, 0.0, -0.5, 0.0],
            [1.0, "ramp_0p2", "ramp_up", 3, 3, 1, 0.00, 0.2, 0.00],
            [1.2, "ramp_0p2", "ramp_up", 3, 3, 1, 0.04, 0.2, 0.02],
            [1.4, "ramp_0p2", "ramp_up", 3, 3, 1, 0.08, 0.2, 0.06],
            [1.6, "ramp_0p2", "ramp_up", 3, 3, 1, 0.12, 0.2, 0.10],
            [4.0, "ramp_0p2", "ramp_down", 3, 3, 1, 0.50, -0.2, 0.50],
            [4.2, "ramp_0p2", "ramp_down", 3, 3, 1, 0.46, -0.2, 0.48],
            [4.4, "ramp_0p2", "ramp_down", 3, 3, 1, 0.42, -0.2, 0.44],
            [4.6, "ramp_0p2", "ramp_down", 3, 3, 1, 0.38, -0.2, 0.40],
        ])

    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--analyze-result", str(tmp_path)],
        text=True, capture_output=True, check=False,
    )

    assert result.returncode == 0, result.stderr
    with (tmp_path / "round2_analysis.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert abs(float(rows[0]["acceleration_response_delay_s"]) - 0.4) < 1e-9
    assert abs(float(rows[0]["deceleration_response_delay_s"]) - 0.4) < 1e-9


def test_verify_wire_pairs_reports_missing_pair(tmp_path: Path) -> None:
    sys.path.insert(0, str(SCRIPT.parent))
    import run_can_round2_calibration as calibration

    calibration.EXPECTED_WIRE_PAIRS = ((2, 10), (10, 2))
    can_log = tmp_path / "can_201.log"
    can_log.write_text(
        "(1.000000) can0 201 [8] 00 00 00 00 1B 02 0A 00\n",
        encoding="utf-8",
    )
    output = tmp_path / "dynamic_wire_check.txt"

    assert not calibration.verify_wire_pairs(can_log, output)
    report = output.read_text(encoding="utf-8")
    assert "byte5=2,byte6=10,frame_count=1" in report
    assert "byte5=10,byte6=2,frame_count=0" in report
    assert "result=fail" in report
