#!/usr/bin/env python3

import csv
from pathlib import Path
import subprocess
import sys


SCRIPT = Path(__file__).with_name("run_can_step_calibration.py")


def write_csv(path: Path, header: list[str], rows: list[list[object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def test_analyze_result_uses_only_acceleration_phase(tmp_path: Path) -> None:
    write_csv(
        tmp_path / "events.csv",
        [
            "timestamp", "test_type", "byte_value", "repetition", "phase",
            "target_velocity", "command_acceleration", "measured_velocity", "result",
        ],
        [
            [0.0, "byte5_acceleration", 1, 1, "phase_start", 0.5, 0.2, 0.0, "running"],
            [5.0, "byte5_acceleration", 1, 1, "target_speed", 0.5, 0.2, 0.5, "stable"],
            [7.0, "byte5_acceleration", 1, 1, "phase_start", 0.0, -0.5, 0.5, "running"],
            [9.0, "byte5_acceleration", 1, 1, "zero_speed", 0.0, -0.5, 0.0, "stable"],
        ],
    )
    write_csv(
        tmp_path / "velocity_samples.csv",
        ["timestamp", "velocity_mps"],
        [
            [0.0, 0.0], [1.0, 0.1], [2.0, 0.2], [3.0, 0.3], [4.0, 0.4], [5.0, 0.5],
            [7.0, 0.5], [7.5, 0.4], [8.0, 0.3], [8.5, 0.2], [9.0, 0.0],
        ],
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--analyze-result", str(tmp_path)],
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    with (tmp_path / "calibration_analysis.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert float(rows[0]["fitted_acceleration_mps2"]) == 0.1
    assert float(rows[0]["response_delay_s"]) == 1.0


def test_analyze_result_reports_robust_candidate_and_outlier(tmp_path: Path) -> None:
    event_rows = []
    sample_rows = []
    start = 0.0
    slopes = {1: (0.05, 0.05, 0.05), 2: (0.10, 0.02, 0.10), 3: (0.15, 0.15, 0.15)}
    for byte_value, repetitions in slopes.items():
        for repetition, slope in enumerate(repetitions, 1):
            duration = 0.4 / slope
            event_rows.extend([
                [start, "byte5_acceleration", byte_value, repetition, "phase_start", 0.5, 0.2, 0.0, "running"],
                [start + duration + 1.0, "byte5_acceleration", byte_value, repetition, "target_speed", 0.5, 0.2, 0.5, "stable"],
            ])
            for index in range(9):
                velocity = index * 0.05
                sample_rows.append([start + velocity / slope, velocity])
            start += duration + 2.0
    write_csv(
        tmp_path / "events.csv",
        ["timestamp", "test_type", "byte_value", "repetition", "phase",
         "target_velocity", "command_acceleration", "measured_velocity", "result"],
        event_rows,
    )
    write_csv(tmp_path / "velocity_samples.csv", ["timestamp", "velocity_mps"], sample_rows)

    result = subprocess.run(
        [sys.executable, str(SCRIPT), "--analyze-result", str(tmp_path)],
        text=True, capture_output=True, check=False,
    )

    assert result.returncode == 0, result.stderr
    with (tmp_path / "calibration_groups.csv").open(encoding="utf-8") as stream:
        groups = list(csv.DictReader(stream))
    assert [round(float(row["median_acceleration_mps2"]), 2) for row in groups] == [0.05, 0.10, 0.15]
    with (tmp_path / "calibration_analysis.csv").open(encoding="utf-8") as stream:
        trials = list(csv.DictReader(stream))
    flagged = [row for row in trials if row["quality"] == "outlier"]
    assert [(int(row["byte_value"]), int(row["repetition"])) for row in flagged] == [(2, 2)]
    candidates = (tmp_path / "calibration_candidates.yaml").read_text(encoding="utf-8")
    assert "acceleration_step_counts_per_mps2: 20.0" in candidates
    assert "speed_mapping: not_calibrated" in candidates
    assert "steering_mapping: not_calibrated" in candidates
