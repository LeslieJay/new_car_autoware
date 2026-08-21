#!/usr/bin/env python3
"""Rank delay_compensation_time candidates by signed longitudinal stop error."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys

from analyze_forward_endpoint_accuracy import load_bag_series, yaw_from_quat


TOPICS = [
    "/planning/trajectory",
    "/localization/kinematic_state",
    "/vehicle/status/velocity_status",
]


def read_metadata(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def analyze_run(run_dir: Path) -> dict:
    metadata = read_metadata(run_dir / "metadata.env")
    result = {
        "run": str(run_dir),
        "delay_s": float(metadata["delay_compensation_time"]),
        "repetition": int(metadata["repetition"]),
    }
    series, _ = load_bag_series(str(run_dir / "rosbag"), TOPICS)
    if not series.get("/planning/trajectory"):
        return {**result, "error": "missing planning trajectory"}
    if not series.get("/localization/kinematic_state"):
        return {**result, "error": "missing localization state"}

    trajectory = series["/planning/trajectory"][-1][1]
    stop_point = next(
        (point for point in trajectory.points
         if abs(point.longitudinal_velocity_mps) <= 1.0e-3),
        None,
    )
    if stop_point is None:
        return {**result, "error": "trajectory has no zero-velocity point"}

    final_pose = series["/localization/kinematic_state"][-1][1].pose.pose
    stop_pose = stop_point.pose
    yaw = yaw_from_quat(stop_pose.orientation)
    dx = final_pose.position.x - stop_pose.position.x
    dy = final_pose.position.y - stop_pose.position.y
    longitudinal_error = math.cos(yaw) * dx + math.sin(yaw) * dy
    lateral_error = -math.sin(yaw) * dx + math.cos(yaw) * dy

    speeds = series.get("/vehicle/status/velocity_status", [])
    final_speed = (
        abs(float(speeds[-1][1].longitudinal_velocity)) if speeds else math.nan
    )
    return {
        **result,
        "signed_longitudinal_error_m": longitudinal_error,
        "lateral_error_m": lateral_error,
        "absolute_position_error_m": math.hypot(dx, dy),
        "final_speed_mps": final_speed,
        "valid": math.isfinite(final_speed) and final_speed <= 0.05,
    }


def summarize(rows: list[dict]) -> list[dict]:
    summaries = []
    for delay in sorted({row["delay_s"] for row in rows if row.get("valid")}):
        values = [
            row["signed_longitudinal_error_m"]
            for row in rows if row.get("valid") and row["delay_s"] == delay
        ]
        summaries.append({
            "delay_s": delay,
            "valid_runs": len(values),
            "mean_signed_error_m": statistics.mean(values),
            "median_signed_error_m": statistics.median(values),
            "stddev_m": statistics.stdev(values) if len(values) > 1 else 0.0,
            "rmse_m": math.sqrt(statistics.mean(value * value for value in values)),
            "max_abs_error_m": max(abs(value) for value in values),
        })
    return summaries


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_root", type=Path)
    args = parser.parse_args()

    run_dirs = sorted(
        path.parent for path in args.result_root.glob("delay_*_rep_*/metadata.env")
    )
    if not run_dirs:
        print("未找到 delay_*_rep_*/metadata.env", file=sys.stderr)
        return 2

    rows = []
    for run_dir in run_dirs:
        try:
            rows.append(analyze_run(run_dir))
        except Exception as error:  # Keep other completed runs analyzable.
            rows.append({"run": str(run_dir), "error": str(error)})
    summaries = summarize(rows)

    (args.result_root / "delay_calibration_details.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    with (args.result_root / "delay_calibration_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        fieldnames = [
            "delay_s", "valid_runs", "mean_signed_error_m",
            "median_signed_error_m", "stddev_m", "rmse_m", "max_abs_error_m",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)

    print("delay(s)  valid  mean_signed(m)  std(m)  rmse(m)  max_abs(m)")
    for row in summaries:
        print(
            f"{row['delay_s']:>7.2f}  {row['valid_runs']:>5d}  "
            f"{row['mean_signed_error_m']:>14.4f}  {row['stddev_m']:>6.4f}  "
            f"{row['rmse_m']:>7.4f}  {row['max_abs_error_m']:>10.4f}"
        )
    eligible = [row for row in summaries if row["valid_runs"] >= 2]
    if eligible:
        best = min(eligible, key=lambda row: row["rmse_m"])
        print(f"\n初步推荐 delay_compensation_time={best['delay_s']:.2f}s")
        print("正误差表示越过零速度点，负误差表示提前停车。")
    else:
        print("\n有效重复次数不足，暂不推荐参数。")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
