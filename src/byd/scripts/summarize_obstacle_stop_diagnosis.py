#!/usr/bin/env python3
"""Aggregate timeline.json artifacts from obstacle-stop diagnosis runs."""

from __future__ import annotations

import argparse
import csv
import json
import re
from pathlib import Path


def load_timelines(root: Path) -> list[dict]:
    rows = []
    for path in sorted(root.rglob("timeline.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        classification = data.get("classification", {})
        monitor = data.get("monitor", {})
        distance = data.get("distance_m")
        if distance is None:
            distance = next(
                (float(match.group(1)) for part in reversed(path.parts)
                 if (match := re.fullmatch(r"([0-9]+(?:\.[0-9]+)?)m", part))),
                None,
            )
        rows.append({
            "distance_m": distance,
            "protocol": next((name for name in (
                "original_protocol", "baseline_no_obstacle", "obstacle_stop_off", "start_planner_freespace_off"
            ) if any(name in part for part in path.parts)), "unknown"),
            "case_path": str(path.parent),
            "monitor_result": classification.get("monitor_result", monitor.get("monitor_result")),
            "vehicle_passed": classification.get("vehicle_passed", monitor.get("vehicle_passed")),
            "obstacle_crossed": classification.get("obstacle_crossed", monitor.get("obstacle_crossed")),
            "goal_reached": classification.get("goal_reached", monitor.get("goal_reached")),
            "obstacle_crossed_epoch": classification.get("obstacle_crossed_epoch"),
            "goal_reached_epoch": classification.get("goal_reached_epoch"),
            "route_state": classification.get("route_state", monitor.get("route_state")),
            "path_generated_count": classification.get("path_generated_count"),
            "first_commit_time": classification.get("first_commit_time"),
            "candidate_commitment_count": classification.get("candidate_commitment_count"),
            "target_expiry_count": classification.get("target_expiry_count"),
            "first_obstacle_stop_factor_time": classification.get("first_obstacle_stop_factor_time"),
            "expiry_to_stop_s": classification.get("expiry_to_stop_s"),
            "trajectory_gap_after_target_lock_s": classification.get("trajectory_gap_after_target_lock_s"),
            "trajectory_timeout_count": classification.get("trajectory_timeout_count"),
            "max_reported_trajectory_delay_s": classification.get("max_reported_trajectory_delay_s"),
            "startup_contaminated": classification.get("startup_contaminated"),
            "planner_cycles": classification.get("planner_cycles", []),
            "bag_path": str(Path(data.get("case_dir", path.parent)) / "rosbag"),
        })
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    rows = load_timelines(args.root)
    fieldnames = [
        "distance_m", "protocol", "case_path", "monitor_result", "vehicle_passed",
        "obstacle_crossed", "goal_reached", "obstacle_crossed_epoch", "goal_reached_epoch", "route_state",
        "path_generated_count", "first_commit_time", "candidate_commitment_count",
        "target_expiry_count", "first_obstacle_stop_factor_time", "expiry_to_stop_s",
        "trajectory_gap_after_target_lock_s", "trajectory_timeout_count",
        "max_reported_trajectory_delay_s", "startup_contaminated", "planner_cycles", "bag_path",
    ]
    csv_path = args.root / "diagnosis_summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    cycle_path = args.root / "cycle_comparison.csv"
    cycle_fields = [
        "distance_m", "case_path", "monitor_result", "vehicle_passed", "goal_reached",
        "cycle", "commitment_distance_to_shift_start_m", "commitment_state", "committed",
        "path_dist_to_avoid_start_m", "path_required_before_front_m", "commitment_path_order",
        "path_commitment_distance_delta_m", "timing_entry_gap_ms",
    ]
    cycle_rows = []
    for row in rows:
        for cycle in row.get("planner_cycles", []):
            cycle_rows.append({
                "distance_m": row["distance_m"],
                "case_path": row["case_path"],
                "monitor_result": row["monitor_result"],
                "vehicle_passed": row["vehicle_passed"],
                "goal_reached": row["goal_reached"],
                **{field: cycle.get(field) for field in cycle_fields[5:]},
            })
    with cycle_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=cycle_fields)
        writer.writeheader()
        writer.writerows(cycle_rows)

    by_distance: dict[str, list[dict]] = {}
    for row in rows:
        by_distance.setdefault(str(row["distance_m"]), []).append(row)
    summary = {
        "root": str(args.root),
        "case_count": len(rows),
        "rows": rows,
        "cycle_comparison_path": str(cycle_path),
        "by_distance": by_distance,
        "notes": [
            "A vehicle_passed result is not considered a valid bypass when startup_contaminated is true.",
            "The diagnosis artifacts are observational; no planner parameter is changed by this summarizer.",
        ],
    }
    (args.root / "diagnosis_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    lines = [
        "# Obstacle Stop 非单调结果诊断汇总",
        "",
        f"测试根目录：`{args.root}`",
        f"时间轴产物数：{len(rows)}",
        "",
        "| 距离 | 协议 | 监视结果 | 越过障碍物 | 路径生成次数 | commit 次数 | 目标过期 | 首次停车因子 | trajectory 最大延迟 | 启动污染 |",
        "|---:|---|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in sorted(rows, key=lambda item: (float(item["distance_m"] or 1e9), item["protocol"], item["case_path"])):
        lines.append(
            "| {distance_m} | {protocol} | {monitor_result} | {vehicle_passed} | {path_generated_count} | {first_commit_time} | {target_expiry_count} | {first_obstacle_stop_factor_time} | {max_reported_trajectory_delay_s} | {startup_contaminated} |".format(**row)
        )
    (args.root / "diagnosis_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps({"case_count": len(rows), "csv": str(csv_path), "json": str(args.root / "diagnosis_summary.json")}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
