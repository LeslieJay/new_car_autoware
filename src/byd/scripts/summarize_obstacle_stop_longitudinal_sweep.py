#!/usr/bin/env python3
"""Summarize independent Obstacle Stop longitudinal test cases."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


FIELDS = [
    "longitudinal_m",
    "result",
    "obstacle_stop_module_loaded",
    "dynamic_obstacle_stop_module_loaded",
    "module_configuration_ok",
    "object_count",
    "duplicate_ground_truth_uuid",
    "has_simple_avoidance",
    "has_obstacle_stop",
    "has_dynamic_obstacle_stop",
    "obstacle_stop_behavior_ok",
    "vehicle_passed",
    "obstacle_crossed",
    "goal_reached",
    "obstacle_crossed_epoch",
    "goal_reached_epoch",
    "route_state",
    "route_set_seen",
    "collision_free",
    "min_collision_clearance_m",
    "pre_obstacle_motion",
    "invalid_trajectory_count",
    "mrm_operation_count",
    "monitor_result",
    "early_stop_reason",
    "timeout",
    "bag_path",
]


def load_case(case_dir: Path, distance: float) -> dict:
    result_path = case_dir / "result.json"
    if not result_path.exists():
        return {
            "longitudinal_m": distance,
            "result": "INCOMPLETE",
            "early_stop_reason": "MISSING_RESULT_JSON",
            "bag_path": str(case_dir / "rosbag"),
        }
    try:
        result = json.loads(result_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {
            "longitudinal_m": distance,
            "result": "INCOMPLETE",
            "early_stop_reason": f"INVALID_RESULT_JSON:{exc}",
            "bag_path": str(case_dir / "rosbag"),
        }
    result.setdefault("longitudinal_m", distance)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path, help="sweep 输出目录")
    parser.add_argument(
        "--distances",
        nargs="+",
        type=float,
        default=[8, 10, 12, 14, 16, 18, 20],
    )
    args = parser.parse_args()

    rows = [
        load_case(args.root / f"{distance:g}m", distance)
        for distance in args.distances
    ]
    rows.sort(key=lambda row: float(row.get("longitudinal_m", 0.0)))
    passed = sum(row.get("result") == "PASS" for row in rows)
    failed_rows = [row for row in rows if row.get("result") != "PASS"]
    overall_result = (
        "PASS"
        if len(rows) == len(args.distances) and not failed_rows
        else "FAIL"
    )
    first_failure_distance = (
        failed_rows[0].get("longitudinal_m") if failed_rows else None
    )

    args.root.mkdir(parents=True, exist_ok=True)
    with (args.root / "summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in FIELDS} for row in rows)

    summary = {
        "overall_result": overall_result,
        "expected_cases": len(args.distances),
        "completed_cases": len(rows),
        "passed_cases": passed,
        "failed_cases": len(failed_rows),
        "first_failure_distance_m": first_failure_distance,
        "results": rows,
    }
    (args.root / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    markdown_rows = []
    for row in rows:
        markdown_rows.append(
            "| {distance:g} | {result} | {passed} | {collision_free} | "
            "{clearance} | {stop} | {reason} |".format(
                distance=float(row.get("longitudinal_m", 0.0)),
                result=row.get("result", "INCOMPLETE"),
                passed=row.get("vehicle_passed", False),
                collision_free=row.get("collision_free", False),
                clearance=row.get("min_collision_clearance_m", ""),
                stop=row.get("has_obstacle_stop", False),
                reason=row.get("early_stop_reason", ""),
            )
        )
    (args.root / "summary.md").write_text(
        "# Obstacle Stop longitudinal sweep\n\n"
        f"- Overall: **{overall_result}**\n"
        f"- Passed: {passed}/{len(args.distances)}\n"
        f"- First failure: {first_failure_distance or 'none'}\n\n"
        "| Distance (m) | Result | Passed | Collision-free | Min clearance (m) | Stop factor | Reason |\n"
        "|---:|---|---|---|---:|---|---|\n"
        + "\n".join(markdown_rows)
        + "\n",
        encoding="utf-8",
    )
    print(args.root / "summary.csv")
    print(f"overall_result={overall_result}")
    return 0 if overall_result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
