#!/usr/bin/env python3
"""Summarize independent Obstacle Stop cases and a 2-D matrix."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median


CASE_FIELDS = [
    "case_id", "repetition", "longitudinal_m", "intrusion_m", "shoulder", "label",
    "control_label", "result", "failure_category", "failure_reason", "clear_confirmed",
    "early_stop_reason", "object_count", "duplicate_ground_truth_uuid",
    "has_simple_avoidance", "has_obstacle_stop", "obstacle_stop_behavior_ok", "has_dynamic_obstacle_stop",
    "obstacle_stop_module_loaded", "dynamic_obstacle_stop_module_loaded",
    "module_configuration_ok", "vehicle_passed", "obstacle_crossed",
    "goal_reached", "route_state", "route_set_seen", "collision_free",
    "obstacle_longitudinal_m", "obstacle_x", "obstacle_y", "obstacle_yaw",
    "min_collision_clearance_m", "max_abs_lateral_offset_m",
    "max_signed_lateral_offset_m", "min_signed_lateral_offset_m",
    "min_speed_mps", "max_speed_mps", "max_stopped_sec",
    "time_to_obstacle_cross_sec", "time_to_goal_sec", "invalid_trajectory_count",
    "mrm_operation_count", "pre_obstacle_motion", "monitor_result", "timeout",
    "bag_path",
]

RESULTS = (
    "PASS", "STOPPED_BEFORE_OBSTACLE", "STOPPED_AFTER_OBSTACLE", "COLLISION",
    "MRM", "INVALID_TRAJECTORY", "TIMEOUT", "SETUP_INVALID",
)


def _float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_rows(root: Path) -> list[dict]:
    rows: list[dict] = []
    for result_path in sorted(root.rglob("result.json")):
        try:
            row = json.loads(result_path.read_text(encoding="utf-8"))
            if not isinstance(row, dict):
                raise ValueError("result is not an object")
        except (OSError, json.JSONDecodeError, ValueError) as exc:
            row = {
                "case_id": result_path.parent.name,
                "result": "SETUP_INVALID",
                "failure_category": "SETUP_INVALID",
                "failure_reason": f"INVALID_RESULT_JSON:{type(exc).__name__}",
                "bag_path": str(result_path.parent / "rosbag"),
            }
        row.setdefault("case_id", result_path.parent.name)
        row.setdefault("repetition", 1)
        row.setdefault("result", "SETUP_INVALID")
        row.setdefault("failure_category", None if row["result"] == "PASS" else row["result"])
        row.setdefault("bag_path", str(result_path.parent / "rosbag"))
        row["_result_path"] = str(result_path)
        rows.append(row)
    return rows


def expected_rows(
    root: Path,
    rows: list[dict],
    distances: list[float] | None,
    intrusions: list[float] | None,
    shoulder: str | None,
    repetitions: int | None,
) -> list[dict]:
    """Add explicit SETUP_INVALID rows for missing matrix cases."""
    if not distances or not intrusions or repetitions is None:
        return rows
    existing = {
        (
            _float(row.get("longitudinal_m")), _float(row.get("intrusion_m")),
            row.get("shoulder", shoulder), int(row.get("repetition", 1)),
        )
        for row in rows
        if not row.get("baseline_without_obstacle")
    }
    result = list(rows)
    for distance in distances:
        for intrusion in intrusions:
            for repetition in range(1, repetitions + 1):
                key = (distance, intrusion, shoulder, repetition)
                if key in existing:
                    continue
                case_id = f"d{distance:g}m_i{intrusion:g}m_{shoulder}_r{repetition:02d}"
                result.append({
                    "case_id": case_id,
                    "repetition": repetition,
                    "longitudinal_m": distance,
                    "intrusion_m": intrusion,
                    "shoulder": shoulder,
                    "result": "SETUP_INVALID",
                    "failure_category": "SETUP_INVALID",
                    "failure_reason": "MISSING_RESULT_JSON",
                    "bag_path": str(root / case_id / "rosbag"),
                })
    return result


def write_cases(root: Path, rows: list[dict]) -> Path:
    path = root / "cases.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CASE_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in CASE_FIELDS})
    return path


def aggregate(rows: list[dict], expected_repetitions: int | None) -> list[dict]:
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for row in rows:
        if row.get("baseline_without_obstacle"):
            continue
        key = (
            _float(row.get("longitudinal_m")), _float(row.get("intrusion_m")),
            row.get("shoulder", "right"),
        )
        if key[0] is not None and key[1] is not None:
            groups[key].append(row)

    aggregate_rows: list[dict] = []
    for (distance, intrusion, side), group in sorted(groups.items()):
        counts = Counter(row.get("result", "SETUP_INVALID") for row in group)
        clearances = [_float(row.get("min_collision_clearance_m")) for row in group]
        cross_times = [_float(row.get("time_to_obstacle_cross_sec")) for row in group]
        goal_times = [_float(row.get("time_to_goal_sec")) for row in group]
        clearances = [value for value in clearances if value is not None]
        cross_times = [value for value in cross_times if value is not None]
        goal_times = [value for value in goal_times if value is not None]
        expected = expected_repetitions or len(group)
        pass_count = counts.get("PASS", 0)
        setup_invalid_count = counts.get("SETUP_INVALID", 0)
        valid_expected = max(0, expected - setup_invalid_count)
        aggregate_rows.append({
            "longitudinal_m": distance,
            "intrusion_m": intrusion,
            "shoulder": side,
            "expected_repetitions": expected,
            "completed_cases": len(group),
            "valid_cases": len(group) - setup_invalid_count,
            "pass_count": pass_count,
            "pass_rate": round(pass_count / valid_expected, 4) if valid_expected else None,
            "stable_result": "INCOMPLETE" if setup_invalid_count else ("STABLE_PASS" if pass_count == expected and len(group) == expected else (
                "UNSTABLE" if pass_count else "FAIL"
            )),
            **{f"count_{result.lower()}": counts.get(result, 0) for result in RESULTS},
            "min_clearance_m": min(clearances) if clearances else None,
            "median_clearance_m": median(clearances) if clearances else None,
            "median_time_to_cross_sec": median(cross_times) if cross_times else None,
            "median_time_to_goal_sec": median(goal_times) if goal_times else None,
            "setup_invalid_count": setup_invalid_count,
            "failure_reasons": json.dumps(
                dict(Counter(
                    row.get("failure_reason") or row.get("early_stop_reason") or row.get("result")
                    for row in group if row.get("result") != "PASS"
                )), ensure_ascii=False, sort_keys=True,
            ),
        })
    return aggregate_rows


def write_aggregate(root: Path, rows: list[dict], expected_repetitions: int | None) -> Path:
    aggregate_rows = aggregate(rows, expected_repetitions)
    path = root / "aggregate.csv"
    fields = list(aggregate_rows[0]) if aggregate_rows else [
        "longitudinal_m", "intrusion_m", "shoulder", "expected_repetitions",
        "completed_cases", "pass_count", "pass_rate", "stable_result",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(aggregate_rows)
    return path


def write_report(
    root: Path,
    rows: list[dict],
    distances: list[float] | None,
    intrusions: list[float] | None,
    shoulder: str | None,
    repetitions: int | None,
) -> Path:
    matrix_rows = [row for row in rows if not row.get("baseline_without_obstacle")]
    controls = [row for row in rows if row.get("baseline_without_obstacle")]
    distances = distances or sorted({_float(row.get("longitudinal_m")) for row in matrix_rows if _float(row.get("longitudinal_m")) is not None})
    intrusions = intrusions or sorted({_float(row.get("intrusion_m")) for row in matrix_rows if _float(row.get("intrusion_m")) is not None})
    side = shoulder or (matrix_rows[0].get("shoulder") if matrix_rows else "right")
    expected = repetitions or max((int(row.get("repetition", 1)) for row in matrix_rows), default=1)
    grouped = defaultdict(list)
    for row in matrix_rows:
        grouped[(_float(row.get("longitudinal_m")), _float(row.get("intrusion_m")), row.get("shoulder", side))].append(row)

    pass_cases = sum(row.get("result") == "PASS" for row in matrix_rows)
    baseline_pass_cases = sum(row.get("result") == "PASS" for row in controls)
    report = root / "summary.md"
    lines = [
        "# Obstacle Stop 二维绕障测试汇总", "",
        f"- 参数点：{len(distances)} × {len(intrusions)}；每点期望重复：{expected}",
        f"- 障碍物案例 PASS：{pass_cases}/{len(matrix_rows)}；无障碍基线 PASS：{baseline_pass_cases}/{len(controls)}", "",
        "## 通过率矩阵（右侧侵入量）", "",
        "| 纵向位置 \\ 侵入量 | " + " | ".join(f"{value:g} m" for value in intrusions) + " |",
        "|---:|" + "---:|" * len(intrusions),
    ]
    for distance in distances:
        cells = []
        for intrusion in intrusions:
            group = grouped.get((distance, intrusion, side), [])
            passed = sum(row.get("result") == "PASS" for row in group)
            denominator = expected
            setup_invalid = sum(row.get("result") == "SETUP_INVALID" for row in group)
            if setup_invalid:
                cells.append(f"{passed}/{denominator} (SETUP_INVALID×{setup_invalid})")
            else:
                cells.append(f"{passed}/{denominator} ({passed / denominator:.0%})" if denominator else "-")
        lines.append(f"| {distance:g} m | " + " | ".join(cells) + " |")

    lines += ["", "## 失败原因矩阵", "", "| 纵向位置 \\ 侵入量 | " + " | ".join(f"{value:g} m" for value in intrusions) + " |", "|---:|" + "---:|" * len(intrusions)]
    for distance in distances:
        cells = []
        for intrusion in intrusions:
            group = grouped.get((distance, intrusion, side), [])
            failures = Counter(row.get("result", "SETUP_INVALID") for row in group if row.get("result") != "PASS")
            cells.append(", ".join(f"{name}×{count}" for name, count in failures.most_common()) or "-")
        lines.append(f"| {distance:g} m | " + " | ".join(cells) + " |")

    lines += ["", "## 无障碍基线", "", "| 案例 | 重复 | 结果 | 原因 |", "|---|---:|---|---|"]
    for row in sorted(controls, key=lambda item: int(item.get("repetition", 1))):
        lines.append(f"| {row.get('case_id', '')} | {row.get('repetition', '')} | {row.get('result', '')} | {row.get('failure_reason') or row.get('early_stop_reason') or ''} |")

    weakest = sorted(
        aggregate(matrix_rows, repetitions),
        key=lambda row: (
            float(row["pass_rate"]) if row.get("pass_rate") is not None else -1.0,
            float(row.get("longitudinal_m", 0)),
            float(row.get("intrusion_m", 0)),
        ),
    )
    if weakest:
        row = weakest[0]
        lines += ["", f"最薄弱参数点：纵向 {row['longitudinal_m']:g} m、侵入 {row['intrusion_m']:g} m，{row['pass_count']}/{row['expected_repetitions']} 通过。"]
    report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--distances", nargs="+", type=float, default=None)
    parser.add_argument("--intrusions", nargs="+", type=float, default=None)
    parser.add_argument("--shoulder", choices=("left", "right"), default=None)
    parser.add_argument("--repetitions", type=int, default=None)
    args = parser.parse_args()
    rows = load_rows(args.root)
    rows = expected_rows(args.root, rows, args.distances, args.intrusions, args.shoulder, args.repetitions)
    write_cases(args.root, rows)
    write_aggregate(args.root, rows, args.repetitions)
    write_report(args.root, rows, args.distances, args.intrusions, args.shoulder, args.repetitions)
    matrix_expected = len(args.distances) * len(args.intrusions) * args.repetitions if args.distances and args.intrusions and args.repetitions else len([row for row in rows if not row.get("baseline_without_obstacle")])
    baseline_expected = args.repetitions if args.repetitions and any(row.get("baseline_without_obstacle") for row in rows) else len([row for row in rows if row.get("baseline_without_obstacle")])
    summary = {
        "expected_cases": matrix_expected + baseline_expected,
        "completed_cases": len(rows),
        "passed_cases": sum(row.get("result") == "PASS" for row in rows),
        "obstacle_failures": sum(
            row.get("result") not in ("PASS", "SETUP_INVALID")
            and not row.get("baseline_without_obstacle")
            for row in rows
        ),
        "setup_invalid_cases": sum(row.get("result") == "SETUP_INVALID" for row in rows),
        "failed_cases": sum(row.get("result") not in ("PASS", "SETUP_INVALID") for row in rows),
        "results": rows,
    }
    (args.root / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")
    print(args.root / "cases.csv")
    print(args.root / "aggregate.csv")
    print(args.root / "summary.md")
    # Setup-invalid cases are not behavior failures, but return nonzero so the
    # matrix caller knows those points must be rerun.
    return 0 if summary["failed_cases"] == 0 and summary["setup_invalid_cases"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
