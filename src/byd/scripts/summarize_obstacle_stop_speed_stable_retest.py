#!/usr/bin/env python3
"""Aggregate stable-speed retest repetitions and causal controls."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def load_case(case: Path, distance: float, label: str) -> dict:
    result = read_json(case / "result.json")
    monitor = read_json(case / "monitor.json")
    timeline = read_json(case / "timeline.json")
    cls = timeline.get("classification", {})
    uuid_topics = cls.get("perception_uuid_continuity", {})
    uuid_stable = all(
        details.get("uuid_stable") is True
        for details in uuid_topics.values()
        if details.get("max_object_count", 0) > 0
    ) if uuid_topics else None
    perception_gaps = cls.get("perception_max_gap_s", {})
    bpp_timing = cls.get("bpp_timing", [])
    scene_timing = cls.get("bpp_scene_timing", [])
    row = {
        "label": label,
        "case_dir": str(case),
        "distance_m": distance,
        "result": result.get("result", "INCOMPLETE"),
        "monitor_result": monitor.get("monitor_result"),
        "obstacle_crossed": monitor.get("obstacle_crossed", cls.get("obstacle_crossed", False)),
        "goal_reached": monitor.get("goal_reached", cls.get("goal_reached", False)),
        "route_state": monitor.get("route_state"),
        "placement_valid": monitor.get("placement", {}).get("placement_valid") if isinstance(monitor.get("placement"), dict) else None,
        "placement_speed_mps": monitor.get("placement", {}).get("speed_mps") if isinstance(monitor.get("placement"), dict) else None,
        "actual_ahead_m": monitor.get("placement", {}).get("actual_ahead_m") if isinstance(monitor.get("placement"), dict) else None,
        "first_commit_time": cls.get("first_commit_time"),
        "first_target_expiry_time": cls.get("first_target_expiry_time"),
        "first_obstacle_stop_factor_time": cls.get("first_obstacle_stop_factor_time"),
        "target_expiry_count": cls.get("target_expiry_count"),
        "candidate_commitment_count": cls.get("candidate_commitment_count"),
        "trajectory_timeout_count": cls.get("trajectory_timeout_count"),
        "max_reported_trajectory_delay_s": cls.get("max_reported_trajectory_delay_s"),
        "trajectory_gap_after_monitor_s": cls.get("trajectory_gap_after_monitor_s"),
        "min_collision_clearance_m": result.get("min_collision_clearance_m"),
        "module_configuration_ok": result.get("module_configuration_ok"),
        "obstacle_stop_module_loaded": result.get("obstacle_stop_module_loaded"),
        "dynamic_obstacle_stop_module_loaded": result.get("dynamic_obstacle_stop_module_loaded"),
        "has_simple_avoidance": result.get("has_simple_avoidance"),
        "has_obstacle_stop": result.get("has_obstacle_stop"),
        "perception_uuid_stable": uuid_stable,
        "max_perception_gap_s": max(perception_gaps.values(), default=None),
        "max_bpp_scene_run_ms": max((float(event.get("run_ms", 0.0)) for event in scene_timing), default=None),
        "max_bpp_planner_run_ms": max((float(event.get("planner_run_ms", 0.0)) for event in bpp_timing), default=None),
        "early_stop_reason": monitor.get("early_stop_reason", result.get("early_stop_reason")),
        "bag_path": str(case / "rosbag"),
        "planner_cycles": cls.get("planner_cycles", []),
        "perception_max_gap_s": cls.get("perception_max_gap_s", {}),
    }
    if row["obstacle_crossed"] and row["goal_reached"] and row["result"] == "PASS":
        row["acceptance"] = "PASS"
    elif row["obstacle_crossed"] and row["goal_reached"]:
        row["acceptance"] = "CROSSED_AND_ARRIVED_FAIL"
    elif row["obstacle_crossed"]:
        row["acceptance"] = "CROSSED_NOT_ARRIVED"
    else:
        row["acceptance"] = row["result"]
    return row


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("--distances", default="10,12")
    args = parser.parse_args()
    distances = [float(value) for value in args.distances.split(",")]
    rows: list[dict] = []
    default_roots = sorted(args.root.glob("repetition_*/"))
    for repeat_root in default_roots:
        for distance in distances:
            case = repeat_root / f"{distance:g}m"
            if case.exists():
                rows.append(load_case(case, distance, "stable_speed_default"))
    for label in ("control_target_hold_8s", "control_commitment_5m"):
        for case in sorted((args.root / label).glob("repetition_*/")):
            for distance in [12.0]:
                dcase = case / f"{distance:g}m"
                if dcase.exists():
                    rows.append(load_case(dcase, distance, label))
    rows.sort(key=lambda row: (row["label"], row["distance_m"], row["case_dir"]))
    fields = [
        "label", "case_dir", "distance_m", "acceptance", "result", "monitor_result",
        "obstacle_crossed", "goal_reached", "route_state", "placement_valid",
        "placement_speed_mps", "actual_ahead_m", "first_commit_time",
        "first_target_expiry_time", "first_obstacle_stop_factor_time",
        "target_expiry_count", "candidate_commitment_count", "trajectory_timeout_count",
        "max_reported_trajectory_delay_s", "trajectory_gap_after_monitor_s",
        "min_collision_clearance_m", "module_configuration_ok", "obstacle_stop_module_loaded",
        "dynamic_obstacle_stop_module_loaded", "has_simple_avoidance", "has_obstacle_stop",
        "perception_uuid_stable", "max_perception_gap_s", "max_bpp_scene_run_ms",
        "max_bpp_planner_run_ms", "early_stop_reason", "bag_path",
    ]
    with (args.root / "diagnosis_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)

    # Flatten the planner cycles for the requested 10 m success/failure
    # comparison.  Each row retains the commitment distance, the path distance
    # generated in that cycle, and the monotonic timing gap that preceded it.
    comparison_fields = [
        "label", "distance_m", "case_dir", "acceptance", "obstacle_crossed", "goal_reached",
        "cycle", "commitment_distance_to_shift_start_m", "commitment_state", "committed",
        "path_dist_to_avoid_start_m", "path_required_before_front_m", "commitment_path_order",
        "path_commitment_distance_delta_m", "timing_entry_gap_ms",
    ]
    comparison_rows = []
    for row in rows:
        if row["label"] != "stable_speed_default" or row["distance_m"] != 10.0:
            continue
        cycles = row.get("planner_cycles") or [{}]
        for cycle in cycles:
            comparison_rows.append({
                "label": row["label"],
                "distance_m": row["distance_m"],
                "case_dir": row["case_dir"],
                "acceptance": row["acceptance"],
                "obstacle_crossed": row["obstacle_crossed"],
                "goal_reached": row["goal_reached"],
                **{field: cycle.get(field) for field in comparison_fields[6:]},
            })
    with (args.root / "cycle_comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=comparison_fields)
        writer.writeheader()
        writer.writerows(comparison_rows)

    by_label: dict[str, list[dict]] = {}
    for row in rows:
        by_label.setdefault(row["label"], []).append(row)
    repeatability = {}
    for label, label_rows in by_label.items():
        repeatability[label] = {}
        for distance in sorted({row["distance_m"] for row in label_rows}):
            selected = [row for row in label_rows if row["distance_m"] == distance]
            outcomes = [row["acceptance"] for row in selected]
            repeatability[label][str(distance)] = {
                "runs": len(selected),
                "outcomes": outcomes,
                "deterministic": len(set(outcomes)) <= 1,
                "cross_rate": sum(bool(row["obstacle_crossed"]) for row in selected) / len(selected) if selected else 0.0,
                "arrival_rate": sum(bool(row["goal_reached"]) for row in selected) / len(selected) if selected else 0.0,
            }
    summary = {
        "overall_result": "PASS" if rows and all(row["acceptance"] == "PASS" for row in rows if row["label"] == "stable_speed_default") else "FAIL",
        "formal_runs": sum(row["label"] == "stable_speed_default" for row in rows),
        "rows": rows,
        "cycle_comparison_path": str(args.root / "cycle_comparison.csv"),
        "repeatability": repeatability,
        "interpretation": {
            "crossed_not_arrived_is_not_pass": True,
            "controls_are_not_formal_results": True,
            "causal_signal": "target_hold_or_commitment_control_changes 12m outcome while perception remains continuous",
        },
    }
    (args.root / "diagnosis_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = ["# Stable-speed Obstacle Stop diagnosis", "", f"Formal runs: {summary['formal_runs']}", f"Overall: {summary['overall_result']}", "", "| Label | Distance | Acceptance | Crossed | Arrived | Commit | Expiry | Stop |", "|---|---:|---|---|---|---|---|---|"]
    for row in rows:
        lines.append(f"| {row['label']} | {row['distance_m']:g} | {row['acceptance']} | {row['obstacle_crossed']} | {row['goal_reached']} | {row['first_commit_time'] or ''} | {row['first_target_expiry_time'] or ''} | {row['first_obstacle_stop_factor_time'] or ''} |")
    lines.extend([
        "",
        "## 关键观测",
        "",
        "| Label | Distance | Modules OK | Obstacle Stop | UUID stable | Max perception gap (s) | Max BPP scene run (ms) | Max BPP planner run (ms) | Min clearance (m) |",
        "|---|---:|---|---|---|---:|---:|---:|---:|",
    ])
    for row in rows:
        lines.append(
            f"| {row['label']} | {row['distance_m']:g} | {row.get('module_configuration_ok')} | "
            f"{row.get('has_obstacle_stop')} | {row.get('perception_uuid_stable')} | "
            f"{row.get('max_perception_gap_s')} | {row.get('max_bpp_scene_run_ms')} | "
            f"{row.get('max_bpp_planner_run_ms')} | {row.get('min_collision_clearance_m')} |"
        )
    lines.extend([
        "",
        "## 解释",
        "",
        "- `CROSSED_NOT_ARRIVED` 表示越过障碍物但未收到 `RouteState.ARRIVED`；不计为 PASS。",
        "- `CROSSED_AND_ARRIVED_FAIL` 表示虽到达终点，但存在碰撞包络、Invalid Trajectory、MRM 或其他验收失败证据。",
        "- 对照组只用于因果诊断，不纳入 `overall_result`。",
    ])
    (args.root / "diagnosis_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(args.root / "diagnosis_summary.md")
    return 0 if summary["overall_result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
