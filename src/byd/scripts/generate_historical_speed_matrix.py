#!/usr/bin/env python3
"""Create a labelled 0.3/1.0/2.0 m/s historical speed matrix.

The 0.3 m/s and missing 1.0 m/s rows are deterministic estimates.  No ROS
process is started and the generated rows are never mixed into measured-case
aggregates.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from generate_historical_1mps_supplement import (
    SUPPLEMENT_FIELDS,
    _float,
    _fmt,
    format_table_value,
    generate_estimates,
    measured_1mps_row,
    read_rows,
    write_supplement,
)


ALL_TARGETS = (
    (10.0, 0.5),
    (12.0, 0.5),
    (15.0, 0.5),
    (18.0, 0.5),
    (15.0, 1.0),
    (18.0, 1.0),
)


def generate_03mps_estimates(rows: list[dict]) -> list[dict]:
    estimates: list[dict] = []
    for distance, intrusion in ALL_TARGETS:
        base = next(
            row for row in rows
            if abs((_float(row, "longitudinal_m") or -1.0) - distance) < 0.01
            and abs((_float(row, "intrusion_m") or -1.0) - intrusion) < 0.01
            and (_float(row, "speed_mps") or 0.0) > 1.5
        )
        clearance = _float(base, "min_collision_clearance_m") or 0.0
        lateral = _float(base, "max_abs_lateral_offset_near_obstacle_m") or 0.0
        cross = _float(base, "time_to_obstacle_cross_sec") or (distance / 2.0)
        clearance_scale = 0.86 + 0.02 * min(distance - 10.0, 8.0) / 4.0
        lateral_scale = 0.82 + 0.02 * min(distance - 10.0, 8.0) / 4.0
        estimates.append({
            "case_id": f"d{_fmt(distance)}m_i{_fmt(intrusion)}m_right_v0.3_est",
            "module": "Simple Avoidance",
            "speed_target_mps": 0.3,
            "speed_mps": round(0.31 + 0.01 * ((int(distance) + int(intrusion * 10)) % 3), 3),
            "longitudinal_m": distance,
            "intrusion_m": intrusion,
            "shoulder": "right",
            "data_origin": "GENERATED_NOT_RUN",
            "result": "ESTIMATED_PASS",
            "obstacle_phase_result": "PASSED_CLEAR_ESTIMATED",
            "min_clearance_m": round(max(0.05, clearance * clearance_scale), 3),
            "max_lateral_offset_m": round(max(0.05, lateral * lateral_scale), 3),
            "obstacle_passed": "YES_ESTIMATED",
            "return_completed": "YES_ESTIMATED",
            "stop_count": 1,
            "max_stopped_sec": round(0.85 + 0.05 * ((int(distance) + int(intrusion * 10)) % 4), 3),
            "MRM_count": 0,
            "Invalid_Trajectory_count": 0,
            "time_to_obstacle_cross_sec": round(cross * 2.75, 3),
            "time_to_goal_sec": round(111.0 + distance * 0.8 + intrusion * 3.0, 3),
            "remarks": "GENERATED_NOT_RUN;EXTRAPOLATED_FROM_RECORDED_2MPS_CASE;NOT_FOR_ACCEPTANCE",
        })
    return estimates


def normalize_measured_row(row: dict) -> dict:
    speed = _float(row, "speed_mps")
    return {
        "case_id": row.get("case_id"),
        "module": row.get("module", "Simple Avoidance"),
        "speed_target_mps": 1.0 if speed is not None and speed < 1.5 else 2.0,
        "speed_mps": speed,
        "longitudinal_m": _float(row, "longitudinal_m"),
        "intrusion_m": _float(row, "intrusion_m"),
        "shoulder": row.get("shoulder", "right"),
        "data_origin": "MEASURED",
        "result": row.get("overall_result", row.get("result")),
        "obstacle_phase_result": row.get("obstacle_phase_result"),
        "min_clearance_m": _float(row, "min_collision_clearance_m"),
        "max_lateral_offset_m": _float(row, "max_abs_lateral_offset_near_obstacle_m"),
        "obstacle_passed": row.get("obstacle_passed", "YES"),
        "return_completed": row.get("return_completed", "UNKNOWN_NOT_RECORDED"),
        "stop_count": row.get("stop_count"),
        "max_stopped_sec": _float(row, "max_stopped_sec"),
        "MRM_count": row.get("mrm_count"),
        "Invalid_Trajectory_count": row.get("invalid_trajectory_count"),
        "time_to_obstacle_cross_sec": _float(row, "time_to_obstacle_cross_sec"),
        "time_to_goal_sec": _float(row, "time_to_goal_sec"),
        "remarks": row.get("remarks", "RECORDED_HISTORICAL_CASE"),
    }


def combined_rows(measured: list[dict]) -> list[dict]:
    one_mps = [measured_1mps_row(measured)] + generate_estimates(measured)
    zero_mps = generate_03mps_estimates(measured)
    normalized_measured = [normalize_measured_row(row) for row in measured]
    combined = normalized_measured + one_mps[1:] + zero_mps
    return sorted(
        combined,
        key=lambda row: (
            float(row.get("speed_target_mps") or 0.0),
            float(row.get("longitudinal_m") or 0.0),
            float(row.get("intrusion_m") or 0.0),
            row.get("case_id", ""),
        ),
    )


def write_combined(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUPPLEMENT_FIELDS)
        writer.writeheader()
        writer.writerows(
            {
                field: format_table_value(field, row.get(field))
                for field in SUPPLEMENT_FIELDS
            }
            for row in rows
        )


def render_combined(rows: list[dict]) -> str:
    lines = [
        "<!-- BEGIN SPEED_MATRIX -->",
        "## 0.3 / 1.0 / 2.0 m/s 多速度条件汇总表",
        "",
        "> 表中 `MEASURED` 为已有 bag/log 实测；`GENERATED_NOT_RUN` 为根据 2 m/s 实测结果外推的补齐数据，未启动仿真，不计入实测通过率。",
        "> `speed_target_mps` 是速度条件；`speed_mps` 是实测或估计的最大速度。生成行只用于排程和格式补齐。",
        "",
        "| case_id | data_origin | speed_target_mps | speed_mps | longitudinal_m | intrusion_m | shoulder | result | obstacle_phase_result | min_clearance_m | obstacle_passed | return_completed | stop_count | MRM_count | Invalid_Trajectory_count |",
        "|---|---|---:|---:|---:|---:|---|---|---|---:|---|---|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['case_id']} | {row['data_origin']} | {row['speed_target_mps']} | "
            f"{format_table_value('speed_mps', row['speed_mps']) if row['speed_mps'] is not None else 'UNKNOWN_NOT_RECORDED'} | "
            f"{row['longitudinal_m']} | {row['intrusion_m']} | {row['shoulder']} | {row['result']} | "
            f"{row['obstacle_phase_result']} | "
            f"{format_table_value('min_clearance_m', row['min_clearance_m'])} | {row['obstacle_passed']} | "
            f"{row['return_completed']} | {row['stop_count']} | {row['MRM_count']} | "
            f"{row['Invalid_Trajectory_count']} |"
        )
    lines += [
        "",
        "### 口径说明",
        "",
        "- 2.0 m/s 条件包含已有的 6 个参数点；10 m × 0.5 m 有两次历史实测，因此合并表保留两行。",
        "- 1.0 m/s 条件包含 10 m × 0.5 m 的 1 次实测，以及其余 5 个参数点的生成估计。",
        "- 0.3 m/s 条件的 6 个参数点均为生成估计。",
        "- 生成数据不能替代真实仿真验收，最终报告仍应以 `cases.csv` 和 `aggregate.csv` 为准。",
        "<!-- END SPEED_MATRIX -->",
    ]
    return "\n".join(lines) + "\n"


def update_summary(summary_path: Path, section: str) -> None:
    text = summary_path.read_text(encoding="utf-8") if summary_path.exists() else ""
    for begin_marker, end_marker in (
        ("<!-- BEGIN GENERATED_1MPS -->", "<!-- END GENERATED_1MPS -->"),
        ("<!-- BEGIN SPEED_MATRIX -->", "<!-- END SPEED_MATRIX -->"),
    ):
        begin = text.find(begin_marker)
        end = text.find(end_marker, begin) if begin >= 0 else -1
        if begin >= 0 and end >= 0:
            text = text[:begin].rstrip() + "\n"
    summary_path.write_text(text.rstrip() + "\n\n" + section, encoding="utf-8")


def run(analysis_root: Path) -> list[dict]:
    measured = read_rows(analysis_root / "cases.csv")
    one_mps = [measured_1mps_row(measured)] + generate_estimates(measured)
    combined = combined_rows(measured)
    zero_rows = [row for row in combined if row["speed_target_mps"] == 0.3]
    write_supplement(analysis_root / "speed_1mps_supplement.csv", one_mps)
    write_supplement(analysis_root / "speed_0.3mps_supplement.csv", zero_rows)
    write_combined(analysis_root / "speed_conditions_combined.csv", combined)
    update_summary(analysis_root / "summary.md", render_combined(combined))
    return combined


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate labelled multi-speed historical matrix")
    parser.add_argument("--analysis-root", type=Path, required=True)
    args = parser.parse_args()
    rows = run(args.analysis_root)
    print(f"combined {len(rows)} rows across 0.3/1.0/2.0 m/s")
    print(args.analysis_root / "speed_0.3mps_supplement.csv")
    print(args.analysis_root / "speed_conditions_combined.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
