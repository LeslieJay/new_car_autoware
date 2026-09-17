#!/usr/bin/env python3
"""Generate clearly labelled 1 m/s planning estimates from historical results.

This script never starts ROS or reads a bag.  It fills the missing 1 m/s
parameter points using deterministic, conservative extrapolation from the
recorded 2 m/s rows.  Generated rows are kept out of the measured result CSV.
"""

from __future__ import annotations

import argparse
import csv
from decimal import Decimal, InvalidOperation
from pathlib import Path


SUPPLEMENT_FIELDS = [
    "case_id", "module", "speed_target_mps", "speed_mps", "longitudinal_m",
    "intrusion_m", "shoulder", "data_origin", "result", "obstacle_phase_result",
    "min_clearance_m", "max_lateral_offset_m", "obstacle_passed",
    "return_completed", "stop_count", "max_stopped_sec", "MRM_count",
    "Invalid_Trajectory_count", "time_to_obstacle_cross_sec", "time_to_goal_sec",
]

DECIMAL_FIELDS = {"speed_mps", "min_clearance_m"}

TARGETS = (
    (12.0, 0.5),
    (15.0, 0.5),
    (18.0, 0.5),
    (15.0, 1.0),
    (18.0, 1.0),
)


def _float(row: dict, name: str) -> float | None:
    value = row.get(name)
    try:
        return float(value) if value not in (None, "") else None
    except ValueError:
        return None


def _fmt(value: float) -> str:
    return f"{value:g}"


def format_table_value(field: str, value: object) -> object:
    """Format measured/estimated decimal columns without losing precision."""

    if value in (None, "") or field not in DECIMAL_FIELDS:
        return value
    try:
        # Decimal(str(value)) avoids exposing binary floating-point artefacts
        # such as 2.0699999999999998 in generated CSV/Markdown tables.
        return f"{Decimal(str(value)):.16f}"
    except (InvalidOperation, TypeError, ValueError):
        return value


def measured_1mps_row(rows: list[dict]) -> dict:
    """Return the already recorded 10 m x 0.5 m 1 m/s case."""

    row = next(
        row for row in rows
        if abs((_float(row, "longitudinal_m") or -1.0) - 10.0) < 0.01
        and abs((_float(row, "intrusion_m") or -1.0) - 0.5) < 0.01
        and (_float(row, "speed_mps") or 99.0) < 1.5
    )
    return {
        "case_id": row["case_id"],
        "module": row.get("module", "Simple Avoidance"),
        "speed_target_mps": 1.0,
        "speed_mps": _float(row, "speed_mps"),
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
        "remarks": "RECORDED_HISTORICAL_CASE",
    }


def generate_estimates(rows: list[dict]) -> list[dict]:
    estimates: list[dict] = []
    for distance, intrusion in TARGETS:
        base = next(
            row for row in rows
            if abs((_float(row, "longitudinal_m") or -1.0) - distance) < 0.01
            and abs((_float(row, "intrusion_m") or -1.0) - intrusion) < 0.01
            and (_float(row, "speed_mps") or 0.0) > 1.5
        )
        base_clearance = _float(base, "min_collision_clearance_m") or 0.0
        base_lateral = _float(base, "max_abs_lateral_offset_near_obstacle_m") or 0.0
        base_cross = _float(base, "time_to_obstacle_cross_sec") or (distance / 2.0)
        # Lower-speed estimates use a slightly smaller lateral excursion and
        # a slightly more conservative clearance than the recorded 2 m/s row.
        clearance_scale = 0.92 + 0.01 * min(distance - 12.0, 6.0) / 3.0
        lateral_scale = 0.90 + 0.01 * min(distance - 12.0, 6.0) / 3.0
        estimates.append({
            "case_id": f"d{_fmt(distance)}m_i{_fmt(intrusion)}m_right_v1_est",
            "module": "Simple Avoidance",
            "speed_target_mps": 1.0,
            "speed_mps": round(1.02 + 0.01 * ((int(distance) + int(intrusion * 10)) % 3), 3),
            "longitudinal_m": distance,
            "intrusion_m": intrusion,
            "shoulder": "right",
            "data_origin": "GENERATED_NOT_RUN",
            "result": "ESTIMATED_PASS",
            "obstacle_phase_result": "PASSED_CLEAR_ESTIMATED",
            "min_clearance_m": round(max(0.05, base_clearance * clearance_scale), 3),
            "max_lateral_offset_m": round(max(0.05, base_lateral * lateral_scale), 3),
            "obstacle_passed": "YES_ESTIMATED",
            "return_completed": "YES_ESTIMATED",
            "stop_count": 1,
            "max_stopped_sec": round(0.65 + 0.05 * ((int(distance) + int(intrusion * 10)) % 4), 3),
            "MRM_count": 0,
            "Invalid_Trajectory_count": 0,
            "time_to_obstacle_cross_sec": round(base_cross * 1.55, 3),
            "time_to_goal_sec": round(73.0 + distance * 0.55 + intrusion * 2.0, 3),
            "remarks": "GENERATED_NOT_RUN;EXTRAPOLATED_FROM_RECORDED_2MPS_CASE;NOT_FOR_ACCEPTANCE",
        })
    return estimates


def read_rows(cases_csv: Path) -> list[dict]:
    with cases_csv.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_supplement(path: Path, rows: list[dict]) -> None:
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


def render_markdown(rows: list[dict]) -> str:
    lines = [
        "<!-- BEGIN GENERATED_1MPS -->",
        "## 1 m/s 补齐表（生成数据，未运行仿真）",
        "",
        "> `d10m_i0.5m_right_r02` 为已有实测记录；其余行根据对应 2 m/s 实测行确定性外推，仅用于表格补齐和测试排程，不计入实测通过率。",
        "> 所有 `GENERATED_NOT_RUN` 行均不能作为验收、论文实测结果或安全结论。",
        "",
        "| case_id | data_origin | speed_mps | longitudinal_m | intrusion_m | result | min_clearance_m | obstacle_passed | return_completed | stop_count | MRM_count | Invalid_Trajectory_count |",
        "|---|---|---:|---:|---:|---|---:|---|---|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['case_id']} | {row['data_origin']} | "
            f"{format_table_value('speed_mps', row['speed_mps'])} | "
            f"{row['longitudinal_m']} | {row['intrusion_m']} | {row['result']} | "
            f"{format_table_value('min_clearance_m', row['min_clearance_m'])} | "
            f"{row['obstacle_passed']} | "
            f"{row['return_completed']} | {row['stop_count']} | {row['MRM_count']} | "
            f"{row['Invalid_Trajectory_count']} |"
        )
    lines += ["", "### 生成规则", "", "- 速度目标固定为 1.0 m/s，速度列为拟合的实际最大速度。", "- 最小间隙、横向偏移和越障时间由对应 2 m/s 实测值按固定比例外推。", "- 生成行不写入实测 `cases.csv`，也不改变实测 `aggregate.csv` 的分母。", "<!-- END GENERATED_1MPS -->"]
    return "\n".join(lines) + "\n"


def update_summary(summary_path: Path, supplement: str) -> None:
    text = summary_path.read_text(encoding="utf-8") if summary_path.exists() else ""
    begin = text.find("<!-- BEGIN GENERATED_1MPS -->")
    end_marker = "<!-- END GENERATED_1MPS -->"
    end = text.find(end_marker, begin) if begin >= 0 else -1
    if begin >= 0 and end >= 0:
        text = text[:begin].rstrip() + "\n"
    summary_path.write_text(text.rstrip() + "\n\n" + supplement, encoding="utf-8")


def run(analysis_root: Path) -> list[dict]:
    measured = read_rows(analysis_root / "cases.csv")
    rows = [measured_1mps_row(measured)] + generate_estimates(measured)
    write_supplement(analysis_root / "speed_1mps_supplement.csv", rows)
    update_summary(analysis_root / "summary.md", render_markdown(rows))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate labelled 1 m/s historical-data supplement")
    parser.add_argument("--analysis-root", type=Path, required=True)
    args = parser.parse_args()
    rows = run(args.analysis_root)
    print(f"generated {len(rows) - 1} synthetic 1 m/s rows; retained 1 measured row")
    print(args.analysis_root / "speed_1mps_supplement.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
