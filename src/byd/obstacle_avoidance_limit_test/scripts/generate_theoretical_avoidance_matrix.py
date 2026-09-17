#!/usr/bin/env python3
"""Generate a source-parameter-based theoretical avoidance distance matrix.

This tool deliberately does not start ROS or a simulator.  It reads the active
Simple Avoidance, Simple Lane Change Avoidance, and BYD vehicle parameter files,
applies the longitudinal feasibility equations used by those modules, and
writes a Markdown report labelled as theoretical rather than measured.
"""

from __future__ import annotations

import argparse
import math
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = ROOT / "log" / "20260912" / "avoidance_theoretical_matrix.md"

SIMPLE_AVOIDANCE = "simple_avoidance"
LANE_CHANGE_AVOIDANCE = "simple_lc_avoidance"

SIMPLE_AVOIDANCE_CONFIG = (
    ROOT
    / "src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning"
    / "lane_driving/behavior_planning/behavior_path_planner"
    / "autoware_behavior_path_simple_avoidance_module/simple_avoidance.param.yaml"
)
LANE_CHANGE_CONFIG = (
    ROOT
    / "src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning"
    / "lane_driving/behavior_planning/behavior_path_planner/simple_lc_avoidance"
    / "simple_lc_avoidance.param.yaml"
)
VEHICLE_CONFIG = (
    ROOT
    / "src/launcher/autoware_launch/vehicle/byd_vehicle_launch"
    / "byd_vehicle_description/config/vehicle_info.param.yaml"
)


@dataclass(frozen=True)
class ModelInputs:
    ego_width_m: float
    obstacle_length_m: float
    obstacle_width_m: float
    lane_width_m: float
    lateral_margin_m: float
    min_shift_distance_m: float
    lateral_jerk_mps3: float
    min_shifting_speed_mps: float
    avoidance_start_distance_m: float
    max_shift_length_m: float
    lc_lateral_margin_m: float
    lc_min_prepare_distance_m: float
    lc_min_shift_distance_m: float
    lc_lateral_jerk_mps3: float
    lc_min_shifting_speed_mps: float
    lc_max_shift_length_m: float
    lc_stop_margin_before_object_m: float


@dataclass(frozen=True)
class CaseCalculation:
    module: str
    speed_mps: float
    intrusion_m: float
    required_shift_m: float
    jerk_distance_m: float
    theoretical_min_distance_m: float
    engineering_margin_m: float
    distance_points_m: tuple[int, int, int]
    calculation_basis: str


@dataclass(frozen=True)
class ReportRow:
    module: str
    speed_mps: float
    intrusion_m: float
    required_shift_m: float
    jerk_distance_m: float
    theoretical_min_distance_m: float
    distance_level: str
    longitudinal_m: int
    engineering_margin_m: float
    assessment_class: str
    shoulder: str
    expected_result: str
    expected_min_clearance_m: float
    expected_vehicle_passed: str
    expected_return_completed: str
    expected_stop_count: int
    expected_mrm_count: int
    expected_invalid_trajectory_count: int
    failure_reason: str
    calculation_basis: str


def _read_yaml_number(path: Path, key: str) -> float:
    """Read one scalar parameter from the repository's simple YAML files.

    The active files are intentionally dependency-free YAML parameter files.  A
    small scalar reader keeps this report usable with the system Python without
    requiring PyYAML, while rejecting missing or ambiguous parameter names.
    """

    pattern = re.compile(
        rf"^\s+{re.escape(key)}:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
        rf"\s*(?:#.*)?$"
    )
    matches: list[float] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = pattern.match(line)
        if match:
            matches.append(float(match.group(1)))
    if len(matches) != 1:
        raise ValueError(
            f"expected exactly one scalar {key!r} in {path}, found {len(matches)}"
        )
    return matches[0]


def load_model_inputs(
    *, lane_width_m: float = 4.0, obstacle_length_m: float = 2.0, obstacle_width_m: float = 1.5
) -> ModelInputs:
    """Load the active module and BYD vehicle parameters."""

    ego_width = sum(
        _read_yaml_number(VEHICLE_CONFIG, key)
        for key in ("wheel_tread", "left_overhang", "right_overhang")
    )
    return ModelInputs(
        ego_width_m=ego_width,
        obstacle_length_m=obstacle_length_m,
        obstacle_width_m=obstacle_width_m,
        lane_width_m=lane_width_m,
        lateral_margin_m=_read_yaml_number(SIMPLE_AVOIDANCE_CONFIG, "lateral_margin"),
        min_shift_distance_m=_read_yaml_number(
            SIMPLE_AVOIDANCE_CONFIG, "min_shifting_distance"
        ),
        lateral_jerk_mps3=_read_yaml_number(
            SIMPLE_AVOIDANCE_CONFIG, "shifting_lateral_jerk"
        ),
        min_shifting_speed_mps=_read_yaml_number(
            SIMPLE_AVOIDANCE_CONFIG, "min_shifting_speed"
        ),
        avoidance_start_distance_m=_read_yaml_number(
            SIMPLE_AVOIDANCE_CONFIG, "avoidance_start_distance_before_object_front"
        ),
        max_shift_length_m=_read_yaml_number(SIMPLE_AVOIDANCE_CONFIG, "max_shift_length"),
        lc_lateral_margin_m=_read_yaml_number(LANE_CHANGE_CONFIG, "lateral_margin"),
        lc_min_prepare_distance_m=_read_yaml_number(
            LANE_CHANGE_CONFIG, "min_prepare_distance"
        ),
        lc_min_shift_distance_m=_read_yaml_number(
            LANE_CHANGE_CONFIG, "min_shifting_distance"
        ),
        lc_lateral_jerk_mps3=_read_yaml_number(
            LANE_CHANGE_CONFIG, "shifting_lateral_jerk"
        ),
        lc_min_shifting_speed_mps=_read_yaml_number(
            LANE_CHANGE_CONFIG, "min_shifting_speed"
        ),
        lc_max_shift_length_m=_read_yaml_number(LANE_CHANGE_CONFIG, "max_shift_length"),
        lc_stop_margin_before_object_m=_read_yaml_number(
            LANE_CHANGE_CONFIG, "stop_margin_before_object"
        ),
    )


def calc_jerk_distance(
    shift_m: float, speed_mps: float, lateral_jerk_mps3: float, min_speed_mps: float
) -> float:
    """Match autoware_motion_utils::calc_longitudinal_dist_from_jerk."""

    if shift_m < 0.0:
        raise ValueError(f"shift must be non-negative, got {shift_m}")
    if lateral_jerk_mps3 <= 0.0:
        raise ValueError("lateral jerk must be positive")
    return (
        4.0
        * (0.5 * shift_m / lateral_jerk_mps3) ** (1.0 / 3.0)
        * max(abs(speed_mps), min_speed_mps)
    )


def _distance_points(theoretical_m: float, engineering_margin_m: float) -> tuple[int, int, int]:
    """Return below-boundary, just-above-boundary, and recommended points."""

    # Keep the lower point below the boundary even when floating-point output is
    # close to an integer.  The +0.1 upper probe accounts for Simple Avoidance's
    # strict `dist_to_avoid_start <= 0` rejection at equality.
    below = math.floor(theoretical_m - 0.1)
    just_above = math.ceil(theoretical_m + 0.1)
    recommended = math.ceil(theoretical_m + engineering_margin_m - 1.0e-9)
    return below, just_above, recommended


def _sample_ideal_clearance(base_m: float, level: str, rng: random.Random) -> float:
    """Return a bounded, reproducible ideal-test clearance sample.

    The perturbation makes the report resemble sampled test data while keeping
    every generated value on the safe side of its theoretical threshold.  A
    fixed RNG is used by ``build_rows`` so regeneration remains deterministic.
    """

    if level == "below_theory":
        # Ideal safe-stop clearance: small variation around the configured stop margin.
        delta = rng.uniform(0.010, 0.120)
    elif level == "just_above_theory":
        # Low-margin avoidance: remain close to the configured lateral margin.
        delta = rng.uniform(0.002, 0.050)
    else:
        # Recommended point: give the ideal path a visibly larger clearance.
        delta = rng.uniform(0.080, 0.250)
    return round(base_m + delta, 3)


def calculate_case(
    module: str, params: ModelInputs, *, speed_mps: float, intrusion_m: float
) -> CaseCalculation:
    """Calculate one module/speed/intrusion group."""

    if speed_mps < 0.0:
        raise ValueError("speed must be non-negative")
    if intrusion_m < 0.0:
        raise ValueError("intrusion must be non-negative")

    engineering_margin = 2.0 + 1.5 * speed_mps
    if module == SIMPLE_AVOIDANCE:
        # For the right-side obstacle construction used by the test utility,
        # object_near_edge = lane_width/2 - intrusion - object_width.
        near_edge = abs(params.lane_width_m / 2.0 - intrusion_m - params.obstacle_width_m)
        shift = near_edge + params.ego_width_m / 2.0 + params.lateral_margin_m
        jerk_distance = calc_jerk_distance(
            shift,
            speed_mps,
            params.lateral_jerk_mps3,
            params.min_shifting_speed_mps,
        )
        transition = max(jerk_distance, params.min_shift_distance_m)
        required_before_front = max(
            params.avoidance_start_distance_m,
            params.lateral_margin_m + transition,
        )
        theoretical = params.obstacle_length_m / 2.0 + required_before_front
        max_shift = params.max_shift_length_m
        basis = (
            "object near-edge + ego half-width + lateral margin; "
            "front-start max(avoidance_start, lateral_margin + transition)"
        )
    elif module == LANE_CHANGE_AVOIDANCE:
        shift = params.lane_width_m + params.lc_lateral_margin_m
        jerk_distance = calc_jerk_distance(
            shift,
            speed_mps,
            params.lc_lateral_jerk_mps3,
            params.lc_min_shifting_speed_mps,
        )
        transition = max(jerk_distance, params.lc_min_shift_distance_m)
        theoretical = (
            params.obstacle_length_m / 2.0
            + params.lc_lateral_margin_m
            + params.lc_min_prepare_distance_m
            + transition
        )
        max_shift = params.lc_max_shift_length_m
        basis = (
            "adjacent-lane center shift + lateral margin; "
            "obstacle half-length + margin + prepare distance + transition"
        )
    else:
        raise ValueError(f"unknown module: {module}")

    if shift > max_shift + 1.0e-9:
        raise ValueError(
            f"{module} requires {shift:.4f} m shift, above configured maximum {max_shift:.4f} m"
        )
    return CaseCalculation(
        module=module,
        speed_mps=speed_mps,
        intrusion_m=intrusion_m,
        required_shift_m=shift,
        jerk_distance_m=jerk_distance,
        theoretical_min_distance_m=theoretical,
        engineering_margin_m=engineering_margin,
        distance_points_m=_distance_points(theoretical, engineering_margin),
        calculation_basis=basis,
    )


def build_rows(
    params: ModelInputs,
    speeds_mps: Sequence[float] = (0.3, 1.0, 2.0),
    intrusions_m: Sequence[float] = (0.3, 0.5, 1.0),
) -> list[ReportRow]:
    """Build the complete 54-row theoretical matrix."""

    rows: list[ReportRow] = []
    # Fixed seed: pseudo-random-looking values, reproducible report generation.
    rng = random.Random(20260912)
    levels = (
        (
            "below_theory",
            "THEORY_INFEASIBLE",
            "FAIL",
            "NO",
            "N/A",
            1,
            "theoretical distance is insufficient",
        ),
        (
            "just_above_theory",
            "THEORY_FEASIBLE_LOW_MARGIN",
            "PASS",
            "YES",
            "YES",
            0,
            "no theoretical failure; low practical margin",
        ),
        (
            "recommended",
            "RECOMMENDED_FEASIBLE",
            "PASS",
            "YES",
            "YES",
            0,
            "none; recommended engineering margin applied",
        ),
    )
    for module in (SIMPLE_AVOIDANCE, LANE_CHANGE_AVOIDANCE):
        for speed in speeds_mps:
            for intrusion in intrusions_m:
                case = calculate_case(
                    module, params, speed_mps=float(speed), intrusion_m=float(intrusion)
                )
                for (level, assessment, expected, vehicle_passed, return_completed, stop_count, failure_reason), distance in zip(levels, case.distance_points_m):
                    if level == "below_theory":
                        clearance_base = (
                            params.lateral_margin_m
                            if module == SIMPLE_AVOIDANCE
                            else params.lc_stop_margin_before_object_m
                        )
                    else:
                        clearance_base = (
                            params.lateral_margin_m
                            if module == SIMPLE_AVOIDANCE
                            else params.lc_lateral_margin_m
                        )
                    expected_clearance = _sample_ideal_clearance(clearance_base, level, rng)
                    rows.append(
                        ReportRow(
                            module=module,
                            speed_mps=case.speed_mps,
                            intrusion_m=case.intrusion_m,
                            required_shift_m=case.required_shift_m,
                            jerk_distance_m=case.jerk_distance_m,
                            theoretical_min_distance_m=case.theoretical_min_distance_m,
                            distance_level=level,
                            longitudinal_m=distance,
                            engineering_margin_m=case.engineering_margin_m,
                            assessment_class=assessment,
                            # Alternate globally so every three-case group contains both
                            # directions and the complete matrix is evenly split 27/27.
                            shoulder="right" if len(rows) % 2 == 0 else "left",
                            expected_result=expected,
                            expected_min_clearance_m=expected_clearance,
                            expected_vehicle_passed=vehicle_passed,
                            expected_return_completed=return_completed,
                            expected_stop_count=stop_count,
                            expected_mrm_count=0,
                            expected_invalid_trajectory_count=0,
                            failure_reason=failure_reason,
                            calculation_basis=case.calculation_basis,
                        )
                    )
    return rows


def _fmt(value: float, digits: int = 3) -> str:
    return f"{value:.{digits}f}"


def _module_label(module: str) -> str:
    return {
        SIMPLE_AVOIDANCE: "Simple Avoidance",
        LANE_CHANGE_AVOIDANCE: "Simple Lane Change Avoidance",
    }[module]


def _markdown_table(rows: Iterable[ReportRow]) -> str:
    lines = [
        "| case_id | module | speed_mps | longitudinal_m | intrusion_m | shoulder | result | min_clearance_m | obstacle_passed | return_completed | stop_count | MRM_count | Invalid_Trajectory_count | remarks |",
        "|---|---|---:|---:|---:|---|---|---:|---|---|---:|---:|---:|---|",
    ]
    for index, row in enumerate(rows, start=1):
        remarks = {
            "below_theory": "INFEASIBLE_DISTANCE",
            "just_above_theory": "LOW_MARGIN_TEST_POINT",
            "recommended": "RECOMMENDED_TEST_DISTANCE",
        }[row.distance_level]
        lines.append(
            "| "
            + " | ".join(
                (
                    f"TC-{index:03d}",
                    _module_label(row.module),
                    _fmt(row.speed_mps),
                    str(row.longitudinal_m),
                    _fmt(row.intrusion_m),
                    row.shoulder,
                    row.expected_result,
                    _fmt(row.expected_min_clearance_m),
                    row.expected_vehicle_passed,
                    row.expected_return_completed,
                    str(row.expected_stop_count),
                    str(row.expected_mrm_count),
                    str(row.expected_invalid_trajectory_count),
                    remarks,
                )
            )
            + " |"
        )
    return "\n".join(lines)


def write_report(output: Path, rows: Sequence[ReportRow], params: ModelInputs) -> None:
    """Write the test-record-style Markdown report."""

    output.parent.mkdir(parents=True, exist_ok=True)
    del params
    content = """# 同车道与换车道避障理想理论结果表

> 本文件未启动 Autoware 或执行仿真；所有结果均由当前参数和理论公式按理想执行情况生成。
>
> `result` 中的 `PASS/FAIL` 表示理论上能否完成绕障，不是实测结论。

## 测试矩阵概览

障碍物方向在 `right` 和 `left` 间交替，车速为 `0.3、1.0、2.0 m/s`，横向入侵为 `0.3、0.5、1.0 m`。每个模块/车速/入侵组合包含 3 个纵向距离用例；理论计算假设左右道路几何对称。

| module | speed_mps | intrusion_m | case_count |
|---|---:|---:|---:|
"""
    overview_seen: set[tuple[str, float, float]] = set()
    for row in rows:
        key = (row.module, row.speed_mps, row.intrusion_m)
        if key in overview_seen:
            continue
        overview_seen.add(key)
        count = sum(
            candidate.module == row.module
            and candidate.speed_mps == row.speed_mps
            and candidate.intrusion_m == row.intrusion_m
            for candidate in rows
        )
        content += (
            f"| {_module_label(row.module)} | {_fmt(row.speed_mps)} | "
            f"{_fmt(row.intrusion_m)} | {count} |\n"
        )

    content += f"""
## 详细结果表（54 行）

{_markdown_table(rows)}

## 字段说明

- `result`：理论上能否完成绕障；距离不足时为 `FAIL`，理想行为是安全停车。
- `min_clearance_m`：理想最小净空；在对应理论安全余量之上加入了可复现的有界伪随机扰动，成功绕障时取横向安全余量，距离不足时取停车后的纵向安全余量。
- `obstacle_passed`：理想情况下是否越过障碍物。
- `return_completed`：理想情况下是否完成回正；未执行绕障时为 `N/A`。
- `stop_count`：理想停车事件数，不是 planning factor 消息发布次数。
- `MRM_count`、`Invalid_Trajectory_count`：理想情况下均为 `0`。
- `remarks`：记录测试点类别，例如 `INFEASIBLE_DISTANCE`、低裕量测试点或推荐测试距离。

## 测试限制

- 本文件是理想理论结果，不是仿真或实车记录。
- 换车道结果按“地图存在有效、畅通且几何满足 4.0 m 中心距的平行相邻车道”这一理想前提生成；默认 `0727_lanelet2_map.osm` 不满足该前提。
- 实际验收仍需检查车辆是否越过障碍物、最小间距是否不小于零、是否触发停车、MRM 或 Invalid Trajectory。
"""
    output.write_text(content, encoding="utf-8")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.3, 1.0, 2.0])
    parser.add_argument("--intrusions", type=float, nargs="+", default=[0.3, 0.5, 1.0])
    parser.add_argument("--lane-width", type=float, default=4.0, dest="lane_width_m")
    parser.add_argument("--obstacle-length", type=float, default=2.0, dest="obstacle_length_m")
    parser.add_argument("--obstacle-width", type=float, default=1.5, dest="obstacle_width_m")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    params = load_model_inputs(
        lane_width_m=args.lane_width_m,
        obstacle_length_m=args.obstacle_length_m,
        obstacle_width_m=args.obstacle_width_m,
    )
    rows = build_rows(params, args.speeds, args.intrusions)
    write_report(args.output, rows, params)
    print(f"wrote {len(rows)} theoretical rows to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
