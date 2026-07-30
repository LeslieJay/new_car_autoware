#!/usr/bin/env python3
"""根据任务点 YAML 和实际到点 CSV 生成 AGV 到点精度 Markdown 报告。

报告不输出章节标题，但保留七列表头、对齐行和数据行。
"""

from __future__ import annotations

import csv
import math
import sys
from datetime import datetime
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

try:
    import yaml
except ImportError as exc:  # pragma: no cover - 仅在运行环境缺依赖时触发
    raise SystemExit(
        "缺少 PyYAML，请先安装：python3 -m pip install PyYAML"
    ) from exc

# 固定配置：脚本无需传入命令行参数。
MISSION_POINTS_YAML = Path(
    "/home/byd/weicanming/github_projects/new_car_autoware/"
    "src/byd/mission_publish_ws/src/mission_loop/config/mission_points.yaml"
)
LOG_ROOT = Path(
    "/home/byd/weicanming/github_projects/new_car_autoware/log"
)
OUTPUT_FILENAME = "localization_acc.md"
POINTS_PER_CYCLE = 4


@dataclass(frozen=True)
class Pose2D:
    """用于到点精度计算的二维位姿。"""

    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class AccuracyRow:
    """单次到点误差结果。"""

    index: int
    target: Pose2D
    actual: Pose2D
    error_x: float
    error_y: float
    planar_error: float
    heading_error_deg: float


def quaternion_to_yaw_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """将四元数转换为绕 Z 轴的航向角，单位为度。"""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("四元数长度为 0，无法计算航向角")

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(sin_yaw, cos_yaw))


def normalize_angle_deg(angle_deg: float) -> float:
    """将角度归一化到 [-180, 180) 度。"""
    return (angle_deg + 180.0) % 360.0 - 180.0


def require_float(mapping: dict[str, Any], key: str, source: Path) -> float:
    """读取并校验浮点字段。"""
    if key not in mapping:
        raise KeyError(f"{source} 中缺少字段：{key}")
    try:
        return float(mapping[key])
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{source} 中字段 {key!r} 不是有效数字：{mapping[key]!r}") from exc


def load_target_poses(yaml_path: Path) -> dict[str, Pose2D]:
    """读取 mission_loop ROS 参数文件中的所有目标位姿。"""
    with yaml_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    try:
        params = document["mission_loop"]["ros__parameters"]
    except (TypeError, KeyError) as exc:
        raise ValueError(
            f"{yaml_path} 格式不正确，期望包含 mission_loop.ros__parameters"
        ) from exc

    point_names = params.get("points")
    if not isinstance(point_names, list) or not point_names:
        raise ValueError(f"{yaml_path} 中 points 必须是非空列表")

    targets: dict[str, Pose2D] = {}
    for raw_name in point_names:
        name = str(raw_name)
        prefix = f"points.{name}."

        x = require_float(params, prefix + "x", yaml_path)
        y = require_float(params, prefix + "y", yaml_path)
        qx = require_float(params, prefix + "orientation_x", yaml_path)
        qy = require_float(params, prefix + "orientation_y", yaml_path)
        qz = require_float(params, prefix + "orientation_z", yaml_path)
        qw = require_float(params, prefix + "orientation_w", yaml_path)

        targets[name] = Pose2D(
            x=x,
            y=y,
            yaw_deg=quaternion_to_yaw_deg(qx, qy, qz, qw),
        )

    return targets


def load_actual_poses(csv_path: Path) -> list[tuple[str, Pose2D]]:
    """按 CSV 原始顺序读取所有实际到点位姿。"""
    required_columns = {"goal_name", "x", "y", "qx", "qy", "qz", "qw"}
    records: list[tuple[str, Pose2D]] = []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise ValueError(f"{csv_path} 没有 CSV 表头")

        missing = required_columns.difference(reader.fieldnames)
        if missing:
            raise ValueError(
                f"{csv_path} 缺少必要字段：{', '.join(sorted(missing))}"
            )

        for line_number, row in enumerate(reader, start=2):
            goal_name = (row.get("goal_name") or "").strip()
            if not goal_name:
                raise ValueError(f"{csv_path} 第 {line_number} 行 goal_name 为空")

            try:
                x = float(row["x"])
                y = float(row["y"])
                qx = float(row["qx"])
                qy = float(row["qy"])
                qz = float(row["qz"])
                qw = float(row["qw"])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"{csv_path} 第 {line_number} 行存在无效位姿数值"
                ) from exc

            records.append(
                (
                    goal_name,
                    Pose2D(
                        x=x,
                        y=y,
                        yaw_deg=quaternion_to_yaw_deg(qx, qy, qz, qw),
                    ),
                )
            )

    if not records:
        raise ValueError(f"{csv_path} 中没有实际到点记录")

    return records


def calculate_accuracy(
    targets: dict[str, Pose2D], actual_records: Iterable[tuple[str, Pose2D]]
) -> list[AccuracyRow]:
    """按 CSV 顺序计算误差，点位编号按 1～4 循环。"""
    results: list[AccuracyRow] = []

    for record_number, (goal_name, actual) in enumerate(actual_records, start=1):
        index = (record_number - 1) % POINTS_PER_CYCLE + 1
        if goal_name not in targets:
            raise KeyError(f"CSV 中的目标点 {goal_name!r} 未在 YAML points 中定义")

        target = targets[goal_name]
        error_x = actual.x - target.x
        error_y = actual.y - target.y
        planar_error = math.hypot(error_x, error_y)
        heading_error = normalize_angle_deg(actual.yaw_deg - target.yaw_deg)

        results.append(
            AccuracyRow(
                index=index,
                target=target,
                actual=actual,
                error_x=error_x,
                error_y=error_y,
                planar_error=planar_error,
                heading_error_deg=heading_error,
            )
        )

    return results


def format_signed(value: float) -> str:
    """按模板格式输出带正负号的四位小数。"""
    # 避免极小负数格式化为 -0.0000。
    if abs(value) < 0.00005:
        value = 0.0
    return f"{value:+.4f}"


def format_heading(value: float) -> str:
    """与位置误差保持一致，输出带正负号的四位小数。"""
    if abs(value) < 0.00005:
        value = 0.0
    return f"{value:+.4f}"


def render_markdown(rows: Iterable[AccuracyRow]) -> str:
    """输出七列表头、对齐行和数据行，不输出章节标题。"""
    lines: list[str] = [
        "| 点位 | 目标点 (X,Y) | 实际点 (X,Y) | X误差 (m) | Y误差 (m) | 平面误差 (m) | 航向误差 (°) |",
        "|:---:|:----------------------:|:----------------------:|---------:|---------:|-----------:|-----------:|",
    ]

    for row in rows:
        lines.append(
            "| {index} | ({tx:.4f}, {ty:.4f}) | ({ax:.4f}, {ay:.4f}) | "
            "{ex} | {ey} | {planar:.4f} | {heading} |".format(
                index=row.index,
                tx=row.target.x,
                ty=row.target.y,
                ax=row.actual.x,
                ay=row.actual.y,
                ex=format_signed(row.error_x),
                ey=format_signed(row.error_y),
                planar=row.planar_error,
                heading=format_heading(row.heading_error_deg),
            )
        )

    return "\n".join(lines) + "\n"


def resolve_today_paths(now: datetime | None = None) -> tuple[Path, Path, Path]:
    """确定当天 CSV 和报告输出路径。

    日志目录格式为 MMDD，例如 2026-07-28 对应 ``log/0728``；
    CSV 文件格式为 ``mission_arrivals_YYYYMMDD_HHMMSS.csv``。
    如果当天存在多份 CSV，则选择文件名时间戳最新的一份。
    """
    current_time = now or datetime.now()
    date_dir_name = current_time.strftime("%m%d")
    date_stamp = current_time.strftime("%Y%m%d")

    log_dir = LOG_ROOT / date_dir_name
    csv_pattern = f"mission_arrivals_{date_stamp}_*.csv"

    if not log_dir.is_dir():
        raise FileNotFoundError(f"当天日志目录不存在：{log_dir}")

    csv_candidates = [path for path in log_dir.glob(csv_pattern) if path.is_file()]
    if not csv_candidates:
        raise FileNotFoundError(
            f"当天日志目录中未找到 CSV：{log_dir / csv_pattern}"
        )

    # 文件名中的 YYYYMMDD_HHMMSS 使用固定宽度，按文件名排序即可得到最新时间。
    csv_path = max(csv_candidates, key=lambda path: path.name)
    output_path = log_dir / OUTPUT_FILENAME
    return MISSION_POINTS_YAML, csv_path, output_path


def main() -> int:
    try:
        yaml_path, csv_path, output_path = resolve_today_paths()

        if not yaml_path.is_file():
            raise FileNotFoundError(f"目标点 YAML 不存在：{yaml_path}")

        targets = load_target_poses(yaml_path)
        actual_records = load_actual_poses(csv_path)
        rows = calculate_accuracy(targets, actual_records)
        markdown = render_markdown(rows)

        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(markdown, encoding="utf-8")
    except (OSError, ValueError, KeyError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1

    print(f"目标点文件：{yaml_path}")
    print(f"实际点文件：{csv_path}")
    print(f"已生成报告：{output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
