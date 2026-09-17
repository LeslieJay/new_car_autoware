#!/usr/bin/env python3
"""Compare steering demand through the planning and control pipeline.

The script is intentionally read-only: it opens a rosbag in READ_ONLY mode and
writes a Markdown report.  Curvature is converted to an equivalent front-wheel
angle with ``atan(wheel_base * curvature)`` so path geometry and control
commands use the same unit.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence


WHEEL_BASE_M = 1.008
DEFAULT_WINDOWS = (
    ("11:16:48-11:17:02", 1789355808.0, 1789355822.0),
    ("11:12:02-11:12:04", 1789355522.0, 1789355524.0),
)

PATH_TOPICS = {
    "/planning/path_candidate/simple_avoidance": "simple_avoidance_candidate",
    "/planning/path_reference/simple_avoidance": "simple_avoidance_reference",
    "/planning/scenario_planning/lane_driving/behavior_planning/path": "behavior_path",
    "/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id": "behavior_path_with_lane_id",
    "/planning/scenario_planning/lane_driving/motion_planning/elastic_band_smoother/output/traj": "elastic_band_trajectory",
    "/planning/scenario_planning/scenario_selector/trajectory": "scenario_trajectory",
    "/planning/trajectory": "final_trajectory",
}
CONTROL_TOPICS = {
    "/control/command/control_cmd": "control_command",
    "/control/trajectory_follower/control_cmd": "trajectory_follower_command",
    "/system/emergency/control_cmd": "emergency_control_command",
    "/vehicle/status/steering_status": "steering_feedback",
}
DEBUG_TOPIC = "/control/trajectory_follower/controller_node_exe/debug/ld_outputs"
DEFAULT_MAX_STEERING_ANGLE_RAD = 0.65


@dataclass
class TopicSummary:
    name: str
    message_count: int = 0
    empty_count: int = 0
    maximum_angle_rad: float | None = None
    maximum_angle_time: float | None = None
    maximum_curvature: float | None = None
    maximum_curvature_time: float | None = None
    raw_angle_rad: float | None = None
    clamped_angle_rad: float | None = None
    debug_lookahead_m: float | None = None
    debug_lateral_error_m: float | None = None
    samples: list[tuple[float, float]] = field(default_factory=list)

    def update_angle(self, timestamp: float, angle_rad: float) -> None:
        if not math.isfinite(angle_rad):
            return
        self.samples.append((timestamp, angle_rad))
        if self.maximum_angle_rad is None or abs(angle_rad) > abs(self.maximum_angle_rad):
            self.maximum_angle_rad = angle_rad
            self.maximum_angle_time = timestamp


def curvature_to_steering_angle(curvature: float, wheel_base_m: float = WHEEL_BASE_M) -> float:
    """Convert signed path curvature [1/m] to signed tire angle [rad]."""

    return math.atan(wheel_base_m * curvature)


def point_xy(point: object) -> tuple[float, float]:
    if hasattr(point, "point"):
        point = point.point
    pose = point.pose
    return float(pose.position.x), float(pose.position.y)


def maximum_path_curvature(
    points: Sequence[object], wheel_base_m: float = WHEEL_BASE_M
) -> tuple[float | None, float | None, int | None]:
    """Return (signed angle, absolute curvature, index) for a path."""

    if len(points) < 3:
        return None, None, None
    maximum: tuple[float, float, int] | None = None
    for index in range(1, len(points) - 1):
        ax, ay = point_xy(points[index - 1])
        bx, by = point_xy(points[index])
        cx, cy = point_xy(points[index + 1])
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        if min(ab, bc) < 0.05 or ac < 0.1:
            continue
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        curvature = 2.0 * cross / (ab * bc * ac)
        if not math.isfinite(curvature):
            continue
        angle = curvature_to_steering_angle(curvature, wheel_base_m)
        if maximum is None or abs(curvature) > maximum[1]:
            maximum = (angle, abs(curvature), index)
    return maximum if maximum is not None else (None, None, None)


def _format_angle(angle_rad: float | None) -> str:
    if angle_rad is None:
        return "N/A"
    return f"{angle_rad:.6f} rad ({math.degrees(angle_rad):.3f} deg)"


def _format_time(timestamp: float | None) -> str:
    return "N/A" if timestamp is None else f"{timestamp:.6f}"


def analyze_window(
    bag_path: Path,
    start_time: float,
    end_time: float,
    wheel_base_m: float = WHEEL_BASE_M,
) -> dict[str, TopicSummary]:
    """Read one time window from a rosbag and return per-topic summaries."""

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    topics = {**PATH_TOPICS, **CONTROL_TOPICS, DEBUG_TOPIC: "pure_pursuit_debug"}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    available = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    selected = [topic for topic in topics if topic in available]
    summaries = {label: TopicSummary(label) for label in topics.values()}
    if not selected:
        return summaries

    classes = {topic: get_message(available[topic]) for topic in selected}
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    reader.seek(int(start_time * 1e9))
    end_ns = int(end_time * 1e9)

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if timestamp_ns > end_ns:
            break
        timestamp = timestamp_ns / 1e9
        message = deserialize_message(data, classes[topic])
        if topic in PATH_TOPICS:
            summary = summaries[topics[topic]]
            summary.message_count += 1
            points = list(message.points)
            if not points:
                summary.empty_count += 1
                continue
            angle, curvature, _ = maximum_path_curvature(points, wheel_base_m)
            if angle is not None:
                summary.update_angle(timestamp, angle)
                if summary.maximum_curvature is None or abs(curvature) > abs(summary.maximum_curvature):
                    summary.maximum_curvature = curvature
                    summary.maximum_curvature_time = timestamp
            if hasattr(points[0], "front_wheel_angle_rad"):
                for point in points:
                    summary.update_angle(timestamp, float(point.front_wheel_angle_rad))
            continue

        if topic in CONTROL_TOPICS:
            summary = summaries[topics[topic]]
            summary.message_count += 1
            if topic == "/vehicle/status/steering_status":
                summary.update_angle(timestamp, float(message.steering_tire_angle))
            else:
                summary.update_angle(timestamp, float(message.lateral.steering_tire_angle))
            continue

        summary = summaries["pure_pursuit_debug"]
        summary.message_count += 1
        if len(message.data) >= 7:
            curvature = float(message.data[4])
            raw_angle = curvature_to_steering_angle(curvature, wheel_base_m)
            summary.raw_angle_rad = (
                raw_angle
                if summary.raw_angle_rad is None or abs(raw_angle) > abs(summary.raw_angle_rad)
                else summary.raw_angle_rad
            )
            summary.debug_lookahead_m = float(message.data[3])
            summary.debug_lateral_error_m = float(message.data[5])
            summary.update_angle(timestamp, raw_angle)

    return summaries


def render_report(
    bag_path: Path,
    windows: Iterable[tuple[str, float, float]],
    wheel_base_m: float = WHEEL_BASE_M,
    max_steering_angle_rad: float = DEFAULT_MAX_STEERING_ANGLE_RAD,
) -> str:
    lines = [
        "# Avoidance steering diagnosis",
        "",
        f"- bag: `{bag_path}`",
        f"- wheel base: `{wheel_base_m:.3f} m`",
        "- conversion: `front_wheel_angle = atan(wheel_base * curvature)`",
        f"- configured Pure Pursuit steering limit: `{max_steering_angle_rad:.6f} rad ({math.degrees(max_steering_angle_rad):.3f} deg)`",
        "- this report is read-only and does not imply that a rejected candidate was executable.",
        "- an empty `simple_avoidance_candidate` topic means no candidate was published; it does not expose the rejected in-memory candidate, which requires `[DEBUG-SA-STEER]` runtime logging.",
        "",
    ]
    for name, start, end in windows:
        summaries = analyze_window(bag_path, start, end, wheel_base_m)
        lines.extend([f"## Window {name}", "", "| stage | messages | empty | max angle | max angle time | max curvature |", "|---|---:|---:|---|---:|---|"])
        for label, summary in summaries.items():
            if summary.message_count == 0:
                continue
            curvature = "N/A" if summary.maximum_curvature is None else f"{summary.maximum_curvature:.6f} 1/m"
            lines.append(
                f"| {label} | {summary.message_count} | {summary.empty_count} | "
                f"{_format_angle(summary.maximum_angle_rad)} | {_format_time(summary.maximum_angle_time)} | {curvature} |"
            )
        pp = summaries["pure_pursuit_debug"]
        follower = summaries["trajectory_follower_command"]
        control = summaries["control_command"]

        def reached_limit(summary: TopicSummary) -> str:
            if summary.maximum_angle_rad is None:
                return "N/A"
            return "YES" if abs(summary.maximum_angle_rad) >= max_steering_angle_rad - 1.0e-6 else "NO"

        lines.extend(
            [
                "",
                "Pure Pursuit debug:",
                f"- debug current-trajectory curvature steering peak: `{_format_angle(pp.raw_angle_rad)}`",
                "- `ld_outputs.data[4]` is the current trajectory curvature; it is not Pure Pursuit's pre-clamp target command.",
                f"- latest sampled lookahead: `{pp.debug_lookahead_m if pp.debug_lookahead_m is not None else 'N/A'}` m",
                f"- latest sampled lateral error: `{pp.debug_lateral_error_m if pp.debug_lateral_error_m is not None else 'N/A'}` m",
                f"- trajectory follower command peak: `{_format_angle(follower.maximum_angle_rad)}`; at configured limit: `{reached_limit(follower)}`",
                f"- final control command peak: `{_format_angle(control.maximum_angle_rad)}`; at configured limit: `{reached_limit(control)}`",
                "- the command and Pure Pursuit debug peaks are reported independently because a command may be replaced by another downstream source.",
                "",
            ]
        )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    parser.add_argument("--wheel-base", type=float, default=WHEEL_BASE_M)
    parser.add_argument("--max-steering-angle", type=float, default=DEFAULT_MAX_STEERING_ANGLE_RAD)
    parser.add_argument("--start", type=float, help="single window start in bag epoch seconds")
    parser.add_argument("--end", type=float, help="single window end in bag epoch seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (args.start is None) != (args.end is None):
        raise SystemExit("--start and --end must be provided together")
    windows = (
        ((f"{args.start:.3f}-{args.end:.3f}", args.start, args.end),)
        if args.start is not None
        else DEFAULT_WINDOWS
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        render_report(args.bag, windows, args.wheel_base, args.max_steering_angle), encoding="utf-8"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
