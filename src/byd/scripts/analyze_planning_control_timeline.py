#!/usr/bin/env python3
"""Create a compact planning -> validator -> control timeline from a ROS 2 bag."""

from __future__ import annotations

import argparse
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/localization/kinematic_state",
    "/planning/scenario_planning/trajectory",
    "/planning/trajectory",
    "/planning/planning_validator/validation_status",
    "/planning/planning_validator/virtual_wall",
    "/planning/planning_factors/simple_avoidance",
    "/planning/planning_factors/simple_lane_change_avoidance",
    "/planning/planning_factors/obstacle_stop",
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/virtual_wall/simple_lane_change_avoidance",
    "/control/trajectory_follower/control_cmd",
    "/control/command/control_cmd",
    "/control/control_validator/validation_status",
    "/control/control_validator/virtual_wall",
    "/vehicle/status/velocity_status",
}


def stamp_seconds(stamp: Any, fallback: float) -> float:
    if stamp is None:
        return fallback
    value = float(stamp.sec) + float(stamp.nanosec) / 1e9
    return value if value > 0.0 else fallback


def message_stamp(message: Any, fallback: float) -> float:
    if hasattr(message, "header") and hasattr(message.header, "stamp"):
        return stamp_seconds(message.header.stamp, fallback)
    return stamp_seconds(getattr(message, "stamp", None), fallback)


def trajectory_min_acceleration(message: Any) -> float | None:
    values = [
        float(point.acceleration_mps2)
        for point in getattr(message, "points", [])
        if math.isfinite(float(point.acceleration_mps2))
    ]
    return min(values) if values else None


def marker_position(message: Any) -> tuple[float, float] | None:
    for marker in getattr(message, "markers", []):
        position = marker.pose.position
        if math.isfinite(position.x) and math.isfinite(position.y):
            return float(position.x), float(position.y)
    return None


def parse_log(path: Path | None) -> list[dict[str, Any]]:
    if path is None or not path.is_file():
        return []
    patterns = [
        ("target_locked", re.compile(r"active target locked .*?lon=([+-]?\d+(?:\.\d+)?)m")),
        ("avoidance_stop", re.compile(r"stop before target .*?reason=([^ ]+).*?target_lon=([+-]?\d+(?:\.\d+)?)")),
        ("invalid_trajectory_soft_stop", re.compile(r"Invalid Trajectory detected\. Use soft stop trajectory")),
        ("invalid_trajectory_published", re.compile(r"Caution! Invalid Trajectory published")),
        ("stale_control_command", re.compile(r"Control command is stale: (\d+) ms")),
        ("mrm", re.compile(r"(EMERGENCY_STOP|COMFORTABLE_STOP) is (operated|canceled)")),
    ]
    events = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        stamp_match = re.search(r"\[(\d+\.\d+)\]", line)
        if not stamp_match:
            continue
        for kind, pattern in patterns:
            match = pattern.search(line)
            if not match:
                continue
            event: dict[str, Any] = {
                "time": float(stamp_match.group(1)),
                "kind": kind,
                "source": "log",
                "line": line_number,
            }
            if kind == "target_locked":
                event["target_lon_m"] = float(match.group(1))
            elif kind == "avoidance_stop":
                event.update(reason=match.group(1), target_lon_m=float(match.group(2)))
            elif kind == "stale_control_command":
                event["age_ms"] = int(match.group(1))
            elif kind == "mrm":
                event.update(stop_type=match.group(1), action=match.group(2))
            events.append(event)
            break
    compact: list[dict[str, Any]] = []
    last_seen: dict[tuple[str, str], float] = {}
    for event in events:
        discriminator = str(event.get("reason", event.get("action", "")))
        key = (event["kind"], discriminator)
        window = 120.0 if event["kind"] == "avoidance_stop" else 5.0
        if event["kind"] != "target_locked" and event["time"] - last_seen.get(key, -math.inf) < window:
            continue
        compact.append(event)
        last_seen[key] = event["time"]
    return compact


def analyze_bag(path: Path) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_names = {item.name: item.type for item in reader.get_all_topics_and_types()}
    selected = sorted(TOPICS & type_names.keys())
    reader.set_filter(rosbag2_py.StorageFilter(topics=selected))
    types = {topic: get_message(type_names[topic]) for topic in selected}
    events: list[dict[str, Any]] = []
    counts: Counter[str] = Counter()
    minima: dict[str, float] = {}
    first_time: float | None = None
    first_markers: dict[str, tuple[float, float, float]] = {}
    first_kinds: set[tuple[str, str]] = set()

    def add_once(topic: str, kind: str, stamp: float, **values: Any):
        key = (topic, kind)
        if key in first_kinds:
            return
        first_kinds.add(key)
        events.append({"time": stamp, "kind": kind, "source": topic, **values})

    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        bag_stamp = bag_ns / 1e9
        first_time = bag_stamp if first_time is None else min(first_time, bag_stamp)
        counts[topic] += 1
        message = deserialize_message(serialized, types[topic])
        # Use receipt time for cross-node ordering. During replay message headers retain
        # source-bag time while launch logs use replay wall time; mixing them produces
        # meaningless negative offsets.
        stamp = bag_stamp
        if "trajectory" in topic and hasattr(message, "points"):
            minimum = trajectory_min_acceleration(message)
            if minimum is not None:
                minima[topic] = min(minima.get(topic, minimum), minimum)
                if minimum < -1.5:
                    add_once(topic, "trajectory_below_validator_floor", stamp, min_acceleration=minimum)
        elif topic == "/planning/planning_validator/validation_status":
            invalid = [
                name for name in message.get_fields_and_field_types()
                if name.startswith("is_valid_") and getattr(message, name) is False
            ]
            if invalid:
                add_once(
                    topic, "planning_validator_rejection", stamp,
                    failed_checks=invalid,
                    min_longitudinal_acc=float(message.min_longitudinal_acc),
                    invalid_count=int(message.invalid_count),
                )
            if not message.is_valid_longitudinal_min_acc:
                add_once(
                    topic, "planning_validator_min_acc_rejection", stamp,
                    min_longitudinal_acc=float(message.min_longitudinal_acc),
                    invalid_count=int(message.invalid_count),
                )
        elif "planning_factors" in topic:
            stop_factors = [factor for factor in message.factors if int(factor.behavior) == 3]
            if stop_factors:
                factor = stop_factors[0]
                point = factor.control_points[0] if factor.control_points else None
                add_once(
                    topic, "planning_stop_factor", stamp,
                    module=str(factor.module), detail=str(factor.detail),
                    stop_x=float(point.pose.position.x) if point else None,
                    stop_y=float(point.pose.position.y) if point else None,
                    distance=float(point.distance) if point else None,
                )
        elif topic in {"/control/trajectory_follower/control_cmd", "/control/command/control_cmd"}:
            acceleration = float(message.longitudinal.acceleration)
            minima[topic] = min(minima.get(topic, acceleration), acceleration)
            if acceleration < -0.1:
                add_once(topic, "control_brake_request", stamp, acceleration=acceleration,
                         velocity=float(message.longitudinal.velocity))
        elif "virtual_wall" in topic:
            position = marker_position(message)
            if position is not None:
                first_markers.setdefault(topic, (stamp, *position))
                add_once(topic, "virtual_wall", stamp, x=position[0], y=position[1])

    marker_shift = None
    source_wall = first_markers.get(
        "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/virtual_wall/simple_lane_change_avoidance"
    )
    validator_wall = first_markers.get("/planning/planning_validator/virtual_wall")
    if source_wall and validator_wall:
        marker_shift = math.hypot(source_wall[1] - validator_wall[1], source_wall[2] - validator_wall[2])
    return events, {
        "bag": str(path),
        "first_selected_message_time": first_time,
        "selected_topics": selected,
        "topic_counts": dict(counts),
        "minima": minima,
        "first_virtual_walls": {key: {"time": value[0], "x": value[1], "y": value[2]} for key, value in first_markers.items()},
        "first_source_to_validator_wall_distance_m": marker_shift,
    }


def render_markdown(events: list[dict[str, Any]], summary: dict[str, Any]) -> str:
    origin = summary.get("first_selected_message_time")
    lines = [
        "# Planning-control event timeline", "",
        f"- Bag: `{summary['bag']}`",
        f"- Selected topics present: {len(summary['selected_topics'])}/{len(TOPICS)}",
        f"- Source/validator first-wall separation: `{summary['first_source_to_validator_wall_distance_m']}` m",
        "- This is an open-loop evidence timeline; it does not model vehicle response.", "",
        "| Time | Offset | Event | Source | Details |", "|---:|---:|---|---|---|",
    ]
    for event in sorted(events, key=lambda item: (item["time"], item["kind"])):
        offset = event["time"] - origin if origin is not None else 0.0
        details = ", ".join(
            f"{key}={value}" for key, value in event.items()
            if key not in {"time", "kind", "source"}
        ).replace("|", "\\|")
        lines.append(f"| {event['time']:.6f} | {offset:.3f}s | {event['kind']} | `{event['source']}` | {details} |")
    lines.extend(["", "## Topic counts", ""])
    for topic, count in sorted(summary["topic_counts"].items()):
        lines.append(f"- `{topic}`: {count}")
    lines.extend(["", "## Observed minima", ""])
    for topic, value in sorted(summary["minima"].items()):
        lines.append(f"- `{topic}`: {value}")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--json", type=Path, dest="json_path")
    parser.add_argument(
        "--require-runtime-chain", action="store_true",
        help="fail unless planning trajectory, validator status, and control command messages exist",
    )
    args = parser.parse_args()
    events, summary = analyze_bag(args.bag)
    events.extend(parse_log(args.log))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_markdown(events, summary), encoding="utf-8")
    if args.json_path:
        args.json_path.parent.mkdir(parents=True, exist_ok=True)
        args.json_path.write_text(
            json.dumps({"summary": summary, "events": sorted(events, key=lambda item: item["time"])}, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    missing = TOPICS - set(summary["selected_topics"])
    if missing:
        print(f"missing {len(missing)} requested topics: {sorted(missing)}")
    print(f"wrote {args.output} with {len(events)} events")
    if args.require_runtime_chain:
        required = {
            "/planning/trajectory",
            "/planning/planning_validator/validation_status",
            "/control/trajectory_follower/control_cmd",
            "/control/command/control_cmd",
        }
        empty = sorted(topic for topic in required if summary["topic_counts"].get(topic, 0) == 0)
        if empty:
            print(f"runtime chain incomplete; zero messages on: {empty}")
            return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
