#!/usr/bin/env python3
"""Analyze lateral-control commands, steering feedback, and PP debug data from a ROS 2 bag.

The analyzer deliberately opens SQLite rosbag files read-only so it also supports a bag directory
that has lost its metadata.yaml file.  It writes all results to a separate output directory.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import sqlite3
import statistics
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence


WHEEL_BASE_M = 1.008
MAX_STEERING_ANGLE_RAD = 0.65
GRID_STEP_S = 0.01
MAX_FEEDBACK_GAP_S = 0.20

TOPICS = {
    "follower_command": "/control/trajectory_follower/control_cmd",
    "final_command": "/control/command/control_cmd",
    "steering": "/vehicle/status/steering_status",
    "velocity": "/vehicle/status/velocity_status",
    "odometry": "/localization/kinematic_state",
    "acceleration": "/localization/acceleration",
    "trajectory": "/planning/trajectory",
    "pp_debug": "/control/trajectory_follower/controller_node_exe/debug/ld_outputs",
    "predicted_trajectory": "/control/trajectory_follower/lateral/predicted_trajectory",
    "control_mode": "/vehicle/status/control_mode",
    "gate_mode": "/control/current_gate_mode",
    "gear": "/vehicle/status/gear_status",
    "processing_time": "/control/trajectory_follower/controller_node_exe/lateral/debug/processing_time_ms",
}

PP_COLUMNS = (
    "velocity_lookahead_m",
    "curvature_lookahead_m",
    "lateral_error_lookahead_m",
    "lookahead_distance_m",
    "trajectory_curvature_1pm",
    "pp_lateral_error_m",
    "pp_velocity_mps",
)


@dataclass(frozen=True)
class Projection:
    lateral_error_m: float
    heading_error_rad: float
    along_track_m: float


@dataclass(frozen=True)
class DelayResult:
    delay_s: float
    rmse_rad: float
    correlation: float
    sample_count: int


@dataclass(frozen=True)
class CommandSample:
    time_s: float
    bag_time_s: float
    steering_rad: float
    velocity_mps: float


@dataclass(frozen=True)
class SteeringSample:
    time_s: float
    bag_time_s: float
    steering_rad: float


@dataclass(frozen=True)
class VelocitySample:
    time_s: float
    bag_time_s: float
    speed_mps: float
    heading_rate_radps: float
    lateral_velocity_mps: float


@dataclass(frozen=True)
class OdomSample:
    time_s: float
    bag_time_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float


@dataclass(frozen=True)
class ScalarSample:
    time_s: float
    bag_time_s: float
    value: float


@dataclass(frozen=True)
class TrajectorySample:
    time_s: float
    bag_time_s: float
    points: tuple[tuple[float, float, float], ...]


def resolve_bag_database(path: Path) -> Path:
    """Return the one SQLite database to read, without requiring metadata.yaml."""

    path = path.expanduser().resolve()
    if path.is_file():
        if path.suffix != ".db3":
            raise ValueError(f"bag database must end in .db3: {path}")
        return path
    if not path.is_dir():
        raise ValueError(f"bag path does not exist: {path}")
    databases = sorted(path.glob("*.db3"))
    if not databases:
        raise ValueError(f"no .db3 file found under {path}")
    if len(databases) > 1:
        raise ValueError(
            f"multiple bag segments found under {path}; pass a specific .db3 file instead"
        )
    return databases[0]


def wrap_angle(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def zero_order_hold(samples: Sequence[tuple[float, float]], grid: Sequence[float]) -> list[float]:
    """Sample a command signal with a causal zero-order hold."""

    if not samples:
        return [math.nan] * len(grid)
    values: list[float] = []
    sample_index = 0
    last = math.nan
    for timestamp in grid:
        while sample_index < len(samples) and samples[sample_index][0] <= timestamp:
            last = samples[sample_index][1]
            sample_index += 1
        values.append(last)
    return values


def linear_interpolate_with_gap_limit(
    samples: Sequence[tuple[float, float]], grid: Sequence[float], max_gap_s: float
) -> list[float]:
    """Interpolate feedback, leaving unavailable or stale regions as NaN."""

    if not samples:
        return [math.nan] * len(grid)
    times = [row[0] for row in samples]
    result: list[float] = []
    for timestamp in grid:
        index = bisect.bisect_left(times, timestamp)
        if index < len(times) and times[index] == timestamp:
            result.append(samples[index][1])
            continue
        if index == 0 or index == len(times):
            result.append(math.nan)
            continue
        before = samples[index - 1]
        after = samples[index]
        gap = after[0] - before[0]
        if gap <= 0.0 or gap > max_gap_s:
            result.append(math.nan)
            continue
        fraction = (timestamp - before[0]) / gap
        result.append(before[1] + fraction * (after[1] - before[1]))
    return result


def project_pose_to_trajectory(
    x_m: float, y_m: float, yaw_rad: float, trajectory: Sequence[tuple[float, float, float]]
) -> Projection:
    """Project a base_link pose onto a polyline; positive lateral error means path-left."""

    if len(trajectory) < 2:
        raise ValueError("trajectory needs at least two points")
    best: tuple[float, float, float, float, float] | None = None
    accumulated = 0.0
    for (ax, ay, ayaw), (bx, by, byaw) in zip(trajectory, trajectory[1:]):
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1.0e-12:
            continue
        fraction = min(1.0, max(0.0, ((x_m - ax) * dx + (y_m - ay) * dy) / length_sq))
        px = ax + fraction * dx
        py = ay + fraction * dy
        distance_sq = (x_m - px) ** 2 + (y_m - py) ** 2
        segment_length = math.sqrt(length_sq)
        path_yaw = math.atan2(dy, dx)
        candidate = (distance_sq, px, py, path_yaw, accumulated + fraction * segment_length)
        if best is None or candidate[0] < best[0]:
            best = candidate
        accumulated += segment_length
    if best is None:
        raise ValueError("trajectory has no non-zero-length segments")
    _, px, py, path_yaw, along_track = best
    lateral_error = -math.sin(path_yaw) * (x_m - px) + math.cos(path_yaw) * (y_m - py)
    return Projection(
        lateral_error_m=lateral_error,
        heading_error_rad=wrap_angle(yaw_rad - path_yaw),
        along_track_m=along_track,
    )


def _finite_pairs(first: Sequence[float], second: Sequence[float]) -> list[tuple[float, float]]:
    return [
        (left, right)
        for left, right in zip(first, second)
        if math.isfinite(left) and math.isfinite(right)
    ]


def _correlation(first: Sequence[float], second: Sequence[float]) -> float:
    pairs = _finite_pairs(first, second)
    if len(pairs) < 3:
        return math.nan
    left = [row[0] for row in pairs]
    right = [row[1] for row in pairs]
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    numerator = sum((a - left_mean) * (b - right_mean) for a, b in pairs)
    denominator = math.sqrt(
        sum((a - left_mean) ** 2 for a in left) * sum((b - right_mean) ** 2 for b in right)
    )
    return numerator / denominator if denominator > 1.0e-12 else math.nan


def estimate_actuator_delay(
    command: Sequence[tuple[float, float]],
    actual: Sequence[tuple[float, float]],
    max_delay_s: float = 0.5,
    step_s: float = GRID_STEP_S,
) -> DelayResult:
    """Find causal actuator delay by minimizing debiased command-to-feedback RMSE."""

    if len(command) < 3 or len(actual) < 3:
        raise ValueError("at least three command and feedback samples are required")
    start = max(command[0][0], actual[0][0])
    end = min(command[-1][0], actual[-1][0])
    if end - start <= max_delay_s + step_s:
        raise ValueError("time span is too short to estimate delay")
    grid_count = int((end - start) / step_s) + 1
    grid = [start + index * step_s for index in range(grid_count)]
    command_grid = zero_order_hold(command, grid)
    actual_grid = linear_interpolate_with_gap_limit(actual, grid, max_gap_s=3.0 * step_s)
    max_steps = int(round(max_delay_s / step_s))
    candidates: list[tuple[float, float, float, int]] = []
    for lag_steps in range(max_steps + 1):
        pairs = _finite_pairs(command_grid[:-lag_steps or None], actual_grid[lag_steps:])
        if len(pairs) < 30:
            continue
        errors = [measured - demand for demand, measured in pairs]
        bias = statistics.fmean(errors)
        rmse = math.sqrt(statistics.fmean((error - bias) ** 2 for error in errors))
        correlation = _correlation([row[0] for row in pairs], [row[1] for row in pairs])
        candidates.append((rmse, lag_steps * step_s, correlation, len(pairs)))
    if not candidates:
        raise ValueError("no finite overlap while estimating delay")
    rmse, delay_s, correlation, count = min(candidates, key=lambda item: (item[0], -item[2]))
    return DelayResult(delay_s=delay_s, rmse_rad=rmse, correlation=correlation, sample_count=count)


def select_pp_control_samples(
    command_times: Sequence[float], debug_rows: Sequence[tuple[float, tuple[float, ...]]]
) -> tuple[list[tuple[float, tuple[float, ...]]], int]:
    """Keep the first PP debug sample after each controller command timestamp.

    Pure Pursuit publishes another debug sample while constructing its predicted trajectory.  Both
    occur in the same control cycle; the first one corresponds to the real control calculation.
    """

    selected: list[tuple[float, tuple[float, ...]]] = []
    debug_index = 0
    duplicate_count = 0
    for command_index, command_time in enumerate(command_times):
        next_time = (
            command_times[command_index + 1]
            if command_index + 1 < len(command_times)
            else math.inf
        )
        while debug_index < len(debug_rows) and debug_rows[debug_index][0] < command_time:
            debug_index += 1
        if debug_index >= len(debug_rows) or debug_rows[debug_index][0] >= next_time:
            continue
        selected.append(debug_rows[debug_index])
        debug_index += 1
        while debug_index < len(debug_rows) and debug_rows[debug_index][0] < next_time:
            duplicate_count += 1
            debug_index += 1
    return selected, duplicate_count


def _timestamp_seconds(stamp: Any, fallback_s: float) -> float:
    if stamp is None:
        return fallback_s
    seconds = float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1.0e-9
    return seconds if seconds > 0.0 else fallback_s


def _message_timestamp(message: Any, fallback_s: float) -> float:
    header = getattr(message, "header", None)
    if header is not None:
        return _timestamp_seconds(getattr(header, "stamp", None), fallback_s)
    return _timestamp_seconds(getattr(message, "stamp", None), fallback_s)


def _trajectory_points(message: Any) -> tuple[tuple[float, float, float], ...]:
    result = []
    for point in getattr(message, "points", []):
        pose = point.pose
        orientation = pose.orientation
        result.append(
            (
                float(pose.position.x),
                float(pose.position.y),
                quaternion_to_yaw(
                    float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)
                ),
            )
        )
    return tuple(result)


def _topic_rows(database: Path) -> dict[str, tuple[int, str]]:
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        return {
            str(name): (int(topic_id), str(type_name))
            for topic_id, name, type_name in connection.execute("SELECT id, name, type FROM topics")
        }
    finally:
        connection.close()


def read_bag(database: Path) -> dict[str, Any]:
    """Deserialize the requested topics from a SQLite bag without writing to it."""

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    available = _topic_rows(database)
    selected = {
        label: (topic_name, available[topic_name])
        for label, topic_name in TOPICS.items()
        if topic_name in available
    }
    classes = {
        label: get_message(type_name)
        for label, (_, (_, type_name)) in selected.items()
    }
    rows: dict[str, list[Any]] = {label: [] for label in TOPICS}
    counts: Counter[str] = Counter()
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        for label, (topic_name, (topic_id, _)) in selected.items():
            query = "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp"
            last_predicted_time = -math.inf
            for timestamp_ns, serialized in connection.execute(query, (topic_id,)):
                bag_time_s = int(timestamp_ns) * 1.0e-9
                # A 50 Hz predicted horizon is expensive to deserialize and adds no information
                # to an offline map overlay.  Skip its serialized payload before deserialization.
                if label == "predicted_trajectory" and bag_time_s - last_predicted_time < 0.2:
                    continue
                message = deserialize_message(serialized, classes[label])
                counts[label] += 1
                time_s = _message_timestamp(message, bag_time_s)
                if label in {"follower_command", "final_command"}:
                    rows[label].append(
                        CommandSample(
                            time_s, bag_time_s,
                            float(message.lateral.steering_tire_angle),
                            float(message.longitudinal.velocity),
                        )
                    )
                elif label == "steering":
                    rows[label].append(SteeringSample(time_s, bag_time_s, float(message.steering_tire_angle)))
                elif label == "velocity":
                    rows[label].append(
                        VelocitySample(
                            time_s, bag_time_s, float(message.longitudinal_velocity),
                            float(message.heading_rate), float(message.lateral_velocity),
                        )
                    )
                elif label == "odometry":
                    pose = message.pose.pose
                    orientation = pose.orientation
                    rows[label].append(
                        OdomSample(
                            time_s, bag_time_s, float(pose.position.x), float(pose.position.y),
                            quaternion_to_yaw(
                                float(orientation.x), float(orientation.y),
                                float(orientation.z), float(orientation.w),
                            ),
                            float(message.twist.twist.linear.x),
                        )
                    )
                elif label == "acceleration":
                    rows[label].append(
                        ScalarSample(time_s, bag_time_s, float(message.accel.accel.linear.y))
                    )
                elif label in {"trajectory", "predicted_trajectory"}:
                    # Planning trajectory is needed at its native rate.  One predicted sample every
                    # 0.2 s is enough for diagnostic overlays and prevents holding every 50 Hz horizon.
                    points = _trajectory_points(message)
                    if len(points) >= 2:
                        rows[label].append(TrajectorySample(time_s, bag_time_s, points))
                        if label == "predicted_trajectory":
                            last_predicted_time = bag_time_s
                elif label == "pp_debug":
                    values = tuple(float(value) for value in message.data)
                    if len(values) >= len(PP_COLUMNS):
                        rows[label].append((time_s, values[: len(PP_COLUMNS)]))
                elif label == "control_mode":
                    rows[label].append(ScalarSample(time_s, bag_time_s, float(message.mode)))
                elif label == "gate_mode":
                    rows[label].append(ScalarSample(time_s, bag_time_s, float(message.data)))
                elif label == "gear":
                    rows[label].append(ScalarSample(time_s, bag_time_s, float(message.report)))
                elif label == "processing_time":
                    rows[label].append(ScalarSample(time_s, bag_time_s, float(message.data)))
    finally:
        connection.close()
    return {
        "database": str(database),
        "available_topics": {label: TOPICS[label] for label in selected},
        "missing_topics": [label for label in TOPICS if label not in selected],
        "counts": dict(counts),
        "rows": rows,
    }


def _finite(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def percentile(values: Sequence[float], fraction: float) -> float:
    values = sorted(_finite(values))
    if not values:
        return math.nan
    if len(values) == 1:
        return values[0]
    position = (len(values) - 1) * fraction
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return values[lower]
    remainder = position - lower
    return values[lower] * (1.0 - remainder) + values[upper] * remainder


def signal_metrics(values: Iterable[float]) -> dict[str, float | int | None]:
    values = _finite(values)
    if not values:
        return {"count": 0, "mean": None, "mae": None, "rmse": None, "p95_abs": None, "max_abs": None}
    absolute = [abs(value) for value in values]
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "mae": statistics.fmean(absolute),
        "rmse": math.sqrt(statistics.fmean(value * value for value in values)),
        "p95_abs": percentile(absolute, 0.95),
        "max_abs": max(absolute),
    }


def _nearest_index(times: Sequence[float], target: float) -> int | None:
    if not times:
        return None
    index = bisect.bisect_left(times, target)
    options = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(times)]
    return min(options, key=lambda candidate: abs(times[candidate] - target)) if options else None


def command_chain_metrics(
    follower: Sequence[CommandSample], final: Sequence[CommandSample]
) -> tuple[dict[str, Any], list[float]]:
    """Compare controller output and Gate output by their embedded command timestamps."""

    follower_times = [sample.time_s for sample in follower]
    differences: list[float] = []
    receipt_delays_ms: list[float] = []
    unmatched = 0
    for output in final:
        index = _nearest_index(follower_times, output.time_s)
        if index is None or abs(follower[index].time_s - output.time_s) > 0.05:
            unmatched += 1
            continue
        source = follower[index]
        differences.append(output.steering_rad - source.steering_rad)
        receipt_delays_ms.append((output.bag_time_s - source.bag_time_s) * 1000.0)
    changed = sum(abs(value) > 1.0e-4 for value in differences)
    return (
        {
            "matched_samples": len(differences),
            "unmatched_final_samples": unmatched,
            "steering_delta_rad": signal_metrics(differences),
            "changed_fraction": changed / len(differences) if differences else None,
            "receipt_delay_ms": {
                "median": percentile(receipt_delays_ms, 0.50),
                "p95": percentile(receipt_delays_ms, 0.95),
                "max": max(_finite(receipt_delays_ms), default=math.nan),
            },
        },
        differences,
    )


def _derivative(values: Sequence[float], step_s: float) -> list[float]:
    result = [math.nan] * len(values)
    for index in range(1, len(values)):
        if math.isfinite(values[index]) and math.isfinite(values[index - 1]):
            result[index] = (values[index] - values[index - 1]) / step_s
    return result


def _interpolate_angles(
    samples: Sequence[tuple[float, float]], grid: Sequence[float], max_gap_s: float
) -> list[float]:
    if not samples:
        return [math.nan] * len(grid)
    unwrapped: list[tuple[float, float]] = []
    previous = samples[0][1]
    unwrapped.append((samples[0][0], previous))
    for timestamp, angle in samples[1:]:
        previous += wrap_angle(angle - previous)
        unwrapped.append((timestamp, previous))
    return [wrap_angle(value) if math.isfinite(value) else value for value in linear_interpolate_with_gap_limit(unwrapped, grid, max_gap_s)]


def _default_output_dir(bag: Path) -> Path:
    label = bag.name if bag.is_dir() else bag.stem.removesuffix("_0")
    return Path("/home/nvidia/autoware/log/lateral_gap_analysis") / label


def _relative_time_grid(start_s: float, end_s: float, step_s: float) -> list[float]:
    return [start_s + index * step_s for index in range(int((end_s - start_s) / step_s) + 1)]


def _trajectory_at(
    trajectories: Sequence[TrajectorySample], timestamp_s: float, index: int
) -> tuple[TrajectorySample | None, int]:
    while index + 1 < len(trajectories) and trajectories[index + 1].time_s <= timestamp_s:
        index += 1
    if not trajectories or trajectories[index].time_s > timestamp_s:
        return None, index
    return trajectories[index], index


def _active_segments(active: Sequence[bool], grid: Sequence[float]) -> list[tuple[float, float]]:
    segments: list[tuple[float, float]] = []
    start: float | None = None
    for timestamp, enabled in zip(grid, active):
        if enabled and start is None:
            start = timestamp
        elif not enabled and start is not None:
            segments.append((start, timestamp))
            start = None
    if start is not None:
        segments.append((start, grid[-1]))
    return segments


def analyze(data: dict[str, Any], wheel_base_m: float, motion_threshold: float) -> dict[str, Any]:
    rows = data["rows"]
    required = ("follower_command", "final_command", "steering", "velocity", "odometry", "trajectory", "pp_debug")
    missing_required = [name for name in required if not rows[name]]
    if missing_required:
        raise ValueError(f"required topics contain no messages: {', '.join(missing_required)}")

    follower: list[CommandSample] = rows["follower_command"]
    final: list[CommandSample] = rows["final_command"]
    steering: list[SteeringSample] = rows["steering"]
    velocity: list[VelocitySample] = rows["velocity"]
    odometry: list[OdomSample] = rows["odometry"]
    trajectories: list[TrajectorySample] = rows["trajectory"]
    start_s = max(follower[0].time_s, final[0].time_s, steering[0].time_s, velocity[0].time_s, odometry[0].time_s)
    end_s = min(follower[-1].time_s, final[-1].time_s, steering[-1].time_s, velocity[-1].time_s, odometry[-1].time_s)
    grid = _relative_time_grid(start_s, end_s, GRID_STEP_S)

    follower_angle = zero_order_hold([(sample.time_s, sample.steering_rad) for sample in follower], grid)
    final_angle = zero_order_hold([(sample.time_s, sample.steering_rad) for sample in final], grid)
    actual_angle = linear_interpolate_with_gap_limit(
        [(sample.time_s, sample.steering_rad) for sample in steering], grid, MAX_FEEDBACK_GAP_S
    )
    speed = linear_interpolate_with_gap_limit(
        [(sample.time_s, sample.speed_mps) for sample in velocity], grid, MAX_FEEDBACK_GAP_S
    )
    heading_rate = linear_interpolate_with_gap_limit(
        [(sample.time_s, sample.heading_rate_radps) for sample in velocity], grid, MAX_FEEDBACK_GAP_S
    )
    odom_x = linear_interpolate_with_gap_limit(
        [(sample.time_s, sample.x_m) for sample in odometry], grid, MAX_FEEDBACK_GAP_S
    )
    odom_y = linear_interpolate_with_gap_limit(
        [(sample.time_s, sample.y_m) for sample in odometry], grid, MAX_FEEDBACK_GAP_S
    )
    odom_yaw = _interpolate_angles(
        [(sample.time_s, sample.yaw_rad) for sample in odometry], grid, MAX_FEEDBACK_GAP_S
    )
    control_mode = zero_order_hold([(sample.time_s, sample.value) for sample in rows["control_mode"]], grid)
    gate_mode = zero_order_hold([(sample.time_s, sample.value) for sample in rows["gate_mode"]], grid)
    active = [
        math.isfinite(measured_speed)
        and abs(measured_speed) > motion_threshold
        and (not rows["control_mode"] or mode == 1.0)
        and (not rows["gate_mode"] or gate == 0.0)
        for measured_speed, mode, gate in zip(speed, control_mode, gate_mode)
    ]

    active_command = [(timestamp, angle) for timestamp, angle, enabled in zip(grid, final_angle, active) if enabled]
    active_actual = [(timestamp, angle) for timestamp, angle, enabled in zip(grid, actual_angle, active) if enabled]
    delay = estimate_actuator_delay(active_command, active_actual)
    delayed_command = zero_order_hold([(sample.time_s, sample.steering_rad) for sample in final], [time - delay.delay_s for time in grid])
    raw_error = [actual - command if math.isfinite(actual) and math.isfinite(command) else math.nan for actual, command in zip(actual_angle, final_angle)]
    compensated_error = [
        actual - command if math.isfinite(actual) and math.isfinite(command) else math.nan
        for actual, command in zip(actual_angle, delayed_command)
    ]
    actual_rate = _derivative(actual_angle, GRID_STEP_S)
    command_rate = _derivative(delayed_command, GRID_STEP_S)

    pp_selected, pp_duplicate_count = select_pp_control_samples(
        [sample.time_s for sample in follower], rows["pp_debug"]
    )
    pp_series = {
        column: zero_order_hold([(time_s, values[index]) for time_s, values in pp_selected], grid)
        for index, column in enumerate(PP_COLUMNS)
    }

    lateral_error = [math.nan] * len(grid)
    heading_error = [math.nan] * len(grid)
    trajectory_age = [math.nan] * len(grid)
    trajectory_index = 0
    for index, timestamp in enumerate(grid):
        if not active[index] or not (math.isfinite(odom_x[index]) and math.isfinite(odom_y[index]) and math.isfinite(odom_yaw[index])):
            continue
        trajectory, trajectory_index = _trajectory_at(trajectories, timestamp, trajectory_index)
        if trajectory is None:
            continue
        try:
            projection = project_pose_to_trajectory(odom_x[index], odom_y[index], odom_yaw[index], trajectory.points)
        except ValueError:
            continue
        lateral_error[index] = projection.lateral_error_m
        heading_error[index] = projection.heading_error_rad
        trajectory_age[index] = timestamp - trajectory.time_s
    expected_yaw_rate = [
        measured_speed * math.tan(steering_angle) / wheel_base_m
        if math.isfinite(measured_speed) and math.isfinite(steering_angle)
        else math.nan
        for measured_speed, steering_angle in zip(speed, actual_angle)
    ]
    yaw_rate_error = [
        observed - expected if math.isfinite(observed) and math.isfinite(expected) else math.nan
        for observed, expected in zip(heading_rate, expected_yaw_rate)
    ]

    active_filter = lambda values: [value for value, enabled in zip(values, active) if enabled]
    chain, chain_values = command_chain_metrics(follower, final)
    speed_bands = {
        "0.05_to_0.5_mps": (motion_threshold, 0.5),
        "0.5_to_1.5_mps": (0.5, 1.5),
        "at_least_1.5_mps": (1.5, math.inf),
    }
    by_speed = {}
    for label, (lower, upper) in speed_bands.items():
        indices = [index for index, value in enumerate(speed) if active[index] and lower <= abs(value) < upper]
        by_speed[label] = {
            "samples": len(indices),
            "steering_error_rad": signal_metrics(compensated_error[index] for index in indices),
            "lateral_error_m": signal_metrics(lateral_error[index] for index in indices),
        }
    by_direction = {}
    for label, predicate in {
        "left_command": lambda angle: angle > 0.01,
        "right_command": lambda angle: angle < -0.01,
    }.items():
        indices = [index for index, angle in enumerate(delayed_command) if active[index] and math.isfinite(angle) and predicate(angle)]
        by_direction[label] = {
            "samples": len(indices),
            "steering_error_rad": signal_metrics(compensated_error[index] for index in indices),
        }

    series = {
        "epoch_s": grid,
        "elapsed_s": [time - start_s for time in grid],
        "active": active,
        "follower_steering_rad": follower_angle,
        "final_steering_rad": final_angle,
        "actual_steering_rad": actual_angle,
        "lag_compensated_command_rad": delayed_command,
        "raw_steering_error_rad": raw_error,
        "steering_error_rad": compensated_error,
        "command_rate_radps": command_rate,
        "actual_rate_radps": actual_rate,
        "speed_mps": speed,
        "heading_rate_radps": heading_rate,
        "expected_yaw_rate_radps": expected_yaw_rate,
        "yaw_rate_error_radps": yaw_rate_error,
        "x_m": odom_x,
        "y_m": odom_y,
        "yaw_rad": odom_yaw,
        "lateral_error_m": lateral_error,
        "heading_error_rad": heading_error,
        "trajectory_age_s": trajectory_age,
        **pp_series,
    }
    quality = {
        label: {
            "count": len(values),
            "first_epoch_s": values[0].time_s if values and hasattr(values[0], "time_s") else None,
            "last_epoch_s": values[-1].time_s if values and hasattr(values[0], "time_s") else None,
        }
        for label, values in rows.items()
        if values
    }
    anomalies: list[str] = []
    if data["missing_topics"]:
        anomalies.append("optional topics unavailable: " + ", ".join(data["missing_topics"]))
    if chain["changed_fraction"] is not None and chain["changed_fraction"] > 0.01:
        anomalies.append("Gate output differs from controller output in more than 1% of matched samples")
    if not math.isfinite(delay.correlation) or delay.correlation < 0.5:
        anomalies.append("command-to-feedback delay estimate has weak correlation; interpret the delay cautiously")
    if pp_duplicate_count:
        anomalies.append(
            f"discarded {pp_duplicate_count} Pure Pursuit predicted-trajectory debug samples from controller metrics"
        )
    return {
        "database": data["database"],
        "start_epoch_s": start_s,
        "end_epoch_s": end_s,
        "duration_s": end_s - start_s,
        "wheel_base_m": wheel_base_m,
        "motion_threshold_mps": motion_threshold,
        "topic_counts": data["counts"],
        "topic_quality": quality,
        "missing_topics": data["missing_topics"],
        "active_duration_s": sum(active) * GRID_STEP_S,
        "active_segments": _active_segments(active, grid),
        "command_chain": chain,
        "actuator_delay": asdict(delay),
        "steering": {
            "raw_error_rad": signal_metrics(active_filter(raw_error)),
            "lag_compensated_error_rad": signal_metrics(active_filter(compensated_error)),
            "command_rate_radps": signal_metrics(active_filter(command_rate)),
            "actual_rate_radps": signal_metrics(active_filter(actual_rate)),
        },
        "vehicle_tracking": {
            "lateral_error_m": signal_metrics(active_filter(lateral_error)),
            "heading_error_rad": signal_metrics(active_filter(heading_error)),
            "yaw_rate_error_radps": signal_metrics(active_filter(yaw_rate_error)),
            "pp_minus_recomputed_lateral_error_m": signal_metrics(
                pp - computed
                for pp, computed, enabled in zip(pp_series["pp_lateral_error_m"], lateral_error, active)
                if enabled and math.isfinite(pp) and math.isfinite(computed)
            ),
        },
        "pure_pursuit": {
            "control_samples": len(pp_selected),
            "predicted_trajectory_duplicate_debug_samples": pp_duplicate_count,
            "lookahead_distance_m": signal_metrics(active_filter(pp_series["lookahead_distance_m"])),
            "pp_lateral_error_m": signal_metrics(active_filter(pp_series["pp_lateral_error_m"])),
            "trajectory_curvature_1pm": signal_metrics(active_filter(pp_series["trajectory_curvature_1pm"])),
            "command_steering_margin_to_limit_rad": min(
                (MAX_STEERING_ANGLE_RAD - abs(value) for value in active_filter(final_angle) if math.isfinite(value)),
                default=math.nan,
            ),
        },
        "by_speed": by_speed,
        "by_direction": by_direction,
        "anomalies": anomalies,
        "series": series,
        "trajectory_snapshots": trajectories,
        "predicted_snapshots": rows["predicted_trajectory"],
        "chain_values": chain_values,
    }


def _clean_value(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _clean_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean_value(item) for item in value]
    return value


def worst_windows(analysis: dict[str, Any], window_s: float = 1.0, limit: int = 5) -> list[dict[str, Any]]:
    series = analysis["series"]
    width = int(round(window_s / GRID_STEP_S))
    candidates = []
    for start in range(0, len(series["epoch_s"]) - width + 1, width):
        stop = start + width
        active_count = sum(series["active"][start:stop])
        if active_count < int(0.7 * width):
            continue
        steering_rmse = signal_metrics(series["steering_error_rad"][start:stop])["rmse"]
        lateral_rmse = signal_metrics(series["lateral_error_m"][start:stop])["rmse"]
        score = (float(steering_rmse or 0.0) / 0.03) + (float(lateral_rmse or 0.0) / 0.10)
        candidates.append(
            {
                "start_epoch_s": series["epoch_s"][start],
                "end_epoch_s": series["epoch_s"][stop - 1],
                "start_elapsed_s": series["elapsed_s"][start],
                "steering_rmse_rad": steering_rmse,
                "lateral_rmse_m": lateral_rmse,
                "score": score,
            }
        )
    return sorted(candidates, key=lambda item: item["score"], reverse=True)[:limit]


def write_timeseries(path: Path, analysis: dict[str, Any]) -> None:
    series = analysis["series"]
    columns = list(series)
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=columns)
        writer.writeheader()
        for index in range(len(series["epoch_s"])):
            writer.writerow(
                {
                    column: "" if isinstance(series[column][index], float) and not math.isfinite(series[column][index]) else series[column][index]
                    for column in columns
                }
            )


def write_worst_windows(path: Path, windows: Sequence[dict[str, Any]]) -> None:
    fields = ["start_epoch_s", "end_epoch_s", "start_elapsed_s", "steering_rmse_rad", "lateral_rmse_m", "score"]
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(windows)


def _format_number(value: Any, precision: int = 5) -> str:
    return "N/A" if value is None or not isinstance(value, (int, float)) or not math.isfinite(value) else f"{value:.{precision}f}"


def render_report(analysis: dict[str, Any], windows: Sequence[dict[str, Any]]) -> str:
    chain = analysis["command_chain"]
    steering = analysis["steering"]
    tracking = analysis["vehicle_tracking"]
    pp = analysis["pure_pursuit"]
    lines = [
        "# 横向控制指令与实际执行差距分析",
        "",
        f"- 数据库：`{analysis['database']}`",
        f"- 分析时长：`{analysis['duration_s']:.3f} s`；有效运动：`{analysis['active_duration_s']:.3f} s`",
        f"- 轮距：`{analysis['wheel_base_m']:.3f} m`；运动阈值：`{analysis['motion_threshold_mps']:.3f} m/s`",
        "- 误差定义：`实际前轮转角 - 延迟补偿后的 Gate 最终前轮转角`。本报告仅量化和标注异常，不给出验收通过/失败结论。",
        "",
        "## 控制链与转向执行",
        "",
        f"- 估计命令到转向反馈延迟：`{analysis['actuator_delay']['delay_s']:.3f} s`；去偏置匹配 RMSE：`{analysis['actuator_delay']['rmse_rad']:.5f} rad`；相关性：`{analysis['actuator_delay']['correlation']:.4f}`。",
        f"- Gate 相对控制器命令的转角改写比例：`{_format_number(chain['changed_fraction'] * 100.0 if chain['changed_fraction'] is not None else None, 3)} %`；匹配样本：`{chain['matched_samples']}`。",
        f"- Gate 接收相对控制器输出延迟（bag 接收时间）：中位数 `{_format_number(chain['receipt_delay_ms']['median'], 3)} ms`，P95 `{_format_number(chain['receipt_delay_ms']['p95'], 3)} ms`。",
        "",
        "| 指标 | 未补偿 | 延迟补偿后 |",
        "|---|---:|---:|",
        f"| bias (rad) | {_format_number(steering['raw_error_rad']['mean'])} | {_format_number(steering['lag_compensated_error_rad']['mean'])} |",
        f"| MAE (rad) | {_format_number(steering['raw_error_rad']['mae'])} | {_format_number(steering['lag_compensated_error_rad']['mae'])} |",
        f"| RMSE (rad) | {_format_number(steering['raw_error_rad']['rmse'])} | {_format_number(steering['lag_compensated_error_rad']['rmse'])} |",
        f"| P95 absolute (rad) | {_format_number(steering['raw_error_rad']['p95_abs'])} | {_format_number(steering['lag_compensated_error_rad']['p95_abs'])} |",
        f"| max absolute (rad) | {_format_number(steering['raw_error_rad']['max_abs'])} | {_format_number(steering['lag_compensated_error_rad']['max_abs'])} |",
        "",
        "## 车辆轨迹与 Pure Pursuit",
        "",
        f"- 重算横向误差：RMSE `{_format_number(tracking['lateral_error_m']['rmse'])} m`，P95 `{_format_number(tracking['lateral_error_m']['p95_abs'])} m`，峰值 `{_format_number(tracking['lateral_error_m']['max_abs'])} m`。",
        f"- 航向误差：RMSE `{_format_number(tracking['heading_error_rad']['rmse'])} rad`；PP/重算横向误差差值 RMSE `{_format_number(tracking['pp_minus_recomputed_lateral_error_m']['rmse'])} m`。",
        f"- PP 前视距离：均值 `{_format_number(pp['lookahead_distance_m']['mean'])} m`，P95 `{_format_number(pp['lookahead_distance_m']['p95_abs'])} m`；控制转角到 `0.65 rad` 上限的最小余量 `{_format_number(pp['command_steering_margin_to_limit_rad'])} rad`。",
        f"- PP 控制样本：`{pp['control_samples']}`；已剔除预测轨迹内部重复 debug：`{pp['predicted_trajectory_duplicate_debug_samples']}`。",
        "",
        "## 最差 1 秒时间窗",
        "",
        "| 起始相对时间 (s) | 转角 RMSE (rad) | 横向误差 RMSE (m) | 综合排序分 |",
        "|---:|---:|---:|---:|",
    ]
    for window in windows:
        lines.append(
            f"| {window['start_elapsed_s']:.3f} | {_format_number(window['steering_rmse_rad'])} | "
            f"{_format_number(window['lateral_rmse_m'])} | {window['score']:.3f} |"
        )
    lines.extend(["", "## 数据质量与提示", ""])
    if analysis["anomalies"]:
        lines.extend(f"- {message}" for message in analysis["anomalies"])
    else:
        lines.append("- 未检测到话题缺失、控制链改写或低置信度延迟估计。")
    lines.extend(["", "## 分速度段（延迟补偿后）", "", "| 速度段 | 样本 | 转角 RMSE (rad) | 横向误差 RMSE (m) |", "|---|---:|---:|---:|"])
    for label, result in analysis["by_speed"].items():
        lines.append(
            f"| {label} | {result['samples']} | {_format_number(result['steering_error_rad']['rmse'])} | "
            f"{_format_number(result['lateral_error_m']['rmse'])} |"
        )
    lines.append("")
    return "\n".join(lines)


def write_plots(output_dir: Path, analysis: dict[str, Any], windows: Sequence[dict[str, Any]]) -> list[str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    series = analysis["series"]
    time = series["elapsed_s"]
    figures: list[tuple[str, Any]] = []
    figure, axis = plt.subplots(figsize=(14, 5))
    axis.plot(time, series["follower_steering_rad"], label="trajectory follower", alpha=0.75)
    axis.plot(time, series["final_steering_rad"], label="gate final", alpha=0.75)
    axis.plot(time, series["actual_steering_rad"], label="measured steering", alpha=0.75)
    axis.set(xlabel="elapsed time (s)", ylabel="steering angle (rad)", title="Control command chain and steering feedback")
    axis.legend(loc="best")
    axis.grid(True, alpha=0.3)
    figures.append(("control_chain.png", figure))

    figure, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    axes[0].plot(time, series["raw_steering_error_rad"], label="same-time error")
    axes[0].plot(time, series["steering_error_rad"], label="delay-compensated error")
    axes[0].set(ylabel="steering error (rad)", title="Steering tracking error")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(time, series["speed_mps"], label="speed")
    axes[1].set(xlabel="elapsed time (s)", ylabel="speed (m/s)")
    axes[1].grid(True, alpha=0.3)
    figures.append(("steering_error.png", figure))

    figure, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    axes[0].plot(time, series["lookahead_distance_m"], label="total")
    axes[0].plot(time, series["velocity_lookahead_m"], label="velocity term")
    axes[0].plot(time, series["curvature_lookahead_m"], label="curvature term")
    axes[0].plot(time, series["lateral_error_lookahead_m"], label="lateral-error term")
    axes[0].set(ylabel="lookahead distance (m)", title="Pure Pursuit lookahead decomposition")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(time, series["pp_lateral_error_m"], label="PP lateral error")
    axes[1].plot(time, series["lateral_error_m"], label="recomputed lateral error")
    axes[1].set(xlabel="elapsed time (s)", ylabel="lateral error (m)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    figures.append(("pure_pursuit.png", figure))

    figure, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    axes[0].plot(time, series["lateral_error_m"], label="lateral error")
    axes[0].plot(time, series["heading_error_rad"], label="heading error")
    axes[0].set(ylabel="tracking error", title="Trajectory tracking")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)
    axes[1].plot(time, series["heading_rate_radps"], label="measured yaw rate")
    axes[1].plot(time, series["expected_yaw_rate_radps"], label="v tan(delta) / wheelbase")
    axes[1].set(xlabel="elapsed time (s)", ylabel="yaw rate (rad/s)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)
    figures.append(("tracking_and_yaw_rate.png", figure))

    if windows:
        window = windows[0]
        center = window["start_epoch_s"] + 0.5
        snapshot_times = [sample.time_s for sample in analysis["trajectory_snapshots"]]
        predicted_times = [sample.time_s for sample in analysis["predicted_snapshots"]]
        trajectory_index = _nearest_index(snapshot_times, center)
        predicted_index = _nearest_index(predicted_times, center)
        figure, axis = plt.subplots(figsize=(8, 8))
        start = center - 5.0
        end = center + 5.0
        actual_xy = [
            (x, y) for epoch, x, y in zip(series["epoch_s"], series["x_m"], series["y_m"])
            if start <= epoch <= end and math.isfinite(x) and math.isfinite(y)
        ]
        if actual_xy:
            axis.plot(*zip(*actual_xy), label="actual base_link", linewidth=2)
        if trajectory_index is not None:
            points = analysis["trajectory_snapshots"][trajectory_index].points
            axis.plot([point[0] for point in points], [point[1] for point in points], label="reference trajectory")
        if predicted_index is not None:
            points = analysis["predicted_snapshots"][predicted_index].points
            axis.plot([point[0] for point in points], [point[1] for point in points], label="PP predicted", linestyle="--")
        axis.set(title="Worst-window trajectory overlay", xlabel="map x (m)", ylabel="map y (m)")
        axis.axis("equal")
        axis.legend(loc="best")
        axis.grid(True, alpha=0.3)
        figures.append(("worst_window_overlay.png", figure))

    written = []
    for name, figure in figures:
        figure.tight_layout()
        figure.savefig(output_dir / name, dpi=160)
        plt.close(figure)
        written.append(name)
    return written


def run_analysis(bag: Path, output_dir: Path, wheel_base_m: float, motion_threshold: float) -> dict[str, Any]:
    database = resolve_bag_database(bag)
    output_dir.mkdir(parents=True, exist_ok=True)
    analysis = analyze(read_bag(database), wheel_base_m, motion_threshold)
    windows = worst_windows(analysis)
    write_timeseries(output_dir / "timeseries.csv", analysis)
    write_worst_windows(output_dir / "worst_windows.csv", windows)
    (output_dir / "report.md").write_text(render_report(analysis, windows), encoding="utf-8")
    plot_names = write_plots(output_dir, analysis, windows)
    summary = {key: value for key, value in analysis.items() if key not in {"series", "trajectory_snapshots", "predicted_snapshots", "chain_values"}}
    summary["worst_windows"] = windows
    summary["plots"] = plot_names
    (output_dir / "summary.json").write_text(
        json.dumps(_clean_value(summary), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="bag directory or a single .db3 file")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--wheel-base", type=float, default=WHEEL_BASE_M)
    parser.add_argument("--motion-threshold", type=float, default=0.05)
    args = parser.parse_args()
    output_dir = args.output_dir or _default_output_dir(args.bag)
    summary = run_analysis(args.bag, output_dir, args.wheel_base, args.motion_threshold)
    print(f"wrote {output_dir}")
    print(
        "delay={delay:.3f}s, steering_rmse={rmse:.5f}rad, lateral_rmse={lateral:.5f}m".format(
            delay=summary["actuator_delay"]["delay_s"],
            rmse=summary["steering"]["lag_compensated_error_rad"]["rmse"],
            lateral=summary["vehicle_tracking"]["lateral_error_m"]["rmse"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
