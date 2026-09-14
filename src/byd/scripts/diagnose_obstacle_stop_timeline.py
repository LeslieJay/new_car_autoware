#!/usr/bin/env python3
"""Build a compact, evidence-oriented timeline for one obstacle-stop case.

This is intentionally an offline reader.  It does not replay or mutate a bag and
therefore can be run repeatedly while investigating a host-side simulation run.
The output keeps raw event timestamps next to derived gaps so that a conclusion
can be checked against the original launch log.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Any

import rosbag2_py
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_internal_debug_msgs.msg import ProcessingTimeTree
from autoware_perception_msgs.msg import PredictedObjects, TrackedObjects
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tier4_simulation_msgs.msg import DummyObject


START_X = 279.52191162109375
START_Y = -33.24296569824219
START_OX = -0.00024534598908445367
START_OY = -0.00041156653669880767
START_OZ = -0.5120475691803335
START_OW = 0.8589569589419735
VEHICLE_LENGTH_M = 1.008 + 0.546 + 0.676
VEHICLE_WIDTH_M = 0.835 + 0.235 + 0.235


def yaw(ox: float, oy: float, oz: float, ow: float) -> float:
    return math.atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz))


ROUTE_YAW = yaw(START_OX, START_OY, START_OZ, START_OW)


def longitudinal(x: float, y: float) -> float:
    return (x - START_X) * math.cos(ROUTE_YAW) + (y - START_Y) * math.sin(ROUTE_YAW)


def lateral(x: float, y: float) -> float:
    return -(x - START_X) * math.sin(ROUTE_YAW) + (y - START_Y) * math.cos(ROUTE_YAW)


def ts_from_log(line: str) -> float | None:
    match = re.search(r"\[(\d+\.\d+)\]", line)
    return float(match.group(1)) if match else None


def max_gap(times: list[float]) -> float:
    return max((b - a for a, b in zip(times, times[1:])), default=0.0)


def first_at_or_after(times: list[float], start: float | None) -> float | None:
    if start is None:
        return times[0] if times else None
    return next((t for t in times if t >= start), None)


def parse_log_events(log_path: Path, monitor: dict[str, Any]) -> dict[str, Any]:
    events: list[dict[str, Any]] = []
    if not log_path.exists():
        return {"events": events, "log_exists": False}

    patterns = {
        "target_locked": re.compile(r"active target locked uuid=([0-9a-f]+) lon=([+-]?\d+(?:\.\d+)?)m lat=([+-]?\d+(?:\.\d+)?)m"),
        "path_generated": re.compile(r"avoidance path generated shift=([+-]?\d+(?:\.\d+)?) target_lon=([+-]?\d+(?:\.\d+)?) target_lat=([+-]?\d+(?:\.\d+)?) required_clearance=([+-]?\d+(?:\.\d+)?) required_before_front=([+-]?\d+(?:\.\d+)?) dist_to_avoid_start=([+-]?\d+(?:\.\d+)?) transition=([+-]?\d+(?:\.\d+)?) jerk_distance=([+-]?\d+(?:\.\d+)?) ego_speed=([+-]?\d+(?:\.\d+)?) dist_to_shift_end=([+-]?\d+(?:\.\d+)?) dist_to_obstacle=([+-]?\d+(?:\.\d+)?) lon_margin=([+-]?\d+(?:\.\d+)?)"),
        "commitment": re.compile(r"commitment diagnostics: distance_to_shift_start=([+-]?\d+(?:\.\d+)?)m commitment_lead_distance=([+-]?\d+(?:\.\d+)?)m expected_current_shift=([+-]?\d+(?:\.\d+)?)m actual_lateral_offset=([+-]?\d+(?:\.\d+)?)m .*lifecycle_state=([A-Z]+) committed=(\d+)"),
        "target_expired": re.compile(r"(?:success check releases expired target|target hold expired) uuid=([0-9a-f]+).*last_seen_age=([+-]?\d+(?:\.\d+)?)s"),
        "pass_through": re.compile(r"pass-through reason=([a-z_]+).*?(?:lon=([+-]?\d+(?:\.\d+)?)|objects=)"),
        "mrm": re.compile(r"(EMERGENCY_STOP|COMFORTABLE_STOP) is (operated|canceled)"),
        "trajectory_timeout": re.compile(r"/planning/trajectory (?:topic is timeout|has not received)"),
        "trajectory_delay": re.compile(r"trajectory is delayed: .*?delay = ([+-]?\d+(?:\.\d+)?)"),
        "freespace_callback": re.compile(r"freespace planner callback finished"),
        "autoware_state": re.compile(r"AutowareState: ([A-Za-z]+) => ([A-Za-z]+)"),
        "planner_timing": re.compile(r"\[SIMPLE_AVOIDANCE_TIMING\] cycle=(\d+) entry_gap_ms=([+-]?\d+(?:\.\d+)?) state=([A-Z]+)"),
        "bpp_timing": re.compile(r"\[BPP_TIMING\] cycle=(\d+)(?: entry_mono_ns=(\d+) exit_mono_ns=(\d+))? entry_gap_ms=([+-]?\d+(?:\.\d+)?) planner_run_ms=([+-]?\d+(?:\.\d+)?) total_run_ms=([+-]?\d+(?:\.\d+)?)"),
        "bpp_scene_timing": re.compile(r"\[BPP_SCENE_TIMING\] module=([^ ]+)(?: start_mono_ns=(\d+) end_mono_ns=(\d+))? run_ms=([+-]?\d+(?:\.\d+)?)"),
    }
    monitor_start = float(monitor.get("start_epoch", "inf"))
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        stamp = ts_from_log(line)
        if stamp is None:
            continue
        for kind, pattern in patterns.items():
            match = pattern.search(line)
            if not match:
                continue
            item: dict[str, Any] = {"time": stamp, "kind": kind}
            if kind == "target_locked":
                item.update({"uuid": match.group(1), "lon_m": float(match.group(2)), "lat_m": float(match.group(3))})
            elif kind == "path_generated":
                names = ("shift_m", "target_lon_m", "target_lat_m", "required_clearance_m", "required_before_front_m", "dist_to_avoid_start_m", "transition_m", "jerk_distance_m", "ego_speed_mps", "dist_to_shift_end_m", "dist_to_obstacle_m", "lon_margin_m")
                item.update(dict(zip(names, (float(value) for value in match.groups()))))
            elif kind == "commitment":
                names = ("distance_to_shift_start_m", "commitment_lead_distance_m", "expected_shift_m", "actual_lateral_offset_m", "lifecycle_state", "committed")
                values = list(match.groups())
                item.update(dict(zip(names, [*map(float, values[:4]), values[4], bool(int(values[5]))])))
            elif kind == "target_expired":
                item.update({"uuid": match.group(1), "last_seen_age_s": float(match.group(2))})
            elif kind == "pass_through":
                item["reason"] = match.group(1)
                if match.group(2) is not None:
                    item["lon_m"] = float(match.group(2))
            elif kind == "mrm":
                item.update({"stop_type": match.group(1), "action": match.group(2), "active_window": stamp >= monitor_start})
            elif kind == "trajectory_delay":
                item["delay_s"] = float(match.group(1))
            elif kind == "autoware_state":
                item.update({"old": match.group(1), "new": match.group(2)})
            elif kind == "planner_timing":
                item.update({"cycle": int(match.group(1)), "entry_gap_ms": float(match.group(2)), "state": match.group(3)})
            elif kind == "bpp_timing":
                item.update({
                    "cycle": int(match.group(1)),
                    "entry_mono_ns": int(match.group(2)) if match.group(2) else None,
                    "exit_mono_ns": int(match.group(3)) if match.group(3) else None,
                    "entry_gap_ms": float(match.group(4)),
                    "planner_run_ms": float(match.group(5)),
                    "total_run_ms": float(match.group(6)),
                })
            elif kind == "bpp_scene_timing":
                item.update({
                    "module": match.group(1),
                    "start_mono_ns": int(match.group(2)) if match.group(2) else None,
                    "end_mono_ns": int(match.group(3)) if match.group(3) else None,
                    "run_ms": float(match.group(4)),
                })
            item["raw"] = line
            events.append(item)

    events.sort(key=lambda item: item["time"])
    # A repeated lock for the same UUID is a re-lock/refresh event.  The
    # planner log does not emit a separate "refresh" line, so retain this
    # derived lifecycle marker beside the raw lock event.
    locked_uuids: set[str] = set()
    for event in events:
        if event["kind"] != "target_locked":
            continue
        uuid = event.get("uuid")
        event["lock_kind"] = "relock" if uuid in locked_uuids else "initial"
        if uuid:
            locked_uuids.add(uuid)
    active_events = [item for item in events if item["time"] >= monitor_start]
    delays = [item["delay_s"] for item in events if item["kind"] == "trajectory_delay"]
    # Pair each commitment diagnostic with the path generated immediately after it.
    # This exposes the one-cycle ordering without pretending that a plain text log
    # contains an explicit planner cycle id.
    cycles: list[dict[str, Any]] = []
    commitment_events = [e for e in events if e["kind"] == "commitment"]
    path_events = [e for e in events if e["kind"] == "path_generated"]
    timing_events = [e for e in events if e["kind"] == "planner_timing"]
    bpp_timing_events = [e for e in events if e["kind"] == "bpp_timing"]
    bpp_scene_timing_events = [e for e in events if e["kind"] == "bpp_scene_timing"]
    for index, commitment in enumerate(commitment_events, start=1):
        following = next(
            (path for path in path_events if 0.0 <= path["time"] - commitment["time"] <= 0.2),
            None,
        )
        cycles.append(
            {
                "cycle": index,
                "commitment_time": commitment["time"],
                "commitment_distance_to_shift_start_m": commitment.get("distance_to_shift_start_m"),
                "commitment_state": commitment.get("lifecycle_state"),
                "committed": commitment.get("committed", False),
                "path_time": following.get("time") if following else None,
                "path_dist_to_avoid_start_m": following.get("dist_to_avoid_start_m") if following else None,
                "path_required_before_front_m": following.get("required_before_front_m") if following else None,
                "path_target_lon_m": following.get("target_lon_m") if following else None,
                "path_ego_speed_mps": following.get("ego_speed_mps") if following else None,
                "commitment_path_order": (
                    "new_path_after_commitment"
                    if following is not None
                    and following.get("time", commitment["time"]) >= commitment["time"]
                    else "previous_path_or_missing"
                ),
                "path_commitment_distance_delta_m": (
                    following.get("dist_to_avoid_start_m")
                    - commitment.get("distance_to_shift_start_m")
                    if following is not None
                    and following.get("dist_to_avoid_start_m") is not None
                    and commitment.get("distance_to_shift_start_m") is not None
                    else None
                ),
                "timing_entry_gap_ms": min(
                    (abs(timing["time"] - commitment["time"]), timing["entry_gap_ms"])
                    for timing in timing_events
                )[1]
                if timing_events
                else None,
            }
        )
    return {
        "events": events,
        "active_events": active_events,
        "log_exists": True,
        "trajectory_timeout_count": sum(item["kind"] == "trajectory_timeout" for item in events),
        "max_reported_trajectory_delay_s": max(delays, default=0.0),
        "freespace_callback_count": sum(item["kind"] == "freespace_callback" for item in events),
        "cycles": cycles,
        "planner_timing": timing_events,
        "bpp_timing": bpp_timing_events,
        "bpp_scene_timing": bpp_scene_timing_events,
    }


def analyze_bag(bag_path: Path, monitor: dict[str, Any]) -> dict[str, Any]:
    types = {
        "/simulation/dummy_perception_publisher/object_info": DummyObject,
        "/simulation/dummy_perception_publisher/output/debug/ground_truth_objects": TrackedObjects,
        "/simulation/debug/ground_truth_objects": TrackedObjects,
        "/perception/object_recognition/tracking/objects": TrackedObjects,
        "/perception/object_recognition/objects": PredictedObjects,
        "/localization/kinematic_state": Odometry,
        "/planning/trajectory": Trajectory,
        "/planning/planning_factors/simple_avoidance": PlanningFactorArray,
        "/planning/planning_factors/obstacle_stop": PlanningFactorArray,
        "/planning/planning_factors/dynamic_obstacle_stop": PlanningFactorArray,
        "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time_detail_ms/start_planner": ProcessingTimeTree,
        "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time_detail_ms/simple_avoidance": ProcessingTimeTree,
        "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/processing_time_detail_ms/behavior_path_planner": ProcessingTimeTree,
    }
    if not bag_path.exists():
        return {"exists": False, "topics": {}}

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    topic_times: dict[str, list[float]] = {}
    nonempty_times: dict[str, list[float]] = {}
    nonempty_object_ids: dict[str, list[set[str]]] = {}
    unique_object_ids: dict[str, set[str]] = {}
    max_object_counts: dict[str, int] = {}
    factor_events: dict[str, list[dict[str, Any]]] = {}
    trajectory_times: list[float] = []
    trajectory_near_distances: list[dict[str, float]] = []
    ego_lons: list[tuple[float, float]] = []
    obstacle_center: tuple[float, float] | None = None
    obstacle_shape = (2.0, 1.5)
    processing_time: dict[str, dict[str, float]] = {}

    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        stamp = bag_time / 1e9
        topic_times.setdefault(topic, []).append(stamp)
        msg_type = types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, msg_type)
        if topic == "/simulation/dummy_perception_publisher/object_info":
            if msg.action == DummyObject.DELETEALL:
                # Define the analysis epoch at the last clear-first command;
                # stale objects from another host process are outside this case.
                obstacle_center = None
                obstacle_shape = (2.0, 1.5)
                continue
            if msg.action == DummyObject.ADD and obstacle_center is None:
                p = msg.initial_state.pose_covariance.pose.position
                obstacle_center = (p.x, p.y)
                obstacle_shape = (float(msg.shape.dimensions.x) or 2.0, float(msg.shape.dimensions.y) or 1.5)
        elif topic.endswith("processing_time_detail_ms/start_planner") or topic.endswith("processing_time_detail_ms/simple_avoidance") or topic.endswith("processing_time_detail_ms/behavior_path_planner"):
            maxima = processing_time.setdefault(topic, {})
            for node in msg.nodes:
                maxima[node.name] = max(maxima.get(node.name, 0.0), float(node.processing_time))
        elif hasattr(msg, "objects"):
            object_count = len(msg.objects)
            max_object_counts[topic] = max(max_object_counts.get(topic, 0), object_count)
            if msg.objects:
                nonempty_times.setdefault(topic, []).append(stamp)
                ids = {
                    bytes(obj.object_id.uuid).hex()
                    for obj in msg.objects
                    if hasattr(obj, "object_id")
                }
                unique_object_ids.setdefault(topic, set()).update(ids)
                nonempty_object_ids.setdefault(topic, []).append(ids)
        elif topic == "/planning/trajectory":
            if msg.points:
                trajectory_times.append(stamp)
                if obstacle_center is not None:
                    nearest = min(msg.points, key=lambda point: math.hypot(point.pose.position.x - obstacle_center[0], point.pose.position.y - obstacle_center[1]))
                    trajectory_near_distances.append({
                        "time": stamp,
                        "center_distance_m": math.hypot(nearest.pose.position.x - obstacle_center[0], nearest.pose.position.y - obstacle_center[1]),
                        "nearest_lon_m": longitudinal(nearest.pose.position.x, nearest.pose.position.y),
                        "nearest_lat_m": lateral(nearest.pose.position.x, nearest.pose.position.y),
                        "max_velocity_mps": max(point.longitudinal_velocity_mps for point in msg.points),
                    })
        elif topic == "/localization/kinematic_state":
            p = msg.pose.pose.position
            ego_lons.append((stamp, longitudinal(p.x, p.y)))
        elif topic.startswith("/planning/planning_factors/"):
            behaviors = [int(factor.behavior) for factor in msg.factors]
            if behaviors:
                factor_events.setdefault(topic, []).append({"time": stamp, "behaviors": behaviors})

    topic_summary: dict[str, Any] = {}
    for topic, times in topic_times.items():
        nonempty = nonempty_times.get(topic, [])
        topic_summary[topic] = {
            "message_count": len(times),
            "first_time": times[0] if times else None,
            "last_time": times[-1] if times else None,
            "nonempty_count": len(nonempty),
            "first_nonempty_time": nonempty[0] if nonempty else None,
            "last_nonempty_time": nonempty[-1] if nonempty else None,
            "max_gap_s": max_gap(times),
            "max_nonempty_gap_s": max_gap(nonempty),
            "max_object_count": max_object_counts.get(topic, 0),
            "unique_object_ids": sorted(unique_object_ids.get(topic, set())),
            "uuid_stable": (
                len(unique_object_ids.get(topic, set())) <= 1
                if nonempty_object_ids.get(topic)
                else None
            ),
            "uuid_switch_count": sum(
                previous != current
                for previous, current in zip(
                    nonempty_object_ids.get(topic, []),
                    nonempty_object_ids.get(topic, [])[1:],
                )
            ),
        }
    monitor_start = float(monitor.get("start_epoch", "inf"))
    active_trajectory_times = [t for t in trajectory_times if t >= monitor_start]
    max_trajectory_gap_after_monitor = max_gap(active_trajectory_times)
    bag_end_time = max((times[-1] for times in topic_times.values() if times), default=None)
    return {
        "exists": True,
        "topics": topic_summary,
        "factor_events": factor_events,
        "trajectory": {
            "count": len(trajectory_times),
            "first_time": trajectory_times[0] if trajectory_times else None,
            "last_time": trajectory_times[-1] if trajectory_times else None,
            "max_gap_s": max_gap(trajectory_times),
            "max_gap_after_monitor_s": max_trajectory_gap_after_monitor,
            "bag_end_time": bag_end_time,
            "near_obstacle": trajectory_near_distances,
        },
        "ego": {
            "count": len(ego_lons),
            "max_longitudinal_m": max((value for _, value in ego_lons), default=None),
            "last_longitudinal_m": ego_lons[-1][1] if ego_lons else None,
        },
        "obstacle": {
            "center": obstacle_center,
            "length_m": obstacle_shape[0],
            "width_m": obstacle_shape[1],
        },
        "processing_time_max_ms": processing_time,
    }


def classify(monitor: dict[str, Any], logs: dict[str, Any], bag: dict[str, Any]) -> dict[str, Any]:
    active = logs.get("active_events", [])
    # The first lock often precedes monitor start by 1–2 seconds.  Keep all
    # timestamped events for causality, while retaining the active subset for
    # the user's acceptance window.
    all_events = logs.get("events", [])
    locks = [e for e in all_events if e["kind"] == "target_locked"]
    generated = [e for e in all_events if e["kind"] == "path_generated"]
    commitments = [e for e in all_events if e["kind"] == "commitment"]
    expiries = [e for e in all_events if e["kind"] == "target_expired"]
    committed = [e for e in commitments if e.get("committed")]
    factors = bag.get("factor_events", {})
    stop_events = factors.get("/planning/planning_factors/obstacle_stop", [])
    first_stop = stop_events[0]["time"] if stop_events else None
    first_expiry = expiries[0]["time"] if expiries else None
    first_commit = committed[0]["time"] if committed else None
    trajectory_times = [item["time"] for item in bag.get("trajectory", {}).get("near_obstacle", [])]
    first_lock = locks[0]["time"] if locks else None
    post_lock_trajectory = [t for t in trajectory_times if first_lock is None or t >= first_lock]
    if first_lock is not None and post_lock_trajectory:
        trajectory_gap = max(post_lock_trajectory[0] - first_lock, max_gap(post_lock_trajectory))
    elif first_lock is not None:
        bag_end = bag.get("trajectory", {}).get("bag_end_time")
        trajectory_gap = max(0.0, float(bag_end) - first_lock) if bag_end is not None else 0.0
    else:
        trajectory_gap = 0.0
    return {
        "monitor_result": monitor.get("monitor_result"),
        "vehicle_passed": bool(monitor.get("vehicle_passed", False)),
        "obstacle_crossed": bool(monitor.get("obstacle_crossed", monitor.get("vehicle_passed", False))),
        "goal_reached": bool(monitor.get("goal_reached", monitor.get("monitor_result") == "ARRIVED")),
        "obstacle_crossed_epoch": monitor.get("obstacle_crossed_epoch"),
        "goal_reached_epoch": monitor.get("goal_reached_epoch"),
        "route_state": monitor.get("route_state"),
        "route_set_seen": bool(monitor.get("route_set_seen", False)),
        "first_target_lock_time": locks[0]["time"] if locks else None,
        "target_lock_events": [
            {
                "time": event["time"],
                "uuid": event.get("uuid"),
                "kind": event.get("lock_kind", "initial"),
            }
            for event in locks
        ],
        "target_relock_count": sum(event.get("lock_kind") == "relock" for event in locks),
        "path_generated_count": len(generated),
        "first_commit_time": first_commit,
        "candidate_commitment_count": len(commitments) - len(committed),
        "target_expiry_count": len(expiries),
        "first_target_expiry_time": first_expiry,
        "first_obstacle_stop_factor_time": first_stop,
        "expiry_to_stop_s": first_stop - first_expiry if first_expiry is not None and first_stop is not None else None,
        "trajectory_gap_after_target_lock_s": trajectory_gap,
        "trajectory_gap_after_monitor_s": float(bag.get("trajectory", {}).get("max_gap_after_monitor_s", 0.0)),
        "trajectory_timeout_count": logs.get("trajectory_timeout_count", 0),
        "max_reported_trajectory_delay_s": logs.get("max_reported_trajectory_delay_s", 0.0),
        "planner_cycles": logs.get("cycles", []),
        "bpp_timing": logs.get("bpp_timing", []),
        "bpp_scene_timing": logs.get("bpp_scene_timing", []),
        "perception_max_gap_s": {
            topic: details.get("max_gap_s", details.get("max_nonempty_gap_s", 0.0))
            for topic, details in bag.get("topics", {}).items()
            if "object_recognition" in topic or "ground_truth_objects" in topic
        },
        "perception_uuid_continuity": {
            topic: {
                "uuid_stable": details.get("uuid_stable"),
                "uuid_switch_count": details.get("uuid_switch_count", 0),
                "unique_object_ids": details.get("unique_object_ids", []),
                "max_object_count": details.get("max_object_count", 0),
            }
            for topic, details in bag.get("topics", {}).items()
            if "object_recognition" in topic or "ground_truth_objects" in topic
        },
        "placement_valid": monitor.get("placement", {}).get("placement_valid")
        if isinstance(monitor.get("placement"), dict)
        else None,
        "startup_contaminated": trajectory_gap > 1.0 or logs.get("max_reported_trajectory_delay_s", 0.0) > 1.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    case_dir = args.case_dir
    monitor_path = case_dir / "monitor.json"
    monitor = json.loads(monitor_path.read_text(encoding="utf-8")) if monitor_path.exists() else {}
    logs = parse_log_events(case_dir / "launch.log", monitor)
    bag = analyze_bag(case_dir / "rosbag", monitor)
    result = {
        "case_dir": str(case_dir),
        "distance_m": monitor.get("distance_m"),
        "monitor": monitor,
        "logs": logs,
        "bag": bag,
    }
    result["classification"] = classify(monitor, logs, bag)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    cycles = logs.get("cycles", [])
    cycle_fields = [
        "cycle", "commitment_time", "commitment_distance_to_shift_start_m", "commitment_state",
        "committed", "path_time", "path_dist_to_avoid_start_m", "path_required_before_front_m",
        "path_target_lon_m", "path_ego_speed_mps", "commitment_path_order",
        "path_commitment_distance_delta_m", "timing_entry_gap_ms",
    ]
    with (case_dir / "cycle_timeline.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=cycle_fields)
        writer.writeheader()
        writer.writerows({field: cycle.get(field) for field in cycle_fields} for cycle in cycles)
    with (case_dir / "bpp_timing.csv").open("w", newline="", encoding="utf-8") as stream:
        timing_fields = [
            "time", "kind", "cycle", "entry_mono_ns", "exit_mono_ns",
            "entry_gap_ms", "planner_run_ms", "total_run_ms",
        ]
        writer = csv.DictWriter(stream, fieldnames=timing_fields)
        writer.writeheader()
        writer.writerows(
            {field: event.get(field) for field in timing_fields}
            for event in logs.get("bpp_timing", [])
        )
    with (case_dir / "bpp_scene_timing.csv").open("w", newline="", encoding="utf-8") as stream:
        scene_fields = ["time", "kind", "module", "start_mono_ns", "end_mono_ns", "run_ms"]
        writer = csv.DictWriter(stream, fieldnames=scene_fields)
        writer.writeheader()
        writer.writerows(
            {field: event.get(field) for field in scene_fields}
            for event in logs.get("bpp_scene_timing", [])
        )
    with (case_dir / "target_lifecycle.csv").open("w", newline="", encoding="utf-8") as stream:
        lifecycle_fields = ["time", "kind", "uuid", "lock_kind", "last_seen_age_s", "lon_m", "lat_m"]
        writer = csv.DictWriter(stream, fieldnames=lifecycle_fields)
        writer.writeheader()
        writer.writerows(
            {field: event.get(field) for field in lifecycle_fields}
            for event in logs.get("events", [])
            if event.get("kind") in {"target_locked", "target_expired"}
        )
    print(json.dumps(result["classification"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
