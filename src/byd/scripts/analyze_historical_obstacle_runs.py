#!/usr/bin/env python3
"""Offline analysis for the manually recorded 20260911 obstacle runs.

The historical recordings predate the structured case runner.  This adapter
keeps the source bags/logs read-only, pairs them by capture time, and emits the
same evidence-oriented records used by the current obstacle-stop reports.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from datetime import datetime
from decimal import Decimal, InvalidOperation
from pathlib import Path
from statistics import median
from typing import Iterable

import rosbag2_py
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_perception_msgs.msg import PredictedObjects, TrackedObjects
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tier4_simulation_msgs.msg import DummyObject

from analyze_longitudinal_obstacle_case import (
    BASE_LINK_TO_VEHICLE_CENTER_M,
    OBSTACLE_LENGTH_M,
    factor_has_behavior,
    lateral_offset,
    longitudinal,
    quat_to_yaw,
    rectangle_signed_clearance,
    uuid_hex,
)


VEHICLE_REAR_OVERHANG_M = 0.676
RESULTS = (
    "PASS",
    "STOPPED_BEFORE_OBSTACLE",
    "STOPPED_AFTER_OBSTACLE",
    "COLLISION",
    "MRM",
    "INVALID_TRAJECTORY",
    "TIMEOUT",
    "INCOMPLETE_OBSERVATION",
    "SETUP_INVALID",
)


@dataclass(frozen=True)
class BagRecord:
    path: Path
    start_epoch: float
    duration_sec: float


@dataclass(frozen=True)
class LogRecord:
    path: Path
    launch_epoch: float


@dataclass(frozen=True)
class PairRecord:
    bag_path: Path | None
    log_path: Path | None
    bag_start_epoch: float | None
    bag_duration_sec: float | None
    distance: float | None
    intrusion: float | None
    shoulder: str
    repetition: int
    case_id: str
    pairing_status: str
    pairing_reason: str


def _number(value: float | None) -> str:
    return f"{value:g}" if value is not None else "unknown"


def parse_case_parameters(path: Path) -> tuple[float, float]:
    """Parse ``<longitudinal>_<intrusion-tenths>.log`` historical names."""

    match = re.search(r"(?<!\d)(\d+(?:\.\d+)?)_(\d+)(?:$|\.)", path.name)
    if match is None:
        raise ValueError(f"cannot parse distance/intrusion from {path.name}")
    distance = float(match.group(1))
    token = match.group(2)
    intrusion = float(token) / 10.0 if token.isdigit() else float(token)
    return distance, intrusion


def parse_launch_epoch(path: Path) -> float:
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(
        r"All log files can be found below .*/(\d{4}-\d{2}-\d{2}-\d{2}-\d{2}-\d{2})",
        text,
    )
    if match is None:
        raise ValueError(f"launch timestamp not found in {path}")
    return datetime.strptime(match.group(1), "%Y-%m-%d-%H-%M-%S").timestamp()


def read_bag_metadata(metadata_path: Path) -> BagRecord:
    text = metadata_path.read_text(encoding="utf-8", errors="replace")
    start_match = re.search(
        r"starting_time:\s*\n\s*nanoseconds_since_epoch:\s*(\d+)", text
    )
    duration_match = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if start_match is None or duration_match is None:
        raise ValueError(f"bag timestamps not found in {metadata_path}")
    return BagRecord(
        path=metadata_path.parent,
        start_epoch=int(start_match.group(1)) / 1e9,
        duration_sec=int(duration_match.group(1)) / 1e9,
    )


def pair_records(
    bags: Iterable[BagRecord], logs: Iterable[LogRecord], shoulder: str = "right"
) -> list[PairRecord]:
    """Pair sorted captures and assign repetitions for equal parameter points."""

    sorted_bags = sorted(bags, key=lambda item: item.start_epoch)
    sorted_logs = sorted(logs, key=lambda item: item.launch_epoch)
    repetitions: defaultdict[tuple[float, float, str], int] = defaultdict(int)
    pairs: list[PairRecord] = []
    for index in range(max(len(sorted_bags), len(sorted_logs))):
        bag = sorted_bags[index] if index < len(sorted_bags) else None
        log = sorted_logs[index] if index < len(sorted_logs) else None
        distance: float | None = None
        intrusion: float | None = None
        reason: list[str] = []
        if log is not None:
            try:
                distance, intrusion = parse_case_parameters(log.path)
            except ValueError as exc:
                reason.append(str(exc))
        if bag is None:
            reason.append("MISSING_BAG")
        if log is None:
            reason.append("MISSING_LOG")
        if bag is not None and log is not None and log.launch_epoch > bag.start_epoch:
            reason.append("LOG_START_AFTER_BAG_START")
        if distance is not None and intrusion is not None:
            key = (distance, intrusion, shoulder)
            repetitions[key] += 1
            repetition = repetitions[key]
            case_id = (
                f"d{_number(distance)}m_i{_number(intrusion)}m_"
                f"{shoulder}_r{repetition:02d}"
            )
        else:
            repetition = 1
            case_id = f"unpaired_{index + 1:02d}"
        pairs.append(
            PairRecord(
                bag_path=bag.path if bag else None,
                log_path=log.path if log else None,
                bag_start_epoch=bag.start_epoch if bag else None,
                bag_duration_sec=bag.duration_sec if bag else None,
                distance=distance,
                intrusion=intrusion,
                shoulder=shoulder,
                repetition=repetition,
                case_id=case_id,
                pairing_status="OK" if not reason else "SETUP_INVALID",
                pairing_reason=";".join(reason),
            )
        )
    return pairs


def discover_records(root: Path) -> tuple[list[BagRecord], list[LogRecord]]:
    bags: list[BagRecord] = []
    for metadata in root.rglob("metadata.yaml"):
        if "analysis" in metadata.parts:
            continue
        try:
            bags.append(read_bag_metadata(metadata))
        except (OSError, ValueError):
            continue
    logs: list[LogRecord] = []
    for log in root.rglob("*.log"):
        if "analysis" in log.parts:
            continue
        try:
            logs.append(LogRecord(log, parse_launch_epoch(log)))
        except (OSError, ValueError):
            continue
    return bags, logs


def _timestamped_events(path: Path) -> list[tuple[float, str]]:
    if not path.exists():
        return []
    timestamp = re.compile(r"\[(\d+\.\d+)\]")
    return [
        (float(match.group(1)), line)
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
        if (match := timestamp.search(line)) is not None
    ]


def parse_log_events(path: Path) -> dict:
    """Extract state, avoidance, invalid-trajectory and runtime-MRM evidence."""

    lines = _timestamped_events(path)
    state_re = re.compile(
        r"autoware_state.*?(?:AutowareState:\s*)?([A-Za-z]+)\s*=>\s*([A-Za-z]+)",
        re.IGNORECASE,
    )
    state_events: list[dict] = []
    for timestamp, line in lines:
        match = state_re.search(line)
        if match:
            state_events.append(
                {"timestamp": timestamp, "from": match.group(1), "to": match.group(2)}
            )
    driving = next(
        (event["timestamp"] for event in state_events if event["to"] == "Driving"),
        None,
    )
    arrived = next(
        (
            event["timestamp"]
            for event in state_events
            if event["to"] == "ArrivedGoal"
            and driving is not None
            and event["timestamp"] >= driving
        ),
        None,
    )
    mrm_events = [
        {"timestamp": timestamp, "line": line}
        for timestamp, line in lines
        if re.search(r"(?:EMERGENCY_STOP|COMFORTABLE_STOP) is operated", line, re.I)
    ]
    runtime_mrm = [
        event for event in mrm_events if driving is not None and event["timestamp"] >= driving
    ]
    invalid_events = [
        {"timestamp": timestamp, "line": line}
        for timestamp, line in lines
        if re.search(r"Invalid Trajectory detected", line, re.I)
    ]
    simple_lines = [
        {"timestamp": timestamp, "line": line}
        for timestamp, line in lines
        if re.search(r"\[SIMPLE_AVOIDANCE\].*avoidance path generated", line, re.I)
    ]
    last_state = state_events[-1]["to"] if state_events else None
    return {
        "state_events": state_events,
        "driving_epoch": driving,
        "arrived_goal_epoch": arrived,
        "last_state": last_state,
        "mrm_events": mrm_events,
        "runtime_mrm_events": runtime_mrm,
        "runtime_mrm_count": len(runtime_mrm),
        "invalid_trajectory_events": invalid_events,
        "invalid_trajectory_count": len(invalid_events),
        "simple_avoidance_events": simple_lines,
        "simple_avoidance_log_count": len(simple_lines),
    }


def classify_historical_result(evidence: dict) -> str:
    if not evidence.get("setup_valid", False):
        return "SETUP_INVALID"
    if evidence.get("runtime_mrm_count", 0) > 0:
        return "MRM"
    if evidence.get("invalid_trajectory_count", 0) > 0:
        return "INVALID_TRAJECTORY"
    if evidence.get("min_collision_clearance_m") is not None and evidence["min_collision_clearance_m"] < 0:
        return "COLLISION"
    phase = evidence.get("obstacle_phase_result")
    if phase == "NOT_CROSSED" or phase == "NO_AVOIDANCE_TRIGGER":
        return "STOPPED_BEFORE_OBSTACLE"
    if phase != "PASSED_CLEAR":
        return "STOPPED_AFTER_OBSTACLE"
    if not evidence.get("arrived_goal", False):
        return "INCOMPLETE_OBSERVATION"
    return "PASS"


def historical_failure_reason(result: str, phase: str) -> str:
    return {
        "SETUP_INVALID": "PAIRING_OR_PERCEPTION_INVALID",
        "MRM": "RUNTIME_MRM_DETECTED",
        "INVALID_TRAJECTORY": "INVALID_TRAJECTORY_DETECTED",
        "COLLISION": "NEGATIVE_RECTANGLE_CLEARANCE",
        "STOPPED_BEFORE_OBSTACLE": (
            "NO_SIMPLE_AVOIDANCE_TRIGGER" if phase == "NO_AVOIDANCE_TRIGGER"
            else "OBSTACLE_NOT_CROSSED"
        ),
        "STOPPED_AFTER_OBSTACLE": "OBSTACLE_CROSSED_BUT_ROUTE_NOT_CONFIRMED",
        "INCOMPLETE_OBSERVATION": "ARRIVED_GOAL_NOT_RECORDED",
        "PASS": "",
    }.get(result, result)


def max_stopped_duration(
    samples: list[tuple[int, float, float, float, float, float]],
    start_time: int | None,
) -> float:
    """Return the longest contiguous near-zero-speed interval after start_time."""

    active = [sample for sample in samples if start_time is None or sample[0] >= start_time]
    stopped_start: int | None = None
    previous: int | None = None
    maximum = 0.0
    for timestamp, _s, _x, _y, _yaw, speed in active:
        contiguous = previous is not None and timestamp - previous <= 2_000_000_000
        if abs(speed) <= 0.2:
            if stopped_start is None or not contiguous:
                stopped_start = timestamp
        elif stopped_start is not None and previous is not None:
            maximum = max(maximum, (previous - stopped_start) / 1e9)
            stopped_start = None
        previous = timestamp
    if stopped_start is not None and previous is not None:
        maximum = max(maximum, (previous - stopped_start) / 1e9)
    return maximum


def stopped_event_count(
    samples: list[tuple[int, float, float, float, float, float]],
    start_time: int | None,
    minimum_duration_sec: float = 0.5,
) -> int:
    """Count distinct near-zero-speed intervals longer than the noise threshold."""

    active = [sample for sample in samples if start_time is None or sample[0] >= start_time]
    stopped_start: int | None = None
    previous: int | None = None
    count = 0
    for timestamp, _s, _x, _y, _yaw, speed in active:
        contiguous = previous is not None and timestamp - previous <= 2_000_000_000
        if abs(speed) <= 0.2:
            if stopped_start is None or not contiguous:
                stopped_start = timestamp
        elif stopped_start is not None and previous is not None:
            if (previous - stopped_start) / 1e9 >= minimum_duration_sec:
                count += 1
            stopped_start = None
        previous = timestamp
    if stopped_start is not None and previous is not None:
        if (previous - stopped_start) / 1e9 >= minimum_duration_sec:
            count += 1
    return count


def _empty_bag_result(pair: PairRecord, reason: str) -> dict:
    return {
        "case_id": pair.case_id,
        "module": "Simple Avoidance",
        "repetition": pair.repetition,
        "longitudinal_m": pair.distance,
        "intrusion_m": pair.intrusion,
        "shoulder": pair.shoulder,
        "pairing_status": pair.pairing_status,
        "pairing_reason": pair.pairing_reason,
        "setup_valid": False,
        "overall_result": "SETUP_INVALID",
        "result": "SETUP_INVALID",
        "failure_reason": reason,
        "obstacle_phase_result": "UNKNOWN_NOT_RECORDED",
        "obstacle_passed": "UNKNOWN_NOT_RECORDED",
        "return_completed": "UNKNOWN_NOT_RECORDED",
        "stop_count": None,
        "remarks": reason,
        "bag_path": str(pair.bag_path) if pair.bag_path else None,
        "log_path": str(pair.log_path) if pair.log_path else None,
    }


def analyze_pair(pair: PairRecord) -> tuple[dict, list[dict]]:
    if pair.bag_path is None or pair.log_path is None:
        return _empty_bag_result(pair, "MISSING_BAG_OR_LOG"), []
    dbs = sorted(pair.bag_path.glob("*.db3"))
    if not dbs:
        return _empty_bag_result(pair, "MISSING_ROSBAG_DATABASE"), []

    types = {
        "/simulation/dummy_perception_publisher/object_info": DummyObject,
        "/simulation/dummy_perception_publisher/output/debug/ground_truth_objects": TrackedObjects,
        "/simulation/debug/ground_truth_objects": TrackedObjects,
        "/perception/object_recognition/tracking/objects": TrackedObjects,
        "/perception/object_recognition/objects": PredictedObjects,
        "/localization/kinematic_state": Odometry,
        "/planning/planning_factors/simple_avoidance": PlanningFactorArray,
        "/planning/planning_factors/obstacle_stop": PlanningFactorArray,
        "/planning/planning_factors/dynamic_obstacle_stop": PlanningFactorArray,
    }
    dummy_ids: set[str] = set()
    dummy_add_total = 0
    clear_count = 0
    max_gt = max_tracking = max_predicted = 0
    duplicate_gt = False
    obstacle_center: tuple[float, float] | None = None
    obstacle_yaw: float | None = None
    obstacle_lon: float | None = None
    obstacle_length = OBSTACLE_LENGTH_M
    obstacle_width = 1.5
    first_add_time: int | None = None
    ego_samples: list[tuple[int, float, float, float, float, float]] = []
    simple_factor_count = obstacle_stop_factor_count = dynamic_stop_factor_count = 0

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(dbs[0]), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""),
        )
        while reader.has_next():
            topic, data, bag_time = reader.read_next()
            msg_type = types.get(topic)
            if msg_type is None:
                continue
            msg = deserialize_message(data, msg_type)
            if topic == "/simulation/dummy_perception_publisher/object_info":
                if msg.action == DummyObject.DELETEALL:
                    clear_count += 1
                    dummy_ids.clear()
                    dummy_add_total = 0
                    first_add_time = None
                    obstacle_center = obstacle_yaw = obstacle_lon = None
                    continue
                uid = uuid_hex(msg.id)
                if uid == "0" * 32:
                    continue
                dummy_ids.add(uid)
                if msg.action == DummyObject.ADD:
                    dummy_add_total += 1
                    if first_add_time is None:
                        first_add_time = bag_time
                pose = msg.initial_state.pose_covariance.pose
                if obstacle_center is None:
                    obstacle_center = (pose.position.x, pose.position.y)
                    obstacle_lon = longitudinal(pose.position.x, pose.position.y)
                    obstacle_yaw = quat_to_yaw(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    )
                    obstacle_length = float(msg.shape.dimensions.x) or obstacle_length
                    obstacle_width = float(msg.shape.dimensions.y) or obstacle_width
            elif topic in {
                "/simulation/dummy_perception_publisher/output/debug/ground_truth_objects",
                "/simulation/debug/ground_truth_objects",
            }:
                ids = [uuid_hex(item.object_id) for item in msg.objects]
                max_gt = max(max_gt, len(ids))
                duplicate_gt = duplicate_gt or len(ids) != len(set(ids))
            elif topic == "/perception/object_recognition/tracking/objects":
                max_tracking = max(max_tracking, len(msg.objects))
            elif topic == "/perception/object_recognition/objects":
                max_predicted = max(max_predicted, len(msg.objects))
            elif topic == "/localization/kinematic_state":
                pose = msg.pose.pose
                yaw = quat_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                ego_samples.append(
                    (
                        bag_time,
                        longitudinal(pose.position.x, pose.position.y),
                        pose.position.x,
                        pose.position.y,
                        yaw,
                        float(msg.twist.twist.linear.x),
                    )
                )
            elif topic == "/planning/planning_factors/simple_avoidance":
                if factor_has_behavior(msg, 4) or factor_has_behavior(msg, 5):
                    simple_factor_count += 1
            elif topic == "/planning/planning_factors/obstacle_stop":
                if factor_has_behavior(msg, 3):
                    obstacle_stop_factor_count += 1
            elif topic == "/planning/planning_factors/dynamic_obstacle_stop":
                if factor_has_behavior(msg, 3):
                    dynamic_stop_factor_count += 1
    except Exception as exc:  # rosbag corruption/type-support failures are setup errors
        return _empty_bag_result(pair, f"ROSBAG_ANALYSIS_ERROR:{type(exc).__name__}"), []

    log_events = parse_log_events(pair.log_path)
    simple_log_count = log_events["simple_avoidance_log_count"]
    simple_triggered = bool(simple_factor_count or simple_log_count)
    perception_valid = (
        len(dummy_ids) == 1
        and max_gt == 1
        and max_tracking <= 1
        and max_predicted <= 1
        and not duplicate_gt
        and obstacle_center is not None
    )
    min_clearance: float | None = None
    clearance_epoch: float | None = None
    max_lateral: float | None = None
    passed = False
    cross_epoch: float | None = None
    if obstacle_center is not None and obstacle_yaw is not None and obstacle_lon is not None:
        near = [
            sample for sample in ego_samples
            if obstacle_lon - 8.0 <= sample[1] <= obstacle_lon + 8.0
        ]
        if not near:
            near = ego_samples
        clearances = [
            rectangle_signed_clearance(
                sample[2], sample[3], sample[4],
                obstacle_center[0], obstacle_center[1], obstacle_yaw,
                obstacle_length, obstacle_width,
            )
            for sample in near
        ]
        if clearances:
            minimum_index = min(range(len(clearances)), key=clearances.__getitem__)
            min_clearance = clearances[minimum_index]
            clearance_epoch = near[minimum_index][0] / 1e9
        lateral_samples = [
            abs(lateral_offset(sample[2], sample[3]))
            for sample in near
        ]
        max_lateral = max(lateral_samples) if lateral_samples else None
        threshold = obstacle_lon + obstacle_length / 2.0 + VEHICLE_REAR_OVERHANG_M
        motion_start = first_add_time or 0
        if log_events["driving_epoch"] is not None:
            motion_start = max(motion_start, int(log_events["driving_epoch"] * 1e9))
        crossing = [
            sample for sample in ego_samples
            if sample[0] >= motion_start and sample[1] >= threshold
        ]
        if crossing:
            passed = True
            cross_epoch = crossing[0][0] / 1e9

    if not perception_valid:
        phase = "UNKNOWN_NOT_RECORDED"
    elif min_clearance is not None and min_clearance < 0.0:
        phase = "COLLISION"
    elif not simple_triggered:
        phase = "NO_AVOIDANCE_TRIGGER"
    elif not passed:
        phase = "NOT_CROSSED"
    else:
        phase = "PASSED_CLEAR"

    parameter_distance_error = (
        abs(obstacle_lon - pair.distance)
        if obstacle_lon is not None and pair.distance is not None
        else None
    )
    parameter_match = parameter_distance_error is not None and parameter_distance_error <= 0.05
    setup_valid = perception_valid and parameter_match and pair.pairing_status == "OK"
    evidence = {
        "setup_valid": setup_valid,
        "runtime_mrm_count": log_events["runtime_mrm_count"],
        "invalid_trajectory_count": log_events["invalid_trajectory_count"],
        "min_collision_clearance_m": min_clearance,
        "obstacle_phase_result": phase,
        "arrived_goal": log_events["arrived_goal_epoch"] is not None,
    }
    overall = classify_historical_result(evidence)
    motion_start = first_add_time
    if log_events["driving_epoch"] is not None:
        driving_ns = int(log_events["driving_epoch"] * 1e9)
        motion_start = max(motion_start or driving_ns, driving_ns)
    speeds = [
        sample[5]
        for sample in ego_samples
        if motion_start is None or sample[0] >= motion_start
    ]
    time_to_cross = (
        cross_epoch - log_events["driving_epoch"]
        if cross_epoch is not None and log_events["driving_epoch"] is not None
        else None
    )
    time_to_goal = (
        log_events["arrived_goal_epoch"] - log_events["driving_epoch"]
        if log_events["arrived_goal_epoch"] is not None and log_events["driving_epoch"] is not None
        else None
    )
    result = {
        "case_id": pair.case_id,
        "module": "Simple Avoidance",
        "repetition": pair.repetition,
        "longitudinal_m": pair.distance,
        "intrusion_m": pair.intrusion,
        "shoulder": pair.shoulder,
        "pairing_status": pair.pairing_status,
        "pairing_reason": pair.pairing_reason,
        "bag_start_epoch": pair.bag_start_epoch,
        "bag_duration_sec": pair.bag_duration_sec,
        "setup_valid": setup_valid,
        "overall_result": overall,
        "result": overall,
        "obstacle_phase_result": phase,
        "failure_reason": historical_failure_reason(overall, phase),
        "object_count": max_gt,
        "dummy_unique_ids": len(dummy_ids),
        "dummy_add_count_after_last_clear": dummy_add_total,
        "clear_count": clear_count,
        "perception_chain_valid": perception_valid,
        "duplicate_ground_truth_uuid": duplicate_gt,
        "max_tracking_count": max_tracking,
        "max_predicted_count": max_predicted,
        "simple_avoidance_triggered": simple_triggered,
        "simple_avoidance_factor_count": simple_factor_count,
        "simple_avoidance_log_count": simple_log_count,
        "obstacle_stop_status": "RECORDED_FACTOR" if obstacle_stop_factor_count else "UNKNOWN_NOT_RECORDED",
        "obstacle_stop_factor_count": obstacle_stop_factor_count,
        "dynamic_obstacle_stop_status": "RECORDED_FACTOR" if dynamic_stop_factor_count else "UNKNOWN_NOT_RECORDED",
        "dynamic_obstacle_stop_factor_count": dynamic_stop_factor_count,
        "clear_confirmation_status": "UNKNOWN_NOT_RECORDED",
        "obstacle_longitudinal_m": obstacle_lon,
        "obstacle_x": obstacle_center[0] if obstacle_center else None,
        "obstacle_y": obstacle_center[1] if obstacle_center else None,
        "obstacle_yaw": obstacle_yaw,
        "obstacle_length_m": obstacle_length,
        "obstacle_width_m": obstacle_width,
        "parameter_distance_error_m": parameter_distance_error,
        "parameter_match": parameter_match,
        "vehicle_passed": passed,
        "goal_reached": log_events["arrived_goal_epoch"] is not None,
        "driving_epoch": log_events["driving_epoch"],
        "arrived_goal_epoch": log_events["arrived_goal_epoch"],
        "last_autoware_state": log_events["last_state"],
        "obstacle_crossed_epoch": cross_epoch,
        "min_collision_clearance_m": min_clearance,
        "min_collision_clearance_epoch": clearance_epoch,
        "max_abs_lateral_offset_near_obstacle_m": max_lateral,
        "base_link_to_vehicle_center_m": BASE_LINK_TO_VEHICLE_CENTER_M,
        "max_ego_longitudinal_m": max((sample[1] for sample in ego_samples), default=None),
        "min_speed_mps": min(speeds) if speeds else None,
        "max_speed_mps": max(speeds) if speeds else None,
        "max_stopped_sec": max_stopped_duration(ego_samples, motion_start),
        "stop_count": stopped_event_count(ego_samples, motion_start),
        "speed_mps": max(speeds) if speeds else None,
        "time_to_obstacle_cross_sec": time_to_cross,
        "time_to_goal_sec": time_to_goal,
        "invalid_trajectory_count": log_events["invalid_trajectory_count"],
        "mrm_count": log_events["runtime_mrm_count"],
        "mrm_startup_count": len(log_events["mrm_events"]) - log_events["runtime_mrm_count"],
        "log_path": str(pair.log_path),
        "bag_path": str(pair.bag_path),
    }

    result["obstacle_passed"] = "YES" if passed else "NO"
    result["return_completed"] = (
        "YES" if log_events["arrived_goal_epoch"] is not None else "UNKNOWN_NOT_RECORDED"
    )
    remarks = [result["failure_reason"]]
    if log_events["invalid_trajectory_count"]:
        remarks.append("INVALID_TRAJECTORY_DETECTED")
    if simple_triggered:
        remarks.append("SIMPLE_AVOIDANCE_TRIGGERED")
    if result["obstacle_stop_status"] == "UNKNOWN_NOT_RECORDED":
        remarks.append("OBSTACLE_STOP_STATUS_UNKNOWN")
    result["remarks"] = ";".join(item for item in remarks if item)

    # A second pass over odometry is unnecessary for the core result, but the
    # speed topic is not always available in the historical schema.  Keep the
    # fields explicit and derive them from the available kinematic state when
    # possible by reopening only the small odometry stream.
    result["ego_sample_count"] = len(ego_samples)
    result["observation_end_epoch"] = (
        max((sample[0] for sample in ego_samples), default=0) / 1e9
        if ego_samples else None
    )
    timeline: list[dict] = []
    if first_add_time is not None:
        timeline.append({"timestamp": first_add_time / 1e9, "event": "OBSTACLE_ADD"})
    if log_events["driving_epoch"] is not None:
        timeline.append({"timestamp": log_events["driving_epoch"], "event": "DRIVING"})
    for event in log_events["simple_avoidance_events"]:
        timeline.append({"timestamp": event["timestamp"], "event": "SIMPLE_AVOIDANCE"})
    for event in log_events["invalid_trajectory_events"]:
        timeline.append({"timestamp": event["timestamp"], "event": "INVALID_TRAJECTORY"})
    for event in log_events["runtime_mrm_events"]:
        timeline.append({"timestamp": event["timestamp"], "event": "MRM"})
    if cross_epoch is not None:
        timeline.append({"timestamp": cross_epoch, "event": "OBSTACLE_CROSSED"})
    if log_events["arrived_goal_epoch"] is not None:
        timeline.append({"timestamp": log_events["arrived_goal_epoch"], "event": "ARRIVED_GOAL"})
    timeline.sort(key=lambda event: event["timestamp"])
    return result, timeline


CASE_FIELDS = [
    "case_id", "module", "repetition", "longitudinal_m", "intrusion_m", "shoulder",
    "pairing_status", "pairing_reason", "bag_start_epoch", "bag_duration_sec",
    "setup_valid", "overall_result", "obstacle_phase_result", "failure_reason",
    "object_count", "dummy_unique_ids", "dummy_add_count_after_last_clear",
    "clear_count", "perception_chain_valid", "duplicate_ground_truth_uuid",
    "max_tracking_count", "max_predicted_count", "simple_avoidance_triggered",
    "simple_avoidance_factor_count", "simple_avoidance_log_count",
    "obstacle_stop_status", "obstacle_stop_factor_count",
    "dynamic_obstacle_stop_status", "dynamic_obstacle_stop_factor_count",
    "clear_confirmation_status", "obstacle_longitudinal_m", "obstacle_x",
    "obstacle_y", "obstacle_yaw", "obstacle_length_m", "obstacle_width_m",
    "parameter_distance_error_m", "parameter_match",
    "vehicle_passed", "obstacle_passed", "goal_reached", "return_completed",
    "driving_epoch", "arrived_goal_epoch",
    "last_autoware_state", "obstacle_crossed_epoch", "min_collision_clearance_m",
    "min_collision_clearance_epoch", "max_abs_lateral_offset_near_obstacle_m",
    "base_link_to_vehicle_center_m", "max_ego_longitudinal_m", "min_speed_mps",
    "max_speed_mps", "speed_mps", "max_stopped_sec", "stop_count",
    "time_to_obstacle_cross_sec",
    "time_to_goal_sec", "ego_sample_count", "observation_end_epoch",
    "invalid_trajectory_count", "mrm_count",
    "mrm_startup_count", "log_path", "bag_path",
]


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in fields} for row in rows)


def aggregate(rows: list[dict]) -> list[dict]:
    groups: defaultdict[tuple[float, float, str], list[dict]] = defaultdict(list)
    for row in rows:
        if row.get("longitudinal_m") is not None and row.get("intrusion_m") is not None:
            groups[(row["longitudinal_m"], row["intrusion_m"], row["shoulder"])].append(row)
    output: list[dict] = []
    for (distance, intrusion, shoulder), group in sorted(groups.items()):
        counts = Counter(row.get("overall_result", "SETUP_INVALID") for row in group)
        clearances = [
            row["min_collision_clearance_m"]
            for row in group
            if isinstance(row.get("min_collision_clearance_m"), (float, int))
        ]
        crossed = [
            row["obstacle_crossed_epoch"] - row["driving_epoch"]
            for row in group
            if row.get("obstacle_crossed_epoch") is not None and row.get("driving_epoch") is not None
        ]
        arrived = [
            row["arrived_goal_epoch"] - row["driving_epoch"]
            for row in group
            if row.get("arrived_goal_epoch") is not None and row.get("driving_epoch") is not None
        ]
        output.append({
            "longitudinal_m": distance,
            "intrusion_m": intrusion,
            "shoulder": shoulder,
            "observed_repetitions": len(group),
            "pass_count": counts.get("PASS", 0),
            "pass_rate": round(counts.get("PASS", 0) / len(group), 4) if group else None,
            "obstacle_clear_count": sum(
                row.get("obstacle_phase_result") == "PASSED_CLEAR" for row in group
            ),
            "min_clearance_m": min(clearances) if clearances else None,
            "median_clearance_m": median(clearances) if clearances else None,
            "median_time_to_cross_sec": median(crossed) if crossed else None,
            "median_time_to_goal_sec": median(arrived) if arrived else None,
            **{f"count_{name.lower()}": counts.get(name, 0) for name in RESULTS},
        })
    return output


def _format_summary_decimal(value: object) -> str:
    """Render summary decimal metrics with stable 16-place precision."""

    try:
        return f"{Decimal(str(value)):.16f}"
    except (InvalidOperation, TypeError, ValueError):
        return str(value)


def write_summary(path: Path, rows: list[dict], grouped: list[dict]) -> None:
    distances = sorted({row["longitudinal_m"] for row in rows if row.get("longitudinal_m") is not None})
    intrusions = sorted({row["intrusion_m"] for row in rows if row.get("intrusion_m") is not None})
    by_key = {(row["longitudinal_m"], row["intrusion_m"]): row for row in grouped}
    overview: defaultdict[tuple[str, str, float], list[dict]] = defaultdict(list)
    for row in rows:
        if row.get("intrusion_m") is not None:
            overview[(row.get("module", "Simple Avoidance"), row.get("shoulder", "right"), row["intrusion_m"])].append(row)
    lines = [
        "# 20260911 同车道障碍物绕障实测结果表",
        "",
        "> 本文件由 `/home/nvidia/autoware/log/20260911` 中已有 rosbag 和 launch 日志离线生成，未重新启动 Autoware。",
        "> `result` 是历史仿真观测结论，不是理论预测；缺少记录的指标显示为 `UNKNOWN_NOT_RECORDED`。",
        "",
        "## 测试矩阵概览",
        "",
        "| module | shoulder | intrusion_m | longitudinal_points | case_count | obstacle_pass_count | route_arrived_count | PASS_count |",
        "|---|---|---:|---|---:|---:|---:|---:|",
    ]
    for (module, shoulder, intrusion), group in sorted(overview.items()):
        points = ", ".join(f"{value:g}" for value in sorted({row["longitudinal_m"] for row in group}))
        lines.append(
            f"| {module} | {shoulder} | {intrusion:.3f} | {points} | {len(group)} | "
            f"{sum(row.get('obstacle_phase_result') == 'PASSED_CLEAR' for row in group)} | "
            f"{sum(bool(row.get('goal_reached')) for row in group)} | "
            f"{sum(row.get('overall_result') == 'PASS' for row in group)} |"
        )

    lines += [
        "",
        "## 最终结果矩阵",
        "",
        "| 纵向\\侵入量 | " + " | ".join(f"{value:g} m" for value in intrusions) + " |",
        "|---:|" + "---:|" * len(intrusions),
    ]
    for distance in distances:
        cells = []
        for intrusion in intrusions:
            row = by_key.get((distance, intrusion))
            if row is None:
                cells.append("-")
            else:
                cells.append(
                    f"{row['pass_count']}/{row['observed_repetitions']} PASS; "
                    f"{row['obstacle_clear_count']}/{row['observed_repetitions']} obstacle_passed"
                )
        lines.append(f"| {distance:g} m | " + " | ".join(cells) + " |")
    lines += [
        "",
        "## 详细结果表",
        "",
        "| case_id | module | speed_mps | longitudinal_m | intrusion_m | shoulder | result | obstacle_phase_result | min_clearance_m | obstacle_passed | return_completed | stop_count | MRM_count | Invalid_Trajectory_count |",
        "|---|---|---:|---:|---:|---|---|---|---:|---|---|---:|---:|---:|",
    ]
    for row in rows:
        speed = row.get("speed_mps")
        speed_text = (
            _format_summary_decimal(speed)
            if speed not in (None, "")
            else "UNKNOWN_NOT_RECORDED"
        )
        clearance = row.get("min_collision_clearance_m")
        clearance_text = (
            _format_summary_decimal(clearance)
            if clearance not in (None, "")
            else "UNKNOWN_NOT_RECORDED"
        )
        lines.append(
            f"| {row.get('case_id')} | {row.get('module')} | {speed_text} | "
            f"{row.get('longitudinal_m')} | {row.get('intrusion_m')} | {row.get('shoulder')} | "
            f"{row.get('overall_result')} | {row.get('obstacle_phase_result')} | {clearance_text} | "
            f"{row.get('obstacle_passed')} | "
            f"{row.get('return_completed')} | {row.get('stop_count')} | {row.get('mrm_count')} | "
            f"{row.get('invalid_trajectory_count')} |"
        )
    lines += [
        "",
        "## 字段说明",
        "",
        "- `result`：最终实测分类；只有无运行期 MRM/Invalid Trajectory、无碰撞、完成越障且日志到达终点才为 `PASS`。",
        "- `obstacle_phase_result`：单独表示障碍物阶段；`PASSED_CLEAR` 不等价于整条路线 PASS。",
        "- `speed_mps`：该案例记录到的最大纵向速度；完整速度范围见 `cases.csv` 的 `min_speed_mps/max_speed_mps`。",
        "- `min_clearance_m`：按 BYD 车辆包络和修正后的 OBB 分离轴判定计算，正值表示无重叠。",
        "- `stop_count`：速度低于 0.2 m/s 且持续至少 0.5 s 的独立停车区间数量；`max_stopped_sec` 在 `cases.csv` 中给出。",
        "",
        "## 测试限制",
        "",
        "- 历史 bag 未录制 `/api/routing/state`，因此只有日志明确出现 `Driving => ArrivedGoal` 时才认定到达终点。",
        "- 历史 bag 未包含 Obstacle Stop/Dynamic Obstacle Stop planning factor；模块状态和清障确认不做推断。",
        "- 历史参数点重复次数不均，聚合分母使用实际观测次数，不补造重复试验；完整 4×3 矩阵需按当前 runner 补测。",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(root: Path, output: Path, shoulder: str) -> list[dict]:
    bags, logs = discover_records(root)
    pairs = pair_records(bags, logs, shoulder)
    output.mkdir(parents=True, exist_ok=True)
    write_csv(
        output / "pairing.csv",
        [
            {
                **asdict(pair),
                "bag_path": str(pair.bag_path) if pair.bag_path else None,
                "log_path": str(pair.log_path) if pair.log_path else None,
            }
            for pair in pairs
        ],
        [
            "case_id", "repetition", "distance", "intrusion", "shoulder",
            "bag_start_epoch", "bag_duration_sec", "bag_path", "log_path",
            "pairing_status", "pairing_reason",
        ],
    )
    rows: list[dict] = []
    cases_root = output / "cases"
    for pair in pairs:
        result, timeline = analyze_pair(pair)
        case_dir = cases_root / pair.case_id
        case_dir.mkdir(parents=True, exist_ok=True)
        (case_dir / "result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        (case_dir / "timeline.json").write_text(
            json.dumps(timeline, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        rows.append(result)
    rows.sort(key=lambda row: (row.get("longitudinal_m") is None, row.get("longitudinal_m") or 0, row.get("intrusion_m") or 0, row.get("repetition", 0)))
    write_csv(output / "cases.csv", rows, CASE_FIELDS)
    grouped = aggregate(rows)
    write_csv(output / "aggregate.csv", grouped, list(grouped[0]) if grouped else ["longitudinal_m", "intrusion_m", "shoulder", "observed_repetitions"])
    write_summary(output / "summary.md", rows, grouped)
    (output / "summary.json").write_text(
        json.dumps({"cases": rows, "aggregate": grouped}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze historical obstacle bags and launch logs")
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--shoulder", choices=("left", "right"), default="right")
    args = parser.parse_args()
    rows = run(args.root, args.output, args.shoulder)
    print(f"analyzed {len(rows)} historical cases")
    print(args.output / "cases.csv")
    print(args.output / "aggregate.csv")
    print(args.output / "summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
