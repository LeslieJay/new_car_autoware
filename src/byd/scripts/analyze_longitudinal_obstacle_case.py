#!/usr/bin/env python3
"""Analyze one longitudinal obstacle-avoidance simulation bag."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import rosbag2_py
from autoware_internal_planning_msgs.msg import PlanningFactorArray
from autoware_perception_msgs.msg import PredictedObjects, TrackedObjects
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tier4_simulation_msgs.msg import DummyObject


START_X = 279.52191162109375
START_Y = -33.24296569824219
START_OX = -0.00024534598908445367
START_OY = -0.00041156653669880767
START_OZ = -0.5120475691803335
START_OW = 0.8589569589419735
OBSTACLE_LENGTH_M = 2.0
VEHICLE_LENGTH_M = 1.008 + 0.546 + 0.676
VEHICLE_WIDTH_M = 0.835 + 0.235 + 0.235


def quat_to_yaw(ox: float, oy: float, oz: float, ow: float) -> float:
    return math.atan2(2.0 * (ow * oz + ox * oy), 1.0 - 2.0 * (oy * oy + oz * oz))


def longitudinal(x: float, y: float) -> float:
    yaw = quat_to_yaw(START_OX, START_OY, START_OZ, START_OW)
    return (x - START_X) * math.cos(yaw) + (y - START_Y) * math.sin(yaw)


def uuid_hex(value) -> str:
    return bytes(value.uuid).hex()


def factor_has_behavior(msg: PlanningFactorArray, behavior: int) -> bool:
    return any(factor.behavior == behavior for factor in msg.factors)


def rectangle_signed_clearance(
    ego_x: float,
    ego_y: float,
    ego_yaw: float,
    obstacle_x: float,
    obstacle_y: float,
    obstacle_yaw: float,
    obstacle_length: float,
    obstacle_width: float,
) -> float:
    """Return positive separation, or negative overlap, for two OBBs.

    The vehicle dimensions match the BYD vehicle_info parameters.  The base_link
    pose is used as the vehicle rectangle center, which is conservative for this
    collision sanity check and is preferable to treating the object as a point.
    """
    ego_axes = ((math.cos(ego_yaw), math.sin(ego_yaw)), (-math.sin(ego_yaw), math.cos(ego_yaw)))
    obstacle_axes = (
        (math.cos(obstacle_yaw), math.sin(obstacle_yaw)),
        (-math.sin(obstacle_yaw), math.cos(obstacle_yaw)),
    )
    delta = (obstacle_x - ego_x, obstacle_y - ego_y)
    ego_half = (VEHICLE_LENGTH_M / 2.0, VEHICLE_WIDTH_M / 2.0)
    obstacle_half = (obstacle_length / 2.0, obstacle_width / 2.0)
    clearances = []
    for axis in (*ego_axes, *obstacle_axes):
        projection = abs(delta[0] * axis[0] + delta[1] * axis[1])
        ego_radius = sum(
            half * abs(axis[0] * basis[0] + axis[1] * basis[1])
            for half, basis in zip(ego_half, ego_axes)
        )
        obstacle_radius = sum(
            half * abs(axis[0] * basis[0] + axis[1] * basis[1])
            for half, basis in zip(obstacle_half, obstacle_axes)
        )
        clearances.append(projection - ego_radius - obstacle_radius)
    return max(clearances)


def count_log_matches(paths: list[Path], pattern: str) -> int:
    compiled = re.compile(pattern, re.IGNORECASE)
    return sum(
        len(compiled.findall(path.read_text(encoding="utf-8", errors="replace")))
        for path in paths
        if path.exists()
    )


def count_log_matches_since(
    paths: list[Path], pattern: str, start_epoch: float | None
) -> int:
    """Count timestamped log matches after the active test window starts.

    Launch logs contain startup MRM transitions before localization/route setup.
    Those are initialization events, not the obstacle case under test. Once the
    monitor starts, any MRM operation is a real case failure.
    """
    if start_epoch is None:
        return count_log_matches(paths, pattern)
    compiled = re.compile(pattern, re.IGNORECASE)
    timestamped = re.compile(r"\[(\d+\.\d+)\]")
    count = 0
    for path in paths:
        if not path.exists():
            continue
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            match = timestamped.search(line)
            if match is not None and float(match.group(1)) < start_epoch:
                continue
            if compiled.search(line):
                count += 1
    return count


def analyze(args: argparse.Namespace) -> dict:
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
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )

    dummy_adds: dict[str, list[int]] = {}
    dummy_ids: set[str] = set()
    obstacle_lon: float | None = None
    obstacle_center: tuple[float, float] | None = None
    obstacle_yaw: float | None = None
    obstacle_length = OBSTACLE_LENGTH_M
    obstacle_width = 1.5
    first_add_time: int | None = None
    max_ground_truth_count = 0
    ground_truth_duplicate = False
    max_tracking_count = 0
    max_predicted_count = 0
    has_simple_avoidance = False
    has_simple_stop = False
    has_obstacle_stop = False
    has_dynamic_obstacle_stop = False
    ego_samples: list[tuple[int, float]] = []
    ego_poses: list[tuple[int, float, float, float]] = []

    while reader.has_next():
        topic, data, bag_time = reader.read_next()
        msg_type = types.get(topic)
        if msg_type is None:
            continue
        msg = deserialize_message(data, msg_type)

        if topic == "/simulation/dummy_perception_publisher/object_info":
            uid = uuid_hex(msg.id)
            if msg.action == DummyObject.DELETEALL:
                # A bag can start while a previous host test still has an object
                # alive.  --clear-first deliberately defines the new test epoch;
                # ignore pre-clear ADDs and geometry when evaluating this case.
                dummy_adds.clear()
                dummy_ids.clear()
                first_add_time = None
                obstacle_lon = None
                obstacle_center = None
                obstacle_yaw = None
                obstacle_length = OBSTACLE_LENGTH_M
                obstacle_width = 1.5
                continue
            if uid != "0" * 32:
                dummy_ids.add(uid)
                pose = msg.initial_state.pose_covariance.pose.position
                if obstacle_lon is None:
                    obstacle_lon = longitudinal(pose.x, pose.y)
                    obstacle_center = (pose.x, pose.y)
                    obstacle_yaw = quat_to_yaw(
                        msg.initial_state.pose_covariance.pose.orientation.x,
                        msg.initial_state.pose_covariance.pose.orientation.y,
                        msg.initial_state.pose_covariance.pose.orientation.z,
                        msg.initial_state.pose_covariance.pose.orientation.w,
                    )
                    obstacle_length = float(msg.shape.dimensions.x) or OBSTACLE_LENGTH_M
                    obstacle_width = float(msg.shape.dimensions.y) or obstacle_width
                if msg.action == DummyObject.ADD:
                    dummy_adds.setdefault(uid, []).append(bag_time)
                    if first_add_time is None:
                        first_add_time = bag_time
        elif topic in {
            "/simulation/dummy_perception_publisher/output/debug/ground_truth_objects",
            "/simulation/debug/ground_truth_objects",
        }:
            ids = [uuid_hex(obj.object_id) for obj in msg.objects]
            max_ground_truth_count = max(max_ground_truth_count, len(ids))
            ground_truth_duplicate = ground_truth_duplicate or len(ids) != len(set(ids))
        elif topic.endswith("tracking/objects"):
            max_tracking_count = max(max_tracking_count, len(msg.objects))
        elif topic == "/perception/object_recognition/objects":
            max_predicted_count = max(max_predicted_count, len(msg.objects))
        elif topic == "/localization/kinematic_state":
            ego_x = msg.pose.pose.position.x
            ego_y = msg.pose.pose.position.y
            ego_yaw = quat_to_yaw(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            ego_samples.append((bag_time, longitudinal(ego_x, ego_y)))
            ego_poses.append((bag_time, ego_x, ego_y, ego_yaw))
        elif topic == "/planning/planning_factors/simple_avoidance":
            has_simple_avoidance = has_simple_avoidance or factor_has_behavior(msg, 4) or factor_has_behavior(msg, 5)
            has_simple_stop = has_simple_stop or factor_has_behavior(msg, 3)
        elif topic == "/planning/planning_factors/obstacle_stop":
            has_obstacle_stop = has_obstacle_stop or factor_has_behavior(msg, 3)
        elif topic == "/planning/planning_factors/dynamic_obstacle_stop":
            has_dynamic_obstacle_stop = has_dynamic_obstacle_stop or factor_has_behavior(msg, 3)

    object_count_ok = (
        len(dummy_ids) == 1
        and max_ground_truth_count == 1
        and max_tracking_count <= 1
        and max_predicted_count <= 1
        and not ground_truth_duplicate
    )
    vehicle_passed = False
    pre_obstacle_motion = False
    max_ego_lon = None
    min_collision_clearance = None
    if obstacle_lon is not None and ego_samples:
        max_ego_lon = max(value for _, value in ego_samples)
        vehicle_passed = max_ego_lon >= obstacle_lon + OBSTACLE_LENGTH_M / 2.0 + 1.0
        if first_add_time is not None:
            if args.motion_start_epoch is None:
                before_add = [
                    value for timestamp, value in ego_samples if timestamp < first_add_time
                ]
            else:
                motion_start_time = int(args.motion_start_epoch * 1e9)
                before_add = [
                    value
                    for timestamp, value in ego_samples
                    if motion_start_time <= timestamp < first_add_time
                ]
            if before_add:
                pre_obstacle_motion = max(abs(value) for value in before_add) > 1.0
    if obstacle_center is not None and obstacle_yaw is not None and ego_poses:
        min_collision_clearance = min(
            rectangle_signed_clearance(
                ego_x,
                ego_y,
                ego_yaw,
                obstacle_center[0],
                obstacle_center[1],
                obstacle_yaw,
                obstacle_length,
                obstacle_width,
            )
            for _, ego_x, ego_y, ego_yaw in ego_poses
        )
    collision_free = min_collision_clearance is not None and min_collision_clearance >= 0.0

    monitor_data: dict = {}
    if args.monitor is not None and args.monitor.exists():
        monitor_data = json.loads(args.monitor.read_text(encoding="utf-8"))
        if "vehicle_passed" in monitor_data:
            vehicle_passed = bool(monitor_data["vehicle_passed"])

    log_paths = [Path(path) for path in args.log]
    dummy_add_log_count = count_log_matches(log_paths, r"DummyObject\.ADD")
    dummy_add_total = sum(len(times) for times in dummy_adds.values())
    add_count_ok = (dummy_add_total == 1) or (
        dummy_add_total == 0 and dummy_add_log_count == 1
    )
    simple_avoidance_log_count = count_log_matches(
        log_paths,
        r"\[SIMPLE_AVOIDANCE\].*(?:avoidance path generated|success blocked)",
    )
    has_simple_avoidance = has_simple_avoidance or simple_avoidance_log_count > 0
    invalid_trajectory_count = count_log_matches(log_paths, r"Invalid Trajectory detected")
    mrm_start_epoch = monitor_data.get("start_epoch")
    if mrm_start_epoch is None:
        mrm_start_epoch = args.motion_start_epoch
    mrm_operation_count = count_log_matches_since(
        log_paths,
        r"(?:EMERGENCY_STOP|COMFORTABLE_STOP) is operated",
        float(mrm_start_epoch) if mrm_start_epoch is not None else None,
    )

    # The historical distance sweep deliberately disabled Obstacle Stop and
    # therefore treated any obstacle-stop planning factor as a failure.  The
    # obstacle-stop acceptance profile enables that module, so a stop factor is
    # expected/allowed as long as the vehicle eventually clears the obstacle.
    module_names: set[str] = set()
    modules_checked = args.launch_modules is not None
    if modules_checked and args.launch_modules.exists():
        module_text = args.launch_modules.read_text(encoding="utf-8", errors="replace")
        module_names = set(
            re.findall(
                r"autoware::motion_velocity_planner::[A-Za-z0-9_]+", module_text
            )
        )
    obstacle_stop_module_loaded = (
        not modules_checked
        or "autoware::motion_velocity_planner::ObstacleStopModule" in module_names
    )
    dynamic_obstacle_stop_module_loaded = (
        "autoware::motion_velocity_planner::DynamicObstacleStopModule" in module_names
    )
    module_configuration_ok = obstacle_stop_module_loaded and not (
        dynamic_obstacle_stop_module_loaded
    )
    obstacle_stop_behavior_ok = args.allow_obstacle_stop or not has_obstacle_stop

    if not args.bag.exists() or max_ground_truth_count == 0:
        result = "INCOMPLETE"
    elif (
        object_count_ok
        and args.clear_confirmed
        and add_count_ok
        and has_simple_avoidance
        and vehicle_passed
        and obstacle_stop_behavior_ok
        and not has_dynamic_obstacle_stop
        and module_configuration_ok
        and collision_free
        and invalid_trajectory_count == 0
        and mrm_operation_count == 0
        and not pre_obstacle_motion
    ):
        result = "PASS"
    else:
        result = "FAIL"

    return {
        "longitudinal_m": args.distance,
        "intrusion_m": 0.5,
        "shoulder": "right",
        "label": "unknown",
        "result": result,
        "clear_confirmed": args.clear_confirmed,
        "object_count": max_ground_truth_count,
        "dummy_unique_ids": len(dummy_ids),
        "dummy_add_counts": {uid: len(times) for uid, times in dummy_adds.items()},
        "dummy_add_log_count": dummy_add_log_count,
        "dummy_add_total": dummy_add_total,
        "add_count_ok": add_count_ok,
        "duplicate_ground_truth_uuid": ground_truth_duplicate,
        "max_tracking_count": max_tracking_count,
        "max_predicted_count": max_predicted_count,
        "has_simple_avoidance": has_simple_avoidance,
        "simple_avoidance_log_count": simple_avoidance_log_count,
        "has_simple_stop": has_simple_stop,
        "has_obstacle_stop": has_obstacle_stop,
        "has_dynamic_obstacle_stop": has_dynamic_obstacle_stop,
        "obstacle_stop_allowed": args.allow_obstacle_stop,
        "obstacle_stop_behavior_ok": obstacle_stop_behavior_ok,
        "obstacle_stop_module_loaded": obstacle_stop_module_loaded,
        "dynamic_obstacle_stop_module_loaded": dynamic_obstacle_stop_module_loaded,
        "module_configuration_ok": module_configuration_ok,
        "invalid_trajectory_count": invalid_trajectory_count,
        "mrm_operation_count": mrm_operation_count,
        "mrm_count_start_epoch": mrm_start_epoch,
        "obstacle_longitudinal_m": obstacle_lon,
        "max_ego_longitudinal_m": max_ego_lon,
        "vehicle_passed": vehicle_passed,
        "obstacle_crossed": bool(monitor_data.get("obstacle_crossed", vehicle_passed)),
        "goal_reached": bool(monitor_data.get("goal_reached", monitor_data.get("monitor_result") == "ARRIVED")),
        "obstacle_crossed_epoch": monitor_data.get("obstacle_crossed_epoch"),
        "goal_reached_epoch": monitor_data.get("goal_reached_epoch"),
        "route_state": monitor_data.get("route_state"),
        "route_set_seen": monitor_data.get("route_set_seen", False),
        "min_collision_clearance_m": min_collision_clearance,
        "collision_free": collision_free,
        "pre_obstacle_motion": pre_obstacle_motion,
        "early_stop_reason": monitor_data.get("early_stop_reason"),
        "timeout": bool(monitor_data.get("timeout", False)),
        "monitor_result": monitor_data.get("monitor_result"),
        "bag_path": str(args.bag),
        "log_paths": [str(path) for path in log_paths],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--distance", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--log", type=Path, action="append", default=[])
    parser.add_argument("--monitor", type=Path, default=None)
    parser.add_argument(
        "--launch-modules",
        type=Path,
        default=None,
        help="launch_modules 参数输出；提供后校验 ObstacleStop 已启用且 DynamicObstacleStop 未启用",
    )
    parser.add_argument(
        "--allow-obstacle-stop",
        action="store_true",
        help="允许 Obstacle Stop 规划因子出现（安全绕障验收配置）",
    )
    parser.add_argument(
        "--motion-start-epoch",
        type=float,
        default=None,
        help="只从初始化完成后的 wall-clock epoch 统计 ADD 前车辆移动",
    )
    parser.add_argument(
        "--clear-confirmed",
        action="store_true",
        help="调用方已在该测试点结束后确认三条感知链路为空",
    )
    args = parser.parse_args()
    result = analyze(args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["result"] != "INCOMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
