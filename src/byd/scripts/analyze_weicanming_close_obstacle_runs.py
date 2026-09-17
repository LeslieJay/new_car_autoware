#!/usr/bin/env python3
"""Generate an evidence-oriented table for the 20260917 close-obstacle runs.

The launch logs are the primary source for the planner-facing values.  Bag
data is only used to supplement object classification, object geometry and the
vehicle/object envelope clearance when those values are not present in logs.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import re
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

import rosbag2_py
from autoware_perception_msgs.msg import PredictedObjects, TrackedObjects
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from tier4_simulation_msgs.msg import DummyObject

from analyze_longitudinal_obstacle_case import (
    lateral_offset,
    longitudinal,
    quat_to_yaw,
    rectangle_signed_clearance,
    uuid_hex,
)


LABELS = {
    0: "UNKNOWN",
    1: "CAR",
    2: "TRUCK",
    3: "BUS",
    4: "TRAILER",
    5: "MOTORCYCLE",
    6: "BICYCLE",
    7: "PEDESTRIAN",
    8: "ANIMAL",
    9: "HAZARD",
    10: "OVER_DRIVABLE",
    11: "UNDER_DRIVABLE",
}

TYPE_ZH = {
    "UNKNOWN": "未知/未分类",
    "CAR": "小汽车",
    "TRUCK": "卡车",
    "BUS": "公交车",
    "TRAILER": "挂车",
    "MOTORCYCLE": "摩托车",
    "BICYCLE": "自行车",
    "PEDESTRIAN": "行人",
    "ANIMAL": "动物",
    "HAZARD": "危险物",
    "OVER_DRIVABLE": "可驶越障碍物",
    "UNDER_DRIVABLE": "可穿越障碍物",
}

TRACKING_TOPIC = "/perception/object_recognition/tracking/objects"
PREDICTED_TOPIC = "/perception/object_recognition/objects"
GROUND_TRUTH_TOPICS = {
    "/simulation/dummy_perception_publisher/output/debug/ground_truth_objects",
    "/simulation/debug/ground_truth_objects",
}
ODOM_TOPIC = "/localization/kinematic_state"
DUMMY_TOPIC = "/simulation/dummy_perception_publisher/object_info"


def number(value: float | None, digits: int = 2) -> str:
    if value is None:
        return "未记录"
    return f"{value:.{digits}f}"


def stamp_from_log(line: str) -> float | None:
    match = re.search(r"\[(\d+\.\d+)\]", line)
    return float(match.group(1)) if match else None


def parse_log(path: Path) -> dict:
    lock_re = re.compile(
        r"active target locked uuid=([0-9a-f]+) lon=([+-]?\d+(?:\.\d+)?)m "
        r"lat=([+-]?\d+(?:\.\d+)?)m"
    )
    path_re = re.compile(
        r"avoidance path generated shift=([+-]?\d+(?:\.\d+)?) "
        r"target_lon=([+-]?\d+(?:\.\d+)?) target_lat=([+-]?\d+(?:\.\d+)?) "
        r"required_clearance=([+-]?\d+(?:\.\d+)?) .*?"
        r"dist_to_shift_end=([+-]?\d+(?:\.\d+)?) "
        r"dist_to_obstacle=([+-]?\d+(?:\.\d+)?)"
    )
    safety_re = re.compile(
        r"safety stop before target uuid=([0-9a-f]+).*?target_lon=([+-]?\d+(?:\.\d+)?)m "
        r"target_half_length=([+-]?\d+(?:\.\d+)?)m"
    )
    blocked_re = re.compile(
        r"success blocked: active=(\d+) passed=(\d+) shift_lines=(\d+)"
    )
    events: list[tuple[float, str]] = []
    locks: list[dict] = []
    paths: list[dict] = []
    safety: list[dict] = []
    blocked: list[dict] = []
    pass_through = 0
    arrived_goal = 0
    invalid_trajectory = 0
    mrm = 0
    process_died = 0
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        timestamp = stamp_from_log(line)
        if timestamp is None:
            continue
        events.append((timestamp, line))
        if match := lock_re.search(line):
            locks.append(
                {
                    "time": timestamp,
                    "uuid": match.group(1),
                    "lon_m": float(match.group(2)),
                    "lat_m": float(match.group(3)),
                }
            )
        if match := path_re.search(line):
            paths.append(
                {
                    "time": timestamp,
                    "shift_m": float(match.group(1)),
                    "target_lon_m": float(match.group(2)),
                    "target_lat_m": float(match.group(3)),
                    "required_clearance_m": float(match.group(4)),
                    "dist_to_shift_end_m": float(match.group(5)),
                    "dist_to_obstacle_m": float(match.group(6)),
                }
            )
        if match := safety_re.search(line):
            safety.append(
                {
                    "time": timestamp,
                    "uuid": match.group(1),
                    "target_lon_m": float(match.group(2)),
                    "target_half_length_m": float(match.group(3)),
                }
            )
        if match := blocked_re.search(line):
            blocked.append(
                {
                    "time": timestamp,
                    "active": int(match.group(1)),
                    "passed": int(match.group(2)),
                    "shift_lines": int(match.group(3)),
                }
            )
        if "pass-through reason=" in line:
            pass_through += 1
        if "AutowareState: Driving => ArrivedGoal" in line:
            arrived_goal += 1
        if "Invalid Trajectory detected" in line:
            invalid_trajectory += 1
        if re.search(r"(?:EMERGENCY_STOP|COMFORTABLE_STOP) is operated", line):
            mrm += 1
        if "process has died" in line:
            process_died += 1
    first_active = min(
        [item["time"] for item in locks + paths], default=None
    )
    runtime_mrm = sum(
        1
        for timestamp, line in events
        if first_active is not None
        and timestamp >= first_active
        and re.search(r"(?:EMERGENCY_STOP|COMFORTABLE_STOP) is operated", line)
    )
    return {
        "log_path": str(path),
        "locks": locks,
        "paths": paths,
        "safety": safety,
        "blocked": blocked,
        "pass_through_count": pass_through,
        "arrived_goal_count": arrived_goal,
        "invalid_trajectory_count": invalid_trajectory,
        "mrm_count": mrm,
        "runtime_mrm_count": runtime_mrm,
        "process_died_count": process_died,
    }


def pose_from_object(obj):
    kinematics = obj.kinematics
    pose = getattr(kinematics, "pose_with_covariance", None)
    if pose is None:
        pose = getattr(kinematics, "initial_pose_with_covariance", None)
    if pose is None:
        return None
    pose = pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        quat_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def object_label(obj) -> tuple[str, float]:
    classifications = list(getattr(obj, "classification", []))
    if not classifications:
        return "UNKNOWN", 0.0
    selected = max(classifications, key=lambda item: float(item.probability))
    return LABELS.get(int(selected.label), "UNKNOWN"), float(selected.probability)


def object_dimensions(obj) -> tuple[float, float]:
    shape = getattr(obj, "shape", None)
    if shape is None:
        return 0.0, 0.0
    length = float(shape.dimensions.x)
    width = float(shape.dimensions.y)
    if length > 0.0 and width > 0.0:
        return length, width
    points = list(getattr(shape.footprint, "points", []))
    if points:
        xs = [float(point.x) for point in points]
        ys = [float(point.y) for point in points]
        return max(xs) - min(xs), max(ys) - min(ys)
    return length, width


def speed_from_object(obj) -> float:
    twist = getattr(obj.kinematics, "twist_with_covariance", None)
    if twist is None:
        return 0.0
    return math.hypot(float(twist.twist.linear.x), float(twist.twist.linear.y))


def nearest_ego(ego_times: list[float], ego_samples: list[dict], timestamp: float) -> dict | None:
    if not ego_times:
        return None
    index = bisect.bisect_left(ego_times, timestamp)
    candidates = []
    if index < len(ego_times):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    best = min(candidates, key=lambda item: abs(ego_times[item] - timestamp))
    return ego_samples[best] if abs(ego_times[best] - timestamp) <= 0.5 else None


def summarize_observations(
    observations: list[dict],
    ego_samples: list[dict],
    log_half_length: float | None,
    fallback_label: str = "UNKNOWN",
) -> dict:
    if not observations:
        return {
            "label": fallback_label,
            "probability": None,
            "length_m": None,
            "width_m": None,
            "min_clearance_m": None,
            "min_center_distance_m": None,
            "obs_count": 0,
        }
    labels = Counter(item["label"] for item in observations)
    label = max(labels, key=labels.get)
    probabilities = [item["probability"] for item in observations if item["probability"] > 0.0]
    lengths = [item["length_m"] for item in observations if item["length_m"] > 0.0]
    widths = [item["width_m"] for item in observations if item["width_m"] > 0.0]
    length = median(lengths) if lengths else (2.0 * log_half_length if log_half_length else 0.0)
    width = median(widths) if widths else 1.5
    clearances: list[float] = []
    center_distances: list[float] = []
    ego_times = [item["time"] for item in ego_samples]
    for item in observations:
        ego = nearest_ego(ego_times, ego_samples, item["time"])
        if ego is None:
            continue
        clearances.append(
            rectangle_signed_clearance(
                ego["x"],
                ego["y"],
                ego["yaw"],
                item["x"],
                item["y"],
                item["yaw"],
                length,
                width,
            )
        )
        center_distances.append(math.hypot(item["x"] - ego["x"], item["y"] - ego["y"]))
    return {
        "label": label,
        "probability": median(probabilities) if probabilities else None,
        "length_m": length if length > 0.0 else None,
        "width_m": width if width > 0.0 else None,
        "min_clearance_m": min(clearances) if clearances else None,
        "min_center_distance_m": min(center_distances) if center_distances else None,
        "obs_count": len(observations),
    }


def read_bag(path: Path, logs: dict) -> dict:
    dbs = sorted(path.glob("*.db3"))
    if not dbs:
        return {"bag_path": str(path), "error": "没有 db3 文件"}
    types = {
        TRACKING_TOPIC: TrackedObjects,
        PREDICTED_TOPIC: PredictedObjects,
        ODOM_TOPIC: Odometry,
        DUMMY_TOPIC: DummyObject,
        **{topic: TrackedObjects for topic in GROUND_TRUTH_TOPICS},
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(dbs[0]), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    ego_samples: list[dict] = []
    observations: dict[str, list[dict]] = defaultdict(list)
    all_observations: list[dict] = []
    auxiliary: list[dict] = []
    ground_truth: list[dict] = []
    dummy_shapes: list[dict] = []
    while reader.has_next():
        topic, data, bag_time_ns = reader.read_next()
        msg_type = types.get(topic)
        if msg_type is None:
            continue
        timestamp = bag_time_ns / 1e9
        msg = deserialize_message(data, msg_type)
        if topic == ODOM_TOPIC:
            pose = msg.pose.pose
            ego_samples.append(
                {
                    "time": timestamp,
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "yaw": quat_to_yaw(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                    "speed_mps": float(msg.twist.twist.linear.x),
                }
            )
            continue
        if topic == DUMMY_TOPIC:
            if msg.action in (DummyObject.ADD, DummyObject.MODIFY):
                pose = msg.initial_state.pose_covariance.pose
                label = LABELS.get(int(msg.classification.label), "UNKNOWN")
                dummy_shapes.append(
                    {
                        "label": label,
                        "probability": float(msg.classification.probability),
                        "length_m": float(msg.shape.dimensions.x),
                        "width_m": float(msg.shape.dimensions.y),
                        "x": float(pose.position.x),
                        "y": float(pose.position.y),
                        "yaw": quat_to_yaw(
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ),
                    }
                )
            continue
        if topic in GROUND_TRUTH_TOPICS:
            for obj in msg.objects:
                pose = pose_from_object(obj)
                if pose is None:
                    continue
                length, width = object_dimensions(obj)
                label, probability = object_label(obj)
                ground_truth.append(
                    {
                        "time": timestamp,
                        "uuid": uuid_hex(obj.object_id),
                        "label": label,
                        "probability": probability,
                        "length_m": length,
                        "width_m": width,
                        "x": pose[0],
                        "y": pose[1],
                        "yaw": pose[2],
                    }
                )
            continue
        if topic not in (TRACKING_TOPIC, PREDICTED_TOPIC):
            continue
        for obj in msg.objects:
            object_id = getattr(obj, "object_id", None)
            if object_id is None:
                continue
            pose = pose_from_object(obj)
            if pose is None:
                continue
            label, probability = object_label(obj)
            length, width = object_dimensions(obj)
            observation = {
                "time": timestamp,
                "uuid": uuid_hex(object_id),
                "label": label,
                "probability": probability,
                "length_m": length,
                "width_m": width,
                "x": pose[0],
                "y": pose[1],
                "yaw": pose[2],
                "speed_mps": speed_from_object(obj),
                "topic": topic,
            }
            observations[observation["uuid"]].append(observation)
            all_observations.append(observation)
    ego_samples.sort(key=lambda item: item["time"])
    locks = logs["locks"]
    target_uuids = list(dict.fromkeys(item["uuid"] for item in locks))
    target_details: list[dict] = []
    for target_uuid in target_uuids:
        target_logs = [item for item in locks if item["uuid"] == target_uuid]
        target_safety = [item for item in logs["safety"] if item["uuid"] == target_uuid]
        half_length = median(
            [item["target_half_length_m"] for item in target_safety]
        ) if target_safety else None
        summary = summarize_observations(observations.get(target_uuid, []), ego_samples, half_length)
        summary.update(
            {
                "uuid": target_uuid,
                "lock_count": len(target_logs),
                "first_lock_lon_m": target_logs[0]["lon_m"],
                "first_lock_lat_m": target_logs[0]["lat_m"],
                "target_half_length_m": half_length,
                "log_safety_stop": bool(target_safety),
                "source": "bag+log" if observations.get(target_uuid) else "log",
            }
        )
        target_details.append(summary)
    # If the planner never locked a target, retain the closest persistent bag
    # object near the route as a supplementary candidate, without calling it a
    # confirmed obstacle target.
    if not target_details:
        candidate_groups: dict[str, list[dict]] = defaultdict(list)
        for item in all_observations:
            lon = longitudinal(item["x"], item["y"])
            lat = lateral_offset(item["x"], item["y"])
            if -10.0 <= lon <= 35.0 and abs(lat) <= 5.0:
                candidate_groups[item["uuid"]].append(item)
        candidates = sorted(
            candidate_groups.values(),
            key=lambda group: (-len(group), min(abs(lateral_offset(item["x"], item["y"])) for item in group)),
        )
        if candidates:
            group = candidates[0]
            summary = summarize_observations(group, ego_samples, None)
            first = min(group, key=lambda item: item["time"])
            summary.update(
                {
                    "uuid": first["uuid"],
                    "lock_count": 0,
                    "first_lock_lon_m": longitudinal(first["x"], first["y"]),
                    "first_lock_lat_m": lateral_offset(first["x"], first["y"]),
                    "target_half_length_m": None,
                    "log_safety_stop": False,
                    "source": "bag candidate",
                }
            )
            target_details.append(summary)
    # Use one-object ground truth/dummy geometry only to fill missing type and
    # dimensions.  Classification is intentionally kept UNKNOWN if that is
    # what the recording actually contains.
    if len(target_details) == 1:
        target = target_details[0]
        aux = ground_truth or dummy_shapes
        if aux:
            if target["label"] == "UNKNOWN":
                aux_labels = [item["label"] for item in aux]
                target["label"] = Counter(aux_labels).most_common(1)[0][0]
                target["probability"] = median(
                    [item["probability"] for item in aux if item["probability"] > 0.0]
                ) if any(item["probability"] > 0.0 for item in aux) else None
            # Ground truth is the preferred geometry when present.  Tracking
            # shape estimates can fluctuate or be zero-filled for a static
            # obstacle, while the recorded ground-truth shape is fixed.
            lengths = [item["length_m"] for item in aux if item["length_m"] > 0.0]
            widths = [item["width_m"] for item in aux if item["width_m"] > 0.0]
            if lengths and ground_truth:
                target["length_m"] = median(lengths)
            elif lengths and (target["length_m"] is None or target["length_m"] <= 0.0):
                target["length_m"] = median(lengths)
            if widths and ground_truth:
                target["width_m"] = median(widths)
            elif widths and (target["width_m"] is None or target["width_m"] <= 0.0):
                target["width_m"] = median(widths)
    return {
        "bag_path": str(path),
        "target_details": target_details,
        "ego_sample_count": len(ego_samples),
        "tracking_object_count": len(all_observations),
        "ground_truth_count": len(ground_truth),
        "dummy_shape_count": len(dummy_shapes),
        "error": None,
    }


def target_text(targets: list[dict]) -> str:
    if not targets:
        return "未找到目标"
    pieces = []
    for target in targets:
        label = TYPE_ZH.get(target["label"], target["label"])
        pieces.append(
            f"{label}/{target['uuid'][:8]}"
            f"(锁定{target['lock_count']}次, lon={number(target['first_lock_lon_m'])}m, "
            f"lat={number(target['first_lock_lat_m'])}m)"
        )
    return "; ".join(pieces)


def choose_primary(targets: list[dict]) -> dict | None:
    if not targets:
        return None
    confirmed = [target for target in targets if target["lock_count"] > 0]
    return min(confirmed or targets, key=lambda target: abs(target["first_lock_lon_m"]))


def make_row(index: int, log_path: Path, bag_path: Path, logs: dict, bag: dict) -> dict:
    targets = bag.get("target_details", [])
    primary = choose_primary(targets)
    first_lock = logs["locks"][0] if logs["locks"] else None
    first_path = logs["paths"][0] if logs["paths"] else None
    all_clearances = [
        target["min_clearance_m"]
        for target in targets
        if target.get("min_clearance_m") is not None
    ]
    all_center_distances = [
        target["min_center_distance_m"]
        for target in targets
        if target.get("min_center_distance_m") is not None
    ]
    if logs["pass_through_count"]:
        result = "触发穿越保护/未完成绕障"
    elif logs["runtime_mrm_count"] or logs["invalid_trajectory_count"]:
        result = "异常停车/接管"
    elif logs["locks"] and logs["blocked"]:
        result = "已触发绕障"
    elif logs["locks"]:
        result = "已锁定目标"
    else:
        result = "未触发绕障"
    if logs["arrived_goal_count"]:
        result += "+到达终点"
    elif logs["process_died_count"]:
        result += "+日志未记录到达终点"
    type_names = sorted({TYPE_ZH.get(target["label"], target["label"]) for target in targets})
    if not type_names:
        obstacle_type = "未记录"
    elif type_names == ["未知/未分类"]:
        obstacle_type = "未知/未分类（bag有目标但未给出类别）"
    else:
        obstacle_type = "/".join(type_names)
    min_half = None
    if logs["safety"]:
        min_half = min(item["target_half_length_m"] for item in logs["safety"])
    target_size = None
    if primary and primary.get("length_m") is not None and primary.get("width_m") is not None:
        target_size = f"{primary['length_m']:.2f}*{primary['width_m']:.2f}"
    return {
        "序号": index,
        "测试组": log_path.stem,
        "日志": log_path.name,
        "bag": bag_path.name,
        "障碍物类型": obstacle_type,
        "目标数/锁定目标": len(targets),
        "首个锁定纵向距离_m": first_lock["lon_m"] if first_lock else (primary.get("first_lock_lon_m") if primary else None),
        "首个锁定横向J距离_m": first_lock["lat_m"] if first_lock else (primary.get("first_lock_lat_m") if primary else None),
        "最小包络间距_m": min(all_clearances) if all_clearances else None,
        "最小中心距离_m": min(all_center_distances) if all_center_distances else None,
        "目标尺寸_m": target_size,
        "目标长度_m": primary.get("length_m") if primary else None,
        "目标宽度_m": primary.get("width_m") if primary else None,
        "日志目标半长_m": min_half,
        "日志最小required_clearance_m": min((item["required_clearance_m"] for item in logs["paths"]), default=None),
        "日志最小dist_to_obstacle_m": min((item["dist_to_obstacle_m"] for item in logs["paths"]), default=None),
        "日志最大锁定横向J距离_m": max((abs(item["lat_m"]) for item in logs["locks"]), default=None),
        "避障路径输出次数": len(logs["paths"]),
        "安全停车日志次数": len(logs["safety"]),
        "结果": result,
        "运行期MRM次数": logs["runtime_mrm_count"],
        "InvalidTrajectory次数": logs["invalid_trajectory_count"],
        "目标明细": target_text(targets),
        "数据来源说明": "日志为主；类别/几何/最小包络间距由 bag 补充" if targets else "日志为主；bag未找到可匹配目标",
        "备注": "; ".join(
            item for item in [
                "bag无metadata.yaml但直接读取db3" if not (bag_path / "metadata.yaml").exists() else "",
                "日志存在进程退出记录" if logs["process_died_count"] else "",
                "bag未找到分类" if primary and primary["label"] == "UNKNOWN" else "",
            ] if item
        ),
    }


FIELDS = [
    "序号", "日志", "bag", "目标数/锁定目标",
    "首个锁定纵向距离_m", "首个锁定横向J距离_m", "最小包络间距_m", "最小中心距离_m",
    "目标尺寸_m", "目标明细",
]


def render_markdown(rows: list[dict], path: Path) -> None:
    lines = [
        "# 2026-09-17 近距离绕障测试结果记录表",
        "",
        "> 统计范围：`/home/nvidia/autoware/log/20260917/weicanming`。字段提取优先使用 launch log；日志没有记录的障碍物类别、目标几何和车辆-障碍物包络间距由 bag 补充。",
        "> `最小包络间距_m` 是按车辆与目标 OBB 的保守包络计算的分离间隙，正值表示有间隙，负值表示包络重叠；不是激光点到障碍物表面的原始距离。",
        "> `首个锁定横向J距离_m` 沿用日志的 signed lateral 值；负号表示目标位于路线右侧（以当前日志坐标约定为准）。",
        "> `目标尺寸_m` 使用 `长*宽` 格式，例如 `2.00*1.00`。",
        "",
        "| " + " | ".join(FIELDS) + " |",
        "|" + "|".join("---" for _ in FIELDS) + "|",
    ]
    for row in rows:
        values = []
        for field in FIELDS:
            value = row.get(field)
            if isinstance(value, float):
                values.append(f"{value:.2f}")
            else:
                values.append("" if value is None else str(value).replace("|", "/"))
        lines.append("| " + " | ".join(values) + " |")
    lines += [
        "",
        "## 口径说明",
        "",
        "- 日志中的 `target_lon/target_lat`、`required_clearance`、`dist_to_obstacle` 反映规划模块计算值；其中 `dist_to_obstacle` 是纵向可用空间，不等同于车辆与障碍物的实际最小间距。",
        "- bag 中若分类字段为 UNKNOWN，表格保留为“未知/未分类”，不根据尺寸主观猜测障碍物类别。",
        "- 3 个 bag（100035、105340、111600）没有 `metadata.yaml`，但 db3 可读，已直接按数据库分析并在备注标出。",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    logs = sorted(args.root.glob("*.log"))
    bags = sorted(args.root.glob("*_bag"))
    if len(logs) != len(bags):
        raise SystemExit(f"日志数量 {len(logs)} 与 bag 数量 {len(bags)} 不一致")
    rows = []
    raw = []
    for index, (log_path, bag_path) in enumerate(zip(logs, bags), start=1):
        log_data = parse_log(log_path)
        try:
            bag_data = read_bag(bag_path, log_data)
        except Exception as exc:
            bag_data = {"bag_path": str(bag_path), "target_details": [], "error": f"{type(exc).__name__}: {exc}"}
        row = make_row(index, log_path, bag_path, log_data, bag_data)
        if bag_data.get("error"):
            row["备注"] = (row["备注"] + "; " if row["备注"] else "") + f"bag读取错误: {bag_data['error']}"
        rows.append(row)
        raw.append({"log": log_data, "bag": bag_data, "row": row})
        print(f"[{index}/{len(logs)}] {log_path.name} -> {bag_path.name}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    with (args.output_dir / "近距离绕障测试结果记录表.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    render_markdown(rows, args.output_dir / "近距离绕障测试结果记录表.md")
    (args.output_dir / "近距离绕障测试结果原始证据.json").write_text(
        json.dumps(raw, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(args.output_dir / "近距离绕障测试结果记录表.md")
    print(args.output_dir / "近距离绕障测试结果记录表.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
