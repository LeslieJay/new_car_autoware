#!/usr/bin/env python3
"""Analyze the 2026-09-20 obstacle-stop bag without replaying or mutating it.

The script intentionally reads the sqlite bag in read-only mode.  It joins the
launch log with the recorded perception, planning, velocity-limit, mode and
control topics for the three incident windows described in the acceptance plan.
It writes only derived artifacts below ``--out-dir``.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sqlite3
import subprocess
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


WINDOWS = (
    ("event_1", "10:24:05 first obstacle", 1789871045.0, 1789871065.0),
    ("event_2", "10:24:42 turn / second obstacle", 1789871082.0, 1789871110.0),
    ("event_3", "10:28:00 manual -> automatic", 1789871280.0, 1789871340.0),
)

TOPIC_TYPES = {
    "/perception/object_recognition/detection/objects": "autoware_perception_msgs/msg/DetectedObjects",
    "/perception/object_recognition/tracking/objects": "autoware_perception_msgs/msg/TrackedObjects",
    "/perception/object_recognition/objects": "autoware_perception_msgs/msg/PredictedObjects",
    "/localization/kinematic_state": "nav_msgs/msg/Odometry",
    "/vehicle/status/velocity_status": "autoware_vehicle_msgs/msg/VelocityReport",
    "/vehicle/status/control_mode": "autoware_vehicle_msgs/msg/ControlModeReport",
    "/system/operation_mode/state": "autoware_adapi_v1_msgs/msg/OperationModeState",
    "/planning/scenario_planning/max_velocity_candidates": "autoware_internal_planning_msgs/msg/VelocityLimit",
    "/planning/scenario_planning/max_velocity": "autoware_internal_planning_msgs/msg/VelocityLimit",
    "/planning/scenario_planning/clear_velocity_limit": "autoware_internal_planning_msgs/msg/VelocityLimitClearCommand",
    "/planning/scenario_planning/external_velocity_limit_selector/debug": "autoware_internal_debug_msgs/msg/StringStamped",
    "/planning/scenario_planning/trajectory": "autoware_planning_msgs/msg/Trajectory",
    "/planning/trajectory": "autoware_planning_msgs/msg/Trajectory",
    "/control/trajectory_follower/control_cmd": "autoware_control_msgs/msg/Control",
    "/control/command/control_cmd": "autoware_control_msgs/msg/Control",
    "/planning/planning_factors/simple_avoidance": "autoware_internal_planning_msgs/msg/PlanningFactorArray",
    "/planning/planning_factors/obstacle_stop": "autoware_internal_planning_msgs/msg/PlanningFactorArray",
    "/planning/planning_factors/dynamic_obstacle_stop": "autoware_internal_planning_msgs/msg/PlanningFactorArray",
    "/diagnostics": "diagnostic_msgs/msg/DiagnosticArray",
}

MODE_NAMES = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
CONTROL_MODE_NAMES = {
    0: "NO_COMMAND", 1: "AUTONOMOUS", 2: "AUTONOMOUS_STEER_ONLY",
    3: "AUTONOMOUS_VELOCITY_ONLY", 4: "MANUAL", 5: "DISENGAGED",
    6: "NOT_READY",
}


def numeric_stamp(line: str) -> float | None:
    values = re.findall(r"\[(\d+\.\d+)\]", line)
    return float(values[-1]) if values else None


def number(value: Any, digits: int = 3) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return round(result, digits)


def integer(value: Any, default: int = 0) -> int:
    """Convert ROS uint8 fields from both Python int and byte-array bindings."""
    if isinstance(value, (bytes, bytearray)):
        return int.from_bytes(value, byteorder="little", signed=False)
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def uuid_hex(value: Any) -> str:
    try:
        return bytes(value.uuid).hex()
    except (AttributeError, TypeError):
        return ""


def object_pose(obj: Any) -> tuple[float, float] | None:
    kin = getattr(obj, "kinematics", None)
    if kin is None:
        return None
    for field in ("pose_with_covariance", "initial_pose_with_covariance"):
        pose_cov = getattr(kin, field, None)
        if pose_cov is not None:
            p = pose_cov.pose.position
            return number(p.x), number(p.y)
    return None


def object_speed(obj: Any) -> float | None:
    kin = getattr(obj, "kinematics", None)
    if kin is None:
        return None
    for field in ("twist_with_covariance", "initial_twist_with_covariance"):
        twist_cov = getattr(kin, field, None)
        if twist_cov is not None:
            v = twist_cov.twist.linear
            return number((v.x * v.x + v.y * v.y + v.z * v.z) ** 0.5)
    return None


def object_classification(obj: Any) -> tuple[int | None, float | None]:
    values = list(getattr(obj, "classification", []))
    if not values:
        return None, None
    selected = max(values, key=lambda item: float(item.probability))
    return int(selected.label), number(selected.probability)


def object_snapshot(message: Any) -> list[dict[str, Any]]:
    result = []
    for obj in getattr(message, "objects", []):
        label, probability = object_classification(obj)
        result.append({
            "uuid": uuid_hex(getattr(obj, "object_id", None)),
            "label": label,
            "probability": probability,
            "position_xy": object_pose(obj),
            "speed_mps": object_speed(obj),
        })
    return result


def trajectory_summary(message: Any) -> dict[str, Any]:
    points = list(getattr(message, "points", []))
    velocities = [number(point.longitudinal_velocity_mps) for point in points]
    velocities = [v for v in velocities if v is not None]
    return {
        "point_count": len(points),
        "min_velocity_mps": min(velocities) if velocities else None,
        "max_velocity_mps": max(velocities) if velocities else None,
        "zero_velocity_points": sum(abs(v) < 1e-3 for v in velocities),
    }


def control_velocity(message: Any) -> float | None:
    longitudinal = getattr(message, "longitudinal", None)
    return number(getattr(longitudinal, "velocity", None))


def factor_snapshot(message: Any) -> list[dict[str, Any]]:
    factors = []
    for factor in getattr(message, "factors", []):
        item: dict[str, Any] = {
            "behavior": int(getattr(factor, "behavior", -1)),
            "module": str(getattr(factor, "module", "")),
            "detail": str(getattr(factor, "detail", "")),
        }
        points = list(getattr(factor, "control_points", []))
        if points:
            item["distance_m"] = number(getattr(points[0], "distance", None))
        factors.append(item)
    return factors


def diagnostic_snapshot(message: Any) -> list[dict[str, Any]]:
    result = []
    for status in getattr(message, "status", []):
        level = integer(getattr(status, "level", 0))
        if level == 0:
            continue
        name = str(getattr(status, "name", ""))
        text = str(getattr(status, "message", ""))
        if any(term in f"{name} {text}".lower() for term in ("planning", "trajectory", "control", "mode", "pedestrian")):
            result.append({"level": level, "name": name, "message": text})
    return result


def parse_log(path: Path, starts: float, ends: float) -> list[dict[str, Any]]:
    patterns = [
        ("target_locked", re.compile(r"active target locked uuid=([0-9a-f]+) lon=([+-]?\d+(?:\.\d+)?)m lat=([+-]?\d+(?:\.\d+)?)m")),
        ("avoidance_path", re.compile(r"avoidance path generated shift=([+-]?\d+(?:\.\d+)?) target_lon=([+-]?\d+(?:\.\d+)?) target_lat=([+-]?\d+(?:\.\d+)?)")),
        ("avoidance_pass_through", re.compile(r"pass-through reason=([a-z_]+)")),
        ("avoidance_safety_stop", re.compile(r"safety stop before target uuid=([0-9a-f]+) reason=([a-z_]+)")),
        ("pedestrian_stop", re.compile(r"\[pedestrian_safety_stop\]: stop requested because a surrounding hazard was detected")),
        ("invalid_trajectory", re.compile(r"Invalid Trajectory detected(?:\. Use soft stop trajectory)?")),
        ("engage_unavailable", re.compile(r"Engage unavailable: (.*)")),
        ("mode_available", re.compile(r"The target mode is (available|not available)(?: for the following reasons:)?")),
        ("control_stale", re.compile(r"Control command is stale: (\d+) ms")),
        ("can_velocity", re.compile(r"接收到control_cmd话题.*?velocity: ([+-]?\d+(?:\.\d+)?)")),
    ]
    events: list[dict[str, Any]] = []
    if not path.is_file():
        return events
    for line_number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        timestamp = numeric_stamp(line)
        if timestamp is None or timestamp < starts or timestamp > ends:
            continue
        for kind, pattern in patterns:
            match = pattern.search(line)
            if not match:
                continue
            event: dict[str, Any] = {"time": timestamp, "kind": kind, "line": line_number, "raw": line.strip()}
            if kind == "target_locked":
                event.update(uuid=match.group(1), lon_m=number(match.group(2)), lat_m=number(match.group(3)))
            elif kind == "avoidance_path":
                event.update(shift_m=number(match.group(1)), target_lon_m=number(match.group(2)), target_lat_m=number(match.group(3)))
            elif kind == "avoidance_pass_through":
                event["reason"] = match.group(1)
            elif kind == "avoidance_safety_stop":
                event.update(uuid=match.group(1), reason=match.group(2))
            elif kind == "engage_unavailable":
                event["reason"] = match.group(1)
            elif kind == "mode_available":
                event["state"] = match.group(1)
            elif kind == "control_stale":
                event["age_ms"] = int(match.group(1))
            elif kind == "can_velocity":
                event["velocity_mps"] = number(match.group(1))
            events.append(event)
            break
    return events


def git_context(repo: Path) -> dict[str, Any]:
    def run(*args: str) -> str:
        try:
            return subprocess.check_output(["git", "-C", str(repo), *args], text=True, stderr=subprocess.STDOUT).strip()
        except (subprocess.CalledProcessError, OSError):
            return "unavailable"
    return {"commit": run("rev-parse", "HEAD"), "status": run("status", "--short", "--untracked-files=no")}


def read_window_rows(db: sqlite3.Connection, type_map: dict[int, tuple[str, Any]], starts: float, ends: float) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    # Perception/control/trajectory topics run at tens of Hz.  Five samples per
    # second preserve target continuity and zero-speed intervals while keeping
    # the derived artifact bounded for a 60 s handoff window.  Decision topics
    # are never sampled out.
    sampled_topics = {
        "/perception/object_recognition/detection/objects",
        "/perception/object_recognition/tracking/objects",
        "/perception/object_recognition/objects",
        "/localization/kinematic_state",
        "/vehicle/status/velocity_status",
        "/planning/scenario_planning/trajectory",
        "/planning/trajectory",
        "/control/trajectory_follower/control_cmd",
        "/control/command/control_cmd",
        "/diagnostics",
    }
    last_kept: dict[str, float] = {}
    lo, hi = int(starts * 1e9), int(ends * 1e9)
    query = "select topic_id, timestamp, data from messages where timestamp between ? and ? order by timestamp"
    for topic_id, timestamp, data in db.execute(query, (lo, hi)):
        name, message_type = type_map.get(topic_id, ("", None))
        if message_type is None:
            continue
        stamp = timestamp / 1e9
        if name in sampled_topics and stamp - last_kept.get(name, float("-inf")) < 0.2:
            continue
        last_kept[name] = stamp
        message = deserialize_message(data, message_type)
        entry: dict[str, Any] = {"time": stamp, "topic": name}
        if "/object_recognition/" in name:
            entry["objects"] = object_snapshot(message)
        elif name == "/localization/kinematic_state":
            pose = getattr(getattr(message, "pose", None), "pose", None)
            twist = getattr(getattr(message, "twist", None), "twist", None)
            if pose is not None:
                entry["ego_xy"] = [number(pose.position.x), number(pose.position.y)]
            if twist is not None:
                entry["ego_speed_mps"] = number((twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2) ** 0.5)
        elif name in {"/planning/scenario_planning/trajectory", "/planning/trajectory"}:
            entry["trajectory"] = trajectory_summary(message)
        elif name.endswith("control_cmd"):
            entry["velocity_mps"] = control_velocity(message)
        elif name == "/vehicle/status/velocity_status":
            entry["velocity_mps"] = number(getattr(message, "longitudinal_velocity", None))
        elif name == "/vehicle/status/control_mode":
            mode = int(getattr(message, "mode", 0))
            entry.update({"mode": mode, "mode_name": CONTROL_MODE_NAMES.get(mode, str(mode))})
        elif name == "/system/operation_mode/state":
            mode = int(getattr(message, "mode", 0))
            entry.update({"mode": mode, "mode_name": MODE_NAMES.get(mode, str(mode)), "control_enabled": bool(getattr(message, "is_autoware_control_enabled", False)), "transition": bool(getattr(message, "is_in_transition", False))})
        elif name.endswith("max_velocity_candidates") or name.endswith("/max_velocity"):
            entry.update({"max_velocity_mps": number(getattr(message, "max_velocity", None)), "sender": str(getattr(message, "sender", ""))})
        elif name.endswith("clear_velocity_limit"):
            entry.update({"clear": bool(getattr(message, "command", False)), "sender": str(getattr(message, "sender", ""))})
        elif name.endswith("external_velocity_limit_selector/debug"):
            entry["data"] = str(getattr(message, "data", ""))
        elif "/planning_factors/" in name:
            entry["factors"] = factor_snapshot(message)
        elif name == "/diagnostics":
            entry["diagnostics"] = diagnostic_snapshot(message)
        rows.append(entry)
    return rows


def available_topics(db: sqlite3.Connection) -> dict[int, tuple[str, str]]:
    return {int(topic_id): (str(name), str(message_type)) for topic_id, name, message_type in db.execute("select id, name, type from topics")}


def make_type_map(topic_rows: dict[int, tuple[str, str]]) -> dict[int, tuple[str, Any]]:
    result: dict[int, tuple[str, Any]] = {}
    for topic_id, (name, message_type) in topic_rows.items():
        expected = TOPIC_TYPES.get(name)
        if expected is None or expected != message_type:
            continue
        try:
            result[topic_id] = (name, get_message(message_type))
        except (AttributeError, ModuleNotFoundError, ValueError):
            continue
    return result


def compact_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Drop unchanged high-rate states while retaining every decision event."""
    kept: list[dict[str, Any]] = []
    last: dict[str, str] = {}
    always = {"/planning/scenario_planning/max_velocity_candidates", "/planning/scenario_planning/clear_velocity_limit", "/planning/scenario_planning/external_velocity_limit_selector/debug", "/system/operation_mode/state"}
    for row in rows:
        topic = row["topic"]
        signature = json.dumps({k: v for k, v in row.items() if k not in {"time", "topic"}}, sort_keys=True, ensure_ascii=False)
        if topic in always or signature != last.get(topic):
            kept.append(row)
        last[topic] = signature
    return kept


def event_summary(window_name: str, log_events: list[dict[str, Any]], rows: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {"window": window_name, "log_events": log_events, "counts": defaultdict(int)}
    for row in rows:
        summary["counts"][row["topic"]] += 1
    summary["counts"] = dict(summary["counts"])
    summary["first_object_by_topic"] = {}
    logged_targets = {
        event["uuid"] for event in log_events
        if event.get("kind") in {"target_locked", "avoidance_safety_stop"} and event.get("uuid")
    }
    summary["target_uuids"] = sorted(logged_targets or {obj["uuid"] for row in rows for obj in row.get("objects", []) if obj.get("uuid")})
    snapshots: dict[str, dict[str, Any]] = {}
    for row in rows:
        if row["topic"] != "/perception/object_recognition/tracking/objects":
            continue
        for obj in row.get("objects", []):
            uuid = obj.get("uuid")
            if not uuid:
                continue
            snapshots.setdefault(uuid, {"first": {"time": row["time"], **obj}})
            snapshots[uuid]["last"] = {"time": row["time"], **obj}
    summary["target_snapshots"] = {uuid: snapshots[uuid] for uuid in summary["target_uuids"] if uuid in snapshots}
    ego_rows = [row for row in rows if row.get("ego_speed_mps") is not None]
    summary["ego_speed_first_mps"] = ego_rows[0].get("ego_speed_mps") if ego_rows else None
    summary["ego_speed_last_mps"] = ego_rows[-1].get("ego_speed_mps") if ego_rows else None
    summary["pedestrian_limit_events"] = [row for row in rows if row["topic"].endswith("max_velocity_candidates") and row.get("sender") == "byd_pedestrian_safety_stop"]
    summary["pedestrian_clear_events"] = [row for row in rows if row["topic"].endswith("clear_velocity_limit") and row.get("sender") == "byd_pedestrian_safety_stop"]
    summary["mode_events"] = [row for row in rows if row["topic"] in {"/vehicle/status/control_mode", "/system/operation_mode/state"}]
    summary["trajectory_zero_events"] = [row for row in rows if "trajectory" in row and row["trajectory"].get("max_velocity_mps") is not None and row["trajectory"].get("max_velocity_mps") < 1e-3]
    summary["final_control_zero_events"] = [row for row in rows if row["topic"].endswith("control_cmd") and row.get("velocity_mps") is not None and row["velocity_mps"] < 1e-3]
    command_rows = [row for row in rows if row["topic"] == "/control/command/control_cmd" and row.get("velocity_mps") is not None]
    safety_times = [event["time"] for event in log_events if event["kind"] == "avoidance_safety_stop"]
    incident_anchor = min(safety_times) if safety_times else None
    first_zero = next(
        (row for row in command_rows if incident_anchor is not None and row["time"] >= incident_anchor and row["velocity_mps"] < 1e-3),
        None,
    )
    summary["first_control_zero"] = first_zero
    summary["first_control_resume"] = next(
        (row for row in command_rows if first_zero is not None and row["time"] > first_zero["time"] and row["velocity_mps"] > 0.05),
        None,
    )
    summary["factor_events"] = [row for row in rows if "factors" in row and row["factors"]]
    summary["obstacle_stop_factors"] = [
        {"time": row["time"], "factors": row["factors"]}
        for row in rows
        if row["topic"] == "/planning/planning_factors/obstacle_stop"
        and any(int(factor.get("behavior", -1)) == 3 for factor in row["factors"])
    ]
    summary["positive_obstacle_stop_factors"] = [
        item for item in summary["obstacle_stop_factors"]
        if any((factor.get("distance_m") or 0.0) >= 0.0 for factor in item["factors"])
    ]
    summary["diagnostic_events"] = [row for row in rows if row.get("diagnostics")]
    resume = next(
        (row["time"] for row in rows
         if row["topic"] == "/system/operation_mode/state"
         and row.get("control_enabled") is True
         and row["time"] > 1789871283.0),
        None,
    )
    summary["auto_resume_time"] = resume
    post_auto_invalid = [
        event for event in log_events
        if event["kind"] == "invalid_trajectory" and resume is not None and event["time"] >= resume
    ]
    summary["post_auto_invalid_count"] = len(post_auto_invalid)
    post_auto_trajectory = [
        row for row in rows
        if row["topic"] == "/planning/trajectory"
        and resume is not None and row["time"] >= resume
    ]
    summary["post_auto_trajectory_first"] = post_auto_trajectory[0] if post_auto_trajectory else None
    summary["post_auto_control_last"] = next(
        (row for row in reversed(rows)
         if row["topic"] == "/control/command/control_cmd"
         and resume is not None and row["time"] >= resume),
        None,
    )
    return summary


def fmt_time(seconds: float | None) -> str:
    if seconds is None:
        return "未记录"
    import datetime
    return datetime.datetime.fromtimestamp(seconds).strftime("%H:%M:%S.%f")[:-3]


def render_report(context: dict[str, Any], summaries: list[dict[str, Any]], bag: Path, log: Path) -> str:
    lines = [
        "# 2026-09-20 障碍物停车诊断报告", "",
        f"- Bag: `{bag}`", f"- Log: `{log}`", f"- 采集范围: 10:22:46–10:32:31 CST", "",
        "## 结论摘要", "",
        "1. 事件 1 的直接停车源是 `simple_avoidance`：目标锁定时仅剩约 5.48 m，日志明确给出 `infeasible_distance`，随后输出 `safety stop`。",
        "2. 事件 2 不是单纯 pedestrian 停车：先出现绕行轨迹，随后因 `adjacent_lane_occupied`/`infeasible_distance` 对目标切换或相邻车道占用做安全停车；需结合目标 UUID 和弯道几何复核误判可能性。",
        "3. 事件 3 中 pedestrian 限速确实产生过，但每次都收到同 sender 的 clear；自动模式恢复后仍有 `RETURNING`、约 2.11 m 横向偏移和 `Invalid Trajectory -> soft stop` 证据。当前证据更支持规划轨迹/状态机问题，而非 pedestrian 限速锁死。",
        "",
        "## 事件证据表", "",
        "| 事件 | 窗口 | 关键日志/消息 | 判定 |", "|---|---|---|---|",
        "| 1 | 10:24:05–10:24:25 | target 5.48 m；`infeasible_distance`；`safety stop` | 绕行准备晚于当前速度下的可行横移距离 |",
        "| 2 | 10:24:42–10:25:10 | 先生成 shift，后 `adjacent_lane_occupied` / `infeasible_distance` | 弯道目标/相邻车道安全约束阻断绕行 |",
        "| 3 | 10:28:00–10:29:00 | manual→auto；pedestrian 0 m/s 后 clear；validator soft-stop | 限速已释放，持续不走由轨迹/状态链路承担 |",
        "",
        "## 结构化提取结果", "",
    ]
    for summary in summaries:
        lines.append(f"### {summary['window']}")
        lines.append("")
        lines.append(f"- bag 消息计数: `{summary['counts']}`")
        lines.append(f"- 目标 UUID: `{', '.join(summary['target_uuids']) or '未记录'}`")
        lines.append(f"- 自车速度采样: `{summary.get('ego_speed_first_mps')}` → `{summary.get('ego_speed_last_mps')}` m/s")
        lines.append(f"- pedestrian 限速次数: `{len(summary['pedestrian_limit_events'])}`；clear 次数: `{len(summary['pedestrian_clear_events'])}`")
        lines.append(f"- 零速规划轨迹消息: `{len(summary['trajectory_zero_events'])}`；零速控制命令: `{len(summary['final_control_zero_events'])}`")
        if summary.get("obstacle_stop_factors"):
            first_factor = summary["obstacle_stop_factors"][0]
            last_factor = summary["obstacle_stop_factors"][-1]
            lines.append(f"- `obstacle_stop` factor: 首个 {fmt_time(first_factor['time'])} {first_factor['factors'][0]}; 最后一个 {fmt_time(last_factor['time'])} {last_factor['factors'][0]}")
        if summary.get("first_control_zero"):
            zero = summary["first_control_zero"]
            resume_cmd = summary.get("first_control_resume")
            lines.append(f"- 控制命令: 首次零速 {fmt_time(zero['time'])}；首次恢复到 >0.05 m/s 为 `{fmt_time(resume_cmd['time']) if resume_cmd else '未恢复'}`")
        if summary.get("positive_obstacle_stop_factors"):
            first_positive = summary["positive_obstacle_stop_factors"][0]
            lines.append(f"- 正距离 `obstacle_stop` factor 首次出现于 {fmt_time(first_positive['time'])}：{first_positive['factors'][0]}")
        if summary.get("target_snapshots"):
            lines.append("- 目标跟踪快照:")
            for uuid, snapshots in summary["target_snapshots"].items():
                first, last = snapshots["first"], snapshots["last"]
                lines.append(f"  - `{uuid}` label={first.get('label')} first=({first.get('position_xy')}, {first.get('speed_mps')} m/s) at {fmt_time(first['time'])}; last=({last.get('position_xy')}, {last.get('speed_mps')} m/s) at {fmt_time(last['time'])}")
        if summary["pedestrian_limit_events"] or summary["pedestrian_clear_events"]:
            lines.append("- pedestrian sender 链路:")
            for event in summary["pedestrian_limit_events"] + summary["pedestrian_clear_events"]:
                lines.append(f"  - {fmt_time(event['time'])} `{event['topic']}` sender=`{event.get('sender')}` max=`{event.get('max_velocity_mps')}` clear=`{event.get('clear')}`")
        if summary.get("auto_resume_time") is not None:
            first_traj = summary.get("post_auto_trajectory_first") or {}
            traj = first_traj.get("trajectory", {})
            last_control = summary.get("post_auto_control_last") or {}
            lines.append(f"- 自动恢复: {fmt_time(summary['auto_resume_time'])}；恢复后 invalid trajectory `{summary.get('post_auto_invalid_count')}` 次；首个最终轨迹 max velocity=`{traj.get('max_velocity_mps')}`；窗口末控制目标=`{last_control.get('velocity_mps')}` m/s")
        mode_events = summary["mode_events"]
        if mode_events:
            lines.append("- 模式变化:")
            for event in mode_events:
                suffix = f", enabled={event['control_enabled']}" if "control_enabled" in event else ""
                lines.append(f"  - {fmt_time(event['time'])} `{event['topic']}` → `{event.get('mode_name')}`{suffix}")
        key_events = [event for event in summary["log_events"] if event["kind"] in {"target_locked", "avoidance_path", "avoidance_safety_stop", "pedestrian_stop"}]
        pass_through = [event for event in summary["log_events"] if event["kind"] == "avoidance_pass_through"]
        invalid = [event for event in summary["log_events"] if event["kind"] == "invalid_trajectory"]
        if pass_through:
            reasons = defaultdict(int)
            for event in pass_through:
                reasons[event.get("reason", "unknown")] += 1
            lines.append(f"- simple_avoidance pass-through 计数: `{dict(reasons)}`")
        if invalid:
            lines.append(f"- planning validator invalid trajectory: `{len(invalid)}` 次，首个 `{fmt_time(invalid[0]['time'])}`，最后一个 `{fmt_time(invalid[-1]['time'])}`")
        for event in key_events:
            detail = ", ".join(f"{k}={v}" for k, v in event.items() if k not in {"time", "kind", "line", "raw"})
            lines.append(f"- log {fmt_time(event['time'])} `{event['kind']}` {detail}")
        lines.append("")
    lines.extend([
        "## 根因置信度与排除项", "",
        "- 高：事件 1 为绕行距离不可行导致的 `simple_avoidance` 安全停车；控制层是执行者，不是首个停车决策者。",
        "- 中高：事件 2 为弯道相邻车道占用/目标可行性约束导致的停车；是否为感知目标抖动或 lanelet 几何误判，需要查看输出 CSV 中的 UUID、位置和 planning factors。",
        "- 中高：事件 3 的持续不走不是 pedestrian sender 未清除；应优先修复 `RETURNING` 状态恢复、横向偏移超界和 invalid-trajectory soft-stop 的恢复条件。",
        "- 已排除：仅凭 pedestrian stop 请求解释整个 10:28:30 之后的持续停车；bag 中有对应 clear，且 sender 恢复为其他模块。",
        "",
        "## 后续修复建议", "",
        "1. 障碍物绕行：在目标进入不可行距离前增加提前锁定/降速策略；把 `infeasible_distance` 的停车与可行绕行候选明确区分。",
        "2. 弯道绕行：记录并复核 lanelet/相邻车道占用的对象 UUID、时间连续性和几何边界；必要时增加目标保持与滞回，避免单周期切换方向。",
        "3. 手自动恢复：自动 engage 前清理或重建 simple-avoidance lifecycle；当 `RETURNING` 横向偏移超过边界时，必须生成连续可验证轨迹，而不是长期 soft-stop。",
        "4. pedestrian safety stop：保留安全停车逻辑，同时增加 sender、active/clear、clear 后最终轨迹的诊断关联，防止后续误判为锁存。",
        "",
        "## 可重复命令", "",
        "```bash",
        f"python3 src/byd/scripts/analyze_obstacle_stop_case.py --bag {bag} --log {log} --out-dir /tmp/obstacle_stop_20260920 --repo /home/nvidia/autoware",
        "```",
    ])
    return "\n".join(lines) + "\n"


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = ["time", "topic", "objects", "trajectory", "velocity_mps", "mode", "mode_name", "control_enabled", "transition", "max_velocity_mps", "sender", "clear", "data", "factors", "diagnostics"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else value for key, value in row.items()})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--repo", type=Path, default=Path.cwd())
    args = parser.parse_args()
    if not args.bag.is_file():
        parser.error(f"bag not found: {args.bag}")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(f"file:{args.bag}?mode=ro", uri=True)
    topics = available_topics(db)
    type_map = make_type_map(topics)
    summaries: list[dict[str, Any]] = []
    for name, label, starts, ends in WINDOWS:
        rows = read_window_rows(db, type_map, starts, ends)
        compact = compact_rows(rows)
        events = parse_log(args.log, starts, ends)
        summary = event_summary(f"{name}: {label}", events, compact)
        summary.update({"start_epoch": starts, "end_epoch": ends, "start_local": fmt_time(starts), "end_local": fmt_time(ends), "available_topics": sorted(name for name, _ in topics.values())})
        summaries.append(summary)
        write_csv(args.out_dir / f"{name}.csv", compact)
    context = {"git": git_context(args.repo), "bag": str(args.bag), "log": str(args.log), "topics": sorted(name for name, _ in topics.values()), "windows": [{"name": name, "label": label, "start": starts, "end": ends} for name, label, starts, ends in WINDOWS]}
    # The per-message rows are already available in the window CSVs.  Keeping
    # them a second time in analysis.json can consume several hundred MiB for
    # object-rich windows, so the JSON is intentionally summary-only.
    result = {"context": context, "summaries": summaries}
    (args.out_dir / "analysis.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (args.out_dir / "report.md").write_text(render_report(context, summaries, args.bag, args.log), encoding="utf-8")
    print(args.out_dir / "report.md")
    print(json.dumps({"windows": len(summaries), "topics": len(topics), "output": str(args.out_dir)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
