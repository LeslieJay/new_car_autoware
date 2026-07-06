#!/usr/bin/env python3
"""Analyze forward endpoint accuracy from rosbag2 recordings.

Usage:
  ./analyze_forward_endpoint_accuracy.py /path/to/bag_dir
  ./analyze_forward_endpoint_accuracy.py /path/to/bag_dir --goal-x 261.444 --goal-y -28.175
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# OperationModeState constants
OP_STOP = 1
OP_AUTONOMOUS = 2
OP_LOCAL = 3

# GateMode
GATE_AUTO = 0
GATE_EXTERNAL = 1


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def load_bag_series(bag_path: str, topics: list[str]) -> tuple[dict[str, list], int]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    cls = {tp: get_message(topic_types[tp]) for tp in topics if tp in topic_types}
    series: dict[str, list] = {tp: [] for tp in cls}
    end_ts = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        end_ts = max(end_ts, t)
        if topic not in cls:
            continue
        series[topic].append((t, deserialize_message(data, cls[topic])))
    return series, end_ts


def find_standstill_start(speed_series: list[tuple[int, float]], threshold: float = 0.05) -> int | None:
    if not speed_series:
        return None
    idx = len(speed_series) - 1
    while idx >= 0 and speed_series[idx][1] <= threshold:
        idx -= 1
    if idx + 1 < len(speed_series):
        return speed_series[idx + 1][0]
    return speed_series[0][0]


def nearest_value(series: list[tuple[int, float]], ts: int) -> float | None:
    if not series:
        return None
    i = min(range(len(series)), key=lambda k: abs(series[k][0] - ts))
    return series[i][1]


def analyze_bag(bag_path: str, goal_x: float | None, goal_y: float | None) -> dict:
    topics = [
        "/planning/mission_planning/goal",
        "/localization/kinematic_state",
        "/planning/trajectory",
        "/vehicle/status/velocity_status",
        "/planning/mission_remaining_distance_time",
        "/control/current_gate_mode",
        "/control/vehicle_cmd_gate/operation_mode",
        "/control/vehicle_cmd_gate/is_paused",
        "/control/vehicle_cmd_gate/is_start_requested",
    ]
    series, end_ts = load_bag_series(bag_path, topics)

    result: dict = {"bag": bag_path, "end_ts": end_ts}

    # Goal
    goal = None
    if series["/planning/mission_planning/goal"]:
        _, goal_msg = series["/planning/mission_planning/goal"][-1]
        goal = goal_msg.pose
    if goal_x is not None and goal_y is not None:
        gx, gy = goal_x, goal_y
        goal_source = "cli_override"
    elif goal is not None:
        gx, gy = goal.position.x, goal.position.y
        goal_source = "bag_goal_topic"
    else:
        return {"bag": bag_path, "error": "no goal available"}

    result["goal"] = {"x": gx, "y": gy, "source": goal_source}

    if not series["/localization/kinematic_state"]:
        result["error"] = "missing /localization/kinematic_state"
        return result

    _, odom_msg = series["/localization/kinematic_state"][-1]
    odom = odom_msg.pose.pose
    oy = yaw_from_quat(odom.orientation)

    # Trajectory endpoint
    traj_end = None
    if series["/planning/trajectory"]:
        _, traj = series["/planning/trajectory"][-1]
        if traj.points:
            p = traj.points[-1].pose.position
            traj_end = (p.x, p.y)

    planning_err = None
    control_err = None
    total_err = math.hypot(odom.position.x - gx, odom.position.y - gy)

    if traj_end:
        planning_err = math.hypot(traj_end[0] - gx, traj_end[1] - gy)
        control_err = math.hypot(odom.position.x - traj_end[0], odom.position.y - traj_end[1])

    result["final_odom"] = {"x": odom.position.x, "y": odom.position.y, "yaw_deg": math.degrees(oy)}
    result["errors_m"] = {
        "total_goal_to_odom": round(total_err, 4),
        "planning_goal_to_traj_end": round(planning_err, 4) if planning_err is not None else None,
        "control_traj_end_to_odom": round(control_err, 4) if control_err is not None else None,
    }
    if traj_end:
        result["traj_end"] = {"x": traj_end[0], "y": traj_end[1]}

    # Speed / standstill
    speed_series = []
    for t, m in series["/vehicle/status/velocity_status"]:
        sp = math.hypot(m.longitudinal_velocity, m.lateral_velocity)
        speed_series.append((t, sp))
    t_stop = find_standstill_start(speed_series)
    if t_stop is not None:
        standstill_dur = (end_ts - t_stop) / 1e9
        rem_at_stop = nearest_value(
            [(t, m.remaining_distance) for t, m in series["/planning/mission_remaining_distance_time"]],
            t_stop,
        )
        rem_at_end = nearest_value(
            [(t, m.remaining_distance) for t, m in series["/planning/mission_remaining_distance_time"]],
            end_ts,
        )
        result["standstill"] = {
            "duration_s": round(standstill_dur, 1),
            "remaining_at_stop_m": round(rem_at_stop, 3) if rem_at_stop is not None else None,
            "remaining_at_end_m": round(rem_at_end, 3) if rem_at_end is not None else None,
        }

    # Mode at end and at standstill start
    def last_mode_snapshot(ts: int) -> dict:
        snap = {}
        for tp, key in [
            ("/control/current_gate_mode", "gate_mode"),
            ("/control/vehicle_cmd_gate/operation_mode", "operation_mode"),
            ("/control/vehicle_cmd_gate/is_paused", "is_paused"),
            ("/control/vehicle_cmd_gate/is_start_requested", "is_start_requested"),
        ]:
            if not series[tp]:
                continue
            i = min(range(len(series[tp])), key=lambda k: abs(series[tp][k][0] - ts))
            t, m = series[tp][i]
            if tp == "/control/current_gate_mode":
                snap["gate_mode"] = m.data
                snap["gate_mode_name"] = "AUTO" if m.data == GATE_AUTO else "EXTERNAL"
            elif tp == "/control/vehicle_cmd_gate/operation_mode":
                snap["operation_mode"] = m.mode
                snap["is_autoware_control_enabled"] = m.is_autoware_control_enabled
                mode_names = {OP_STOP: "STOP", OP_AUTONOMOUS: "AUTONOMOUS", OP_LOCAL: "LOCAL"}
                snap["operation_mode_name"] = mode_names.get(m.mode, f"UNKNOWN({m.mode})")
            else:
                snap[key] = m.data
            snap[f"{key}_age_s"] = round((end_ts - t) / 1e9, 2)
        return snap

    result["mode_at_end"] = last_mode_snapshot(end_ts)
    if t_stop is not None:
        result["mode_at_standstill_start"] = last_mode_snapshot(t_stop)

    # Operation mode transitions in last 300s
    transitions = []
    prev = None
    for t, m in series["/control/vehicle_cmd_gate/operation_mode"]:
        if t < end_ts - 300_000_000_000:
            continue
        key = (m.mode, m.is_autoware_control_enabled, m.is_in_transition)
        if key != prev:
            transitions.append(
                {
                    "before_end_s": round((end_ts - t) / 1e9, 1),
                    "mode": key[0],
                    "auto_enabled": key[1],
                    "in_transition": key[2],
                }
            )
            prev = key
    result["operation_mode_transitions_last_300s"] = transitions

    # Root cause classification
    planning_ok = planning_err is not None and planning_err < 0.5
    early_stop = (
        result.get("standstill", {}).get("remaining_at_end_m") is not None
        and result["standstill"]["remaining_at_end_m"] > 2.0
        and result.get("standstill", {}).get("duration_s", 0) > 30
    )
    mode_end = result["mode_at_end"]
    mode_issue = (
        mode_end.get("gate_mode") == GATE_EXTERNAL
        or mode_end.get("operation_mode") == OP_LOCAL
        or mode_end.get("is_paused") is True
        or mode_end.get("is_start_requested") is False
    )

    if planning_ok and early_stop and mode_issue:
        root = "mode_or_pause_blocked_execution"
    elif planning_ok and early_stop:
        root = "early_stop_despite_valid_plan"
    elif not planning_ok:
        root = "planning_goal_mismatch"
    elif control_err is not None and control_err > 1.0:
        root = "control_tracking_error"
    else:
        root = "within_tolerance"

    result["classification"] = {
        "planning_ok": planning_ok,
        "early_stop": early_stop,
        "mode_issue_at_end": mode_issue,
        "root_cause": root,
    }

    return result


def print_report(results: list[dict]) -> None:
    print("=" * 72)
    print("Forward Endpoint Accuracy Report")
    print("=" * 72)
    for r in results:
        print(f"\nBag: {r['bag']}")
        if "error" in r:
            print(f"  ERROR: {r['error']}")
            continue
        e = r["errors_m"]
        print(f"  Goal ({r['goal']['source']}): ({r['goal']['x']:.3f}, {r['goal']['y']:.3f})")
        print(f"  Final odom: ({r['final_odom']['x']:.3f}, {r['final_odom']['y']:.3f})")
        if "traj_end" in r:
            print(f"  Traj end:   ({r['traj_end']['x']:.3f}, {r['traj_end']['y']:.3f})")
        print(f"  Total error (goal->odom):     {e['total_goal_to_odom']:.3f} m")
        print(f"  Planning error (goal->traj):  {e['planning_goal_to_traj_end']} m")
        print(f"  Control error (traj->odom):   {e['control_traj_end_to_odom']} m")
        if "standstill" in r:
            s = r["standstill"]
            print(
                f"  Standstill: {s['duration_s']}s, remaining@stop={s['remaining_at_stop_m']}m, "
                f"remaining@end={s['remaining_at_end_m']}m"
            )
        me = r["mode_at_end"]
        print(
            f"  Mode@end: gate={me.get('gate_mode_name')}, op={me.get('operation_mode_name')}, "
            f"paused={me.get('is_paused')}, start_req={me.get('is_start_requested')}"
        )
        c = r["classification"]
        print(f"  Root cause: {c['root_cause']}")
        if r.get("operation_mode_transitions_last_300s"):
            print("  Op mode transitions (last 300s):")
            for tr in r["operation_mode_transitions_last_300s"]:
                print(f"    -{tr['before_end_s']}s mode={tr['mode']} auto={tr['auto_enabled']}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze forward endpoint accuracy from rosbag2")
    parser.add_argument("bags", nargs="+", help="Bag directory path(s)")
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)
    parser.add_argument("--json-out", type=str, default=None)
    args = parser.parse_args()

    results = [analyze_bag(b, args.goal_x, args.goal_y) for b in args.bags]
    print_report(results)

    if args.json_out:
        Path(args.json_out).write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(f"\nJSON written to {args.json_out}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
