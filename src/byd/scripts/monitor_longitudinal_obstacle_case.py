#!/usr/bin/env python3
"""Stop one obstacle-avoidance case as soon as the vehicle has passed the obstacle."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from autoware_adapi_v1_msgs.msg import RouteState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tier4_control_msgs.srv import SetPause


START_X = 279.52191162109375
START_Y = -33.24296569824219
START_OX = -0.00024534598908445367
START_OY = -0.00041156653669880767
START_OZ = -0.5120475691803335
START_OW = 0.8589569589419735
DEFAULT_OBSTACLE_LENGTH_M = 2.0


def quat_to_yaw(ox: float, oy: float, oz: float, ow: float) -> float:
    return math.atan2(
        2.0 * (ow * oz + ox * oy),
        1.0 - 2.0 * (oy * oy + oz * oz),
    )


ROUTE_YAW = quat_to_yaw(START_OX, START_OY, START_OZ, START_OW)


def longitudinal(x: float, y: float) -> float:
    return (x - START_X) * math.cos(ROUTE_YAW) + (y - START_Y) * math.sin(ROUTE_YAW)


class CaseMonitor(Node):
    def __init__(self, obstacle_ready_file: Path | None = None) -> None:
        super().__init__("longitudinal_obstacle_case_monitor")
        self.obstacle_ready_file = obstacle_ready_file
        self.latest: tuple[float, float] | None = None
        self.latest_wall = 0.0
        self.route_state = RouteState.UNKNOWN
        self.route_set_seen = False
        self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._on_kinematic_state,
            10,
        )
        route_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            RouteState,
            "/api/routing/state",
            self._on_route_state,
            route_qos,
        )

    def _on_kinematic_state(self, msg: Odometry) -> None:
        pose = msg.pose.pose.position
        self.latest = (longitudinal(pose.x, pose.y), msg.twist.twist.linear.x)
        self.latest_wall = time.time()

    def _on_route_state(self, msg: RouteState) -> None:
        self.route_state = msg.state
        if msg.state in (RouteState.SET, RouteState.CHANGING):
            self.route_set_seen = True

    def obstacle_is_ready(self) -> bool:
        return self.obstacle_ready_file is None or self.obstacle_ready_file.exists()

    def pause_vehicle(self) -> bool:
        client = self.create_client(SetPause, "/control/vehicle_cmd_gate/set_pause")
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("set_pause service unavailable")
            return False
        future = client.call_async(SetPause.Request(pause=True))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("set_pause request timed out")
            return False
        return True


def monitor(args: argparse.Namespace) -> dict:
    threshold = None
    started_epoch = time.time()
    started_mono = time.monotonic()
    max_lon: float | None = None
    last_lon: float | None = None
    last_speed = 0.0
    last_progress_mono = started_mono
    last_pose_mono: float | None = None
    passed_streak = 0
    obstacle_armed = False
    placement_data: dict | None = None
    obstacle_crossed = False
    obstacle_crossed_epoch: float | None = None
    goal_reached = False
    goal_reached_epoch: float | None = None
    result = "INCOMPLETE"
    reason = ""
    timeout = False

    rclpy.init()
    node = CaseMonitor(args.obstacle_ready_file)
    try:
        node.get_logger().info(
            f"监视启动: obstacle={args.distance:.2f}m "
            "threshold=待障碍物元数据就绪后计算"
            if threshold is None
            else f"监视启动: obstacle={args.distance:.2f}m threshold={threshold:.2f}m"
        )
        while rclpy.ok():
            now = time.monotonic()
            if now - started_mono >= args.max_observe_sec:
                result = "TIMEOUT"
                reason = "OBSERVATION_TIMEOUT"
                timeout = True
                break

            rclpy.spin_once(node, timeout_sec=args.sample_sec)
            now = time.monotonic()
            if node.latest is None:
                if now - started_mono >= args.no_pose_sec:
                    result = "BLOCKED"
                    reason = "NO_LOCALIZATION"
                    break
                continue

            current_lon, current_speed = node.latest
            last_lon = current_lon
            last_speed = current_speed
            if not node.obstacle_is_ready():
                continue
            if args.obstacle_metadata_file is not None:
                try:
                    placement_data = json.loads(
                        args.obstacle_metadata_file.read_text(encoding="utf-8")
                    )
                except (OSError, json.JSONDecodeError):
                    continue
                ego_s = placement_data.get("ego_route_longitudinal_m")
                actual_ahead = placement_data.get("actual_ahead_m")
                if not isinstance(ego_s, (int, float)) or not isinstance(actual_ahead, (int, float)):
                    result = "INCOMPLETE"
                    reason = "INVALID_PLACEMENT_METADATA"
                    break
                threshold = float(ego_s) + float(actual_ahead) + args.obstacle_length / 2.0 + args.pass_margin
                speed = float(placement_data.get("speed_mps", float("nan")))
                actual_ahead = float(placement_data.get("actual_ahead_m", float("nan")))
                requested_ahead = float(placement_data.get("requested_ahead_m", args.distance))
                if (
                    not bool(placement_data.get("placement_valid", False))
                    or abs(speed - args.target_speed) > args.speed_tolerance
                    or abs(actual_ahead - requested_ahead) > args.distance_tolerance
                ):
                    result = "INCOMPLETE"
                    reason = "PLACEMENT_SPEED_OR_DISTANCE_INVALID"
                    break
            if not obstacle_armed:
                obstacle_armed = True
                last_progress_mono = now
                if threshold is None:
                    threshold = args.distance + args.obstacle_length / 2.0 + args.pass_margin
                if current_lon >= threshold:
                    result = "INCOMPLETE"
                    reason = "OBSTACLE_PLACED_TOO_LATE"
                    break
            if max_lon is None or current_lon > max_lon + args.progress_epsilon:
                max_lon = current_lon
                last_progress_mono = now
            last_pose_mono = now

            if threshold is not None and current_lon >= threshold:
                passed_streak += 1
            else:
                passed_streak = 0

            recently_moving = abs(current_speed) > args.min_speed or (
                now - last_progress_mono <= args.recent_progress_sec
            )
            if passed_streak >= args.confirm_samples and recently_moving:
                if not obstacle_crossed:
                    obstacle_crossed = True
                    obstacle_crossed_epoch = time.time()
                    node.get_logger().info(
                        "obstacle_crossed: continuing until /api/routing/state=ARRIVED"
                    )

            if obstacle_crossed and node.route_set_seen and node.route_state == RouteState.ARRIVED:
                result = "ARRIVED"
                reason = "ROUTE_STATE_ARRIVED"
                goal_reached = True
                goal_reached_epoch = time.time()
                break

            if not obstacle_crossed and now - last_progress_mono >= args.blocked_sec:
                result = "BLOCKED_BEFORE_OBSTACLE"
                reason = "NO_PROGRESS_BEFORE_OBSTACLE"
                break
            if obstacle_crossed and now - last_progress_mono >= args.blocked_sec:
                result = "BLOCKED_AFTER_OBSTACLE"
                reason = "NO_PROGRESS_AFTER_OBSTACLE"
                break

        if result in {
            "ARRIVED",
            "PASSED",
            "BLOCKED",
            "BLOCKED_BEFORE_OBSTACLE",
            "BLOCKED_AFTER_OBSTACLE",
            "TIMEOUT",
        }:
            node.pause_vehicle()
    finally:
        ended_epoch = time.time()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    output = {
        "distance_m": args.distance,
        "monitor_result": result,
        "vehicle_passed": obstacle_crossed,
        "obstacle_crossed": obstacle_crossed,
        "obstacle_armed": obstacle_armed,
        "obstacle_crossed_epoch": obstacle_crossed_epoch,
        "goal_reached": goal_reached,
        "goal_reached_epoch": goal_reached_epoch,
        "route_state": int(node.route_state),
        "route_set_seen": node.route_set_seen,
        "max_longitudinal_m": max_lon,
        "last_longitudinal_m": last_lon,
        "threshold_longitudinal_m": threshold,
        "placement": placement_data,
        "last_speed_mps": last_speed,
        "early_stop_reason": reason,
        "timeout": timeout,
        "start_epoch": started_epoch,
        "end_epoch": ended_epoch,
        "elapsed_sec": ended_epoch - started_epoch,
        "last_pose_age_sec": None
        if last_pose_mono is None
        else max(0.0, time.monotonic() - last_pose_mono),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description="越障后继续运行至路线到达或明确失败")
    parser.add_argument("--distance", type=float, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-observe-sec", type=float, default=180.0)
    parser.add_argument("--blocked-sec", type=float, default=15.0)
    parser.add_argument("--obstacle-length", type=float, default=DEFAULT_OBSTACLE_LENGTH_M)
    parser.add_argument("--pass-margin", type=float, default=1.0)
    parser.add_argument("--min-speed", type=float, default=0.2)
    parser.add_argument("--recent-progress-sec", type=float, default=1.5)
    parser.add_argument("--progress-epsilon", type=float, default=0.05)
    parser.add_argument("--sample-sec", type=float, default=0.2)
    parser.add_argument("--confirm-samples", type=int, default=5)
    parser.add_argument("--no-pose-sec", type=float, default=15.0)
    parser.add_argument(
        "--obstacle-ready-file",
        type=Path,
        default=None,
        help="仅在该文件出现后开始越障/阻塞判定",
    )
    parser.add_argument(
        "--obstacle-metadata-file",
        type=Path,
        default=None,
        help="动态投放元数据；存在后以真实障碍物路线坐标计算越障阈值",
    )
    parser.add_argument("--target-speed", type=float, default=2.0)
    parser.add_argument("--speed-tolerance", type=float, default=0.1)
    parser.add_argument("--distance-tolerance", type=float, default=0.25)
    args = parser.parse_args()
    if args.max_observe_sec <= 0 or args.blocked_sec <= 0 or args.confirm_samples <= 0:
        parser.error("观察时间、阻塞时间和确认样本数必须为正数")
    output = monitor(args)
    return {
        "ARRIVED": 0,
        "PASSED": 0,
        "BLOCKED": 2,
        "BLOCKED_BEFORE_OBSTACLE": 2,
        "BLOCKED_AFTER_OBSTACLE": 2,
        "TIMEOUT": 3,
    }.get(
        output["monitor_result"], 4
    )


if __name__ == "__main__":
    raise SystemExit(main())
