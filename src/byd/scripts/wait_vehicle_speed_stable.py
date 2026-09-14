#!/usr/bin/env python3
"""Wait until ego speed remains near a target for a fixed duration."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SpeedMonitor(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_speed_stability_monitor")
        self.latest_speed: float | None = None
        self.create_subscription(Odometry, "/localization/kinematic_state", self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_speed = abs(float(msg.twist.twist.linear.x))


def run(args: argparse.Namespace) -> dict:
    started = time.time()
    started_mono = time.monotonic()
    stable_since: float | None = None
    stable_samples = 0
    speeds: list[float] = []
    result = "TIMEOUT"
    reason = "SPEED_NOT_STABLE"

    rclpy.init()
    node = SpeedMonitor()
    try:
        while rclpy.ok():
            if time.monotonic() - started_mono >= args.timeout_sec:
                break
            rclpy.spin_once(node, timeout_sec=args.sample_sec)
            if node.latest_speed is None:
                continue
            speed = node.latest_speed
            speeds.append(speed)
            now = time.monotonic()
            if abs(speed - args.target_speed) <= args.tolerance:
                if stable_since is None:
                    stable_since = now
                    stable_samples = 0
                stable_samples += 1
                if now - stable_since >= args.stable_sec:
                    result = "STABLE"
                    reason = "TARGET_SPEED_STABLE"
                    break
            else:
                stable_since = None
                stable_samples = 0
    finally:
        ended = time.time()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    output = {
        "result": result,
        "reason": reason,
        "target_speed_mps": args.target_speed,
        "tolerance_mps": args.tolerance,
        "stable_sec": args.stable_sec,
        "stable_samples": stable_samples,
        "sample_count": len(speeds),
        "min_speed_mps": min(speeds) if speeds else None,
        "max_speed_mps": max(speeds) if speeds else None,
        "last_speed_mps": speeds[-1] if speeds else None,
        "start_epoch": started,
        "end_epoch": ended,
        "elapsed_sec": ended - started,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-speed", type=float, default=2.0)
    parser.add_argument("--tolerance", type=float, default=0.1)
    parser.add_argument("--stable-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    parser.add_argument("--sample-sec", type=float, default=0.1)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.target_speed < 0 or args.tolerance < 0 or args.stable_sec <= 0 or args.timeout_sec <= 0:
        parser.error("速度、容差、稳定时间和超时必须有效")
    return 0 if run(args)["result"] == "STABLE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
