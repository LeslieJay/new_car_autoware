#!/usr/bin/env python3
"""Wait until /planning/trajectory is refreshed continuously.

This is a readiness gate for host-side obstacle tests.  A single trajectory
message proves that the publisher exists, but it does not prove that the
planning callback is keeping up with the simulator.  The gate therefore
requires several messages over a wall-clock interval and rejects a large
inter-message gap.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from autoware_planning_msgs.msg import Trajectory
from rclpy.node import Node


class TrajectoryMonitor(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_stability_monitor")
        self.receipts: list[float] = []
        self.stamps: list[float] = []
        self.create_subscription(Trajectory, "/planning/trajectory", self._on_trajectory, 10)

    def _on_trajectory(self, msg: Trajectory) -> None:
        now = time.monotonic()
        self.receipts.append(now)
        stamp = msg.header.stamp
        self.stamps.append(float(stamp.sec) + float(stamp.nanosec) * 1e-9)


def run(args: argparse.Namespace) -> dict:
    wall_start = time.time()
    mono_start = time.monotonic()
    stable_start: float | None = None
    result = "TIMEOUT"
    reason = "TRAJECTORY_NOT_STABLE"

    rclpy.init()
    node = TrajectoryMonitor()
    try:
        while rclpy.ok() and time.monotonic() - mono_start < args.timeout_sec:
            rclpy.spin_once(node, timeout_sec=args.sample_sec)
            if not node.receipts:
                continue
            if stable_start is None:
                stable_start = node.receipts[0]
            recent = [t for t in node.receipts if t >= stable_start]
            elapsed = time.monotonic() - stable_start
            gaps = [b - a for a, b in zip(recent, recent[1:])]
            max_gap = max(gaps, default=0.0)
            if elapsed >= args.stable_sec and len(recent) >= args.min_samples:
                if max_gap <= args.max_gap_sec:
                    result = "STABLE"
                    reason = "TRAJECTORY_CONTINUOUS"
                    break
                # Keep only the newest continuous window after a stall.
                stable_start = node.receipts[-1]
    finally:
        ended = time.time()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    gaps = [b - a for a, b in zip(node.receipts, node.receipts[1:])]
    output = {
        "result": result,
        "reason": reason,
        "stable_sec": args.stable_sec,
        "min_samples": args.min_samples,
        "max_gap_sec": args.max_gap_sec,
        "sample_count": len(node.receipts),
        "max_observed_gap_sec": max(gaps, default=None),
        "trajectory_stamp_count": len(node.stamps),
        "trajectory_stamp_monotonic": all(
            b > a for a, b in zip(node.stamps, node.stamps[1:])
        ),
        "start_epoch": wall_start,
        "end_epoch": ended,
        "elapsed_sec": ended - wall_start,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stable-sec", type=float, default=2.0)
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--max-gap-sec", type=float, default=0.5)
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    parser.add_argument("--sample-sec", type=float, default=0.1)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if (
        args.stable_sec <= 0
        or args.min_samples <= 1
        or args.max_gap_sec <= 0
        or args.timeout_sec <= 0
    ):
        parser.error("trajectory 稳定门禁参数必须有效")
    output = run(args)
    return 0 if output["result"] == "STABLE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
