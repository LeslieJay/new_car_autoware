#!/usr/bin/env python3
# Copyright 2026 BYD
"""Sweep lateral/longitudinal obstacle positions and record avoidance limits."""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Iterable

import rclpy
import yaml
from autoware_perception_msgs.msg import PredictedObjects
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray, PlanningFactorArray

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from dummy_object_utils import compute_obstacle_pose, make_delete_all, make_dummy_object  # noqa: E402


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def frange(start: float, stop: float, step: float) -> list[float]:
    values: list[float] = []
    v = start
    while v <= stop + 1e-9:
        values.append(round(v, 4))
        v += step
    return values


@dataclass
class SampleAggregate:
    allow_count: int = 0
    sample_count: int = 0
    max_shift_length: float = 0.0
    failed_reasons: set[str] = field(default_factory=set)
    has_shift: bool = False
    has_stop: bool = False
    object_seen: bool = False

    def ingest_debug(self, msg: AvoidanceDebugMsgArray) -> None:
        self.sample_count += 1
        for info in msg.avoidance_info:
            if info.allow_avoidance:
                self.allow_count += 1
            self.max_shift_length = max(self.max_shift_length, info.max_shift_length)
            if info.failed_reason:
                self.failed_reasons.add(info.failed_reason)

    def ingest_factors(self, msg: PlanningFactorArray, shift_behaviors: set[int], stop_behavior: int) -> None:
        for factor in msg.factors:
            if factor.behavior in shift_behaviors:
                self.has_shift = True
            if factor.behavior == stop_behavior:
                self.has_stop = True


class AvoidanceSweepNode(Node):
    def __init__(self, baseline: dict, metrics: dict) -> None:
        super().__init__("obstacle_avoidance_sweep")
        self.baseline = baseline
        self.metrics = metrics
        self.topics = baseline["topics"]
        self.frame_id = baseline["frame_id"]
        self.labels = metrics["result_labels"]
        self.shift_behaviors = set(metrics["shift_behaviors"])
        self.stop_behavior = metrics["stop_behavior"]
        self.min_shift = float(metrics["min_shift_length_m"])

        self.dummy_pub = self.create_publisher(DummyObject, self.topics["dummy_object"], 10)
        self.latest_debug: AvoidanceDebugMsgArray | None = None
        self.latest_factors: PlanningFactorArray | None = None
        self.latest_objects: PredictedObjects | None = None

        self.create_subscription(
            AvoidanceDebugMsgArray, self.topics["avoidance_debug"], self._on_debug, 10
        )
        self.create_subscription(
            PlanningFactorArray, self.topics["planning_factors"], self._on_factors, 10
        )
        self.create_subscription(
            PredictedObjects, self.topics["predicted_objects"], self._on_objects, 10
        )

    def _on_debug(self, msg: AvoidanceDebugMsgArray) -> None:
        self.latest_debug = msg

    def _on_factors(self, msg: PlanningFactorArray) -> None:
        self.latest_factors = msg

    def _on_objects(self, msg: PredictedObjects) -> None:
        self.latest_objects = msg

    def clear_obstacles(self) -> None:
        self.dummy_pub.publish(make_delete_all(self.get_clock().now().to_msg(), self.frame_id))

    def publish_obstacle(self, longitudinal_m: float, intrusion_m: float) -> tuple[float, float]:
        obs = self.baseline["obstacle"]
        ego = self.baseline["ego"]
        x, y, yaw = compute_obstacle_pose(
            float(ego["x"]),
            float(ego["y"]),
            float(obs["lane_yaw"]),
            float(obs["lane_width"]),
            obs["shoulder_side"],
            longitudinal_m,
            intrusion_m,
            float(obs["width"]),
        )
        z = float(ego.get("z", 0.0))
        return x, y, self._publish_pose(x, y, z, yaw)

    def _publish_pose(self, x: float, y: float, z: float, yaw: float):
        obs = self.baseline["obstacle"]
        msg = make_dummy_object(
            self.frame_id,
            self.get_clock().now().to_msg(),
            x,
            y,
            z,
            yaw,
            length=float(obs["length"]),
            width=float(obs["width"]),
            height=float(obs["height"]),
            velocity=float(obs["velocity"]),
        )
        self.dummy_pub.publish(msg)
        return msg

    def run_case(self, longitudinal_m: float, intrusion_m: float) -> dict:
        timing = self.metrics["timing"]
        settle = float(timing["settle_sec"])
        sample = float(timing["sample_sec"])
        rate_hz = float(timing.get("publish_rate_hz", 10.0))

        self.clear_obstacles()
        time.sleep(0.3)

        ox, oy, _ = self.publish_obstacle(longitudinal_m, intrusion_m)
        agg = SampleAggregate()

        end_settle = time.time() + settle
        period = 1.0 / rate_hz
        while time.time() < end_settle and rclpy.ok():
            self._publish_pose(ox, oy, float(self.baseline["ego"].get("z", 0.0)), float(self.baseline["obstacle"]["lane_yaw"]))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        end_sample = time.time() + sample
        while time.time() < end_sample and rclpy.ok():
            self._publish_pose(ox, oy, float(self.baseline["ego"].get("z", 0.0)), float(self.baseline["obstacle"]["lane_yaw"]))
            if self.latest_objects is not None and len(self.latest_objects.objects) > 0:
                agg.object_seen = True
            if self.latest_debug is not None:
                agg.ingest_debug(self.latest_debug)
            if self.latest_factors is not None:
                agg.ingest_factors(self.latest_factors, self.shift_behaviors, self.stop_behavior)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        return self._classify(longitudinal_m, intrusion_m, ox, oy, agg)

    def _classify(
        self,
        longitudinal_m: float,
        intrusion_m: float,
        ox: float,
        oy: float,
        agg: SampleAggregate,
    ) -> dict:
        allow_rate = agg.allow_count / agg.sample_count if agg.sample_count else 0.0
        failed = ";".join(sorted(agg.failed_reasons))

        if not agg.object_seen:
            result = self.labels["no_object"]
        elif agg.allow_count > 0 and (agg.has_shift or agg.max_shift_length >= self.min_shift):
            result = self.labels["success"]
        elif agg.allow_count > 0 or agg.has_stop:
            result = self.labels["partial"]
        else:
            result = self.labels["fail"]

        return {
            "longitudinal_m": longitudinal_m,
            "intrusion_m": intrusion_m,
            "obstacle_x": round(ox, 4),
            "obstacle_y": round(oy, 4),
            "result": result,
            "allow_avoidance_rate": round(allow_rate, 3),
            "max_shift_length": round(agg.max_shift_length, 4),
            "has_shift_behavior": agg.has_shift,
            "has_stop_behavior": agg.has_stop,
            "object_seen": agg.object_seen,
            "failed_reason": failed,
        }


def summarize_lateral_limit(rows: Iterable[dict]) -> dict:
    rows = list(rows)
    success = [r for r in rows if r["result"] == "SUCCESS"]
    fail = [r for r in rows if r["result"] in ("FAIL", "NO_OBJECT")]

    max_success_intrusion = max((r["intrusion_m"] for r in success), default=None)
    min_fail_intrusion = min((r["intrusion_m"] for r in fail), default=None)

    return {
        "max_success_intrusion_m": max_success_intrusion,
        "min_fail_intrusion_m": min_fail_intrusion,
        "estimated_lateral_limit_m": max_success_intrusion,
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep obstacle intrusion and measure avoidance limit")
    parser.add_argument(
        "--config",
        type=Path,
        default=SCRIPT_DIR.parent / "config" / "baseline.yaml",
    )
    parser.add_argument(
        "--metrics",
        type=Path,
        default=SCRIPT_DIR.parent / "config" / "metrics.yaml",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=SCRIPT_DIR.parent / "results",
    )
    parser.add_argument(
        "--skip-setup",
        action="store_true",
        help="Skip scenario setup (initialpose/goal); assume already set",
    )
    args = parser.parse_args()

    baseline = load_yaml(args.config)
    metrics = load_yaml(args.metrics)

    if not args.skip_setup:
        import subprocess

        setup_script = SCRIPT_DIR / "setup_scenario.py"
        subprocess.run([sys.executable, str(setup_script), "--config", str(args.config)], check=True)
        time.sleep(float(metrics["timing"]["setup_wait_sec"]))

    rclpy.init()
    node = AvoidanceSweepNode(baseline, metrics)

    sweep = metrics["lateral_sweep"]
    intrusions = frange(float(sweep["start_m"]), float(sweep["stop_m"]), float(sweep["step_m"]))
    default_lon = float(baseline["obstacle"]["longitudinal_distance_m"])

    rows: list[dict] = []
    node.get_logger().info(f"Starting lateral sweep: {intrusions}")

    for intrusion in intrusions:
        node.get_logger().info(f"Testing intrusion={intrusion:.2f} m")
        row = node.run_case(default_lon, intrusion)
        rows.append(row)
        node.get_logger().info(
            f"  -> {row['result']} (allow_rate={row['allow_avoidance_rate']}, "
            f"max_shift={row['max_shift_length']}, reason={row['failed_reason']})"
        )

    lon_cfg = metrics.get("longitudinal_sweep", {})
    if lon_cfg.get("enabled", False):
        fixed = float(lon_cfg["fixed_intrusion_m"])
        for lon in lon_cfg["distances_m"]:
            node.get_logger().info(f"Testing longitudinal={lon:.1f} m, intrusion={fixed:.2f} m")
            row = node.run_case(float(lon), fixed)
            rows.append(row)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.output_dir / timestamp
    csv_path = out_dir / "sweep_results.csv"
    write_csv(csv_path, rows)

    summary = summarize_lateral_limit(rows)
    summary_path = out_dir / "summary.yaml"
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    with summary_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(summary, f, allow_unicode=True, sort_keys=False)

    node.get_logger().info(f"Results written to {csv_path}")
    node.get_logger().info(f"Summary: {summary}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
