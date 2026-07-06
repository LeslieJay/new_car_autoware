#!/usr/bin/env python3
# Copyright 2026 BYD
"""Place a single static dummy obstacle and optionally verify avoidance response."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject
from tier4_planning_msgs.msg import AvoidanceDebugMsgArray, PlanningFactorArray

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from dummy_object_utils import (  # noqa: E402
    compute_obstacle_pose,
    make_delete_all,
    make_dummy_object,
)


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class PlaceObstacleNode(Node):
    def __init__(self, baseline: dict, metrics: dict) -> None:
        super().__init__("place_obstacle")
        self.baseline = baseline
        self.metrics = metrics
        self.topics = baseline["topics"]
        self.frame_id = baseline["frame_id"]

        self.dummy_pub = self.create_publisher(DummyObject, self.topics["dummy_object"], 10)
        self.latest_debug: AvoidanceDebugMsgArray | None = None
        self.latest_factors: PlanningFactorArray | None = None

        self.create_subscription(
            AvoidanceDebugMsgArray,
            self.topics["avoidance_debug"],
            self._on_debug,
            10,
        )
        self.create_subscription(
            PlanningFactorArray,
            self.topics["planning_factors"],
            self._on_factors,
            10,
        )

    def _on_debug(self, msg: AvoidanceDebugMsgArray) -> None:
        self.latest_debug = msg

    def _on_factors(self, msg: PlanningFactorArray) -> None:
        self.latest_factors = msg

    def clear_obstacles(self) -> None:
        self.dummy_pub.publish(make_delete_all(self.get_clock().now().to_msg(), self.frame_id))

    def place(
        self,
        longitudinal_m: float | None,
        intrusion_m: float,
        hold_sec: float,
    ) -> tuple[float, float, float]:
        obs = self.baseline["obstacle"]
        ego = self.baseline["ego"]
        lon = (
            float(longitudinal_m)
            if longitudinal_m is not None
            else float(obs["longitudinal_distance_m"])
        )

        x, y, yaw = compute_obstacle_pose(
            float(ego["x"]),
            float(ego["y"]),
            float(obs["lane_yaw"]),
            float(obs["lane_width"]),
            obs["shoulder_side"],
            lon,
            intrusion_m,
            float(obs["width"]),
        )

        z = float(ego.get("z", 0.0))
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

        rate = self.create_rate(self.metrics["timing"].get("publish_rate_hz", 10.0))
        end = time.time() + hold_sec
        while time.time() < end and rclpy.ok():
            msg.header.stamp = self.get_clock().now().to_msg()
            self.dummy_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()

        self.get_logger().info(
            f"Placed obstacle at ({x:.3f}, {y:.3f}), lon={lon:.2f}m, intrusion={intrusion_m:.2f}m"
        )
        return x, y, yaw

    def evaluate(self) -> dict:
        min_shift = float(self.metrics["min_shift_length_m"])
        shift_behaviors = set(self.metrics["shift_behaviors"])

        allow = False
        max_shift = 0.0
        failed_reason = ""
        if self.latest_debug is not None:
            for info in self.latest_debug.avoidance_info:
                allow = allow or info.allow_avoidance
                max_shift = max(max_shift, info.max_shift_length)
                if info.failed_reason:
                    failed_reason = info.failed_reason

        has_shift = False
        has_stop = False
        if self.latest_factors is not None:
            for factor in self.latest_factors.factors:
                if factor.behavior in shift_behaviors:
                    has_shift = True
                if factor.behavior == self.metrics["stop_behavior"]:
                    has_stop = True

        if allow and (has_shift or max_shift >= min_shift):
            result = self.metrics["result_labels"]["success"]
        elif allow or has_stop:
            result = self.metrics["result_labels"]["partial"]
        else:
            result = self.metrics["result_labels"]["fail"]

        return {
            "result": result,
            "allow_avoidance": allow,
            "max_shift_length": max_shift,
            "failed_reason": failed_reason,
            "has_shift_behavior": has_shift,
            "has_stop_behavior": has_stop,
        }


def main() -> None:
    parser = argparse.ArgumentParser(description="Place one dummy obstacle for avoidance test")
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
        "--intrusion",
        type=float,
        default=0.4,
        help="Lateral intrusion from road shoulder into lane [m]",
    )
    parser.add_argument(
        "--longitudinal",
        type=float,
        default=None,
        help="Override longitudinal distance from ego [m]",
    )
    parser.add_argument(
        "--clear-first",
        action="store_true",
        help="Send DELETEALL before placing obstacle",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Wait and print avoidance module response (smoke test)",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=5.0,
        help="Seconds to keep publishing obstacle",
    )
    args = parser.parse_args()

    baseline = load_yaml(args.config)
    metrics = load_yaml(args.metrics)

    rclpy.init()
    node = PlaceObstacleNode(baseline, metrics)

    if args.clear_first:
        node.clear_obstacles()
        time.sleep(0.5)

    node.place(args.longitudinal, args.intrusion, args.hold)

    if args.verify:
        wait = float(metrics["timing"]["settle_sec"])
        node.get_logger().info(f"Waiting {wait:.1f}s for planning response...")
        end = time.time() + wait
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

        report = node.evaluate()
        node.get_logger().info("=== Avoidance smoke test result ===")
        for key, value in report.items():
            node.get_logger().info(f"  {key}: {value}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
