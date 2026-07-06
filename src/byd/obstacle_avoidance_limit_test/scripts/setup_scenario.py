#!/usr/bin/env python3
# Copyright 2026 BYD
"""Publish fixed initial pose and mission goal for obstacle avoidance tests."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from dummy_object_utils import yaw_to_quaternion  # noqa: E402


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class ScenarioSetupNode(Node):
    def __init__(self, baseline: dict) -> None:
        super().__init__("obstacle_avoidance_scenario_setup")
        self.baseline = baseline
        self.frame_id = baseline["frame_id"]
        topics = baseline["topics"]

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_pub = self.create_publisher(
            PoseWithCovarianceStamped, topics["initial_pose"], qos
        )
        self.goal_pub = self.create_publisher(PoseStamped, topics["goal"], qos)

    def publish_initial_pose(self) -> None:
        ego = self.baseline["ego"]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(ego["x"])
        msg.pose.pose.position.y = float(ego["y"])
        msg.pose.pose.position.z = float(ego.get("z", 0.0))
        msg.pose.pose.orientation = yaw_to_quaternion(float(ego["yaw"]))
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685
        self.initial_pub.publish(msg)
        self.get_logger().info(
            f"Published initial pose: ({ego['x']}, {ego['y']}, yaw={ego['yaw']})"
        )

    def publish_goal(self) -> None:
        goal = self.baseline["goal"]
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(goal["x"])
        msg.pose.position.y = float(goal["y"])
        msg.pose.position.z = float(goal.get("z", 0.0))
        msg.pose.orientation = yaw_to_quaternion(float(goal["yaw"]))
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"Published goal: ({goal['x']}, {goal['y']}, yaw={goal['yaw']})"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description="Setup ego pose and goal for avoidance test")
    parser.add_argument(
        "--config",
        type=Path,
        default=SCRIPT_DIR.parent / "config" / "baseline.yaml",
        help="Baseline scenario YAML",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=3,
        help="Publish count for transient_local topics",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Seconds between repeated publishes",
    )
    args = parser.parse_args()

    baseline = load_yaml(args.config)
    rclpy.init()
    node = ScenarioSetupNode(baseline)

    for i in range(args.repeat):
        node.publish_initial_pose()
        node.publish_goal()
        if i + 1 < args.repeat:
            time.sleep(args.interval)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
