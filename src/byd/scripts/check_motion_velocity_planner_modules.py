#!/usr/bin/env python3
"""Read MotionVelocityPlanner.launch_modules without using the ROS CLI daemon."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node


DEFAULT_NODE = (
    "/planning/scenario_planning/lane_driving/motion_planning/"
    "motion_velocity_planner"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node", default=DEFAULT_NODE)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    rclpy.init()
    node = Node("check_motion_velocity_planner_modules")
    client = node.create_client(GetParameters, f"{args.node}/get_parameters")
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.1, min(1.0, deadline - time.monotonic()))
            if client.wait_for_service(timeout_sec=remaining):
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        else:
            print(f"parameter service unavailable: {args.node}/get_parameters", file=sys.stderr)
            return 2

        future = client.call_async(GetParameters.Request(names=["launch_modules"]))
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=max(0.1, deadline - time.monotonic())
        )
        if not future.done() or future.result() is None:
            print("get_parameters request timed out", file=sys.stderr)
            return 3
        values = future.result().values
        if len(values) != 1 or values[0].type != ParameterType.PARAMETER_STRING_ARRAY:
            print("launch_modules is not a string-array parameter", file=sys.stderr)
            return 4
        print(f"String values are: {list(values[0].string_array_value)!r}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
