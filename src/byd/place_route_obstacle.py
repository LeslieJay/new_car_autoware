#!/usr/bin/env python3
# Copyright 2026 BYD
"""在 sim_init_and_set_goal.py 起终点路径上发布静态障碍物，触发绕障。

默认坐标与 src/byd/sim_init_and_set_goal.py 中 INIT_* / GOAL_* 一致。
默认分类为 UNKNOWN（ObjectClassification.label=0）。

用法（建议先暂停车辆，避免驶过障碍物位置）:
  ros2 service call /control/vehicle_cmd_gate/set_pause tier4_control_msgs/srv/SetPause "{pause: true}"
  python3 src/byd/sim_init_and_set_goal.py
  python3 src/byd/place_route_obstacle.py --clear-first --label unknown
  ros2 service call /control/vehicle_cmd_gate/set_pause tier4_control_msgs/srv/SetPause "{pause: false}"

  # 调整沿路径距离与横向侵入:
  python3 src/byd/place_route_obstacle.py --longitudinal 30 --intrusion 1.5 --label unknown --clear-first
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject

# 复用 obstacle_avoidance_limit_test 中的 DummyObject 工具
_UTILS_DIR = Path(__file__).resolve().parent / "obstacle_avoidance_limit_test" / "scripts"
sys.path.insert(0, str(_UTILS_DIR))

from dummy_object_utils import (  # noqa: E402
    compute_obstacle_pose,
    make_delete_all,
    make_dummy_object,
    unit_vectors,
)
from autoware_perception_msgs.msg import ObjectClassification  # noqa: E402

# ---------- 与 sim_init_and_set_goal.py 同步 ----------
START = {
    "x": 140.9329833984375,
    "y": -87.67423248291016,
    "z": -0.7914574554294266,
    "ox": -2.375657692689429e-05,
    "oy": 0.00010749767904279106,
    "oz": 0.21578949292521266,
    "ow": 0.9764399022074803,
}
GOAL = {
    "x": 221.13540649414062,
    "y": -48.24163818359375,
    "z": -0.811721052081776,
    "ox": 0.0,
    "oy": 0.0,
    "oz": 0.21578963533417528,
    "ow": 0.9764398769419158,
}

FRAME_ID = "map"
DUMMY_OBJECT_TOPIC = "/simulation/dummy_perception_publisher/object_info"

DEFAULT_LANE_WIDTH = 4.0
DEFAULT_SHOULDER = "right"
# 放在前方约 25m、略偏右侵入车道，保证与自车包络横向重叠以触发 simple_avoidance
DEFAULT_LONGITUDINAL = 25.0
DEFAULT_INTRUSION = 1.5
DEFAULT_LENGTH = 2.0
DEFAULT_WIDTH = 1.5
DEFAULT_HEIGHT = 1.5

LABEL_MAP = {
    "unknown": ObjectClassification.UNKNOWN,
    "car": ObjectClassification.CAR,
    "truck": ObjectClassification.TRUCK,
    "bus": ObjectClassification.BUS,
    "trailer": ObjectClassification.TRAILER,
    "motorcycle": ObjectClassification.MOTORCYCLE,
    "bicycle": ObjectClassification.BICYCLE,
    "pedestrian": ObjectClassification.PEDESTRIAN,
}

def quat_to_yaw(ox: float, oy: float, oz: float, ow: float) -> float:
    siny_cosp = 2.0 * (ow * oz + ox * oy)
    cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
    return math.atan2(siny_cosp, cosy_cosp)


def path_length(start: dict, goal: dict) -> float:
    dx = float(goal["x"]) - float(start["x"])
    dy = float(goal["y"]) - float(start["y"])
    return math.hypot(dx, dy)


def path_yaw(start: dict, goal: dict) -> float:
    return math.atan2(
        float(goal["y"]) - float(start["y"]),
        float(goal["x"]) - float(start["x"]),
    )


def pose_on_route(
    start: dict,
    goal: dict,
    *,
    fraction: float | None,
    longitudinal_m: float | None,
    lateral_m: float,
    use_start_heading: bool,
) -> tuple[float, float, float, float]:
    """返回 (x, y, z, yaw)。"""
    sx, sy, sz = float(start["x"]), float(start["y"]), float(start["z"])
    gx, gy = float(goal["x"]), float(goal["y"])

    if use_start_heading:
        yaw = quat_to_yaw(start["ox"], start["oy"], start["oz"], start["ow"])
    else:
        yaw = path_yaw(start, goal)

    if fraction is not None:
        frac = max(0.0, min(1.0, fraction))
        x = sx + frac * (gx - sx)
        y = sy + frac * (gy - sy)
    else:
        lon = longitudinal_m if longitudinal_m is not None else DEFAULT_LONGITUDINAL
        forward, right = unit_vectors(yaw)
        x = sx + forward[0] * lon + right[0] * lateral_m
        y = sy + forward[1] * lon + right[1] * lateral_m

    return x, y, sz, yaw


class RouteObstacleNode(Node):
    def __init__(self) -> None:
        super().__init__("place_route_obstacle")
        self.pub = self.create_publisher(DummyObject, DUMMY_OBJECT_TOPIC, 10)

    def clear_obstacles(self) -> None:
        self.pub.publish(make_delete_all(self.get_clock().now().to_msg(), FRAME_ID))

    def publish_obstacle(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        *,
        length: float,
        width: float,
        height: float,
        label: int,
        hold_sec: float,
        rate_hz: float,
    ) -> None:
        msg = make_dummy_object(
            FRAME_ID,
            self.get_clock().now().to_msg(),
            x,
            y,
            z,
            yaw,
            length=length,
            width=width,
            height=height,
            velocity=0.0,
            label=label,
        )

        rate = self.create_rate(rate_hz)
        if hold_sec <= 0.0:
            self.get_logger().info("持续发布障碍物，按 Ctrl+C 停止")
            while rclpy.ok():
                msg.header.stamp = self.get_clock().now().to_msg()
                self.pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.0)
                rate.sleep()
            return

        end = time.time() + hold_sec
        while time.time() < end and rclpy.ok():
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            rate.sleep()


def main() -> None:
    route_len = path_length(START, GOAL)
    start_yaw = quat_to_yaw(START["ox"], START["oy"], START["oz"], START["ow"])
    route_heading = path_yaw(START, GOAL)

    parser = argparse.ArgumentParser(
        description="在 sim_init_and_set_goal 路径上放置静态障碍物以触发绕障"
    )
    parser.add_argument(
        "--fraction",
        type=float,
        default=None,
        help="沿起终点连线的比例位置 [0,1]，例如 0.45 表示路径 45%% 处",
    )
    parser.add_argument(
        "--longitudinal",
        type=float,
        default=DEFAULT_LONGITUDINAL,
        help=f"沿起点朝向前进距离 [m]（默认 {DEFAULT_LONGITUDINAL}，路径总长约 {route_len:.1f}m）",
    )
    parser.add_argument(
        "--lateral",
        type=float,
        default=0.0,
        help="相对路径中心线的横向偏移 [m]，正=路径左侧",
    )
    parser.add_argument(
        "--intrusion",
        type=float,
        default=DEFAULT_INTRUSION,
        help=f"从路肩向车道内侵入 [m]（默认 {DEFAULT_INTRUSION}，与 --fraction 互斥时优先 lane 模式）",
    )
    parser.add_argument(
        "--lane-width",
        type=float,
        default=DEFAULT_LANE_WIDTH,
        help=f"车道宽度 [m]（默认 {DEFAULT_LANE_WIDTH}）",
    )
    parser.add_argument(
        "--shoulder",
        choices=("right", "left"),
        default=DEFAULT_SHOULDER,
        help="路肩所在侧（默认 right）",
    )
    parser.add_argument(
        "--use-route-heading",
        action="store_true",
        help="使用起终点连线方向而非起点朝向（默认用起点朝向）",
    )
    parser.add_argument(
        "--length",
        type=float,
        default=DEFAULT_LENGTH,
        help="障碍物长度 [m]",
    )
    parser.add_argument(
        "--width",
        type=float,
        default=DEFAULT_WIDTH,
        help="障碍物宽度 [m]",
    )
    parser.add_argument(
        "--height",
        type=float,
        default=DEFAULT_HEIGHT,
        help="障碍物高度 [m]",
    )
    parser.add_argument(
        "--label",
        choices=sorted(LABEL_MAP.keys()),
        default="unknown",
        help="感知分类标签（默认 unknown，对应 ObjectClassification.UNKNOWN=0）",
    )
    parser.add_argument(
        "--clear-first",
        action="store_true",
        help="放置前先 DELETEALL 清除已有 dummy 障碍物",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=0.0,
        help="持续发布秒数，0 表示一直发布直到 Ctrl+C",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="发布频率 [Hz]",
    )
    args = parser.parse_args()

    use_start_heading = not args.use_route_heading
    heading = start_yaw if use_start_heading else route_heading

    if args.fraction is not None:
        x, y, z, yaw = pose_on_route(
            START,
            GOAL,
            fraction=args.fraction,
            longitudinal_m=None,
            lateral_m=args.lateral,
            use_start_heading=use_start_heading,
        )
        placement = f"fraction={args.fraction:.2f}, lateral={args.lateral:.2f}m"
    else:
        x, y, yaw = compute_obstacle_pose(
            float(START["x"]),
            float(START["y"]),
            heading,
            args.lane_width,
            args.shoulder,
            args.longitudinal,
            args.intrusion,
            args.width,
        )
        z = float(START["z"])
        placement = (
            f"longitudinal={args.longitudinal:.1f}m, "
            f"intrusion={args.intrusion:.2f}m, shoulder={args.shoulder}"
        )

    rclpy.init()
    node = RouteObstacleNode()

    node.get_logger().info("=== 路径绕障障碍物 ===")
    node.get_logger().info(
        f"  起点: ({START['x']:.2f}, {START['y']:.2f}), yaw={math.degrees(start_yaw):.1f}°"
    )
    node.get_logger().info(
        f"  终点: ({GOAL['x']:.2f}, {GOAL['y']:.2f}), 路径长约 {route_len:.1f}m"
    )
    node.get_logger().info(f"  放置: {placement}")
    node.get_logger().info(f"  障碍物: ({x:.3f}, {y:.3f}, {z:.3f}), yaw={math.degrees(yaw):.1f}°")
    node.get_logger().info(
        f"  尺寸: L={args.length:.1f} W={args.width:.1f} H={args.height:.1f}, "
        f"label={args.label}({LABEL_MAP[args.label]})"
    )
    node.get_logger().info(f"  Topic: {DUMMY_OBJECT_TOPIC}")

    if args.clear_first:
        node.clear_obstacles()
        time.sleep(0.5)

    try:
        node.publish_obstacle(
            x,
            y,
            z,
            yaw,
            length=args.length,
            width=args.width,
            height=args.height,
            label=LABEL_MAP[args.label],
            hold_sec=args.hold,
            rate_hz=args.rate,
        )
    except KeyboardInterrupt:
        node.get_logger().info("停止发布")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
