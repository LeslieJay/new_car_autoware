#!/usr/bin/env python3
"""Autoware planning 仿真：初始化车辆位姿并发布导航终点。

用法:
  ./sim_init_and_set_goal.py              # 使用下方默认坐标
  ./sim_init_and_set_goal.py --engage     # 额外切换到 AUTONOMOUS 并 Engage

坐标可在脚本顶部 INIT_* / GOAL_* 变量中修改，对应地图:
  planning_simulator.launch.xml -> /home/nvidia/autoware_map/9_out/parking.osm
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from autoware_adapi_v1_msgs.msg import RouteOption
from autoware_adapi_v1_msgs.srv import (
    ChangeOperationMode,
    InitializeLocalization,
    SetRoutePoints,
)
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Header
from tier4_control_msgs.srv import SetPause
from tier4_external_api_msgs.srv import Engage

# ---------- 初始位姿 (map 坐标系) ----------
INIT_X = 140.9329833984375
INIT_Y = -87.67423248291016
INIT_Z = -0.7914574554294266
INIT_OX = -2.375657692689429e-05
INIT_OY = 0.00010749767904279106
INIT_OZ = 0.21578949292521266
INIT_OW = 0.9764399022074803

# ---------- 导航终点 (map 坐标系) ----------
GOAL_X = 221.13540649414062
GOAL_Y = -48.24163818359375
GOAL_Z = -0.811721052081776
GOAL_OX = 0.0
GOAL_OY = 0.0
GOAL_OZ = 0.21578963533417528
GOAL_OW = 0.9764398769419158

ALLOW_GOAL_MODIFICATION = False
SERVICE_TIMEOUT = 120

INIT_COVARIANCE = [
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 10.0,
]


class SimInitAndSetGoalNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_init_and_set_goal")

    def wait_for_service(self, service_name: str, timeout: int = SERVICE_TIMEOUT) -> None:
        print(f"  等待服务: {service_name} (最长 {timeout}s)...")
        elapsed = 0
        while elapsed < timeout:
            names = {name for name, _ in self.get_service_names_and_types()}
            if service_name in names:
                print(f"  ✓ {service_name} 已就绪")
                return
            elapsed += 1
            if elapsed % 10 == 0:
                print(
                    f"    ... 已等待 {elapsed}s（请确认 planning_simulator 已启动并就绪）"
                )
            time.sleep(1)

        print(f"✗ 超时: {service_name} 不可用", file=sys.stderr)
        print("  排查:", file=sys.stderr)
        print(
            "    1. 先启动: ros2 launch autoware_launch planning_simulator.launch.xml",
            file=sys.stderr,
        )
        print(
            "    2. 等待日志出现 simple_planning_simulator / mission_planner 后再运行本脚本",
            file=sys.stderr,
        )
        print("    3. 检查: ros2 service list | grep localization", file=sys.stderr)
        raise RuntimeError(f"service unavailable: {service_name}")

    def call_initialize_pose(self) -> None:
        print("[1/3] 初始化车辆位姿...")
        client = self.create_client(InitializeLocalization, "/api/localization/initialize")

        pose = PoseWithCovarianceStamped()
        pose.header = Header(frame_id="map")
        pose.pose.pose.position.x = INIT_X
        pose.pose.pose.position.y = INIT_Y
        pose.pose.pose.position.z = INIT_Z
        pose.pose.pose.orientation.x = INIT_OX
        pose.pose.pose.orientation.y = INIT_OY
        pose.pose.pose.orientation.z = INIT_OZ
        pose.pose.pose.orientation.w = INIT_OW
        pose.pose.covariance = INIT_COVARIANCE

        request = InitializeLocalization.Request()
        request.pose = [pose]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def call_set_goal(self) -> None:
        print("[2/3] 发布导航终点...")
        client = self.create_client(SetRoutePoints, "/api/routing/set_route_points")

        goal = Pose()
        goal.position.x = GOAL_X
        goal.position.y = GOAL_Y
        goal.position.z = GOAL_Z
        goal.orientation.x = GOAL_OX
        goal.orientation.y = GOAL_OY
        goal.orientation.z = GOAL_OZ
        goal.orientation.w = GOAL_OW

        request = SetRoutePoints.Request()
        request.header = Header(frame_id="map")
        request.option = RouteOption(allow_goal_modification=ALLOW_GOAL_MODIFICATION)
        request.goal = goal
        request.waypoints = []
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def call_engage(self) -> None:
        print("[3/3] 切换到 AUTONOMOUS 并 Engage...")
        change_mode = self.create_client(
            ChangeOperationMode, "/api/operation_mode/change_to_autonomous"
        )
        enable_control = self.create_client(
            ChangeOperationMode, "/api/operation_mode/enable_autoware_control"
        )
        engage_client = self.create_client(Engage, "/api/autoware/set/engage")
        pause_client = self.create_client(SetPause, "/control/vehicle_cmd_gate/set_pause")

        for client in (change_mode, enable_control, engage_client, pause_client):
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"service unavailable: {client.srv_name}")

        for client in (change_mode, enable_control):
            future = client.call_async(ChangeOperationMode.Request())
            rclpy.spin_until_future_complete(self, future)
            time.sleep(1)

        engage_future = engage_client.call_async(Engage.Request(engage=True))
        rclpy.spin_until_future_complete(self, engage_future)
        time.sleep(1)

        pause_future = pause_client.call_async(SetPause.Request(pause=False))
        rclpy.spin_until_future_complete(self, pause_future)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Autoware 仿真：初始化位姿 + 发布 Goal")
    parser.add_argument(
        "--engage",
        action="store_true",
        help="初始化并设终点后，切换到 AUTONOMOUS 模式并 Engage",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    print("=== Autoware 仿真：初始化位姿 + 发布 Goal ===")
    print(f"  初始位姿: ({INIT_X}, {INIT_Y}, {INIT_Z})")
    print(f"  导航终点: ({GOAL_X}, {GOAL_Y}, {GOAL_Z})")
    print()

    rclpy.init()
    node = SimInitAndSetGoalNode()
    try:
        node.wait_for_service("/api/localization/initialize")
        node.wait_for_service("/api/routing/set_route_points")

        node.call_initialize_pose()
        time.sleep(2)
        node.call_set_goal()

        if args.engage:
            node.wait_for_service("/api/operation_mode/change_to_autonomous")
            node.wait_for_service("/api/autoware/set/engage")
            node.call_engage()
        else:
            print("[3/3] 跳过 Engage（planning_simulator 默认 initial_engage_state=true）")
            print(f"      若车辆未动，可重新运行: {sys.argv[0]} --engage")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print()
    print("=== 完成 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
