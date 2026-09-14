#!/usr/bin/env python3
"""Autoware planning 仿真：初始化车辆位姿并发布导航终点。

用法:
  ./sim_init_and_set_goal.py              # 使用下方默认坐标
  ./sim_init_and_set_goal.py --engage     # 额外切换到 AUTONOMOUS 并 Engage

坐标可在脚本顶部 INIT_* / GOAL_* 变量中修改，对应地图:
  planning_simulator.launch.xml -> /home/nvidia/autoware_map/3_test/0727_lanelet2_map.osm
"""

from __future__ import annotations

import argparse
import math
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from tier4_control_msgs.srv import SetPause
from tier4_external_api_msgs.srv import Engage

# ---------- 初始位姿 (map 坐标系，log/local.log) ----------
INIT_X = 279.52191162109375
INIT_Y = -33.24296569824219
INIT_Z = -1.0602384937589355
INIT_OX = -0.00024534598908445367
INIT_OY = -0.00041156653669880767
INIT_OZ = -0.5120475691803335
INIT_OW = 0.8589569589419735

# ---------- 导航终点 (map 坐标系，log/goal.log) ----------
GOAL_X = 305.87670392887543
GOAL_Y = -123.50780948384602
GOAL_Z = -1.2911186136124073
GOAL_OX = -0.0008168357982079818
GOAL_OY = -0.00019858884363137125
GOAL_OZ = -0.9716948308437129
GOAL_OW = 0.23623811939091904

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
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose3d", pose_qos
        )

    def make_initial_pose(self) -> PoseWithCovarianceStamped:
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
        return pose

    def publish_initial_pose(self, repeat: int = 3) -> None:
        """Initialize the simulator's pose latch used by simple_planning_simulator."""
        deadline = time.monotonic() + 15.0
        while (
            self.initial_pose_pub.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.2)
        for index in range(repeat):
            pose = self.make_initial_pose()
            pose.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.1)
            if index + 1 < repeat:
                time.sleep(0.5)

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
            # Service discovery is delivered through this node's executor.  Spin a
            # little between probes so a newly started simulator is discoverable
            # without forcing the user to wait for a second terminal retry.
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

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

    def call_initialize_pose(self) -> bool:
        print("[1/3] 初始化车辆位姿...")
        client = self.create_client(InitializeLocalization, "/api/localization/initialize")
        pose = self.make_initial_pose()
        request = InitializeLocalization.Request()
        request.pose = [pose]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            print("  ! /api/localization/initialize 未在 10s 内响应，继续使用 /initialpose3d")
            return False
        return True

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("/api/routing/set_route_points request failed or timed out")

    def call_pause(self) -> None:
        print("  暂停车辆，准备设置路线...")
        client = self.create_client(SetPause, "/control/vehicle_cmd_gate/set_pause")
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("service unavailable: /control/vehicle_cmd_gate/set_pause")
        future = client.call_async(SetPause.Request(pause=True))
        rclpy.spin_until_future_complete(self, future)
        if not future.done() or future.result() is None:
            raise RuntimeError("set_pause request failed")

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
    parser.add_argument("--init-x", type=float, default=None, help="覆盖初始位姿 x")
    parser.add_argument("--init-y", type=float, default=None, help="覆盖初始位姿 y")
    parser.add_argument("--init-yaw", type=float, default=None, help="覆盖初始位姿 yaw [rad]")
    parser.add_argument("--goal-x", type=float, default=None, help="覆盖终点 x")
    parser.add_argument("--goal-y", type=float, default=None, help="覆盖终点 y")
    parser.add_argument("--goal-yaw", type=float, default=None, help="覆盖终点 yaw [rad]")
    return parser.parse_args()


def apply_pose_overrides(args: argparse.Namespace) -> None:
    """Apply optional map-coordinate overrides while keeping the legacy defaults."""
    global INIT_X, INIT_Y, INIT_OX, INIT_OY, INIT_OZ, INIT_OW
    global GOAL_X, GOAL_Y, GOAL_OX, GOAL_OY, GOAL_OZ, GOAL_OW

    init_values = (args.init_x, args.init_y, args.init_yaw)
    if any(value is not None for value in init_values):
        if not all(value is not None for value in init_values):
            raise ValueError("--init-x、--init-y、--init-yaw 必须同时提供")
        INIT_X = float(args.init_x)
        INIT_Y = float(args.init_y)
        INIT_OX = 0.0
        INIT_OY = 0.0
        INIT_OZ = math.sin(float(args.init_yaw) / 2.0)
        INIT_OW = math.cos(float(args.init_yaw) / 2.0)

    goal_values = (args.goal_x, args.goal_y, args.goal_yaw)
    if any(value is not None for value in goal_values):
        if not all(value is not None for value in goal_values):
            raise ValueError("--goal-x、--goal-y、--goal-yaw 必须同时提供")
        GOAL_X = float(args.goal_x)
        GOAL_Y = float(args.goal_y)
        GOAL_OX = 0.0
        GOAL_OY = 0.0
        GOAL_OZ = math.sin(float(args.goal_yaw) / 2.0)
        GOAL_OW = math.cos(float(args.goal_yaw) / 2.0)


def main() -> int:
    args = parse_args()
    try:
        apply_pose_overrides(args)
    except ValueError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2

    print("=== Autoware 仿真：初始化位姿 + 发布 Goal ===")
    print(f"  初始位姿: ({INIT_X}, {INIT_Y}, {INIT_Z})")
    print(f"  导航终点: ({GOAL_X}, {GOAL_Y}, {GOAL_Z})")
    print()

    rclpy.init()
    node = SimInitAndSetGoalNode()
    try:
        node.wait_for_service("/api/localization/initialize")
        node.wait_for_service("/api/routing/set_route_points")

        # The API localization request alone does not latch the pose in
        # simple_planning_simulator.  Publish the same transient-local pose
        # used by the acceptance driver before and after the API call.
        node.publish_initial_pose()
        node.call_initialize_pose()
        node.publish_initial_pose()
        # initialize_pose can release the gate even when the caller paused before
        # this script. Re-apply pause before the route request to honor
        # initial_engage_state=true without allowing pre-ADD movement.
        node.wait_for_service("/control/vehicle_cmd_gate/set_pause")
        node.call_pause()
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
