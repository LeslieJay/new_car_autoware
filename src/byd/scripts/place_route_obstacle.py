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
import json
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from tier4_simulation_msgs.msg import DummyObject

# 复用 obstacle_avoidance_limit_test 中的 DummyObject 工具
_UTILS_DIR = (
    Path(__file__).resolve().parent.parent / "obstacle_avoidance_limit_test" / "scripts"
)
sys.path.insert(0, str(_UTILS_DIR))

from dummy_object_utils import (  # noqa: E402
    compute_obstacle_pose,
    make_delete_all,
    make_dummy_object,
    unit_vectors,
)
from autoware_perception_msgs.msg import (  # noqa: E402
    ObjectClassification,
    PredictedObjects,
    TrackedObjects,
)
from nav_msgs.msg import Odometry

# ---------- 与 sim_init_and_set_goal.py 同步 ----------
START = {
    "x": 279.52191162109375,
    "y": -33.24296569824219,
    "z": -1.0602384937589355,
    "ox": -0.00024534598908445367,
    "oy": -0.00041156653669880767,
    "oz": -0.5120475691803335,
    "ow": 0.8589569589419735,
}
GOAL = {
    "x": 305.87670392887543,
    "y": -123.50780948384602,
    "z": -1.2911186136124073,
    "ox": -0.0008168357982079818,
    "oy": -0.00019858884363137125,
    "oz": -0.9716948308437129,
    "ow": 0.23623811939091904,
}

FRAME_ID = "map"
DUMMY_OBJECT_TOPIC = "/simulation/dummy_perception_publisher/object_info"
GROUND_TRUTH_TOPIC = "/simulation/dummy_perception_publisher/output/debug/ground_truth_objects"
GROUND_TRUTH_TOPIC_REMAP = "/simulation/debug/ground_truth_objects"
TRACKING_TOPIC = "/perception/object_recognition/tracking/objects"
PREDICTED_TOPIC = "/perception/object_recognition/objects"

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


def _raise_keyboard_interrupt(_signum: int, _frame) -> None:
    # Background processes inherit SIGINT=SIG_IGN from a non-interactive shell.
    # Restore Ctrl+C semantics explicitly so the finally block can DELETEALL.
    raise KeyboardInterrupt

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


def longitudinal(x: float, y: float) -> float:
    """Project a map point onto the selected straight test route."""
    route_yaw = path_yaw(START, GOAL)
    return (x - float(START["x"])) * math.cos(route_yaw) + (
        y - float(START["y"])
    ) * math.sin(route_yaw)


def apply_coordinate_overrides(
    start: dict,
    goal: dict,
    args: argparse.Namespace,
) -> None:
    """Apply the same map-coordinate overrides accepted by the init script."""
    start_values = (args.start_x, args.start_y, args.start_yaw)
    if any(value is not None for value in start_values):
        if not all(value is not None for value in start_values):
            raise ValueError("--start-x、--start-y、--start-yaw 必须同时提供")
        start["x"] = float(args.start_x)
        start["y"] = float(args.start_y)
        start["ox"] = 0.0
        start["oy"] = 0.0
        start["oz"] = math.sin(float(args.start_yaw) / 2.0)
        start["ow"] = math.cos(float(args.start_yaw) / 2.0)

    goal_values = (args.goal_x, args.goal_y, args.goal_yaw)
    if any(value is not None for value in goal_values):
        if not all(value is not None for value in goal_values):
            raise ValueError("--goal-x、--goal-y、--goal-yaw 必须同时提供")
        goal["x"] = float(args.goal_x)
        goal["y"] = float(args.goal_y)
        goal["ox"] = 0.0
        goal["oy"] = 0.0
        goal["oz"] = math.sin(float(args.goal_yaw) / 2.0)
        goal["ow"] = math.cos(float(args.goal_yaw) / 2.0)


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
        _, right = unit_vectors(yaw)
        # CLI 约定 lateral_m 正值表示路径左侧；unit_vectors 的第二个向量指向右侧。
        x = sx + frac * (gx - sx) - right[0] * lateral_m
        y = sy + frac * (gy - sy) - right[1] * lateral_m
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
        self.active_object_uuid: str | None = None
        self.ground_truth_objects: TrackedObjects | None = None
        self.tracking_objects: TrackedObjects | None = None
        self.predicted_objects: PredictedObjects | None = None
        self.latest_ego: tuple[float, float, float, float] | None = None
        self.create_subscription(
            TrackedObjects, GROUND_TRUTH_TOPIC, self._on_ground_truth, 10
        )
        # The default simulator launch remaps the relative publisher output to
        # /simulation/debug/ground_truth_objects.  Keep the original topic too,
        # because some simulator variants expose the unremapped name.
        self.create_subscription(
            TrackedObjects, GROUND_TRUTH_TOPIC_REMAP, self._on_ground_truth, 10
        )
        self.create_subscription(TrackedObjects, TRACKING_TOPIC, self._on_tracking, 10)
        self.create_subscription(PredictedObjects, PREDICTED_TOPIC, self._on_predicted, 10)
        self.create_subscription(Odometry, "/localization/kinematic_state", self._on_ego, 10)

    def _on_ground_truth(self, msg: TrackedObjects) -> None:
        self.ground_truth_objects = msg

    def _on_tracking(self, msg: TrackedObjects) -> None:
        self.tracking_objects = msg

    def _on_predicted(self, msg: PredictedObjects) -> None:
        self.predicted_objects = msg

    def _on_ego(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self.latest_ego = (
            float(pose.position.x),
            float(pose.position.y),
            float(quat_to_yaw(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)),
            float(msg.twist.twist.linear.x),
        )

    def wait_for_localization(self, timeout_sec: float = 15.0) -> tuple[float, float, float, float]:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_ego is not None:
                return self.latest_ego
        raise RuntimeError("定位数据未在规定时间内就绪")

    def wait_for_dummy_perception(
        self, timeout_sec: float = 15.0, min_subscribers: int = 1
    ) -> None:
        """Do not lose one-shot ADD before all requested subscribers discover this publisher."""
        deadline = time.monotonic() + timeout_sec
        while (
            self.pub.get_subscription_count() < min_subscribers
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.pub.get_subscription_count() < min_subscribers:
            raise RuntimeError(
                f"only {self.pub.get_subscription_count()} subscriber(s) discovered on "
                f"{DUMMY_OBJECT_TOPIC}; required {min_subscribers} within {timeout_sec:.1f}s"
            )

    @staticmethod
    def _tracked_ids(msg: TrackedObjects | PredictedObjects | None) -> set[str]:
        if msg is None:
            return set()
        return {bytes(obj.object_id.uuid).hex() for obj in msg.objects}

    def _publish_delete_all(self) -> None:
        self.pub.publish(make_delete_all(self.get_clock().now().to_msg(), FRAME_ID))

    def clear_obstacles_and_wait(
        self,
        timeout_sec: float = 10.0,
        confirm_samples: int = 5,
        forbidden_ids: set[str] | None = None,
    ) -> bool:
        """Delete all dummy objects and require an observed empty perception chain."""
        forbidden_ids = forbidden_ids or set()
        self.wait_for_dummy_perception(timeout_sec=min(timeout_sec, 15.0))
        deadline = time.monotonic() + timeout_sec
        empty_streak = 0
        while rclpy.ok() and time.monotonic() < deadline:
            self._publish_delete_all()
            rclpy.spin_once(self, timeout_sec=0.1)
            states = (
                self.ground_truth_objects,
                self.tracking_objects,
                self.predicted_objects,
            )
            ids = set().union(*(self._tracked_ids(state) for state in states))
            all_empty = (
                all(state is not None for state in states)
                and all(len(state.objects) == 0 for state in states if state is not None)
                and not (ids & forbidden_ids)
            )
            empty_streak = empty_streak + 1 if all_empty else 0
            if empty_streak >= confirm_samples:
                self.get_logger().info(
                    "障碍物清除确认成功: ground_truth=0 tracking=0 predicted=0"
                )
                return True
            time.sleep(0.1)
        self.get_logger().error(
            "障碍物清除确认超时: "
            f"ground_truth={None if self.ground_truth_objects is None else len(self.ground_truth_objects.objects)} "
            f"tracking={None if self.tracking_objects is None else len(self.tracking_objects.objects)} "
            f"predicted={None if self.predicted_objects is None else len(self.predicted_objects.objects)}"
        )
        return False

    def wait_for_active_obstacle(self, object_uuid: str, timeout_sec: float = 10.0) -> bool:
        """Require exactly one object in all perception outputs before driving."""
        deadline = time.monotonic() + timeout_sec
        active_streak = 0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            states = (
                self.ground_truth_objects,
                self.tracking_objects,
                self.predicted_objects,
            )
            ids = set().union(*(self._tracked_ids(state) for state in states))
            active = (
                all(state is not None for state in states)
                and all(len(state.objects) == 1 for state in states if state is not None)
                and object_uuid in ids
            )
            active_streak = active_streak + 1 if active else 0
            if active_streak >= 3:
                self.get_logger().info(
                    "障碍物生效确认成功: ground_truth=1 tracking=1 predicted=1"
                )
                return True
            time.sleep(0.1)
        self.get_logger().error("障碍物生效确认超时或对象数量不是 1")
        return False

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
        verify_active: bool,
        placement_metadata: dict | None = None,
        placement_json: Path | None = None,
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

        period_sec = 1.0 / rate_hz
        # Publish ADD exactly once. The dummy perception publisher treats every ADD as a
        # new object, even when the UUID is unchanged; subsequent messages must be MODIFY.
        object_uuid = bytes(msg.id.uuid).hex()
        self.active_object_uuid = object_uuid

        def publish_once(action: int) -> None:
            msg.action = action
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.get_logger().info("发送 DummyObject.ADD（仅一次）")
        publish_once(DummyObject.ADD)
        if placement_metadata is not None and placement_json is not None:
            metadata = dict(placement_metadata)
            metadata.update(
                {
                    "add_epoch": time.time(),
                    "add_ros_time": self.get_clock().now().nanoseconds / 1e9,
                    "object_uuid": object_uuid,
                }
            )
            placement_json.parent.mkdir(parents=True, exist_ok=True)
            placement_json.write_text(
                json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
        if verify_active and not self.wait_for_active_obstacle(object_uuid):
            raise RuntimeError("障碍物未能在三条感知链路中确认数量为 1")
        if not verify_active:
            self.get_logger().warning("跳过发布后感知数量确认，由 rosbag 分析最终计数")

        if hold_sec <= 0.0:
            self.get_logger().info("持续发布障碍物，按 Ctrl+C 停止")
            while rclpy.ok():
                publish_once(DummyObject.MODIFY)
                time.sleep(period_sec)
            return

        end = time.time() + hold_sec
        while time.time() < end and rclpy.ok():
            publish_once(DummyObject.MODIFY)
            time.sleep(period_sec)


def main() -> int:
    route_len = path_length(START, GOAL)

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
        "--ahead-of-ego",
        type=float,
        default=None,
        help="在发送 ADD 前将障碍物放到当前自车前方指定距离；与 --longitudinal/--fraction 互斥",
    )
    parser.add_argument(
        "--placement-json",
        type=Path,
        default=None,
        help="保存动态投放的自车位置、障碍物位置、实际间距、速度和时间戳",
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
        help="放置前持续 DELETEALL，直到三条感知链路连续确认为空",
    )
    parser.add_argument(
        "--clear-only",
        action="store_true",
        help="只执行清除并确认，不发送 ADD；失败返回非零",
    )
    parser.add_argument(
        "--clear-once",
        action="store_true",
        help="只发送一次 DELETEALL，不等待感知链路（快速切换测试点）",
    )
    parser.add_argument(
        "--clear-timeout",
        type=float,
        default=10.0,
        help="清除确认超时时间 [s]（默认 10）",
    )
    parser.add_argument(
        "--clear-confirm-samples",
        type=int,
        default=5,
        help="连续空样本确认次数（默认 5）",
    )
    parser.add_argument(
        "--wait-for-subscribers",
        type=int,
        default=1,
        help="发送 ADD 前等待的订阅者数量；录 bag 时建议为 2（默认 1）",
    )
    parser.add_argument(
        "--skip-active-check",
        action="store_true",
        help="发送 ADD 后不阻塞等待感知链路，由 bag 分析对象数量",
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
    parser.add_argument("--start-x", type=float, default=None, help="覆盖障碍物路径起点 x")
    parser.add_argument("--start-y", type=float, default=None, help="覆盖障碍物路径起点 y")
    parser.add_argument("--start-yaw", type=float, default=None, help="覆盖障碍物路径起点 yaw [rad]")
    parser.add_argument("--goal-x", type=float, default=None, help="覆盖障碍物路径终点 x")
    parser.add_argument("--goal-y", type=float, default=None, help="覆盖障碍物路径终点 y")
    parser.add_argument("--goal-yaw", type=float, default=None, help="覆盖障碍物路径终点 yaw [rad]")
    # 保留标准 ROS 2 参数（例如 ``--ros-args -r __node:=case_name``），使并行或
    # 连续边界测试可以为每个发布器使用唯一节点名，避免重复节点健康检查误报。
    args, ros_args = parser.parse_known_args()

    try:
        apply_coordinate_overrides(START, GOAL, args)
    except ValueError as exc:
        parser.error(str(exc))

    # Recompute all route geometry after applying CLI coordinates.  Keeping the
    # legacy defaults in the module-level dictionaries is useful for help text,
    # but placement must always use the selected test route.
    route_len = path_length(START, GOAL)
    start_yaw = quat_to_yaw(START["ox"], START["oy"], START["oz"], START["ow"])
    route_heading = path_yaw(START, GOAL)

    if args.clear_timeout <= 0.0 or args.clear_confirm_samples <= 0:
        parser.error("--clear-timeout 和 --clear-confirm-samples 必须为正数")
    if args.rate <= 0.0 or args.wait_for_subscribers <= 0:
        parser.error("--rate 必须为正数")
    if (args.clear_only or args.clear_once) and args.clear_first:
        parser.error("清除模式与 --clear-first 不能同时使用")
    if args.clear_only and args.clear_once:
        parser.error("--clear-only 与 --clear-once 不能同时使用")
    if args.ahead_of_ego is not None and args.ahead_of_ego <= 0.0:
        parser.error("--ahead-of-ego 必须为正数")
    if args.ahead_of_ego is not None and (
        args.fraction is not None
        or args.longitudinal != DEFAULT_LONGITUDINAL
        or "--longitudinal" in sys.argv[1:]
    ):
        parser.error("--ahead-of-ego 不能与 --fraction 或显式 --longitudinal 同时使用")
    if args.ahead_of_ego is not None and args.placement_json is None:
        parser.error("使用 --ahead-of-ego 时必须同时提供 --placement-json")

    # --clear-only 不需要计算或打印当前障碍物的放置位姿，但仍保留同一套
    # ROS 参数解析，方便直接在已经 source 的 Autoware 终端中调用。
    if args.clear_only:
        signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
        rclpy.init(args=ros_args)
        node = RouteObstacleNode()
        try:
            ok = node.clear_obstacles_and_wait(
                timeout_sec=args.clear_timeout,
                confirm_samples=args.clear_confirm_samples,
            )
            return 0 if ok else 2
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if args.clear_once:
        signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
        rclpy.init(args=ros_args)
        node = RouteObstacleNode()
        try:
            node.wait_for_dummy_perception()
            node._publish_delete_all()
            rclpy.spin_once(node, timeout_sec=0.2)
            node.get_logger().info("已发送一次 DELETEALL（快速切换，不等待空链路确认）")
            return 0
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    use_start_heading = not args.use_route_heading
    heading = start_yaw if use_start_heading else route_heading
    placement_metadata = None

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
    elif args.ahead_of_ego is not None:
        x = y = z = yaw = 0.0
        placement = (
            f"ahead_of_ego={args.ahead_of_ego:.2f}m, "
            f"intrusion={args.intrusion:.2f}m, shoulder={args.shoulder}"
        )
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

    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    rclpy.init(args=ros_args)
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

    obstacle_started = False
    try:
        node.wait_for_dummy_perception(min_subscribers=args.wait_for_subscribers)
        if args.clear_first and not node.clear_obstacles_and_wait(
            timeout_sec=args.clear_timeout,
            confirm_samples=args.clear_confirm_samples,
        ):
            return 2

        if args.ahead_of_ego is not None:
            ego_x, ego_y, ego_yaw, ego_speed = node.wait_for_localization()
            x, y, yaw = compute_obstacle_pose(
                ego_x,
                ego_y,
                ego_yaw,
                args.lane_width,
                args.shoulder,
                args.ahead_of_ego,
                args.intrusion,
                args.width,
            )
            z = float(START["z"])
            ego_forward = (math.cos(ego_yaw), math.sin(ego_yaw))
            ego_right = (math.sin(ego_yaw), -math.cos(ego_yaw))
            delta = (x - ego_x, y - ego_y)
            actual_ahead = delta[0] * ego_forward[0] + delta[1] * ego_forward[1]
            actual_lateral = delta[0] * ego_right[0] + delta[1] * ego_right[1]
            placement_metadata = {
                "requested_ahead_m": args.ahead_of_ego,
                "actual_ahead_m": actual_ahead,
                "actual_lateral_offset_m": actual_lateral,
                "speed_mps": abs(ego_speed),
                "ego": {"x": ego_x, "y": ego_y, "yaw": ego_yaw},
                "obstacle": {"x": x, "y": y, "z": z, "yaw": yaw},
                "route_longitudinal_m": longitudinal(x, y),
                "ego_route_longitudinal_m": longitudinal(ego_x, ego_y),
                "monitor_obstacle_longitudinal_m": longitudinal(ego_x, ego_y) + actual_ahead,
                "speed_target_mps": 2.0,
                "speed_tolerance_mps": 0.1,
                "distance_tolerance_m": 0.25,
                "placement_valid": (
                    abs(abs(ego_speed) - 2.0) <= 0.1
                    and abs(actual_ahead - args.ahead_of_ego) <= 0.25
                ),
            }
            node.get_logger().info(
                f"动态投放: ego_s={placement_metadata['ego_route_longitudinal_m']:.3f}m "
                f"obstacle_s={placement_metadata['route_longitudinal_m']:.3f}m "
                f"ahead={actual_ahead:.3f}m speed={abs(ego_speed):.3f}m/s"
            )

        obstacle_started = True
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
            verify_active=not args.skip_active_check,
            placement_metadata=placement_metadata,
            placement_json=args.placement_json,
        )
    except KeyboardInterrupt:
        node.get_logger().info("停止发布")
    except Exception as exc:  # noqa: BLE001 - report the lifecycle failure to the shell
        node.get_logger().error(f"障碍物发布失败: {exc}")
        return 1
    finally:
        if obstacle_started and rclpy.ok():
            node.get_logger().info("发布器退出，开始清除当前障碍物")
            if not node.clear_obstacles_and_wait(
                timeout_sec=args.clear_timeout,
                confirm_samples=args.clear_confirm_samples,
                forbidden_ids={node.active_object_uuid} if node.active_object_uuid else None,
            ):
                node.get_logger().error("退出时未能确认障碍物清除")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
