#!/usr/bin/env python3
"""Measure Simple Avoidance distance limits and candidate/final/actual paths.

The simulator is intentionally started separately with planning_simulator.launch.xml.
This node only publishes the initial pose/goal and dummy obstacle, records live
planning/vehicle samples, and writes reproducible CSV metrics.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from autoware_adapi_v1_msgs.msg import RouteOption
from autoware_adapi_v1_msgs.srv import ChangeOperationMode, InitializeLocalization, SetRoutePoints
from autoware_control_msgs.msg import Control
from autoware_internal_planning_msgs.msg import PathWithLaneId, PlanningFactorArray
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_perception_msgs.msg import PredictedObjects
from autoware_planning_msgs.msg import Path as PlanningPath
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import ControlModeReport, GearCommand, GearReport, TurnIndicatorsCommand
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log as RosLog
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tier4_simulation_msgs.msg import DummyObject
from byd_vehicle_msgs.msg import TrailerConfiguration
from tier4_control_msgs.srv import SetPause
from tier4_external_api_msgs.srv import Engage, InitializePose

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from dummy_object_utils import (  # noqa: E402
    compute_obstacle_pose,
    make_delete_all,
    make_dummy_object,
    yaw_to_quaternion,
)


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def find_param(data: object, key: str, default: float) -> float:
    if isinstance(data, dict):
        if key in data:
            return float(data[key])
        for value in data.values():
            found = find_param(value, key, default)
            if found != default:
                return found
    elif isinstance(data, list):
        for value in data:
            found = find_param(value, key, default)
            if found != default:
                return found
    return default


def pose_xy(msg: object) -> tuple[float, float]:
    pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
    return float(pose.position.x), float(pose.position.y)


def path_points(msg: object) -> list[tuple[float, float]]:
    points = getattr(msg, "points", [])
    result = []
    for item in points:
        pose = item.point.pose if hasattr(item, "point") else item.pose
        result.append((float(pose.position.x), float(pose.position.y)))
    return result


def project(x: float, y: float, ego: dict) -> tuple[float, float]:
    dx, dy = x - float(ego["x"]), y - float(ego["y"])
    c, s = math.cos(float(ego["yaw"])), math.sin(float(ego["yaw"]))
    return dx * c + dy * s, -dx * s + dy * c


def project_to_reference(x: float, y: float, reference: list[tuple[float, float]]) -> tuple[float, float]:
    """Project a point to a sampled route and return (s, signed lateral)."""
    if len(reference) < 2:
        return project(x, y, {"x": 0.0, "y": 0.0, "yaw": 0.0})
    best = (float("inf"), 0.0, 0.0)
    accumulated = 0.0
    for (x0, y0), (x1, y1) in zip(reference[:-1], reference[1:]):
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1.0e-6:
            continue
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (length * length)))
        px, py = x0 + t * dx, y0 + t * dy
        ex, ey = x - px, y - py
        d2 = ex * ex + ey * ey
        if d2 < best[0]:
            tx, ty = dx / length, dy / length
            best = (d2, accumulated + t * length, -ty * ex + tx * ey)
        accumulated += length
    return best[1], best[2]


def point_at_reference_s(
    reference: list[tuple[float, float]], target_s: float
) -> tuple[float, float, float] | None:
    """Interpolate a point and tangent on the sampled route centerline."""
    if len(reference) < 2:
        return None
    remaining = max(0.0, target_s)
    for (x0, y0), (x1, y1) in zip(reference[:-1], reference[1:]):
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length < 1.0e-6:
            continue
        if remaining <= length:
            ratio = remaining / length
            return x0 + ratio * dx, y0 + ratio * dy, math.atan2(dy, dx)
        remaining -= length
    x0, y0 = reference[-2]
    x1, y1 = reference[-1]
    return x1, y1, math.atan2(y1 - y0, x1 - x0)


def point_at_reference_s_lateral(
    reference: list[tuple[float, float]], target_s: float, lateral: float
) -> tuple[float, float, float] | None:
    """Return a point offset from the route centerline in its right-positive frame."""
    centerline = point_at_reference_s(reference, target_s)
    if centerline is None:
        return None
    x, y, yaw = centerline
    return (
        x + lateral * math.sin(yaw),
        y - lateral * math.cos(yaw),
        yaw,
    )


class DistanceNode(Node):
    def __init__(
        self,
        cfg: dict,
        mode: str,
        module_params: dict | None = None,
        lock_mode: str = "start_lock",
        launch_log: Path | None = None,
        scenario: str = "normal",
        speed_limit_mps: float | None = None,
        trailer_types: list[str] | None = None,
        target_loss_start_sec: float | None = None,
        target_loss_duration_sec: float | None = None,
        case_name: str | None = None,
    ) -> None:
        super().__init__("simple_avoidance_distance_test")
        self.cfg = cfg
        self.mode = mode
        self.module_params = module_params or {}
        self.lock_mode = lock_mode
        self.launch_log = launch_log
        self.ego = cfg["ego"]
        topics = cfg["topics"]
        mode_cfg = cfg.get("modes", {}).get(mode, {})
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.initial_pub = self.create_publisher(PoseWithCovarianceStamped, topics["initial_pose"], qos)
        self.goal_pub = self.create_publisher(PoseStamped, topics["goal"], qos)
        self.object_pub = self.create_publisher(DummyObject, topics["dummy_object"], 10)
        self.velocity_limit_pub = self.create_publisher(
            VelocityLimit, "/planning/scenario_planning/max_velocity_default", qos
        )
        self.trailer_configuration_pub = self.create_publisher(
            TrailerConfiguration, "/vehicle/status/trailer_configuration", qos
        )
        self.pose_service = self.create_client(InitializePose, "/api/simulator/set/pose")
        self.localization_service = self.create_client(InitializeLocalization, "/api/localization/initialize")
        self.route_service = self.create_client(SetRoutePoints, "/api/routing/set_route_points")
        self.change_mode_service = self.create_client(ChangeOperationMode, "/api/operation_mode/change_to_autonomous")
        self.enable_control_service = self.create_client(ChangeOperationMode, "/api/operation_mode/enable_autoware_control")
        self.engage_service = self.create_client(Engage, "/api/autoware/set/engage")
        self.pause_service = self.create_client(SetPause, "/control/vehicle_cmd_gate/set_pause")
        # The dummy vehicle model is geared. With initial_engage_state=false
        # keep an explicit DRIVE request available for the command gate.
        gear_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.gear_pub = self.create_publisher(
            GearCommand, "/control/shift_decider/gear_cmd", gear_qos
        )
        self.turn_indicator_pub = self.create_publisher(
            TurnIndicatorsCommand, "/planning/turn_indicators_cmd", gear_qos
        )
        self.latest_candidate: list[tuple[float, float]] = []
        self.latest_trajectory: list[tuple[float, float]] = []
        self.latest_behavior: list[tuple[float, float]] = []
        self.candidate_history: list[list[tuple[float, float]]] = []
        self.behavior_history: list[list[tuple[float, float]]] = []
        self.trajectory_history: list[list[tuple[float, float]]] = []
        self.trajectory_time_history: list[float] = []
        self.actual: list[tuple[float, float, float]] = []
        self.actual_time_history: list[float] = []
        self.speed_history: list[tuple[float, float]] = []
        self.latest_pose: tuple[float, float] = (float(self.ego["x"]), float(self.ego["y"]))
        self.latest_speed = 0.0
        # Command-chain probes.  These are deliberately recorded separately
        # from the planner metrics: a non-empty trajectory does not prove that
        # the controller received acceleration/odometry and emitted a command.
        self.topic_seen: dict[str, bool] = {}
        self.topic_last_monotonic: dict[str, float] = {}
        self.latest_gear_cmd: int | None = None
        self.latest_turn_cmd: int | None = None
        self.latest_gear_report: int | None = None
        self.latest_control_mode: int | None = None
        self.object_seen = False
        self.simple_shift = False
        self.obstacle_stop = False
        self.failure_reasons: set[str] = set()
        self.obstacle_msg: DummyObject | None = None
        self.secondary_obstacle_msg: DummyObject | None = None
        self.target_hidden = False
        self.secondary_hidden = False
        self.scenario = scenario if scenario != "normal" else str(mode_cfg.get("scenario", "normal"))
        self.speed_limit_mps = speed_limit_mps
        self.trailer_types = list(trailer_types or [])
        self.target_loss_start_sec = target_loss_start_sec
        self.target_loss_duration_sec = target_loss_duration_sec
        self.case_name = case_name or self.scenario
        self.turn_signal_history: list[tuple[float, int]] = []
        self.baseline_path: list[tuple[float, float]] = []
        self.obstacle_active = False
        self.first_target_lock: dict[str, float] | None = None
        self.first_feasible: dict[str, float] | None = None
        self.first_infeasible: dict[str, float] | None = None
        self.first_decision: str | None = None
        self.log_metrics: dict[str, object] = {}
        self.case_start_epoch = time.time()
        self.obstacle_s_center: float | None = None
        self.obstacle_l_center: float | None = None
        self.obstacle_yaw: float | None = None
        self.invalid_reason: str | None = None
        self.preflight_s_end_m: float | None = None
        self.preflight_speed_max_mps: float | None = None
        self.candidate_topic = mode_cfg.get("candidate_path", topics["candidate_path"])
        self.factors_topic = mode_cfg.get("planning_factors", topics["simple_factors"])
        self.create_subscription(PlanningPath, self.candidate_topic, self._candidate, 10)
        self.create_subscription(PathWithLaneId, topics["behavior_path"], self._behavior_path, 10)
        self.create_subscription(Trajectory, topics["trajectory"], self._trajectory, 10)
        self.create_subscription(Odometry, topics["odometry"], self._odom, 10)
        self.create_subscription(AccelWithCovarianceStamped, "/localization/acceleration", self._accel, 10)
        self.create_subscription(Control, "/control/trajectory_follower/control_cmd", self._follower_control, 10)
        self.create_subscription(Control, "/control/command/control_cmd", self._gate_control, 10)
        self.create_subscription(GearCommand, "/control/command/gear_cmd", self._gate_gear, 10)
        self.create_subscription(TurnIndicatorsCommand, "/control/command/turn_indicators_cmd", self._gate_turn, 10)
        self.create_subscription(GearReport, "/vehicle/status/gear_status", self._gear_report, 10)
        self.create_subscription(ControlModeReport, "/vehicle/status/control_mode", self._control_mode, 10)
        self.create_subscription(PredictedObjects, topics["predicted_objects"], self._objects, 10)
        self.create_subscription(PlanningFactorArray, self.factors_topic, self._simple_factors, 10)
        self.create_subscription(PlanningFactorArray, topics["obstacle_stop_factors"], self._stop_factors, 10)
        rosout_qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            # /rosout uses the live log stream QoS.  Requesting transient-local
            # here can make the subscription incompatible with a volatile
            # rosout publisher, so runtime lock messages would be silently
            # missed even though `ros2 topic echo` can observe them.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(RosLog, "/rosout", self._rosout, rosout_qos)

    def _candidate(self, msg: PlanningPath) -> None:
        points = path_points(msg)
        # The module publishes a nominal/empty candidate again after the
        # maneuver completes.  Keep non-empty samples so the report measures
        # the maneuver rather than the final idle publication.
        if points:
            self.latest_candidate = points
            self.candidate_history.append(points)

    def _trajectory(self, msg: Trajectory) -> None:
        points = path_points(msg)
        if points:
            self.latest_trajectory = points
            self.trajectory_history.append(points)
            self.trajectory_time_history.append(time.monotonic())

    def _behavior_path(self, msg: PathWithLaneId) -> None:
        points = path_points(msg)
        if points:
            self.latest_behavior = points
            self.behavior_history.append(points)
            # Once a module is isolated, a shifted behavior path is a robust
            # indication of an approved maneuver.  Planning factors and path
            # candidates are intentionally cleared when the module is active.
            if self.obstacle_active:
                lateral = (
                    max(abs(project_to_reference(x, y, self.baseline_path)[1]) for x, y in points)
                    if self.baseline_path else
                    max(abs(project(x, y, self.ego)[1]) for x, y in points)
                )
                if lateral >= 0.2:
                    self.simple_shift = True

    def _odom(self, msg: Odometry) -> None:
        x, y = pose_xy(msg)
        speed = float(msg.twist.twist.linear.x)
        # Keep world coordinates here.  Reconstructing a world pose from the
        # initial ego frame loses the route curvature after the vehicle moves
        # through a bend and makes obstacle-passing/clearance metrics disagree
        # with the planner's actual pose.
        self.actual.append((x, y, speed))
        self.actual_time_history.append(time.monotonic())
        self.latest_pose = (x, y)
        self.latest_speed = speed
        self.speed_history.append((time.monotonic(), abs(speed)))

    def _mark_topic(self, name: str) -> None:
        self.topic_seen[name] = True
        self.topic_last_monotonic[name] = time.monotonic()

    def _accel(self, _msg: AccelWithCovarianceStamped) -> None:
        self._mark_topic("localization_acceleration")

    def _follower_control(self, _msg: Control) -> None:
        self._mark_topic("trajectory_follower_control")

    def _gate_control(self, _msg: Control) -> None:
        self._mark_topic("gate_control")

    def _gate_gear(self, msg: GearCommand) -> None:
        self._mark_topic("gate_gear")
        self.latest_gear_cmd = int(msg.command)

    def _gate_turn(self, msg: TurnIndicatorsCommand) -> None:
        self._mark_topic("gate_turn")
        self.latest_turn_cmd = int(msg.command)
        self.turn_signal_history.append((time.monotonic(), int(msg.command)))

    def _gear_report(self, msg: GearReport) -> None:
        self._mark_topic("gear_report")
        self.latest_gear_report = int(msg.report)

    def _control_mode(self, msg: ControlModeReport) -> None:
        self._mark_topic("control_mode")
        self.latest_control_mode = int(msg.mode)

    def _objects(self, msg: PredictedObjects) -> None:
        self.object_seen = self.object_seen or bool(msg.objects)

    def _simple_factors(self, msg: PlanningFactorArray) -> None:
        for factor in msg.factors:
            if int(factor.behavior) == 3:
                self.obstacle_stop = True
            if int(factor.behavior) in (4, 5):
                self.simple_shift = True
            reason = getattr(factor, "reason", "")
            if reason:
                self.failure_reasons.add(str(reason))

    def _stop_factors(self, msg: PlanningFactorArray) -> None:
        self.obstacle_stop = self.obstacle_stop or any(int(f.behavior) == 3 for f in msg.factors)

    def _rosout(self, msg: RosLog) -> None:
        """Capture the module's runtime target distance and feasibility decision.

        The Simple Avoidance module publishes these values through its standard
        ROS log stream.  Keeping the first occurrence per case avoids confusing
        the initial placement distance with the continuously updated target
        distance after the ego vehicle starts moving.
        """
        if not self.obstacle_active:
            return
        module_name = "simple_avoidance" if self.mode == "simple_avoidance" else "simple_lane_change_avoidance"
        if not str(msg.name).endswith(module_name):
            return
        text = str(msg.msg)
        stamp = float(msg.stamp.sec) + 1.0e-9 * float(msg.stamp.nanosec)
        reason_match = re.search(r"reason=([a-z_]+)", text)
        if reason_match:
            reason = reason_match.group(1)
            self.failure_reasons.add(reason)
            if reason == "infeasible_distance" and self.first_decision is None:
                self.first_decision = "INFEASIBLE_DISTANCE"
        lock = re.search(r"active target locked .* lon=([+-]?\d+(?:\.\d+)?)m lat=([+-]?\d+(?:\.\d+)?)m", text)
        if lock and self.first_target_lock is None:
            self.first_target_lock = {
                "lon": float(lock.group(1)), "lat": float(lock.group(2)), "time": stamp,
            }
        feasible = re.search(
            r"avoidance path generated .* target_lon=([+-]?\d+(?:\.\d+)?) target_lat=([+-]?\d+(?:\.\d+)?) "
            r".* jerk_distance=([+-]?\d+(?:\.\d+)?) ego_speed=([+-]?\d+(?:\.\d+)?) "
            r"dist_to_shift_end=([+-]?\d+(?:\.\d+)?) dist_to_obstacle=([+-]?\d+(?:\.\d+)?)",
            text,
        )
        if feasible and self.first_feasible is None:
            self.first_feasible = {
                "lon": float(feasible.group(1)),
                "lat": float(feasible.group(2)),
                "jerk_distance": float(feasible.group(3)),
                "ego_speed": float(feasible.group(4)),
                "dist_to_shift_end": float(feasible.group(5)),
                "dist_to_obstacle": float(feasible.group(6)),
                "time": stamp,
            }
        infeasible = re.search(
            r"pass-through reason=infeasible_distance .* lon=([+-]?\d+(?:\.\d+)?) obj_hl=([+-]?\d+(?:\.\d+)?) "
            r".* dist_to_shift_end=([+-]?\d+(?:\.\d+)?) .* dist_to_obstacle=([+-]?\d+(?:\.\d+)?) "
            r".* shortfall=([+-]?\d+(?:\.\d+)?) .* ego_speed=([+-]?\d+(?:\.\d+)?)m/s jerk_distance=([+-]?\d+(?:\.\d+)?)",
            text,
        )
        if infeasible and self.first_infeasible is None:
            self.first_infeasible = {
                "lon": float(infeasible.group(1)),
                "object_half_length": float(infeasible.group(2)),
                "dist_to_shift_end": float(infeasible.group(3)),
                "dist_to_obstacle": float(infeasible.group(4)),
                "shortfall": float(infeasible.group(5)),
                "ego_speed": float(infeasible.group(6)),
                "jerk_distance": float(infeasible.group(7)),
                "time": stamp,
            }

    def _call_service(self, client, request, wait_sec: float = 10.0, timeout_sec: float = 5.0) -> bool:
        if not client.wait_for_service(timeout_sec=wait_sec):
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        return bool(future.done() and future.result() is not None)

    def engage_vehicle(self) -> bool:
        return self._call_service(self.engage_service, Engage.Request(engage=True), wait_sec=5.0)

    def publish_drive_gear(self) -> None:
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = GearCommand.DRIVE
        self.gear_pub.publish(msg)

    def publish_turn_indicator(self) -> None:
        # The planner owns /planning/turn_indicators_cmd.  A test-side DISABLE
        # publication would race the module and make signal verification
        # meaningless.  Keep the old override only as an explicit diagnostic
        # escape hatch.
        if os.environ.get("PUBLISH_TEST_TURN_DISABLE", "0") != "1":
            return
        msg = TurnIndicatorsCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = TurnIndicatorsCommand.DISABLE
        self.turn_indicator_pub.publish(msg)

    def publish_speed_limit(self) -> None:
        if self.speed_limit_mps is None:
            return
        msg = VelocityLimit()
        msg.stamp = self.get_clock().now().to_msg()
        msg.max_velocity = max(0.0, float(self.speed_limit_mps))
        msg.use_constraints = False
        msg.sender = "simple_lc_avoidance_acceptance"
        self.velocity_limit_pub.publish(msg)

    def publish_trailer_configuration(self) -> None:
        if not self.trailer_types:
            return
        msg = TrailerConfiguration()
        msg.header.frame_id = self.cfg["frame_id"]
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.trailer_types = list(self.trailer_types)
        self.trailer_configuration_pub.publish(msg)

    def activate_vehicle(self, retries: int = 5) -> bool:
        """Enter autonomous control after the route callback has settled."""
        acknowledged = False
        for _ in range(retries):
            acknowledged = self.set_autonomous(engage=True) or acknowledged
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_speed > 0.05:
                return True
            time.sleep(0.5)
        return acknowledged

    def set_autonomous(self, engage: bool = False) -> bool:
        """Apply the same mode hand-off used by the manual simulator workflow.

        ``initial_engage_state=true`` makes the simulator motion-enabled at
        startup.  In that configuration the test only needs the two operation
        mode transitions; Engage is retained as an optional compatibility step
        for instances started with ``initial_engage_state=false``.
        """
        acknowledged = False
        for client in (self.change_mode_service, self.enable_control_service):
            acknowledged = self._call_service(
                client, ChangeOperationMode.Request(), wait_sec=10.0
            ) or acknowledged
            time.sleep(0.5)
        if engage:
            acknowledged = self.engage_vehicle() or acknowledged
            acknowledged = self._call_service(
                self.pause_service, SetPause.Request(pause=False), wait_sec=5.0
            ) or acknowledged
        # Keep these alive for the command gate/shift decider while the mode
        # transition propagates.  They are not used as a substitute for the
        # actual control command in the motion test.
        self.publish_drive_gear()
        self.publish_turn_indicator()
        return acknowledged

    def wait_for_stable_speed(
        self, lower: float = 1.9, upper: float = 2.1, duration_sec: float = 1.0, timeout_sec: float = 25.0
    ) -> bool:
        start = time.monotonic()
        last_command = 0.0
        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_command >= 0.2:
                self.publish_drive_gear()
                self.publish_turn_indicator()
                last_command = now
            recent = [speed for stamp, speed in self.speed_history if now - stamp <= duration_sec]
            if len(recent) >= 8 and all(lower <= speed <= upper for speed in recent):
                return True
            time.sleep(0.02)
        return False

    def wait_for_vehicle_motion(self, timeout_sec: float = 8.0) -> bool:
        """Verify that the four-step mode hand-off actually releases motion."""
        start_s = project_to_reference(*self.latest_pose, self.baseline_path)[0] if self.baseline_path else 0.0
        start = time.monotonic()
        last_command = 0.0
        while rclpy.ok() and time.monotonic() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_command >= 0.2:
                self.publish_drive_gear()
                self.publish_turn_indicator()
                last_command = now
            current_s = project_to_reference(*self.latest_pose, self.baseline_path)[0] if self.baseline_path else 0.0
            if self.latest_speed > 0.2 or current_s - start_s > 0.2:
                return True
            time.sleep(0.02)
        return False

    def publish_setup(self, repeat: int = 3, engage: bool = True) -> None:
        deadline = time.monotonic() + 20.0
        while self.initial_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.initial_pub.get_subscription_count() == 0:
            self.get_logger().warning("no simulator subscriber discovered on initial pose topic")
        # These are transient-local inputs. Publishing them before the route
        # hand-off makes the requested speed class and articulated geometry
        # available to every planner cycle in the case.
        self.publish_speed_limit()
        self.publish_trailer_configuration()
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.cfg["frame_id"]
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = float(self.ego["x"])
        pose.pose.pose.position.y = float(self.ego["y"])
        pose.pose.pose.position.z = float(self.ego.get("z", 0.0))
        pose.pose.pose.orientation = yaw_to_quaternion(float(self.ego["yaw"]))
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685
        goal_cfg = self.cfg["goal"]
        goal = PoseStamped()
        goal.header.frame_id = self.cfg["frame_id"]
        goal.header.stamp = pose.header.stamp
        goal.pose.position.x = float(goal_cfg["x"])
        goal.pose.position.y = float(goal_cfg["y"])
        goal.pose.position.z = float(goal_cfg.get("z", 0.0))
        goal.pose.orientation = yaw_to_quaternion(float(goal_cfg["yaw"]))
        if self.localization_service.wait_for_service(timeout_sec=10.0):
            request = InitializeLocalization.Request()
            request.pose = [pose]
            if self._call_service(self.localization_service, request, wait_sec=10.0, timeout_sec=5.0):
                self.get_logger().info("initialized localization through /api/localization/initialize")

        # This is the actual "set initial position" step used by the manual
        # planning_simulator workflow.  The simulator subscribes to
        # /initialpose3d; localization initialization alone can leave its
        # is_initialized_ flag false and the controller then waits forever for
        # acceleration/trajectory data.  Publish before setting the route so a
        # later autonomous-mode transition never resets the pose.
        for index in range(repeat):
            pose.header.stamp = self.get_clock().now().to_msg()
            self.initial_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.1)
            if index + 1 < repeat:
                time.sleep(0.5)

        # The direct simulator pose API can be useful for diagnostics, but can
        # reset the vehicle after the route is accepted, so it is opt-in.
        # Keep the old API as an explicit diagnostic escape hatch only.
        if os.environ.get("DIRECT_SIMULATOR_POSE", "0") == "1" and self.pose_service.wait_for_service(timeout_sec=10.0):
            request = InitializePose.Request()
            request.pose = pose
            if self._call_service(self.pose_service, request, wait_sec=10.0, timeout_sec=5.0):
                self.get_logger().info("initialized simulator through /api/simulator/set/pose")

        if self.route_service.wait_for_service(timeout_sec=10.0):
            request = SetRoutePoints.Request()
            request.header = pose.header
            request.option = RouteOption(allow_goal_modification=False)
            request.goal = goal.pose
            request.waypoints = []
            if self._call_service(self.route_service, request, wait_sec=10.0, timeout_sec=5.0):
                self.get_logger().info("set route through /api/routing/set_route_points")

        service_available = False
        # Route services are asynchronous from the planner's point of view.
        # Let route/state messages propagate before changing operation mode.
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)
        # Follow the fourth manual step after pose and route have settled. The
        # simulator launch normally starts engaged, but an explicit Engage /
        # unpause makes the test deterministic when operation-mode services
        # finish discovery after the initial route request. Do not publish
        # /initialpose here: doing so would reset the simulator after a
        # route/control command has been accepted.
        service_available = self.set_autonomous(engage=True)
        if not service_available:
            self.get_logger().warning("autonomous mode services unavailable; publishing initial pose fallback")
            pose.header.stamp = self.get_clock().now().to_msg()
            self.initial_pub.publish(pose)

    def clear(self) -> None:
        self.object_pub.publish(make_delete_all(self.get_clock().now().to_msg(), self.cfg["frame_id"]))
        self.obstacle_msg = None
        self.secondary_obstacle_msg = None

    def parse_launch_log(self, case_start: float, case_end: float) -> None:
        """Extract the first lock and its first same-target decision from launch.log.

        The module logger is the authoritative source for the target's runtime
        longitudinal distance.  Parsing the launch log also avoids DDS QoS
        differences on /rosout and lets us associate a decision with the UUID
        that was actually locked in this fresh simulator instance.
        """
        if self.launch_log is None or not self.launch_log.exists():
            return
        module_name = "simple_avoidance" if self.mode == "simple_avoidance" else "simple_lane_change_avoidance"
        prefix = re.compile(r"^\s*(\d+(?:\.\d+)?)")
        lock_re = re.compile(
            r"active target locked uuid=([0-9a-f]+) lon=([+-]?\d+(?:\.\d+)?)m lat=([+-]?\d+(?:\.\d+)?)m"
        )
        feasible_re = re.compile(
            r"avoidance path generated shift=([+-]?\d+(?:\.\d+)?) target_lon=([+-]?\d+(?:\.\d+)?) "
            r"target_lat=([+-]?\d+(?:\.\d+)?) required_clearance=([+-]?\d+(?:\.\d+)?) "
            r"jerk_distance=([+-]?\d+(?:\.\d+)?) ego_speed=([+-]?\d+(?:\.\d+)?) "
            r"dist_to_shift_end=([+-]?\d+(?:\.\d+)?) dist_to_obstacle=([+-]?\d+(?:\.\d+)?)"
        )
        infeasible_re = re.compile(
            r"pass-through reason=infeasible_distance \| target uuid=([0-9a-f]+) "
            r"lon=([+-]?\d+(?:\.\d+)?)m obj_hl=([+-]?\d+(?:\.\d+)?)m \| "
            r"dist_to_shift_end=([+-]?\d+(?:\.\d+)?)m .*?dist_to_obstacle=([+-]?\d+(?:\.\d+)?)m "
            r"\| shortfall=([+-]?\d+(?:\.\d+)?)m .*?ego_speed=([+-]?\d+(?:\.\d+)?)m/s "
            r"jerk_distance=([+-]?\d+(?:\.\d+)?)m"
        )
        locks = []
        try:
            lines = self.launch_log.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return
        for line in lines:
            if module_name not in line:
                continue
            match_stamp = prefix.match(line)
            if not match_stamp:
                continue
            stamp = float(match_stamp.group(1))
            if stamp < case_start - 2.0 or stamp > case_end + 2.0:
                continue
            lock = lock_re.search(line)
            if lock:
                locks.append((stamp, lock.group(1), float(lock.group(2)), float(lock.group(3))))

        if not locks:
            # ros2 launch can flush launch.log with a clock source that is a
            # few seconds outside the Python wall-clock window, especially
            # while the host is under load. This runner starts one simulator
            # per acceptance scenario, so a lock outside the window is still
            # unambiguously part of this case. Apply the spatial selection
            # below rather than turning a valid maneuver into INVALID merely
            # because the log timestamp was delayed.
            for line in lines:
                if module_name not in line:
                    continue
                lock = lock_re.search(line)
                if lock:
                    match_stamp = prefix.match(line)
                    stamp = float(match_stamp.group(1)) if match_stamp else case_start
                    locks.append((stamp, lock.group(1), float(lock.group(2)), float(lock.group(3))))
        if not locks:
            return
        # A fresh simulator should have one target.  If a tracker diagnostic
        # leaves a ghost target, prefer a lock whose lateral coordinate is near
        # the published obstacle and whose longitudinal coordinate is near its
        # expected reference-path location.
        selected = locks[0]
        if self.obstacle_s_center is not None and self.obstacle_l_center is not None:
            candidates = [
                item for item in locks
                if abs(item[3] - self.obstacle_l_center) <= 0.8
                and abs(item[2] - self.obstacle_s_center) <= max(4.0, 0.35 * abs(self.obstacle_s_center))
            ]
            if candidates:
                selected = candidates[0]
        lock_stamp, lock_uuid, lock_lon, lock_lat = selected
        self.log_metrics["first_target_lock_speed_mps"] = None
        self.log_metrics["first_target_lock_lon_m"] = lock_lon
        self.log_metrics["first_target_lock_lat_m"] = lock_lat
        self.log_metrics["first_target_lock_time_s"] = lock_stamp

        for line in lines:
            if module_name not in line:
                continue
            match_stamp = prefix.match(line)
            if not match_stamp:
                continue
            stamp = float(match_stamp.group(1))
            if stamp < lock_stamp or stamp > case_end + 2.0:
                continue
            feasible = feasible_re.search(line)
            if feasible and abs(float(feasible.group(3)) - lock_lat) > 0.8:
                # A stale/ghost tracker target can produce a feasible path in
                # the same route.  It is not the decision for the target that
                # was first locked in this case.
                feasible = None
            if feasible:
                self.first_decision = "FEASIBLE"
                self.log_metrics.update(
                    {
                        "first_decision": "FEASIBLE",
                        "first_target_lock_speed_mps": float(feasible.group(6)),
                        "first_feasible_lon_m": float(feasible.group(2)),
                        "first_feasible_ego_speed_mps": float(feasible.group(6)),
                        "first_feasible_time_s": stamp,
                        "first_feasible_dist_to_shift_end_m": float(feasible.group(7)),
                        "first_feasible_dist_to_obstacle_m": float(feasible.group(8)),
                        "first_feasible_jerk_distance_m": float(feasible.group(5)),
                    }
                )
                return
            if lock_uuid not in line:
                continue
            infeasible = infeasible_re.search(line)
            if infeasible:
                self.first_decision = "INFEASIBLE_DISTANCE"
                self.log_metrics.update(
                    {
                        "first_decision": "INFEASIBLE_DISTANCE",
                        "first_target_lock_speed_mps": float(infeasible.group(7)),
                        "first_infeasible_lon_m": float(infeasible.group(2)),
                        "first_infeasible_object_half_length_m": float(infeasible.group(3)),
                        "first_infeasible_time_s": stamp,
                        "first_infeasible_dist_to_shift_end_m": float(infeasible.group(4)),
                        "first_infeasible_dist_to_obstacle_m": float(infeasible.group(5)),
                        "first_infeasible_shortfall_m": float(infeasible.group(6)),
                        "first_infeasible_ego_speed_mps": float(infeasible.group(7)),
                        "first_infeasible_jerk_distance_m": float(infeasible.group(8)),
                    }
                )
                return

    def _apply_log_metrics(self) -> None:
        if not self.log_metrics:
            return
        lock = self.log_metrics
        self.first_target_lock = {
            "lon": float(lock["first_target_lock_lon_m"]),
            "lat": float(lock["first_target_lock_lat_m"]),
            "time": float(lock["first_target_lock_time_s"]),
        }
        self.first_decision = str(lock.get("first_decision")) if lock.get("first_decision") else self.first_decision
        if lock.get("first_feasible_lon_m") is not None:
            self.first_feasible = {
                "lon": float(lock["first_feasible_lon_m"]),
                "ego_speed": float(lock["first_feasible_ego_speed_mps"]),
                "time": float(lock["first_feasible_time_s"]),
                "dist_to_shift_end": float(lock["first_feasible_dist_to_shift_end_m"]),
                "dist_to_obstacle": float(lock["first_feasible_dist_to_obstacle_m"]),
                "jerk_distance": float(lock["first_feasible_jerk_distance_m"]),
            }
        if lock.get("first_infeasible_lon_m") is not None:
            self.first_infeasible = {
                "lon": float(lock["first_infeasible_lon_m"]),
                "object_half_length": float(lock["first_infeasible_object_half_length_m"]),
                "ego_speed": float(lock["first_infeasible_ego_speed_mps"]),
                "time": float(lock["first_infeasible_time_s"]),
                "dist_to_shift_end": float(lock["first_infeasible_dist_to_shift_end_m"]),
                "dist_to_obstacle": float(lock["first_infeasible_dist_to_obstacle_m"]),
                "shortfall": float(lock["first_infeasible_shortfall_m"]),
            }

    def publish_obstacle(
        self,
        longitudinal: float,
        intrusion: float,
        shoulder: str,
        anchor_xy: tuple[float, float] | None = None,
        visible: bool = True,
    ) -> tuple[float, float, float]:
        obs = self.cfg["obstacle"]
        anchor_x, anchor_y = anchor_xy or (float(self.ego["x"]), float(self.ego["y"]))
        # Follow the selected route when it bends.  A fixed start yaw would
        # place a long-distance target off-road on this map, whose start and
        # goal are on different lanelets.
        target = None
        if self.baseline_path:
            anchor_s, _ = project_to_reference(anchor_x, anchor_y, self.baseline_path)
            target = point_at_reference_s(self.baseline_path, anchor_s + longitudinal)
        if target is not None:
            target_x, target_y, target_yaw = target
            x, y, yaw = compute_obstacle_pose(
                target_x, target_y, target_yaw, float(obs["lane_width"]),
                shoulder, 0.0, intrusion, float(obs["width"]),
            )
        else:
            x, y, yaw = compute_obstacle_pose(
                anchor_x, anchor_y, float(obs["lane_yaw"]),
                float(obs["lane_width"]), shoulder, longitudinal, intrusion,
                float(obs["width"]),
            )
        self.obstacle_yaw = yaw
        if self.obstacle_msg is None:
            self.obstacle_msg = make_dummy_object(
                self.cfg["frame_id"], self.get_clock().now().to_msg(), x, y,
                float(self.ego.get("z", 0.0)), yaw, length=float(obs["length"]),
                width=float(obs["width"]), height=float(obs["height"]), velocity=0.0,
            )
        msg = self.obstacle_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        if not visible:
            if not self.target_hidden:
                msg.action = DummyObject.DELETE
                self.object_pub.publish(msg)
                self.target_hidden = True
            return x, y, yaw
        msg.action = DummyObject.ADD if self.target_hidden or not self.object_seen else DummyObject.MODIFY
        self.object_pub.publish(msg)
        self.target_hidden = False
        return x, y, yaw

    def publish_secondary_obstacle(
        self,
        longitudinal: float,
        shoulder: str,
        anchor_xy: tuple[float, float],
        visible: bool = True,
    ) -> tuple[float, float] | None:
        """Publish a second object in the lane borrowed by the maneuver."""
        obs = self.cfg["obstacle"]
        anchor_s = project_to_reference(*anchor_xy, self.baseline_path)[0] if self.baseline_path else longitudinal
        # Right-positive route coordinates: an object on the right shoulder
        # requires the left lane, and vice versa.
        lateral = -float(obs["lane_width"]) if shoulder == "right" else float(obs["lane_width"])
        target = point_at_reference_s_lateral(self.baseline_path, anchor_s + longitudinal, lateral)
        if target is None:
            return None
        x, y, yaw = target
        is_new_object = self.secondary_obstacle_msg is None
        if self.secondary_obstacle_msg is None:
            self.secondary_obstacle_msg = make_dummy_object(
                self.cfg["frame_id"], self.get_clock().now().to_msg(), x, y,
                float(self.ego.get("z", 0.0)), yaw, length=float(obs["length"]),
                width=float(obs["width"]), height=float(obs["height"]), velocity=0.0,
            )
        msg = self.secondary_obstacle_msg
        msg.header.stamp = self.get_clock().now().to_msg()
        if not visible:
            if not self.secondary_hidden:
                msg.action = DummyObject.DELETE
                self.object_pub.publish(msg)
                self.secondary_hidden = True
            return x, y
        msg.action = DummyObject.ADD if is_new_object or self.secondary_hidden else DummyObject.MODIFY
        self.object_pub.publish(msg)
        self.secondary_hidden = False
        return x, y

    def target_visible(self, elapsed_sec: float, longitudinal: float, speed: float) -> bool:
        start = self.target_loss_start_sec
        duration = self.target_loss_duration_sec
        if start is None:
            if self.scenario == "target_loss_recovery":
                start, duration = 3.0, 1.0
            elif self.scenario == "target_loss_stop":
                start, duration = 3.0, float("inf")
            elif self.scenario == "target_passed_loss":
                start = max(8.0, longitudinal / max(abs(speed), 0.3) + 3.0)
                duration = float("inf")
        if start is None or duration is None:
            return True
        return not (start <= elapsed_sec < start + duration)

    def reset_samples(self) -> None:
        self.latest_candidate = []
        self.latest_trajectory = []
        self.latest_behavior = []
        self.candidate_history = []
        self.behavior_history = []
        self.trajectory_history = []
        self.trajectory_time_history = []
        self.actual = []
        self.actual_time_history = []
        self.speed_history = []
        self.latest_pose = (float(self.ego["x"]), float(self.ego["y"]))
        self.latest_speed = 0.0
        self.topic_seen = {}
        self.topic_last_monotonic = {}
        self.latest_gear_cmd = None
        self.latest_turn_cmd = None
        self.latest_gear_report = None
        self.latest_control_mode = None
        self.object_seen = False
        self.obstacle_msg = None
        self.secondary_obstacle_msg = None
        self.target_hidden = False
        self.secondary_hidden = False
        self.simple_shift = False
        self.obstacle_stop = False
        self.failure_reasons.clear()
        self.baseline_path = []
        self.obstacle_active = False
        self.first_target_lock = None
        self.first_feasible = None
        self.first_infeasible = None
        self.first_decision = None
        self.log_metrics = {}
        self.case_start_epoch = time.time()
        self.obstacle_s_center = None
        self.obstacle_l_center = None
        self.obstacle_yaw = None
        self.invalid_reason = None
        self.preflight_s_end_m = None
        self.preflight_speed_max_mps = None
        self.turn_signal_history = []

    def run_case(
        self, speed: float, longitudinal: float, intrusion: float, shoulder: str,
        bag_dir: Path | None = None,
    ) -> dict:
        test = self.cfg["test"]
        self.reset_samples()
        self.clear()
        # Re-publishing the initial pose three times after setting the route
        # creates new route UUIDs and can bounce the simulator between
        # WaitingForRoute and Driving.  A single topic publication is enough
        # because the API calls above are retried and are authoritative.
        # The initial-pose adaptor can appear before the simulator's direct
        # subscriber during a large launch.  Repeat the same manual pose
        # publication a few times before setting the route; never publish it
        # again after the route/mode hand-off, which would reset motion.
        self.publish_setup(repeat=5, engage=self.lock_mode == "steady_speed_lock")
        # Capture an obstacle-free route so map curvature is not mistaken for
        # an avoidance shift when metrics are expressed in s-l coordinates.
        # API calls return before the simulator has finished publishing its
        # initialized odometry and before behavior planning consumes the new
        # route. Wait for an actual non-empty behavior path instead of using a
        # fixed short sleep; otherwise a slow start is incorrectly marked
        # INVALID and no obstacle is ever tested.
        deadline = time.monotonic() + max(15.0, float(test["settle_sec"]) + 8.0)
        last_command = 0.0
        while rclpy.ok() and time.monotonic() < deadline and not self.latest_behavior:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.publish_speed_limit()
            self.publish_trailer_configuration()
            now = time.monotonic()
            if now - last_command >= 0.2:
                self.publish_drive_gear()
                self.publish_turn_indicator()
                last_command = now
            time.sleep(0.02)
        if self.latest_behavior:
            self.baseline_path = list(self.latest_behavior)
        self.latest_behavior = []
        self.behavior_history = []
        self.candidate_history = []
        self.trajectory_history = []
        setup_end = time.time()
        if not self.baseline_path:
            self.invalid_reason = "ROUTE_OR_BEHAVIOR_PATH_NOT_READY"

        obstacle_anchor = (float(self.ego["x"]), float(self.ego["y"]))
        if self.lock_mode == "steady_speed_lock":
            if not self.wait_for_stable_speed():
                self.invalid_reason = "STABLE_2MPS_NOT_REACHED"
            obstacle_anchor = self.latest_pose
        else:
            # The requested startup sequence is pose -> goal -> autonomous
            # mode.  Require a small amount of real simulator motion before
            # placing the target, so command-gate/startup failures cannot be
            # mistaken for a distance-infeasible avoidance case.
            if not self.wait_for_vehicle_motion():
                self.invalid_reason = "VEHICLE_NOT_MOVING_AFTER_MODE_SWITCH"
            obstacle_anchor = self.latest_pose
        # Snapshot motion before the target is introduced.  This separates a
        # command/startup failure from a later avoidance stop in the report.
        self.preflight_s_end_m = max(
            (
                project_to_reference(sample[0], sample[1], self.baseline_path)[0]
                if self.baseline_path
                else project(sample[0], sample[1], self.ego)[0]
                for sample in self.actual
            ),
            default=0.0,
        )
        self.preflight_speed_max_mps = max((abs(sample[2]) for sample in self.actual), default=0.0)

        self.obstacle_active = self.invalid_reason is None
        if self.obstacle_active:
            # For start_lock the obstacle is published before engagement.  For
            # steady_speed_lock it is placed relative to the live ego pose
            # after the measured speed has been stable for one second.
            ox, oy, obstacle_yaw = self.publish_obstacle(
                longitudinal, intrusion, shoulder, obstacle_anchor
            )
            if self.baseline_path:
                self.obstacle_s_center, self.obstacle_l_center = project_to_reference(
                    ox, oy, self.baseline_path
                )
            else:
                self.obstacle_s_center, self.obstacle_l_center = project(ox, oy, self.ego)
            self.obstacle_yaw = obstacle_yaw
            if self.scenario == "adjacent_lane_occupied":
                self.publish_secondary_obstacle(longitudinal + 3.0, shoulder, obstacle_anchor)
            if self.lock_mode == "start_lock" and not self.activate_vehicle():
                self.invalid_reason = "ENGAGE_FAILED"
            # Start scenario timing after the mode hand-off. This keeps the
            # loss/recovery windows deterministic even when service discovery
            # takes several seconds.
            self.case_start_epoch = time.time()
        else:
            ox, oy = obstacle_anchor
        self._case_obstacle_x = ox
        self._case_obstacle_y = oy
        bag_process = None
        bag_log_handle = None
        bag_returncode = None
        if bag_dir is not None:
            bag_dir.parent.mkdir(parents=True, exist_ok=True)
            # ros2 bag creates the output directory itself.  Avoid the
            # recorder's "output folder already exists" failure on reruns.
            requested_bag_dir = bag_dir
            suffix = 2
            while bag_dir.exists():
                bag_dir = requested_bag_dir.with_name(f"{requested_bag_dir.name}_{suffix}")
                suffix += 1
            topics = [
                self.cfg["topics"][name]
                for name in ("candidate_path", "behavior_path", "trajectory", "odometry",
                             "predicted_objects", "obstacle_stop_factors")
            ]
            topics.append(self.candidate_topic)
            topics.append(self.factors_topic)
            topics.extend([
                "/vehicle/status/trailer_configuration",
                "/planning/scenario_planning/max_velocity_default",
                "/planning/turn_indicators_cmd",
                "/control/command/turn_indicators_cmd",
                "/rosout",
                "/tf",
                "/tf_static",
            ])
            # Keep the recorder diagnostics beside the bag.  A recorder can
            # fail before creating metadata (for example when DDS discovery
            # is not ready); suppressing stderr made those cases look like a
            # successful test with no bag.
            bag_log_path = bag_dir.parent / f"{bag_dir.name}_record.log"
            bag_log_handle = bag_log_path.open("w", encoding="utf-8")
            bag_process = subprocess.Popen(
                ["ros2", "bag", "record", "-o", str(bag_dir), *topics],
                stdout=bag_log_handle, stderr=subprocess.STDOUT,
                env=os.environ.copy(), start_new_session=True,
            )
            time.sleep(1.0)
        deadline = time.monotonic() + float(test["settle_sec"]) + float(test["sample_sec"])
        period = 1.0 / float(test["publish_rate_hz"])
        while rclpy.ok() and time.monotonic() < deadline:
            if self.obstacle_active:
                elapsed = time.time() - self.case_start_epoch
                self.publish_obstacle(
                    longitudinal, intrusion, shoulder, obstacle_anchor,
                    visible=self.target_visible(elapsed, longitudinal, speed),
                )
                if self.scenario == "adjacent_lane_occupied":
                    self.publish_secondary_obstacle(longitudinal + 3.0, shoulder, obstacle_anchor)
                self.publish_drive_gear()
                self.publish_turn_indicator()
                self.publish_speed_limit()
                self.publish_trailer_configuration()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        if bag_process is not None:
            try:
                os.killpg(bag_process.pid, signal.SIGINT)
                bag_process.wait(timeout=10.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                bag_process.kill()
                bag_process.wait(timeout=3.0)
            finally:
                bag_returncode = bag_process.returncode
                if bag_log_handle is not None:
                    bag_log_handle.flush()
                    bag_log_handle.close()
        case_end = time.time()
        self.parse_launch_log(self.case_start_epoch, case_end)
        self._apply_log_metrics()
        metrics = self.metrics(speed, longitudinal, intrusion, shoulder, ox, oy)
        metrics["launch_log"] = str(self.launch_log) if self.launch_log else ""
        metrics["lock_mode"] = self.lock_mode
        metrics["requested_distance_m"] = longitudinal
        if bag_dir is not None:
            metrics["bag_path"] = str(bag_dir)
            metrics["bag_returncode"] = bag_returncode
            metrics["bag_metadata"] = (bag_dir / "metadata.yaml").exists()
        return metrics

    def metrics(self, speed: float, longitudinal: float, intrusion: float, shoulder: str, ox: float, oy: float) -> dict:
        def to_sl(x: float, y: float) -> tuple[float, float]:
            return project_to_reference(x, y, self.baseline_path) if self.baseline_path else project(x, y, self.ego)

        def max_lat(points: list[tuple[float, float]]) -> float:
            return max((abs(to_sl(x, y)[1]) for x, y in points), default=0.0)

        candidate_source = "path_candidate"
        candidate_paths = self.candidate_history
        if not candidate_paths:
            # The candidate publisher is empty for a module that is already
            # approved/running.  In the isolated test the behavior path is
            # therefore the module's effective calculated path.
            candidate_source = "behavior_path_fallback"
            candidate_paths = self.behavior_history
        candidate_max = max((max_lat(path) for path in candidate_paths), default=0.0)
        trajectory_max = max((max_lat(path) for path in self.trajectory_history), default=0.0)
        actual_sl = [to_sl(x, y) for x, y, _ in self.actual]
        actual_max = max((abs(y) for _, y in actual_sl), default=0.0)
        candidate_points = [point for path in candidate_paths for point in path]
        trajectory_points = [point for path in self.trajectory_history for point in path]
        trajectory_samples = [to_sl(x, y) for x, y in trajectory_points]
        actual_s = [s for s, _ in actual_sl]
        actual_speed_max = max((abs(v) for _, _, v in self.actual), default=0.0)
        lock_speed = self.log_metrics.get("first_target_lock_speed_mps")
        if lock_speed is None and self.first_target_lock:
            lock_speed = self.latest_speed
        # start_lock intentionally measures whatever speed the controller has
        # at first detection; steady_speed_lock requires the requested
        # 2 m/s band at the lock itself.
        speed_valid = (
            self.lock_mode == "start_lock" and actual_speed_max >= 0.2
        ) or (
            self.lock_mode == "steady_speed_lock"
            and lock_speed is not None
            and 1.8 <= float(lock_speed) <= 2.2
        )
        shifted_candidate_s = [s for s, y in (to_sl(x, y) for x, y in candidate_points) if abs(y) >= 0.1]
        shifted_actual_s = [s for s, y in actual_sl if abs(y) >= 0.1]
        tracking_errors = []
        if trajectory_samples:
            timed_trajectory_samples = [
                (stamp, [to_sl(*point) for point in path])
                for stamp, path in zip(self.trajectory_time_history, self.trajectory_history)
            ]
            for index, (_, y) in enumerate(actual_sl):
                if index < len(self.actual_time_history) and timed_trajectory_samples:
                    _, path_samples = min(
                        timed_trajectory_samples,
                        key=lambda item: abs(item[0] - self.actual_time_history[index]),
                    )
                    if path_samples:
                        _, nearest_y = min(path_samples, key=lambda item: abs(item[0] - actual_s[index]))
                        tracking_errors.append(abs(y - nearest_y))
        obs = self.cfg["obstacle"]
        clearances = []
        for ex, ey, _ in self.actual:
            # Conservative oriented-rectangle separation in the lane frame.
            # A negative value means the AGV and dummy object overlap on both
            # longitudinal/lateral axes; unlike a circle bound this does not
            # turn a clear diagonal pass into a false collision.
            dx, dy = ex - ox, ey - oy
            obstacle_yaw = self.obstacle_yaw
            if obstacle_yaw is None:
                obstacle_yaw = float(obs["lane_yaw"])
            c, si = math.cos(obstacle_yaw), math.sin(obstacle_yaw)
            along = abs(dx * c + dy * si) - (0.5 * (1.008 + 0.546 + 0.676) + 0.5 * float(obs["length"]))
            lateral = abs(-dx * si + dy * c) - (0.5 * 1.305 + 0.5 * float(obs["width"]))
            clearances.append(max(along, lateral))
        jerk = find_param(self.module_params, "shifting_lateral_jerk", 0.8)
        min_shift_speed = find_param(self.module_params, "min_shifting_speed", 1.2)
        theoretical_speed = max(abs(speed), min_shift_speed)
        theoretical_jerk_distance = 4.0 * (0.5 * max(candidate_max, 0.0) / max(jerk, 1.0e-6)) ** (1.0 / 3.0) * theoretical_speed
        theoretical_peak_ay = (0.5 * max(candidate_max, 0.0) * jerk * jerk) ** (1.0 / 3.0) if candidate_max > 0.0 else 0.0
        shift = candidate_max >= float(self.cfg["test"]["min_shift_m"]) and self.simple_shift
        min_clearance = min(clearances, default=0.0)
        clear = min_clearance >= float(self.cfg["test"].get("min_clearance_m", 0.0))
        obstacle_s = self.obstacle_s_center if self.obstacle_s_center is not None else longitudinal
        obstacle_passed = bool(actual_s) and max(actual_s) >= obstacle_s + 0.5 * float(obs["length"])
        passed_s = obstacle_s + 0.5 * float(obs["length"])
        return_completed = any(abs(y) <= 0.3 for s, y in actual_sl if s >= passed_s + 2.0)
        path_valid = bool(self.latest_trajectory) and all(
            math.isfinite(x) and math.isfinite(y)
            for path in self.trajectory_history for x, y in path
        )
        tracking_valid = (
            max(tracking_errors, default=0.0) <= float(self.cfg["test"].get("max_tracking_error_m", 0.8))
        )
        stop_samples = [s for s in actual_s if s <= obstacle_s]
        stop_front_clearance = (
            obstacle_s - 0.5 * float(obs["length"]) - 1.2 - max(stop_samples)
            if stop_samples and not obstacle_passed else None
        )
        expected_outbound = (
            TurnIndicatorsCommand.ENABLE_LEFT if shoulder == "right"
            else TurnIndicatorsCommand.ENABLE_RIGHT
        )
        expected_return = (
            TurnIndicatorsCommand.ENABLE_RIGHT if shoulder == "right"
            else TurnIndicatorsCommand.ENABLE_LEFT
        )
        outbound_signal_seen = any(command == expected_outbound for _, command in self.turn_signal_history)
        return_signal_seen = any(
            command == expected_return and stamp >= (self.turn_signal_history[0][0] if self.turn_signal_history else 0.0)
            for stamp, command in self.turn_signal_history
        )
        signal_off_after_case = bool(self.turn_signal_history) and self.turn_signal_history[-1][1] in (
            TurnIndicatorsCommand.DISABLE, TurnIndicatorsCommand.NO_COMMAND
        )

        if self.invalid_reason:
            result = "INVALID"
        elif self.mode == "simple_lc_avoidance" and not path_valid:
            result = "INVALID"
        elif not clear:
            result = "COLLISION"
        elif not self.first_target_lock:
            result = "INVALID"
        elif self.mode == "simple_lc_avoidance" and (
            self.obstacle_stop or self.first_decision == "INFEASIBLE_DISTANCE" or
            (not shift and not obstacle_passed)
        ):
            result = "SAFE_STOP"
        elif not shift:
            result = "PATH_GENERATION_FAILED"
        elif not obstacle_passed or not return_completed:
            result = "TRACKING_FAILURE"
        elif not speed_valid or not tracking_valid:
            result = "INVALID"
        else:
            result = "SUCCESS"
        result_reasons = set(self.failure_reasons)
        if not self.object_seen:
            result_reasons.add("OBJECT_NOT_SEEN")
        if not shift:
            result_reasons.add("NO_EFFECTIVE_SHIFT")
        if not clear:
            result_reasons.add("INSUFFICIENT_CLEARANCE")
        if not obstacle_passed:
            result_reasons.add("OBSTACLE_NOT_PASSED")
        if not speed_valid:
            result_reasons.add("SPEED_CLASS_INVALID")
        if self.invalid_reason:
            result_reasons.add(self.invalid_reason)
        if self.first_decision == "INFEASIBLE_DISTANCE":
            result_reasons.add("DISTANCE_INFEASIBLE")
        if not return_completed and obstacle_passed:
            result_reasons.add("RETURN_NOT_COMPLETED")
        if self.obstacle_stop:
            result_reasons.add("OBSTACLE_STOP_TAKEOVER")
        if not path_valid:
            result_reasons.add("EMPTY_OR_INVALID_TRAJECTORY")
        if not tracking_valid:
            result_reasons.add("TRACKING_ERROR_OVER_LIMIT")
        if stop_front_clearance is not None and stop_front_clearance < 1.0:
            result_reasons.add("STOP_MARGIN_UNDER_1M")
        distance_margin = None
        if self.first_feasible:
            distance_margin = self.first_feasible["dist_to_obstacle"] - self.first_feasible["dist_to_shift_end"]
        elif self.first_infeasible:
            distance_margin = self.first_infeasible["dist_to_obstacle"] - self.first_infeasible["dist_to_shift_end"]
        return {
            "mode": self.mode, "scenario": self.scenario, "acceptance_case": self.case_name, "speed_mps": speed, "longitudinal_m": longitudinal, "intrusion_m": intrusion,
            "shoulder": shoulder, "obstacle_x": round(ox, 4), "obstacle_y": round(oy, 4),
            "result": result, "object_seen": self.object_seen, "simple_shift": self.simple_shift,
            "candidate_source": candidate_source,
            "obstacle_stop": self.obstacle_stop, "candidate_max_lateral_m": round(candidate_max, 4),
            "obstacle_passed": obstacle_passed,
            "lock_mode": self.lock_mode,
            "requested_distance_m": longitudinal,
            "first_target_lock_lon_m": self.log_metrics.get("first_target_lock_lon_m") if self.log_metrics else (self.first_target_lock.get("lon") if self.first_target_lock else None),
            "first_target_lock_lat_m": self.log_metrics.get("first_target_lock_lat_m") if self.log_metrics else (self.first_target_lock.get("lat") if self.first_target_lock else None),
            "first_target_lock_time_s": self.log_metrics.get("first_target_lock_time_s") if self.log_metrics else (self.first_target_lock.get("time") if self.first_target_lock else None),
            "first_target_lock_speed_mps": lock_speed,
            "first_decision": self.first_decision,
            "distance_margin_m": round(distance_margin, 4) if distance_margin is not None else None,
            "first_feasible_lon_m": self.first_feasible.get("lon") if self.first_feasible else None,
            "first_feasible_ego_speed_mps": self.first_feasible.get("ego_speed") if self.first_feasible else None,
            "first_feasible_time_s": self.first_feasible.get("time") if self.first_feasible else None,
            "first_feasible_dist_to_shift_end_m": self.first_feasible.get("dist_to_shift_end") if self.first_feasible else None,
            "first_feasible_dist_to_obstacle_m": self.first_feasible.get("dist_to_obstacle") if self.first_feasible else None,
            "first_infeasible_lon_m": self.first_infeasible.get("lon") if self.first_infeasible else None,
            "first_infeasible_object_half_length_m": self.first_infeasible.get("object_half_length") if self.first_infeasible else None,
            "first_infeasible_ego_speed_mps": self.first_infeasible.get("ego_speed") if self.first_infeasible else None,
            "first_infeasible_time_s": self.first_infeasible.get("time") if self.first_infeasible else None,
            "first_infeasible_dist_to_shift_end_m": self.first_infeasible.get("dist_to_shift_end") if self.first_infeasible else None,
            "first_infeasible_dist_to_obstacle_m": self.first_infeasible.get("dist_to_obstacle") if self.first_infeasible else None,
            "first_infeasible_shortfall_m": self.first_infeasible.get("shortfall") if self.first_infeasible else None,
            "trajectory_max_lateral_m": round(trajectory_max, 4), "actual_max_lateral_m": round(actual_max, 4),
            "candidate_shift_span_m": round(max(shifted_candidate_s) - min(shifted_candidate_s), 4) if shifted_candidate_s else 0.0,
            "actual_shift_span_m": round(max(shifted_actual_s) - min(shifted_actual_s), 4) if shifted_actual_s else 0.0,
            "max_actual_to_final_lateral_error_m": round(max(tracking_errors, default=0.0), 4),
            "rms_actual_to_final_lateral_error_m": round(math.sqrt(sum(e * e for e in tracking_errors) / len(tracking_errors)), 4) if tracking_errors else 0.0,
            "min_approx_clearance_m": round(min_clearance, 4),
            "actual_s_start_m": round(min(actual_s), 4) if actual_s else None,
            "actual_s_end_m": round(max(actual_s), 4) if actual_s else None,
            "actual_speed_max_mps": round(actual_speed_max, 4),
            "preflight_s_end_m": round(self.preflight_s_end_m, 4) if self.preflight_s_end_m is not None else None,
            "preflight_speed_max_mps": round(self.preflight_speed_max_mps, 4) if self.preflight_speed_max_mps is not None else None,
            "return_completed": return_completed,
            "speed_valid": speed_valid,
            "path_valid": path_valid,
            "tracking_valid": tracking_valid,
            "stop_front_clearance_m": round(stop_front_clearance, 4) if stop_front_clearance is not None else None,
            "outbound_signal_seen": outbound_signal_seen,
            "return_signal_seen": return_signal_seen,
            "signal_off_after_case": signal_off_after_case,
            "speed_limit_mps": self.speed_limit_mps,
            "trailer_types": ";".join(self.trailer_types),
            "theoretical_jerk_distance_m": round(theoretical_jerk_distance, 4),
            "theoretical_peak_lateral_accel_mps2": round(theoretical_peak_ay, 4),
            "topic_localization_acceleration": self.topic_seen.get("localization_acceleration", False),
            "topic_trajectory_follower_control": self.topic_seen.get("trajectory_follower_control", False),
            "topic_gate_control": self.topic_seen.get("gate_control", False),
            "topic_gate_gear": self.topic_seen.get("gate_gear", False),
            "topic_gate_turn": self.topic_seen.get("gate_turn", False),
            "topic_gear_report": self.topic_seen.get("gear_report", False),
            "topic_control_mode": self.topic_seen.get("control_mode", False),
            "last_gate_gear": self.latest_gear_cmd,
            "last_gate_turn": self.latest_turn_cmd,
            "last_gear_report": self.latest_gear_report,
            "last_control_mode": self.latest_control_mode,
            "failure_reasons": ";".join(sorted(result_reasons)),
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=SCRIPT_DIR.parent / "config" / "simple_avoidance_distance.yaml")
    parser.add_argument("--output-dir", type=Path, default=Path("log") / time.strftime("%Y%m%d") / "simple_avoidance_distance_test")
    parser.add_argument("--speed", type=float, default=None)
    parser.add_argument("--distance", type=float, default=None)
    parser.add_argument("--intrusion", type=float, default=None)
    parser.add_argument("--shoulder", choices=("left", "right"), default=None)
    parser.add_argument("--mode", choices=("simple_avoidance", "simple_lc_avoidance"), default="simple_avoidance")
    parser.add_argument(
        "--lock-mode",
        choices=("start_lock", "steady_speed_lock"),
        default="start_lock",
        help="publish obstacle before engage, or after measured stable 2 m/s",
    )
    parser.add_argument("--sample-sec", type=float, default=None, help="override post-obstacle sampling duration")
    parser.add_argument("--settle-sec", type=float, default=None, help="override pre-obstacle settling duration")
    parser.add_argument("--launch-log", type=Path, default=None, help="launch.log used to parse first target lock")
    parser.add_argument(
        "--simple-avoidance-param",
        type=Path,
        default=Path("src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/autoware_behavior_path_simple_avoidance_module/simple_avoidance.param.yaml"),
    )
    parser.add_argument("--module-param", type=Path, default=None, help="override the selected module parameter file")
    parser.add_argument(
        "--scenario",
        choices=(
            "normal", "adjacent_lane_occupied", "target_loss_recovery", "target_loss_stop",
            "target_passed_loss",
        ),
        default="normal",
        help="LC acceptance scenario; normal preserves the distance-sweep behavior",
    )
    parser.add_argument("--speed-limit", type=float, default=None, help="publish max velocity input")
    parser.add_argument(
        "--trailer-types", default="", help="comma-separated trailer type names to publish"
    )
    parser.add_argument("--target-loss-start", type=float, default=None)
    parser.add_argument("--target-loss-duration", type=float, default=None)
    parser.add_argument("--case-name", default=None)
    parser.add_argument("--ego-x", type=float, default=None)
    parser.add_argument("--ego-y", type=float, default=None)
    parser.add_argument("--ego-yaw", type=float, default=None)
    parser.add_argument("--goal-x", type=float, default=None)
    parser.add_argument("--goal-y", type=float, default=None)
    parser.add_argument("--goal-yaw", type=float, default=None)
    parser.add_argument("--record-bag", action="store_true", help="record the measured topics for every case")
    args = parser.parse_args()
    cfg = load_yaml(args.config)
    for key, value in (
        ("x", args.ego_x),
        ("y", args.ego_y),
        ("yaw", args.ego_yaw),
    ):
        if value is not None:
            cfg["ego"][key] = value
    for key, value in (
        ("x", args.goal_x),
        ("y", args.goal_y),
        ("yaw", args.goal_yaw),
    ):
        if value is not None:
            cfg["goal"][key] = value
    if args.sample_sec is not None:
        cfg["test"]["sample_sec"] = args.sample_sec
    if args.settle_sec is not None:
        cfg["test"]["settle_sec"] = args.settle_sec
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/simple_avoidance_distance_test_roslog")
    rclpy.init()
    default_module_param = args.simple_avoidance_param
    if args.mode == "simple_lc_avoidance" and args.module_param is None:
        default_module_param = Path(
            "src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/"
            "lane_driving/behavior_planning/behavior_path_planner/simple_lc_avoidance/"
            "simple_lc_avoidance.param.yaml"
        )
    simple_params = load_yaml(args.module_param or default_module_param)
    trailer_types = [item for item in args.trailer_types.split(",") if item]
    node = DistanceNode(
        cfg, args.mode, simple_params, args.lock_mode, args.launch_log, args.scenario,
        args.speed_limit, trailer_types, args.target_loss_start, args.target_loss_duration,
        args.case_name,
    )
    speeds = [args.speed] if args.speed is not None else cfg["test"]["speeds_mps"]
    distances = [args.distance] if args.distance is not None else cfg["test"]["longitudinal_distances_m"]
    intrusions = [args.intrusion] if args.intrusion is not None else cfg["test"]["intrusions_m"]
    shoulders = [args.shoulder] if args.shoulder else cfg["test"]["shoulders"]
    rows = []
    try:
        for speed in speeds:
            for shoulder in shoulders:
                for intrusion in intrusions:
                    for distance in distances:
                        node.get_logger().info(f"case speed={speed} distance={distance} intrusion={intrusion} shoulder={shoulder}")
                        bag_dir = None
                        if args.record_bag:
                            case_name = f"v{speed:g}_d{distance:g}_i{intrusion:g}_{shoulder}"
                            bag_dir = args.output_dir / f"scenario_{case_name}" / "rosbag2"
                        rows.append(node.run_case(float(speed), float(distance), float(intrusion), str(shoulder), bag_dir))
    finally:
        try:
            node.clear()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "distance_sweep.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["result"])
        writer.writeheader()
        writer.writerows(rows)
    trajectory_path = args.output_dir / "trajectory_metrics.csv"
    trajectory_fields = [
        "mode", "scenario", "acceptance_case", "lock_mode", "speed_mps", "speed_limit_mps", "trailer_types", "requested_distance_m", "longitudinal_m", "intrusion_m", "shoulder", "result", "candidate_source", "obstacle_passed", "return_completed",
        "first_target_lock_lon_m", "first_target_lock_lat_m", "first_target_lock_time_s",
        "first_target_lock_speed_mps", "first_decision", "distance_margin_m",
        "first_feasible_lon_m", "first_feasible_ego_speed_mps", "first_feasible_time_s",
        "first_feasible_dist_to_shift_end_m", "first_feasible_dist_to_obstacle_m",
        "first_infeasible_lon_m", "first_infeasible_object_half_length_m",
        "first_infeasible_ego_speed_mps", "first_infeasible_time_s",
        "first_infeasible_dist_to_shift_end_m", "first_infeasible_dist_to_obstacle_m",
        "first_infeasible_shortfall_m",
        "candidate_max_lateral_m", "trajectory_max_lateral_m", "actual_max_lateral_m",
        "candidate_shift_span_m", "actual_shift_span_m",
        "max_actual_to_final_lateral_error_m", "rms_actual_to_final_lateral_error_m",
        "min_approx_clearance_m", "actual_speed_max_mps", "preflight_s_end_m",
        "preflight_speed_max_mps", "speed_valid", "path_valid", "tracking_valid",
        "stop_front_clearance_m", "outbound_signal_seen", "return_signal_seen",
        "signal_off_after_case",
    ]
    with trajectory_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=trajectory_fields)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in trajectory_fields} for row in rows)
    success = [r for r in rows if r["result"] == "SUCCESS"]
    report = args.output_dir / "README.md"
    mode_label = "Simple Avoidance" if args.mode == "simple_avoidance" else "Simple Lane Change Avoidance"
    with report.open("w", encoding="utf-8") as f:
        f.write(f"# {mode_label} distance test\n\n")
        f.write(f"cases: {len(rows)}\n\n")
        f.write(
            "The primary runtime distance is `first_target_lock_lon_m`: the signed "
            "arc length from the ego reference pose to the target center when the "
            "module first logs `active target locked`. `longitudinal_m` is only the "
            "dummy obstacle placement label. `dist_to_obstacle` is the target-center "
            "distance after subtracting object half-length and lateral margin.\n\n"
        )
        locked = [r for r in rows if r.get("first_target_lock_lon_m") is not None]
        infeasible_locked = [
            r for r in rows if r.get("first_infeasible_lon_m") is not None
        ]
        f.write(
            "first-lock runtime distance range: "
            f"{min((r['first_target_lock_lon_m'] for r in locked), default='N/A')}"
            ".."
            f"{max((r['first_target_lock_lon_m'] for r in locked), default='N/A')} m\n\n"
        )
        f.write(
            "first-lock samples with INSUFFICIENT_DISTANCE: "
            f"{len(infeasible_locked)}\n\n"
        )
        f.write(f"minimum successful placement distance: {min((r['longitudinal_m'] for r in success), default='N/A')} m\n\n")
        f.write(f"maximum successful placement distance: {max((r['longitudinal_m'] for r in success), default='N/A')} m\n\n")
        not_triggered = [r for r in rows if not r["simple_shift"]]
        failed = [r for r in rows if r["result"] == "FAIL"]
        f.write(f"minimum failed placement distance: {min((r['longitudinal_m'] for r in failed), default='N/A')} m\n\n")
        f.write(f"maximum non-triggered placement distance: {max((r['longitudinal_m'] for r in not_triggered), default='N/A')} m\n\n")
        f.write("`OBSTACLE_STOP_TAKEOVER` is excluded from pure module success.\n")
        f.write("`simple_shift=false` means the selected module did not produce an effective lateral shift; it is the trigger boundary, not a collision result.\n")
        f.write("`candidate_source=behavior_path_fallback` is expected when the active module clears its candidate topic after approval; the behavior path is then used as the effective module path.\n")
        f.write("`speed_valid=false` means the simulator did not actually reach the requested speed class.\n")
        f.write("\n## Per speed/side/intrusion summary\n\n")
        f.write("| lock mode | speed label | side | intrusion | min successful first lock | max infeasible first lock | transition | stop takeover |\n")
        f.write("|:---|---:|:---:|---:|---:|---:|:---:|---:|\n")
        groups = sorted({(r.get("lock_mode", "start_lock"), r["speed_mps"], r["shoulder"], r["intrusion_m"]) for r in rows})
        for lock_mode, speed, shoulder, intrusion in groups:
            group = [r for r in rows if (r.get("lock_mode", "start_lock"), r["speed_mps"], r["shoulder"], r["intrusion_m"]) == (lock_mode, speed, shoulder, intrusion)]
            group_success = [r for r in group if r["result"] == "SUCCESS"]
            group_infeasible = [r for r in group if r.get("first_decision") == "INFEASIBLE_DISTANCE"]
            stops = sum(1 for r in group if r["result"] == "OBSTACLE_STOP_TAKEOVER")
            min_success_lock = min((r["first_target_lock_lon_m"] for r in group_success if r.get("first_target_lock_lon_m") is not None), default="N/A")
            max_infeasible_lock = max((r["first_target_lock_lon_m"] for r in group_infeasible if r.get("first_target_lock_lon_m") is not None), default="N/A")
            transition = f"{max_infeasible_lock}..{min_success_lock}" if max_infeasible_lock != "N/A" and min_success_lock != "N/A" else "N/A"
            f.write(f"| {lock_mode} | {speed} | {shoulder} | {intrusion} | {min_success_lock} | {max_infeasible_lock} | {transition} | {stops} |\n")
    print(f"wrote {csv_path} and {report}")


if __name__ == "__main__":
    main()
