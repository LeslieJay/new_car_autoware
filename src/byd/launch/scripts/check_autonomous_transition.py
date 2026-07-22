#!/usr/bin/env python3
"""Continuously diagnose failed autonomous-mode transitions without changing state."""

import argparse
from dataclasses import dataclass
import sys
import time
from typing import Dict, List, Optional


OPERATION_MODE_AUTONOMOUS = 2
VEHICLE_MODE_AUTONOMOUS = 1
SPEED_EPSILON = 0.01
STALE_AFTER_SEC = 2.0
SERVICE_GRACE_SEC = 5.0

REQUIRED_SERVICES = (
    "/system/operation_mode/change_operation_mode",
    "/system/operation_mode/change_autoware_control",
    "/control/control_mode_request",
)


@dataclass(frozen=True)
class Snapshot:
    operation_mode: Optional[int]
    autoware_control_enabled: Optional[bool]
    vehicle_control_mode: Optional[int]
    trajectory_speed: Optional[float]
    follower_speed: Optional[float]
    gated_speed: Optional[float]
    service_available: Dict[str, bool]


def diagnose(snapshot: Snapshot) -> List[str]:
    reasons = []
    for service in REQUIRED_SERVICES:
        if snapshot.service_available.get(service) is False:
            reasons.append(f"{service} is unavailable")

    if snapshot.operation_mode is None:
        reasons.append("operation mode state is unavailable or stale")
    elif snapshot.operation_mode != OPERATION_MODE_AUTONOMOUS:
        reasons.append("operation mode did not become AUTONOMOUS")

    if snapshot.autoware_control_enabled is None:
        reasons.append("Autoware control state is unavailable or stale")
    elif not snapshot.autoware_control_enabled:
        reasons.append("Autoware control is not enabled")

    if snapshot.vehicle_control_mode is None:
        reasons.append("vehicle control mode is unavailable or stale")
    elif snapshot.vehicle_control_mode != VEHICLE_MODE_AUTONOMOUS:
        reasons.append("vehicle control mode is not AUTONOMOUS")

    if snapshot.trajectory_speed is None:
        reasons.append("planning trajectory is unavailable or stale")
    elif snapshot.trajectory_speed <= SPEED_EPSILON:
        reasons.append("planning trajectory has no nonzero target speed")

    if snapshot.follower_speed is None:
        reasons.append("trajectory follower command is unavailable or stale")
    elif (
        snapshot.trajectory_speed is not None
        and snapshot.trajectory_speed > SPEED_EPSILON
        and abs(snapshot.follower_speed) <= SPEED_EPSILON
    ):
        reasons.append("follower command remains zero despite a nonzero trajectory")

    if snapshot.gated_speed is None:
        reasons.append("final gated command is unavailable or stale")
    elif snapshot.follower_speed is not None and abs(snapshot.follower_speed) > SPEED_EPSILON:
        if abs(snapshot.gated_speed) <= SPEED_EPSILON:
            reasons.append("final gated command is zero despite a nonzero follower command")
        elif abs(snapshot.gated_speed) < 0.5 * abs(snapshot.follower_speed):
            reasons.append("final gated command is less than half the follower command")

    return reasons


class TransitionTracker:
    def __init__(self, timeout_sec: float):
        self.timeout_sec = timeout_sec
        self.state = "idle"
        self.started_at = 0.0
        self.previous_mode = None
        self.previous_in_transition = False

    def observe(
        self, mode: int, enabled: bool, in_transition: bool, now: float
    ) -> Optional[str]:
        transition_edge = in_transition and not self.previous_in_transition
        autonomous_edge = (
            self.previous_mode is not None
            and self.previous_mode != OPERATION_MODE_AUTONOMOUS
            and mode == OPERATION_MODE_AUTONOMOUS
            and not enabled
        )
        self.previous_mode = mode
        self.previous_in_transition = in_transition

        if transition_edge or autonomous_edge:
            self.state = "running"
            self.started_at = now
            return "start"

        if self.state == "active" and mode != OPERATION_MODE_AUTONOMOUS and not in_transition:
            self.state = "idle"
        return None

    def tick(self, success: bool, now: float) -> Optional[str]:
        if self.state == "running":
            if success:
                self.state = "active"
                return "success"
            if now - self.started_at >= self.timeout_sec:
                self.state = "failed"
                return "failure"
        elif self.state == "failed" and success:
            self.state = "active"
            return "recovered"
        return None


def parse_cli(argv=None):
    parser = argparse.ArgumentParser(
        description="Continuously diagnose autonomous-mode transition failures"
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to wait before reporting a failed transition (default: 10)",
    )
    args, ros_args = parser.parse_known_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be greater than zero")
    return args, ros_args


def create_ros_node_class():
    from autoware_adapi_v1_msgs.msg import OperationModeState
    from autoware_control_msgs.msg import Control
    from autoware_planning_msgs.msg import Trajectory
    from autoware_vehicle_msgs.msg import ControlModeReport
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class AutonomousTransitionChecker(Node):
        def __init__(self, timeout_sec: float):
            super().__init__("autonomous_transition_checker")
            self.timeout_sec = timeout_sec
            self.tracker = TransitionTracker(timeout_sec)
            self.started_at = time.monotonic()
            self.samples = {}
            self.service_available = {}
            self.reported_service_available = {}

            state_qos = QoSProfile(depth=1)
            state_qos.reliability = ReliabilityPolicy.RELIABLE
            state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            default_qos = QoSProfile(depth=1)

            self.subscriptions = [
                self.create_subscription(
                    OperationModeState,
                    "/api/operation_mode/state",
                    self._on_operation_mode,
                    state_qos,
                ),
                self.create_subscription(
                    ControlModeReport,
                    "/vehicle/status/control_mode",
                    self._on_vehicle_control_mode,
                    default_qos,
                ),
                self.create_subscription(
                    Trajectory,
                    "/planning/trajectory",
                    self._on_trajectory,
                    default_qos,
                ),
                self.create_subscription(
                    Control,
                    "/control/trajectory_follower/control_cmd",
                    self._on_follower_command,
                    default_qos,
                ),
                self.create_subscription(
                    Control,
                    "/control/command/control_cmd",
                    self._on_gated_command,
                    default_qos,
                ),
            ]
            self.create_timer(0.5, self._check_transition)
            self.create_timer(1.0, self._check_services)
            print(
                f"[AUTO CHECK][READY] passive monitor started; timeout={timeout_sec:.1f}s",
                flush=True,
            )

        def _record(self, key: str, value, now: Optional[float] = None):
            self.samples[key] = (value, time.monotonic() if now is None else now)

        def _fresh(self, key: str, now: float):
            sample = self.samples.get(key)
            if sample is None or now - sample[1] > STALE_AFTER_SEC:
                return None
            return sample[0]

        def _age(self, key: str, now: float) -> str:
            sample = self.samples.get(key)
            return "no messages" if sample is None else f"age={now - sample[1]:.1f}s"

        def _on_operation_mode(self, msg):
            now = time.monotonic()
            self._record("operation_mode", msg.mode, now)
            self._record("autoware_control_enabled", msg.is_autoware_control_enabled, now)
            self._record("is_in_transition", msg.is_in_transition, now)
            event = self.tracker.observe(
                msg.mode, msg.is_autoware_control_enabled, msg.is_in_transition, now
            )
            if event == "start":
                print("[AUTO CHECK][START] autonomous transition detected", flush=True)

        def _on_vehicle_control_mode(self, msg):
            self._record("vehicle_control_mode", msg.mode)

        def _on_trajectory(self, msg):
            speed = max(
                (abs(point.longitudinal_velocity_mps) for point in msg.points), default=0.0
            )
            self._record("trajectory_speed", speed)

        def _on_follower_command(self, msg):
            self._record("follower_speed", msg.longitudinal.velocity)

        def _on_gated_command(self, msg):
            self._record("gated_speed", msg.longitudinal.velocity)

        def _snapshot(self, now: float) -> Snapshot:
            return Snapshot(
                operation_mode=self._fresh("operation_mode", now),
                autoware_control_enabled=self._fresh("autoware_control_enabled", now),
                vehicle_control_mode=self._fresh("vehicle_control_mode", now),
                trajectory_speed=self._fresh("trajectory_speed", now),
                follower_speed=self._fresh("follower_speed", now),
                gated_speed=self._fresh("gated_speed", now),
                service_available=dict(self.service_available),
            )

        def _check_transition(self):
            now = time.monotonic()
            snapshot = self._snapshot(now)
            success = (
                snapshot.operation_mode == OPERATION_MODE_AUTONOMOUS
                and snapshot.autoware_control_enabled is True
                and snapshot.vehicle_control_mode == VEHICLE_MODE_AUTONOMOUS
            )
            event = self.tracker.tick(success, now)
            if event == "success":
                print("[AUTO CHECK][SUCCESS] autonomous control is active", flush=True)
            elif event == "failure":
                self._print_failure(snapshot, now)
            elif event == "recovered":
                print("[AUTO CHECK][RECOVERED] autonomous control is active", flush=True)

        def _check_services(self):
            names = {name for name, _ in self.get_service_names_and_types()}
            now = time.monotonic()
            for service in REQUIRED_SERVICES:
                available = service in names
                self.service_available[service] = available
                if now - self.started_at < SERVICE_GRACE_SEC:
                    continue
                previous = self.reported_service_available.get(service)
                if previous == available:
                    continue
                self.reported_service_available[service] = available
                level = "READY" if available else "WARN"
                state = "available" if available else "unavailable"
                print(f"[AUTO CHECK][{level}] {service} is {state}", flush=True)

        @staticmethod
        def _format_speed(value: Optional[float]) -> str:
            return "unavailable/stale" if value is None else f"{value:.2f} m/s"

        def _print_failure(self, snapshot: Snapshot, now: float):
            operation_names = {0: "UNKNOWN", 1: "STOP", 2: "AUTONOMOUS", 3: "LOCAL", 4: "REMOTE"}
            vehicle_names = {
                0: "NO_COMMAND",
                1: "AUTONOMOUS",
                2: "AUTONOMOUS_STEER_ONLY",
                3: "AUTONOMOUS_VELOCITY_ONLY",
                4: "MANUAL",
                5: "DISENGAGED",
                6: "NOT_READY",
            }
            operation = operation_names.get(
                snapshot.operation_mode, "unavailable/stale"
            )
            vehicle = vehicle_names.get(
                snapshot.vehicle_control_mode, "unavailable/stale"
            )
            enabled = (
                "unavailable/stale"
                if snapshot.autoware_control_enabled is None
                else "enabled" if snapshot.autoware_control_enabled else "disabled"
            )
            print(
                f"[AUTO CHECK][FAIL] transition timeout: {self.timeout_sec:.1f}s",
                flush=True,
            )
            print(
                f"  operation_mode: {operation} ({self._age('operation_mode', now)})"
            )
            print(
                f"  autoware_control: {enabled} "
                f"({self._age('autoware_control_enabled', now)})"
            )
            print(
                f"  vehicle_control_mode: {vehicle} "
                f"({self._age('vehicle_control_mode', now)})"
            )
            print(f"  trajectory_target_speed: {self._format_speed(snapshot.trajectory_speed)}")
            print(f"  follower_command_speed: {self._format_speed(snapshot.follower_speed)}")
            print(f"  gated_command_speed: {self._format_speed(snapshot.gated_speed)}")
            reasons = diagnose(snapshot)
            if not reasons:
                reasons = ["transition success conditions were not met"]
            for priority, reason in enumerate(reasons, start=1):
                print(f"  [P{priority}] {reason}")
            sys.stdout.flush()

    return AutonomousTransitionChecker


def main(argv=None) -> int:
    args, ros_args = parse_cli(argv)
    try:
        import rclpy
    except ModuleNotFoundError as error:
        print(f"ROS 2 Python dependency is unavailable: {error}", file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = create_ros_node_class()(args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
