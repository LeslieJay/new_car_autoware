#!/usr/bin/env python3
"""Test lateral steering by publishing directly to the standalone CAN node.

This script does not use vehicle_cmd_gate and does not require Autoware nodes.
It publishes continuously to /control/command/control_cmd and
/control/command/gear_cmd, which are subscribed by can_node.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, SteeringReport, VelocityReport
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


CONTROL_TOPIC = "/control/command/control_cmd"
GEAR_TOPIC = "/control/command/gear_cmd"
STEERING_TOPIC = "/vehicle/status/steering_status"
VELOCITY_TOPIC = "/vehicle/status/velocity_status"
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
CONTROL_PERIOD_S = 0.02
FEEDBACK_MAX_AGE_S = 0.5
STOP_HOLD_S = 3.0


@dataclass(frozen=True)
class Sample:
    elapsed_s: float
    phase: str
    command_angle_rad: float
    command_speed_mps: float
    measured_angle_rad: float
    measured_speed_mps: float
    heading_rate_radps: float


@dataclass(frozen=True)
class PhaseResult:
    phase: str
    command_angle_rad: float
    sample_count: int
    median_angle_rad: float | None
    median_error_rad: float | None
    maximum_error_rad: float | None
    median_speed_mps: float | None
    median_heading_rate_radps: float | None
    passed: bool
    reason: str


class DirectCanLateralTest(Node):
    def __init__(self, sample_writer: csv.writer, sample_file) -> None:
        super().__init__("direct_can_lateral_test")
        self._lock = threading.Lock()
        self._started_at = time.monotonic()
        self._phase = "initialising"
        self._target_angle = 0.0
        self._target_speed = 0.0
        self._target_acceleration = -0.5
        self._publishing_enabled = False
        self._steering_angle = math.nan
        self._steering_time = 0.0
        self._speed = math.nan
        self._heading_rate = math.nan
        self._velocity_time = 0.0
        self._phase_samples: list[Sample] = []
        self._sample_writer = sample_writer
        self._sample_file = sample_file

        self._control_pub = self.create_publisher(Control, CONTROL_TOPIC, 10)
        self._gear_pub = self.create_publisher(GearCommand, GEAR_TOPIC, 10)
        self.create_subscription(SteeringReport, STEERING_TOPIC, self._on_steering, 50)
        self.create_subscription(VelocityReport, VELOCITY_TOPIC, self._on_velocity, 50)
        self.create_timer(CONTROL_PERIOD_S, self._publish_control)

    def _on_steering(self, msg: SteeringReport) -> None:
        now = time.monotonic()
        with self._lock:
            self._steering_angle = float(msg.steering_tire_angle)
            self._steering_time = now
            sample = Sample(
                elapsed_s=now - self._started_at,
                phase=self._phase,
                command_angle_rad=self._target_angle,
                command_speed_mps=self._target_speed,
                measured_angle_rad=self._steering_angle,
                measured_speed_mps=self._speed,
                heading_rate_radps=self._heading_rate,
            )
            self._phase_samples.append(sample)
            self._sample_writer.writerow(asdict(sample).values())
            self._sample_file.flush()

    def _on_velocity(self, msg: VelocityReport) -> None:
        with self._lock:
            self._speed = float(msg.longitudinal_velocity)
            self._heading_rate = float(msg.heading_rate)
            self._velocity_time = time.monotonic()

    def _publish_control(self) -> None:
        with self._lock:
            if not self._publishing_enabled:
                return
            angle = self._target_angle
            speed = self._target_speed
            acceleration = self._target_acceleration
        msg = Control()
        msg.stamp = self.get_clock().now().to_msg()
        msg.lateral.steering_tire_angle = angle
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.lateral.is_defined_steering_tire_rotation_rate = False
        msg.longitudinal.velocity = speed
        msg.longitudinal.acceleration = acceleration
        msg.longitudinal.jerk = 0.0
        msg.longitudinal.is_defined_acceleration = True
        msg.longitudinal.is_defined_jerk = False
        self._control_pub.publish(msg)

    def enable_publishing(self) -> None:
        with self._lock:
            self._publishing_enabled = True

    def publishing_enabled(self) -> bool:
        with self._lock:
            return self._publishing_enabled

    def set_command(
        self, phase: str, angle: float, speed: float, acceleration: float
    ) -> None:
        with self._lock:
            self._phase = phase
            self._target_angle = angle
            self._target_speed = speed
            self._target_acceleration = acceleration
            self._phase_samples = []

    def publish_drive_gear(self) -> None:
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = GearCommand.DRIVE
        for _ in range(3):
            self._gear_pub.publish(msg)
            time.sleep(0.1)

    def feedback_ages(self) -> tuple[float, float]:
        with self._lock:
            now = time.monotonic()
            return now - self._steering_time, now - self._velocity_time

    def latest_speed(self) -> float:
        with self._lock:
            return self._speed

    def phase_samples(self) -> list[Sample]:
        with self._lock:
            return list(self._phase_samples)

    def other_control_publishers(self) -> list[str]:
        return sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in self.get_publishers_info_by_topic(CONTROL_TOPIC)
            if info.node_name != self.get_name()
        )

    def control_subscribers(self) -> list[str]:
        return sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in self.get_subscriptions_info_by_topic(CONTROL_TOPIC)
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Direct low-speed lateral steering test for the standalone CAN node."
    )
    parser.add_argument(
        "--arm", action="store_true",
        help="required confirmation that the closed test area and physical E-stop are ready",
    )
    parser.add_argument("--speed", type=float, default=0.15, metavar="MPS")
    parser.add_argument("--steering-angle", type=float, default=0.08, metavar="RAD")
    parser.add_argument("--segment-seconds", type=float, default=3.0, metavar="SEC")
    parser.add_argument("--cycles", type=int, default=1, metavar="N")
    parser.add_argument("--acceleration", type=float, default=0.2, metavar="MPS2")
    parser.add_argument("--deceleration", type=float, default=-0.5, metavar="MPS2")
    parser.add_argument("--tracking-tolerance", type=float, default=0.03, metavar="RAD")
    parser.add_argument("--evaluation-window", type=float, default=1.0, metavar="SEC")
    parser.add_argument("--feedback-timeout", type=float, default=5.0, metavar="SEC")
    parser.add_argument("--motion-timeout", type=float, default=8.0, metavar="SEC")
    parser.add_argument("--min-measured-speed", type=float, default=0.03, metavar="MPS")
    parser.add_argument("--output-dir", type=Path, default=None, metavar="DIR")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.arm:
        raise ValueError(
            "use --arm only after clearing the test area and checking the physical E-stop"
        )
    if not 0.0 <= args.speed <= 0.5:
        raise ValueError("--speed must be in [0, 0.5] m/s")
    if not 0.01 <= args.steering_angle <= 0.20:
        raise ValueError("--steering-angle must be in [0.01, 0.20] rad")
    if not 0.5 <= args.segment_seconds <= 15.0:
        raise ValueError("--segment-seconds must be in [0.5, 15] s")
    if not 1 <= args.cycles <= 5:
        raise ValueError("--cycles must be in [1, 5]")
    if not 0.01 <= args.acceleration <= 1.0:
        raise ValueError("--acceleration must be in [0.01, 1.0] m/s^2")
    if not -2.0 <= args.deceleration <= -0.01:
        raise ValueError("--deceleration must be in [-2.0, -0.01] m/s^2")
    if not 0.005 <= args.tracking_tolerance <= 0.10:
        raise ValueError("--tracking-tolerance must be in [0.005, 0.10] rad")
    if not 0.2 <= args.evaluation_window <= args.segment_seconds:
        raise ValueError("--evaluation-window must be in [0.2, --segment-seconds] s")
    if not 0.5 <= args.feedback_timeout <= 30.0:
        raise ValueError("--feedback-timeout must be in [0.5, 30] s")
    if not 1.0 <= args.motion_timeout <= 30.0:
        raise ValueError("--motion-timeout must be in [1, 30] s")
    if not 0.0 <= args.min_measured_speed <= args.speed:
        raise ValueError("--min-measured-speed must be in [0, --speed]")


def wait_until(predicate, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}")


def finite_median(values: list[float]) -> float | None:
    values = [value for value in values if math.isfinite(value)]
    return float(statistics.median(values)) if values else None


def evaluate_phase(
    phase: str, command_angle: float, samples: list[Sample],
    evaluation_window_s: float, tolerance_rad: float,
) -> PhaseResult:
    if not samples:
        return PhaseResult(
            phase, command_angle, 0, None, None, None, None, None,
            False, "no steering feedback",
        )
    last_time = samples[-1].elapsed_s
    samples = [
        sample for sample in samples
        if sample.elapsed_s >= last_time - evaluation_window_s
    ]
    angles = [sample.measured_angle_rad for sample in samples]
    errors = [abs(angle - command_angle) for angle in angles]
    median_angle = finite_median(angles)
    median_error = finite_median(errors)
    passed = median_error is not None and median_error <= tolerance_rad
    if median_error is None:
        reason = "no finite steering feedback"
    elif passed:
        reason = f"median error {median_error:.4f} rad <= {tolerance_rad:.4f} rad"
    else:
        reason = f"median error {median_error:.4f} rad > {tolerance_rad:.4f} rad"
    return PhaseResult(
        phase=phase,
        command_angle_rad=command_angle,
        sample_count=len(samples),
        median_angle_rad=median_angle,
        median_error_rad=median_error,
        maximum_error_rad=max(errors) if errors else None,
        median_speed_mps=finite_median(
            [sample.measured_speed_mps for sample in samples]
        ),
        median_heading_rate_radps=finite_median(
            [sample.heading_rate_radps for sample in samples]
        ),
        passed=passed,
        reason=reason,
    )


def run_phase(
    node: DirectCanLateralTest, phase: str, angle: float, args: argparse.Namespace
) -> PhaseResult:
    node.set_command(phase, angle, args.speed, args.acceleration)
    print(
        f"[phase] {phase}: angle={angle:+.3f} rad, "
        f"speed={args.speed:.2f} m/s, duration={args.segment_seconds:.1f} s"
    )
    time.sleep(args.segment_seconds)
    steering_age, velocity_age = node.feedback_ages()
    if steering_age > FEEDBACK_MAX_AGE_S or velocity_age > FEEDBACK_MAX_AGE_S:
        raise RuntimeError(
            "feedback became stale "
            f"(steering={steering_age:.2f}s, velocity={velocity_age:.2f}s)"
        )
    result = evaluate_phase(
        phase, angle, node.phase_samples(),
        args.evaluation_window, args.tracking_tolerance,
    )
    print(f"[result] {phase}: {'PASS' if result.passed else 'FAIL'}; {result.reason}")
    return result


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or WORKSPACE_ROOT / "log/direct_can_lateral_test" / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    results: list[PhaseResult] = []
    exit_code = 1
    node: DirectCanLateralTest | None = None
    executor: MultiThreadedExecutor | None = None
    spin_thread: threading.Thread | None = None

    with (output_dir / "feedback.csv").open("w", newline="", encoding="utf-8") as sample_file, \
         (output_dir / "phase_summary.csv").open("w", newline="", encoding="utf-8") as result_file:
        sample_writer = csv.writer(sample_file)
        sample_writer.writerow(Sample.__dataclass_fields__.keys())
        result_writer = csv.writer(result_file)
        result_writer.writerow(PhaseResult.__dataclass_fields__.keys())

        try:
            rclpy.init()
            node = DirectCanLateralTest(sample_writer, sample_file)
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            time.sleep(0.8)

            other_publishers = node.other_control_publishers()
            if other_publishers:
                raise RuntimeError(
                    f"another publisher is active on {CONTROL_TOPIC}: "
                    f"{', '.join(other_publishers)}"
                )
            subscribers = node.control_subscribers()
            if "/can_node" not in subscribers:
                raise RuntimeError(
                    f"/can_node is not subscribed to {CONTROL_TOPIC}; "
                    f"observed subscribers: {subscribers or ['none']}"
                )

            print(f"[route] direct to /can_node via {CONTROL_TOPIC}")
            print(f"[record] {output_dir}")
            wait_until(
                lambda: max(node.feedback_ages()) <= FEEDBACK_MAX_AGE_S,
                args.feedback_timeout,
                "fresh steering and velocity feedback from the CAN node",
            )
            node.set_command("warmup", 0.0, args.speed, args.acceleration)
            node.publish_drive_gear()
            node.enable_publishing()
            if args.speed > 0.0:
                wait_until(
                    lambda: abs(node.latest_speed()) >= args.min_measured_speed,
                    args.motion_timeout,
                    "vehicle motion; check chassis automatic-control enable and DRIVE gear",
                )

            for cycle in range(1, args.cycles + 1):
                for suffix, angle in (
                    ("left", args.steering_angle),
                    ("centre_after_left", 0.0),
                    ("right", -args.steering_angle),
                    ("centre_after_right", 0.0),
                ):
                    result = run_phase(node, f"cycle_{cycle}_{suffix}", angle, args)
                    results.append(result)
                    result_writer.writerow(asdict(result).values())
                    result_file.flush()
                    if not result.passed:
                        raise RuntimeError(
                            f"steering tracking failed in {result.phase}: {result.reason}"
                        )
            exit_code = 0
            print("[completed] all direct-CAN steering phases passed")
        except KeyboardInterrupt:
            print("\n[interrupted] operator requested stop", file=sys.stderr)
            exit_code = 130
        except Exception as error:
            print(f"ERROR: {error}", file=sys.stderr)
            exit_code = 1
        finally:
            if node is not None and node.publishing_enabled():
                node.set_command("safety_stop", 0.0, 0.0, args.deceleration)
                print(f"[safety] publishing zero speed and centre steering for {STOP_HOLD_S:.0f} s")
                time.sleep(STOP_HOLD_S)
            if executor is not None:
                executor.shutdown(timeout_sec=2.0)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if spin_thread is not None:
                spin_thread.join(timeout=2.0)

    summary = {
        "topics": {
            "control": CONTROL_TOPIC,
            "gear": GEAR_TOPIC,
            "steering_feedback": STEERING_TOPIC,
            "velocity_feedback": VELOCITY_TOPIC,
        },
        "parameters": {
            "speed_mps": args.speed,
            "steering_angle_rad": args.steering_angle,
            "segment_seconds": args.segment_seconds,
            "cycles": args.cycles,
            "acceleration_mps2": args.acceleration,
            "deceleration_mps2": args.deceleration,
            "tracking_tolerance_rad": args.tracking_tolerance,
        },
        "passed": exit_code == 0,
        "phases": [asdict(result) for result in results],
    }
    with (output_dir / "summary.json").open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, indent=2, ensure_ascii=False)
        summary_file.write("\n")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
