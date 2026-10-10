#!/usr/bin/env python3
"""Low-speed steering step-response test through Autoware external control.

The script only publishes to /external/selected/control_cmd.  It intentionally
does not bypass vehicle_cmd_gate or write CAN frames directly.  Before every
motion test, select LOCAL external control with:

  bash src/byd/launch/scripts/setup_external_control_mode.sh

The test sequence is: centre -> left -> centre -> right -> centre.  Steering
feedback, speed, and yaw-rate feedback are recorded to CSV and each phase is
marked pass/fail by the final-window steering tracking error.
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


EXTERNAL_CONTROL_TOPIC = "/external/selected/control_cmd"
EXTERNAL_GEAR_TOPIC = "/external/selected/gear_cmd"
STEERING_STATUS_TOPIC = "/vehicle/status/steering_status"
VELOCITY_STATUS_TOPIC = "/vehicle/status/velocity_status"

CONTROL_PERIOD_S = 0.02
FEEDBACK_MAX_AGE_S = 0.5
STOP_HOLD_S = 3.0
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class FeedbackSample:
    elapsed_s: float
    phase: str
    commanded_angle_rad: float
    commanded_speed_mps: float
    measured_angle_rad: float
    measured_speed_mps: float
    heading_rate_radps: float


@dataclass(frozen=True)
class PhaseResult:
    phase: str
    command_angle_rad: float
    duration_s: float
    sample_count: int
    median_angle_rad: float | None
    median_error_rad: float | None
    maximum_error_rad: float | None
    median_speed_mps: float | None
    median_heading_rate_radps: float | None
    passed: bool
    reason: str


class LateralSteeringTest(Node):
    """Continuously publishes a bounded command and collects chassis feedback."""

    def __init__(self, sample_writer: csv.writer, sample_file) -> None:
        super().__init__("lateral_steering_response_test")
        self._lock = threading.Lock()
        self._started_at = time.monotonic()
        self._phase = "initialising"
        self._target_angle = 0.0
        self._target_speed = 0.0
        self._steering_angle = math.nan
        self._steering_time = 0.0
        self._speed = math.nan
        self._heading_rate = math.nan
        self._velocity_time = 0.0
        self._phase_samples: list[FeedbackSample] = []
        self._sample_writer = sample_writer
        self._sample_file = sample_file

        self._control_pub = self.create_publisher(Control, EXTERNAL_CONTROL_TOPIC, 10)
        self._gear_pub = self.create_publisher(GearCommand, EXTERNAL_GEAR_TOPIC, 10)
        self.create_subscription(
            SteeringReport, STEERING_STATUS_TOPIC, self._on_steering, 50
        )
        self.create_subscription(
            VelocityReport, VELOCITY_STATUS_TOPIC, self._on_velocity, 50
        )
        self.create_timer(CONTROL_PERIOD_S, self._publish_control)

    def _on_steering(self, msg: SteeringReport) -> None:
        now = time.monotonic()
        with self._lock:
            self._steering_angle = float(msg.steering_tire_angle)
            self._steering_time = now
            sample = FeedbackSample(
                elapsed_s=now - self._started_at,
                phase=self._phase,
                commanded_angle_rad=self._target_angle,
                commanded_speed_mps=self._target_speed,
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
            angle = self._target_angle
            speed = self._target_speed
        command = Control()
        command.stamp = self.get_clock().now().to_msg()
        command.lateral.steering_tire_angle = angle
        command.lateral.steering_tire_rotation_rate = 0.0
        command.lateral.is_defined_steering_tire_rotation_rate = False
        command.longitudinal.velocity = speed
        command.longitudinal.acceleration = 0.0
        command.longitudinal.jerk = 0.0
        command.longitudinal.is_defined_acceleration = False
        command.longitudinal.is_defined_jerk = False
        self._control_pub.publish(command)

    def other_external_publishers(self) -> list[str]:
        publishers = self.get_publishers_info_by_topic(EXTERNAL_CONTROL_TOPIC)
        return sorted(
            f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
            for info in publishers
            if info.node_name != self.get_name()
        )

    def set_command(self, phase: str, steering_angle_rad: float, speed_mps: float) -> None:
        with self._lock:
            self._phase = phase
            self._target_angle = steering_angle_rad
            self._target_speed = speed_mps
            self._phase_samples = []

    def publish_drive_gear(self) -> None:
        command = GearCommand()
        command.stamp = self.get_clock().now().to_msg()
        command.command = GearCommand.DRIVE
        for _ in range(3):
            self._gear_pub.publish(command)
            time.sleep(0.1)

    def feedback_ages(self) -> tuple[float, float]:
        with self._lock:
            now = time.monotonic()
            return now - self._steering_time, now - self._velocity_time

    def latest_speed(self) -> float:
        with self._lock:
            return self._speed

    def collect_phase_samples(self) -> list[FeedbackSample]:
        with self._lock:
            return list(self._phase_samples)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Low-speed left/centre/right steering response test."
    )
    parser.add_argument(
        "--arm", action="store_true",
        help="required confirmation that the closed test area and physical E-stop are ready",
    )
    parser.add_argument(
        "--speed", type=float, default=0.15, metavar="MPS",
        help="constant forward speed in m/s; 0 permits a stationary actuator test (default: 0.15)",
    )
    parser.add_argument(
        "--steering-angle", type=float, default=0.08, metavar="RAD",
        help="positive tyre steering angle in rad; the script tests both signs (default: 0.08)",
    )
    parser.add_argument(
        "--segment-seconds", type=float, default=3.0, metavar="SEC",
        help="duration of each left, centre, right, and final-centre segment (default: 3)",
    )
    parser.add_argument(
        "--cycles", type=int, default=1, metavar="N",
        help="number of left/centre/right/centre cycles (default: 1)",
    )
    parser.add_argument(
        "--tracking-tolerance", type=float, default=0.03, metavar="RAD",
        help="maximum final-window median steering error for a pass (default: 0.03)",
    )
    parser.add_argument(
        "--evaluation-window", type=float, default=1.0, metavar="SEC",
        help="final part of each phase used for the tracking verdict (default: 1)",
    )
    parser.add_argument(
        "--feedback-timeout", type=float, default=5.0, metavar="SEC",
        help="maximum time to wait for steering and speed feedback (default: 5)",
    )
    parser.add_argument(
        "--motion-timeout", type=float, default=8.0, metavar="SEC",
        help="maximum time to observe forward motion after the warm-up command (default: 8)",
    )
    parser.add_argument(
        "--min-measured-speed", type=float, default=0.03, metavar="MPS",
        help="required observed speed for a moving test (default: 0.03)",
    )
    parser.add_argument(
        "--output-dir", type=Path, default=None, metavar="DIR",
        help="directory for feedback.csv, phase_summary.csv, and summary.json",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if not args.arm:
        raise ValueError(
            "refusing to run: use --arm only after clearing the test area and checking the physical E-stop"
        )
    if not 0.0 <= args.speed <= 0.5:
        raise ValueError("--speed must be in [0, 0.5] m/s")
    if not 0.01 <= args.steering_angle <= 0.20:
        raise ValueError("--steering-angle must be in [0.01, 0.20] rad")
    if not 0.5 <= args.segment_seconds <= 15.0:
        raise ValueError("--segment-seconds must be in [0.5, 15] s")
    if not 1 <= args.cycles <= 5:
        raise ValueError("--cycles must be in [1, 5]")
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


def median_or_none(values: list[float]) -> float | None:
    finite_values = [value for value in values if math.isfinite(value)]
    return float(statistics.median(finite_values)) if finite_values else None


def evaluate_phase(
    name: str, command_angle: float, duration_s: float, samples: list[FeedbackSample],
    evaluation_window_s: float, tolerance_rad: float,
) -> PhaseResult:
    if not samples:
        return PhaseResult(
            name, command_angle, duration_s, 0, None, None, None, None, None,
            False, "no steering feedback received",
        )
    last_time = samples[-1].elapsed_s
    relevant = [
        sample for sample in samples
        if sample.elapsed_s >= last_time - evaluation_window_s
    ]
    angles = [sample.measured_angle_rad for sample in relevant]
    errors = [abs(angle - command_angle) for angle in angles]
    median_angle = median_or_none(angles)
    median_error = median_or_none(errors)
    maximum_error = max(errors) if errors else None
    median_speed = median_or_none([sample.measured_speed_mps for sample in relevant])
    median_heading_rate = median_or_none(
        [sample.heading_rate_radps for sample in relevant]
    )
    passed = median_error is not None and median_error <= tolerance_rad
    reason = (
        f"median error {median_error:.4f} rad <= {tolerance_rad:.4f} rad"
        if passed else
        f"median error {median_error:.4f} rad > {tolerance_rad:.4f} rad"
        if median_error is not None else "no finite steering feedback"
    )
    return PhaseResult(
        name, command_angle, duration_s, len(relevant), median_angle, median_error,
        maximum_error, median_speed, median_heading_rate, passed, reason,
    )


def run_phase(
    node: LateralSteeringTest, phase: str, steering_angle: float, args: argparse.Namespace
) -> PhaseResult:
    node.set_command(phase, steering_angle, args.speed)
    print(
        f"[phase] {phase}: angle={steering_angle:+.3f} rad, "
        f"speed={args.speed:.2f} m/s, duration={args.segment_seconds:.1f} s"
    )
    time.sleep(args.segment_seconds)
    steering_age, velocity_age = node.feedback_ages()
    if steering_age > FEEDBACK_MAX_AGE_S or velocity_age > FEEDBACK_MAX_AGE_S:
        raise RuntimeError(
            "feedback became stale during test "
            f"(steering age={steering_age:.2f}s, velocity age={velocity_age:.2f}s)"
        )
    result = evaluate_phase(
        phase, steering_angle, args.segment_seconds, node.collect_phase_samples(),
        args.evaluation_window, args.tracking_tolerance,
    )
    verdict = "PASS" if result.passed else "FAIL"
    print(f"[result] {phase}: {verdict}; {result.reason}")
    return result


def safely_stop(node: LateralSteeringTest | None) -> None:
    if node is None:
        return
    node.set_command("safety_stop", 0.0, 0.0)
    print(f"[safety] publishing centre steering and zero velocity for {STOP_HOLD_S:.0f} s")
    time.sleep(STOP_HOLD_S)


def main() -> int:
    args = parse_args()
    try:
        validate_args(args)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or WORKSPACE_ROOT / "log/lateral_steering_test" / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    node: LateralSteeringTest | None = None
    executor: MultiThreadedExecutor | None = None
    spin_thread: threading.Thread | None = None
    results: list[PhaseResult] = []
    exit_code = 1

    with (output_dir / "feedback.csv").open("w", newline="", encoding="utf-8") as sample_file, \
         (output_dir / "phase_summary.csv").open("w", newline="", encoding="utf-8") as result_file:
        sample_writer = csv.writer(sample_file)
        sample_writer.writerow(FeedbackSample.__dataclass_fields__.keys())
        result_writer = csv.writer(result_file)
        result_writer.writerow(PhaseResult.__dataclass_fields__.keys())

        try:
            rclpy.init()
            node = LateralSteeringTest(sample_writer, sample_file)
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()

            time.sleep(0.5)  # Allow ROS graph discovery before the exclusivity check.
            other_publishers = node.other_external_publishers()
            if other_publishers:
                raise RuntimeError(
                    "another publisher already owns "
                    f"{EXTERNAL_CONTROL_TOPIC}: {', '.join(other_publishers)}"
                )

            print(f"[record] {output_dir}")
            print("[preflight] waiting for fresh steering and velocity feedback")
            wait_until(
                lambda: max(node.feedback_ages()) <= FEEDBACK_MAX_AGE_S,
                args.feedback_timeout,
                "fresh /vehicle/status/steering_status and /vehicle/status/velocity_status",
            )

            node.publish_drive_gear()
            print("[preflight] DRIVE gear sent; applying a centre-steering warm-up command")
            node.set_command("warmup", 0.0, args.speed)
            if args.speed > 0.0:
                wait_until(
                    lambda: abs(node.latest_speed()) >= args.min_measured_speed,
                    args.motion_timeout,
                    "vehicle motion; check LOCAL external control, gear, and vehicle enable",
                )

            for cycle in range(1, args.cycles + 1):
                prefix = f"cycle_{cycle}"
                for suffix, angle in (
                    ("left", args.steering_angle),
                    ("centre_after_left", 0.0),
                    ("right", -args.steering_angle),
                    ("centre_after_right", 0.0),
                ):
                    result = run_phase(node, f"{prefix}_{suffix}", angle, args)
                    results.append(result)
                    result_writer.writerow(asdict(result).values())
                    result_file.flush()
                    if not result.passed:
                        raise RuntimeError(
                            f"steering tracking failed in {result.phase}: {result.reason}"
                        )

            exit_code = 0
            print("[completed] all steering phases passed")
        except KeyboardInterrupt:
            print("\n[interrupted] operator requested stop", file=sys.stderr)
            exit_code = 130
        except Exception as error:  # Ensure the controlled stop runs for every failure path.
            print(f"ERROR: {error}", file=sys.stderr)
            exit_code = 1
        finally:
            safely_stop(node)
            if executor is not None:
                executor.shutdown(timeout_sec=2.0)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if spin_thread is not None:
                spin_thread.join(timeout=2.0)

    summary = {
        "command": {
            "speed_mps": args.speed,
            "steering_angle_rad": args.steering_angle,
            "segment_seconds": args.segment_seconds,
            "cycles": args.cycles,
            "tracking_tolerance_rad": args.tracking_tolerance,
            "evaluation_window_seconds": args.evaluation_window,
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
