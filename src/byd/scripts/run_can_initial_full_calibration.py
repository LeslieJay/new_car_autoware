#!/usr/bin/env python3
"""Initial forward calibration for 0-1 m/s and requested 0-1 m/s²."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import threading
import time

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, VelocityReport
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter


SPEED_POINTS = tuple(round(value / 10.0, 1) for value in range(1, 11))
BYTE_SCAN = (1, 2, 3, 4, 5, 6, 8, 10, 12, 15, 20, 30)
LOOKUP_TARGETS = (0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50,
                  0.60, 0.70, 0.80, 0.90, 1.00)
REPETITIONS = 3
TARGET_SPEED = 1.0
SPEED_RAMP_RATE = 0.10
SPEED_HOLD_DURATION = 3.0
ACCELERATION_LABEL = 1.0
DECELERATION_LABEL = -1.0
SAFE_ACCEL_BYTE = 8
SAFE_DECEL_BYTE = 9
SATURATION_GAIN_THRESHOLD = 0.10
CONTROL_PERIOD = 0.02
STABLE_DURATION = 1.0
SPEED_TOLERANCE = 0.02
FEEDBACK_TIMEOUT = 0.3
ACCEL_TIMEOUT = 25.0
STOP_TIMEOUT = 20.0
PARAMETER_SERVICE_TIMEOUT = 15.0
CAN_PARAMETER_SETTLE_TIME = 0.06
CAN_NODE = "/can_node"
ROS2_PARAM_OPTIONS = ("--no-daemon", "--spin-time", "3.0")
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "can0")
WORKSPACE = Path(os.environ.get("AUTOWARE_WORKSPACE", "/home/nvidia/autoware"))
RESULT_ROOT = Path(os.environ.get(
    "RESULT_ROOT", str(WORKSPACE / "log" / "can_initial_full_calibration")
))


class CalibrationNode(Node):
    def __init__(self, event_writer, event_file, sample_writer, sample_file):
        super().__init__("can_initial_full_calibration")
        self._lock = threading.Lock()
        self._velocity = math.nan
        self._velocity_time = 0.0
        self._target_velocity = 0.0
        self._acceleration = DECELERATION_LABEL
        self._phase = "initializing"
        self._test_name = "none"
        self._byte5 = 0
        self._byte6 = 0
        self._repetition = 0
        self._ramp_start_time = None
        self._ramp_start_velocity = 0.0
        self._ramp_end_velocity = 0.0
        self._ramp_rate = 0.0
        self._feedback_samples = []
        self._event_writer = event_writer
        self._event_file = event_file
        self._sample_writer = sample_writer
        self._sample_file = sample_file
        self.control_pub = self.create_publisher(
            Control, "/control/command/control_cmd", 10
        )
        self.gear_pub = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", 10
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self._on_velocity, 50
        )
        self.can_parameter_client = self.create_client(
            SetParametersAtomically, f"{CAN_NODE}/set_parameters_atomically"
        )
        self.create_timer(CONTROL_PERIOD, self._publish_control)

    def _on_velocity(self, msg: VelocityReport) -> None:
        now = time.monotonic()
        with self._lock:
            self._velocity = float(msg.longitudinal_velocity)
            self._velocity_time = now
            self._feedback_samples.append((now, self._velocity))

    def _publish_control(self) -> None:
        now = time.monotonic()
        with self._lock:
            if self._ramp_start_time is not None:
                elapsed = max(0.0, now - self._ramp_start_time)
                velocity = self._ramp_start_velocity + self._ramp_rate * elapsed
                if self._ramp_rate >= 0.0:
                    velocity = min(velocity, self._ramp_end_velocity)
                else:
                    velocity = max(velocity, self._ramp_end_velocity)
                self._target_velocity = velocity
            velocity = self._target_velocity
            acceleration = self._acceleration
            measured = self._velocity
            row = [
                time.time(), self._test_name, self._phase, self._byte5,
                self._byte6, self._repetition, velocity, acceleration, measured,
            ]
        msg = Control()
        msg.stamp = self.get_clock().now().to_msg()
        msg.lateral.steering_tire_angle = 0.0
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.lateral.is_defined_steering_tire_rotation_rate = False
        msg.longitudinal.velocity = velocity
        msg.longitudinal.acceleration = acceleration
        msg.longitudinal.jerk = 0.0
        msg.longitudinal.is_defined_acceleration = True
        msg.longitudinal.is_defined_jerk = False
        self.control_pub.publish(msg)
        self._sample_writer.writerow(row)
        self._sample_file.flush()

    def set_trial(self, name: str, byte5: int, byte6: int, repetition: int) -> None:
        with self._lock:
            self._test_name = name
            self._byte5 = byte5
            self._byte6 = byte6
            self._repetition = repetition
            self._feedback_samples = []

    def set_command(self, velocity: float, acceleration: float, phase: str) -> float:
        with self._lock:
            self._ramp_start_time = None
            self._target_velocity = velocity
            self._acceleration = acceleration
            self._phase = phase
        self.event("phase_start", phase)
        return time.monotonic()

    def start_ramp(self, start: float, end: float, rate: float, phase: str) -> float:
        if rate == 0.0 or (end - start) * rate <= 0.0:
            raise ValueError("ramp rate must point from start velocity to end velocity")
        now = time.monotonic()
        with self._lock:
            self._target_velocity = start
            self._acceleration = rate
            self._phase = phase
            self._ramp_start_time = now
            self._ramp_start_velocity = start
            self._ramp_end_velocity = end
            self._ramp_rate = rate
        self.event("phase_start", phase)
        return now

    def feedback(self):
        with self._lock:
            return self._velocity, time.monotonic() - self._velocity_time

    def snapshot_feedback(self):
        with self._lock:
            return list(self._feedback_samples)

    def event(self, event: str, result: str) -> None:
        with self._lock:
            row = [
                time.time(), self._test_name, self._byte5, self._byte6,
                self._repetition, event, self._phase, self._target_velocity,
                self._acceleration, self._velocity, result,
            ]
        self._event_writer.writerow(row)
        self._event_file.flush()

    def publish_drive_gear(self) -> None:
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = GearCommand.DRIVE
        for _ in range(3):
            self.gear_pub.publish(msg)
            time.sleep(0.1)


def run_command(command, output=None, check=True, timeout=15.0):
    try:
        return subprocess.run(
            command, text=True, stdout=output or subprocess.PIPE,
            stderr=subprocess.STDOUT, check=check, timeout=timeout,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f"command timed out: {' '.join(command)}") from error


def set_can_steps(node: CalibrationNode, byte5: int, byte6: int) -> None:
    client = node.can_parameter_client
    if not client.wait_for_service(timeout_sec=PARAMETER_SERVICE_TIMEOUT):
        raise RuntimeError(
            f"parameter service unavailable: {CAN_NODE}/set_parameters_atomically"
        )
    request = SetParametersAtomically.Request()
    request.parameters = [
        Parameter(
            "default_acceleration_step_command", value=int(byte5)
        ).to_parameter_msg(),
        Parameter(
            "default_deceleration_step_command", value=int(byte6)
        ).to_parameter_msg(),
    ]
    future = client.call_async(request)
    deadline = time.monotonic() + PARAMETER_SERVICE_TIMEOUT
    while not future.done() and time.monotonic() < deadline:
        time.sleep(CONTROL_PERIOD)
    if not future.done():
        future.cancel()
        raise RuntimeError(
            f"timed out setting CAN steps Byte5={byte5}, Byte6={byte6}"
        )
    if future.exception() is not None:
        raise RuntimeError(
            f"failed to set CAN steps Byte5={byte5}, Byte6={byte6}: "
            f"{future.exception()}"
        )
    response = future.result()
    if response is None or not response.result.successful:
        reason = "no response" if response is None else response.result.reason
        raise RuntimeError(
            f"failed to set CAN steps Byte5={byte5}, Byte6={byte6}: {reason}"
        )
    # Allow several 20 ms control/CAN cycles to observe the new pair before motion.
    time.sleep(CAN_PARAMETER_SETTLE_TIME)


def require_fixed_mode() -> None:
    result = run_command([
        "ros2", "param", "get", *ROS2_PARAM_OPTIONS,
        CAN_NODE, "use_dynamic_acceleration_steps",
    ], check=False)
    if result.returncode != 0:
        raise RuntimeError(
            "can_node does not expose use_dynamic_acceleration_steps; "
            "deploy the calibration-capable can_driver first"
        )
    if "False" not in result.stdout:
        raise RuntimeError(
            "use_dynamic_acceleration_steps must be false; got: "
            f"{result.stdout.strip()}"
        )


def wait_stable(node, predicate, timeout: float, label: str) -> bool:
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline:
        velocity, age = node.feedback()
        if math.isnan(velocity):
            time.sleep(CONTROL_PERIOD)
            continue
        if age > FEEDBACK_TIMEOUT:
            raise RuntimeError(f"velocity feedback lost for {age:.3f}s")
        if predicate(velocity):
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= STABLE_DURATION:
                node.event(label, "stable")
                return True
        else:
            stable_since = None
        time.sleep(CONTROL_PERIOD)
    node.event(label, "timeout")
    return False


def wait_ramp_duration(node, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        _, age = node.feedback()
        if age > FEEDBACK_TIMEOUT:
            raise RuntimeError(f"velocity feedback lost for {age:.3f}s")
        time.sleep(CONTROL_PERIOD)


def start_recorders(result_dir: Path):
    topics = [
        "/control/command/control_cmd", "/control/command/gear_cmd",
        "/can_driver/debug/control_cmd_rx", "/can_driver/debug/control_cmd_can",
        "/vehicle/status/velocity_status", "/vehicle/status/steering_status",
    ]
    recorder_log = open(result_dir / "recorder.log", "w", encoding="utf-8")
    bag = subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(result_dir / "rosbag"), *topics],
        stdin=subprocess.DEVNULL, stdout=recorder_log, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    can_log = open(result_dir / "can_201.log", "w", encoding="utf-8")
    candump = subprocess.Popen(
        ["candump", "-t", "a", f"{CAN_INTERFACE},201:7FF"],
        stdin=subprocess.DEVNULL, stdout=can_log, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    bag._owned_log = recorder_log
    candump._owned_log = can_log
    return [bag, candump]


def stop_recorders(processes) -> None:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        process._owned_log.close()


def analyze_can_periods(can_log_path: Path, output_path: Path) -> None:
    pattern = re.compile(r"^\((\d+(?:\.\d+)?)\)")
    timestamps = []
    with can_log_path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = pattern.match(line.strip())
            if match:
                timestamps.append(float(match.group(1)))
    periods = [(current - previous) * 1000.0
               for previous, current in zip(timestamps, timestamps[1:])
               if current >= previous]
    with output_path.open("w", encoding="utf-8") as stream:
        stream.write(f"frame_count={len(timestamps)}\n")
        if not periods:
            stream.write("result=insufficient_data\n")
            return
        ordered = sorted(periods)
        p99 = ordered[min(len(ordered) - 1, math.ceil(len(ordered) * 0.99) - 1)]
        mean = statistics.mean(periods)
        stream.write(f"mean_period_ms={mean:.3f}\n")
        stream.write(f"mean_frequency_hz={1000.0 / mean:.3f}\n")
        stream.write(f"p99_period_ms={p99:.3f}\n")
        stream.write(f"max_period_ms={max(periods):.3f}\n")
        stream.write(f"intervals_over_25ms={sum(value > 25.0 for value in periods)}\n")
        stream.write(f"intervals_over_40ms={sum(value > 40.0 for value in periods)}\n")


def enter_to_continue(prompt: str) -> None:
    try:
        input(prompt)
    except EOFError as error:
        raise KeyboardInterrupt from error


def fit_slope(samples: list[tuple[float, float]], start: float, end: float,
              low: float = 0.10, high: float = 0.80) -> float:
    points = [(timestamp, velocity) for timestamp, velocity in samples
              if start <= timestamp <= end and low <= velocity <= high]
    if len(points) < 3:
        return math.nan
    mean_t = statistics.mean(timestamp for timestamp, _ in points)
    mean_v = statistics.mean(velocity for _, velocity in points)
    denominator = sum((timestamp - mean_t) ** 2 for timestamp, _ in points)
    if denominator == 0.0:
        return math.nan
    return sum((timestamp - mean_t) * (velocity - mean_v)
               for timestamp, velocity in points) / denominator


def ensure_zero(node: CalibrationNode) -> bool:
    set_can_steps(node, SAFE_ACCEL_BYTE, SAFE_DECEL_BYTE)
    node.set_command(0.0, DECELERATION_LABEL, "ensure_zero")
    return wait_stable(
        node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
        STOP_TIMEOUT, "zero_speed",
    )


def run_speed_trial(node: CalibrationNode, target: float, repetition: int) -> dict:
    name = f"speed_{target:.2f}"
    node.set_trial(name, 3, 4, repetition)
    if not ensure_zero(node):
        return {"result": "precheck_timeout"}
    set_can_steps(node, 3, 4)
    duration = target / SPEED_RAMP_RATE
    start = node.start_ramp(0.0, target, SPEED_RAMP_RATE, "speed_ramp")
    wait_ramp_duration(node, duration)
    node.set_command(target, 0.0, "speed_hold")
    stable = wait_stable(
        node, lambda velocity: abs(velocity - target) <= SPEED_TOLERANCE,
        ACCEL_TIMEOUT, "target_speed",
    )
    time.sleep(max(0.0, SPEED_HOLD_DURATION - STABLE_DURATION))
    end = time.monotonic()
    hold = [velocity for timestamp, velocity in node.snapshot_feedback()
            if end - 1.0 <= timestamp <= end]
    measured = statistics.median(hold) if hold else math.nan
    ensure_zero(node)
    result = "success" if stable else "target_timeout"
    node.event("trial_end", result)
    return {
        "result": result, "command_velocity_mps": target,
        "measured_velocity_mps": measured,
        "steady_error_mps": measured - target,
        "duration_s": end - start,
    }


def run_acceleration_trial(node: CalibrationNode, byte5: int,
                           repetition: int) -> dict:
    node.set_trial("byte5_acceleration", byte5, SAFE_DECEL_BYTE, repetition)
    if not ensure_zero(node):
        return {"result": "precheck_timeout"}
    set_can_steps(node, byte5, SAFE_DECEL_BYTE)
    start = time.monotonic()
    node.set_command(TARGET_SPEED, ACCELERATION_LABEL, "measured_acceleration")
    reached = wait_stable(
        node, lambda velocity: abs(velocity - TARGET_SPEED) <= SPEED_TOLERANCE,
        ACCEL_TIMEOUT, "target_speed",
    )
    end = time.monotonic()
    samples = node.snapshot_feedback()
    slope = fit_slope(samples, start, end)
    delay = next((timestamp - start for timestamp, velocity in samples
                  if timestamp >= start and velocity >= 0.05), math.nan)
    minimum = min((velocity for timestamp, velocity in samples if timestamp >= start),
                  default=math.nan)
    maximum = max((velocity for timestamp, velocity in samples if timestamp >= start),
                  default=math.nan)
    ensure_zero(node)
    result = "success" if reached and math.isfinite(slope) else "invalid"
    node.event("trial_end", result)
    return {
        "result": result, "measured_acceleration_mps2": slope,
        "response_delay_s": delay, "duration_s": end - start,
        "minimum_velocity_mps": minimum, "maximum_velocity_mps": maximum,
    }


def run_deceleration_trial(node: CalibrationNode, byte6: int,
                           repetition: int) -> dict:
    node.set_trial("byte6_deceleration", SAFE_ACCEL_BYTE, byte6, repetition)
    if not ensure_zero(node):
        return {"result": "precheck_timeout"}
    set_can_steps(node, SAFE_ACCEL_BYTE, byte6)
    node.set_command(TARGET_SPEED, 0.3, "accelerate_to_test_speed")
    reached = wait_stable(
        node, lambda velocity: abs(velocity - TARGET_SPEED) <= SPEED_TOLERANCE,
        ACCEL_TIMEOUT, "target_speed",
    )
    if not reached:
        ensure_zero(node)
        return {"result": "acceleration_timeout"}
    node.set_command(TARGET_SPEED, 0.0, "hold")
    time.sleep(2.0)
    start = time.monotonic()
    node.set_command(0.0, DECELERATION_LABEL, "measured_deceleration")
    stopped = wait_stable(
        node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
        STOP_TIMEOUT, "zero_speed",
    )
    end = time.monotonic()
    samples = node.snapshot_feedback()
    slope = -fit_slope(samples, start, end)
    delay = next((timestamp - start for timestamp, velocity in samples
                  if timestamp >= start and velocity <= TARGET_SPEED - 0.05), math.nan)
    minimum = min((velocity for timestamp, velocity in samples if timestamp >= start),
                  default=math.nan)
    result = "success" if stopped and math.isfinite(slope) else "invalid"
    node.event("trial_end", result)
    return {
        "result": result, "measured_acceleration_mps2": slope,
        "response_delay_s": delay, "duration_s": end - start,
        "minimum_velocity_mps": minimum,
    }


def median_success(rows: list[dict], field: str) -> float:
    values = [float(row[field]) for row in rows
              if row.get("result") == "success" and math.isfinite(float(row[field]))]
    return statistics.median(values) if values else math.nan


def should_stop_scan(medians: list[float]) -> str | None:
    if medians and medians[-1] >= 1.0:
        return "target_1mps2_reached"
    if len(medians) >= 3 and all(math.isfinite(value) and value > 0.0
                                 for value in medians[-3:]):
        gains = [
            (medians[index] - medians[index - 1]) / medians[index - 1]
            for index in (-2, -1)
        ]
        if all(gain < SATURATION_GAIN_THRESHOLD for gain in gains):
            return "two_consecutive_gains_below_10_percent"
    return None


def build_lookup(mapping: list[tuple[int, float]]) -> list[tuple[float, int | None]]:
    usable = [(byte, value) for byte, value in mapping
              if math.isfinite(value) and value > 0.0]
    monotonic = []
    running_max = 0.0
    for byte, value in sorted(usable):
        running_max = max(running_max, value)
        monotonic.append((byte, running_max))
    return [
        (target, next((byte for byte, value in monotonic if value >= target), None))
        for target in LOOKUP_TARGETS
    ]


def write_lookup(path: Path, acceleration, deceleration,
                 acceleration_max: float, deceleration_max: float) -> None:
    def format_values(rows):
        return "[" + ", ".join("not_calibrated" if value is None else str(value)
                                for _, value in rows) + "]"
    targets = "[" + ", ".join(f"{target:.2f}" for target, _ in acceleration) + "]"
    with path.open("w", encoding="utf-8") as stream:
        stream.write("initial_can_lookup:\n")
        stream.write("  speed_range_mps: [0.0, 1.0]\n")
        stream.write(f"  acceleration_targets_mps2: {targets}\n")
        stream.write(f"  byte5: {format_values(acceleration)}\n")
        stream.write(f"  byte6: {format_values(deceleration)}\n")
        stream.write(f"  calibrated_acceleration_max_mps2: {acceleration_max:.6f}\n")
        stream.write(f"  calibrated_deceleration_max_mps2: {deceleration_max:.6f}\n")
        stream.write("  protocol_byte_range: [1, 255]\n")
        stream.write("  note: preliminary fixed-byte lookup; do not extrapolate not_calibrated entries\n")


def run_cross_validation(node: CalibrationNode, writer, acceleration_lookup,
                         deceleration_lookup) -> None:
    cases = ((0.3, 0.10), (0.5, 0.20), (0.8, 0.30), (1.0, 0.50))
    accel_map = dict(acceleration_lookup)
    decel_map = dict(deceleration_lookup)
    for speed, rate in cases:
        byte5, byte6 = accel_map.get(rate), decel_map.get(rate)
        if byte5 is None or byte6 is None:
            writer.writerow([speed, rate, byte5, byte6, 0, "not_calibrated"])
            continue
        for repetition in range(1, REPETITIONS + 1):
            name = f"cross_v{speed:.1f}_a{rate:.2f}"
            node.set_trial(name, byte5, byte6, repetition)
            if not ensure_zero(node):
                writer.writerow([speed, rate, byte5, byte6, repetition,
                                 "precheck_timeout", math.nan, math.nan,
                                 math.nan, math.nan])
                continue
            set_can_steps(node, byte5, byte6)
            duration = speed / rate
            up_start = node.start_ramp(0.0, speed, rate, "cross_ramp_up")
            wait_ramp_duration(node, duration)
            up_end = time.monotonic()
            node.set_command(speed, 0.0, "cross_hold")
            reached = wait_stable(
                node, lambda velocity: abs(velocity - speed) <= SPEED_TOLERANCE,
                ACCEL_TIMEOUT, "target_speed",
            )
            down_start = node.start_ramp(speed, 0.0, -rate, "cross_ramp_down")
            wait_ramp_duration(node, duration)
            down_end = time.monotonic()
            samples = node.snapshot_feedback()
            def metrics(start, end, command_at):
                errors = [velocity - command_at(timestamp - start)
                          for timestamp, velocity in samples if start <= timestamp <= end]
                if not errors:
                    return math.nan, math.nan
                return (math.sqrt(statistics.mean(error * error for error in errors)),
                        max(abs(error) for error in errors))
            up_rmse, up_max = metrics(
                up_start, up_end, lambda elapsed: min(speed, rate * elapsed)
            )
            down_rmse, down_max = metrics(
                down_start, down_end,
                lambda elapsed: max(0.0, speed - rate * elapsed),
            )
            stopped = ensure_zero(node)
            result = "success" if reached and stopped else "timeout"
            node.event("trial_end", result)
            writer.writerow([speed, rate, byte5, byte6, repetition, result,
                             up_rmse, down_rmse, up_max, down_max])


def run_calibration() -> int:
    result_dir = RESULT_ROOT / time.strftime("%Y%m%d_%H%M%S")
    result_dir.mkdir(parents=True)
    print(f"结果目录：{result_dir}")
    print("前进方向全范围初标定：速度0～1m/s，请保证至少20m封闭直线和急停可用。")
    enter_to_continue("确认安全后按 Enter 开始（Ctrl-C中止）：")
    for executable in ("ros2", "candump", "ip"):
        if run_command(["which", executable], check=False).returncode != 0:
            raise RuntimeError(f"missing executable: {executable}")
    run_command(["ip", "link", "show", CAN_INTERFACE])
    require_fixed_mode()
    with (result_dir / "environment.txt").open("w", encoding="utf-8") as stream:
        stream.write(f"start_time={time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
        stream.write("direction=forward\n")
        stream.write("speed_range_mps=[0.0,1.0]\n")
        stream.write("requested_acceleration_range_mps2=[0.0,1.0]\n")
        stream.write(f"speed_points={SPEED_POINTS}\nbyte_scan={BYTE_SCAN}\n")
        stream.write(f"repetitions={REPETITIONS}\n")
    with (result_dir / "can_node_params_before.yaml").open(
        "w", encoding="utf-8"
    ) as stream:
        run_command([
            "ros2", "param", "dump", *ROS2_PARAM_OPTIONS, CAN_NODE,
        ], output=stream)

    events_file = (result_dir / "events.csv").open("w", newline="", encoding="utf-8")
    events_writer = csv.writer(events_file)
    events_writer.writerow([
        "timestamp", "test_name", "byte5", "byte6", "repetition", "event",
        "phase", "target_velocity", "command_acceleration", "measured_velocity", "result",
    ])
    samples_file = (result_dir / "tracking_samples.csv").open(
        "w", newline="", encoding="utf-8"
    )
    samples_writer = csv.writer(samples_file)
    samples_writer.writerow([
        "timestamp", "test_name", "phase", "byte5", "byte6", "repetition",
        "command_velocity_mps", "command_acceleration_mps2", "measured_velocity_mps",
    ])
    rclpy.init()
    node = CalibrationNode(events_writer, events_file, samples_writer, samples_file)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    recorders = []
    return_code = 0
    try:
        time.sleep(1.0)
        if len(node.get_publishers_info_by_topic("/control/command/control_cmd")) > 1:
            raise RuntimeError("another control_cmd publisher is active")
        node.publish_drive_gear()
        if not ensure_zero(node):
            raise RuntimeError("vehicle is not stationary")
        recorders = start_recorders(result_dir)
        time.sleep(2.0)
        if recorders[1].poll() is not None or (result_dir / "can_201.log").stat().st_size == 0:
            raise RuntimeError(f"no 0x201 frames captured on {CAN_INTERFACE}")

        speed_file = (result_dir / "speed_mapping.csv").open(
            "w", newline="", encoding="utf-8"
        )
        speed_writer = csv.DictWriter(speed_file, fieldnames=[
            "repetition", "result", "command_velocity_mps", "measured_velocity_mps",
            "steady_error_mps", "duration_s",
        ])
        speed_writer.writeheader()
        speed_groups = {}
        print("\n阶段1：速度映射")
        for target in SPEED_POINTS:
            target_results = []
            for repetition in range(1, REPETITIONS + 1):
                result = run_speed_trial(node, target, repetition)
                target_results.append(result)
                speed_writer.writerow({"repetition": repetition, **result})
                speed_file.flush()
            speed_groups[target] = target_results
            enter_to_continue(f"速度{target:.1f}m/s完成，复位安全后按 Enter：")
        speed_file.close()
        with (result_dir / "speed_mapping_summary.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "command_velocity_mps", "measured_velocity_median_mps",
                "steady_error_median_mps", "successful_repetitions",
            ])
            for target, trials in speed_groups.items():
                accepted = [trial for trial in trials if trial.get("result") == "success"]
                measured = [float(trial["measured_velocity_mps"]) for trial in accepted]
                errors = [float(trial["steady_error_mps"]) for trial in accepted]
                writer.writerow([
                    target, statistics.median(measured) if measured else math.nan,
                    statistics.median(errors) if errors else math.nan, len(accepted),
                ])

        mapping_results = {}
        for test_type, runner, output_name in (
            ("byte5", run_acceleration_trial, "byte5_acceleration_mapping.csv"),
            ("byte6", run_deceleration_trial, "byte6_deceleration_mapping.csv"),
        ):
            print(f"\n阶段：{test_type}自适应扫描")
            output_file = (result_dir / output_name).open("w", newline="", encoding="utf-8")
            fieldnames = [
                "byte", "repetition", "result", "measured_acceleration_mps2",
                "response_delay_s", "duration_s", "minimum_velocity_mps",
            ]
            if test_type == "byte5":
                fieldnames.append("maximum_velocity_mps")
            writer = csv.DictWriter(output_file, fieldnames=fieldnames)
            writer.writeheader()
            medians = []
            mapping = []
            for byte in BYTE_SCAN:
                trials = []
                for repetition in range(1, REPETITIONS + 1):
                    result = runner(node, byte, repetition)
                    trials.append(result)
                    writer.writerow({"byte": byte, "repetition": repetition, **result})
                    output_file.flush()
                median = median_success(trials, "measured_acceleration_mps2")
                medians.append(median)
                mapping.append((byte, median))
                reason = should_stop_scan(medians)
                print(f"{test_type} Byte={byte}：中位数={median:.4f}m/s²")
                if reason:
                    print(f"自适应停止：{reason}")
                    break
                enter_to_continue("复位并确认安全后按 Enter 继续；Ctrl-C中止：")
            output_file.close()
            mapping_results[test_type] = mapping

        acceleration_lookup = build_lookup(mapping_results["byte5"])
        deceleration_lookup = build_lookup(mapping_results["byte6"])
        with (result_dir / "byte_mapping_summary.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(["mapping", "byte", "measured_acceleration_median_mps2"])
            for mapping_name, mapping in mapping_results.items():
                writer.writerows((mapping_name, byte, value) for byte, value in mapping)
        acceleration_max = max(
            (value for _, value in mapping_results["byte5"] if math.isfinite(value)),
            default=math.nan,
        )
        deceleration_max = max(
            (value for _, value in mapping_results["byte6"] if math.isfinite(value)),
            default=math.nan,
        )
        write_lookup(result_dir / "initial_lookup_table.yaml", acceleration_lookup,
                     deceleration_lookup, acceleration_max, deceleration_max)

        enter_to_continue("初步查表已生成，确认安全后按 Enter 开始交叉验证：")
        with (result_dir / "cross_validation.csv").open(
            "w", newline="", encoding="utf-8"
        ) as cross_file:
            cross_writer = csv.writer(cross_file)
            cross_writer.writerow([
                "target_speed_mps", "ramp_rate_mps2", "byte5", "byte6",
                "repetition", "result", "ramp_up_rmse_mps",
                "ramp_down_rmse_mps", "ramp_up_max_error_mps",
                "ramp_down_max_error_mps",
            ])
            run_cross_validation(node, cross_writer, acceleration_lookup,
                                 deceleration_lookup)
    except KeyboardInterrupt:
        print("标定已中止，正在停车并保存已有数据。")
        return_code = 130
    except Exception as error:
        print(f"标定失败：{type(error).__name__}: {error}", file=sys.stderr)
        node.event("fatal", str(error))
        return_code = 2
    finally:
        try:
            node.set_command(0.0, DECELERATION_LABEL, "final_stop")
            time.sleep(2.0)
            stop_recorders(recorders)
            can_log = result_dir / "can_201.log"
            if can_log.exists():
                analyze_can_periods(can_log, result_dir / "can_period_stats.txt")
            try:
                set_can_steps(node, 10, 10)
            except Exception as error:
                print(f"警告：恢复Byte5/Byte6=10失败：{error}", file=sys.stderr)
                return_code = return_code or 2
            with (result_dir / "can_node_params_after.yaml").open(
                "w", encoding="utf-8"
            ) as stream:
                run_command([
                    "ros2", "param", "dump", *ROS2_PARAM_OPTIONS,
                    CAN_NODE,
                ], output=stream, check=False)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            spin_thread.join(timeout=2.0)
            events_file.close()
            samples_file.close()
            print(f"数据已保存：{result_dir}")
    return return_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    return run_calibration()


if __name__ == "__main__":
    sys.exit(main())
