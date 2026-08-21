#!/usr/bin/env python3
"""Round-2 CAN calibration: fill Byte6 gaps and validate 50 Hz ramp tracking."""

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
from rclpy.node import Node


CONTROL_PERIOD = 0.02
TARGET_SPEED = 0.5
BYTE6_VALUES = (4, 6, 7, 9)
RAMP_PROFILES = (
    # name, acceleration, Byte5, Byte6
    ("ramp_0p2", 0.2, 3, 3),
    ("ramp_0p3", 0.3, 5, 5),
)
REPETITIONS = 3
STABLE_DURATION = 1.0
HOLD_DURATION = 2.0
RAMP_HOLD_TIMEOUT = 10.0
SPEED_TOLERANCE = 0.02
FEEDBACK_TIMEOUT = 0.3
ACCEL_TIMEOUT = 20.0
STOP_TIMEOUT = 15.0
CAN_NODE = "/can_node"
ROS2_PARAM_OPTIONS = ("--no-daemon", "--spin-time", "3.0")
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "can0")
WORKSPACE = Path(os.environ.get("AUTOWARE_WORKSPACE", "/home/nvidia/autoware"))
RESULT_ROOT = Path(
    os.environ.get("RESULT_ROOT", str(WORKSPACE / "log" / "can_round2_calibration"))
)
ROUND_LABEL = "第二轮"
ANALYSIS_FILENAME = "round2_analysis.csv"
CLI_DESCRIPTION = __doc__
EXPECT_DYNAMIC_MODE = False
EXPECTED_ACCELERATION_COEFFICIENT: float | None = None
EXPECTED_DECELERATION_COEFFICIENT: float | None = None
USE_FIXED_STEPS = True
WARMUP_PROFILE: tuple[str, float, int, int] | None = None
EXPECTED_WIRE_PAIRS: tuple[tuple[int, int], ...] = ()


class Round2Node(Node):
    def __init__(self, event_writer, event_file, sample_writer, sample_file):
        super().__init__("can_round2_calibration")
        self._lock = threading.Lock()
        self._velocity = math.nan
        self._velocity_time = 0.0
        self._target_velocity = 0.0
        self._acceleration = -0.5
        self._phase = "initializing"
        self._test_name = "none"
        self._byte5 = 0
        self._byte6 = 0
        self._repetition = 0
        self._ramp_start_time: float | None = None
        self._ramp_start_velocity = 0.0
        self._ramp_end_velocity = 0.0
        self._ramp_rate = 0.0
        self._feedback_samples: list[tuple[float, float]] = []
        self._event_writer = event_writer
        self._event_file = event_file
        self._sample_writer = sample_writer
        self._sample_file = sample_file
        self.control_pub = self.create_publisher(Control, "/control/command/control_cmd", 10)
        self.gear_pub = self.create_publisher(GearCommand, "/control/command/gear_cmd", 10)
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self._on_velocity, 50
        )
        self.create_timer(CONTROL_PERIOD, self._publish_control)

    def _on_velocity(self, msg: VelocityReport) -> None:
        with self._lock:
            self._velocity = float(msg.longitudinal_velocity)
            self._velocity_time = time.monotonic()
            self._feedback_samples.append((self._velocity_time, self._velocity))

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
                time.time(), self._test_name, self._phase, self._byte5, self._byte6,
                self._repetition, velocity, acceleration, measured,
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

    def snapshot_feedback(self) -> list[tuple[float, float]]:
        with self._lock:
            return list(self._feedback_samples)

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

    def feedback(self) -> tuple[float, float]:
        with self._lock:
            return self._velocity, time.monotonic() - self._velocity_time

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


def run_command(command: list[str], output=None, check: bool = True,
                timeout: float = 15.0) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            command, text=True, stdout=output or subprocess.PIPE,
            stderr=subprocess.STDOUT, check=check, timeout=timeout,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(f"command timed out: {' '.join(command)}") from error


def set_can_steps(byte5: int, byte6: int) -> None:
    for name, value in (
        ("default_acceleration_step_command", byte5),
        ("default_deceleration_step_command", byte6),
    ):
        result = run_command(
            ["ros2", "param", "set", *ROS2_PARAM_OPTIONS, CAN_NODE, name, str(value)],
            check=False,
        )
        if "Set parameter successful" not in result.stdout:
            raise RuntimeError(f"failed to set {name}={value}: {result.stdout.strip()}")


def get_can_parameter(name: str) -> str:
    return run_command([
        "ros2", "param", "get", *ROS2_PARAM_OPTIONS, CAN_NODE, name,
    ]).stdout.strip()


def require_can_configuration() -> None:
    dynamic = get_can_parameter("use_dynamic_acceleration_steps")
    expected_text = "True" if EXPECT_DYNAMIC_MODE else "False"
    if expected_text not in dynamic:
        raise RuntimeError(
            f"use_dynamic_acceleration_steps must be {EXPECT_DYNAMIC_MODE}; got: {dynamic}"
        )
    for name, expected in (
        ("acceleration_step_counts_per_mps2", EXPECTED_ACCELERATION_COEFFICIENT),
        ("deceleration_step_counts_per_mps2", EXPECTED_DECELERATION_COEFFICIENT),
    ):
        if expected is None:
            continue
        output = get_can_parameter(name)
        try:
            actual = float(output.rsplit(" ", 1)[-1])
        except ValueError as error:
            raise RuntimeError(f"cannot parse {name}: {output}") from error
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-6):
            raise RuntimeError(f"{name} must be {expected}; got {actual}")


def wait_stable(node: Round2Node, predicate, timeout: float, label: str) -> bool:
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


def wait_ramp_duration(node: Round2Node, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        _, age = node.feedback()
        if age > FEEDBACK_TIMEOUT:
            raise RuntimeError(f"velocity feedback lost for {age:.3f}s")
        time.sleep(CONTROL_PERIOD)


def enter_to_continue(prompt: str) -> None:
    try:
        input(prompt)
    except EOFError as error:
        raise KeyboardInterrupt from error


def start_recorders(result_dir: Path) -> list[subprocess.Popen]:
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
    bag._owned_log = recorder_log  # type: ignore[attr-defined]
    candump._owned_log = can_log  # type: ignore[attr-defined]
    return [bag, candump]


def stop_recorders(processes: list[subprocess.Popen]) -> None:
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        getattr(process, "_owned_log", None).close()


def analyze_can_periods(can_log_path: Path, output_path: Path) -> None:
    pattern = re.compile(r"^\((\d+(?:\.\d+)?)\)")
    timestamps = []
    with can_log_path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = pattern.match(line.strip())
            if match:
                timestamps.append(float(match.group(1)))
    periods = [
        (current - previous) * 1000.0
        for previous, current in zip(timestamps, timestamps[1:])
        if current >= previous
    ]
    with output_path.open("w", encoding="utf-8") as stream:
        stream.write(f"frame_count={len(timestamps)}\n")
        if not periods:
            stream.write("result=insufficient_data\n")
            return
        ordered = sorted(periods)
        p99_index = min(len(ordered) - 1, math.ceil(len(ordered) * 0.99) - 1)
        mean_period = statistics.mean(periods)
        stream.write(f"mean_period_ms={mean_period:.3f}\n")
        stream.write(f"mean_frequency_hz={1000.0 / mean_period:.3f}\n")
        stream.write(f"p99_period_ms={ordered[p99_index]:.3f}\n")
        stream.write(f"max_period_ms={max(periods):.3f}\n")
        stream.write(f"intervals_over_25ms={sum(value > 25.0 for value in periods)}\n")
        stream.write(f"intervals_over_40ms={sum(value > 40.0 for value in periods)}\n")


def verify_wire_pairs(can_log_path: Path, output_path: Path) -> bool:
    counts = {pair: 0 for pair in EXPECTED_WIRE_PAIRS}
    with can_log_path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            octets = re.findall(r"\b[0-9A-Fa-f]{2}\b", line)
            if len(octets) < 8:
                continue
            frame = [int(value, 16) for value in octets[-8:]]
            pair = (frame[5], frame[6])
            if pair in counts:
                counts[pair] += 1
    with output_path.open("w", encoding="utf-8") as stream:
        for pair, count in counts.items():
            stream.write(f"byte5={pair[0]},byte6={pair[1]},frame_count={count}\n")
        passed = all(count > 0 for count in counts.values())
        stream.write(f"result={'pass' if passed else 'fail'}\n")
    return all(count > 0 for count in counts.values())


def stop_vehicle(node: Round2Node, byte6: int = 10) -> bool:
    if USE_FIXED_STEPS:
        set_can_steps(3, byte6)
    node.set_command(0.0, -0.5, "safety_stop")
    return wait_stable(
        node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
        STOP_TIMEOUT, "zero_speed",
    )


def run_byte6_trial(node: Round2Node, byte6: int, repetition: int) -> str:
    node.set_trial("byte6_gap", 8, byte6, repetition)
    set_can_steps(8, byte6)
    node.set_command(0.0, -0.5, "precheck_zero")
    if not wait_stable(node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
                       STOP_TIMEOUT, "precheck_zero"):
        return "precheck_timeout"
    node.set_command(TARGET_SPEED, 0.2, "accelerate_to_test_speed")
    if not wait_stable(node, lambda velocity: abs(velocity - TARGET_SPEED) <= SPEED_TOLERANCE,
                       ACCEL_TIMEOUT, "target_speed"):
        stop_vehicle(node)
        return "acceleration_timeout"
    node.set_command(TARGET_SPEED, 0.0, "hold")
    time.sleep(HOLD_DURATION)
    node.set_command(0.0, -0.5, "measured_stop")
    stopped = wait_stable(node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
                          STOP_TIMEOUT, "zero_speed")
    result = "success" if stopped else "stop_timeout"
    node.event("trial_end", result)
    return result


def run_ramp_trial(node: Round2Node, name: str, rate: float, byte5: int,
                   byte6: int, repetition: int) -> str:
    node.set_trial(name, byte5, byte6, repetition)
    if USE_FIXED_STEPS:
        set_can_steps(byte5, byte6)
    node.set_command(0.0, -rate, "precheck_zero")
    if not wait_stable(node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
                       STOP_TIMEOUT, "precheck_zero"):
        return "precheck_timeout"
    duration = TARGET_SPEED / rate
    node.start_ramp(0.0, TARGET_SPEED, rate, "ramp_up")
    wait_ramp_duration(node, duration)
    node.set_command(TARGET_SPEED, 0.0, "hold")
    if not wait_stable(
        node, lambda velocity: abs(velocity - TARGET_SPEED) <= SPEED_TOLERANCE,
        RAMP_HOLD_TIMEOUT, "target_speed",
    ):
        stop_vehicle(node)
        node.event("trial_end", "hold_timeout")
        return "hold_timeout"
    node.start_ramp(TARGET_SPEED, 0.0, -rate, "ramp_down")
    wait_ramp_duration(node, duration)
    node.set_command(0.0, -rate, "settle_zero")
    stopped = wait_stable(node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
                          STOP_TIMEOUT, "zero_speed")
    result = "success" if stopped else "stop_timeout"
    node.event("trial_end", result)
    return result


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def fitted_slope(rows: list[dict[str, str]], descending: bool) -> float:
    points = []
    for row in rows:
        measured = float(row["measured_velocity_mps"])
        if math.isfinite(measured) and 0.05 <= measured <= 0.4:
            points.append((float(row["timestamp"]), measured))
    if len(points) < 3:
        return math.nan
    mean_t = statistics.mean(point[0] for point in points)
    mean_v = statistics.mean(point[1] for point in points)
    denominator = sum((point[0] - mean_t) ** 2 for point in points)
    slope = sum((t - mean_t) * (v - mean_v) for t, v in points) / denominator
    return -slope if descending else slope


def tracking_metrics(rows: list[dict[str, str]]) -> tuple[float, float, float, float]:
    errors = []
    velocities = []
    for row in rows:
        measured = float(row["measured_velocity_mps"])
        if math.isfinite(measured):
            errors.append(measured - float(row["command_velocity_mps"]))
            velocities.append(measured)
    if not errors:
        return math.nan, math.nan, math.nan, math.nan
    rmse = math.sqrt(statistics.mean(error * error for error in errors))
    return rmse, statistics.mean(abs(error) for error in errors), max(
        abs(error) for error in errors
    ), min(velocities)


def response_delay(rows: list[dict[str, str]], descending: bool) -> float:
    if not rows:
        return math.nan
    start_time = float(rows[0]["timestamp"])
    threshold = TARGET_SPEED - 0.05 if descending else 0.05
    for row in rows:
        measured = float(row["measured_velocity_mps"])
        crossed = measured <= threshold if descending else measured >= threshold
        if math.isfinite(measured) and crossed:
            return float(row["timestamp"]) - start_time
    return math.nan


def analyze_result(result_dir: Path) -> int:
    rows = read_rows(result_dir / "tracking_samples.csv")
    events = read_rows(result_dir / "events.csv")
    groups: dict[tuple[str, int, int, int], list[dict[str, str]]] = {}
    for row in rows:
        if row["test_name"] == "none" or int(row["repetition"]) <= 0:
            continue
        key = (
            row["test_name"], int(row["byte5"]), int(row["byte6"]),
            int(row["repetition"]),
        )
        groups.setdefault(key, []).append(row)

    def phase_bounds(key, start_phase: str, end_event: str, end_phase: str):
        name, byte5, byte6, repetition = key
        matching = [
            event for event in events
            if event["test_name"] == name
            and int(event["byte5"]) == byte5
            and int(event["byte6"]) == byte6
            and int(event["repetition"]) == repetition
        ]
        start = next(
            float(event["timestamp"]) for event in matching
            if event["event"] == "phase_start" and event["phase"] == start_phase
        )
        end = next(
            float(event["timestamp"]) for event in matching
            if float(event["timestamp"]) > start
            and event["event"] == end_event and event["phase"] == end_phase
        )
        return start, end

    def phase_rows(key, start_phase: str, end_event: str, end_phase: str):
        start, end = phase_bounds(key, start_phase, end_event, end_phase)
        return [
            row for row in groups[key]
            if start <= float(row["timestamp"]) <= end
        ]

    output = result_dir / ANALYSIS_FILENAME
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "test_name", "repetition", "byte5", "byte6", "ramp_up_rmse_mps",
            "ramp_down_rmse_mps", "ramp_up_mae_mps", "ramp_down_mae_mps",
            "ramp_up_max_error_mps", "ramp_down_max_error_mps",
            "measured_acceleration_mps2", "measured_deceleration_mps2",
            "acceleration_response_delay_s", "deceleration_response_delay_s",
            "ramp_up_end_velocity_mps", "hold_catchup_s",
            "ramp_down_end_velocity_mps", "minimum_velocity_mps",
        ])
        for key, trial_rows in sorted(groups.items()):
            name, byte5, byte6, repetition = key
            if name == "byte6_gap":
                up = []
                down = phase_rows(key, "measured_stop", "zero_speed", "measured_stop")
            else:
                up = phase_rows(key, "ramp_up", "phase_start", "hold")
                down = phase_rows(key, "ramp_down", "phase_start", "settle_zero")
            up_metrics = tracking_metrics(up)
            down_metrics = tracking_metrics(down)
            minimum_velocities = [
                value for value in (up_metrics[3], down_metrics[3]) if math.isfinite(value)
            ]
            hold_catchup = math.nan
            if name != "byte6_gap":
                hold_start, _ = phase_bounds(key, "ramp_up", "phase_start", "hold")
                target_stable = next((
                    float(event["timestamp"]) for event in events
                    if event["test_name"] == name
                    and int(event["byte5"]) == byte5
                    and int(event["byte6"]) == byte6
                    and int(event["repetition"]) == repetition
                    and event["event"] == "target_speed"
                    and event["phase"] == "hold"
                ), math.nan)
                if math.isfinite(target_stable):
                    hold_catchup = target_stable - hold_start - STABLE_DURATION
            writer.writerow([
                name, repetition, byte5, byte6,
                up_metrics[0], down_metrics[0], up_metrics[1], down_metrics[1],
                up_metrics[2], down_metrics[2], fitted_slope(up, False),
                fitted_slope(down, True), response_delay(up, False),
                response_delay(down, True),
                float(up[-1]["measured_velocity_mps"]) if up else math.nan,
                hold_catchup,
                float(down[-1]["measured_velocity_mps"]) if down else math.nan,
                min(minimum_velocities, default=math.nan),
            ])
    print(f"{ROUND_LABEL}分析完成：{output}")
    return 0


def run_calibration() -> int:
    result_dir = RESULT_ROOT / time.strftime("%Y%m%d_%H%M%S")
    result_dir.mkdir(parents=True)
    print(f"测试脚本：{Path(__file__).resolve()}")
    print(
        f"运行模式：{ROUND_LABEL}，动态Byte5/Byte6="
        f"{'开启' if EXPECT_DYNAMIC_MODE else '关闭'}"
    )
    print(f"结果目录：{result_dir}")
    print(f"{ROUND_LABEL}封闭道路测试：最高0.5m/s；确认急停、净空和安全员均已就绪。")
    enter_to_continue("确认安全后按 Enter 开始（中止请按 Ctrl-C）：")
    for executable in ("ros2", "candump", "ip"):
        if subprocess.call(["which", executable], stdout=subprocess.DEVNULL) != 0:
            raise RuntimeError(f"missing executable: {executable}")
    run_command(["ip", "link", "show", CAN_INTERFACE])
    require_can_configuration()

    with (result_dir / "environment.txt").open("w", encoding="utf-8") as stream:
        stream.write(f"start_time={time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
        stream.write(f"byte6_values={BYTE6_VALUES}\nramp_profiles={RAMP_PROFILES}\n")
        stream.write(f"repetitions={REPETITIONS}\ntarget_speed={TARGET_SPEED}\n")
    with (result_dir / "can_node_params_before.yaml").open("w", encoding="utf-8") as stream:
        run_command(["ros2", "param", "dump", *ROS2_PARAM_OPTIONS, CAN_NODE], output=stream)

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
    node = Round2Node(events_writer, events_file, samples_writer, samples_file)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    recorders: list[subprocess.Popen] = []
    return_code = 0
    try:
        time.sleep(1.0)
        if len(node.get_publishers_info_by_topic("/control/command/control_cmd")) > 1:
            raise RuntimeError("another control_cmd publisher is active")
        node.publish_drive_gear()
        if not wait_stable(node, lambda velocity: abs(velocity) <= SPEED_TOLERANCE,
                           5.0, "initial_zero"):
            raise RuntimeError("vehicle is not stationary")
        recorders = start_recorders(result_dir)
        time.sleep(2.0)
        if recorders[1].poll() is not None or (result_dir / "can_201.log").stat().st_size == 0:
            raise RuntimeError(f"no 0x201 frames captured on {CAN_INTERFACE}")

        if WARMUP_PROFILE is not None:
            name, rate, byte5, byte6 = WARMUP_PROFILE
            print("执行一次不计入统计的预热运行...")
            warmup_result = run_ramp_trial(node, name, rate, byte5, byte6, 0)
            if warmup_result != "success":
                raise RuntimeError(f"warmup failed: {warmup_result}")
            stop_vehicle(node)
            enter_to_continue("预热完成，车辆复位并确认安全后按 Enter 开始正式测试：")
        for byte6 in BYTE6_VALUES:
            results = [run_byte6_trial(node, byte6, repetition)
                       for repetition in range(1, REPETITIONS + 1)]
            stop_vehicle(node)
            print(f"Byte6={byte6} 完成：{results}")
            enter_to_continue("车辆复位并确认安全后按 Enter 继续（中止请按 Ctrl-C）：")
        for name, rate, byte5, byte6 in RAMP_PROFILES:
            results = [run_ramp_trial(node, name, rate, byte5, byte6, repetition)
                       for repetition in range(1, REPETITIONS + 1)]
            stop_vehicle(node)
            print(f"{name} 完成：{results}")
            enter_to_continue("车辆复位并确认安全后按 Enter 继续（中止请按 Ctrl-C）：")
    except KeyboardInterrupt:
        print(f"{ROUND_LABEL}测试已中止，正在停车并保存已有数据。")
        return_code = 130
    except Exception as error:
        print(f"{ROUND_LABEL}测试失败：{type(error).__name__}: {error}", file=sys.stderr)
        node.event("fatal", str(error))
        return_code = 2
    finally:
        try:
            node.set_command(0.0, -0.5, "final_stop")
            time.sleep(2.0)
            stop_recorders(recorders)
            can_log_path = result_dir / "can_201.log"
            if can_log_path.exists():
                analyze_can_periods(can_log_path, result_dir / "can_period_stats.txt")
                if EXPECTED_WIRE_PAIRS and not verify_wire_pairs(
                    can_log_path, result_dir / "dynamic_wire_check.txt"
                ):
                    print("动态Byte5/Byte6线值核验失败。", file=sys.stderr)
                    if return_code == 0:
                        return_code = 2
            analyze_result(result_dir)
            if USE_FIXED_STEPS:
                try:
                    set_can_steps(10, 10)
                except Exception as error:
                    print(f"警告：CAN步进参数恢复为10失败：{error}", file=sys.stderr)
                    if return_code == 0:
                        return_code = 2
            with (result_dir / "can_node_params_after.yaml").open("w", encoding="utf-8") as stream:
                run_command([
                    "ros2", "param", "dump", *ROS2_PARAM_OPTIONS, CAN_NODE,
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
    parser = argparse.ArgumentParser(description=CLI_DESCRIPTION)
    parser.add_argument("--analyze-result", type=Path, metavar="DIR")
    args = parser.parse_args()
    return analyze_result(args.analyze_result) if args.analyze_result else run_calibration()


if __name__ == "__main__":
    sys.exit(main())
