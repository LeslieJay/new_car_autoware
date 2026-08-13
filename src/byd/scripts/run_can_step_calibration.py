#!/usr/bin/env python3
"""Collect a complete Byte5/Byte6 calibration matrix in one continuous run."""

from __future__ import annotations

import csv
import argparse
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import threading
import time
import re
import sqlite3

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, VelocityReport
from rclpy.node import Node


BYTE_VALUES = (1, 2, 3, 5, 8, 10, 15)
REPETITIONS = 3
TARGET_SPEED = 0.5
ACCELERATION_LABEL = 0.2
DECELERATION_LABEL = -0.5
SPEED_TOLERANCE = 0.02
STABLE_DURATION = 1.0
ACCEL_TIMEOUT = 25.0
STOP_TIMEOUT = 15.0
HOLD_DURATION = 2.0
FEEDBACK_TIMEOUT = 0.3
CONTROL_PERIOD = 0.02
CAN_NODE = "/can_node"
ROS2_PARAM_OPTIONS = ("--no-daemon", "--spin-time", "3.0")
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "can0")
WORKSPACE = Path(
    os.environ.get("AUTOWARE_WORKSPACE", "/home/nvidia/autoware")
)
RESULT_ROOT = Path(
    os.environ.get("RESULT_ROOT", str(WORKSPACE / "log" / "can_step_calibration"))
)


class CalibrationNode(Node):
    def __init__(self, event_writer: csv.writer, event_file,
                 velocity_writer: csv.writer, velocity_file):
        super().__init__("can_step_calibration")
        self._lock = threading.Lock()
        self._velocity = math.nan
        self._velocity_time = 0.0
        self._target_velocity = 0.0
        self._acceleration = DECELERATION_LABEL
        self._phase = "initializing"
        self._test_type = "none"
        self._byte_value = 0
        self._repetition = 0
        self._samples: list[tuple[float, float]] = []
        self._event_writer = event_writer
        self._event_file = event_file
        self._velocity_writer = velocity_writer
        self._velocity_file = velocity_file
        self.control_pub = self.create_publisher(Control, "/control/command/control_cmd", 10)
        self.gear_pub = self.create_publisher(GearCommand, "/control/command/gear_cmd", 10)
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self._on_velocity, 50
        )
        self.create_timer(CONTROL_PERIOD, self._publish_control)

    def _on_velocity(self, msg: VelocityReport) -> None:
        now = time.monotonic()
        velocity = float(msg.longitudinal_velocity)
        with self._lock:
            self._velocity = velocity
            self._velocity_time = now
            self._samples.append((now, velocity))
            self._velocity_writer.writerow([time.time(), velocity])
            self._velocity_file.flush()

    def _publish_control(self) -> None:
        with self._lock:
            velocity = self._target_velocity
            acceleration = self._acceleration
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

    def publish_drive_gear(self) -> None:
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = GearCommand.DRIVE
        for _ in range(3):
            self.gear_pub.publish(msg)
            time.sleep(0.1)

    def set_command(self, velocity: float, acceleration: float, phase: str) -> None:
        with self._lock:
            self._target_velocity = velocity
            self._acceleration = acceleration
            self._phase = phase
        self.event("phase_start", "running")

    def set_trial(self, test_type: str, byte_value: int, repetition: int) -> None:
        with self._lock:
            self._test_type = test_type
            self._byte_value = byte_value
            self._repetition = repetition
            self._samples = []

    def snapshot_samples(self) -> list[tuple[float, float]]:
        with self._lock:
            return list(self._samples)

    def feedback(self) -> tuple[float, float]:
        with self._lock:
            return self._velocity, time.monotonic() - self._velocity_time

    def event(self, phase: str, result: str) -> None:
        with self._lock:
            row = [
                time.time(), self._test_type, self._byte_value, self._repetition,
                phase, self._target_velocity, self._acceleration, self._velocity, result,
            ]
        self._event_writer.writerow(row)
        self._event_file.flush()


def run_command(command: list[str], output=None, check: bool = True,
                timeout: float = 15.0) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            command, text=True, stdout=output or subprocess.PIPE,
            stderr=subprocess.STDOUT, check=check, timeout=timeout,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f"command timed out after {timeout:g}s: {' '.join(command)}"
        ) from error


def set_can_steps(acceleration_step: int, deceleration_step: int) -> None:
    for name, value in (
        ("default_acceleration_step_command", acceleration_step),
        ("default_deceleration_step_command", deceleration_step),
    ):
        result = run_command(
            ["ros2", "param", "set", *ROS2_PARAM_OPTIONS,
             CAN_NODE, name, str(value)],
            check=False,
        )
        output = result.stdout.strip()
        if "Set parameter successful" not in output:
            raise RuntimeError(
                f"failed to set {name}={value} (exit {result.returncode}): {output}"
            )
        if result.returncode != 0:
            print(
                f"警告：{name}={value} 已设置成功，但 ros2 param set "
                f"退出码为 {result.returncode}；继续标定。",
                file=sys.stderr,
            )


def wait_stable(node: CalibrationNode, predicate, timeout: float, label: str) -> bool:
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline:
        velocity, feedback_age = node.feedback()
        if math.isnan(velocity):
            time.sleep(0.02)
            continue
        if feedback_age > FEEDBACK_TIMEOUT:
            raise RuntimeError(f"velocity feedback lost for {feedback_age:.3f}s")
        if predicate(velocity):
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= STABLE_DURATION:
                node.event(label, "stable")
                return True
        else:
            stable_since = None
        time.sleep(0.02)
    node.event(label, "timeout")
    return False


def linear_slope(samples: list[tuple[float, float]], low: float, high: float) -> float:
    points = [(t, v) for t, v in samples if low <= abs(v) <= high]
    if len(points) < 3:
        return math.nan
    mean_t = statistics.mean(t for t, _ in points)
    mean_v = statistics.mean(v for _, v in points)
    denominator = sum((t - mean_t) ** 2 for t, _ in points)
    if denominator == 0.0:
        return math.nan
    return sum((t - mean_t) * (v - mean_v) for t, v in points) / denominator


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def load_velocity_samples(result_dir: Path) -> list[tuple[float, float]]:
    sample_path = result_dir / "velocity_samples.csv"
    if sample_path.exists():
        return [
            (float(row["timestamp"]), float(row["velocity_mps"]))
            for row in read_csv_rows(sample_path)
        ]

    bag_files = sorted((result_dir / "rosbag").glob("*.db3"))
    if not bag_files:
        raise FileNotFoundError("velocity_samples.csv or rosbag/*.db3 is required")
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    samples = []
    for bag_path in bag_files:
        database = sqlite3.connect(bag_path)
        try:
            topic = database.execute(
                "SELECT id, type FROM topics WHERE name = ?",
                ("/vehicle/status/velocity_status",),
            ).fetchone()
            if topic is None:
                continue
            topic_id, type_name = topic
            message_type = get_message(type_name)
            for timestamp, data in database.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (topic_id,),
            ):
                message = deserialize_message(data, message_type)
                samples.append((timestamp / 1e9, float(message.longitudinal_velocity)))
        finally:
            database.close()
    return samples


def analyze_result(result_dir: Path) -> int:
    events = read_csv_rows(result_dir / "events.csv")
    velocity_samples = load_velocity_samples(result_dir)
    phase_ends = {
        (row["test_type"], int(row["byte_value"]), int(row["repetition"])):
        float(row["timestamp"])
        for row in events
        if (row["test_type"] == "byte5_acceleration" and row["phase"] == "target_speed")
        or (row["test_type"] == "byte6_deceleration" and row["phase"] == "zero_speed")
    }
    trials = []
    for row in events:
        if row["phase"] != "phase_start":
            continue
        test_type = row["test_type"]
        target_velocity = float(row["target_velocity"])
        acceleration = float(row["command_acceleration"])
        is_acceleration = test_type == "byte5_acceleration" and target_velocity > 0.0 and acceleration > 0.0
        is_deceleration = test_type == "byte6_deceleration" and target_velocity == 0.0 and acceleration < 0.0
        if not (is_acceleration or is_deceleration):
            continue
        byte_value = int(row["byte_value"])
        repetition = int(row["repetition"])
        start = float(row["timestamp"])
        end = phase_ends.get((test_type, byte_value, repetition))
        if end is None or end <= start:
            continue
        samples = [(timestamp - start, velocity) for timestamp, velocity in velocity_samples
                   if start <= timestamp <= end]
        low = TARGET_SPEED * 0.1
        high = TARGET_SPEED * 0.8
        slope = linear_slope(samples, low, high)
        if is_acceleration:
            onset = next((timestamp for timestamp, velocity in samples if velocity >= low), math.nan)
        else:
            onset = next(
                (timestamp for timestamp, velocity in samples if velocity <= TARGET_SPEED - low),
                math.nan,
            )
        trials.append({
            "test_type": test_type,
            "byte_value": byte_value,
            "repetition": repetition,
            "response_delay_s": onset,
            "fitted_acceleration_mps2": abs(slope),
            "max_velocity_mps": max((velocity for _, velocity in samples), default=math.nan),
            "min_velocity_mps": min((velocity for _, velocity in samples), default=math.nan),
            "phase_duration_s": end - start,
            "quality": "ok",
        })

    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for trial in trials:
        grouped.setdefault(
            (str(trial["test_type"]), int(trial["byte_value"])), []
        ).append(trial)
    group_rows = []
    usable_acceleration_ratios = []
    usable_deceleration_ratios = []
    for (test_type, byte_value), group_trials in sorted(grouped.items()):
        values = [float(trial["fitted_acceleration_mps2"]) for trial in group_trials]
        median_value = statistics.median(values)
        for trial in group_trials:
            value = float(trial["fitted_acceleration_mps2"])
            if median_value > 0.0 and abs(value - median_value) / median_value > 0.30:
                trial["quality"] = "outlier"
            elif value > 0.0:
                if test_type == "byte5_acceleration":
                    usable_acceleration_ratios.append(byte_value / value)
                elif test_type == "byte6_deceleration":
                    usable_deceleration_ratios.append(byte_value / value)
        usable_values = [
            float(trial["fitted_acceleration_mps2"])
            for trial in group_trials if trial["quality"] == "ok"
        ]
        group_rows.append([
            test_type, byte_value, median_value,
            statistics.median(usable_values) if usable_values else math.nan,
            sum(trial["quality"] == "outlier" for trial in group_trials),
        ])

    output_path = result_dir / "calibration_analysis.csv"
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "test_type", "byte_value", "repetition", "response_delay_s",
            "fitted_acceleration_mps2", "max_velocity_mps", "min_velocity_mps",
            "phase_duration_s", "quality",
        ])
        writer.writerows([[trial[column] for column in (
            "test_type", "byte_value", "repetition", "response_delay_s",
            "fitted_acceleration_mps2", "max_velocity_mps", "min_velocity_mps",
            "phase_duration_s", "quality",
        )] for trial in trials])
    with (result_dir / "calibration_groups.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "test_type", "byte_value", "median_acceleration_mps2",
            "accepted_median_mps2", "outlier_count",
        ])
        writer.writerows(group_rows)
    acceleration_groups = [row for row in group_rows if row[0] == "byte5_acceleration"]
    acceleration_medians = [float(row[3]) for row in acceleration_groups]
    monotonic = len(acceleration_medians) >= 3 and all(
        current > previous
        for previous, current in zip(acceleration_medians, acceleration_medians[1:])
    )
    coefficient = (
        statistics.median(usable_acceleration_ratios)
        if monotonic and usable_acceleration_ratios else math.nan
    )
    deceleration_groups = [row for row in group_rows if row[0] == "byte6_deceleration"]
    deceleration_medians = [float(row[3]) for row in deceleration_groups]
    deceleration_monotonic = len(deceleration_medians) >= 3 and all(
        current > previous
        for previous, current in zip(deceleration_medians, deceleration_medians[1:])
    )
    deceleration_coefficient = (
        statistics.median(usable_deceleration_ratios)
        if deceleration_monotonic and usable_deceleration_ratios else math.nan
    )
    with (result_dir / "calibration_candidates.yaml").open("w", encoding="utf-8") as stream:
        stream.write("calibration_candidates:\n")
        stream.write(f"  acceleration_monotonic: {str(monotonic).lower()}\n")
        if math.isfinite(coefficient):
            stream.write(
                f"  acceleration_step_counts_per_mps2: {round(coefficient, 6)}\n"
            )
        else:
            stream.write("  acceleration_step_counts_per_mps2: not_ready\n")
        stream.write(
            f"  deceleration_monotonic: {str(deceleration_monotonic).lower()}\n"
        )
        if math.isfinite(deceleration_coefficient):
            stream.write(
                "  deceleration_step_counts_per_mps2: "
                f"{round(deceleration_coefficient, 6)}\n"
            )
        else:
            stream.write("  deceleration_step_counts_per_mps2: not_ready\n")
        stream.write("  speed_mapping: not_calibrated\n")
        stream.write("  steering_mapping: not_calibrated\n")
    if not trials:
        print("未找到完整的加速或减速阶段。", file=sys.stderr)
        return 2
    print(f"分析完成：{output_path}")
    return 0


def summarize_trial(writer: csv.writer, node: CalibrationNode, test_type: str,
                    byte_value: int, repetition: int, start: float, end: float,
                    result: str) -> None:
    samples = [(t - start, v) for t, v in node.snapshot_samples() if start <= t <= end]
    if test_type == "byte5_acceleration":
        onset = next((t for t, v in samples if v >= TARGET_SPEED * 0.1), math.nan)
    else:
        onset = next((t for t, v in samples if v <= TARGET_SPEED - 0.01), math.nan)
    slope = linear_slope(samples, TARGET_SPEED * 0.1, TARGET_SPEED * 0.8)
    velocities = [v for _, v in samples]
    writer.writerow([
        test_type, byte_value, repetition, result, onset, slope,
        max(velocities, default=math.nan), min(velocities, default=math.nan),
        samples[-1][0] if samples else math.nan,
    ])


def stop_vehicle(node: CalibrationNode) -> bool:
    node.set_command(0.0, DECELERATION_LABEL, "stop")
    return wait_stable(node, lambda value: abs(value) <= SPEED_TOLERANCE,
                       STOP_TIMEOUT, "zero_speed")


def run_trial(node: CalibrationNode, summary_writer: csv.writer, test_type: str,
              byte_value: int, repetition: int) -> str:
    node.set_trial(test_type, byte_value, repetition)
    if test_type == "byte5_acceleration":
        set_can_steps(byte_value, 10)
    else:
        set_can_steps(10, byte_value)

    node.set_command(0.0, DECELERATION_LABEL, "precheck_zero")
    if not wait_stable(node, lambda value: abs(value) <= SPEED_TOLERANCE,
                       STOP_TIMEOUT, "precheck_zero"):
        return "precheck_timeout"

    start = time.monotonic()
    node.set_command(TARGET_SPEED, ACCELERATION_LABEL, "accelerate")
    reached = wait_stable(
        node, lambda value: abs(value - TARGET_SPEED) <= SPEED_TOLERANCE,
        ACCEL_TIMEOUT, "target_speed",
    )
    if not reached:
        stop_vehicle(node)
        analysis_end = time.monotonic()
        summarize_trial(summary_writer, node, test_type, byte_value, repetition,
                        start, analysis_end, "acceleration_timeout")
        return "acceleration_timeout"

    acceleration_end = time.monotonic()

    node.set_command(TARGET_SPEED, 0.0, "hold")
    time.sleep(HOLD_DURATION)
    stop_start = time.monotonic()
    stopped = stop_vehicle(node)
    stop_end = time.monotonic()
    result = "success" if stopped else "stop_timeout"
    analysis_start = start if test_type == "byte5_acceleration" else stop_start
    analysis_end = acceleration_end if test_type == "byte5_acceleration" else stop_end
    summarize_trial(summary_writer, node, test_type, byte_value, repetition,
                    analysis_start, analysis_end, result)
    node.event("trial_end", result)
    return result


def start_recorders(result_dir: Path) -> list[subprocess.Popen]:
    topics = [
        "/control/command/control_cmd", "/control/command/gear_cmd",
        "/can_driver/debug/control_cmd_rx", "/can_driver/debug/control_cmd_can",
        "/vehicle/status/velocity_status", "/vehicle/status/steering_status",
    ]
    recorder_log = open(result_dir / "recorder.log", "w", encoding="utf-8")
    bag = subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(result_dir / "rosbag"), *topics],
        stdin=subprocess.DEVNULL,
        stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    can_log = open(result_dir / "can_201.log", "w", encoding="utf-8")
    candump = subprocess.Popen(
        ["candump", "-t", "a", f"{CAN_INTERFACE},201:7FF"],
        stdin=subprocess.DEVNULL,
        stdout=can_log, stderr=subprocess.STDOUT, start_new_session=True,
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
        owned_log = getattr(process, "_owned_log", None)
        if owned_log:
            owned_log.close()


def analyze_can_periods(can_log_path: Path, output_path: Path) -> None:
    timestamps: list[float] = []
    timestamp_pattern = re.compile(r"^\((\d+(?:\.\d+)?)\)")
    with open(can_log_path, encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = timestamp_pattern.match(line.strip())
            if match:
                timestamps.append(float(match.group(1)))

    intervals_ms = [
        (current - previous) * 1000.0
        for previous, current in zip(timestamps, timestamps[1:])
        if current >= previous
    ]
    with open(output_path, "w", encoding="utf-8") as stream:
        stream.write(f"frame_count={len(timestamps)}\n")
        if not intervals_ms:
            stream.write("result=insufficient_data\n")
            return
        sorted_intervals = sorted(intervals_ms)
        p99_index = min(len(sorted_intervals) - 1, math.ceil(len(sorted_intervals) * 0.99) - 1)
        mean_period = statistics.mean(intervals_ms)
        stream.write(f"mean_period_ms={mean_period:.3f}\n")
        stream.write(f"mean_frequency_hz={1000.0 / mean_period:.3f}\n")
        stream.write(f"p99_period_ms={sorted_intervals[p99_index]:.3f}\n")
        stream.write(f"max_period_ms={max(intervals_ms):.3f}\n")
        stream.write(f"intervals_over_25ms={sum(value > 25.0 for value in intervals_ms)}\n")
        stream.write(f"intervals_over_40ms={sum(value > 40.0 for value in intervals_ms)}\n")


def run_calibration() -> int:
    result_dir = RESULT_ROOT / time.strftime("%Y%m%d_%H%M%S")
    result_dir.mkdir(parents=True)
    print(f"结果目录：{result_dir}")
    print("封闭道路测试，目标速度0.5m/s。确认急停可用、前方净空充足。")
    input("确认安全后按 Enter 开始：")

    for executable in ("ros2", "candump", "ip"):
        if subprocess.call(["which", executable], stdout=subprocess.DEVNULL) != 0:
            raise RuntimeError(f"missing executable: {executable}")
    print(f"检查 CAN 接口 {CAN_INTERFACE}...", flush=True)
    run_command(["ip", "link", "show", CAN_INTERFACE])
    print(f"读取 {CAN_NODE} 参数...", flush=True)
    dynamic_mode = run_command(
        ["ros2", "param", "get", *ROS2_PARAM_OPTIONS,
         CAN_NODE, "use_dynamic_acceleration_steps"]
    ).stdout
    if "False" not in dynamic_mode:
        raise RuntimeError(
            "use_dynamic_acceleration_steps must be false when can_node starts"
        )

    with open(result_dir / "environment.txt", "w", encoding="utf-8") as stream:
        stream.write(f"start_time={time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
        stream.write(f"workspace={WORKSPACE}\ncan_interface={CAN_INTERFACE}\n")
        stream.write(f"byte_values={BYTE_VALUES}\nrepetitions={REPETITIONS}\n")
        stream.write(f"target_speed={TARGET_SPEED}\n")
    with open(result_dir / "can_node_params_before.yaml", "w", encoding="utf-8") as stream:
        run_command(
            ["ros2", "param", "dump", *ROS2_PARAM_OPTIONS, CAN_NODE],
            output=stream,
        )

    events_file = open(result_dir / "events.csv", "w", newline="", encoding="utf-8")
    events_writer = csv.writer(events_file)
    events_writer.writerow([
        "timestamp", "test_type", "byte_value", "repetition", "phase",
        "target_velocity", "command_acceleration", "measured_velocity", "result",
    ])
    summary_file = open(result_dir / "summary.csv", "w", newline="", encoding="utf-8")
    summary_writer = csv.writer(summary_file)
    summary_writer.writerow([
        "test_type", "byte_value", "repetition", "result", "response_delay_s",
        "fitted_acceleration_mps2", "max_velocity_mps", "min_velocity_mps", "duration_s",
    ])
    velocity_file = open(
        result_dir / "velocity_samples.csv", "w", newline="", encoding="utf-8"
    )
    velocity_writer = csv.writer(velocity_file)
    velocity_writer.writerow(["timestamp", "velocity_mps"])

    rclpy.init()
    node = CalibrationNode(events_writer, events_file, velocity_writer, velocity_file)
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    recorders: list[subprocess.Popen] = []
    try:
        print("等待 ROS 话题与速度反馈...", flush=True)
        time.sleep(1.0)
        if len(node.get_publishers_info_by_topic("/control/command/control_cmd")) > 1:
            raise RuntimeError("another control_cmd publisher is active; stop it before calibration")
        node.publish_drive_gear()
        if not wait_stable(node, lambda value: abs(value) <= SPEED_TOLERANCE,
                           5.0, "initial_zero"):
            raise RuntimeError("vehicle is not stationary")
        recorders = start_recorders(result_dir)
        print("启动 rosbag 与 CAN 记录器...", flush=True)
        time.sleep(2.0)
        if recorders[1].poll() is not None or (result_dir / "can_201.log").stat().st_size == 0:
            raise RuntimeError(f"no 0x201 frames captured on {CAN_INTERFACE}")

        with open(result_dir / "test_matrix.csv", "w", newline="", encoding="utf-8") as matrix:
            matrix_writer = csv.writer(matrix)
            matrix_writer.writerow(["test_type", "byte_value", "repetition"])
            for test_type in ("byte5_acceleration", "byte6_deceleration"):
                for byte_value in BYTE_VALUES:
                    results = []
                    for repetition in range(1, REPETITIONS + 1):
                        matrix_writer.writerow([test_type, byte_value, repetition])
                        matrix.flush()
                        print(f"\n{test_type}: Byte={byte_value}, repeat={repetition}/{REPETITIONS}")
                        results.append(run_trial(
                            node, summary_writer, test_type, byte_value, repetition
                        ))
                        summary_file.flush()
                    node.set_command(0.0, DECELERATION_LABEL, "manual_reset")
                    print(f"Byte={byte_value} 完成：{results}")
                    try:
                        answer = input(
                            "车辆复位并确认安全后按 Enter 继续；输入 ABORT 结束："
                        ).strip()
                    except EOFError:
                        print("未能读取终端输入，按安全中止处理。", file=sys.stderr)
                        raise KeyboardInterrupt
                    if answer.upper() == "ABORT":
                        raise KeyboardInterrupt

        return_code = 0
    except KeyboardInterrupt:
        print("标定已中止，正在安全停车并保存已有数据。")
        return_code = 130
    except Exception as error:
        error_message = f"{type(error).__name__}: {error}"
        print(f"标定失败：{error_message}", file=sys.stderr)
        node.event("fatal", error_message)
        return_code = 2
    finally:
        try:
            node.set_command(0.0, DECELERATION_LABEL, "final_stop")
            time.sleep(2.0)
            stop_recorders(recorders)
            can_log_path = result_dir / "can_201.log"
            if can_log_path.exists():
                analyze_can_periods(can_log_path, result_dir / "can_period_stats.txt")
            try:
                analyze_result(result_dir)
            except Exception as error:
                print(f"警告：自动生成标定候选失败：{error}", file=sys.stderr)
            try:
                set_can_steps(10, 10)
            except Exception as error:
                print(f"警告：CAN 步进参数恢复为 10 失败：{error}", file=sys.stderr)
                node.event("cleanup", f"failed to restore CAN steps: {error}")
                if return_code == 0:
                    return_code = 2
            with open(
                result_dir / "can_node_params_after.yaml", "w", encoding="utf-8"
            ) as stream:
                run_command(
                    ["ros2", "param", "dump", *ROS2_PARAM_OPTIONS, CAN_NODE],
                    output=stream, check=False,
                )
        finally:
            node.destroy_node()
            rclpy.shutdown()
            executor_thread.join(timeout=2.0)
            events_file.close()
            summary_file.close()
            velocity_file.close()
            print(f"数据已保存：{result_dir}")
    return return_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--analyze-result", type=Path, metavar="DIR",
        help="reanalyze an existing calibration result without moving the vehicle",
    )
    args = parser.parse_args()
    if args.analyze_result:
        return analyze_result(args.analyze_result)
    return run_calibration()


if __name__ == "__main__":
    sys.exit(main())
