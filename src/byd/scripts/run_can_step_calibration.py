#!/usr/bin/env python3
"""Collect a complete Byte5/Byte6 calibration matrix in one continuous run."""

from __future__ import annotations

import csv
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
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "can0")
WORKSPACE = Path(
    os.environ.get("AUTOWARE_WORKSPACE", "/home/byd/weicanming/github_projects/new_car_autoware")
)
RESULT_ROOT = Path(
    os.environ.get("RESULT_ROOT", str(WORKSPACE / "log" / "can_step_calibration"))
)


class CalibrationNode(Node):
    def __init__(self, event_writer: csv.writer, event_file):
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


def run_command(command: list[str], output=None, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(command, text=True, stdout=output or subprocess.PIPE,
                          stderr=subprocess.STDOUT, check=check)


def set_can_steps(acceleration_step: int, deceleration_step: int) -> None:
    for name, value in (
        ("default_acceleration_step_command", acceleration_step),
        ("default_deceleration_step_command", deceleration_step),
    ):
        result = run_command(
            ["ros2", "param", "set", CAN_NODE, name, str(value)], check=False
        )
        if result.returncode != 0 or "Successful" not in result.stdout:
            raise RuntimeError(f"failed to set {name}={value}: {result.stdout.strip()}")


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


def summarize_trial(writer: csv.writer, node: CalibrationNode, test_type: str,
                    byte_value: int, repetition: int, start: float, result: str) -> None:
    samples = [(t - start, v) for t, v in node.snapshot_samples() if t >= start]
    if test_type == "byte5_acceleration":
        onset = next((t for t, v in samples if v >= 0.005), math.nan)
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
        summarize_trial(summary_writer, node, test_type, byte_value, repetition,
                        start, "acceleration_timeout")
        return "acceleration_timeout"

    node.set_command(TARGET_SPEED, 0.0, "hold")
    time.sleep(HOLD_DURATION)
    stop_start = time.monotonic()
    stopped = stop_vehicle(node)
    result = "success" if stopped else "stop_timeout"
    analysis_start = start if test_type == "byte5_acceleration" else stop_start
    summarize_trial(summary_writer, node, test_type, byte_value, repetition,
                    analysis_start, result)
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
        stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    can_log = open(result_dir / "can_201.log", "w", encoding="utf-8")
    candump = subprocess.Popen(
        ["candump", "-t", "a", f"{CAN_INTERFACE},201:7FF"],
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


def main() -> int:
    result_dir = RESULT_ROOT / time.strftime("%Y%m%d_%H%M%S")
    result_dir.mkdir(parents=True)
    print(f"结果目录：{result_dir}")
    print("封闭道路测试，目标速度0.5m/s。确认急停可用、前方净空充足。")
    if input("输入 RUN 开始：").strip() != "RUN":
        return 1

    for executable in ("ros2", "candump", "ip"):
        if subprocess.call(["which", executable], stdout=subprocess.DEVNULL) != 0:
            raise RuntimeError(f"missing executable: {executable}")
    run_command(["ip", "link", "show", CAN_INTERFACE])
    dynamic_mode = run_command(
        ["ros2", "param", "get", CAN_NODE, "use_dynamic_acceleration_steps"]
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
        run_command(["ros2", "param", "dump", CAN_NODE], output=stream)

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

    rclpy.init()
    node = CalibrationNode(events_writer, events_file)
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    recorders: list[subprocess.Popen] = []
    try:
        time.sleep(1.0)
        if len(node.get_publishers_info_by_topic("/control/command/control_cmd")) > 1:
            raise RuntimeError("another control_cmd publisher is active; stop it before calibration")
        node.publish_drive_gear()
        if not wait_stable(node, lambda value: abs(value) <= SPEED_TOLERANCE,
                           5.0, "initial_zero"):
            raise RuntimeError("vehicle is not stationary")
        recorders = start_recorders(result_dir)
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
                    answer = input("车辆复位并确认安全后输入 NEXT；输入 ABORT 结束：").strip()
                    if answer != "NEXT":
                        raise KeyboardInterrupt

        return_code = 0
    except KeyboardInterrupt:
        print("标定已中止，正在安全停车并保存已有数据。")
        return_code = 130
    except Exception as error:
        print(f"标定失败：{error}", file=sys.stderr)
        node.event("fatal", str(error))
        return_code = 2
    finally:
        node.set_command(0.0, DECELERATION_LABEL, "final_stop")
        time.sleep(2.0)
        stop_recorders(recorders)
        can_log_path = result_dir / "can_201.log"
        if can_log_path.exists():
            analyze_can_periods(can_log_path, result_dir / "can_period_stats.txt")
        set_can_steps(10, 10)
        with open(result_dir / "can_node_params_after.yaml", "w", encoding="utf-8") as stream:
            run_command(["ros2", "param", "dump", CAN_NODE], output=stream, check=False)
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=2.0)
        events_file.close()
        summary_file.close()
        print(f"数据已保存：{result_dir}")
    return return_code


if __name__ == "__main__":
    sys.exit(main())
