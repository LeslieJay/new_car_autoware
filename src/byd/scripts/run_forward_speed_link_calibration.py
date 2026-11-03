#!/usr/bin/env python3
"""Collect a forward 0--1 m/s CAN/RTK speed-link calibration dataset.

This tool deliberately produces candidate files only.  It does not modify the
vehicle YAML configuration.  Run it with only ``can_node`` publishing the
vehicle command topic, in a closed test area with a physical E-stop.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import shutil
import statistics
import subprocess
import sys
import threading
import time

import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import GearCommand, VelocityReport
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.node import Node


CONTROL_PERIOD = 0.02
SPEED_POINTS = tuple(round(i / 10.0, 1) for i in range(1, 11))
RAMP_RATES = (0.05, 0.10, 0.20, 0.30, 0.50)
REPETITIONS = 3
FEEDBACK_TIMEOUT = 0.5
STOP_TIMEOUT = 20.0
CAN_INTERFACE = os.environ.get("CAN_INTERFACE", "can0")
WORKSPACE = Path(os.environ.get("AUTOWARE_WORKSPACE", Path.cwd()))
RESULT_ROOT = Path(os.environ.get(
    "RESULT_ROOT", str(WORKSPACE / "log" / "forward_speed_link_calibration")
))


class CalibrationNode(Node):
    def __init__(self, samples: csv.writer, sample_file, events: csv.writer, event_file,
                 command_topic: str, gear_topic: str):
        super().__init__("forward_speed_link_calibration")
        self._lock = threading.Lock()
        self._target = 0.0
        self._acceleration = 0.0
        self._defined_acceleration = False
        self._phase = "initializing"
        self._trial = "none"
        self._repetition = 0
        self._ramp_start: float | None = None
        self._ramp_end = 0.0
        self._ramp_rate = 0.0
        self._can_velocity = math.nan
        self._rtk_velocity = math.nan
        self._can_time = 0.0
        self._rtk_time = 0.0
        self._samples = samples
        self._sample_file = sample_file
        self._events = events
        self._event_file = event_file

        self.command_topic = command_topic
        self.gear_topic = gear_topic
        self.control_pub = self.create_publisher(Control, command_topic, 10)
        self.gear_pub = self.create_publisher(GearCommand, gear_topic, 10)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status", self._on_can, 50)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "/sensing/gnss/rtk/velocity_twist",
            self._on_rtk,
            50,
        )
        self.create_timer(CONTROL_PERIOD, self._publish)

    def _on_can(self, msg: VelocityReport) -> None:
        with self._lock:
            self._can_velocity = float(msg.longitudinal_velocity)
            self._can_time = time.monotonic()

    def _on_rtk(self, msg: TwistWithCovarianceStamped) -> None:
        with self._lock:
            self._rtk_velocity = float(msg.twist.twist.linear.x)
            self._rtk_time = time.monotonic()

    def _publish(self) -> None:
        now = time.time()
        monotonic_now = time.monotonic()
        with self._lock:
            if self._ramp_start is not None:
                elapsed = max(0.0, monotonic_now - self._ramp_start)
                self._target = min(self._ramp_end, self._ramp_rate * elapsed)
            target = self._target
            acceleration = self._acceleration
            defined = self._defined_acceleration
            phase = self._phase
            row = [
                f"{now:.9f}", self._trial, self._repetition, phase,
                f"{target:.6f}", f"{acceleration:.6f}",
                f"{self._can_velocity:.6f}", f"{self._rtk_velocity:.6f}",
            ]
        msg = Control()
        msg.stamp = self.get_clock().now().to_msg()
        msg.longitudinal.velocity = target
        msg.longitudinal.acceleration = acceleration
        msg.longitudinal.is_defined_acceleration = defined
        msg.longitudinal.jerk = 0.0
        msg.longitudinal.is_defined_jerk = False
        msg.lateral.steering_tire_angle = 0.0
        msg.lateral.steering_tire_rotation_rate = 0.0
        msg.lateral.is_defined_steering_tire_rotation_rate = False
        self.control_pub.publish(msg)
        self._samples.writerow(row)
        self._sample_file.flush()

    def set_trial(self, name: str, repetition: int) -> None:
        with self._lock:
            self._trial = name
            self._repetition = repetition

    def set_command(self, target: float, acceleration: float, phase: str,
                    defined_acceleration: bool = False) -> float:
        with self._lock:
            self._ramp_start = None
            self._target = target
            self._acceleration = acceleration
            self._defined_acceleration = defined_acceleration
            self._phase = phase
        self._event("phase_start", phase)
        return time.monotonic()

    def start_ramp(self, end: float, rate: float, phase: str) -> float:
        with self._lock:
            self._ramp_start = time.monotonic()
            self._ramp_end = end
            self._ramp_rate = rate
            self._target = 0.0 if rate > 0.0 else end
            self._acceleration = rate
            self._defined_acceleration = True
            self._phase = phase
        self._event("phase_start", phase)
        return self._ramp_start

    def feedback(self) -> tuple[float, float, float]:
        with self._lock:
            now = time.monotonic()
            return self._can_velocity, now - self._can_time, self._rtk_velocity

    def _event(self, event: str, result: str) -> None:
        with self._lock:
            row = [
                f"{time.time():.9f}", self._trial, self._repetition, event,
                self._phase, f"{self._target:.6f}", f"{self._acceleration:.6f}", result,
            ]
        self._events.writerow(row)
        self._event_file.flush()


def run_command(command: list[str], *, check: bool = True, timeout: float = 20.0,
                output=None) -> subprocess.CompletedProcess:
    return subprocess.run(
        command, check=check, timeout=timeout, text=True,
        stdout=output or subprocess.PIPE, stderr=subprocess.STDOUT,
    )


def wait_for_can(node: CalibrationNode, predicate, timeout: float, label: str) -> bool:
    deadline = time.monotonic() + timeout
    stable_since: float | None = None
    while time.monotonic() < deadline:
        can_velocity, age, _ = node.feedback()
        if age > FEEDBACK_TIMEOUT:
            raise RuntimeError(f"CAN feedback lost during {label}: age={age:.3f}s")
        if predicate(can_velocity):
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 1.0:
                return True
        else:
            stable_since = None
        time.sleep(CONTROL_PERIOD)
    return False


def wait_for_rtk(node: CalibrationNode, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _, _, rtk_velocity = node.feedback()
        if math.isfinite(rtk_velocity):
            return
        time.sleep(CONTROL_PERIOD)
    raise RuntimeError("RTK velocity feedback is unavailable; refusing an ungrounded calibration")


def start_recorders(result_dir: Path):
    topics = [
        "/planning/trajectory", "/control/trajectory_follower/control_cmd",
        "/control/command/control_cmd", "/control/command/gear_cmd",
        "/can_driver/debug/control_cmd_rx", "/can_driver/debug/control_cmd_can",
        "/vehicle/status/velocity_status", "/vehicle/status/steering_status",
        "/localization/kinematic_state", "/localization/acceleration",
        "/sensing/gnss/rtk/velocity_twist",
    ]
    recorder_log = (result_dir / "recorder.log").open("w", encoding="utf-8")
    bag = subprocess.Popen(
        ["ros2", "bag", "record", "-o", str(result_dir / "rosbag"), *topics],
        stdin=subprocess.DEVNULL, stdout=recorder_log, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    can_log = (result_dir / "can_201_181.log").open("w", encoding="utf-8")
    candump = subprocess.Popen(
        ["candump", "-t", "a", f"{CAN_INTERFACE},201:7FF",
         f"{CAN_INTERFACE},181:7FF"],
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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", action="store_true",
                        help="confirm closed area, safety observer and physical E-stop")
    parser.add_argument("--result-root", type=Path, default=RESULT_ROOT)
    parser.add_argument("--repetitions", type=int, default=REPETITIONS)
    parser.add_argument("--can-tx-log", type=Path, default=Path("/tmp/can_driver_tx.csv"),
                        help="driver write-timestamp CSV to copy into the result directory")
    parser.add_argument(
        "--command-topic", choices=("/control/command/control_cmd", "/external/selected/control_cmd"),
        default="/control/command/control_cmd",
        help="publish direct to can_node or through the vehicle command gate",
    )
    args = parser.parse_args()
    if not args.arm:
        parser.error("refusing to move the vehicle without --arm")
    if not 1 <= args.repetitions <= 10:
        parser.error("--repetitions must be in [1, 10]")

    for executable in ("ros2", "candump", "ip"):
        if run_command(["which", executable], check=False).returncode != 0:
            raise RuntimeError(f"missing executable: {executable}")
    run_command(["ip", "link", "show", CAN_INTERFACE])
    result_dir = args.result_root / time.strftime("%Y%m%d_%H%M%S")
    result_dir.mkdir(parents=True, exist_ok=False)
    (result_dir / "environment.json").write_text(json.dumps({
        "started_at": time.time(), "can_interface": CAN_INTERFACE,
        "speed_points_mps": SPEED_POINTS, "ramp_rates_mps2": RAMP_RATES,
        "repetitions": args.repetitions, "ground_truth": "RTK velocity_twist",
        "command_topic": args.command_topic,
        "can_tx_log": str(args.can_tx_log),
    }, indent=2), encoding="utf-8")

    samples_file = (result_dir / "control_samples.csv").open("w", newline="", encoding="utf-8")
    events_file = (result_dir / "events.csv").open("w", newline="", encoding="utf-8")
    samples = csv.writer(samples_file)
    events = csv.writer(events_file)
    samples.writerow(["timestamp_epoch_s", "trial", "repetition", "phase",
                      "command_velocity_mps", "command_acceleration_mps2",
                      "can_velocity_mps", "rtk_velocity_mps"])
    events.writerow(["timestamp_epoch_s", "trial", "repetition", "event",
                     "phase", "command_velocity_mps", "command_acceleration_mps2", "result"])

    rclpy.init()
    gear_topic = ("/external/selected/gear_cmd" if
                  args.command_topic == "/external/selected/control_cmd" else
                  "/control/command/gear_cmd")
    node = CalibrationNode(samples, samples_file, events, events_file,
                           args.command_topic, gear_topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    recorders = []
    try:
        time.sleep(1.0)
        publishers = node.get_publishers_info_by_topic(args.command_topic)
        if len(publishers) > 1:
            raise RuntimeError(f"another publisher is active on {args.command_topic}")
        wait_for_rtk(node)
        gear = GearCommand()
        gear.command = GearCommand.DRIVE
        for _ in range(3):
            node.gear_pub.publish(gear)
            time.sleep(0.1)
        recorders = start_recorders(result_dir)
        time.sleep(2.0)

        for speed in SPEED_POINTS:
            for repetition in range(1, args.repetitions + 1):
                node.set_trial(f"steady_{speed:.1f}", repetition)
                node.set_command(0.0, 0.0, "zero", False)
                if not wait_for_can(node, lambda value: abs(value) <= 0.02, STOP_TIMEOUT, "zero"):
                    raise RuntimeError("vehicle did not reach zero before steady trial")
                node.set_command(speed, 0.0, "steady_speed", False)
                if not wait_for_can(node, lambda value, target=speed:
                                    abs(value - target) <= 0.05, STOP_TIMEOUT, "steady_speed"):
                    raise RuntimeError(f"CAN speed did not settle at {speed:.2f} m/s")
                time.sleep(1.0)
                node.set_command(0.0, -0.2, "zero", True)
                wait_for_can(node, lambda value: abs(value) <= 0.02, STOP_TIMEOUT, "zero")

        for rate in RAMP_RATES:
            for repetition in range(1, args.repetitions + 1):
                node.set_trial(f"ramp_{rate:.2f}", repetition)
                node.set_command(0.0, 0.0, "zero", False)
                if not wait_for_can(node, lambda value: abs(value) <= 0.02, STOP_TIMEOUT, "zero"):
                    raise RuntimeError("vehicle did not reach zero before ramp trial")
                node.start_ramp(1.0, rate, "ramp_acceleration")
                time.sleep(1.0 / rate + 2.0)
                node.set_command(1.0, 0.0, "hold", False)
                time.sleep(2.0)
                node.set_command(0.0, -rate, "ramp_deceleration", True)
                time.sleep(1.0 / rate + 2.0)
                node.set_command(0.0, -0.2, "zero", True)
                wait_for_can(node, lambda value: abs(value) <= 0.02, STOP_TIMEOUT, "zero")
    finally:
        node.set_command(0.0, -0.2, "final_stop", True)
        time.sleep(2.0)
        stop_recorders(recorders)
        if args.can_tx_log.exists():
            shutil.copyfile(args.can_tx_log, result_dir / "can_driver_tx.csv")
        samples_file.close()
        events_file.close()
        rclpy.shutdown()
    print(f"Calibration data written to {result_dir}")
    print("Run analyze_forward_speed_link_calibration.py on this directory to create candidates.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
