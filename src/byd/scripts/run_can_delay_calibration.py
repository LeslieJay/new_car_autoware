#!/usr/bin/env python3
"""Measure effective deceleration delay with only can_node running."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import re
import statistics
import sys
import threading
import time

from geometry_msgs.msg import Twist
import rclpy

import run_can_initial_full_calibration as base


DEFAULT_SPEEDS = (0.3, 0.5)
DEFAULT_BYTE6 = (5, 8)
DEFAULT_REPETITIONS = 3
FIT_MAX_DELAY_S = 0.60
FIT_DELAY_STEP_S = 0.005
RESULT_ROOT = Path(os.environ.get(
    "RESULT_ROOT", str(base.WORKSPACE / "log" / "can_delay_calibration")
))


def fit_effective_delay(
    samples: list[tuple[float, float]], initial_speed: float
) -> dict[str, float]:
    """Fit v(t)=v0-a*max(0,t-delay) over the upper 80% of a stop."""
    usable = [
        (elapsed, velocity)
        for elapsed, velocity in samples
        if elapsed >= 0.0 and math.isfinite(velocity)
        and velocity >= 0.2 * initial_speed
    ]
    if len(usable) < 8 or min(velocity for _, velocity in usable) > 0.8 * initial_speed:
        raise ValueError("insufficient velocity decrease for delay fit")

    best = None
    candidate_count = int(FIT_MAX_DELAY_S / FIT_DELAY_STEP_S) + 1
    for index in range(candidate_count):
        delay = index * FIT_DELAY_STEP_S
        x_values = [max(0.0, elapsed - delay) for elapsed, _ in usable]
        denominator = sum(value * value for value in x_values)
        if denominator <= 0.0:
            continue
        deceleration = sum(
            x_value * (initial_speed - velocity)
            for x_value, (_, velocity) in zip(x_values, usable)
        ) / denominator
        if deceleration <= 0.0:
            continue
        errors = [
            velocity - (initial_speed - deceleration * x_value)
            for x_value, (_, velocity) in zip(x_values, usable)
        ]
        rmse = math.sqrt(statistics.mean(error * error for error in errors))
        candidate = (rmse, delay, deceleration)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise ValueError("unable to fit effective delay")
    return {
        "delay_s": best[1],
        "deceleration_mps2": best[2],
        "rmse_mps": best[0],
    }


def read_integer_parameter(name: str) -> int:
    result = base.run_command([
        "ros2", "param", "get", *base.ROS2_PARAM_OPTIONS, base.CAN_NODE, name,
    ])
    match = re.search(r"(-?\d+)\s*$", result.stdout.strip())
    if not match:
        raise RuntimeError(f"cannot parse {name}: {result.stdout.strip()}")
    return int(match.group(1))


def wait_for_zero(node: base.CalibrationNode) -> bool:
    node.set_command(0.0, 0.0, "zero_speed")
    return base.wait_stable(
        node, lambda velocity: abs(velocity) <= base.SPEED_TOLERANCE,
        base.STOP_TIMEOUT, "zero_speed",
    )


def run(args: argparse.Namespace) -> int:
    for byte in args.byte6:
        if not 1 <= byte <= 255:
            raise ValueError(f"Byte6 out of range: {byte}")
    result_dir = args.result_root / time.strftime("%Y%m%d_%H%M%S")
    result_dir.mkdir(parents=True)
    print(f"结果目录：{result_dir}")
    print(f"初始速度：{tuple(args.speeds)}m/s")
    print(f"Byte6：{tuple(args.byte6)}；每组重复{args.repetitions}次")
    print("只需启动can_node；禁止启动其他control_cmd发布者。")
    base.enter_to_continue(
        "确认封闭直线、急停和安全员就位后按 Enter 开始（Ctrl-C中止）："
    )

    for executable in ("ros2", "candump", "ip"):
        if base.run_command(["which", executable], check=False).returncode != 0:
            raise RuntimeError(f"missing executable: {executable}")
    base.run_command(["ip", "link", "show", base.CAN_INTERFACE])
    original_byte5 = read_integer_parameter("default_acceleration_step_command")
    original_byte6 = read_integer_parameter("default_deceleration_step_command")

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
    trials_file = (result_dir / "delay_trials.csv").open(
        "w", newline="", encoding="utf-8"
    )
    trial_fields = [
        "initial_speed_mps", "byte6", "repetition", "result",
        "effective_delay_s", "command_to_can_s", "fitted_deceleration_mps2",
        "fit_rmse_mps", "stop_duration_s", "initial_measured_speed_mps",
    ]
    trials_writer = csv.DictWriter(trials_file, fieldnames=trial_fields)
    trials_writer.writeheader()

    rclpy.init()
    node = base.CalibrationNode(events_writer, events_file, samples_writer, samples_file)
    can_zero_time = math.nan
    waiting_for_zero = False

    def on_can_debug(message: Twist) -> None:
        nonlocal can_zero_time
        if waiting_for_zero and math.isnan(can_zero_time) and abs(message.linear.x) < 0.5:
            can_zero_time = time.monotonic()

    node.create_subscription(Twist, "/can_driver/debug/control_cmd_can", on_can_debug, 50)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    recorders = []
    rows = []
    return_code = 0
    try:
        time.sleep(1.0)
        publisher_count = len(
            node.get_publishers_info_by_topic("/control/command/control_cmd")
        )
        if publisher_count > 1:
            raise RuntimeError("another control_cmd publisher is active")
        node.publish_drive_gear()
        base.set_can_steps(node, 8, args.byte6[0])
        if not wait_for_zero(node):
            raise RuntimeError("vehicle is not stationary")
        recorders = base.start_recorders(result_dir)
        time.sleep(2.0)

        for speed in args.speeds:
            for byte6 in args.byte6:
                for repetition in range(1, args.repetitions + 1):
                    base.enter_to_continue(
                        f"确认前方安全：v={speed:.2f}m/s Byte6={byte6} "
                        f"第{repetition}次，按 Enter 执行："
                    )
                    base.set_can_steps(node, 8, byte6)
                    node.set_trial("delay_step", 8, byte6, repetition)
                    if not wait_for_zero(node):
                        result = {"result": "precheck_timeout"}
                    else:
                        node.set_command(speed, 0.0, "accelerate_to_initial_speed")
                        reached = base.wait_stable(
                            node,
                            lambda velocity, target=speed:
                                abs(velocity - target) <= base.SPEED_TOLERANCE,
                            base.ACCEL_TIMEOUT,
                            "initial_speed",
                        )
                        if not reached:
                            wait_for_zero(node)
                            result = {"result": "acceleration_timeout"}
                        else:
                            node.set_command(speed, 0.0, "hold_initial_speed")
                            time.sleep(1.0)
                            initial_measured, _ = node.feedback()
                            can_zero_time = math.nan
                            waiting_for_zero = True
                            step_time = time.monotonic()
                            node.set_command(0.0, 0.0, "step_to_zero")
                            stopped = base.wait_stable(
                                node,
                                lambda velocity: abs(velocity) <= base.SPEED_TOLERANCE,
                                base.STOP_TIMEOUT,
                                "stopped",
                            )
                            stop_time = time.monotonic()
                            waiting_for_zero = False
                            fit_samples = [
                                (timestamp - step_time, velocity)
                                for timestamp, velocity in node.snapshot_feedback()
                                if timestamp >= step_time
                            ]
                            try:
                                fit = fit_effective_delay(fit_samples, initial_measured)
                                fit_result = "success" if stopped else "stop_timeout"
                            except ValueError as error:
                                fit = {
                                    "delay_s": math.nan,
                                    "deceleration_mps2": math.nan,
                                    "rmse_mps": math.nan,
                                }
                                fit_result = f"fit_invalid:{error}"
                            result = {
                                "result": fit_result,
                                "effective_delay_s": fit["delay_s"],
                                "command_to_can_s": (
                                    can_zero_time - step_time
                                    if math.isfinite(can_zero_time) else math.nan
                                ),
                                "fitted_deceleration_mps2": fit["deceleration_mps2"],
                                "fit_rmse_mps": fit["rmse_mps"],
                                "stop_duration_s": stop_time - step_time,
                                "initial_measured_speed_mps": initial_measured,
                            }
                    row = {
                        "initial_speed_mps": speed,
                        "byte6": byte6,
                        "repetition": repetition,
                        **result,
                    }
                    rows.append(row)
                    trials_writer.writerow(row)
                    trials_file.flush()
                    print(
                        f"结果：{row['result']}，等效延迟="
                        f"{row.get('effective_delay_s', math.nan):.3f}s"
                    )

        with (result_dir / "delay_summary.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "initial_speed_mps", "byte6", "valid_runs",
                "median_effective_delay_s", "min_delay_s", "max_delay_s",
            ])
            all_delays = []
            for speed in args.speeds:
                for byte6 in args.byte6:
                    delays = [
                        row["effective_delay_s"] for row in rows
                        if row.get("result") == "success"
                        and row["initial_speed_mps"] == speed
                        and row["byte6"] == byte6
                    ]
                    if delays:
                        all_delays.extend(delays)
                        writer.writerow([
                            speed, byte6, len(delays), statistics.median(delays),
                            min(delays), max(delays),
                        ])
            if all_delays:
                recommendation = statistics.median(all_delays)
                writer.writerow(["overall", "all", len(all_delays), recommendation,
                                 min(all_delays), max(all_delays)])
                print(f"初步推荐 delay_compensation_time={recommendation:.3f}s")
            else:
                print("没有有效试验，无法推荐延迟参数。", file=sys.stderr)
                return_code = 2
    except KeyboardInterrupt:
        print("测试已中止，正在停车并保存已有数据。")
        return_code = 130
    except Exception as error:
        print(f"测试失败：{type(error).__name__}: {error}", file=sys.stderr)
        return_code = 2
    finally:
        try:
            node.set_command(0.0, 0.0, "final_stop")
            time.sleep(2.0)
            base.stop_recorders(recorders)
            can_log = result_dir / "can_201.log"
            if can_log.exists():
                base.analyze_can_periods(can_log, result_dir / "can_period_stats.txt")
            try:
                base.set_can_steps(node, original_byte5, original_byte6)
            except Exception as error:
                print(f"警告：恢复原Byte参数失败：{error}", file=sys.stderr)
                return_code = return_code or 2
        finally:
            node.destroy_node()
            rclpy.shutdown()
            spin_thread.join(timeout=2.0)
            events_file.close()
            samples_file.close()
            trials_file.close()
            print(f"数据已保存：{result_dir}")
    return return_code


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speeds", nargs="+", type=float, default=DEFAULT_SPEEDS)
    parser.add_argument("--byte6", nargs="+", type=int, default=DEFAULT_BYTE6)
    parser.add_argument("--repetitions", type=int, default=DEFAULT_REPETITIONS)
    parser.add_argument("--result-root", type=Path, default=RESULT_ROOT)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be at least 1")
    if any(speed <= 0.0 or speed > 1.0 for speed in args.speeds):
        parser.error("--speeds must be in (0, 1.0]")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
