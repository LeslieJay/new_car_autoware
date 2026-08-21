#!/usr/bin/env python3
"""Supplement Byte6 calibration and merge it with an existing initial result."""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import statistics
import sys
import threading
import time

import rclpy

import run_can_initial_full_calibration as calibration


BYTE6_SCAN = (4, 5, 6, 7, 8, 9, 10, 12, 15, 20, 30)
MINIMUM_SATURATION_CHECK_BYTE = 10
RESULT_ROOT = Path(os.environ.get(
    "RESULT_ROOT",
    str(calibration.WORKSPACE / "log" / "can_deceleration_supplement"),
))


def load_mapping(path: Path, byte_column: str = "byte") -> list[tuple[int, float]]:
    if not path.exists():
        raise FileNotFoundError(path)
    groups = {}
    with path.open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("result") != "success":
                continue
            byte = int(row[byte_column])
            value = float(row["measured_acceleration_mps2"])
            if math.isfinite(value):
                groups.setdefault(byte, []).append(value)
    return sorted(
        (byte, statistics.median(values)) for byte, values in groups.items()
    )


def should_stop(byte: int, medians: list[float]) -> str | None:
    if medians and medians[-1] >= 1.0:
        return "target_1mps2_reached"
    if byte < MINIMUM_SATURATION_CHECK_BYTE or len(medians) < 3:
        return None
    recent = medians[-3:]
    if not all(math.isfinite(value) and value > 0.0 for value in recent):
        return None
    gains = [
        (recent[index] - recent[index - 1]) / recent[index - 1]
        for index in (1, 2)
    ]
    if all(gain < calibration.SATURATION_GAIN_THRESHOLD for gain in gains):
        return "two_consecutive_gains_below_10_percent_after_byte10"
    return None


def write_combined_summary(path: Path, byte5_mapping, byte6_mapping) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["mapping", "byte", "measured_acceleration_median_mps2"])
        writer.writerows(("byte5", byte, value) for byte, value in byte5_mapping)
        writer.writerows(("byte6", byte, value) for byte, value in byte6_mapping)


def run(prior_result: Path) -> int:
    prior_result = prior_result.resolve()
    byte5_mapping = load_mapping(prior_result / "byte5_acceleration_mapping.csv")
    old_byte6_mapping = load_mapping(prior_result / "byte6_deceleration_mapping.csv")
    if not byte5_mapping:
        raise RuntimeError("prior result contains no usable Byte5 mapping")
    if not old_byte6_mapping:
        raise RuntimeError("prior result contains no usable Byte6 mapping")

    result_dir = RESULT_ROOT / time.strftime("%Y%m%d_%H%M%S")
    result_dir.mkdir(parents=True)
    print(f"已有结果：{prior_result}")
    print(f"补测结果：{result_dir}")
    print(f"Byte6补测序列：{BYTE6_SCAN}")
    print("本脚本只补测减速，不重复速度和Byte5测试。")
    calibration.enter_to_continue(
        "确认至少20m封闭直线、急停可用后按 Enter 开始（Ctrl-C中止）："
    )

    for executable in ("ros2", "candump", "ip"):
        if calibration.run_command(["which", executable], check=False).returncode != 0:
            raise RuntimeError(f"missing executable: {executable}")
    calibration.run_command(["ip", "link", "show", calibration.CAN_INTERFACE])
    calibration.require_fixed_mode()
    with (result_dir / "environment.txt").open("w", encoding="utf-8") as stream:
        stream.write(f"start_time={time.strftime('%Y-%m-%dT%H:%M:%S%z')}\n")
        stream.write(f"prior_result={prior_result}\n")
        stream.write(f"byte6_scan={BYTE6_SCAN}\n")
        stream.write(f"repetitions={calibration.REPETITIONS}\n")
        stream.write(f"target_speed={calibration.TARGET_SPEED}\n")
    with (result_dir / "can_node_params_before.yaml").open(
        "w", encoding="utf-8"
    ) as stream:
        calibration.run_command([
            "ros2", "param", "dump", *calibration.ROS2_PARAM_OPTIONS,
            calibration.CAN_NODE,
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
    node = calibration.CalibrationNode(
        events_writer, events_file, samples_writer, samples_file
    )
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    recorders = []
    return_code = 0
    try:
        time.sleep(1.0)
        if len(node.get_publishers_info_by_topic("/control/command/control_cmd")) > 1:
            raise RuntimeError("another control_cmd publisher is active")
        node.publish_drive_gear()
        if not calibration.ensure_zero(node):
            raise RuntimeError("vehicle is not stationary")
        recorders = calibration.start_recorders(result_dir)
        time.sleep(2.0)
        if (recorders[1].poll() is not None or
                (result_dir / "can_201.log").stat().st_size == 0):
            raise RuntimeError(
                f"no 0x201 frames captured on {calibration.CAN_INTERFACE}"
            )

        raw_path = result_dir / "byte6_deceleration_supplement.csv"
        raw_file = raw_path.open("w", newline="", encoding="utf-8")
        fieldnames = [
            "byte", "repetition", "result", "measured_acceleration_mps2",
            "response_delay_s", "duration_s", "minimum_velocity_mps",
        ]
        writer = csv.DictWriter(raw_file, fieldnames=fieldnames)
        writer.writeheader()
        new_mapping = []
        medians = [value for _, value in old_byte6_mapping]
        for byte in BYTE6_SCAN:
            trials = []
            for repetition in range(1, calibration.REPETITIONS + 1):
                result = calibration.run_deceleration_trial(node, byte, repetition)
                trials.append(result)
                writer.writerow({"byte": byte, "repetition": repetition, **result})
                raw_file.flush()
            median = calibration.median_success(
                trials, "measured_acceleration_mps2"
            )
            new_mapping.append((byte, median))
            medians.append(median)
            print(f"Byte6={byte}：减速度中位数={median:.4f}m/s²")
            reason = should_stop(byte, medians)
            if reason:
                print(f"自适应停止：{reason}")
                break
            if byte != BYTE6_SCAN[-1]:
                calibration.enter_to_continue(
                    "车辆复位并确认安全后按 Enter 继续；Ctrl-C中止："
                )
        raw_file.close()

        combined_byte6 = sorted({
            byte: value for byte, value in old_byte6_mapping + new_mapping
            if math.isfinite(value)
        }.items())
        write_combined_summary(
            result_dir / "byte_mapping_summary_combined.csv",
            byte5_mapping, combined_byte6,
        )
        acceleration_lookup = calibration.build_lookup(byte5_mapping)
        deceleration_lookup = calibration.build_lookup(combined_byte6)
        acceleration_max = max(value for _, value in byte5_mapping)
        deceleration_max = max(value for _, value in combined_byte6)
        calibration.write_lookup(
            result_dir / "initial_lookup_table_updated.yaml",
            acceleration_lookup, deceleration_lookup,
            acceleration_max, deceleration_max,
        )

        calibration.enter_to_continue(
            "更新查表已生成，确认安全后按 Enter 开始交叉验证："
        )
        with (result_dir / "cross_validation_updated.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            cross_writer = csv.writer(stream)
            cross_writer.writerow([
                "target_speed_mps", "ramp_rate_mps2", "byte5", "byte6",
                "repetition", "result", "ramp_up_rmse_mps",
                "ramp_down_rmse_mps", "ramp_up_max_error_mps",
                "ramp_down_max_error_mps",
            ])
            calibration.run_cross_validation(
                node, cross_writer, acceleration_lookup, deceleration_lookup
            )
    except KeyboardInterrupt:
        print("补测已中止，正在停车并保存已有数据。")
        return_code = 130
    except Exception as error:
        print(f"补测失败：{type(error).__name__}: {error}", file=sys.stderr)
        node.event("fatal", str(error))
        return_code = 2
    finally:
        try:
            node.set_command(0.0, calibration.DECELERATION_LABEL, "final_stop")
            time.sleep(2.0)
            calibration.stop_recorders(recorders)
            can_log = result_dir / "can_201.log"
            if can_log.exists():
                calibration.analyze_can_periods(
                    can_log, result_dir / "can_period_stats.txt"
                )
            try:
                calibration.set_can_steps(node, 10, 10)
            except Exception as error:
                print(f"警告：恢复Byte5/Byte6=10失败：{error}", file=sys.stderr)
                return_code = return_code or 2
            with (result_dir / "can_node_params_after.yaml").open(
                "w", encoding="utf-8"
            ) as stream:
                calibration.run_command([
                    "ros2", "param", "dump", *calibration.ROS2_PARAM_OPTIONS,
                    calibration.CAN_NODE,
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
    parser.add_argument(
        "prior_result", type=Path,
        help="existing result containing Byte5 and Byte6 mapping CSV files",
    )
    args = parser.parse_args()
    return run(args.prior_result)


if __name__ == "__main__":
    sys.exit(main())
