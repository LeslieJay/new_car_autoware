#!/usr/bin/env python3
"""Analyze a forward speed-link run and emit reviewable candidate YAML."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import statistics
import sys


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def finite(values):
    return [float(value) for value in values if value not in ("", "nan", "NaN")
            and math.isfinite(float(value))]


def robust_group(rows: list[dict[str, str]], key: str, field: str) -> list[dict]:
    output = []
    keys = sorted({row[key] for row in rows if row.get(key)})
    for group_key in keys:
        values = finite([row[field] for row in rows if row.get(key) == group_key])
        if not values:
            continue
        median = statistics.median(values)
        accepted = [value for value in values if abs(value - median) <= max(abs(median) * 0.30, 0.01)]
        output.append({
            "key": group_key, "median": statistics.median(accepted), "accepted": accepted,
            "outliers": len(values) - len(accepted),
        })
    return output


def parse_can_log(path: Path) -> dict[str, list[tuple[float, list[int]]]]:
    """Parse candump -t a lines for 0x201 and 0x181 frames."""
    result = {"201": [], "181": []}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line.startswith("(") or ")" not in line:
            continue
        try:
            timestamp = float(line[1:line.index(")")])
            frame_token = next(token for token in line.split() if "#" in token)
            can_id, payload = frame_token.split("#", 1)
            payload = payload.replace(" ", "")
            can_id = can_id.lower()
            frame_key = can_id[-3:]
            if frame_key not in result or len(payload) < 4:
                continue
            result[frame_key].append((timestamp, [int(payload[i:i + 2], 16)
                                                  for i in range(0, len(payload), 2)]))
        except (ValueError, IndexError):
            continue
    return result


def parse_driver_tx_log(path: Path) -> dict[str, list[tuple[float, list[int]]]]:
    result = {"201": [], "181": []}
    if not path.exists():
        return result
    with path.open(encoding="utf-8", errors="replace", newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                frame_key = row["can_id"].lower().replace("0x", "")[-3:]
                if frame_key not in result:
                    continue
                payload = [int(value, 16) for value in row["data"].split()]
                result[frame_key].append((float(row["timestamp_epoch_s"]), payload))
            except (KeyError, TypeError, ValueError):
                continue
    return result


def nearest_command(timestamp: float, frames: list[tuple[float, list[int]]]):
    candidates = [frame for frame in frames if abs(frame[0] - timestamp) <= 0.10]
    if not candidates:
        return math.nan, math.nan, math.nan
    _, data = min(candidates, key=lambda item: abs(item[0] - timestamp))
    speed_count = int.from_bytes(bytes(data[0:2]), byteorder="little", signed=True)
    byte5 = data[5] if len(data) > 5 else math.nan
    byte6 = data[6] if len(data) > 6 else math.nan
    return float(speed_count), float(byte5), float(byte6)


def write_yaml(path: Path, speed_groups: list[dict], accel_groups: list[dict], decel_groups: list[dict]):
    with path.open("w", encoding="utf-8") as stream:
        stream.write("# Candidate only: review and copy into can_params.yaml after approval.\n")
        if speed_groups:
            stream.write("speed_command_breakpoints_mps: [" + ", ".join(
                group["key"] for group in speed_groups) + "]\n")
            stream.write("speed_command_values: [" + ", ".join(
                str(int(round(group["median_command"]))) for group in speed_groups) + "]\n")
        else:
            stream.write("speed_mapping: not_ready\n")
        if accel_groups:
            stream.write("acceleration_breakpoints_mps2: [" + ", ".join(
                group["key"] for group in accel_groups) + "]\n")
            stream.write("acceleration_step_values: [" + ", ".join(
                str(int(round(group["median_byte"]))) for group in accel_groups) + "]\n")
        else:
            stream.write("acceleration_mapping: not_ready\n")
        if decel_groups:
            stream.write("deceleration_breakpoints_mps2: [" + ", ".join(
                group["key"] for group in decel_groups) + "]\n")
            stream.write("deceleration_step_values: [" + ", ".join(
                str(int(round(group["median_byte"]))) for group in decel_groups) + "]\n")
        else:
            stream.write("deceleration_mapping: not_ready\n")


def estimate_step_delays(rows: list[dict[str, str]]) -> list[dict[str, float]]:
    delays = []
    trials = sorted({(row.get("trial"), row.get("repetition")) for row in rows})
    for trial, repetition in trials:
        if not trial or not trial.startswith("steady_"):
            continue
        trial_rows = [row for row in rows if row.get("trial") == trial and
                      row.get("repetition") == repetition]
        trial_rows.sort(key=lambda row: float(row["timestamp_epoch_s"]))
        try:
            target = float(trial.split("_", 1)[1])
            start_index = next(index for index, row in enumerate(trial_rows)
                               if row.get("phase") == "steady_speed")
            start_time = float(trial_rows[start_index]["timestamp_epoch_s"])
            pre = finite([row["rtk_velocity_mps"] for row in trial_rows[:start_index]])
            steady_rows = []
            for row in trial_rows[start_index:]:
                if row.get("phase") != "steady_speed":
                    break
                steady_rows.append(row)
            post = finite([row["rtk_velocity_mps"] for row in steady_rows])
            if len(pre) < 3 or len(post) < 3:
                continue
            baseline = statistics.median(pre[-min(25, len(pre)):])
            final = statistics.median(post[-min(50, len(post)):])
            rise = baseline + 0.10 * (final - baseline)
            response = next(
                (float(row["timestamp_epoch_s"]) for row in steady_rows
                 if float(row["rtk_velocity_mps"]) >= rise), None)
            if response is not None and final > baseline + 0.02:
                delays.append({"trial": trial, "repetition": float(repetition),
                               "delay_s": max(0.0, response - start_time),
                               "target_mps": target})
        except (KeyError, ValueError, StopIteration):
            continue
    return delays


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", type=Path)
    args = parser.parse_args()
    samples_path = args.result_dir / "control_samples.csv"
    if not samples_path.exists():
        print(f"missing {samples_path}", file=sys.stderr)
        return 2
    rows = read_rows(samples_path)
    tx_frames = parse_driver_tx_log(args.result_dir / "can_driver_tx.csv")
    raw_frames = parse_can_log(args.result_dir / "can_201_181.log")
    frames = tx_frames["201"] or raw_frames["201"]
    for row in rows:
        try:
            speed, byte5, byte6 = nearest_command(float(row["timestamp_epoch_s"]), frames)
            row["can_command_count"] = str(speed)
            row["byte5"] = str(byte5)
            row["byte6"] = str(byte6)
        except (KeyError, ValueError):
            row["can_command_count"] = row["byte5"] = row["byte6"] = "nan"

    steady = [row for row in rows if row.get("phase") == "steady_speed"]
    speed_groups = []
    for group in robust_group(steady, "command_velocity_mps", "rtk_velocity_mps"):
        matching = [row for row in steady if row["command_velocity_mps"] == group["key"]]
        commands = finite([row["can_command_count"] for row in matching])
        group["median_command"] = statistics.median(commands) if commands else math.nan
        group["rtk_error_mps"] = group["median"] - float(group["key"])
        speed_groups.append(group)

    def dynamic_groups(phase: str, byte_field: str):
        groups = []
        for rate in sorted({row["command_acceleration_mps2"] for row in rows
                            if row.get("phase") == phase}):
            selected = [row for row in rows if row.get("phase") == phase and
                        row.get("command_acceleration_mps2") == rate]
            byte_values = finite([row[byte_field] for row in selected])
            rtk_values = finite([row["rtk_velocity_mps"] for row in selected])
            if len(byte_values) < 3 or len(rtk_values) < 3:
                continue
            groups.append({"key": rate, "median_byte": statistics.median(byte_values),
                           "median_rtk": statistics.median(rtk_values)})
        return groups

    accel_groups = dynamic_groups("ramp_acceleration", "byte5")
    decel_groups = dynamic_groups("ramp_deceleration", "byte6")
    write_yaml(args.result_dir / "calibration_candidates.yaml", speed_groups,
               accel_groups, decel_groups)

    delays = estimate_step_delays(rows)
    with (args.result_dir / "response_delay_summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=["trial", "repetition", "target_mps", "delay_s"])
        writer.writeheader()
        writer.writerows(delays)
    if delays:
        median_delay = statistics.median(row["delay_s"] for row in delays)
        with (args.result_dir / "delay_candidate.yaml").open("w", encoding="utf-8") as stream:
            stream.write("# Candidate only; verify with gate-output timestamps before applying.\n")
            stream.write(f"delay_compensation_time: {median_delay:.6f}\n")

    with (args.result_dir / "speed_link_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["target_mps", "median_rtk_mps", "steady_error_mps",
                         "median_0x201_count", "outliers"])
        for group in speed_groups:
            writer.writerow([group["key"], f"{group['median']:.6f}",
                             f"{group['rtk_error_mps']:.6f}",
                             f"{group['median_command']:.1f}", group["outliers"]])
    print(f"wrote {args.result_dir / 'calibration_candidates.yaml'}")
    for group in speed_groups:
        print(f"speed {float(group['key']):.2f} m/s: RTK error {group['rtk_error_mps']:+.4f} m/s")
    if delays:
        print(f"effective 10% response delay: {statistics.median(row['delay_s'] for row in delays):.3f}s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
