#!/usr/bin/env python3
"""Summarize byd_vehicle_state endpoint errors from ROS text logs."""

from __future__ import annotations

import argparse
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path


LINE_RE = re.compile(
    r"(?P<event>Arrival condition met|Goal arrived|Arrival condition lost):\s*"
    r"goal=\((?P<gx>[+-]?[\d.]+),\s*(?P<gy>[+-]?[\d.]+)\)\s*"
    r"pose=\((?P<px>[+-]?[\d.]+),\s*(?P<py>[+-]?[\d.]+)\)\s*"
    r"longitudinal=(?P<long>[+-]?[\d.]+)m\s*"
    r"lateral=(?P<lat>[+-]?[\d.]+)m\s*"
    r"distance=(?P<distance>[+-]?[\d.]+)m\s*"
    r"yaw=(?P<yaw>[+-]?[\d.]+)deg\s*speed=(?P<speed>[+-]?[\d.]+)m/s"
)


@dataclass(frozen=True)
class Sample:
    source: str
    line: int
    event: str
    gx: float
    gy: float
    px: float
    py: float
    longitudinal: float
    lateral: float
    distance: float
    yaw_deg: float
    speed: float

    @property
    def dx(self) -> float:
        return self.px - self.gx

    @property
    def dy(self) -> float:
        return self.py - self.gy


def parse_logs(paths: list[Path], event: str, speed_limit: float) -> tuple[list[Sample], int]:
    samples: list[Sample] = []
    rejected_speed = 0
    for path in paths:
        text = path.read_text(encoding="utf-8", errors="replace")
        for line_number, line in enumerate(text.splitlines(), 1):
            match = LINE_RE.search(line)
            if not match or match.group("event") != event:
                continue
            values = match.groupdict()
            sample = Sample(
                source=str(path), line=line_number, event=values["event"],
                gx=float(values["gx"]), gy=float(values["gy"]),
                px=float(values["px"]), py=float(values["py"]),
                longitudinal=float(values["long"]), lateral=float(values["lat"]),
                distance=float(values["distance"]), yaw_deg=float(values["yaw"]),
                speed=float(values["speed"]),
            )
            if abs(sample.speed) > speed_limit:
                rejected_speed += 1
                continue
            samples.append(sample)
    return samples, rejected_speed


def mean_std(values: list[float]) -> tuple[float, float]:
    return statistics.mean(values), statistics.stdev(values) if len(values) > 1 else 0.0


def print_report(samples: list[Sample], rejected_speed: int, speed_limit: float) -> None:
    print("Endpoint calibration report")
    print(f"accepted={len(samples)} rejected_by_speed={rejected_speed} speed_limit={speed_limit:.3f}m/s")
    print("\n#  dx(cm)  dy(cm)  longitudinal(cm)  lateral(cm)  distance(cm)  yaw(deg)  speed(m/s)")
    for i, s in enumerate(samples, 1):
        print(
            f"{i:2d} {s.dx * 100:+7.2f} {s.dy * 100:+7.2f} "
            f"{s.longitudinal * 100:+16.2f} {s.lateral * 100:+12.2f} "
            f"{s.distance * 100:12.2f} {s.yaw_deg:+9.2f} {s.speed:10.4f}"
        )

    print("\nmetric              mean       std        rms        max_abs")
    metrics = {
        "dx_cm": [s.dx * 100 for s in samples],
        "dy_cm": [s.dy * 100 for s in samples],
        "longitudinal_cm": [s.longitudinal * 100 for s in samples],
        "lateral_cm": [s.lateral * 100 for s in samples],
        "yaw_deg": [s.yaw_deg for s in samples],
        "distance_cm": [s.distance * 100 for s in samples],
    }
    for name, values in metrics.items():
        mean, std = mean_std(values)
        rms = math.sqrt(statistics.mean(v * v for v in values))
        print(f"{name:18s} {mean:+9.3f} {std:10.3f} {rms:10.3f} {max(abs(v) for v in values):12.3f}")

    long_mean, long_std = mean_std(metrics["longitudinal_cm"])
    lat_mean, lat_std = mean_std(metrics["lateral_cm"])
    yaw_mean, yaw_std = mean_std(metrics["yaw_deg"])
    print("\nCalibration interpretation (verify with opposite headings before applying):")
    print(f"- fixed vehicle-frame longitudinal bias candidate: {long_mean:+.2f} cm (std {long_std:.2f})")
    print(f"- fixed vehicle-frame lateral bias candidate:      {lat_mean:+.2f} cm (std {lat_std:.2f})")
    print(f"- fixed yaw bias candidate:                        {yaw_mean:+.2f} deg (std {yaw_std:.2f})")
    if len(samples) < 10:
        print("- WARNING: fewer than 10 accepted arrivals; collect more repeats before calibration.")
    if lat_std > max(2.0, abs(lat_mean) * 0.5):
        print("- Lateral error is not sufficiently constant; inspect trajectory tracking before static offset calibration.")
    if yaw_std > max(0.5, abs(yaw_mean) * 0.5):
        print("- Yaw error is not sufficiently constant; do not treat it as an IMU mounting bias yet.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="ROS console log file(s)")
    parser.add_argument("--event", default="Goal arrived", choices=(
        "Arrival condition met", "Goal arrived", "Arrival condition lost"))
    parser.add_argument("--max-speed", type=float, default=0.05, help="maximum accepted speed [m/s]")
    args = parser.parse_args()
    samples, rejected = parse_logs(args.logs, args.event, args.max_speed)
    if not samples:
        parser.error(f"no '{args.event}' samples found at speed <= {args.max_speed} m/s")
    print_report(samples, rejected, args.max_speed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
