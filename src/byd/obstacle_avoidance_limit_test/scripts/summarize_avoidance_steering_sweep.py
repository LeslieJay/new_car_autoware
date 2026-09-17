#!/usr/bin/env python3
"""Summarize tagged Simple Avoidance parameter-sweep logs into CSV/Markdown."""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path


CANDIDATE_RE = re.compile(
    r"\[DEBUG-SA-STEER\] stage=pre_boundary_candidate target_uuid=(?P<uuid>\S+) "
    r"shift_length=(?P<shift>[-+0-9.eE]+) required_shift=(?P<required>[-+0-9.eE]+) "
    r"jerk_distance=(?P<jerk_distance>[-+0-9.eE]+) "
    r"transition_distance=(?P<transition>[-+0-9.eE]+) "
    r"min_shifting_distance=(?P<min_shift>[-+0-9.eE]+)"
)
PATH_RE = re.compile(
    r"\[DEBUG-SA-STEER\] stage=pre_boundary_candidate path_points=(?P<points>\d+) "
    r"(?:first_index=(?P<first>\d+) )?max_abs_curvature=(?P<curvature>[-+0-9.eE]+) "
    r"signed_curvature=(?P<signed>[-+0-9.eE]+) "
    r"equivalent_front_wheel_angle_rad=(?P<angle>[-+0-9.eE]+) "
    r"equivalent_front_wheel_angle_deg=(?P<angle_deg>[-+0-9.eE]+) "
    r"max_index=(?P<index>\d+)"
)
BOUNDARY_RE = re.compile(
    r"\[DEBUG-SA-STEER\] stage=(?P<stage>boundary_validation(?:_cache_hit)?) "
    r"result=(?P<result>\S+) minimum_boundary_clearance=(?P<clearance>[-+0-9.eE]+) "
    r"required_margin=(?P<margin>[-+0-9.eE]+) clearance_shortfall=(?P<shortfall>[-+0-9.eE]+) "
    r"boundary_side=(?P<side>\S+) path_index=(?P<index>\d+)"
)
PURE_PURSUIT_RE = re.compile(
    r"\[DEBUG-PP-STEER\] target_curvature=(?P<curvature>[-+0-9.eE]+) "
    r"raw_front_wheel_angle_rad=(?P<raw>[-+0-9.eE]+) "
    r"raw_front_wheel_angle_deg=(?P<raw_deg>[-+0-9.eE]+) "
    r"clamped_front_wheel_angle_rad=(?P<clamped>[-+0-9.eE]+) "
    r"clamped_front_wheel_angle_deg=(?P<clamped_deg>[-+0-9.eE]+) "
    r"max_steering_angle_rad=(?P<limit>[-+0-9.eE]+) "
    r"max_steering_angle_deg=(?P<limit_deg>[-+0-9.eE]+) hard_clamp=(?P<hard_clamp>\S+)"
)
SHIFT_LINE_RE = re.compile(
    r"\[DEBUG-SA-STEER\] stage=pre_boundary_candidate shift_line\[(?P<line>\d+)\] "
    r"start_idx=(?P<start_idx>\d+) end_idx=(?P<end_idx>\d+) "
    r"start_shift=(?P<start_shift>[-+0-9.eE]+) end_shift=(?P<end_shift>[-+0-9.eE]+) "
    r"start=\((?P<start_x>[-+0-9.eE]+),(?P<start_y>[-+0-9.eE]+)\) "
    r"end=\((?P<end_x>[-+0-9.eE]+),(?P<end_y>[-+0-9.eE]+)\)"
)


@dataclass
class SweepRow:
    case: str
    diagnostic_log: str
    candidate_count: int = 0
    shift_length_m: float | None = None
    required_shift_m: float | None = None
    jerk_distance_m: float | None = None
    transition_distance_m: float | None = None
    min_shifting_distance_m: float | None = None
    avoid_start_x_m: float | None = None
    avoid_start_y_m: float | None = None
    avoid_end_x_m: float | None = None
    avoid_end_y_m: float | None = None
    max_curvature_1pm: float | None = None
    max_angle_deg: float | None = None
    boundary_result: str = "NO_DIAGNOSTIC"
    min_boundary_clearance_m: float | None = None
    required_margin_m: float | None = None
    clearance_shortfall_m: float | None = None
    boundary_side: str = "none"
    boundary_path_index: int | None = None
    pure_pursuit_raw_angle_deg: float | None = None
    pure_pursuit_clamped_angle_deg: float | None = None
    pure_pursuit_hard_clamp: str = "N/A"
    transition_delta_m: float | None = None
    curvature_delta_percent: float | None = None
    max_shift_endpoint_delta_m: float | None = None
    path_changed_vs_baseline: str = "INDETERMINATE"
    path_steeper_vs_baseline: str = "INDETERMINATE"


def _number(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        return parsed
    return parsed


def find_diagnostic_log(case_dir: Path) -> Path | None:
    logs = sorted(path for path in case_dir.rglob("*") if path.is_file() and path.suffix in {".log", ".txt"})
    for path in logs:
        try:
            if "[DEBUG-SA-STEER]" in path.read_text(encoding="utf-8", errors="replace"):
                return path
        except OSError:
            continue
    return None


def summarize_case(case_dir: Path, record_selection: str = "peak") -> SweepRow:
    """Summarize one case.

    ``peak`` preserves the historical behavior of selecting the candidate with
    the largest absolute steering angle seen during replay.  ``first`` selects
    the first complete candidate record, which is the appropriate comparison
    point when each case is replayed from the same initial state.
    """

    if record_selection not in {"first", "peak"}:
        raise ValueError(f"unsupported record selection: {record_selection}")

    log_path = find_diagnostic_log(case_dir)
    row = SweepRow(case_dir.name, str(log_path) if log_path else "")
    if log_path is None:
        return row

    text = log_path.read_text(encoding="utf-8", errors="replace")
    candidates = list(CANDIDATE_RE.finditer(text))
    row.candidate_count = len(candidates)

    # Each diagnostic candidate is emitted as three adjacent records:
    # path curvature, candidate geometry, and avoid shift line.  Keep the
    # values from one record together; mixing the last candidate with the
    # maximum curvature over the whole replay can describe different paths.
    shift_lines = [match for match in SHIFT_LINE_RE.finditer(text) if match["line"] == "0"]
    paths = list(PATH_RE.finditer(text))
    record_count = min(len(candidates), len(shift_lines), len(paths))
    if record_count:
        record_index = (
            0
            if record_selection == "first"
            else max(
                range(record_count), key=lambda index: abs(_number(paths[index]["angle"]))
            )
        )
        candidate = candidates[record_index]
        line = shift_lines[record_index]
        path = paths[record_index]
        row.shift_length_m = _number(candidate["shift"])
        row.required_shift_m = _number(candidate["required"])
        row.jerk_distance_m = _number(candidate["jerk_distance"])
        row.transition_distance_m = _number(candidate["transition"])
        row.min_shifting_distance_m = _number(candidate["min_shift"])
        row.avoid_start_x_m = _number(line["start_x"])
        row.avoid_start_y_m = _number(line["start_y"])
        row.avoid_end_x_m = _number(line["end_x"])
        row.avoid_end_y_m = _number(line["end_y"])
        row.max_curvature_1pm = _number(path["curvature"])
        row.max_angle_deg = _number(path["angle_deg"])
    else:
        # Preserve partial diagnostics if a process was interrupted between
        # the three tagged log records.
        if candidates:
            candidate = candidates[0] if record_selection == "first" else candidates[-1]
            row.shift_length_m = _number(candidate["shift"])
            row.required_shift_m = _number(candidate["required"])
            row.jerk_distance_m = _number(candidate["jerk_distance"])
            row.transition_distance_m = _number(candidate["transition"])
            row.min_shifting_distance_m = _number(candidate["min_shift"])
        if shift_lines:
            line = shift_lines[0] if record_selection == "first" else shift_lines[-1]
            row.avoid_start_x_m = _number(line["start_x"])
            row.avoid_start_y_m = _number(line["start_y"])
            row.avoid_end_x_m = _number(line["end_x"])
            row.avoid_end_y_m = _number(line["end_y"])
        if paths:
            path = (
                paths[0]
                if record_selection == "first"
                else max(paths, key=lambda match: abs(_number(match["angle"])))
            )
            row.max_curvature_1pm = _number(path["curvature"])
            row.max_angle_deg = _number(path["angle_deg"])

    boundaries = list(BOUNDARY_RE.finditer(text))
    if boundaries:
        boundary = min(
            boundaries,
            key=lambda match: _number(match["clearance"])
            if math.isfinite(_number(match["clearance"]))
            else math.inf,
        )
        row.boundary_result = boundary["result"]
        row.min_boundary_clearance_m = _number(boundary["clearance"])
        row.required_margin_m = _number(boundary["margin"])
        row.clearance_shortfall_m = _number(boundary["shortfall"])
        row.boundary_side = boundary["side"]
        row.boundary_path_index = int(boundary["index"])

    # The Pure Pursuit log may be written by a different ROS process than the
    # Simple Avoidance log, so inspect every text log in this case directory.
    pure_pursuit_matches = []
    for path in case_dir.rglob("*"):
        if not path.is_file() or path.suffix not in {".log", ".txt"}:
            continue
        try:
            pure_pursuit_matches.extend(
                PURE_PURSUIT_RE.finditer(path.read_text(encoding="utf-8", errors="replace"))
            )
        except OSError:
            continue
    if pure_pursuit_matches:
        pure_pursuit = max(
            pure_pursuit_matches, key=lambda match: abs(_number(match["raw"]))
        )
        row.pure_pursuit_raw_angle_deg = _number(pure_pursuit["raw_deg"])
        row.pure_pursuit_clamped_angle_deg = _number(pure_pursuit["clamped_deg"])
        row.pure_pursuit_hard_clamp = pure_pursuit["hard_clamp"]
    return row


def parse_case_parameter(case: str, key: str) -> str:
    match = re.search(rf"{re.escape(key)}_([-+0-9.eE]+)", case)
    return match.group(1) if match else ""


def annotate_baseline_comparison(
    rows: list[SweepRow], baseline_jerk: float = 0.8, baseline_distance: float = 5.0
) -> None:
    """Annotate geometry changes relative to the configured baseline case."""

    baseline = next(
        (
            row
            for row in rows
            if parse_case_parameter(row.case, "jerk")
            and parse_case_parameter(row.case, "distance")
            and math.isclose(float(parse_case_parameter(row.case, "jerk")), baseline_jerk)
            and math.isclose(float(parse_case_parameter(row.case, "distance")), baseline_distance)
        ),
        None,
    )
    if baseline is None:
        return

    def endpoint_delta(row: SweepRow) -> float | None:
        values = (
            row.avoid_start_x_m,
            row.avoid_start_y_m,
            row.avoid_end_x_m,
            row.avoid_end_y_m,
            baseline.avoid_start_x_m,
            baseline.avoid_start_y_m,
            baseline.avoid_end_x_m,
            baseline.avoid_end_y_m,
        )
        if any(value is None for value in values):
            return None
        start_delta = math.hypot(
            row.avoid_start_x_m - baseline.avoid_start_x_m,
            row.avoid_start_y_m - baseline.avoid_start_y_m,
        )
        end_delta = math.hypot(
            row.avoid_end_x_m - baseline.avoid_end_x_m,
            row.avoid_end_y_m - baseline.avoid_end_y_m,
        )
        return max(start_delta, end_delta)

    for row in rows:
        if row.transition_distance_m is not None and baseline.transition_distance_m is not None:
            row.transition_delta_m = row.transition_distance_m - baseline.transition_distance_m
        if (
            row.max_curvature_1pm is not None
            and baseline.max_curvature_1pm is not None
            and abs(baseline.max_curvature_1pm) > 1.0e-9
        ):
            row.curvature_delta_percent = (
                (abs(row.max_curvature_1pm) - abs(baseline.max_curvature_1pm))
                / abs(baseline.max_curvature_1pm)
                * 100.0
            )
        row.max_shift_endpoint_delta_m = endpoint_delta(row)

        comparable = row.curvature_delta_percent is not None
        if comparable:
            endpoint_changed = (
                row.max_shift_endpoint_delta_m is not None
                and row.max_shift_endpoint_delta_m > 0.25
            )
            curvature_changed = abs(row.curvature_delta_percent) >= 5.0
            row.path_changed_vs_baseline = "YES" if endpoint_changed or curvature_changed else "NO"
        if comparable and row.transition_delta_m is not None:
            row.path_steeper_vs_baseline = (
                "YES"
                if row.transition_delta_m < -1.0e-6 and row.curvature_delta_percent >= 5.0
                else "NO"
            )


def write_outputs(rows: list[SweepRow], output: Path, record_selection: str = "peak") -> None:
    annotate_baseline_comparison(rows)
    fields = [field.name for field in SweepRow.__dataclass_fields__.values()]
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(row.__dict__ for row in rows)

    markdown = output.with_suffix(".md")
    headers = ["case", "candidates", "jerk distance [m]", "transition [m]", "transition delta [m]", "max angle [deg]", "max curvature [1/m]", "curvature delta [%]", "shift endpoint delta [m]", "path changed", "path steeper", "boundary", "min clearance [m]", "shortfall [m]", "PP raw [deg]", "PP clamped [deg]", "PP hard clamp"]
    lines = [
        "# Simple Avoidance steering parameter sweep",
        "",
        f"Record selection: `{record_selection}`. Boundary results are informational only and are not a geometry acceptance gate.",
        "",
        "| " + " | ".join(headers) + " |",
        "|" + "|".join("---" for _ in headers) + "|",
    ]
    for row in rows:
        def fmt(value: object) -> str:
            return "N/A" if value is None else str(value)

        lines.append(
            "| "
            + " | ".join(
                [
                    row.case,
                    str(row.candidate_count),
                    fmt(row.jerk_distance_m),
                    fmt(row.transition_distance_m),
                    fmt(row.transition_delta_m),
                    fmt(row.max_angle_deg),
                    fmt(row.max_curvature_1pm),
                    fmt(row.curvature_delta_percent),
                    fmt(row.max_shift_endpoint_delta_m),
                    row.path_changed_vs_baseline,
                    row.path_steeper_vs_baseline,
                    row.boundary_result,
                    fmt(row.min_boundary_clearance_m),
                    fmt(row.clearance_shortfall_m),
                    fmt(row.pure_pursuit_raw_angle_deg),
                    fmt(row.pure_pursuit_clamped_angle_deg),
                    row.pure_pursuit_hard_clamp,
                ]
            )
            + " |"
        )
    markdown.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sweep_root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--record-selection",
        choices=("first", "peak"),
        default="peak",
        help="candidate record to compare within each replay (default: peak)",
    )
    args = parser.parse_args()
    case_dirs = sorted(path for path in args.sweep_root.iterdir() if path.is_dir() and path.name.startswith("jerk_"))
    rows = [summarize_case(case_dir, args.record_selection) for case_dir in case_dirs]
    write_outputs(rows, args.output, args.record_selection)
    print(args.output)
    print(args.output.with_suffix(".md"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
