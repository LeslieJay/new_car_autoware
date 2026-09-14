#!/usr/bin/env python3
"""Aggregate machine-readable Simple LC Avoidance simulation results."""

from __future__ import annotations

import argparse
import csv
import json
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path


EXPECTED = {
    "success_right": "SUCCESS",
    "success_left": "SUCCESS",
    "safe_stop_3m": "SAFE_STOP",
    "safe_stop_5m": "SAFE_STOP",
    "safe_stop_occupied": "SAFE_STOP",
    "loss_recovery": "SUCCESS",
    "loss_stop": "SAFE_STOP",
    "passed_loss": "SUCCESS",
    "trailer_normal": "SUCCESS",
    "trailer_wide": "SAFE_STOP",
    "trailer_articulation": "SAFE_STOP",
    "low_speed_success": "SUCCESS",
    "path_generation_failure": "SAFE_STOP",
}


def as_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def as_float(row: dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def read_rows(root: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for path in sorted(root.glob("scenario_*/result/distance_sweep.csv")):
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                row["scenario_dir"] = str(path.parent.parent)
                rows.append(row)
    return rows


def inspect_lanelet_topology(map_file: Path | None) -> dict[str, object]:
    if map_file is None:
        return {"map_file": "", "available": False, "reason": "MAP_FILE_NOT_PROVIDED"}
    if not map_file.is_file():
        return {
            "map_file": str(map_file),
            "available": False,
            "reason": "MAP_FILE_NOT_FOUND",
        }
    try:
        root = ET.parse(map_file).getroot()
        boundary_counts: Counter[str] = Counter()
        lanelet_count = 0
        for relation in root.findall("relation"):
            tags = {tag.attrib.get("k"): tag.attrib.get("v") for tag in relation.findall("tag")}
            if tags.get("type") != "lanelet":
                continue
            lanelet_count += 1
            for member in relation.findall("member"):
                if member.attrib.get("role") in {"left", "right"}:
                    boundary_counts[member.attrib.get("ref", "")] += 1
        shared_boundaries = sum(1 for ref, count in boundary_counts.items() if ref and count > 1)
        return {
            "map_file": str(map_file),
            "available": True,
            "lanelet_relations": lanelet_count,
            "shared_boundary_ways": shared_boundaries,
            "has_parallel_adjacent_lane": shared_boundaries > 0,
        }
    except (ET.ParseError, OSError) as error:
        return {
            "map_file": str(map_file),
            "available": False,
            "reason": f"MAP_PARSE_FAILED:{error}",
        }


def evaluate(row: dict[str, str]) -> tuple[bool, list[str]]:
    scenario = row.get("acceptance_case", row.get("scenario", ""))
    expected = EXPECTED.get(scenario)
    result = row.get("result", "")
    reasons: list[str] = []
    if expected is None:
        reasons.append("UNKNOWN_SCENARIO")
    elif result != expected:
        reasons.append(f"EXPECTED_{expected}_GOT_{result}")

    if result == "SUCCESS":
        if not as_bool(row.get("obstacle_passed")):
            reasons.append("OBSTACLE_NOT_PASSED")
        if not as_bool(row.get("return_completed")):
            reasons.append("RETURN_NOT_COMPLETED")
        if as_float(row, "min_approx_clearance_m", -1.0) < 0.0:
            reasons.append("NEGATIVE_CLEARANCE")
        if as_float(row, "max_actual_to_final_lateral_error_m", 99.0) > 0.8:
            reasons.append("TRACKING_ERROR_OVER_0_8M")
    if result == "SAFE_STOP":
        if row.get("stop_front_clearance_m", "") and as_float(row, "stop_front_clearance_m", -1.0) < 1.0:
            reasons.append("STOP_CLEARANCE_UNDER_1M")
    return not reasons, reasons


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--map-file", type=Path, default=None)
    args = parser.parse_args()
    root = args.root
    root.mkdir(parents=True, exist_ok=True)
    rows = read_rows(root)
    map_capability = inspect_lanelet_topology(args.map_file)
    evaluated = []
    for row in rows:
        passed, reasons = evaluate(row)
        scenario = row.get("acceptance_case", row.get("scenario", ""))
        if (
            EXPECTED.get(scenario) == "SUCCESS"
            and map_capability.get("available")
            and not map_capability.get("has_parallel_adjacent_lane")
        ):
            reasons.append("MAP_NO_PARALLEL_ADJACENT_LANE")
            passed = False
        row["acceptance_pass"] = str(passed).lower()
        row["acceptance_reasons"] = ";".join(reasons)
        evaluated.append(row)

    csv_path = root / "acceptance_results.csv"
    fields: list[str] = []
    for row in evaluated:
        for field in row:
            if field not in fields:
                fields.append(field)
    if not fields:
        fields = ["scenario", "result", "acceptance_pass", "acceptance_reasons"]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(evaluated)

    summary = {
        "cases": len(evaluated),
        "passed": sum(1 for row in evaluated if as_bool(row["acceptance_pass"])),
        "failed": sum(1 for row in evaluated if not as_bool(row["acceptance_pass"])),
        "results": {result: sum(1 for row in evaluated if row.get("result") == result) for result in sorted({row.get("result", "") for row in evaluated})},
        "map_capability": map_capability,
    }
    summary["acceptance_pass"] = summary["cases"] > 0 and summary["failed"] == 0
    json_path = root / "acceptance_summary.json"
    json_path.write_text(json.dumps({"summary": summary, "cases": evaluated}, indent=2), encoding="utf-8")

    markdown_path = root / "acceptance_summary.md"
    with markdown_path.open("w", encoding="utf-8") as stream:
        stream.write("# Simple Lane Change Avoidance acceptance\n\n")
        stream.write(f"- Cases: {summary['cases']}\n- Passed: {summary['passed']}\n- Failed: {summary['failed']}\n- Overall: **{'PASS' if summary['acceptance_pass'] else 'FAIL'}**\n\n")
        stream.write(f"- Map: `{map_capability.get('map_file', '')}`\n")
        stream.write(f"- Parallel adjacent lane available: **{map_capability.get('has_parallel_adjacent_lane', 'UNKNOWN')}**\n\n")
        stream.write("| scenario | expected | actual | stop clearance (m) | tracking error (m) | result |\n")
        stream.write("|---|---|---|---:|---:|---|\n")
        for row in evaluated:
            scenario = row.get("acceptance_case", row.get("scenario", ""))
            stream.write(
                f"| {scenario} | {EXPECTED.get(scenario, 'UNKNOWN')} | {row.get('result', '')} | "
                f"{row.get('stop_front_clearance_m', '')} | {row.get('max_actual_to_final_lateral_error_m', '')} | "
                f"{'PASS' if as_bool(row['acceptance_pass']) else row['acceptance_reasons']} |\n"
            )
        stream.write("\nForbidden results are INVALID, COLLISION, and TRACKING_FAILURE. A map without a shared lane boundary cannot pass SUCCESS cases.\n")

    print(json.dumps(summary, indent=2))
    return 0 if summary["acceptance_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
