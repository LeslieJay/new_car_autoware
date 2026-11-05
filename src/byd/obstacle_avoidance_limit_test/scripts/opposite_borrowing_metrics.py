#!/usr/bin/env python3
"""Collect articulated borrowing diagnostics without requiring a ROS installation.

The footprint diagnostics are estimates based on measured tractor odometry and
kinematic trailer prediction; they are not independent trailer ground truth.
"""
from __future__ import annotations

import math
import re


def new_metrics() -> dict:
    return {
        "borrowing_diagnostic_samples": 0,
        "borrowing_actual_samples": 0,
        "borrowing_min_body_clearance_m": None,
        "borrowing_boundary_violations": 0,
        "borrowing_all_bodies_returned": False,
        "borrowing_complete_seen": False,
        "borrowing_max_planning_ms": None,
    }


def observe(metrics: dict, text: str) -> None:
    if "[OPPOSITE_BORROWING]" not in text or "phase=" not in text:
        return
    fields = dict(re.findall(r"([a-z_]+)=([^\s]+)", text))
    metrics["borrowing_diagnostic_samples"] += 1
    try:
        outside = int(fields.get("actual_outside_corridor", "-1"))
        returned = int(fields.get("actual_returned", "-1"))
    except ValueError:
        outside = returned = -1
    if outside >= 0:
        metrics["borrowing_actual_samples"] += 1
        metrics["borrowing_boundary_violations"] += int(outside > 0)
    if returned >= 0:
        metrics["borrowing_all_bodies_returned"] = returned == 1
    if fields.get("phase") == "complete" and returned == 1:
        metrics["borrowing_complete_seen"] = True
    for source, dest, operation in (
        ("actual_min_clearance_m", "borrowing_min_body_clearance_m", min),
        ("planning_ms", "borrowing_max_planning_ms", max),
    ):
        try:
            value = float(fields.get(source, "nan"))
        except ValueError:
            continue
        if math.isfinite(value):
            previous = metrics[dest]
            metrics[dest] = value if previous is None else operation(previous, value)


def failures(metrics: dict, require_complete: bool) -> list[str]:
    result = []
    if not metrics["borrowing_diagnostic_samples"]:
        result.append("BORROWING_DIAGNOSTICS_MISSING")
    if metrics["borrowing_boundary_violations"]:
        result.append("ARTICULATED_BOUNDARY_VIOLATION")
    clearance = metrics["borrowing_min_body_clearance_m"]
    if clearance is not None and clearance <= 0.0:
        result.append("ARTICULATED_BODY_COLLISION")
    if require_complete:
        if not metrics["borrowing_actual_samples"]:
            result.append("ARTICULATED_SAMPLES_MISSING")
        if not metrics["borrowing_complete_seen"] or not metrics["borrowing_all_bodies_returned"]:
            result.append("ARTICULATED_RETURN_NOT_COMPLETED")
    return result
