#!/usr/bin/env python3

import math
from pathlib import Path

import run_can_initial_full_calibration as calibration
import run_can_deceleration_supplement as supplement


def test_build_lookup_uses_smallest_byte_that_reaches_target() -> None:
    mapping = [(1, 0.06), (2, 0.12), (3, 0.19), (5, 0.33)]

    lookup = dict(calibration.build_lookup(mapping))

    assert lookup[0.05] == 1
    assert lookup[0.10] == 2
    assert lookup[0.15] == 3
    assert lookup[0.20] == 5
    assert lookup[0.40] is None


def test_build_lookup_does_not_follow_nonmonotonic_measurement_backwards() -> None:
    mapping = [(1, 0.20), (2, 0.19), (3, 0.21)]

    lookup = dict(calibration.build_lookup(mapping))

    assert lookup[0.20] == 1


def test_scan_stops_after_two_small_relative_gains() -> None:
    assert calibration.should_stop_scan([0.50, 0.54, 0.57]) == (
        "two_consecutive_gains_below_10_percent"
    )
    assert calibration.should_stop_scan([0.50, 0.60, 0.70]) is None
    assert calibration.should_stop_scan([0.50, math.nan, 0.70]) is None


def test_deceleration_supplement_does_not_stop_on_low_byte_platform() -> None:
    medians = [0.18, 0.20, 0.18, 0.23, 0.29]

    assert supplement.should_stop(5, medians) is None


def test_deceleration_supplement_can_stop_after_byte10() -> None:
    assert supplement.should_stop(9, [0.45, 0.47, 0.48]) is None
    assert supplement.should_stop(10, [0.45, 0.47, 0.48]) == (
        "two_consecutive_gains_below_10_percent_after_byte10"
    )


def test_deceleration_supplement_uses_node_parameter_interface() -> None:
    source = Path(supplement.__file__).read_text(encoding="utf-8")

    assert "calibration.set_can_steps(node, 10, 10)" in source
    assert "calibration.set_can_steps(10, 10)" not in source
