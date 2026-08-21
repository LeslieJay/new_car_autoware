#!/usr/bin/env python3
"""Round-4 CAN calibration: cross-validate the dynamic Byte5/Byte6 conversion."""

from __future__ import annotations

import os
from pathlib import Path

import run_can_round2_calibration as calibration


calibration.BYTE6_VALUES = ()
calibration.RAMP_PROFILES = (
    # Labels contain the expected active byte after coefficient conversion.
    ("dynamic_0p10_expected_b5_2_b6_2", 0.10, 2, 2),
    ("dynamic_0p15_expected_b5_2_b6_3", 0.15, 2, 3),
)
calibration.WARMUP_PROFILE = ("warmup_dynamic_0p10", 0.10, 2, 2)
calibration.USE_FIXED_STEPS = False
calibration.EXPECT_DYNAMIC_MODE = True
calibration.EXPECTED_ACCELERATION_COEFFICIENT = 16.0
calibration.EXPECTED_DECELERATION_COEFFICIENT = 17.25
calibration.EXPECTED_WIRE_PAIRS = ((2, 10), (10, 2), (10, 3), (10, 10))
calibration.RESULT_ROOT = Path(
    os.environ.get(
        "RESULT_ROOT",
        str(calibration.WORKSPACE / "log" / "can_round4_calibration"),
    )
)
calibration.ROUND_LABEL = "第四轮"
calibration.ANALYSIS_FILENAME = "round4_analysis.csv"
calibration.CLI_DESCRIPTION = __doc__


if __name__ == "__main__":
    raise SystemExit(calibration.main())
