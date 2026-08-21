#!/usr/bin/env python3
"""Round-3 CAN calibration: validate ramp tracking with acceleration headroom."""

from __future__ import annotations

import os
from pathlib import Path

import run_can_round2_calibration as calibration


calibration.BYTE6_VALUES = ()
calibration.RAMP_PROFILES = (
    # profile name, command slope, Byte5 acceleration capacity, Byte6 deceleration capacity
    ("headroom_0p10_b3_b4", 0.10, 3, 4),
    ("headroom_0p15_b4_b4", 0.15, 4, 4),
    ("headroom_0p20_b5_b6", 0.20, 5, 6),
)
calibration.RESULT_ROOT = Path(
    os.environ.get(
        "RESULT_ROOT",
        str(calibration.WORKSPACE / "log" / "can_round3_calibration"),
    )
)
calibration.ROUND_LABEL = "第三轮"
calibration.ANALYSIS_FILENAME = "round3_analysis.csv"
calibration.CLI_DESCRIPTION = __doc__


if __name__ == "__main__":
    raise SystemExit(calibration.main())
