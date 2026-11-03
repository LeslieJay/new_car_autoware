#!/usr/bin/env python3

from pathlib import Path

from analyze_forward_speed_link_calibration import (
    nearest_command,
    parse_can_log,
    parse_driver_tx_log,
    robust_group,
)


def test_parse_can_log_and_decode_nearest_command(tmp_path: Path) -> None:
    log = tmp_path / "can.log"
    log.write_text(
        "(100.000000) can0  201#5A00000000100A00\n"
        "(100.020000) can0  181#0000006400000000\n",
        encoding="utf-8",
    )
    frames = parse_can_log(log)
    speed, byte5, byte6 = nearest_command(100.001, frames["201"])
    assert speed == 90.0
    assert byte5 == 16.0
    assert byte6 == 10.0
    assert len(frames["181"]) == 1


def test_robust_group_flags_large_outlier() -> None:
    rows = [
        {"target": "0.5", "value": "0.49"},
        {"target": "0.5", "value": "0.50"},
        {"target": "0.5", "value": "0.51"},
        {"target": "0.5", "value": "0.90"},
    ]
    groups = robust_group(rows, "target", "value")
    assert len(groups) == 1
    assert groups[0]["outliers"] == 1
    assert round(groups[0]["median"], 2) == 0.50


def test_parse_driver_write_timestamp_log(tmp_path: Path) -> None:
    log = tmp_path / "can_driver_tx.csv"
    log.write_text(
        "timestamp_epoch_s,interface,can_id,dlc,data\n"
        "100.100000000,can0,0x201,8,5a 00 00 00 00 10 0a 00\n",
        encoding="utf-8",
    )
    frames = parse_driver_tx_log(log)
    assert nearest_command(100.1, frames["201"]) == (90.0, 16.0, 10.0)
