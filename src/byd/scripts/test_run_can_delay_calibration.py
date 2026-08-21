#!/usr/bin/env python3

import pytest

import run_can_delay_calibration as calibration


def test_fit_effective_delay_recovers_known_delayed_deceleration() -> None:
    initial_speed = 0.5
    expected_delay = 0.24
    expected_deceleration = 0.5
    samples = []
    for index in range(80):
        elapsed = index * 0.02
        velocity = initial_speed - expected_deceleration * max(
            0.0, elapsed - expected_delay
        )
        samples.append((elapsed, max(0.0, velocity)))

    result = calibration.fit_effective_delay(samples, initial_speed)

    assert result["delay_s"] == pytest.approx(expected_delay, abs=0.011)
    assert result["deceleration_mps2"] == pytest.approx(
        expected_deceleration, abs=0.01
    )
    assert result["rmse_mps"] < 0.002


def test_fit_effective_delay_rejects_uninformative_samples() -> None:
    samples = [(index * 0.02, 0.5) for index in range(20)]

    with pytest.raises(ValueError, match="insufficient velocity decrease"):
        calibration.fit_effective_delay(samples, 0.5)
