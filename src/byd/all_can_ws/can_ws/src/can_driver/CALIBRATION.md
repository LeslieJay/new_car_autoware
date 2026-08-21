# CAN command calibration

## Protocol interpretation

CAN ID `0x201` is sent every 20 ms. Byte 5 and Byte 6 are not time values:

- Byte 5 limits the increase of target motor speed.
- Byte 6 limits the decrease of target motor speed.
- A value of 1 changes target motor speed by 1 r/s per 20 ms cycle.

The dynamic conversion is disabled until calibration coefficients are available. In fixed mode,
`default_acceleration_step_command` and `default_deceleration_step_command` are sent unchanged.

## Calibration objective

Calibrate the complete boundary from `autoware_control_msgs/msg/Control` to VCU `0x201` and
vehicle feedback. Treat the following as four separate mappings; completing Byte 5/6 alone is not
a complete `control_cmd` calibration:

| Control field | 0x201 field | Candidate parameters |
|---|---|---|
| longitudinal velocity | Bytes 0-1 signed target speed | forward/reverse speed scale and offset |
| steering tire angle | Bytes 2-3 signed steering target | left/right steering scale and zero offset |
| positive acceleration | Byte 5 motor-speed increment/cycle | acceleration counts per m/s^2 |
| negative acceleration | Byte 6 motor-speed decrement/cycle | deceleration counts per m/s^2 |

Jerk and steering rotation rate have no `0x201` field and are recorded only; do not fit parameters
for them.

## Low-speed calibration

Use lifted drive wheels or a roller bed first. Ensure no other node publishes
`/control/command/control_cmd` and keep an emergency stop available.

For the complete closed-road Byte5/Byte6 sweep, use:

```bash
source /opt/ros/humble/setup.bash
source /home/byd/weicanming/github_projects/new_car_autoware/install/setup.bash
source /home/byd/weicanming/github_projects/new_car_autoware/src/byd/all_can_ws/can_ws/install/setup.bash
/home/byd/weicanming/github_projects/new_car_autoware/src/byd/scripts/run_can_step_calibration.py
```

The script continuously publishes at 50 Hz, records raw velocity samples and one bag for the full
run, scans
`1,2,3,5,8,10,15` three times for each byte, and waits for `NEXT` after every byte value so the
vehicle can be repositioned. Runtime parameter updates require this version of `can_driver`.
The CAN adapter keeps its own 20 ms clock and transmits the latest command available at each
tick, so ROS and CAN have the same nominal frequency without requiring phase alignment.

1. **Wire verification:** verify signed little-endian Bytes 0-3 and fixed Byte 5/6 values 1, 2, 3,
   5, 8, 10, and 15 directly in `can_201.log`. Reject a trial if the wire value differs.
2. **Speed mapping:** with conservative Byte 5/6, command 0.05, 0.10, 0.20, 0.30, 0.40, and
   0.50 m/s in DRIVE and REVERSE. Hold each point until stable for one second, repeat three times,
   and fit actual steady velocity against raw signed Bytes 0-1 independently by direction. Invert
   those fits to obtain forward/reverse scale and offset.
3. **Acceleration mapping:** from rest to 0.5 m/s, sweep Byte 5. Fit only samples between the
   `accelerate` phase start and `target_speed`; use the 10%-80% velocity interval. Never include
   hold or stopping samples. Compute each candidate as `Byte5 / measured_acceleration`.
4. **Deceleration mapping:** from a stable 0.5 m/s to zero, sweep Byte 6. Fit only samples between
   the `stop` phase start and `zero_speed`; use the 80%-10% interval. Compute each candidate as
   `Byte6 / abs(measured_deceleration)`.
5. **Steering mapping:** at zero longitudinal target, command 0, +/-2, +/-5, +/-10, +/-20, and
   +/-30 degrees. Hold until steering feedback is stable, repeat three times, fit left and right
   independently against raw signed Bytes 2-3, then invert the fits for scale and common offset.
6. **Cross-validation:** write accepted values to `can_params.yaml`, enable
   `use_dynamic_acceleration_steps`, and run
   `0 -> 0.2 -> 0.4 -> 0.2 -> 0 m/s` three times in each direction.

Each three-run group uses its median. A run more than 30% from the group median is flagged as an
outlier and excluded from the candidate coefficient, but remains in the CSV. At least three Byte
groups must remain monotonic before a coefficient is marked ready. Do not automatically write
candidates into the vehicle configuration.

To reanalyse a completed result directory without moving the vehicle:

```bash
run_can_step_calibration.py --analyze-result <result-directory>
```

The command reads `velocity_samples.csv`, or falls back to the recorded rosbag for older runs, and
produces `calibration_analysis.csv`, `calibration_groups.csv`, and
`calibration_candidates.yaml`. Missing mapping stages are explicitly reported as `not_ready` or
`not_calibrated`.

For acceleration, the implemented conversion is:

```text
positive acceleration: Byte5 = round(acceleration * acceleration_step_counts_per_mps2)
negative acceleration: Byte6 = round(abs(acceleration) * deceleration_step_counts_per_mps2)
```

Non-zero results are clamped to 1-255. During positive acceleration Byte6 uses its fixed fallback;
during negative acceleration Byte5 uses its fixed fallback. Undefined or zero acceleration uses both
fixed fallbacks.

## Acceptance criteria

- ROS control input and CAN `0x201` both average approximately 50 Hz.
- CAN mean period is approximately 20 ms, P99 is no more than 25 ms, and the maximum normal
  interval is no more than 40 ms. See `can_period_stats.txt` in the calibration result directory.
- Missing control input warns at 60 ms and switches to the zero-speed brake frame by 100-120 ms;
  the safety frame continues at 50 Hz.
- Steady speed error: no more than 0.02 m/s from 0 to 0.5 m/s.
- Acceleration and deceleration error: no more than 10%.
- Steering angle error: no more than 0.5 degrees.
- Report command delay, 10%-90% rise/fall time, stopping distance, and three-run repeatability.
- Jerk and steering rotation rate are recorded for analysis only; `0x201` has no corresponding
  fields, so the CAN driver does not apply a second software limiter.
