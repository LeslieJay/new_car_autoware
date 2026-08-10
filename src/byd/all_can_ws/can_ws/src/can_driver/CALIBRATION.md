# CAN command calibration

## Protocol interpretation

CAN ID `0x201` is sent every 20 ms. Byte 5 and Byte 6 are not time values:

- Byte 5 limits the increase of target motor speed.
- Byte 6 limits the decrease of target motor speed.
- A value of 1 changes target motor speed by 1 r/s per 20 ms cycle.

The dynamic conversion is disabled until calibration coefficients are available. In fixed mode,
`default_acceleration_step_command` and `default_deceleration_step_command` are sent unchanged.

## Low-speed calibration

Use lifted drive wheels or a roller bed first. Ensure no other node publishes
`/control/command/control_cmd` and keep an emergency stop available.

1. Verify raw `0x201` byte order with fixed Byte 5/6 values 1, 2, 3, 5, 8, 10, and 15.
2. At a low fixed step, command 0.05, 0.10, 0.20, 0.30, 0.40, and 0.50 m/s in both directions.
   Repeat each point three times and fit forward/reverse speed scale and offset separately.
3. From rest to 0.4 m/s, sweep Byte 5 through the same sequence. Measure acceleration over the
   10%-80% speed interval and fit actual acceleration against Byte 5.
4. From a stable 0.4 m/s to zero, sweep Byte 6. Fit deceleration separately and record stopping
   delay and distance.
5. Command steering angles 0, +/-2, +/-5, +/-10, +/-20, and +/-30 degrees three times. Fit left
   and right scale independently and then fit the common zero offset.
6. Write the fitted values to `can_params.yaml`, enable `use_dynamic_acceleration_steps`, and run
   `0 -> 0.2 -> 0.4 -> 0.2 -> 0 m/s` three times in each direction.

For acceleration, the implemented conversion is:

```text
Byte5 = round(abs(acceleration) * acceleration_step_counts_per_mps2)
Byte6 = round(abs(acceleration) * deceleration_step_counts_per_mps2)
```

Non-zero results are clamped to 1-255. Undefined or zero acceleration uses the fixed fallback.

## Acceptance criteria

- Steady speed error: no more than 0.02 m/s from 0 to 0.5 m/s.
- Acceleration and deceleration error: no more than 10%.
- Steering angle error: no more than 0.5 degrees.
- Report command delay, 10%-90% rise/fall time, stopping distance, and three-run repeatability.
- Jerk and steering rotation rate are recorded for analysis only; `0x201` has no corresponding
  fields, so the CAN driver does not apply a second software limiter.
