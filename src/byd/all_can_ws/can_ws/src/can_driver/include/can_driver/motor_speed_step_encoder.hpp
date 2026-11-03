#ifndef CAN_DRIVER__MOTOR_SPEED_STEP_ENCODER_HPP_
#define CAN_DRIVER__MOTOR_SPEED_STEP_ENCODER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace can_driver
{

inline bool isValidMonotonicLookupTable(
  const std::vector<double> & breakpoints, const std::vector<int64_t> & commands,
  const int64_t max_command = 32767) noexcept
{
  if (breakpoints.empty() && commands.empty()) {
    return true;
  }
  if (breakpoints.empty() || breakpoints.size() != commands.size()) {
    return false;
  }
  for (std::size_t i = 0; i < breakpoints.size(); ++i) {
    if (!std::isfinite(breakpoints[i]) || breakpoints[i] <= 0.0 ||
        commands[i] < 0 || commands[i] > max_command ||
        (i > 0 && (breakpoints[i] <= breakpoints[i - 1] ||
                   commands[i] < commands[i - 1]))) {
      return false;
    }
  }
  return true;
}

/// Encode the motor-speed increment applied by the VCU every 20 ms.
/// One command count means that the target motor speed changes by 1 r/s per cycle.
inline uint8_t encodeMotorSpeedStep(
  const double acceleration, const double step_counts_per_mps2, const uint8_t fallback,
  const bool enabled) noexcept
{
  if (!enabled || !std::isfinite(acceleration) || !std::isfinite(step_counts_per_mps2) ||
      std::abs(acceleration) < 1.0e-6 || step_counts_per_mps2 <= 0.0) {
    return fallback;
  }

  const long command = std::lround(std::abs(acceleration) * step_counts_per_mps2);
  return static_cast<uint8_t>(std::clamp(command, 1L, 255L));
}

inline uint8_t encodeDirectionalMotorSpeedStep(
  const double acceleration, const double step_counts_per_mps2, const uint8_t fallback,
  const bool enabled, const bool encode_acceleration) noexcept
{
  const bool direction_matches = encode_acceleration ? acceleration > 0.0 : acceleration < 0.0;
  return encodeMotorSpeedStep(
    acceleration, step_counts_per_mps2, fallback, enabled && direction_matches);
}

inline int16_t encodeSignedCommand(
  const double value, const double positive_scale, const double negative_scale,
  const double positive_offset, const double negative_offset, const int16_t limit) noexcept
{
  if (!std::isfinite(value)) {
    return 0;
  }
  const double scale = value >= 0.0 ? positive_scale : negative_scale;
  const double offset = value > 0.0 ? positive_offset : (value < 0.0 ? -negative_offset : 0.0);
  const long command = std::lround(value * scale + offset);
  return static_cast<int16_t>(std::clamp(command, -static_cast<long>(limit), static_cast<long>(limit)));
}

/// Map a non-negative physical command through a calibrated monotonic lookup
/// table.  The table contains physical breakpoints and wire values, both in
/// ascending order.  Values outside the calibrated range are clamped to the
/// nearest endpoint; no extrapolation is performed.
inline int16_t encodePiecewiseCommand(
  const double value, const std::vector<double> & breakpoints,
  const std::vector<int64_t> & commands, const int16_t limit) noexcept
{
  if (!std::isfinite(value) || value <= 0.0 || breakpoints.empty() ||
      breakpoints.size() != commands.size()) {
    return 0;
  }

  double wire = static_cast<double>(commands.front());
  if (value <= breakpoints.front()) {
    wire = static_cast<double>(commands.front());
  } else if (value >= breakpoints.back()) {
    wire = static_cast<double>(commands.back());
  } else {
    for (std::size_t i = 1; i < breakpoints.size(); ++i) {
      if (value <= breakpoints[i]) {
        const double denominator = breakpoints[i] - breakpoints[i - 1];
        const double ratio = denominator > 0.0 ?
          (value - breakpoints[i - 1]) / denominator : 0.0;
        wire = static_cast<double>(commands[i - 1]) +
          ratio * static_cast<double>(commands[i] - commands[i - 1]);
        break;
      }
    }
  }

  const long rounded = std::lround(wire);
  return static_cast<int16_t>(std::clamp(
    rounded, 0L, static_cast<long>(std::max<int16_t>(limit, 0))));
}

}  // namespace can_driver

#endif  // CAN_DRIVER__MOTOR_SPEED_STEP_ENCODER_HPP_
