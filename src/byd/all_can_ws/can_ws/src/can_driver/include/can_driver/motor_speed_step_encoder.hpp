#ifndef CAN_DRIVER__MOTOR_SPEED_STEP_ENCODER_HPP_
#define CAN_DRIVER__MOTOR_SPEED_STEP_ENCODER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace can_driver
{

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

}  // namespace can_driver

#endif  // CAN_DRIVER__MOTOR_SPEED_STEP_ENCODER_HPP_
