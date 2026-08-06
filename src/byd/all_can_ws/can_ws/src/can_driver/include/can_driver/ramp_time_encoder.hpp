#ifndef CAN_DRIVER__RAMP_TIME_ENCODER_HPP_
#define CAN_DRIVER__RAMP_TIME_ENCODER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace can_driver
{

/// Convert a requested acceleration into the VCU's 0.1 s/bit ramp-time command.
inline uint8_t encodeRampTime(
  const double target_velocity, const double current_velocity, const double acceleration,
  const uint8_t fallback) noexcept
{
  if (!std::isfinite(target_velocity) || !std::isfinite(current_velocity) ||
      !std::isfinite(acceleration) || std::abs(acceleration) < 1.0e-6) {
    return fallback;
  }

  const double velocity_delta =
    std::abs(std::abs(target_velocity) - std::abs(current_velocity));
  const double ramp_time_seconds = velocity_delta / std::abs(acceleration);
  const double command = std::ceil(ramp_time_seconds / 0.1);
  return static_cast<uint8_t>(std::clamp(command, 1.0, 255.0));
}

}  // namespace can_driver

#endif  // CAN_DRIVER__RAMP_TIME_ENCODER_HPP_
