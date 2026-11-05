// Copyright 2026 BYD
// SPDX-License-Identifier: Apache-2.0
#ifndef AVOIDANCE_BY_LANE_CHANGE__BORROWING_MATH_HPP_
#define AVOIDANCE_BY_LANE_CHANGE__BORROWING_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::behavior_path_planner::borrowing
{
// An unreachable stop in the borrowing interval must not be replaced by a minimum speed.
inline std::vector<double> arrivalTimes(
  const std::vector<double> & distances, const std::vector<double> & limits,
  const double initial_speed, const double acceleration, const double deceleration,
  const double speed_scale)
{
  if (distances.empty() || limits.size() != distances.size() || acceleration <= 0.0 ||
      deceleration <= 0.0 || speed_scale <= 0.0 || !std::isfinite(initial_speed) ||
      initial_speed < 0.0 || !std::isfinite(acceleration) || !std::isfinite(deceleration) ||
      !std::isfinite(speed_scale)) return {};
  std::vector<double> speed(limits.size());
  for (size_t i = 0; i < speed.size(); ++i) {
    if (!std::isfinite(distances[i]) || !std::isfinite(limits[i]) || limits[i] <= 0.0 ||
        (i && distances[i] <= distances[i - 1])) return {};
    speed[i] = limits[i] * speed_scale;
  }
  speed.front() = std::max(0.0, initial_speed);
  for (size_t i = 1; i < speed.size(); ++i) {
    speed[i] = std::min(speed[i], std::sqrt(
      speed[i - 1] * speed[i - 1] + 2.0 * acceleration * (distances[i] - distances[i - 1])));
  }
  for (size_t i = speed.size() - 1; i > 0; --i) {
    speed[i - 1] = std::min(speed[i - 1], std::sqrt(
      speed[i] * speed[i] + 2.0 * deceleration * (distances[i] - distances[i - 1])));
  }
  // A profile demanding instantaneous deceleration is not executable.
  if (speed.front() + 1e-3 < initial_speed) return {};
  std::vector<double> time(speed.size(), 0.0);
  for (size_t i = 1; i < speed.size(); ++i) {
    const double sum = speed[i] + speed[i - 1];
    if (sum < 1e-6) return {};
    time[i] = time[i - 1] + 2.0 * (distances[i] - distances[i - 1]) / sum;
  }
  return time;
}

inline bool coversReturn(
  const std::vector<double> & arrivals, const double time_gap, const double horizon)
{
  return !arrivals.empty() && std::isfinite(arrivals.back()) && std::isfinite(time_gap) &&
         std::isfinite(horizon) && time_gap >= 0.0 &&
         arrivals.back() + time_gap <= horizon;
}

inline double brakingVelocity(const double distance, const double deceleration)
{
  return std::sqrt(2.0 * std::max(0.0, distance) * std::max(0.0, deceleration));
}
}  // namespace autoware::behavior_path_planner::borrowing
#endif  // AVOIDANCE_BY_LANE_CHANGE__BORROWING_MATH_HPP_
