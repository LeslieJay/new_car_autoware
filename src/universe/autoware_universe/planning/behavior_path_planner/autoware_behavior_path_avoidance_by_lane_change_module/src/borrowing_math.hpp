// Copyright 2026 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// SPDX-License-Identifier: Apache-2.0
#ifndef AVOIDANCE_BY_LANE_CHANGE__BORROWING_MATH_HPP_
#define AVOIDANCE_BY_LANE_CHANGE__BORROWING_MATH_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace autoware::behavior_path_planner::borrowing
{
// Quintic Hermite segment. Derivatives are with respect to longitudinal distance.
struct LateralSegment
{
  std::array<double, 6> a;
  double length;
  LateralSegment(double y, double slope, double second, double end, double distance)
  : a{y, slope * distance, 0.5 * second * distance * distance, 0.0, 0.0, 0.0},
    length(distance)
  {
    const double b0 = end - a[0] - a[1] - a[2];
    const double b1 = -a[1] - 2.0 * a[2];
    const double b2 = -2.0 * a[2];
    a[3] = 10.0 * b0 - 4.0 * b1 + 0.5 * b2;
    a[4] = -15.0 * b0 + 7.0 * b1 - b2;
    a[5] = 6.0 * b0 - 3.0 * b1 + 0.5 * b2;
  }
  double value(double s) const {
    const double t = std::clamp(s / length, 0.0, 1.0);
    return a[0] + t * (a[1] + t * (a[2] + t * (a[3] + t * (a[4] + t * a[5]))));
  }
  double slope(double s) const {
    const double t = std::clamp(s / length, 0.0, 1.0);
    return (a[1] + t * (2.0*a[2] + t * (3.0*a[3] + t * (4.0*a[4] + t * 5.0*a[5])))) / length;
  }
};

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
