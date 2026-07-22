// Copyright 2026 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "didrive/front_collision_warning/risk_evaluator.hpp"

#include <algorithm>
#include <cmath>

namespace didrive::front_collision_warning
{

std::optional<Risk> evaluateRisk(
  const std::vector<VehicleObservation> & vehicles, const double ego_velocity,
  const Parameters & parameters)
{
  if (!std::isfinite(ego_velocity)) {
    return std::nullopt;
  }

  std::optional<Risk> best;
  for (std::size_t index = 0; index < vehicles.size(); ++index) {
    const auto & vehicle = vehicles[index];
    if (
      !std::isfinite(vehicle.x) || !std::isfinite(vehicle.y) || !std::isfinite(vehicle.yaw) ||
      !std::isfinite(vehicle.length) || !std::isfinite(vehicle.width) ||
      !std::isfinite(vehicle.velocity_x) || !std::isfinite(vehicle.velocity_y) ||
      vehicle.length <= 0.0 || vehicle.width <= 0.0)
    {
      continue;
    }

    const double half_x = 0.5 * (
      std::abs(std::cos(vehicle.yaw)) * vehicle.length +
      std::abs(std::sin(vehicle.yaw)) * vehicle.width);
    const double half_y = 0.5 * (
      std::abs(std::sin(vehicle.yaw)) * vehicle.length +
      std::abs(std::cos(vehicle.yaw)) * vehicle.width);
    const double gap = std::max(0.0, vehicle.x - half_x - parameters.ego_front_offset);
    const double velocity_x =
      std::cos(vehicle.yaw) * vehicle.velocity_x - std::sin(vehicle.yaw) * vehicle.velocity_y;
    const double closing_speed = ego_velocity - velocity_x;

    if (
      vehicle.x + half_x <= parameters.ego_front_offset ||
      std::abs(vehicle.y) - half_y > parameters.ego_half_width + parameters.lateral_margin ||
      gap > parameters.max_longitudinal_gap || closing_speed < parameters.min_closing_speed)
    {
      continue;
    }

    const double ttc = gap / closing_speed;
    if (ttc > parameters.max_ttc && gap > parameters.critical_longitudinal_gap) {
      continue;
    }

    const Risk candidate{index, gap, closing_speed, ttc};
    if (!best || candidate.ttc < best->ttc ||
      (candidate.ttc == best->ttc && candidate.gap < best->gap))
    {
      best = candidate;
    }
  }
  return best;
}

WarningStateMachine::WarningStateMachine(const Parameters & parameters)
: parameters_(parameters) {}

bool WarningStateMachine::update(
  const double now, const bool autonomous, const bool ego_stopped, const bool inputs_fresh,
  const bool raw_risk)
{
  constexpr double epsilon = 1e-9;
  if (!std::isfinite(now)) {
    reset();
    return false;
  }
  if (last_update_ && now < *last_update_) {
    reset();
    return false;
  }
  last_update_ = now;

  if (!autonomous || !ego_stopped || !inputs_fresh) {
    reset();
    return false;
  }

  if (!stopped_since_) {
    stopped_since_ = now;
  }
  if (now - *stopped_since_ + epsilon < parameters_.ego_stop_hold_time) {
    risk_since_.reset();
    clear_since_.reset();
    return active_;
  }

  if (raw_risk) {
    clear_since_.reset();
    if (!risk_since_) {
      risk_since_ = now;
    }
    if (now - *risk_since_ + epsilon >= parameters_.on_time_buffer) {
      active_ = true;
    }
  } else {
    risk_since_.reset();
    if (active_) {
      if (!clear_since_) {
        clear_since_ = now;
      }
      if (now - *clear_since_ + epsilon >= parameters_.off_time_buffer) {
        active_ = false;
        clear_since_.reset();
      }
    }
  }
  return active_;
}

void WarningStateMachine::reset()
{
  stopped_since_.reset();
  risk_since_.reset();
  clear_since_.reset();
  last_update_.reset();
  active_ = false;
}

}  // namespace didrive::front_collision_warning
