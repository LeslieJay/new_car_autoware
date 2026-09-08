// Copyright 2026 BYD.
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

#include "agv_high_precision_reverse_controller/precision_reverse_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace agv_high_precision_reverse_controller
{

namespace
{
constexpr double kMinimumSegmentLength = 1.0e-6;
constexpr double kMinimumDt = 1.0e-3;
constexpr double kMaximumDt = 0.2;
}  // namespace

PrecisionReverseController::PrecisionReverseController(ControllerParameters parameters)
: parameters_(std::move(parameters))
{
}

bool PrecisionReverseController::setPath(std::vector<PathPoint> path, std::string * reason)
{
  if (path.size() < 2U) {
    if (reason) {
      *reason = "path must contain at least two points";
    }
    return false;
  }

  double cumulative_s = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!std::isfinite(path[i].x) || !std::isfinite(path[i].y) || !std::isfinite(path[i].yaw)) {
      if (reason) {
        *reason = "path contains a non-finite point";
      }
      return false;
    }
    if (i > 0U) {
      const double ds = std::hypot(path[i].x - path[i - 1U].x, path[i].y - path[i - 1U].y);
      if (ds < kMinimumSegmentLength) {
        if (reason) {
          *reason = "path contains duplicate adjacent points";
        }
        return false;
      }
      // A reverse-only path advances opposite to the vehicle's reference heading.
      const double motion_projection = (path[i].x - path[i - 1U].x) * std::cos(path[i - 1U].yaw) +
        (path[i].y - path[i - 1U].y) * std::sin(path[i - 1U].yaw);
      if (motion_projection > 0.05 * ds) {
        if (reason) {
          *reason = "path contains a forward segment";
        }
        return false;
      }
      cumulative_s += ds;
    }
    path[i].s = cumulative_s;
  }

  // Curvature uses signed vehicle velocity. Since s increases while reversing,
  // kappa = -d(yaw)/ds for the bicycle model yaw_rate = velocity * kappa.
  for (std::size_t i = 0; i < path.size(); ++i) {
    const std::size_t previous = i == 0U ? i : i - 1U;
    const std::size_t next = i + 1U < path.size() ? i + 1U : i;
    const double ds = path[next].s - path[previous].s;
    path[i].curvature =
      ds > kMinimumSegmentLength ? -normalizeAngle(path[next].yaw - path[previous].yaw) / ds : 0.0;
  }

  path_ = std::move(path);
  reset();
  return true;
}

TrackingResult PrecisionReverseController::compute(const VehicleState & state, double dt)
{
  TrackingResult result;
  if (path_.size() < 2U) {
    result.reason = "no path";
    return result;
  }
  if (
    !std::isfinite(state.x) || !std::isfinite(state.y) || !std::isfinite(state.yaw) ||
    !std::isfinite(state.speed))
  {
    result.reason = "vehicle state is not finite";
    return result;
  }

  const Projection projection = project(state);
  if (!projection.valid) {
    result.reason = "path projection failed";
    return result;
  }

  last_segment_index_ = projection.segment_index;
  // Reject large backward jumps caused by a self-intersection, but permit localization corrections.
  last_projected_s_ = std::max(projection.s, last_projected_s_ - 0.10);
  result.projected_s = last_projected_s_;
  result.remaining_s = std::max(0.0, path_.back().s - result.projected_s);
  result.segment_index = projection.segment_index;

  const double dx = state.x - projection.x;
  const double dy = state.y - projection.y;
  result.lateral_error = -std::sin(projection.yaw) * dx + std::cos(projection.yaw) * dy;
  result.heading_error = normalizeAngle(state.yaw - projection.yaw);

  const PathPoint & goal = path_.back();
  result.goal_distance = std::hypot(goal.x - state.x, goal.y - state.y);
  result.goal_yaw_error = std::abs(normalizeAngle(state.yaw - goal.yaw));
  const double goal_lateral_error =
    std::abs(-std::sin(goal.yaw) * (state.x - goal.x) + std::cos(goal.yaw) * (state.y - goal.y));
  result.goal_reached = result.goal_distance <= parameters_.goal_distance_tolerance &&
    result.goal_yaw_error <= parameters_.goal_yaw_tolerance &&
    std::abs(state.speed) <= parameters_.stop_speed_tolerance;

  const double past_goal =
    -(state.x - goal.x) * std::cos(goal.yaw) - (state.y - goal.y) * std::sin(goal.yaw);

  result.tracking_error = std::abs(result.lateral_error) > parameters_.max_lateral_error ||
    std::abs(result.heading_error) > parameters_.max_heading_error ||
    past_goal > parameters_.overshoot_tolerance;
  if (result.tracking_error) {
    result.valid = true;
    result.reason = "tracking error exceeds safety limit";
    result.acceleration = -parameters_.max_deceleration;
    return result;
  }
  if (result.goal_reached) {
    result.valid = true;
    result.reason = "goal reached";
    result.acceleration = -parameters_.max_deceleration;
    return result;
  }

  const double speed_magnitude = std::abs(state.speed);
  const double velocity_scale = std::max(speed_magnitude, parameters_.softening_speed);
  const double feedforward = std::atan(parameters_.wheel_base * projection.curvature);
  const double lateral_feedback =
    -std::atan2(parameters_.lateral_gain * result.lateral_error, velocity_scale);
  const double heading_feedback = parameters_.heading_gain * result.heading_error;
  const double raw_steering =
    feedforward + lateral_feedback + heading_feedback + parameters_.steering_offset;

  const double bounded_dt = std::clamp(dt, kMinimumDt, kMaximumDt);
  const double steering_step = parameters_.max_steering_rate * bounded_dt;
  result.steering_angle =
    std::clamp(raw_steering, last_steering_ - steering_step, last_steering_ + steering_step);
  result.steering_angle = std::clamp(
    result.steering_angle, -parameters_.max_steering_angle, parameters_.max_steering_angle);
  last_steering_ = result.steering_angle;

  const double braking_distance = std::max(0.0, result.remaining_s - parameters_.stop_margin);
  double desired_speed = std::min(
    parameters_.max_reverse_speed,
    std::sqrt(2.0 * parameters_.comfortable_deceleration * braking_distance));
  if (result.remaining_s < parameters_.final_approach_distance) {
    const double error_scale =
      std::clamp(1.0 - std::abs(result.lateral_error) / parameters_.max_lateral_error, 0.25, 1.0);
    desired_speed *= error_scale;
  }
  const double allowable_longitudinal_error = std::sqrt(
    std::max(
      0.0, parameters_.goal_distance_tolerance * parameters_.goal_distance_tolerance -
      goal_lateral_error * goal_lateral_error));
  if (result.remaining_s > allowable_longitudinal_error) {
    desired_speed = std::max(desired_speed, parameters_.min_creep_speed);
  } else {
    desired_speed = 0.0;
  }

  const double maximum_up_step = parameters_.max_acceleration * bounded_dt;
  const double maximum_down_step = parameters_.max_deceleration * bounded_dt;
  result.target_speed = std::clamp(
    desired_speed, std::max(0.0, last_target_speed_ - maximum_down_step),
    last_target_speed_ + maximum_up_step);
  last_target_speed_ = result.target_speed;

  const double speed_error = result.target_speed - speed_magnitude;
  const double candidate_integral = std::clamp(
    speed_integral_ + speed_error * bounded_dt, -parameters_.integral_limit,
    parameters_.integral_limit);
  const double raw_acceleration =
    parameters_.speed_kp * speed_error + parameters_.speed_ki * candidate_integral;
  result.acceleration =
    std::clamp(raw_acceleration, -parameters_.max_deceleration, parameters_.max_acceleration);
  if (
    raw_acceleration == result.acceleration ||
    (raw_acceleration > result.acceleration && speed_error < 0.0) ||
    (raw_acceleration < result.acceleration && speed_error > 0.0))
  {
    speed_integral_ = candidate_integral;
  }

  result.valid = true;
  result.reason = "tracking";
  return result;
}

void PrecisionReverseController::reset()
{
  last_segment_index_ = 0U;
  last_projected_s_ = 0.0;
  last_steering_ = 0.0;
  last_target_speed_ = 0.0;
  speed_integral_ = 0.0;
}

void PrecisionReverseController::clearPath()
{
  path_.clear();
  reset();
}

PrecisionReverseController::Projection PrecisionReverseController::project(
  const VehicleState & state) const
{
  Projection best;
  best.squared_distance = std::numeric_limits<double>::max();
  const std::size_t begin = last_segment_index_ > 8U ? last_segment_index_ - 8U : 0U;

  for (std::size_t i = begin; i + 1U < path_.size(); ++i) {
    const double sx = path_[i + 1U].x - path_[i].x;
    const double sy = path_[i + 1U].y - path_[i].y;
    const double length_squared = sx * sx + sy * sy;
    if (length_squared < kMinimumSegmentLength * kMinimumSegmentLength) {
      continue;
    }
    const double ratio = std::clamp(
      ((state.x - path_[i].x) * sx + (state.y - path_[i].y) * sy) / length_squared, 0.0, 1.0);
    const double x = path_[i].x + ratio * sx;
    const double y = path_[i].y + ratio * sy;
    const double squared_distance = (state.x - x) * (state.x - x) + (state.y - y) * (state.y - y);
    if (squared_distance < best.squared_distance) {
      best.valid = true;
      best.segment_index = i;
      best.ratio = ratio;
      best.x = x;
      best.y = y;
      best.yaw = interpolateAngle(path_[i].yaw, path_[i + 1U].yaw, ratio);
      best.curvature = path_[i].curvature + ratio * (path_[i + 1U].curvature - path_[i].curvature);
      best.s = path_[i].s + ratio * (path_[i + 1U].s - path_[i].s);
      best.squared_distance = squared_distance;
    }
  }
  return best;
}

double PrecisionReverseController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double PrecisionReverseController::interpolateAngle(double from, double to, double ratio)
{
  return normalizeAngle(from + ratio * normalizeAngle(to - from));
}

}  // namespace agv_high_precision_reverse_controller
