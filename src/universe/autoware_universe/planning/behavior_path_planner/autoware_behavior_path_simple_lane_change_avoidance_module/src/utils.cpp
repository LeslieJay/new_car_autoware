// Copyright 2025 BYD
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

#include "autoware/behavior_path_simple_lane_change_avoidance_module/utils.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace autoware::behavior_path_planner
{

void setOrientation(PathWithLaneId * path)
{
  for (size_t idx = 0; idx < path->points.size(); ++idx) {
    double angle = 0.0;
    auto & pt = path->points.at(idx);
    if (idx + 1 < path->points.size()) {
      const auto & next_pt = path->points.at(idx + 1);
      angle = std::atan2(
        next_pt.point.pose.position.y - pt.point.pose.position.y,
        next_pt.point.pose.position.x - pt.point.pose.position.x);
    } else if (idx != 0) {
      const auto & prev_pt = path->points.at(idx - 1);
      angle = std::atan2(
        pt.point.pose.position.y - prev_pt.point.pose.position.y,
        pt.point.pose.position.x - prev_pt.point.pose.position.x);
    }
    tf2::Quaternion yaw_quat;
    yaw_quat.setRPY(0, 0, angle);
    pt.point.pose.orientation = tf2::toMsg(yaw_quat);
  }
}

PathWithLaneId extendBackwardPath(
  const PathWithLaneId & previous_path, const PathWithLaneId & current_path,
  const geometry_msgs::msg::Point & ego_position, const double backward_length)
{
  if (previous_path.points.size() < 2 || current_path.points.size() < 2) {
    return current_path;
  }

  const auto current_ego_idx =
    autoware::motion_utils::findNearestIndex(current_path.points, ego_position);
  const auto previous_ego_idx = autoware::motion_utils::findNearestIndex(
    previous_path.points,
    autoware_utils_geometry::get_point(current_path.points.at(current_ego_idx)));

  const auto direction_at = [](const auto & points, const size_t idx) {
    const auto next_idx = idx + 1 < points.size() ? idx + 1 : idx;
    const auto prev_idx = next_idx == idx ? idx - 1 : idx;
    const auto & from = points.at(prev_idx).point.pose.position;
    const auto & to = points.at(next_idx).point.pose.position;
    return std::pair{to.x - from.x, to.y - from.y};
  };
  const auto previous_direction = direction_at(previous_path.points, previous_ego_idx);
  const auto current_direction = direction_at(current_path.points, current_ego_idx);
  if (
    previous_direction.first * current_direction.first +
      previous_direction.second * current_direction.second <=
    0.0) {
    return current_path;
  }

  auto clip_idx = previous_ego_idx;
  double accumulated_length = 0.0;
  while (clip_idx > 0 && accumulated_length < backward_length) {
    accumulated_length += autoware_utils_geometry::calc_distance2d(
      previous_path.points.at(clip_idx - 1), previous_path.points.at(clip_idx));
    --clip_idx;
  }

  auto extended_path = current_path;
  extended_path.points.clear();
  extended_path.points.insert(
    extended_path.points.end(), previous_path.points.begin() + clip_idx,
    previous_path.points.begin() + previous_ego_idx);
  extended_path.points.insert(
    extended_path.points.end(), current_path.points.begin() + current_ego_idx,
    current_path.points.end());
  return extended_path;
}

double getClosestShiftLength(
  const ShiftedPath & shifted_path, const geometry_msgs::msg::Point & ego_point)
{
  if (shifted_path.shift_length.empty() || shifted_path.path.points.empty()) {
    return 0.0;
  }
  const auto closest =
    autoware::motion_utils::findNearestIndex(shifted_path.path.points, ego_point);
  return shifted_path.shift_length.at(std::min(closest, shifted_path.shift_length.size() - 1));
}

LCAvoidanceDirection getAvoidanceDirection(const double lateral_offset)
{
  // Object on the left (positive offset) → borrow the right adjacent lane.
  return lateral_offset >= 0.0 ? LCAvoidanceDirection::RIGHT : LCAvoidanceDirection::LEFT;
}

double applyLaneShiftMargin(const double raw_shift_length, const double lateral_margin)
{
  if (raw_shift_length > 0.0) {
    return raw_shift_length + lateral_margin;
  }
  if (raw_shift_length < 0.0) {
    return raw_shift_length - lateral_margin;
  }
  return raw_shift_length;
}

double calcLaneShiftLength(
  const double adjacent_lane_distance_from_reference, const double lateral_margin)
{
  return applyLaneShiftMargin(-adjacent_lane_distance_from_reference, lateral_margin);
}

double limitLaneShiftLength(const double shift_length, const double max_shift_length)
{
  return std::clamp(shift_length, -std::abs(max_shift_length), std::abs(max_shift_length));
}

bool shouldInitializeManeuver(const ShiftLineArray & shift_lines)
{
  return shift_lines.empty();
}

bool canCompleteManeuver(
  const bool has_target, const ShiftLineArray & shift_lines, const double current_shift,
  const double actual_lateral_offset, const double zero_threshold)
{
  return !has_target && shift_lines.empty() && std::abs(current_shift) < zero_threshold &&
         std::abs(actual_lateral_offset) < zero_threshold;
}

bool canCompleteManeuver(
  const LCAvoidanceCompletionStatus & status, const size_t stable_count,
  const size_t required_stable_count)
{
  const auto threshold = std::abs(status.lateral_execution_threshold);
  const bool geometrically_complete =
    !status.has_active_target && status.is_active_target_passed && !status.has_shift_lines &&
    !status.is_ego_on_shift_line && std::abs(status.base_offset) < threshold &&
    std::abs(status.planned_shift) < threshold &&
    std::abs(status.actual_lateral_offset) < threshold;
  return geometrically_complete && stable_count >= std::max<size_t>(required_stable_count, 1U);
}

bool isShiftLengthWithinLimit(const double required_shift_length, const double max_shift_length)
{
  return std::isfinite(required_shift_length) && std::isfinite(max_shift_length) &&
         max_shift_length > 0.0 && std::abs(required_shift_length) <= std::abs(max_shift_length) + 1e-6;
}

bool areFeasibilityParametersValid(const SimpleLCAvoidanceParameters & parameters)
{
  return std::isfinite(parameters.th_moving_speed) && parameters.th_moving_speed >= 0.0 &&
         std::isfinite(parameters.min_forward_distance) &&
         std::isfinite(parameters.max_forward_distance) &&
         parameters.min_forward_distance >= 0.0 &&
         parameters.max_forward_distance >= parameters.min_forward_distance &&
         std::isfinite(parameters.lateral_margin) && parameters.lateral_margin >= 0.0 &&
         std::isfinite(parameters.max_shift_length) && parameters.max_shift_length > 0.0 &&
         std::isfinite(parameters.min_prepare_distance) && parameters.min_prepare_distance >= 0.0 &&
         std::isfinite(parameters.min_shifting_distance) && parameters.min_shifting_distance > 0.0 &&
         std::isfinite(parameters.shifting_lateral_jerk) && parameters.shifting_lateral_jerk > 0.0 &&
         std::isfinite(parameters.min_shifting_speed) && parameters.min_shifting_speed >= 0.0 &&
         std::isfinite(parameters.return_distance_after_object) &&
         parameters.return_distance_after_object >= 0.0 &&
         std::isfinite(parameters.target_lost_time_threshold) &&
         parameters.target_lost_time_threshold >= 0.0 &&
         std::isfinite(parameters.target_hold_lateral_hysteresis) &&
         parameters.target_hold_lateral_hysteresis >= 0.0 &&
         std::isfinite(parameters.road_boundary_margin) && parameters.road_boundary_margin >= 0.0 &&
         std::isfinite(parameters.stop_margin_before_object) &&
         parameters.stop_margin_before_object >= 0.0 &&
         std::isfinite(parameters.path_generation_failure_timeout) &&
         parameters.path_generation_failure_timeout >= 0.0 &&
         std::isfinite(parameters.lateral_execution_threshold) &&
         parameters.lateral_execution_threshold > 0.0 &&
         std::isfinite(parameters.footprint_sampling_interval) &&
         parameters.footprint_sampling_interval > 0.0 &&
         !parameters.trailer_configuration_topic.empty() &&
         std::isfinite(parameters.tractor_rear_axle_to_hitch) &&
         parameters.tractor_rear_axle_to_hitch >= 0.0 &&
         std::isfinite(parameters.trailer_footprint_sampling_interval) &&
         parameters.trailer_footprint_sampling_interval > 0.0 &&
         std::isfinite(parameters.trailer_lateral_search_resolution) &&
         parameters.trailer_lateral_search_resolution > 0.0 &&
         std::isfinite(parameters.trailer_return_search_resolution) &&
         parameters.trailer_return_search_resolution > 0.0 &&
         std::isfinite(parameters.trailer_max_extra_return_distance) &&
         parameters.trailer_max_extra_return_distance >= 0.0 &&
         std::isfinite(parameters.trailer_max_planning_time_ms) &&
         parameters.trailer_max_planning_time_ms > 0.0 &&
         std::isfinite(parameters.trailer_stationary_speed_threshold) &&
         parameters.trailer_stationary_speed_threshold >= 0.0 &&
         parameters.completion_stable_count > 0;
}

bool isValidShiftLineGeometry(const ShiftLineArray & shift_lines, const size_t reference_path_size)
{
  if (shift_lines.empty() || reference_path_size < 2) {
    return false;
  }
  size_t previous_end = 0;
  for (const auto & line : shift_lines) {
    if (line.start_idx >= reference_path_size || line.end_idx >= reference_path_size ||
        line.end_idx <= line.start_idx + 1 || line.start_idx < previous_end ||
        !std::isfinite(line.start_shift_length) || !std::isfinite(line.end_shift_length)) {
      return false;
    }
    previous_end = line.end_idx;
  }
  return true;
}

FeasibilityResult checkFeasibility(
  const LCAvoidanceTarget & target, const double shift_length,
  const SimpleLCAvoidanceParameters & parameters, const double ego_speed)
{
  FeasibilityResult result;
  result.min_prepare_distance = parameters.min_prepare_distance;
  result.min_shifting_distance = parameters.min_shifting_distance;
  result.ego_speed = ego_speed;
  if (!areFeasibilityParametersValid(parameters) || !std::isfinite(shift_length) ||
      !isShiftLengthWithinLimit(shift_length, parameters.max_shift_length)) {
    result.reason = InfeasibleReason::NO_ROOM;
    return result;
  }
  result.jerk_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    std::abs(shift_length), parameters.shifting_lateral_jerk,
    std::max(ego_speed, parameters.min_shifting_speed));
  result.dist_to_shift_end =
    result.min_prepare_distance + std::max(result.jerk_distance, result.min_shifting_distance);
  // lateral_margin is reused as the longitudinal buffer before the obstacle front edge
  result.dist_to_obstacle =
    target.longitudinal_distance - target.object_half_length - parameters.lateral_margin;

  if (result.dist_to_shift_end > result.dist_to_obstacle) {
    result.reason = InfeasibleReason::INSUFFICIENT_DISTANCE;
    return result;
  }

  result.reason = InfeasibleReason::NONE;
  return result;
}

}  // namespace autoware::behavior_path_planner
