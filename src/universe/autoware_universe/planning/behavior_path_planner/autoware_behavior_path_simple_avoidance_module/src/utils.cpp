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

#include "autoware/behavior_path_simple_avoidance_module/utils.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <tf2/utils.hpp>

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
  if (previous_path.points.empty() || current_path.points.empty()) {
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
  if (previous_path.points.size() < 2 || current_path.points.size() < 2) {
    return current_path;
  }
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
  if (shifted_path.shift_length.empty()) {
    return 0.0;
  }
  const auto closest =
    autoware::motion_utils::findNearestIndex(shifted_path.path.points, ego_point);
  return shifted_path.shift_length.at(closest);
}

ShiftLengthResult calcShiftLength(
  const AvoidanceTarget & target, const SimpleAvoidanceParameters & parameters,
  const double ego_half_width)
{
  ShiftLengthResult result;
  // Use the object edge facing the reference path, not the far edge.
  // |lateral_offset| + object_half_width overestimates when the object is wide and encroaches the
  // path.
  const double object_near_edge = target.lateral_offset >= 0.0
                                    ? target.lateral_offset - target.object_half_width
                                    : target.lateral_offset + target.object_half_width;
  result.required_clearance =
    std::max(0.0, std::abs(object_near_edge)) + ego_half_width + parameters.lateral_margin;

  if (target.lateral_offset >= 0.0) {
    result.requested_shift_length = -result.required_clearance;
  } else {
    result.requested_shift_length = result.required_clearance;
  }
  result.shift_length = result.requested_shift_length;

  if (std::abs(result.shift_length) > parameters.max_shift_length) {
    const double sign = result.shift_length >= 0.0 ? 1.0 : -1.0;
    result.shift_length = sign * parameters.max_shift_length;
    result.remaining_gap = result.required_clearance - parameters.max_shift_length;
    if (result.remaining_gap > 0.0) {
      result.reason = InfeasibleReason::NO_ROOM;
      return result;
    }
  }

  result.reason = InfeasibleReason::NONE;
  return result;
}

FeasibilityResult checkFeasibility(
  const AvoidanceTarget & target, const double shift_length,
  const SimpleAvoidanceParameters & parameters, const double ego_speed)
{
  FeasibilityResult result;
  result.min_prepare_distance = parameters.min_prepare_distance;
  result.min_shifting_distance = parameters.min_shifting_distance;
  result.ego_speed = ego_speed;
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

bool isTargetWithinOverlap(
  const double lateral_offset, const double object_half_width, const double ego_half_width,
  const double lateral_margin, const double hysteresis)
{
  const double overlap = std::abs(lateral_offset) - object_half_width;
  return overlap < ego_half_width + lateral_margin + hysteresis;
}

bool isTargetPassed(const AvoidanceTarget & target, const SimpleAvoidanceParameters & parameters)
{
  return target.longitudinal_distance <
         -(target.object_half_length + parameters.return_distance_after_object);
}

bool isTargetHoldExpired(
  const AvoidanceTarget & target, const rclcpp::Time & now, const double lost_time_threshold)
{
  if (lost_time_threshold <= 0.0) {
    return true;
  }
  return (now - target.last_seen).seconds() > lost_time_threshold;
}

bool canCompleteAvoidance(const AvoidanceCompletionStatus & status)
{
  if (status.has_active_target && !status.is_active_target_passed) {
    return false;
  }
  if (status.has_shift_lines || status.is_ego_on_shift_line) {
    return false;
  }
  if (std::abs(status.base_offset) > status.lateral_execution_threshold) {
    return false;
  }
  if (std::abs(status.ego_shift) > status.lateral_execution_threshold) {
    return false;
  }
  return true;
}

}  // namespace autoware::behavior_path_planner
