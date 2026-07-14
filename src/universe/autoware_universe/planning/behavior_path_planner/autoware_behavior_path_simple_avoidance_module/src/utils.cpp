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
  // |lateral_offset| + object_half_width overestimates when the object is wide and encroaches the path.
  const double object_near_edge =
    target.lateral_offset >= 0.0
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

}  // namespace autoware::behavior_path_planner
