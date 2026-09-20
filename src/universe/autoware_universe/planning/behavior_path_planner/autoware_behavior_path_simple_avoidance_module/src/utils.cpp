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

#include <algorithm>
#include <cmath>
#include <limits>
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

ShiftLineArray mergeShiftLines(
  const ShiftLineArray & registered_lines, const ShiftLineArray & proposed_lines)
{
  ShiftLineArray merged = proposed_lines.empty() ? registered_lines : proposed_lines;
  if (!proposed_lines.empty() && !registered_lines.empty()) {
    const auto front_new_line = std::min_element(
      proposed_lines.begin(), proposed_lines.end(),
      [](const auto & lhs, const auto & rhs) { return lhs.start_idx < rhs.start_idx; });
    const auto min_start_idx = front_new_line->start_idx;
    const auto new_shift_length = front_new_line->end_shift_length;
    const auto new_shift_end_idx = front_new_line->end_idx;

    // Match static_obstacle_avoidance::addNewShiftLines(): keep the committed prefix, while
    // discarding a future line that would be overwritten by the new proposal. PathShifter applies
    // lines in start-index order, so retaining a later old line can otherwise alter the new shift.
    for (const auto & registered_line : registered_lines) {
      if (registered_line.start_idx >= min_start_idx) {
        continue;
      }

      if (registered_line.end_idx > new_shift_end_idx) {
        if (
          registered_line.end_shift_length > -1e-3 && new_shift_length > -1e-3 &&
          registered_line.end_shift_length < new_shift_length) {
          continue;
        }
        if (
          registered_line.end_shift_length < 1e-3 && new_shift_length < 1e-3 &&
          registered_line.end_shift_length > new_shift_length) {
          continue;
        }
      }

      merged.push_back(registered_line);
    }
  }

  std::stable_sort(merged.begin(), merged.end(), [](const auto & lhs, const auto & rhs) {
    if (lhs.start_idx != rhs.start_idx) {
      return lhs.start_idx < rhs.start_idx;
    }
    return lhs.end_idx < rhs.end_idx;
  });
  return merged;
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

PathWithLaneId make_safe_stop_path(
  const PathWithLaneId & preferred_path, const PathWithLaneId & fallback_path,
  const nav_msgs::msg::Odometry & odometry)
{
  constexpr double max_safe_stop_deceleration = 2.5;
  constexpr double minimum_forward_coverage = 0.1;
  const auto has_forward_coverage = [&](const auto & path) {
    return path.points.size() >= 2 &&
           autoware::motion_utils::calcSignedArcLength(
             path.points, odometry.pose.pose.position, path.points.back().point.pose.position) >=
             minimum_forward_coverage;
  };

  PathWithLaneId stopped_path;
  if (has_forward_coverage(preferred_path)) {
    stopped_path = preferred_path;
  } else if (has_forward_coverage(fallback_path)) {
    stopped_path = fallback_path;
  }
  if (stopped_path.points.empty()) {
    stopped_path.header = odometry.header;
    autoware_internal_planning_msgs::msg::PathPointWithLaneId stop_point;
    stop_point.point.pose = odometry.pose.pose;
    stop_point.point.longitudinal_velocity_mps = 0.0;
    stopped_path.points.push_back(stop_point);

    // Keep a valid segment for downstream consumers that require at least two path points.
    auto forward_stop_point = stop_point;
    constexpr double minimum_segment_length = 0.1;
    const double yaw = tf2::getYaw(odometry.pose.pose.orientation);
    forward_stop_point.point.pose.position.x += minimum_segment_length * std::cos(yaw);
    forward_stop_point.point.pose.position.y += minimum_segment_length * std::sin(yaw);
    stopped_path.points.push_back(forward_stop_point);
  }

  const auto ego_index = autoware::motion_utils::findNearestIndex(
    stopped_path.points, odometry.pose.pose.position);
  const double initial_speed = std::abs(odometry.twist.twist.linear.x);
  double previous_speed = initial_speed;
  for (size_t i = ego_index; i < stopped_path.points.size(); ++i) {
    const double distance_from_ego = std::max(
      0.0, autoware::motion_utils::calcSignedArcLength(
             stopped_path.points, odometry.pose.pose.position,
             stopped_path.points.at(i).point.pose.position));
    const double kinematic_speed = std::sqrt(std::max(
      0.0, initial_speed * initial_speed - 2.0 * max_safe_stop_deceleration * distance_from_ego));
    const double requested_speed = std::max(
      0.0, static_cast<double>(std::abs(stopped_path.points.at(i).point.longitudinal_velocity_mps)));
    const double safe_speed = std::min({previous_speed, requested_speed, kinematic_speed});
    stopped_path.points.at(i).point.longitudinal_velocity_mps = safe_speed;
    previous_speed = safe_speed;
  }
  for (size_t i = 0; i < ego_index && i < stopped_path.points.size(); ++i) {
    stopped_path.points.at(i).point.longitudinal_velocity_mps = 0.0;
  }
  return stopped_path;
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

double calcLateralTrackingError(
  const double expected_current_shift, const double actual_lateral_offset)
{
  return std::abs(expected_current_shift - actual_lateral_offset);
}

bool isLateralExecutionLagging(
  const double expected_current_shift, const double actual_lateral_offset, const double threshold)
{
  if (!std::isfinite(expected_current_shift) || !std::isfinite(actual_lateral_offset)) {
    return true;
  }
  // A vehicle that is already ahead of the planned shift is not lagging. Only measure error
  // toward the requested shift direction; this avoids turning overshoot into a false lag stop.
  if (std::abs(expected_current_shift) <= std::numeric_limits<double>::epsilon()) {
    return false;
  }
  const double signed_error = expected_current_shift - actual_lateral_offset;
  const double error_toward_target =
    signed_error * (expected_current_shift > 0.0 ? 1.0 : -1.0);
  return error_toward_target > std::max(0.0, threshold);
}

bool isWithinCommitmentWindow(
  const double distance_to_shift_start, const double commitment_lead_distance)
{
  return distance_to_shift_start <= std::max(0.0, commitment_lead_distance);
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
  result.min_shifting_distance = parameters.min_shifting_distance;
  result.ego_speed = ego_speed;
  result.jerk_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    std::abs(shift_length), parameters.shifting_lateral_jerk,
    std::max(ego_speed, parameters.min_shifting_speed));
  result.transition_distance = std::max(result.jerk_distance, result.min_shifting_distance);
  result.required_start_distance_before_front = std::max(
    parameters.avoidance_start_distance_before_object_front,
    parameters.longitudinal_margin_before_object_front + result.transition_distance);
  result.dist_to_avoid_start = target.longitudinal_distance - target.object_half_length -
                               result.required_start_distance_before_front;
  result.dist_to_shift_end = result.dist_to_avoid_start + result.transition_distance;
  result.dist_to_obstacle = target.longitudinal_distance - target.object_half_length -
                            parameters.longitudinal_margin_before_object_front;

  if (result.dist_to_avoid_start <= 0.0 || result.dist_to_shift_end > result.dist_to_obstacle) {
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
  if (std::abs(status.actual_ego_lateral_offset) > status.lateral_execution_threshold) {
    return false;
  }
  return true;
}

AvoidanceLifecycleDecision decideAvoidanceLifecycle(
  const AvoidanceLifecycleObservation & observation, const double path_generation_failure_timeout,
  const size_t completion_stable_count_required)
{
  AvoidanceLifecycleDecision decision;
  decision.next_state = observation.state;

  if (!observation.generation_succeeded) {
    // A candidate has not changed the vehicle path yet. If its first path generation fails,
    // cancel it and keep the upstream path instead of entering STOPPING without valid geometry.
    if (observation.state == AvoidanceLifecycleState::CANDIDATE) {
      decision.next_state = AvoidanceLifecycleState::IDLE;
      decision.action = AvoidanceLifecycleAction::CANCEL_CANDIDATE;
      return decision;
    }
    if (observation.has_continuous_previous_path) {
      decision.action = AvoidanceLifecycleAction::KEEP_LAST_VALID_PATH;
      return decision;
    }
    static_cast<void>(path_generation_failure_timeout);
    decision.next_state = AvoidanceLifecycleState::IDLE;
    decision.action = AvoidanceLifecycleAction::CANCEL_CANDIDATE;
    return decision;
  }

  if (observation.state == AvoidanceLifecycleState::CANDIDATE) {
    if (observation.commitment_detected) {
      decision.next_state = AvoidanceLifecycleState::COMMITTED;
    } else if (!observation.target_available) {
      decision.next_state = AvoidanceLifecycleState::IDLE;
      decision.action = AvoidanceLifecycleAction::CANCEL_CANDIDATE;
    }
    return decision;
  }

  if (
    observation.state == AvoidanceLifecycleState::COMMITTED &&
    (observation.target_expired || !observation.target_available)) {
    decision.next_state = AvoidanceLifecycleState::RETURNING;
    decision.action = AvoidanceLifecycleAction::KEEP_COMMITTED_PATH;
    return decision;
  }

  if (observation.state == AvoidanceLifecycleState::RETURNING) {
    if (!observation.return_to_center_complete) {
      return decision;
    }
    const size_t next_stable_count = observation.completion_stable_count + 1;
    if (next_stable_count >= completion_stable_count_required) {
      decision.next_state = AvoidanceLifecycleState::IDLE;
      decision.action = AvoidanceLifecycleAction::COMPLETE_MANEUVER;
      return decision;
    }
    decision.completion_stable_count = next_stable_count;
  }

  return decision;
}

}  // namespace autoware::behavior_path_planner
