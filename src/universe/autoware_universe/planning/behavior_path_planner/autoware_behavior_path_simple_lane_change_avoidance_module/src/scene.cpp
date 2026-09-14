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

#include "autoware/behavior_path_simple_lane_change_avoidance_module/scene.hpp"

#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_simple_lane_change_avoidance_module/utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <tf2/utils.h>

#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/strategies/buffer.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace autoware::behavior_path_planner
{
namespace
{
double getObjectHalfWidth(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return shape.dimensions.y / 2.0;
  }
  if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  }
  return 0.5;
}

double getObjectHalfLength(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return shape.dimensions.x / 2.0;
  }
  if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    return shape.dimensions.x / 2.0;
  }
  return 0.5;
}

bool isObjectOverlappingLanelets(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const lanelet::ConstLanelets & lanelets)
{
  return std::any_of(lanelets.begin(), lanelets.end(), [&](const auto & lanelet) {
    return utils::path_safety_checker::isPolygonOverlapLanelet(
      object, lanelet.polygon2d().basicPolygon());
  });
}

autoware_utils::Polygon2d createVehicleFootprint(
  const geometry_msgs::msg::Pose & pose, const PlannerData & planner_data)
{
  autoware_utils::Polygon2d footprint;
  footprint.outer() = autoware_utils::transform_vector(
    planner_data.parameters.vehicle_info.createFootprint(), autoware_utils::pose2transform(pose));
  boost::geometry::correct(footprint);
  return footprint;
}

bool isFootprintInsideLanelets(
  const autoware_utils::Polygon2d & footprint, const lanelet::ConstLanelets & lanelets,
  const double boundary_margin = 0.0)
{
  if (lanelets.empty() || !std::isfinite(boundary_margin) || boundary_margin < 0.0) {
    return false;
  }

  std::vector<autoware_utils::Polygon2d> safety_footprints;
  if (boundary_margin > 1e-6) {
    namespace strategy = boost::geometry::strategy::buffer;
    strategy::distance_symmetric<double> distance_strategy(boundary_margin);
    strategy::side_straight side_strategy;
    strategy::join_round join_strategy(8);
    strategy::end_round end_strategy(8);
    strategy::point_circle point_strategy(8);
    boost::geometry::buffer(
      footprint, safety_footprints, distance_strategy, side_strategy, join_strategy, end_strategy,
      point_strategy);
  }
  if (safety_footprints.empty()) {
    safety_footprints.push_back(footprint);
  }

  return std::all_of(safety_footprints.begin(), safety_footprints.end(), [&](const auto & polygon) {
    return std::all_of(polygon.outer().begin(), polygon.outer().end(), [&](const auto & point) {
      return std::any_of(lanelets.begin(), lanelets.end(), [&](const auto & lanelet) {
        return boost::geometry::covered_by(point, lanelet.polygon2d().basicPolygon());
      });
    });
  });
}

bool isFinitePath(const PathWithLaneId & path)
{
  if (path.points.size() < 2) {
    return false;
  }
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & point = path.points.at(i).point;
    const auto & position = point.pose.position;
    const auto & orientation = point.pose.orientation;
    if (
      !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
      !std::isfinite(point.longitudinal_velocity_mps)) {
      return false;
    }
    if (i > 0 &&
        autoware_utils::calc_distance2d(
          path.points.at(i - 1).point.pose.position, position) <= 1e-6) {
      return false;
    }
  }
  return true;
}

}  // namespace

SimpleLaneChangeAvoidanceModule::SimpleLaneChangeAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SimpleLCAvoidanceParameters> & parameters,
  const std::shared_ptr<TrailerConfigurationStore> & trailer_configuration_store,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: SceneModuleInterface{
    name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface},
  parameters_{parameters},
  trailer_configuration_store_{trailer_configuration_store}
{
}

void SimpleLaneChangeAvoidanceModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  current_lanelets_.clear();
  maneuver_base_lanelets_.clear();
  path_shifter_ = PathShifter{};
  prev_output_ = ShiftedPath{};
  active_target_.reset();
  active_adjacent_lane_.reset();
  lifecycle_state_ = LCAvoidanceLifecycleState::IDLE;
  completion_stable_count_ = 0;
  active_target_passed_ = false;
  path_generation_failure_started_.reset();
  route_id_.reset();
  debug_data_ = SimpleLCAvoidanceDebugData{};
  resetPathCandidate();
  resetPathReference();
}

void SimpleLaneChangeAvoidanceModule::processOnEntry()
{
  lifecycle_state_ = LCAvoidanceLifecycleState::CANDIDATE;
  completion_stable_count_ = 0;
  active_target_passed_ = false;
  if (planner_data_ && planner_data_->prev_route_id.has_value()) {
    route_id_ = autoware_utils_uuid::to_hex_string(*planner_data_->prev_route_id);
  }
  active_trailer_configuration_ = trailer_configuration_store_
                                    ? trailer_configuration_store_->snapshot()
                                    : ResolvedTrailerConfiguration{};
}

void SimpleLaneChangeAvoidanceModule::processOnExit() { initVariables(); }

bool SimpleLaneChangeAvoidanceModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  const auto target = detectTarget();
  // A detected obstacle must be claimed even when no safe lane-change path exists. This gives the
  // module a chance to publish a stop-before-target path instead of silently driving through it.
  return target.has_value();
}

bool SimpleLaneChangeAvoidanceModule::canTransitSuccessState()
{
  const auto target = active_target_;
  if (target.has_value()) {
    active_target_passed_ = active_target_passed_ || isActiveTargetPassed(*target);
  }
  const auto current_shift = getClosestShiftLength(prev_output_, getEgoPose().position);
  LCAvoidanceCompletionStatus status;
  status.has_active_target = target.has_value() && !active_target_passed_;
  status.is_active_target_passed = active_target_passed_;
  status.has_shift_lines = !path_shifter_.getShiftLines().empty();
  status.is_ego_on_shift_line = isEgoOnShiftLine();
  status.base_offset = path_shifter_.getBaseOffset();
  status.planned_shift = current_shift;
  status.actual_lateral_offset = getEgoLateralOffsetToReference();
  status.lateral_execution_threshold = parameters_->lateral_execution_threshold;
  const bool geometrically_complete =
    !status.has_active_target && status.is_active_target_passed && !status.has_shift_lines &&
    !status.is_ego_on_shift_line && std::abs(status.base_offset) < status.lateral_execution_threshold &&
    std::abs(status.planned_shift) < status.lateral_execution_threshold &&
    std::abs(status.actual_lateral_offset) < status.lateral_execution_threshold;
  if (geometrically_complete) {
    ++completion_stable_count_;
  } else {
    completion_stable_count_ = 0;
  }
  if (canCompleteManeuver(
        status, completion_stable_count_, parameters_->completion_stable_count)) {
    lifecycle_state_ = LCAvoidanceLifecycleState::IDLE;
    return true;
  }
  return false;
}

void SimpleLaneChangeAvoidanceModule::updateData()
{
  current_lanelets_.clear();
  if (getPreviousModuleOutput().path.points.size() < 2) {
    return;
  }

  if (planner_data_->prev_route_id.has_value()) {
    const auto current_route_id = autoware_utils_uuid::to_hex_string(*planner_data_->prev_route_id);
    if (route_id_.has_value() && *route_id_ != current_route_id) {
      lifecycle_state_ = LCAvoidanceLifecycleState::STOPPING;
      // A route update invalidates every index and lanelet used by the old shift. Do not carry
      // those indices into the new reference path; the next cycle may create a fresh maneuver
      // only after the new lanelet sequence has been acquired.
      path_shifter_ = PathShifter{};
      active_adjacent_lane_.reset();
      maneuver_base_lanelets_.clear();
      path_generation_failure_started_ = clock_->now();
    }
    route_id_ = current_route_id;
  }

  constexpr double resample_interval = 1.0;
  const auto backward_extended_path = extendBackwardLength(getPreviousModuleOutput().path);
  reference_path_ = utils::resamplePathWithSpline(backward_extended_path, resample_interval);
  path_shifter_.setPath(reference_path_);

  const auto & route_handler = planner_data_->route_handler;
  const auto & p = planner_data_->parameters;
  const auto reference_pose = planner_data_->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (route_handler->getClosestLaneletWithinRoute(reference_pose, &current_lane)) {
    current_lanelets_ = route_handler->getLaneletSequence(
      current_lane, reference_pose, p.backward_path_length, p.forward_path_length);
  }

  const size_t nearest_idx = planner_data_->findEgoIndex(path_shifter_.getReferencePath().points);
  path_shifter_.removeBehindShiftLineAndSetBaseOffset(nearest_idx);
}

std::optional<LCAvoidanceTarget> SimpleLaneChangeAvoidanceModule::detectTarget() const
{
  // SceneModuleManager calls isExecutionRequested() before the new scene has run updateData().
  // Use the upstream path for that first claim; once the scene is active, use the resampled and
  // backward-extended reference path maintained by updateData().
  const auto & detection_path = reference_path_.points.size() >= 2
                                  ? reference_path_
                                  : getPreviousModuleOutput().path;
  if (!planner_data_->dynamic_object || detection_path.points.empty()) {
    return std::nullopt;
  }

  const auto ego_pos = planner_data_->self_odometry->pose.pose.position;
  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;

  std::optional<LCAvoidanceTarget> nearest_target;
  double min_longitudinal = std::numeric_limits<double>::max();

  for (const auto & object : planner_data_->dynamic_object->objects) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const double speed = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);
    if (speed >= parameters_->th_moving_speed) {
      continue;
    }

    const auto & target_lanelets = maneuver_base_lanelets_.empty()
                                     ? current_lanelets_
                                     : maneuver_base_lanelets_;
    if (!target_lanelets.empty() && !isObjectOverlappingLanelets(object, target_lanelets)) {
      continue;
    }

    const auto nearest_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(detection_path.points, pose.position);
    const double longitudinal_distance =
      autoware::motion_utils::calcSignedArcLength(detection_path.points, ego_pos, pose.position);
    if (
      longitudinal_distance < parameters_->min_forward_distance ||
      longitudinal_distance > parameters_->max_forward_distance) {
      continue;
    }

    const double lateral_offset = autoware::motion_utils::calcLateralOffset(
      detection_path.points, pose.position, nearest_seg_idx);
    const double object_half_width = getObjectHalfWidth(object.shape);
    const double overlap = std::abs(lateral_offset) - object_half_width;
    if (overlap >= ego_half_width + parameters_->lateral_margin) {
      continue;
    }

    if (longitudinal_distance < min_longitudinal) {
      LCAvoidanceTarget target;
      target.pose = pose;
      target.longitudinal_distance = longitudinal_distance;
      target.lateral_offset = lateral_offset;
      target.object_half_width = object_half_width;
      target.object_half_length = getObjectHalfLength(object.shape);
      target.uuid = autoware_utils_uuid::to_hex_string(object.object_id);
      target.direction = getAvoidanceDirection(lateral_offset);
      nearest_target = target;
      min_longitudinal = longitudinal_distance;
    }
  }

  return nearest_target;
}

std::optional<LCAvoidanceTarget>
SimpleLaneChangeAvoidanceModule::detectAssociatedTargetByUuid() const
{
  if (
    !active_target_.has_value() || !planner_data_->dynamic_object || reference_path_.points.size() < 2)
  {
    return std::nullopt;
  }

  const auto ego_pos = planner_data_->self_odometry->pose.pose.position;
  const auto makeAssociatedTarget = [&](const auto & object) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto nearest_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(reference_path_.points, pose.position);
    LCAvoidanceTarget target = *active_target_;
    target.pose = pose;
    target.longitudinal_distance = autoware::motion_utils::calcSignedArcLength(
      reference_path_.points, ego_pos, pose.position);
    target.lateral_offset = autoware::motion_utils::calcLateralOffset(
      reference_path_.points, pose.position, nearest_seg_idx);
    target.object_half_width = getObjectHalfWidth(object.shape);
    target.object_half_length = getObjectHalfLength(object.shape);
    target.last_seen = clock_->now();
    return target;
  };

  const auto isWithinHeldTargetCorridor = [&](const auto & object) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto distance_to_held_target = std::hypot(
      pose.position.x - active_target_->pose.position.x,
      pose.position.y - active_target_->pose.position.y);
    // Tracker re-identification can move a static object's reported pose by a
    // few metres while preserving the same physical target. Keep this spatial
    // fallback local to the held target; it must not become a general object
    // detector or allow a distant object to inherit the maneuver.
    return distance_to_held_target <= 3.0;
  };

  // UUID is authoritative whenever the tracker preserves it.
  for (const auto & object : planner_data_->dynamic_object->objects) {
    if (
      autoware_utils_uuid::to_hex_string(object.object_id) == active_target_->uuid &&
      isWithinHeldTargetCorridor(object)) {
      return makeAssociatedTarget(object);
    }
  }

  // Fall back to the nearest spatially associated object when a tracker
  // re-creation changes the UUID. This is deliberately evaluated only after
  // the UUID pass, and only around the already committed target.
  const autoware_perception_msgs::msg::PredictedObject * spatial_match = nullptr;
  double nearest_distance = std::numeric_limits<double>::max();
  for (const auto & object : planner_data_->dynamic_object->objects) {
    if (!isWithinHeldTargetCorridor(object)) {
      continue;
    }
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto distance_to_held_target = std::hypot(
      pose.position.x - active_target_->pose.position.x,
      pose.position.y - active_target_->pose.position.y);
    if (distance_to_held_target < nearest_distance) {
      nearest_distance = distance_to_held_target;
      spatial_match = &object;
    }
  }
  if (spatial_match != nullptr) {
    return makeAssociatedTarget(*spatial_match);
  }
  return std::nullopt;
}

std::optional<LCAvoidanceTarget> SimpleLaneChangeAvoidanceModule::getActiveTargetOrHeldTarget()
{
  const auto associated = detectAssociatedTargetByUuid();
  const auto detected = associated.has_value() ? associated : detectTarget();
  if (detected.has_value()) {
    auto target = *detected;
    target.last_seen = clock_->now();
    const bool had_active_target = active_target_.has_value();
    if (!active_target_.has_value() || active_target_->uuid == target.uuid || active_target_passed_) {
      active_target_ = target;
      active_target_passed_ = false;
    } else {
      // Do not switch to a new obstacle while the committed obstacle has not been passed.
      const double association_distance = std::hypot(
        target.pose.position.x - active_target_->pose.position.x,
        target.pose.position.y - active_target_->pose.position.y);
      const bool lateral_association =
        std::abs(target.lateral_offset - active_target_->lateral_offset) <=
          parameters_->target_hold_lateral_hysteresis &&
        std::abs(target.longitudinal_distance - active_target_->longitudinal_distance) <= 1.5;
      if (association_distance < 1.5 || lateral_association) {
        active_target_->pose = target.pose;
        active_target_->longitudinal_distance = target.longitudinal_distance;
        active_target_->lateral_offset = target.lateral_offset;
        active_target_->object_half_width = target.object_half_width;
        active_target_->object_half_length = target.object_half_length;
        active_target_->last_seen = target.last_seen;
      }
    }
    if (!had_active_target && active_target_.has_value()) {
      RCLCPP_INFO(
        getLogger(),
        "[SIMPLE_LC_AVOIDANCE] active target locked uuid=%s lon=%.2fm lat=%.2fm",
        active_target_->uuid.c_str(), active_target_->longitudinal_distance,
        active_target_->lateral_offset);
    }
    return active_target_;
  }

  if (!active_target_.has_value()) {
    return std::nullopt;
  }
  active_target_passed_ = active_target_passed_ || isActiveTargetPassed(*active_target_);
  return active_target_;
}

bool SimpleLaneChangeAvoidanceModule::isActiveTargetPassed(const LCAvoidanceTarget & target) const
{
  if (reference_path_.points.size() < 2 || !planner_data_->self_odometry) {
    return false;
  }
  const auto signed_distance = autoware::motion_utils::calcSignedArcLength(
    reference_path_.points, planner_data_->self_odometry->pose.pose.position, target.pose.position);
  return signed_distance < -target.object_half_length;
}

bool SimpleLaneChangeAvoidanceModule::isEgoOnShiftLine() const
{
  if (reference_path_.points.empty()) {
    return false;
  }
  const auto ego_index = planner_data_->findEgoIndex(reference_path_.points);
  for (const auto & line : path_shifter_.getShiftLines()) {
    if (ego_index >= line.start_idx && ego_index <= line.end_idx) {
      return true;
    }
  }
  return false;
}

AdjacentLaneResult SimpleLaneChangeAvoidanceModule::findAdjacentLane(
  const LCAvoidanceTarget & target) const
{
  AdjacentLaneResult result;
  result.direction = target.direction;

  const auto & source_lanelets = maneuver_base_lanelets_.empty()
                                   ? current_lanelets_
                                   : maneuver_base_lanelets_;
  if (source_lanelets.empty()) {
    return result;
  }

  const auto & route_handler = planner_data_->route_handler;
  const auto & p = planner_data_->parameters;
  const auto reference_pose = planner_data_->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!maneuver_base_lanelets_.empty()) {
    current_lane = maneuver_base_lanelets_.front();
  } else if (!route_handler->getClosestLaneletWithinRoute(reference_pose, &current_lane)) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_LC_AVOIDANCE] adjacent lookup failed: current route lanelet unavailable "
      "current_lanelets=%zu direction=%s",
      source_lanelets.size(), target.direction == LCAvoidanceDirection::LEFT ? "left" : "right");
    return result;
  }

  std::optional<lanelet::ConstLanelet> adjacent_lane;
  if (target.direction == LCAvoidanceDirection::LEFT) {
    adjacent_lane = route_handler->getLeftLanelet(current_lane, false, false);
    if (!adjacent_lane) {
      // Closed low-speed sites may encode the usable passing strip as a shoulder lanelet rather
      // than a route lanelet. It is still accepted only after the same footprint and object
      // checks below; the fallback does not widen the drivable area by geometry.
      adjacent_lane = route_handler->getLeftLanelet(current_lane, false, true);
    }
  } else {
    adjacent_lane = route_handler->getRightLanelet(current_lane, false, false);
    if (!adjacent_lane) {
      adjacent_lane = route_handler->getRightLanelet(current_lane, false, true);
    }
  }

  if (!adjacent_lane) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_LC_AVOIDANCE] adjacent lookup failed: no %s lanelet current_id=%lld "
      "current_lanelets=%zu pose=(%.2f,%.2f)",
      target.direction == LCAvoidanceDirection::LEFT ? "left" : "right",
      static_cast<long long>(current_lane.id()), source_lanelets.size(), reference_pose.position.x,
      reference_pose.position.y);
    return result;
  }

  result.found = true;
  result.lanelet = *adjacent_lane;
  result.lanelet_sequence = route_handler->getLaneletSequence(
    *adjacent_lane, reference_pose, p.backward_path_length, p.forward_path_length);
  if (result.lanelet_sequence.empty()) {
    result.found = false;
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_LC_AVOIDANCE] adjacent lookup failed: lanelet sequence empty adjacent_id=%lld "
      "current_id=%lld backward=%.1f forward=%.1f",
      static_cast<long long>(adjacent_lane->id()), static_cast<long long>(current_lane.id()),
      p.backward_path_length, p.forward_path_length);
  }
  return result;
}

LaneShiftResult SimpleLaneChangeAvoidanceModule::calcLaneShift(
  const LCAvoidanceTarget & target) const
{
  LaneShiftResult result;
  result.adjacent_lane = findAdjacentLane(target);

  if (!result.adjacent_lane.found) {
    result.reason = InfeasibleReason::NO_ADJACENT_LANE;
    return result;
  }

  const size_t reference_idx = planner_data_->findEgoIndex(reference_path_.points);
  const auto & reference_pose = reference_path_.points.at(reference_idx).point.pose;
  const auto adjacent_arc =
    lanelet::utils::getArcCoordinates(result.adjacent_lane.lanelet_sequence, reference_pose);

  const double lane_center_shift =
    calcLaneShiftLength(adjacent_arc.distance, parameters_->lateral_margin);
  if (!isShiftLengthWithinLimit(lane_center_shift, parameters_->max_shift_length)) {
    result.reason = InfeasibleReason::NO_ROOM;
    return result;
  }
  result.shift_length = lane_center_shift;

  if (std::abs(result.shift_length) < 0.1) {
    result.reason = InfeasibleReason::NO_ADJACENT_LANE;
    return result;
  }

  result.reason = InfeasibleReason::NONE;
  return result;
}

  ShiftLineArray SimpleLaneChangeAvoidanceModule::buildShiftLines(
  const LCAvoidanceTarget & target, const LaneShiftResult & shift_result) const
{
  const auto ego_idx = planner_data_->findEgoIndex(reference_path_.points);
  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const double shift_length = shift_result.shift_length;

  const double dist_to_avoid_start = parameters_->min_prepare_distance;
  const double jerk_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    std::abs(shift_length), parameters_->shifting_lateral_jerk,
    std::max(ego_speed, parameters_->min_shifting_speed));
  const double dist_to_avoid_end =
    dist_to_avoid_start + std::max(jerk_distance, parameters_->min_shifting_distance);
  const double dist_to_return_start =
    target.longitudinal_distance + target.object_half_length + parameters_->return_distance_after_object;
  const double dist_to_return_end =
    dist_to_return_start + std::max(jerk_distance, parameters_->min_shifting_distance);

  ShiftLine avoid_shift;
  ShiftLine return_shift;
  avoid_shift.start_shift_length = getClosestShiftLength(prev_output_, getEgoPose().position);
  avoid_shift.end_shift_length = shift_length;
  avoid_shift.start_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_avoid_start);
  avoid_shift.end_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_avoid_end);
  return_shift.start_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_return_start);
  return_shift.end_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_return_end);
  if (
    avoid_shift.start_idx >= reference_path_.points.size() ||
    avoid_shift.end_idx >= reference_path_.points.size() ||
    return_shift.start_idx >= reference_path_.points.size() ||
    return_shift.end_idx >= reference_path_.points.size() ||
    avoid_shift.end_idx <= avoid_shift.start_idx + 1 ||
    return_shift.end_idx <= return_shift.start_idx + 1 ||
    return_shift.start_idx < avoid_shift.end_idx) {
    return {};
  }
  avoid_shift.start = reference_path_.points.at(avoid_shift.start_idx).point.pose;
  avoid_shift.end = reference_path_.points.at(avoid_shift.end_idx).point.pose;

  return_shift.start_shift_length = shift_length;
  return_shift.end_shift_length = 0.0;
  return_shift.start = reference_path_.points.at(return_shift.start_idx).point.pose;
  return_shift.end = reference_path_.points.at(return_shift.end_idx).point.pose;

  return {avoid_shift, return_shift};
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::passThrough(const InfeasibleReason reason) const
{
  debug_data_.last_reason = reason;
  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 1000, "[SIMPLE_LC_AVOIDANCE] pass-through reason=%s", toString(reason));

  auto output = getPreviousModuleOutput();
  if (output.path.points.empty() && !reference_path_.points.empty()) {
    output.path = reference_path_;
    output.reference_path = reference_path_;
  }
  if (output.path.points.empty() && !prev_output_.path.points.empty()) {
    output.path = prev_output_.path;
  }
  if (output.path.points.empty()) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId stop_point;
    stop_point.point.pose = planner_data_->self_odometry->pose.pose;
    stop_point.point.longitudinal_velocity_mps = 0.0;
    output.path.points.push_back(stop_point);
    auto forward_point = stop_point;
    const double yaw = tf2::getYaw(stop_point.point.pose.orientation);
    forward_point.point.pose.position.x += 0.1 * std::cos(yaw);
    forward_point.point.pose.position.y += 0.1 * std::sin(yaw);
    output.path.points.push_back(forward_point);
    output.path.header.frame_id = "map";
  }
  if (output.reference_path.points.empty()) {
    output.reference_path = output.path;
  }
  return output;
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::makeSafeStopOutput(
  const InfeasibleReason reason) const
{
  auto output = getPreviousModuleOutput();
  if (output.path.points.empty() && !reference_path_.points.empty()) {
    output.path = reference_path_;
  }
  if (output.path.points.empty() && !prev_output_.path.points.empty()) {
    output.path = prev_output_.path;
  }
  if (output.path.points.empty() && planner_data_ && planner_data_->self_odometry) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose = planner_data_->self_odometry->pose.pose;
    point.point.longitudinal_velocity_mps = 0.0;
    output.path.points.push_back(point);
    auto next = point;
    const auto yaw = tf2::getYaw(point.point.pose.orientation);
    next.point.pose.position.x += 0.1 * std::cos(yaw);
    next.point.pose.position.y += 0.1 * std::sin(yaw);
    output.path.points.push_back(next);
    output.path.header.frame_id = "map";
  }
  for (auto & point : output.path.points) {
    point.point.longitudinal_velocity_mps = 0.0;
  }
  if (!reference_path_.points.empty()) {
    output.reference_path = reference_path_;
  } else if (output.reference_path.points.empty()) {
    output.reference_path = output.path;
  }
  if (planning_factor_interface_ && !output.path.points.empty() && planner_data_->self_odometry) {
    const auto ego_index = std::min(
      planner_data_->findEgoIndex(output.path.points), output.path.points.size() - 1);
    planning_factor_interface_->add(
      output.path.points, planner_data_->self_odometry->pose.pose,
      output.path.points.at(ego_index).point.pose, PlanningFactor::STOP, SafetyFactorArray{}, true,
      0.0, 0.0, std::string("simple_lc_avoidance:") + toString(reason));
  }
  return output;
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::stopBeforeTarget(
  const LCAvoidanceTarget & target, const InfeasibleReason reason) const
{
  auto output = getPreviousModuleOutput();
  if (output.path.points.empty() && !reference_path_.points.empty()) {
    output.path = reference_path_;
  }
  if (output.path.points.empty() && !prev_output_.path.points.empty()) {
    output.path = prev_output_.path;
  }
  if (output.path.points.empty()) {
    return makeSafeStopOutput(reason);
  }

  const auto & vehicle = planner_data_->parameters;
  const double base_link_to_front = std::max(
    {0.0, vehicle.base_link2front, vehicle.wheel_base + vehicle.front_overhang,
     vehicle.vehicle_info.max_longitudinal_offset_m});
  const double stop_distance = target.longitudinal_distance - target.object_half_length -
                               base_link_to_front - parameters_->stop_margin_before_object;
  const auto ego_pose = planner_data_->self_odometry->pose.pose;
  std::optional<size_t> stop_index;
  if (stop_distance > 0.0) {
    stop_index = autoware::motion_utils::insertStopPoint(ego_pose, stop_distance, output.path.points);
  }
  const auto ego_index = planner_data_->findEgoIndex(output.path.points);
  const size_t first_stop_index = stop_index.value_or(ego_index);
  for (
    size_t i = std::min(first_stop_index, output.path.points.size()); i < output.path.points.size();
    ++i) {
    output.path.points.at(i).point.longitudinal_velocity_mps = 0.0;
  }
  if (planning_factor_interface_) {
    const auto factor_stop_index = std::min(first_stop_index, output.path.points.size() - 1);
    planning_factor_interface_->add(
      output.path.points, ego_pose, output.path.points.at(factor_stop_index).point.pose,
      PlanningFactor::STOP, SafetyFactorArray{}, true, 0.0, 0.0,
      std::string("simple_lc_avoidance:") + toString(reason));
  }
  if (!reference_path_.points.empty()) {
    output.reference_path = reference_path_;
  }
  debug_data_.last_reason = reason;
  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 1000,
    "[SIMPLE_LC_AVOIDANCE] stop before target uuid=%s reason=%s target_lon=%.2f",
    target.uuid.c_str(), toString(reason), target.longitudinal_distance);
  return output;
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::handlePathGenerationFailure(
  const InfeasibleReason reason)
{
  if (!path_generation_failure_started_.has_value()) {
    path_generation_failure_started_ = clock_->now();
  }
  lifecycle_state_ = LCAvoidanceLifecycleState::STOPPING;
  const auto failure_age = (clock_->now() - *path_generation_failure_started_).seconds();
  // A transient generator failure (for example while the upstream path is being replaced) may
  // safely reuse the last path that was already checked. Once the bounded grace period expires,
  // stop before the target instead of allowing an unbounded retry loop or publishing an empty
  // path.
  if (
    failure_age <= parameters_->path_generation_failure_timeout && prev_output_.path.points.size() >= 2 &&
    isFinitePath(prev_output_.path) && isGeneratedPathContinuous(prev_output_)) {
    auto output = getPreviousModuleOutput();
    output.path = prev_output_.path;
    if (output.reference_path.points.empty() && !reference_path_.points.empty()) {
      output.reference_path = reference_path_;
    }
    return output;
  }

  // Recreate the shifter so a failed set of shift lines cannot be retried forever on the next
  // planning cycle. The returned path is deliberately stopped because safety cannot be proven.
  path_shifter_ = PathShifter{};
  path_shifter_.setPath(reference_path_);
  return active_target_.has_value() ? stopBeforeTarget(*active_target_, reason)
                                    : makeSafeStopOutput(reason);
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::adjustDrivableArea(
  const ShiftedPath & path, const lanelet::ConstLanelets & adjacent_lanelets) const
{
  if (path.path.points.empty() || path.shift_length.empty()) {
    return makeSafeStopOutput(InfeasibleReason::PATH_GENERATION_FAILED);
  }

  auto out = getPreviousModuleOutput();
  const auto & p = planner_data_->parameters;

  auto output_path = path.path;
  const size_t current_seg_idx = planner_data_->findEgoSegmentIndex(output_path.points);
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  output_path.points = autoware::motion_utils::cropPoints(
    output_path.points, current_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  const auto & source_lanelets = maneuver_base_lanelets_.empty()
                                   ? current_lanelets_
                                   : maneuver_base_lanelets_;
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *planner_data_->route_handler, source_lanelets, adjacent_lanelets);
  const auto shorten_lanes = utils::cutOverlappedLanes(output_path, drivable_lanes);
  if (output_path.points.empty()) {
    return makeSafeStopOutput(InfeasibleReason::PATH_GENERATION_FAILED);
  }
  if (!isFinitePath(output_path)) {
    return makeSafeStopOutput(InfeasibleReason::PATH_GENERATION_FAILED);
  }
  // The current and adjacent route lanelets are the complete drivable region. Expanding by the
  // lateral shift would expose the shoulder or a third lane to downstream planners.
  const auto expanded_lanes = shorten_lanes;

  out.path = output_path;
  if (!reference_path_.points.empty()) {
    out.reference_path = reference_path_;
  }
  DrivableAreaInfo new_drivable_area;
  new_drivable_area.drivable_lanes = expanded_lanes;
  new_drivable_area.is_already_expanded = true;
  out.drivable_area_info = utils::combineDrivableAreaInfo(
    getPreviousModuleOutput().drivable_area_info, new_drivable_area);
  out.drivable_area_info.is_already_expanded = true;

  if (!path_shifter_.getShiftLines().empty() && !source_lanelets.empty()) {
    const auto & line = lifecycle_state_ == LCAvoidanceLifecycleState::RETURNING &&
                                path_shifter_.getShiftLines().size() > 1
                              ? path_shifter_.getShiftLines().at(1)
                              : path_shifter_.getShiftLines().front();
    auto [signal, ignored] = planner_data_->getBehaviorTurnSignalInfo(
      path, line, source_lanelets, path_shifter_.getBaseOffset(), true,
      std::abs(path_shifter_.getBaseOffset()) > parameters_->lateral_execution_threshold);
    static_cast<void>(ignored);

    // The generic shifted-path overload cannot identify this maneuver as a lane change on all
    // supported PlannerData versions.  On a closed site that can invert or suppress the signal
    // even though the selected adjacent lane is known.  Preserve the decider's timing/geometry,
    // but make the command direction explicit for the active maneuver; returning uses the
    // opposite indicator and completion falls back to the upstream signal on the next cycle.
    if (active_adjacent_lane_.has_value()) {
      const bool returning = lifecycle_state_ == LCAvoidanceLifecycleState::RETURNING;
      const bool borrow_left = active_adjacent_lane_->direction == LCAvoidanceDirection::LEFT;
      const bool signal_left = returning ? !borrow_left : borrow_left;
      signal.turn_signal.command = signal_left ? TurnIndicatorsCommand::ENABLE_LEFT
                                               : TurnIndicatorsCommand::ENABLE_RIGHT;
    }
    out.turn_signal_info = signal;
  }
  return out;
}

bool SimpleLaneChangeAvoidanceModule::isAdjacentLaneOccupied(
  const LCAvoidanceTarget & target, const lanelet::ConstLanelets & adjacent_lanelets) const
{
  if (!planner_data_->dynamic_object || adjacent_lanelets.empty()) {
    return false;
  }

  const double ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const double shift_distance = std::abs(calcLaneShiftLength(
    lanelet::utils::getArcCoordinates(
      adjacent_lanelets, reference_path_.points.at(planner_data_->findEgoIndex(reference_path_.points))
        .point.pose)
      .distance,
    parameters_->lateral_margin));
  const double jerk_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    shift_distance, parameters_->shifting_lateral_jerk,
    std::max(ego_speed, parameters_->min_shifting_speed));
  const double corridor_end = target.longitudinal_distance + target.object_half_length +
                              parameters_->return_distance_after_object +
                              std::max(jerk_distance, parameters_->min_shifting_distance);

  for (const auto & object : planner_data_->dynamic_object->objects) {
    const auto uuid = autoware_utils_uuid::to_hex_string(object.object_id);
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
    const bool is_target =
      uuid == target.uuid &&
      std::hypot(object_pose.position.x - target.pose.position.x,
                 object_pose.position.y - target.pose.position.y) < 1.5;
    if (is_target || !isObjectOverlappingLanelets(object, adjacent_lanelets)) {
      continue;
    }
    const auto longitudinal_distance = autoware::motion_utils::calcSignedArcLength(
      reference_path_.points, planner_data_->self_odometry->pose.pose.position,
      object_pose.position);
    if (longitudinal_distance >= -1.0 && longitudinal_distance <= corridor_end) {
      return true;
    }
  }
  return false;
}

InfeasibleReason SimpleLaneChangeAvoidanceModule::validatePathSafety(
  const ShiftedPath & path, const LCAvoidanceTarget & target,
  const lanelet::ConstLanelets & adjacent_lanelets) const
{
  if (path.path.points.size() < 2 || path.shift_length.size() != path.path.points.size()) {
    return InfeasibleReason::PATH_GENERATION_FAILED;
  }
  if (!isValidShiftLineGeometry(path_shifter_.getShiftLines(), reference_path_.points.size())) {
    return InfeasibleReason::PATH_GENERATION_FAILED;
  }
  const auto & source_lanelets = maneuver_base_lanelets_.empty()
                                   ? current_lanelets_
                                   : maneuver_base_lanelets_;
  if (source_lanelets.empty() || adjacent_lanelets.empty()) {
    return InfeasibleReason::FOOTPRINT_OUT_OF_BOUNDARY;
  }
  if (isAdjacentLaneOccupied(target, adjacent_lanelets)) {
    return InfeasibleReason::ADJACENT_LANE_OCCUPIED;
  }

  std::vector<autoware_utils::Polygon2d> object_polygons;
  if (planner_data_->dynamic_object) {
    object_polygons.reserve(planner_data_->dynamic_object->objects.size());
    for (const auto & object : planner_data_->dynamic_object->objects) {
      object_polygons.push_back(autoware_utils::to_polygon2d(object));
    }
  }

  const auto ego_index = planner_data_->findEgoIndex(path.path.points);
  double accumulated_distance = 0.0;
  size_t last_sample = ego_index;
  for (size_t i = ego_index; i < path.path.points.size(); ++i) {
    if (i > ego_index) {
      accumulated_distance += autoware_utils::calc_distance2d(
        path.path.points.at(i - 1).point.pose, path.path.points.at(i).point.pose);
    }
    if (i != ego_index && accumulated_distance < parameters_->footprint_sampling_interval &&
        i + 1 < path.path.points.size()) {
      continue;
    }
    accumulated_distance = 0.0;
    last_sample = i;
    const auto footprint = createVehicleFootprint(path.path.points.at(i).point.pose, *planner_data_);
    if (!isFootprintInsideLanelets(
          footprint, source_lanelets, parameters_->road_boundary_margin) &&
        !isFootprintInsideLanelets(
          footprint, adjacent_lanelets, parameters_->road_boundary_margin)) {
      // During the lane transition a footprint may straddle two lanelets. Check all lanelets as a
      // union rather than requiring it to fit inside one polygon.
      lanelet::ConstLanelets all_lanelets = source_lanelets;
      all_lanelets.insert(all_lanelets.end(), adjacent_lanelets.begin(), adjacent_lanelets.end());
      if (!isFootprintInsideLanelets(
            footprint, all_lanelets, parameters_->road_boundary_margin)) {
        return InfeasibleReason::FOOTPRINT_OUT_OF_BOUNDARY;
      }
    }
    for (const auto & object_polygon : object_polygons) {
      if (!boost::geometry::disjoint(footprint, object_polygon)) {
        return InfeasibleReason::VEHICLE_COLLISION;
      }
    }
  }
  if (last_sample != path.path.points.size() - 1) {
    const auto footprint = createVehicleFootprint(path.path.points.back().point.pose, *planner_data_);
    lanelet::ConstLanelets all_lanelets = source_lanelets;
    all_lanelets.insert(all_lanelets.end(), adjacent_lanelets.begin(), adjacent_lanelets.end());
    if (!isFootprintInsideLanelets(
          footprint, all_lanelets, parameters_->road_boundary_margin)) {
      return InfeasibleReason::FOOTPRINT_OUT_OF_BOUNDARY;
    }
  }
  lanelet::ConstLanelets drivable_lanelets = source_lanelets;
  drivable_lanelets.insert(
    drivable_lanelets.end(), adjacent_lanelets.begin(), adjacent_lanelets.end());
  const auto articulated_reason = validateArticulatedPath(path.path, drivable_lanelets);
  if (articulated_reason != InfeasibleReason::NONE) {
    return articulated_reason;
  }
  return InfeasibleReason::NONE;
}

InfeasibleReason SimpleLaneChangeAvoidanceModule::validateArticulatedPath(
  const PathWithLaneId & path, const lanelet::ConstLanelets & drivable_lanelets) const
{
  if (active_trailer_configuration_.geometries.empty()) {
    return InfeasibleReason::NONE;
  }
  if (path.points.empty() || drivable_lanelets.empty()) {
    return InfeasibleReason::PATH_GENERATION_FAILED;
  }

  const double sampling_interval = std::max(
    parameters_->footprint_sampling_interval, parameters_->trailer_footprint_sampling_interval);
  const size_t ego_index = planner_data_->findEgoIndex(path.points);
  std::vector<geometry_msgs::msg::Pose> sampled_poses;
  sampled_poses.reserve(path.points.size() - std::min(ego_index, path.points.size() - 1));
  const auto safe_ego_index = std::min(ego_index, path.points.size() - 1);
  sampled_poses.push_back(path.points.at(safe_ego_index).point.pose);
  double accumulated_distance = 0.0;
  for (size_t i = safe_ego_index + 1; i < path.points.size(); ++i) {
    accumulated_distance += autoware_utils::calc_distance2d(
      path.points.at(i - 1).point.pose, path.points.at(i).point.pose);
    if (accumulated_distance >= sampling_interval) {
      sampled_poses.push_back(path.points.at(i).point.pose);
      accumulated_distance = 0.0;
    }
  }
  if (
    autoware_utils::calc_distance2d(
      sampled_poses.back().position, path.points.back().point.pose.position) >
    1e-3) {
    sampled_poses.push_back(path.points.back().point.pose);
  }

  const auto articulated_path = predictArticulatedPath(
    sampled_poses, active_trailer_configuration_.geometries,
    parameters_->tractor_rear_axle_to_hitch);
  if (!articulated_path.articulation_valid) {
    return InfeasibleReason::ARTICULATION_LIMIT;
  }

  std::vector<autoware_utils::Polygon2d> object_polygons;
  if (planner_data_->dynamic_object) {
    object_polygons.reserve(planner_data_->dynamic_object->objects.size());
    for (const auto & object : planner_data_->dynamic_object->objects) {
      object_polygons.push_back(autoware_utils::to_polygon2d(object));
    }
  }

  for (const auto & articulated_pose : articulated_path.poses) {
    for (size_t trailer_index = 0; trailer_index < articulated_pose.trailers.size();
         ++trailer_index) {
      const auto footprint = createTrailerFootprint(
        articulated_pose.trailers.at(trailer_index),
        active_trailer_configuration_.geometries.at(trailer_index));
      if (!isFootprintInsideLanelets(
            footprint, drivable_lanelets, parameters_->road_boundary_margin)) {
        return InfeasibleReason::FOOTPRINT_OUT_OF_BOUNDARY;
      }
      for (const auto & object_polygon : object_polygons) {
        if (!boost::geometry::disjoint(footprint, object_polygon)) {
          return InfeasibleReason::TRAILER_COLLISION;
        }
      }
    }
  }
  return InfeasibleReason::NONE;
}

bool SimpleLaneChangeAvoidanceModule::isGeneratedPathContinuous(const ShiftedPath & path) const
{
  if (prev_output_.path.points.size() < 2 || path.path.points.size() < 2) {
    return true;
  }
  const auto current_ego_idx = planner_data_->findEgoIndex(path.path.points);
  const auto previous_idx = autoware::motion_utils::findNearestIndex(
    prev_output_.path.points, path.path.points.at(current_ego_idx).point.pose.position);
  const auto & current_pose = path.path.points.at(current_ego_idx).point.pose;
  const auto & previous_pose = prev_output_.path.points.at(previous_idx).point.pose;
  const double position_delta = autoware_utils::calc_distance2d(current_pose, previous_pose);
  const double yaw_delta = std::abs(tf2::getYaw(current_pose.orientation) -
                                    tf2::getYaw(previous_pose.orientation));
  return std::isfinite(position_delta) && std::isfinite(yaw_delta) && position_delta <= 2.0 &&
         std::abs(std::atan2(std::sin(yaw_delta), std::cos(yaw_delta))) <= 0.7;
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::plan()
{
  if (!areFeasibilityParametersValid(*parameters_)) {
    return active_target_.has_value() ? stopBeforeTarget(
                                           *active_target_, InfeasibleReason::TARGET_UNCERTAIN_STOP)
                                     : makeSafeStopOutput(InfeasibleReason::TARGET_UNCERTAIN_STOP);
  }

  if (reference_path_.points.size() < 2) {
    return active_target_.has_value() ? handlePathGenerationFailure(
                                           InfeasibleReason::PATH_GENERATION_FAILED)
                                     : passThrough(InfeasibleReason::NO_TARGET);
  }

  const auto target = getActiveTargetOrHeldTarget();
  // getActiveTargetOrHeldTarget() includes the UUID-priority association path.
  // Expiry must therefore use the final associated target's timestamp rather
  // than the result of the first-pass generic detector; otherwise a target
  // recovered by UUID in this cycle is stopped as if it were still missing.
  const bool target_expired = active_target_.has_value() &&
                              (clock_->now() - active_target_->last_seen).seconds() >
                                parameters_->target_lost_time_threshold;
  if (target.has_value()) {
    active_target_passed_ = active_target_passed_ || isActiveTargetPassed(*target);
    debug_data_.target = target;
  }

  if (target_expired && !active_target_passed_) {
    lifecycle_state_ = LCAvoidanceLifecycleState::STOPPING;
    return stopBeforeTarget(*active_target_, InfeasibleReason::TARGET_UNCERTAIN_STOP);
  }

  // PathShifter removes completed shift lines as the ego advances. A passed target therefore
  // needs an explicit recovery pass before the module can be considered complete.
  if (path_shifter_.getShiftLines().empty() && active_target_passed_) {
    if (std::abs(getEgoLateralOffsetToReference()) > parameters_->lateral_execution_threshold) {
      lifecycle_state_ = LCAvoidanceLifecycleState::RETURNING;
      const auto recovery = generateEgoAlignedReturnPath();
      if (!recovery.has_value()) {
        return handlePathGenerationFailure(InfeasibleReason::PATH_GENERATION_FAILED);
      }
      const auto adjacent_lanelets = active_adjacent_lane_.has_value()
                                       ? active_adjacent_lane_->lanelet_sequence
                                       : lanelet::ConstLanelets{};
      const auto safety_reason = validatePathSafety(*recovery, *active_target_, adjacent_lanelets);
      if (safety_reason != InfeasibleReason::NONE) {
        return makeSafeStopOutput(safety_reason);
      }
      return adjustDrivableArea(*recovery, adjacent_lanelets);
    }
    // Keep the held target until canTransitSuccessState() has observed the required number of
    // stable centered cycles.
    return passThrough(InfeasibleReason::NO_TARGET);
  }

  if (!shouldInitializeManeuver(path_shifter_.getShiftLines())) {
    ShiftedPath shifted_path;
    if (!path_shifter_.generate(&shifted_path) || shifted_path.path.points.empty()) {
      return handlePathGenerationFailure(InfeasibleReason::PATH_GENERATION_FAILED);
    }

    if (target.has_value()) {
      const auto safety_reason = validatePathSafety(
        shifted_path, *target,
        active_adjacent_lane_.has_value() ? active_adjacent_lane_->lanelet_sequence
                                          : lanelet::ConstLanelets{});
      if (safety_reason != InfeasibleReason::NONE) {
        lifecycle_state_ = LCAvoidanceLifecycleState::STOPPING;
        return stopBeforeTarget(*target, safety_reason);
      }
    }
    if (!isGeneratedPathContinuous(shifted_path)) {
      return handlePathGenerationFailure(InfeasibleReason::PATH_DISCONTINUITY);
    }

    setOrientation(&shifted_path.path);
    prev_output_ = shifted_path;
    debug_data_.last_reason = InfeasibleReason::NONE;
    debug_data_.path_shifter = std::make_shared<PathShifter>(path_shifter_);
    path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);
    if (parameters_->publish_debug_marker) {
      setDebugMarkersVisualization();
    } else {
      debug_marker_.markers.clear();
    }
    const auto adjacent_lanelets = active_adjacent_lane_.has_value()
                                     ? active_adjacent_lane_->lanelet_sequence
                                     : lanelet::ConstLanelets{};
    if (active_target_passed_) {
      lifecycle_state_ = LCAvoidanceLifecycleState::RETURNING;
    } else {
      lifecycle_state_ = LCAvoidanceLifecycleState::COMMITTED;
    }
    path_generation_failure_started_.reset();
    return adjustDrivableArea(shifted_path, adjacent_lanelets);
  }

  if (!target.has_value()) {
    if (active_target_.has_value() && active_target_passed_ &&
        std::abs(getEgoLateralOffsetToReference()) > parameters_->lateral_execution_threshold) {
      lifecycle_state_ = LCAvoidanceLifecycleState::RETURNING;
      const auto recovery = generateEgoAlignedReturnPath();
      if (recovery.has_value()) {
        const auto adjacent_lanelets = active_adjacent_lane_.has_value()
                                         ? active_adjacent_lane_->lanelet_sequence
                                         : lanelet::ConstLanelets{};
        return adjustDrivableArea(*recovery, adjacent_lanelets);
      }
      return handlePathGenerationFailure(InfeasibleReason::PATH_GENERATION_FAILED);
    }
    active_adjacent_lane_.reset();
    maneuver_base_lanelets_.clear();
    active_target_.reset();
    active_target_passed_ = false;
    lifecycle_state_ = LCAvoidanceLifecycleState::IDLE;
    return passThrough(InfeasibleReason::NO_TARGET);
  }

  active_target_ = *target;
  if (maneuver_base_lanelets_.empty() && !current_lanelets_.empty()) {
    maneuver_base_lanelets_ = current_lanelets_;
  }

  const auto shift_result = calcLaneShift(*target);
  if (shift_result.reason != InfeasibleReason::NONE) {
    lifecycle_state_ = LCAvoidanceLifecycleState::STOPPING;
    return stopBeforeTarget(*target, shift_result.reason);
  }
  active_adjacent_lane_ = shift_result.adjacent_lane;

  if (isAdjacentLaneOccupied(*target, active_adjacent_lane_->lanelet_sequence)) {
    lifecycle_state_ = LCAvoidanceLifecycleState::STOPPING;
    return stopBeforeTarget(*target, InfeasibleReason::ADJACENT_LANE_OCCUPIED);
  }

  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const auto feasibility_result =
    checkFeasibility(*target, shift_result.shift_length, *parameters_, ego_speed);
  if (feasibility_result.reason != InfeasibleReason::NONE) {
    lifecycle_state_ = LCAvoidanceLifecycleState::STOPPING;
    return stopBeforeTarget(*target, feasibility_result.reason);
  }

  const auto shift_lines = buildShiftLines(*target, shift_result);
  if (!isValidShiftLineGeometry(shift_lines, reference_path_.points.size())) {
    return handlePathGenerationFailure(InfeasibleReason::PATH_GENERATION_FAILED);
  }
  path_shifter_.setShiftLines(shift_lines);

  ShiftedPath shifted_path;
  if (!path_shifter_.generate(&shifted_path) || shifted_path.path.points.empty()) {
    return handlePathGenerationFailure(InfeasibleReason::PATH_GENERATION_FAILED);
  }
  const auto safety_reason = validatePathSafety(
    shifted_path, *target, shift_result.adjacent_lane.lanelet_sequence);
  if (safety_reason != InfeasibleReason::NONE) {
    return stopBeforeTarget(*target, safety_reason);
  }
  if (!isGeneratedPathContinuous(shifted_path)) {
    return handlePathGenerationFailure(InfeasibleReason::PATH_DISCONTINUITY);
  }

  setOrientation(&shifted_path.path);
  prev_output_ = shifted_path;
  lifecycle_state_ = LCAvoidanceLifecycleState::CANDIDATE;
  path_generation_failure_started_.reset();
  debug_data_.last_reason = InfeasibleReason::NONE;
  debug_data_.path_shifter = std::make_shared<PathShifter>(path_shifter_);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 2000,
    "[SIMPLE_LC_AVOIDANCE] lane change path generated shift=%.2f direction=%s target_lon=%.2f "
    "target_lat=%.2f",
    shift_result.shift_length,
    shift_result.adjacent_lane.direction == LCAvoidanceDirection::LEFT ? "left" : "right",
    target->longitudinal_distance, target->lateral_offset);

  if (parameters_->publish_debug_marker) {
    setDebugMarkersVisualization();
  } else {
    debug_marker_.markers.clear();
  }

  return adjustDrivableArea(shifted_path, shift_result.adjacent_lane.lanelet_sequence);
}

CandidateOutput SimpleLaneChangeAvoidanceModule::planCandidate() const
{
  CandidateOutput output;
  if (reference_path_.points.empty()) {
    output.path_candidate = getPreviousModuleOutput().path;
    return output;
  }

  auto path_shifter_local = path_shifter_;
  if (shouldInitializeManeuver(path_shifter_local.getShiftLines())) {
    if (!active_target_.has_value()) {
      output.path_candidate = getPreviousModuleOutput().path;
      return output;
    }

    const auto shift_result = calcLaneShift(*active_target_);
    if (shift_result.reason != InfeasibleReason::NONE) {
      output.path_candidate = getPreviousModuleOutput().path;
      return output;
    }
    const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
    if (
      checkFeasibility(*active_target_, shift_result.shift_length, *parameters_, ego_speed)
        .reason != InfeasibleReason::NONE) {
      output.path_candidate = getPreviousModuleOutput().path;
      return output;
    }
    const auto shift_lines = buildShiftLines(*active_target_, shift_result);
    if (!isValidShiftLineGeometry(shift_lines, reference_path_.points.size())) {
      output.path_candidate = getPreviousModuleOutput().path;
      return output;
    }
    path_shifter_local.setShiftLines(shift_lines);
  }

  ShiftedPath shifted_path;
  if (!path_shifter_local.generate(&shifted_path) || shifted_path.path.points.empty()) {
    output.path_candidate = getPreviousModuleOutput().path;
    return output;
  }
  setOrientation(&shifted_path.path);
  output.path_candidate = shifted_path.path;
  if (!shifted_path.shift_length.empty()) {
    const auto max_shift = std::max_element(
      shifted_path.shift_length.begin(), shifted_path.shift_length.end(),
      [](const double lhs, const double rhs) { return std::abs(lhs) < std::abs(rhs); });
    output.lateral_shift = *max_shift;
  }
  if (!path_shifter_local.getShiftLines().empty()) {
    const auto & line = path_shifter_local.getShiftLines().front();
    const auto ego_position = getEgoPose().position;
    output.start_distance_to_path_change = autoware::motion_utils::calcSignedArcLength(
      shifted_path.path.points, ego_position, line.start.position);
    output.finish_distance_to_path_change = autoware::motion_utils::calcSignedArcLength(
      shifted_path.path.points, ego_position, line.end.position);
  }
  return output;
}

PathWithLaneId SimpleLaneChangeAvoidanceModule::extendBackwardLength(
  const PathWithLaneId & original_path) const
{
  const auto longest_dist_to_shift_point = [&]() {
    double max_dist = 0.0;
    for (const auto & pnt : path_shifter_.getShiftLines()) {
      max_dist = std::max(max_dist, autoware_utils::calc_distance2d(getEgoPose(), pnt.start));
    }
    return max_dist;
  }();

  constexpr double extra_margin = 10.0;
  const auto backward_length = std::max(
    planner_data_->parameters.backward_path_length, longest_dist_to_shift_point + extra_margin);
  return extendBackwardPath(reference_path_, original_path, getEgoPose().position, backward_length);
}

double SimpleLaneChangeAvoidanceModule::getEgoLateralOffsetToReference() const
{
  if (reference_path_.points.size() < 2) {
    return 0.0;
  }
  const auto & ego_position = getEgoPose().position;
  const auto nearest_segment =
    autoware::motion_utils::findNearestSegmentIndex(reference_path_.points, ego_position);
  return autoware::motion_utils::calcLateralOffset(
    reference_path_.points, ego_position, nearest_segment);
}

std::optional<ShiftedPath> SimpleLaneChangeAvoidanceModule::generateEgoAlignedReturnPath()
{
  if (reference_path_.points.size() < 2) {
    return std::nullopt;
  }

  const double actual_offset = getEgoLateralOffsetToReference();
  const size_t start_idx = planner_data_->findEgoIndex(reference_path_.points);
  const double ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const double jerk_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    std::abs(actual_offset), parameters_->shifting_lateral_jerk,
    std::max(ego_speed, parameters_->min_shifting_speed));
  const double return_distance = std::max(jerk_distance, parameters_->min_shifting_distance);
  const size_t end_idx = utils::getIdxByArclength(reference_path_, start_idx, return_distance);
  if (start_idx <= 1 || end_idx <= start_idx + 1) {
    return std::nullopt;
  }

  ShiftLine measured_offset;
  measured_offset.start_shift_length = 0.0;
  measured_offset.end_shift_length = actual_offset;
  measured_offset.start_idx = 0;
  measured_offset.end_idx = start_idx;
  measured_offset.start = reference_path_.points.front().point.pose;
  measured_offset.end = reference_path_.points.at(start_idx).point.pose;

  ShiftLine return_shift;
  return_shift.start_shift_length = actual_offset;
  return_shift.end_shift_length = 0.0;
  return_shift.start_idx = start_idx;
  return_shift.end_idx = end_idx;
  return_shift.start = reference_path_.points.at(start_idx).point.pose;
  return_shift.end = reference_path_.points.at(end_idx).point.pose;

  PathShifter recovery_shifter;
  recovery_shifter.setPath(reference_path_);
  recovery_shifter.setShiftLines({measured_offset, return_shift});
  ShiftedPath recovery_path;
  if (!recovery_shifter.generate(&recovery_path) || recovery_path.path.points.empty()) {
    return std::nullopt;
  }

  setOrientation(&recovery_path.path);
  path_shifter_ = recovery_shifter;
  prev_output_ = recovery_path;
  debug_data_.path_shifter = std::make_shared<PathShifter>(path_shifter_);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);
  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 1000,
    "[SIMPLE_LC_AVOIDANCE] keeping module active for ego-aligned return: "
    "actual_offset=%.2fm return_distance=%.2fm",
    actual_offset, return_distance);
  return recovery_path;
}

void SimpleLaneChangeAvoidanceModule::setDebugMarkersVisualization() const
{
  using marker_utils::createShiftLineMarkerArray;
  debug_marker_.markers.clear();
  if (!debug_data_.path_shifter) {
    return;
  }
  const auto markers = createShiftLineMarkerArray(
    debug_data_.path_shifter->getShiftLines(), debug_data_.path_shifter->getBaseOffset(),
    "simple_lc_avoidance_shift_points", 0.1f, 0.6f, 1.0f, 0.4);
  autoware_utils::append_marker_array(markers, &debug_marker_);
}

}  // namespace autoware::behavior_path_planner
