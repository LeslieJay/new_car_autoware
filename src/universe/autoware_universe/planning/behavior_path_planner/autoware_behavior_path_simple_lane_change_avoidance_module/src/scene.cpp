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
#include "autoware/behavior_path_simple_lane_change_avoidance_module/utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

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

std::vector<DrivableLanes> generateDrivableLanesWithAdjacent(
  const lanelet::ConstLanelets & current_lanelets,
  const lanelet::ConstLanelets & adjacent_lanelets, const LCAvoidanceDirection direction)
{
  std::vector<DrivableLanes> drivable_lanes;
  drivable_lanes.reserve(current_lanelets.size());

  for (size_t i = 0; i < current_lanelets.size(); ++i) {
    DrivableLanes lane;
    lane.left_lane = current_lanelets.at(i);
    lane.right_lane = current_lanelets.at(i);

    if (i < adjacent_lanelets.size()) {
      if (direction == LCAvoidanceDirection::LEFT) {
        lane.left_lane = adjacent_lanelets.at(i);
      } else {
        lane.right_lane = adjacent_lanelets.at(i);
      }
    }

    drivable_lanes.push_back(lane);
  }

  if (adjacent_lanelets.size() > current_lanelets.size()) {
    for (size_t i = current_lanelets.size(); i < adjacent_lanelets.size(); ++i) {
      DrivableLanes lane;
      lane.left_lane = adjacent_lanelets.at(i);
      lane.right_lane = adjacent_lanelets.at(i);
      drivable_lanes.push_back(lane);
    }
  }

  return drivable_lanes;
}
}  // namespace

SimpleLaneChangeAvoidanceModule::SimpleLaneChangeAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SimpleLCAvoidanceParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface)
: SceneModuleInterface{
    name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface},
  parameters_{parameters}
{
}

void SimpleLaneChangeAvoidanceModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  current_lanelets_.clear();
  path_shifter_ = PathShifter{};
  prev_output_ = ShiftedPath{};
  active_target_.reset();
  active_adjacent_lane_.reset();
  debug_data_ = SimpleLCAvoidanceDebugData{};
  resetPathCandidate();
  resetPathReference();
}

void SimpleLaneChangeAvoidanceModule::processOnEntry() {}

void SimpleLaneChangeAvoidanceModule::processOnExit() { initVariables(); }

bool SimpleLaneChangeAvoidanceModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }
  return detectTarget().has_value();
}

bool SimpleLaneChangeAvoidanceModule::canTransitSuccessState()
{
  constexpr double zero_threshold = 0.05;
  const auto target = detectTarget();
  const auto current_shift = getClosestShiftLength(prev_output_, getEgoPose().position);
  return canCompleteManeuver(
    target.has_value(), path_shifter_.getShiftLines(), current_shift, zero_threshold);
}

void SimpleLaneChangeAvoidanceModule::updateData()
{
  if (getPreviousModuleOutput().path.points.size() < 2) {
    return;
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
  if (!planner_data_->dynamic_object || reference_path_.points.empty()) {
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

    if (!current_lanelets_.empty() && !isObjectOverlappingLanelets(object, current_lanelets_)) {
      continue;
    }

    const auto nearest_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(reference_path_.points, pose.position);
    const double longitudinal_distance =
      autoware::motion_utils::calcSignedArcLength(reference_path_.points, ego_pos, pose.position);
    if (
      longitudinal_distance < parameters_->min_forward_distance ||
      longitudinal_distance > parameters_->max_forward_distance) {
      continue;
    }

    const double lateral_offset = autoware::motion_utils::calcLateralOffset(
      reference_path_.points, pose.position, nearest_seg_idx);
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

AdjacentLaneResult SimpleLaneChangeAvoidanceModule::findAdjacentLane(
  const LCAvoidanceTarget & target) const
{
  AdjacentLaneResult result;
  result.direction = target.direction;

  if (current_lanelets_.empty()) {
    return result;
  }

  const auto & route_handler = planner_data_->route_handler;
  const auto & p = planner_data_->parameters;
  const auto reference_pose = planner_data_->self_odometry->pose.pose;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(reference_pose, &current_lane)) {
    return result;
  }

  std::optional<lanelet::ConstLanelet> adjacent_lane;
  if (target.direction == LCAvoidanceDirection::LEFT) {
    adjacent_lane = route_handler->getLeftLanelet(current_lane, false, false);
  } else {
    adjacent_lane = route_handler->getRightLanelet(current_lane, false, false);
  }

  if (!adjacent_lane) {
    return result;
  }

  if (!route_handler->isRouteLanelet(*adjacent_lane)) {
    return result;
  }

  result.found = true;
  result.lanelet = *adjacent_lane;
  result.lanelet_sequence = route_handler->getLaneletSequence(
    *adjacent_lane, reference_pose, p.backward_path_length, p.forward_path_length);
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

  const auto ego_pose = getEgoPose();
  const auto current_arc = lanelet::utils::getArcCoordinates(current_lanelets_, ego_pose);
  const auto adjacent_arc =
    lanelet::utils::getArcCoordinates(result.adjacent_lane.lanelet_sequence, ego_pose);

  result.shift_length =
    calcLaneShiftLength(current_arc.distance, adjacent_arc.distance, parameters_->lateral_margin);

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
  avoid_shift.start_shift_length = getClosestShiftLength(prev_output_, getEgoPose().position);
  avoid_shift.end_shift_length = shift_length;
  avoid_shift.start_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_avoid_start);
  avoid_shift.end_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_avoid_end);
  avoid_shift.start = reference_path_.points.at(avoid_shift.start_idx).point.pose;
  avoid_shift.end = reference_path_.points.at(avoid_shift.end_idx).point.pose;

  ShiftLine return_shift;
  return_shift.start_shift_length = shift_length;
  return_shift.end_shift_length = 0.0;
  return_shift.start_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_return_start);
  return_shift.end_idx = utils::getIdxByArclength(reference_path_, ego_idx, dist_to_return_end);
  return_shift.start = reference_path_.points.at(return_shift.start_idx).point.pose;
  return_shift.end = reference_path_.points.at(return_shift.end_idx).point.pose;

  return {avoid_shift, return_shift};
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::passThrough(const InfeasibleReason reason) const
{
  debug_data_.last_reason = reason;
  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 1000, "[SIMPLE_LC_AVOIDANCE] pass-through reason=%s", toString(reason));
  return getPreviousModuleOutput();
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::adjustDrivableArea(
  const ShiftedPath & path, const lanelet::ConstLanelets & adjacent_lanelets) const
{
  BehaviorModuleOutput out;
  const auto & p = planner_data_->parameters;
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  const auto itr = std::minmax_element(path.shift_length.begin(), path.shift_length.end());
  constexpr double threshold = 0.1;
  constexpr double margin = 0.5;
  const double left_offset = std::max(
    *itr.second + (*itr.second > threshold ? margin : 0.0), dp.drivable_area_left_bound_offset);
  const double right_offset = -std::min(
    *itr.first - (*itr.first < -threshold ? margin : 0.0), -dp.drivable_area_right_bound_offset);

  auto output_path = path.path;
  const size_t current_seg_idx = planner_data_->findEgoSegmentIndex(output_path.points);
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  output_path.points = autoware::motion_utils::cropPoints(
    output_path.points, current_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);

  const auto direction = active_adjacent_lane_.has_value()
                           ? active_adjacent_lane_->direction
                           : LCAvoidanceDirection::LEFT;
  const auto drivable_lanes =
    generateDrivableLanesWithAdjacent(current_lanelets_, adjacent_lanelets, direction);
  const auto shorten_lanes = utils::cutOverlappedLanes(output_path, drivable_lanes);
  const auto expanded_lanes =
    utils::expandLanelets(shorten_lanes, left_offset, right_offset, dp.drivable_area_types_to_skip);

  out.path = output_path;
  out.reference_path = getPreviousModuleOutput().reference_path;
  out.drivable_area_info.drivable_lanes = expanded_lanes;
  out.drivable_area_info.is_already_expanded = true;
  return out;
}

BehaviorModuleOutput SimpleLaneChangeAvoidanceModule::plan()
{
  if (reference_path_.points.size() < 2) {
    return passThrough(InfeasibleReason::NO_TARGET);
  }

  if (!shouldInitializeManeuver(path_shifter_.getShiftLines())) {
    ShiftedPath shifted_path;
    if (!path_shifter_.generate(&shifted_path) || shifted_path.path.points.empty()) {
      return passThrough(InfeasibleReason::PATH_GENERATION_FAILED);
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
    return adjustDrivableArea(shifted_path, adjacent_lanelets);
  }

  const auto target = detectTarget();
  if (!target.has_value()) {
    active_target_.reset();
    active_adjacent_lane_.reset();
    return passThrough(InfeasibleReason::NO_TARGET);
  }

  active_target_ = target;
  debug_data_.target = target;

  const auto shift_result = calcLaneShift(*target);
  if (shift_result.reason != InfeasibleReason::NONE) {
    return passThrough(shift_result.reason);
  }
  active_adjacent_lane_ = shift_result.adjacent_lane;

  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const auto feasibility_result =
    checkFeasibility(*target, shift_result.shift_length, *parameters_, ego_speed);
  if (feasibility_result.reason != InfeasibleReason::NONE) {
    return passThrough(feasibility_result.reason);
  }

  const auto shift_lines = buildShiftLines(*target, shift_result);
  path_shifter_.setShiftLines(shift_lines);

  ShiftedPath shifted_path;
  if (!path_shifter_.generate(&shifted_path) || shifted_path.path.points.empty()) {
    return passThrough(InfeasibleReason::PATH_GENERATION_FAILED);
  }

  setOrientation(&shifted_path.path);
  prev_output_ = shifted_path;
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
  if (reference_path_.points.empty()) {
    return CandidateOutput(getPreviousModuleOutput().path);
  }

  auto path_shifter_local = path_shifter_;
  if (shouldInitializeManeuver(path_shifter_local.getShiftLines())) {
    if (!active_target_.has_value()) {
      return CandidateOutput(getPreviousModuleOutput().path);
    }

    const auto shift_result = calcLaneShift(*active_target_);
    if (shift_result.reason != InfeasibleReason::NONE) {
      return CandidateOutput(getPreviousModuleOutput().path);
    }
    const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
    if (
      checkFeasibility(*active_target_, shift_result.shift_length, *parameters_, ego_speed)
        .reason != InfeasibleReason::NONE) {
      return CandidateOutput(getPreviousModuleOutput().path);
    }
    path_shifter_local.setShiftLines(buildShiftLines(*active_target_, shift_result));
  }

  ShiftedPath shifted_path;
  if (!path_shifter_local.generate(&shifted_path) || shifted_path.path.points.empty()) {
    return CandidateOutput(getPreviousModuleOutput().path);
  }
  setOrientation(&shifted_path.path);
  return CandidateOutput(shifted_path.path);
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

  const auto & prev_reference = getPreviousModuleOutput().path;
  const size_t orig_ego_idx =
    autoware::motion_utils::findNearestIndex(original_path.points, getEgoPose().position);
  const size_t prev_ego_idx = autoware::motion_utils::findNearestSegmentIndex(
    prev_reference.points, autoware_utils::get_point(original_path.points.at(orig_ego_idx)));

  size_t clip_idx = 0;
  for (size_t i = 0; i < prev_ego_idx; ++i) {
    if (backward_length > autoware::motion_utils::calcSignedArcLength(
                           prev_reference.points, clip_idx, prev_ego_idx)) {
      break;
    }
    clip_idx = i;
  }

  PathWithLaneId extended_path{};
  extended_path.points.insert(
    extended_path.points.end(), prev_reference.points.begin() + clip_idx,
    prev_reference.points.begin() + prev_ego_idx);
  extended_path.points.insert(
    extended_path.points.end(), original_path.points.begin() + orig_ego_idx,
    original_path.points.end());
  return extended_path;
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
