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

#include "autoware/behavior_path_simple_avoidance_module/scene.hpp"

#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_simple_avoidance_module/utils.hpp"

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

void logNoTargetDiagnosisDetails(
  const rclcpp::Logger & logger, rclcpp::Clock & clock, const char * prefix,
  const NoTargetDiagnosis & d, const SimpleAvoidanceParameters & parameters,
  const size_t reference_path_points, const int throttle_ms)
{
  if (d.precondition == NoTargetPrecondition::NO_DYNAMIC_OBJECT) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, throttle_ms,
      "[SIMPLE_AVOIDANCE] %s | precondition=%s (planner has no perception input yet)",
      prefix, toString(d.precondition));
    return;
  }

  if (d.precondition == NoTargetPrecondition::EMPTY_REFERENCE_PATH) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, throttle_ms,
      "[SIMPLE_AVOIDANCE] %s | precondition=%s ref_path_pts=%zu",
      prefix, toString(d.precondition), reference_path_points);
    return;
  }

  if (d.precondition == NoTargetPrecondition::EMPTY_CURRENT_LANELETS) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, throttle_ms,
      "[SIMPLE_AVOIDANCE] %s | precondition=%s objects=%zu ref_path_pts=%zu",
      prefix, toString(d.precondition), d.total_objects, reference_path_points);
    return;
  }

  if (!d.has_nearest_rejection) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, throttle_ms,
      "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
      "no_overlap=%zu | ref_path_pts=%zu",
      prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane, d.rejected_longitudinal,
      d.rejected_no_overlap, reference_path_points);
    return;
  }

  switch (d.nearest_reject_reason) {
    case TargetRejectReason::MOVING:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s speed=%.2f th=%.2f "
        "(exceeds by %.2fm/s) pos=(%.2f,%.2f) lon=%.2fm lat=%.2fm",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane,
        d.rejected_longitudinal, d.rejected_no_overlap, d.nearest_uuid.c_str(),
        toString(d.nearest_reject_reason), d.nearest_speed, parameters.th_moving_speed,
        d.nearest_shortfall, d.nearest_obj_x, d.nearest_obj_y, d.nearest_lon, d.nearest_lat);
      return;
    case TargetRejectReason::OUT_OF_LANE:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s pos=(%.2f,%.2f) lon=%.2fm lat=%.2fm "
        "(not in current route lanelet)",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane,
        d.rejected_longitudinal, d.rejected_no_overlap, d.nearest_uuid.c_str(),
        toString(d.nearest_reject_reason), d.nearest_obj_x, d.nearest_obj_y, d.nearest_lon,
        d.nearest_lat);
      return;
    case TargetRejectReason::LONGITUDINAL:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s lon=%.2fm allowed=[%.2f,%.2f]m "
        "shortfall=%.2fm pos=(%.2f,%.2f) lat=%.2fm",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane,
        d.rejected_longitudinal, d.rejected_no_overlap, d.nearest_uuid.c_str(),
        toString(d.nearest_reject_reason), d.nearest_lon, parameters.min_forward_distance,
        parameters.max_forward_distance, d.nearest_shortfall, d.nearest_obj_x, d.nearest_obj_y,
        d.nearest_lat);
      return;
    case TargetRejectReason::NO_OVERLAP:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s overlap=%.2fm threshold=%.2fm "
        "(need overlap < threshold, short by %.2fm lateral) pos=(%.2f,%.2f) lon=%.2fm lat=%.2fm",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane,
        d.rejected_longitudinal, d.rejected_no_overlap, d.nearest_uuid.c_str(),
        toString(d.nearest_reject_reason), d.nearest_overlap, d.nearest_threshold,
        d.nearest_shortfall, d.nearest_obj_x, d.nearest_obj_y, d.nearest_lon, d.nearest_lat);
      return;
    default:
      return;
  }
}

void logPassThroughDetails(
  const rclcpp::Logger & logger, rclcpp::Clock & clock, const InfeasibleReason reason,
  const PassThroughDebugInfo & debug_info, const SimpleAvoidanceParameters & parameters,
  const double ego_half_width)
{
  switch (reason) {
    case InfeasibleReason::NO_TARGET: {
      if (!debug_info.no_target.has_value()) {
        RCLCPP_WARN_THROTTLE(
          logger, clock, 1000, "[SIMPLE_AVOIDANCE] pass-through reason=%s (no object data)",
          toString(reason));
        return;
      }
      logNoTargetDiagnosisDetails(
        logger, clock, "pass-through reason=no_target", *debug_info.no_target, parameters,
        debug_info.reference_path_points, 1000);
      return;
    }
    case InfeasibleReason::NO_ROOM: {
      if (!debug_info.shift.has_value() || !debug_info.target.has_value()) {
        RCLCPP_WARN_THROTTLE(
          logger, clock, 1000, "[SIMPLE_AVOIDANCE] pass-through reason=%s", toString(reason));
        return;
      }
      const auto & s = *debug_info.shift;
      const auto & t = *debug_info.target;
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000,
        "[SIMPLE_AVOIDANCE] pass-through reason=%s | target uuid=%s lon=%.2fm lat=%.2fm "
        "obj_hw=%.2fm | required_clearance=%.2fm (|lat|=%.2f+obj_hw=%.2f+ego_hw=%.2f+margin=%.2f) | "
        "max_shift_length=%.2fm requested_shift=%.2fm | remaining_gap=%.2fm (short by %.2fm "
        "lateral space)",
        toString(reason), t.uuid.c_str(), t.longitudinal_distance, t.lateral_offset,
        t.object_half_width, s.required_clearance, std::abs(t.lateral_offset), t.object_half_width,
        ego_half_width, parameters.lateral_margin, parameters.max_shift_length,
        s.requested_shift_length, s.remaining_gap, s.remaining_gap);
      return;
    }
    case InfeasibleReason::INSUFFICIENT_DISTANCE: {
      if (!debug_info.feasibility.has_value() || !debug_info.target.has_value()) {
        RCLCPP_WARN_THROTTLE(
          logger, clock, 1000, "[SIMPLE_AVOIDANCE] pass-through reason=%s", toString(reason));
        return;
      }
      const auto & f = *debug_info.feasibility;
      const auto & t = *debug_info.target;
      const double shortfall = f.dist_to_shift_end - f.dist_to_obstacle;
      const double shifting_dist = std::max(f.jerk_distance, f.min_shifting_distance);
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000,
        "[SIMPLE_AVOIDANCE] pass-through reason=%s | target uuid=%s lon=%.2fm obj_hl=%.2fm | "
        "dist_to_shift_end=%.2fm (prepare=%.2f + shifting=%.2f) | dist_to_obstacle=%.2fm "
        "(lon-obj_hl-margin) | shortfall=%.2fm (need %.2fm more longitudinal space) | "
        "ego_speed=%.2fm/s jerk_distance=%.2fm",
        toString(reason), t.uuid.c_str(), t.longitudinal_distance, t.object_half_length,
        f.dist_to_shift_end, f.min_prepare_distance, shifting_dist, f.dist_to_obstacle, shortfall,
        shortfall, f.ego_speed, f.jerk_distance);
      return;
    }
    case InfeasibleReason::PATH_GENERATION_FAILED: {
      const double shift_length =
        debug_info.shift.has_value() ? debug_info.shift->shift_length : 0.0;
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000,
        "[SIMPLE_AVOIDANCE] pass-through reason=%s | ref_path_pts=%zu shift_lines=%zu "
        "shift_length=%.2fm",
        toString(reason), debug_info.reference_path_points, debug_info.shift_lines_count,
        shift_length);
      return;
    }
    default:
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000, "[SIMPLE_AVOIDANCE] pass-through reason=%s", toString(reason));
      return;
  }
}
}  // namespace

SimpleAvoidanceModule::SimpleAvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<SimpleAvoidanceParameters> & parameters,
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

void SimpleAvoidanceModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  current_lanelets_.clear();
  path_shifter_ = PathShifter{};
  prev_output_ = ShiftedPath{};
  active_target_.reset();
  debug_data_ = SimpleAvoidanceDebugData{};
  resetPathCandidate();
  resetPathReference();
}

void SimpleAvoidanceModule::processOnEntry() {}

void SimpleAvoidanceModule::processOnExit() { initVariables(); }

bool SimpleAvoidanceModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }
  if (detectTarget().has_value()) {
    return true;
  }

  const auto diagnosis = diagnoseNoTarget();
  logNoTargetDiagnosisDetails(
    getLogger(), *clock_, "not requested (module idle)", diagnosis, *parameters_,
    reference_path_.points.size(), 2000);

  return false;
}

bool SimpleAvoidanceModule::canTransitSuccessState()
{
  constexpr double zero_threshold = 0.05;
  const auto target = detectTarget();
  if (target.has_value()) {
    return false;
  }
  const auto current_shift = getClosestShiftLength(prev_output_, getEgoPose().position);
  return std::abs(current_shift) < zero_threshold;
}

void SimpleAvoidanceModule::updateData()
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

std::optional<AvoidanceTarget> SimpleAvoidanceModule::detectTarget() const
{
  if (!planner_data_->dynamic_object || reference_path_.points.empty()) {
    return std::nullopt;
  }

  const auto ego_pos = planner_data_->self_odometry->pose.pose.position;
  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;

  std::optional<AvoidanceTarget> nearest_target;
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
      AvoidanceTarget target;
      target.pose = pose;
      target.longitudinal_distance = longitudinal_distance;
      target.lateral_offset = lateral_offset;
      target.object_half_width = object_half_width;
      target.object_half_length = getObjectHalfLength(object.shape);
      target.uuid = autoware_utils_uuid::to_hex_string(object.object_id);
      nearest_target = target;
      min_longitudinal = longitudinal_distance;
    }
  }

  return nearest_target;
}

NoTargetDiagnosis SimpleAvoidanceModule::diagnoseNoTarget() const
{
  NoTargetDiagnosis diagnosis;

  if (!planner_data_->dynamic_object) {
    diagnosis.precondition = NoTargetPrecondition::NO_DYNAMIC_OBJECT;
    return diagnosis;
  }

  if (reference_path_.points.empty()) {
    diagnosis.precondition = NoTargetPrecondition::EMPTY_REFERENCE_PATH;
    diagnosis.total_objects = planner_data_->dynamic_object->objects.size();
    return diagnosis;
  }

  if (current_lanelets_.empty()) {
    diagnosis.precondition = NoTargetPrecondition::EMPTY_CURRENT_LANELETS;
    diagnosis.total_objects = planner_data_->dynamic_object->objects.size();
    return diagnosis;
  }

  const auto ego_pos = planner_data_->self_odometry->pose.pose.position;
  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;
  const double overlap_threshold = ego_half_width + parameters_->lateral_margin;

  diagnosis.total_objects = planner_data_->dynamic_object->objects.size();
  double min_rejected_lon = std::numeric_limits<double>::max();

  for (const auto & object : planner_data_->dynamic_object->objects) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const double speed = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);

    const auto uuid = autoware_utils_uuid::to_hex_string(object.object_id);
    const auto nearest_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(reference_path_.points, pose.position);
    const double longitudinal_distance =
      autoware::motion_utils::calcSignedArcLength(reference_path_.points, ego_pos, pose.position);
    const double lateral_offset = autoware::motion_utils::calcLateralOffset(
      reference_path_.points, pose.position, nearest_seg_idx);
    const double object_half_width = getObjectHalfWidth(object.shape);
    const double overlap = std::abs(lateral_offset) - object_half_width;

    TargetRejectReason reject_reason{TargetRejectReason::MOVING};
    bool rejected = false;
    double threshold = 0.0;
    double shortfall = 0.0;

    if (speed >= parameters_->th_moving_speed) {
      diagnosis.rejected_moving++;
      reject_reason = TargetRejectReason::MOVING;
      rejected = true;
      threshold = parameters_->th_moving_speed;
      shortfall = speed - threshold;
    } else if (!current_lanelets_.empty() && !isObjectOverlappingLanelets(object, current_lanelets_)) {
      diagnosis.rejected_out_of_lane++;
      reject_reason = TargetRejectReason::OUT_OF_LANE;
      rejected = true;
    } else if (
      longitudinal_distance < parameters_->min_forward_distance ||
      longitudinal_distance > parameters_->max_forward_distance) {
      diagnosis.rejected_longitudinal++;
      reject_reason = TargetRejectReason::LONGITUDINAL;
      rejected = true;
      if (longitudinal_distance < parameters_->min_forward_distance) {
        threshold = parameters_->min_forward_distance;
        shortfall = threshold - longitudinal_distance;
      } else {
        threshold = parameters_->max_forward_distance;
        shortfall = longitudinal_distance - threshold;
      }
    } else if (overlap >= overlap_threshold) {
      diagnosis.rejected_no_overlap++;
      reject_reason = TargetRejectReason::NO_OVERLAP;
      rejected = true;
      threshold = overlap_threshold;
      shortfall = overlap - threshold;
    }

    if (rejected && longitudinal_distance < min_rejected_lon) {
      min_rejected_lon = longitudinal_distance;
      diagnosis.has_nearest_rejection = true;
      diagnosis.nearest_reject_reason = reject_reason;
      diagnosis.nearest_uuid = uuid;
      diagnosis.nearest_speed = speed;
      diagnosis.nearest_lon = longitudinal_distance;
      diagnosis.nearest_lat = lateral_offset;
      diagnosis.nearest_overlap = overlap;
      diagnosis.nearest_obj_x = pose.position.x;
      diagnosis.nearest_obj_y = pose.position.y;
      diagnosis.nearest_threshold = threshold;
      diagnosis.nearest_shortfall = shortfall;
    }
  }

  return diagnosis;
}

ShiftLineArray SimpleAvoidanceModule::buildShiftLines(
  const AvoidanceTarget & target, const double shift_length) const
{
  const auto ego_idx = planner_data_->findEgoIndex(reference_path_.points);
  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);

  const double dist_to_avoid_start = parameters_->min_prepare_distance;
  const double jerk_distance = autoware::motion_utils::calc_longitudinal_dist_from_jerk(
    std::abs(shift_length), parameters_->shifting_lateral_jerk,
    std::max(ego_speed, parameters_->min_shifting_speed));
  const double dist_to_avoid_end =
    dist_to_avoid_start + std::max(jerk_distance, parameters_->min_shifting_distance);
  const double dist_to_return_start =
    target.longitudinal_distance + target.object_half_length + parameters_->return_distance_after_object;
  const double dist_to_return_end = dist_to_return_start + std::max(jerk_distance, parameters_->min_shifting_distance);

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

BehaviorModuleOutput SimpleAvoidanceModule::passThrough(
  const InfeasibleReason reason, const PassThroughDebugInfo & debug_info) const
{
  debug_data_.last_reason = reason;
  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;
  logPassThroughDetails(getLogger(), *clock_, reason, debug_info, *parameters_, ego_half_width);
  return getPreviousModuleOutput();
}

BehaviorModuleOutput SimpleAvoidanceModule::adjustDrivableArea(const ShiftedPath & path) const
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

  const auto drivable_lanes = utils::generateDrivableLanes(current_lanelets_);
  const auto shorten_lanes = utils::cutOverlappedLanes(output_path, drivable_lanes);
  const auto expanded_lanes =
    utils::expandLanelets(shorten_lanes, left_offset, right_offset, dp.drivable_area_types_to_skip);

  out.path = output_path;
  out.reference_path = getPreviousModuleOutput().reference_path;
  out.drivable_area_info.drivable_lanes = expanded_lanes;
  out.drivable_area_info.is_already_expanded = true;
  return out;
}

BehaviorModuleOutput SimpleAvoidanceModule::plan()
{
  PassThroughDebugInfo debug_info;
  debug_info.reference_path_points = reference_path_.points.size();

  if (reference_path_.points.size() < 2) {
    debug_info.no_target = diagnoseNoTarget();
    return passThrough(InfeasibleReason::NO_TARGET, debug_info);
  }

  const auto target = detectTarget();
  if (!target.has_value()) {
    active_target_.reset();
    if (!path_shifter_.getShiftLines().empty()) {
      ShiftedPath shifted_path;
      if (!path_shifter_.generate(&shifted_path)) {
        return passThrough(InfeasibleReason::PATH_GENERATION_FAILED, debug_info);
      }
      if (!shifted_path.path.points.empty()) {
        setOrientation(&shifted_path.path);
        prev_output_ = shifted_path;
        debug_data_.last_reason = InfeasibleReason::NONE;
        path_reference_ =
          std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);
        if (parameters_->publish_debug_marker) {
          setDebugMarkersVisualization();
        }
        return adjustDrivableArea(shifted_path);
      }
    }
    debug_info.no_target = diagnoseNoTarget();
    return passThrough(InfeasibleReason::NO_TARGET, debug_info);
  }

  active_target_ = target;
  debug_data_.target = target;
  debug_info.target = target;

  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;
  const auto shift_result = calcShiftLength(*target, *parameters_, ego_half_width);
  debug_info.shift = shift_result;
  if (shift_result.reason != InfeasibleReason::NONE) {
    return passThrough(shift_result.reason, debug_info);
  }

  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const auto feasibility_result =
    checkFeasibility(*target, shift_result.shift_length, *parameters_, ego_speed);
  debug_info.feasibility = feasibility_result;
  if (feasibility_result.reason != InfeasibleReason::NONE) {
    return passThrough(feasibility_result.reason, debug_info);
  }

  const auto shift_lines = buildShiftLines(*target, shift_result.shift_length);
  path_shifter_.setShiftLines(shift_lines);
  debug_info.shift_lines_count = shift_lines.size();

  ShiftedPath shifted_path;
  if (!path_shifter_.generate(&shifted_path) || shifted_path.path.points.empty()) {
    return passThrough(InfeasibleReason::PATH_GENERATION_FAILED, debug_info);
  }

  setOrientation(&shifted_path.path);
  prev_output_ = shifted_path;
  debug_data_.last_reason = InfeasibleReason::NONE;
  debug_data_.path_shifter = std::make_shared<PathShifter>(path_shifter_);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  const double lon_margin =
    feasibility_result.dist_to_obstacle - feasibility_result.dist_to_shift_end;
  // dist_to_shift_end: prepare + shifting distance needed before obstacle
  // dist_to_obstacle: available longitudinal space before obstacle front edge
  // lon_margin: remaining longitudinal clearance after shift completes
  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 2000,
    "[SIMPLE_AVOIDANCE] avoidance path generated shift=%.2f target_lon=%.2f target_lat=%.2f "
    "required_clearance=%.2f jerk_distance=%.2f ego_speed=%.2f dist_to_shift_end=%.2f "
    "dist_to_obstacle=%.2f lon_margin=%.2f",
    shift_result.shift_length, target->longitudinal_distance, target->lateral_offset,
    shift_result.required_clearance, feasibility_result.jerk_distance, feasibility_result.ego_speed,
    feasibility_result.dist_to_shift_end, feasibility_result.dist_to_obstacle, lon_margin);

  if (parameters_->publish_debug_marker) {
    setDebugMarkersVisualization();
  } else {
    debug_marker_.markers.clear();
  }

  return adjustDrivableArea(shifted_path);
}

CandidateOutput SimpleAvoidanceModule::planCandidate() const
{
  if (!active_target_.has_value() || reference_path_.points.empty()) {
    return CandidateOutput(getPreviousModuleOutput().path);
  }

  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;
  const auto shift_result = calcShiftLength(*active_target_, *parameters_, ego_half_width);
  if (shift_result.reason != InfeasibleReason::NONE) {
    return CandidateOutput(getPreviousModuleOutput().path);
  }
  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  if (
    checkFeasibility(*active_target_, shift_result.shift_length, *parameters_, ego_speed).reason !=
    InfeasibleReason::NONE) {
    return CandidateOutput(getPreviousModuleOutput().path);
  }

  auto path_shifter_local = path_shifter_;
  path_shifter_local.setShiftLines(buildShiftLines(*active_target_, shift_result.shift_length));

  ShiftedPath shifted_path;
  path_shifter_local.generate(&shifted_path);
  setOrientation(&shifted_path.path);
  return CandidateOutput(shifted_path.path);
}

PathWithLaneId SimpleAvoidanceModule::extendBackwardLength(
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

void SimpleAvoidanceModule::setDebugMarkersVisualization() const
{
  using marker_utils::createShiftLineMarkerArray;
  debug_marker_.markers.clear();
  if (!debug_data_.path_shifter) {
    return;
  }
  const auto markers = createShiftLineMarkerArray(
    debug_data_.path_shifter->getShiftLines(), debug_data_.path_shifter->getBaseOffset(),
    "simple_avoidance_shift_points", 0.1f, 0.6f, 1.0f, 0.4);
  autoware_utils::append_marker_array(markers, &debug_marker_);
}

}  // namespace autoware::behavior_path_planner
