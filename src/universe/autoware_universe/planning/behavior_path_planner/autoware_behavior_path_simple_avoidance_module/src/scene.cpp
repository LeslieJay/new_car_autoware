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
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_simple_avoidance_module/utils.hpp"

#include <autoware/motion_utils/trajectory/path_shift.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/is_valid.hpp>

#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

namespace autoware::behavior_path_planner
{
namespace
{
template <class Points>
autoware_utils::LineString2d makeLineString(const Points & points)
{
  autoware_utils::LineString2d line;
  for (const auto & point : points) {
    line.emplace_back(point.x, point.y);
  }
  return line;
}

struct OriginalRoadBoundaries
{
  autoware_utils::LineString2d left;
  autoware_utils::LineString2d right;
  bool from_lanelets{false};
};

void appendLaneletBoundary(
  autoware_utils::LineString2d & output, const lanelet::ConstLineString3d & boundary)
{
  constexpr double duplicate_point_tolerance = 1.0e-3;
  for (const auto & point : boundary) {
    const autoware_utils::Point2d converted{point.x(), point.y()};
    if (
      output.empty() ||
      std::hypot(output.back().x() - converted.x(), output.back().y() - converted.y()) >
        duplicate_point_tolerance) {
      output.push_back(converted);
    }
  }
}

std::optional<OriginalRoadBoundaries> makeOriginalRoadBoundaries(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path)
{
  OriginalRoadBoundaries boundaries;
  if (!lanelets.empty()) {
    for (const auto & lanelet : lanelets) {
      appendLaneletBoundary(boundaries.left, lanelet.leftBound());
      appendLaneletBoundary(boundaries.right, lanelet.rightBound());
    }
    boundaries.from_lanelets = true;
  } else {
    // Unit tests and isolated module users may not have a RouteHandler. This fallback is only
    // accepted when the caller explicitly supplies both bounds. In the production planner the
    // lanelet-derived bounds above are authoritative and cannot be drivable-area-expanded here.
    boundaries.left = makeLineString(path.left_bound);
    boundaries.right = makeLineString(path.right_bound);
  }

  if (boundaries.left.size() < 2 || boundaries.right.size() < 2) {
    return std::nullopt;
  }
  return boundaries;
}

void extendLineStringEnds(autoware_utils::LineString2d & line, const double distance)
{
  if (line.size() < 2) {
    return;
  }
  const auto extend_from = [distance](
                             const autoware_utils::Point2d & end,
                             const autoware_utils::Point2d & neighbor) {
    const double dx = end.x() - neighbor.x();
    const double dy = end.y() - neighbor.y();
    const double norm = std::hypot(dx, dy);
    if (norm < 1.0e-6) {
      return end;
    }
    return autoware_utils::Point2d{end.x() + distance * dx / norm, end.y() + distance * dy / norm};
  };
  line.front() = extend_from(line.front(), line.at(1));
  line.back() = extend_from(line.back(), line.at(line.size() - 2));
}

std::optional<autoware_utils::Polygon2d> makeLaneCorridor(
  autoware_utils::LineString2d left_bound, autoware_utils::LineString2d right_bound,
  const double longitudinal_extension)
{
  if (left_bound.size() < 2 || right_bound.size() < 2) {
    return std::nullopt;
  }
  extendLineStringEnds(left_bound, longitudinal_extension);
  extendLineStringEnds(right_bound, longitudinal_extension);

  autoware_utils::Polygon2d corridor;
  for (const auto & point : left_bound) {
    corridor.outer().push_back(point);
  }
  for (auto itr = right_bound.rbegin(); itr != right_bound.rend(); ++itr) {
    corridor.outer().push_back(*itr);
  }
  corridor.outer().push_back(corridor.outer().front());
  boost::geometry::correct(corridor);
  if (!boost::geometry::is_valid(corridor)) {
    return std::nullopt;
  }
  return corridor;
}

template <class PathPoints>
bool isOnDrivableSide(
  const autoware_utils::Polygon2d & area, const autoware_utils::LineString2d & bound,
  const PathPoints & reference_points)
{
  if (bound.size() < 2 || reference_points.empty()) {
    return false;
  }

  for (const auto & point : area.outer()) {
    size_t nearest_segment_index = 0;
    double nearest_segment_distance = std::numeric_limits<double>::max();
    for (size_t i = 1; i < bound.size(); ++i) {
      const autoware_utils::Segment2d segment{bound.at(i - 1), bound.at(i)};
      const double distance = boost::geometry::distance(point, segment);
      if (distance < nearest_segment_distance) {
        nearest_segment_distance = distance;
        nearest_segment_index = i - 1;
      }
    }

    const auto & segment_start = bound.at(nearest_segment_index);
    const auto & segment_end = bound.at(nearest_segment_index + 1);
    const autoware_utils::Segment2d segment{segment_start, segment_end};
    const auto cross_product = [&](const autoware_utils::Point2d & candidate) {
      return (segment_end.x() - segment_start.x()) * (candidate.y() - segment_start.y()) -
             (segment_end.y() - segment_start.y()) * (candidate.x() - segment_start.x());
    };

    double reference_side = 0.0;
    double nearest_reference_distance = std::numeric_limits<double>::max();
    for (const auto & reference_point : reference_points) {
      const autoware_utils::Point2d candidate{
        reference_point.point.pose.position.x, reference_point.point.pose.position.y};
      const double distance = boost::geometry::distance(candidate, segment);
      if (distance < nearest_reference_distance) {
        nearest_reference_distance = distance;
        reference_side = cross_product(candidate);
      }
    }
    constexpr double side_tolerance = 1.0e-6;
    if (
      std::abs(reference_side) <= side_tolerance ||
      cross_product(point) * reference_side <= side_tolerance) {
      return false;
    }
  }
  return true;
}

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
      "[SIMPLE_AVOIDANCE] %s | precondition=%s (planner has no perception input yet)", prefix,
      toString(d.precondition));
    return;
  }

  if (d.precondition == NoTargetPrecondition::EMPTY_REFERENCE_PATH) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, throttle_ms, "[SIMPLE_AVOIDANCE] %s | precondition=%s ref_path_pts=%zu",
      prefix, toString(d.precondition), reference_path_points);
    return;
  }

  if (d.precondition == NoTargetPrecondition::EMPTY_CURRENT_LANELETS) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, throttle_ms,
      "[SIMPLE_AVOIDANCE] %s | precondition=%s objects=%zu ref_path_pts=%zu", prefix,
      toString(d.precondition), d.total_objects, reference_path_points);
    return;
  }

  if (!d.has_nearest_rejection) {
    RCLCPP_WARN_THROTTLE(
      logger, clock, throttle_ms,
      "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
      "no_overlap=%zu infeasible_no_room=%zu infeasible_distance=%zu | ref_path_pts=%zu",
      prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane, d.rejected_longitudinal,
      d.rejected_no_overlap, d.rejected_no_room, d.rejected_insufficient_distance,
      reference_path_points);
    return;
  }

  switch (d.nearest_reject_reason) {
    case TargetRejectReason::MOVING:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s speed=%.2f th=%.2f "
        "(exceeds by %.2fm/s) pos=(%.2f,%.2f) lon=%.2fm lat=%.2fm",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane, d.rejected_longitudinal,
        d.rejected_no_overlap, d.nearest_uuid.c_str(), toString(d.nearest_reject_reason),
        d.nearest_speed, parameters.th_moving_speed, d.nearest_shortfall, d.nearest_obj_x,
        d.nearest_obj_y, d.nearest_lon, d.nearest_lat);
      return;
    case TargetRejectReason::OUT_OF_LANE:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s pos=(%.2f,%.2f) lon=%.2fm lat=%.2fm "
        "(not in current route lanelet)",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane, d.rejected_longitudinal,
        d.rejected_no_overlap, d.nearest_uuid.c_str(), toString(d.nearest_reject_reason),
        d.nearest_obj_x, d.nearest_obj_y, d.nearest_lon, d.nearest_lat);
      return;
    case TargetRejectReason::LONGITUDINAL:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s lon=%.2fm allowed=[%.2f,%.2f]m "
        "shortfall=%.2fm pos=(%.2f,%.2f) lat=%.2fm",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane, d.rejected_longitudinal,
        d.rejected_no_overlap, d.nearest_uuid.c_str(), toString(d.nearest_reject_reason),
        d.nearest_lon, parameters.min_forward_distance, parameters.max_forward_distance,
        d.nearest_shortfall, d.nearest_obj_x, d.nearest_obj_y, d.nearest_lat);
      return;
    case TargetRejectReason::NO_OVERLAP:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu rejected: moving=%zu out_of_lane=%zu lon_range=%zu "
        "no_overlap=%zu | nearest_rejected uuid=%s reason=%s overlap=%.2fm threshold=%.2fm "
        "(need overlap < threshold, short by %.2fm lateral) pos=(%.2f,%.2f) lon=%.2fm lat=%.2fm",
        prefix, d.total_objects, d.rejected_moving, d.rejected_out_of_lane, d.rejected_longitudinal,
        d.rejected_no_overlap, d.nearest_uuid.c_str(), toString(d.nearest_reject_reason),
        d.nearest_overlap, d.nearest_threshold, d.nearest_shortfall, d.nearest_obj_x,
        d.nearest_obj_y, d.nearest_lon, d.nearest_lat);
      return;
    case TargetRejectReason::NO_ROOM:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu infeasible_no_room=%zu infeasible_distance=%zu | "
        "nearest_rejected uuid=%s reason=%s lon=%.2fm lat=%.2fm "
        "required_clearance=%.2fm max_shift_length=%.2fm remaining_gap=%.2fm "
        "(target excluded; obstacle_stop remains responsible for stopping)",
        prefix, d.total_objects, d.rejected_no_room, d.rejected_insufficient_distance,
        d.nearest_uuid.c_str(), toString(d.nearest_reject_reason), d.nearest_lon, d.nearest_lat,
        d.nearest_threshold, parameters.max_shift_length, d.nearest_shortfall);
      return;
    case TargetRejectReason::INSUFFICIENT_DISTANCE:
      RCLCPP_WARN_THROTTLE(
        logger, clock, throttle_ms,
        "[SIMPLE_AVOIDANCE] %s | objects=%zu infeasible_no_room=%zu infeasible_distance=%zu | "
        "nearest_rejected uuid=%s reason=%s lon=%.2fm object_half_length=%.2fm "
        "required_before_front=%.2fm transition=%.2fm dist_to_avoid_start=%.2fm "
        "dist_to_shift_end=%.2fm dist_to_obstacle=%.2fm "
        "(target excluded; obstacle_stop remains responsible for stopping)",
        prefix, d.total_objects, d.rejected_no_room, d.rejected_insufficient_distance,
        d.nearest_uuid.c_str(), toString(d.nearest_reject_reason), d.nearest_lon,
        d.nearest_object_half_length, d.nearest_threshold,
        d.nearest_transition_distance, d.nearest_dist_to_avoid_start,
        d.nearest_dist_to_shift_end, d.nearest_dist_to_obstacle);
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
        "obj_hw=%.2fm | required_clearance=%.2fm (|lat|=%.2f+obj_hw=%.2f+ego_hw=%.2f+margin=%.2f) "
        "| "
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
      RCLCPP_WARN_THROTTLE(
        logger, clock, 1000,
        "[SIMPLE_AVOIDANCE] pass-through reason=%s | target uuid=%s lon=%.2fm obj_hl=%.2fm | "
        "dist_to_avoid_start=%.2fm required_before_front=%.2fm | transition=%.2fm "
        "dist_to_shift_end=%.2fm | dist_to_obstacle=%.2fm (lon-obj_hl-margin) | "
        "shortfall=%.2fm | ego_speed=%.2fm/s jerk_distance=%.2fm",
        toString(reason), t.uuid.c_str(), t.longitudinal_distance, t.object_half_length,
        f.dist_to_avoid_start, f.required_start_distance_before_front, f.transition_distance,
        f.dist_to_shift_end, f.dist_to_obstacle, shortfall, f.ego_speed, f.jerk_distance);
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

void SimpleAvoidanceModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  current_lanelets_.clear();
  path_shifter_ = PathShifter{};
  prev_output_ = ShiftedPath{};
  active_target_.reset();
  lifecycle_state_ = AvoidanceLifecycleState::IDLE;
  completion_stable_count_ = 0;
  ego_aligned_return_active_ = false;
  path_generation_failure_started_.reset();
  route_id_.reset();
  debug_data_ = SimpleAvoidanceDebugData{};
  resetPathCandidate();
  resetPathReference();
}

void SimpleAvoidanceModule::processOnEntry()
{
  lifecycle_state_ = AvoidanceLifecycleState::CANDIDATE;
  route_id_ = planner_data_->route_handler->getRouteUuid();
  active_trailer_configuration_ = trailer_configuration_store_->snapshot();
  RCLCPP_INFO(
    getLogger(), "[SIMPLE_AVOIDANCE] frozen trailer configuration: count=%zu",
    active_trailer_configuration_.geometries.size());
}

void SimpleAvoidanceModule::processOnExit()
{
  initVariables();
}

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
  if (lifecycle_state_ == AvoidanceLifecycleState::STOPPING) {
    const double speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
    return speed <= parameters_->trailer_stationary_speed_threshold;
  }
  if (active_target_.has_value()) {
    if (const auto updated = updateTargetMetrics(*active_target_)) {
      active_target_ = updated;
    }
    const bool is_passed = isTargetPassed(*active_target_, *parameters_);
    const bool is_expired =
      isTargetHoldExpired(*active_target_, clock_->now(), parameters_->target_lost_time_threshold);
    if (!is_passed && is_expired) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 2000,
        "[SIMPLE_AVOIDANCE] success check releases expired target uuid=%s last_seen_age=%.2fs",
        active_target_->uuid.c_str(), (clock_->now() - active_target_->last_seen).seconds());
      active_target_.reset();
      debug_data_.target.reset();
    } else if (!is_passed) {
      RCLCPP_INFO_THROTTLE(
        getLogger(), *clock_, 2000,
        "[SIMPLE_AVOIDANCE] success blocked: active_target uuid=%s lon=%.2fm",
        active_target_->uuid.c_str(), active_target_->longitudinal_distance);
    }
  }

  const AvoidanceCompletionStatus status{
    active_target_.has_value(),
    active_target_.has_value() && isTargetPassed(*active_target_, *parameters_),
    !path_shifter_.getShiftLines().empty(),
    isEgoOnShiftLine(),
    path_shifter_.getBaseOffset(),
    getClosestShiftLength(prev_output_, getEgoPose().position),
    getEgoLateralOffsetToReference(),
    parameters_->lateral_execution_threshold};

  const bool return_to_center_complete = canCompleteAvoidance(status);
  AvoidanceLifecycleObservation observation;
  observation.state = lifecycle_state_;
  observation.return_to_center_complete = return_to_center_complete;
  observation.completion_stable_count = completion_stable_count_;
  const auto decision = decideAvoidanceLifecycle(
    observation, parameters_->path_generation_failure_timeout,
    parameters_->completion_stable_count);
  lifecycle_state_ = decision.next_state;
  completion_stable_count_ = decision.completion_stable_count;
  const bool can_complete =
    decision.action == AvoidanceLifecycleAction::COMPLETE_MANEUVER ||
    (decision.action == AvoidanceLifecycleAction::CANCEL_CANDIDATE && return_to_center_complete);
  if (!can_complete) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 2000,
      "[SIMPLE_AVOIDANCE] success blocked: active=%d passed=%d shift_lines=%zu "
      "ego_on_shift=%d base_offset=%.2f ego_shift=%.2f actual_ego_offset=%.2f threshold=%.2f",
      status.has_active_target, status.is_active_target_passed,
      path_shifter_.getShiftLines().size(), status.is_ego_on_shift_line, status.base_offset,
      status.ego_shift, status.actual_ego_lateral_offset, status.lateral_execution_threshold);
  }
  return can_complete;
}

void SimpleAvoidanceModule::updateData()
{
  if (
    route_id_.has_value() && *route_id_ != planner_data_->route_handler->getRouteUuid() &&
    (lifecycle_state_ == AvoidanceLifecycleState::COMMITTED ||
     lifecycle_state_ == AvoidanceLifecycleState::RETURNING)) {
    lifecycle_state_ = AvoidanceLifecycleState::STOPPING;
    path_generation_failure_started_ =
      clock_->now() - rclcpp::Duration::from_seconds(parameters_->path_generation_failure_timeout);
    return;
  }

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

  // Do not carry a stale lanelet corridor into the next cycle. If the route handler cannot
  // resolve the current pose, boundary validation must fail closed instead of validating against
  // a lanelet sequence from an earlier pose.
  current_lanelets_.clear();
  lanelet::ConstLanelet current_lane;
  if (route_handler->getClosestLaneletWithinRoute(reference_pose, &current_lane)) {
    current_lanelets_ = route_handler->getLaneletSequence(
      current_lane, reference_pose, p.backward_path_length, p.forward_path_length);
  }

  const size_t nearest_idx = planner_data_->findEgoIndex(path_shifter_.getReferencePath().points);
  path_shifter_.removeBehindShiftLineAndSetBaseOffset(nearest_idx);
}

std::optional<AvoidanceTarget> SimpleAvoidanceModule::detectTarget(
  const std::optional<std::string> & preferred_uuid, const bool use_hold_hysteresis) const
{
  if (!planner_data_->dynamic_object || reference_path_.points.empty()) {
    return std::nullopt;
  }

  const auto ego_pos = planner_data_->self_odometry->pose.pose.position;
  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;

  std::optional<AvoidanceTarget> nearest_target;
  std::optional<AvoidanceTarget> spatially_matched_target;
  double min_longitudinal = std::numeric_limits<double>::max();
  double min_association_distance = std::numeric_limits<double>::max();
  constexpr double position_threshold = 1.5;

  for (const auto & object : planner_data_->dynamic_object->objects) {
    const auto uuid = autoware_utils_uuid::to_hex_string(object.object_id);
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const double speed = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);
    const double object_half_width = getObjectHalfWidth(object.shape);
    const double object_half_length = getObjectHalfLength(object.shape);
    const bool uuid_match = preferred_uuid.has_value() && uuid == *preferred_uuid;
    double association_distance = std::numeric_limits<double>::max();
    bool spatial_match = false;
    if (preferred_uuid.has_value() && active_target_.has_value()) {
      const auto & previous_position = active_target_->pose.position;
      association_distance =
        std::hypot(previous_position.x - pose.position.x, previous_position.y - pose.position.y);
      const bool dimensions_match =
        std::abs(active_target_->object_half_width - object_half_width) <= 1.0 &&
        std::abs(active_target_->object_half_length - object_half_length) <= 1.0;
      spatial_match = association_distance < position_threshold && dimensions_match;
    }
    const bool associated_with_active_target = uuid_match || spatial_match;
    if (speed >= parameters_->th_moving_speed && !associated_with_active_target) {
      continue;
    }
    if (speed >= parameters_->th_moving_speed && associated_with_active_target) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000,
        "[SIMPLE_AVOIDANCE] retain associated target despite velocity spike uuid=%s speed=%.2fm/s",
        uuid.c_str(), speed);
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
    const double hysteresis =
      use_hold_hysteresis ? parameters_->target_hold_lateral_hysteresis : 0.0;
    if (!isTargetWithinOverlap(
          lateral_offset, object_half_width, ego_half_width, parameters_->lateral_margin,
          hysteresis)) {
      continue;
    }

    AvoidanceTarget target;
    target.pose = pose;
    target.longitudinal_distance = longitudinal_distance;
    target.lateral_offset = lateral_offset;
    target.object_half_width = object_half_width;
    target.object_half_length = object_half_length;
    target.uuid = uuid;
    target.last_seen = clock_->now();

    // New targets are admitted only when both the lateral shift and the longitudinal
    // transition can be completed before the obstacle. An already associated target is
    // allowed through this filter so that COMMITTED/RETURNING lifecycle handling remains
    // stable across a transient re-detection failure.
    if (!preferred_uuid.has_value()) {
      const auto shift_result = calcShiftLength(target, *parameters_, ego_half_width);
      if (shift_result.reason != InfeasibleReason::NONE) {
        continue;
      }
      const double ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
      const auto feasibility_result =
        checkFeasibility(target, shift_result.shift_length, *parameters_, ego_speed);
      if (feasibility_result.reason != InfeasibleReason::NONE) {
        continue;
      }
    }

    if (preferred_uuid.has_value()) {
      // Keep the same association order as static_obstacle_avoidance::updateStoredObjects():
      // UUID first, then a position-distance fallback for tracker UUID changes.
      if (uuid_match) {
        return target;
      }
      if (spatial_match && association_distance < min_association_distance) {
        spatially_matched_target = target;
        min_association_distance = association_distance;
      }
      continue;
    }

    if (longitudinal_distance < min_longitudinal) {
      nearest_target = target;
      min_longitudinal = longitudinal_distance;
    }
  }

  if (preferred_uuid.has_value() && spatially_matched_target.has_value()) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] target UUID changed old=%s new=%s; associated by position distance=%.2fm",
      preferred_uuid->c_str(), spatially_matched_target->uuid.c_str(), min_association_distance);
    return spatially_matched_target;
  }

  if (preferred_uuid.has_value()) {
    // Once a target is active, never silently replace it with an unrelated nearest object. The
    // caller will hold the last stable target and decide whether to continue or stop safely.
    return std::nullopt;
  }

  return nearest_target;
}

std::optional<AvoidanceTarget> SimpleAvoidanceModule::updateTargetMetrics(
  const AvoidanceTarget & target) const
{
  if (reference_path_.points.empty()) {
    return std::nullopt;
  }

  auto updated = target;
  const auto ego_pos = planner_data_->self_odometry->pose.pose.position;
  const auto nearest_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(reference_path_.points, target.pose.position);
  updated.longitudinal_distance = autoware::motion_utils::calcSignedArcLength(
    reference_path_.points, ego_pos, target.pose.position);
  updated.lateral_offset = autoware::motion_utils::calcLateralOffset(
    reference_path_.points, target.pose.position, nearest_seg_idx);
  return updated;
}

std::optional<AvoidanceTarget> SimpleAvoidanceModule::getActiveTargetOrHeldTarget()
{
  const auto now = clock_->now();
  if (!active_target_.has_value()) {
    if (auto target = detectTarget()) {
      active_target_ = target;
      RCLCPP_INFO_THROTTLE(
        getLogger(), *clock_, 1000,
        "[SIMPLE_AVOIDANCE] active target locked uuid=%s lon=%.2fm lat=%.2fm",
        active_target_->uuid.c_str(), active_target_->longitudinal_distance,
        active_target_->lateral_offset);
      return active_target_;
    }
    return std::nullopt;
  }

  if (auto target = detectTarget(active_target_->uuid, true)) {
    active_target_ = target;
    debug_data_.target = active_target_;
    return active_target_;
  }

  if (auto updated = updateTargetMetrics(*active_target_)) {
    active_target_ = updated;
  }

  if (isTargetPassed(*active_target_, *parameters_)) {
    RCLCPP_INFO_THROTTLE(
      getLogger(), *clock_, 2000, "[SIMPLE_AVOIDANCE] active target passed uuid=%s lon=%.2fm",
      active_target_->uuid.c_str(), active_target_->longitudinal_distance);
    return std::nullopt;
  }

  if (!isTargetHoldExpired(*active_target_, now, parameters_->target_lost_time_threshold)) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] target temporarily lost but held uuid=%s lon=%.2fm lat=%.2fm "
      "last_seen_age=%.2fs threshold=%.2fs",
      active_target_->uuid.c_str(), active_target_->longitudinal_distance,
      active_target_->lateral_offset, (now - active_target_->last_seen).seconds(),
      parameters_->target_lost_time_threshold);
    return active_target_;
  }

  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 1000,
    "[SIMPLE_AVOIDANCE] target hold expired uuid=%s last_seen_age=%.2fs threshold=%.2fs",
    active_target_->uuid.c_str(), (now - active_target_->last_seen).seconds(),
    parameters_->target_lost_time_threshold);
  active_target_.reset();
  debug_data_.target.reset();
  return std::nullopt;
}

bool SimpleAvoidanceModule::isEgoOnShiftLine() const
{
  const auto reference_path = path_shifter_.getReferencePath();
  if (reference_path.points.empty()) {
    return false;
  }
  const size_t ego_idx = planner_data_->findEgoIndex(reference_path.points);
  const auto shift_lines = path_shifter_.getShiftLines();
  return std::any_of(shift_lines.begin(), shift_lines.end(), [ego_idx](const auto & shift_line) {
    return shift_line.start_idx < ego_idx && ego_idx < shift_line.end_idx;
  });
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
    const double object_half_length = getObjectHalfLength(object.shape);
    const double overlap = std::abs(lateral_offset) - object_half_width;

    TargetRejectReason reject_reason{TargetRejectReason::MOVING};
    bool rejected = false;
    double threshold = 0.0;
    double shortfall = 0.0;
    double transition_distance = 0.0;
    double dist_to_avoid_start = 0.0;
    double dist_to_shift_end = 0.0;
    double dist_to_obstacle = 0.0;

    if (speed >= parameters_->th_moving_speed) {
      diagnosis.rejected_moving++;
      reject_reason = TargetRejectReason::MOVING;
      rejected = true;
      threshold = parameters_->th_moving_speed;
      shortfall = speed - threshold;
    } else if (
      !current_lanelets_.empty() && !isObjectOverlappingLanelets(object, current_lanelets_)) {
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
    } else {
      AvoidanceTarget target;
      target.pose = pose;
      target.longitudinal_distance = longitudinal_distance;
      target.lateral_offset = lateral_offset;
      target.object_half_width = object_half_width;
      target.object_half_length = object_half_length;
      target.uuid = uuid;

      const auto shift_result = calcShiftLength(target, *parameters_, ego_half_width);
      if (shift_result.reason == InfeasibleReason::NO_ROOM) {
        diagnosis.rejected_no_room++;
        reject_reason = TargetRejectReason::NO_ROOM;
        rejected = true;
        threshold = parameters_->max_shift_length;
        shortfall = shift_result.remaining_gap;
      } else if (shift_result.reason == InfeasibleReason::NONE) {
        const double ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
        const auto feasibility_result =
          checkFeasibility(target, shift_result.shift_length, *parameters_, ego_speed);
        if (feasibility_result.reason == InfeasibleReason::INSUFFICIENT_DISTANCE) {
          diagnosis.rejected_insufficient_distance++;
          reject_reason = TargetRejectReason::INSUFFICIENT_DISTANCE;
          rejected = true;
          threshold = feasibility_result.required_start_distance_before_front;
          shortfall = std::max(
            -feasibility_result.dist_to_avoid_start,
            feasibility_result.dist_to_shift_end - feasibility_result.dist_to_obstacle);
          transition_distance = feasibility_result.transition_distance;
          dist_to_avoid_start = feasibility_result.dist_to_avoid_start;
          dist_to_shift_end = feasibility_result.dist_to_shift_end;
          dist_to_obstacle = feasibility_result.dist_to_obstacle;
        }
      }
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
      diagnosis.nearest_object_half_length = object_half_length;
      diagnosis.nearest_threshold = threshold;
      diagnosis.nearest_shortfall = shortfall;
      diagnosis.nearest_transition_distance = transition_distance;
      diagnosis.nearest_dist_to_avoid_start = dist_to_avoid_start;
      diagnosis.nearest_dist_to_shift_end = dist_to_shift_end;
      diagnosis.nearest_dist_to_obstacle = dist_to_obstacle;
    }
  }

  return diagnosis;
}

ShiftLineArray SimpleAvoidanceModule::buildShiftLines(
  const AvoidanceTarget & target, const double shift_length,
  const double extra_return_distance) const
{
  const auto ego_idx = planner_data_->findEgoIndex(reference_path_.points);
  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);

  const auto feasibility = checkFeasibility(target, shift_length, *parameters_, ego_speed);
  if (feasibility.reason != InfeasibleReason::NONE) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] buildShiftLines rejected infeasible transition reason=%s",
      toString(feasibility.reason));
    return {};
  }

  const double dist_to_avoid_start = feasibility.dist_to_avoid_start;
  const double dist_to_avoid_end =
    dist_to_avoid_start + feasibility.transition_distance;
  const double dist_to_return_start = target.longitudinal_distance + target.object_half_length +
                                      parameters_->return_distance_after_object +
                                      extra_return_distance;
  const double dist_to_return_end =
    dist_to_return_start + feasibility.transition_distance;

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

InfeasibleReason SimpleAvoidanceModule::validateVehicleRoadBoundary(
  const PathWithLaneId & path) const
{
  if (path.points.empty()) {
    return InfeasibleReason::PATH_GENERATION_FAILED;
  }

  // The generated shifted path normally has no message-level bounds. When no RouteHandler is
  // available (for example in an isolated module test), use the upstream path that supplied the
  // geometry, not the generated path, as the explicit fallback boundary source.
  const auto & boundary_source_path =
    getPreviousModuleOutput().path.points.empty() ? path : getPreviousModuleOutput().path;
  const auto boundaries = makeOriginalRoadBoundaries(current_lanelets_, boundary_source_path);
  if (!boundaries.has_value()) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] boundary unavailable: lanelets=%zu path_left=%zu path_right=%zu",
      current_lanelets_.size(), boundary_source_path.left_bound.size(),
      boundary_source_path.right_bound.size());
    return InfeasibleReason::BOUNDARY_UNAVAILABLE;
  }

  const auto & left_bound = boundaries->left;
  const auto & right_bound = boundaries->right;

  const auto vehicle_footprint = planner_data_->parameters.vehicle_info.createFootprint();
  const size_t ego_index = planner_data_->findEgoIndex(path.points);
  std::vector<autoware_utils::Polygon2d> footprints;
  const auto append_footprint = [&](const geometry_msgs::msg::Pose & pose) {
    autoware_utils::Polygon2d footprint;
    footprint.outer() =
      autoware_utils::transform_vector(vehicle_footprint, autoware_utils::pose2transform(pose));
    boost::geometry::correct(footprint);
    footprints.push_back(std::move(footprint));
  };
  append_footprint(path.points.at(ego_index).point.pose);
  constexpr double interpolation_distance = 0.1;
  constexpr double interpolation_angle = M_PI / 180.0;
  for (size_t i = ego_index + 1; i < path.points.size(); ++i) {
    const auto & previous_pose = path.points.at(i - 1).point.pose;
    const auto & current_pose = path.points.at(i).point.pose;
    const double distance = autoware_utils::calc_distance2d(previous_pose, current_pose);
    const auto & previous_orientation = previous_pose.orientation;
    const auto & current_orientation = current_pose.orientation;
    const double orientation_dot = std::abs(
      previous_orientation.x * current_orientation.x +
      previous_orientation.y * current_orientation.y +
      previous_orientation.z * current_orientation.z +
      previous_orientation.w * current_orientation.w);
    const double angle = 2.0 * std::acos(std::clamp(orientation_dot, 0.0, 1.0));
    const size_t interpolation_count = static_cast<size_t>(std::max(
      1.0, std::ceil(std::max(distance / interpolation_distance, angle / interpolation_angle))));
    for (size_t step = 1; step <= interpolation_count; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(interpolation_count);
      append_footprint(
        autoware_utils::calc_interpolated_pose(previous_pose, current_pose, ratio, false));
    }
  }

  std::vector<autoware_utils::Polygon2d> swept_areas;
  if (footprints.size() == 1) {
    swept_areas = footprints;
  } else {
    swept_areas.reserve(footprints.size() - 1);
    for (size_t i = 1; i < footprints.size(); ++i) {
      autoware_utils::MultiPoint2d combined;
      for (const auto & point : footprints.at(i - 1).outer()) {
        combined.push_back(point);
      }
      for (const auto & point : footprints.at(i).outer()) {
        combined.push_back(point);
      }
      autoware_utils::Polygon2d swept_area;
      boost::geometry::convex_hull(combined, swept_area);
      boost::geometry::correct(swept_area);
      swept_areas.push_back(std::move(swept_area));
    }
  }

  double longitudinal_extension = 1.0;
  for (const auto & point : vehicle_footprint) {
    longitudinal_extension = std::max(longitudinal_extension, std::hypot(point.x(), point.y()));
  }
  const auto corridor = makeLaneCorridor(left_bound, right_bound, longitudinal_extension);
  if (!corridor.has_value()) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] boundary corridor unavailable: source=%s left=%zu right=%zu",
      boundaries->from_lanelets ? "lanelet" : "path", left_bound.size(), right_bound.size());
    return InfeasibleReason::BOUNDARY_UNAVAILABLE;
  }

  double minimum_boundary_clearance = std::numeric_limits<double>::max();
  for (const auto & swept_area : swept_areas) {
    const double left_clearance = boost::geometry::distance(swept_area, left_bound);
    const double right_clearance = boost::geometry::distance(swept_area, right_bound);
    minimum_boundary_clearance =
      std::min(minimum_boundary_clearance, std::min(left_clearance, right_clearance));
    if (
      boost::geometry::intersects(swept_area, left_bound) ||
      boost::geometry::intersects(swept_area, right_bound) ||
      left_clearance < parameters_->road_boundary_margin ||
      right_clearance < parameters_->road_boundary_margin ||
      !isOnDrivableSide(swept_area, left_bound, reference_path_.points) ||
      !isOnDrivableSide(swept_area, right_bound, reference_path_.points) ||
      !boost::geometry::covered_by(swept_area, *corridor)) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000,
        "[SIMPLE_AVOIDANCE] footprint outside original lane boundary: source=%s "
        "min_clearance=%.2fm required=%.2fm",
        boundaries->from_lanelets ? "lanelet" : "path", minimum_boundary_clearance,
        parameters_->road_boundary_margin);
      return InfeasibleReason::FOOTPRINT_OUT_OF_BOUNDARY;
    }
  }
  RCLCPP_DEBUG(
    getLogger(),
    "[SIMPLE_AVOIDANCE] original lane boundary validated: source=%s left=%zu right=%zu "
    "min_clearance=%.2fm required=%.2fm",
    boundaries->from_lanelets ? "lanelet" : "path", left_bound.size(), right_bound.size(),
    minimum_boundary_clearance, parameters_->road_boundary_margin);
  return InfeasibleReason::NONE;
}

InfeasibleReason SimpleAvoidanceModule::validateArticulatedPath(const PathWithLaneId & path) const
{
  if (active_trailer_configuration_.geometries.empty()) {
    return InfeasibleReason::NONE;
  }

  std::vector<geometry_msgs::msg::Pose> sampled_poses;
  if (path.points.empty()) {
    return InfeasibleReason::PATH_GENERATION_FAILED;
  }
  sampled_poses.push_back(path.points.front().point.pose);
  double accumulated_distance = 0.0;
  for (size_t i = 1; i < path.points.size(); ++i) {
    accumulated_distance += autoware_utils::calc_distance2d(
      path.points.at(i - 1).point.pose.position, path.points.at(i).point.pose.position);
    if (accumulated_distance >= parameters_->trailer_footprint_sampling_interval) {
      sampled_poses.push_back(path.points.at(i).point.pose);
      accumulated_distance = 0.0;
    }
  }
  if (
    autoware_utils::calc_distance2d(
      sampled_poses.back().position, path.points.back().point.pose.position) > 1e-3) {
    sampled_poses.push_back(path.points.back().point.pose);
  }

  const auto articulated_path = predictArticulatedPath(
    sampled_poses, active_trailer_configuration_.geometries,
    parameters_->tractor_rear_axle_to_hitch);
  if (!articulated_path.articulation_valid) {
    return InfeasibleReason::ARTICULATION_LIMIT;
  }

  const auto ego_position = planner_data_->self_odometry->pose.pose.position;
  size_t ego_sample_index = 0;
  double min_ego_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < sampled_poses.size(); ++i) {
    const double distance =
      autoware_utils::calc_distance2d(ego_position, sampled_poses.at(i).position);
    if (distance < min_ego_distance) {
      min_ego_distance = distance;
      ego_sample_index = i;
    }
  }

  const auto & boundary_source_path =
    getPreviousModuleOutput().path.points.empty() ? path : getPreviousModuleOutput().path;
  const auto boundaries = makeOriginalRoadBoundaries(current_lanelets_, boundary_source_path);
  if (!boundaries.has_value()) {
    return InfeasibleReason::BOUNDARY_UNAVAILABLE;
  }
  const auto & left_bound = boundaries->left;
  const auto & right_bound = boundaries->right;
  double longitudinal_extension = 1.0;
  for (const auto & point : planner_data_->parameters.vehicle_info.createFootprint()) {
    longitudinal_extension = std::max(longitudinal_extension, std::hypot(point.x(), point.y()));
  }
  for (const auto & geometry : active_trailer_configuration_.geometries) {
    longitudinal_extension = std::max(
      longitudinal_extension, std::hypot(geometry.axle_to_body_front, geometry.width * 0.5));
    longitudinal_extension = std::max(
      longitudinal_extension, std::hypot(geometry.axle_to_body_rear, geometry.width * 0.5));
  }
  const auto corridor = makeLaneCorridor(left_bound, right_bound, longitudinal_extension);
  if (!corridor.has_value()) {
    return InfeasibleReason::BOUNDARY_UNAVAILABLE;
  }

  std::vector<autoware_utils::Polygon2d> static_obstacles;
  if (planner_data_->dynamic_object) {
    for (const auto & object : planner_data_->dynamic_object->objects) {
      const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
      const auto uuid = autoware_utils_uuid::to_hex_string(object.object_id);
      const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
      const bool is_active_target = [&]() {
        if (!active_target_.has_value()) {
          return false;
        }
        if (uuid == active_target_->uuid) {
          return true;
        }
        const double distance = std::hypot(
          pose.position.x - active_target_->pose.position.x,
          pose.position.y - active_target_->pose.position.y);
        return distance < 1.5 &&
               std::abs(getObjectHalfWidth(object.shape) - active_target_->object_half_width) <=
                 1.0 &&
               std::abs(getObjectHalfLength(object.shape) - active_target_->object_half_length) <=
                 1.0;
      }();
      if (
        std::hypot(twist.linear.x, twist.linear.y) < parameters_->th_moving_speed ||
        is_active_target) {
        static_obstacles.push_back(autoware_utils::to_polygon2d(object));
      }
    }
  }

  const auto tractor_footprint = planner_data_->parameters.vehicle_info.createFootprint();
  const auto make_swept_footprint =
    [](const autoware_utils::Polygon2d & previous, const autoware_utils::Polygon2d & current) {
      autoware_utils::MultiPoint2d combined;
      for (const auto & point : previous.outer()) {
        combined.push_back(point);
      }
      for (const auto & point : current.outer()) {
        combined.push_back(point);
      }
      autoware_utils::Polygon2d swept;
      boost::geometry::convex_hull(combined, swept);
      boost::geometry::correct(swept);
      return swept;
    };
  const auto validate_footprint = [&](const autoware_utils::Polygon2d & footprint) {
    if (
      boost::geometry::intersects(footprint, left_bound) ||
      boost::geometry::intersects(footprint, right_bound) ||
      boost::geometry::distance(footprint, left_bound) < parameters_->road_boundary_margin ||
      boost::geometry::distance(footprint, right_bound) < parameters_->road_boundary_margin ||
      !boost::geometry::covered_by(footprint, *corridor)) {
      return InfeasibleReason::FOOTPRINT_OUT_OF_BOUNDARY;
    }
    for (const auto & obstacle : static_obstacles) {
      if (
        boost::geometry::intersects(footprint, obstacle) ||
        boost::geometry::distance(footprint, obstacle) < parameters_->lateral_margin) {
        return InfeasibleReason::TRAILER_COLLISION;
      }
    }
    return InfeasibleReason::NONE;
  };

  std::vector<autoware_utils::Polygon2d> previous_footprints;
  for (size_t path_index = ego_sample_index; path_index < articulated_path.poses.size();
       ++path_index) {
    const auto & articulated_pose = articulated_path.poses.at(path_index);
    std::vector<autoware_utils::Polygon2d> footprints;
    autoware_utils::Polygon2d tractor_polygon;
    tractor_polygon.outer() = autoware_utils::transform_vector(
      tractor_footprint, autoware_utils::pose2transform(articulated_pose.tractor));
    boost::geometry::correct(tractor_polygon);
    footprints.push_back(std::move(tractor_polygon));
    for (size_t i = 0; i < articulated_pose.trailers.size(); ++i) {
      footprints.push_back(createTrailerFootprint(
        articulated_pose.trailers.at(i), active_trailer_configuration_.geometries.at(i)));
    }

    for (size_t footprint_index = 0; footprint_index < footprints.size(); ++footprint_index) {
      const auto current_reason = validate_footprint(footprints.at(footprint_index));
      if (current_reason != InfeasibleReason::NONE) {
        return current_reason;
      }
      if (previous_footprints.size() == footprints.size()) {
        const auto swept = make_swept_footprint(
          previous_footprints.at(footprint_index), footprints.at(footprint_index));
        const auto swept_reason = validate_footprint(swept);
        if (swept_reason != InfeasibleReason::NONE) {
          return swept_reason;
        }
      }
    }
    previous_footprints = std::move(footprints);
  }

  return InfeasibleReason::NONE;
}

std::optional<ShiftedPath> SimpleAvoidanceModule::generateTrailerAwarePath(
  const AvoidanceTarget & target, const double initial_shift_length,
  ShiftLineArray & selected_lines, InfeasibleReason & failure_reason) const
{
  const auto start_time = std::chrono::steady_clock::now();
  const double direction = initial_shift_length >= 0.0 ? 1.0 : -1.0;
  const double initial_magnitude = std::abs(initial_shift_length);
  failure_reason = InfeasibleReason::TRAILER_COLLISION;

  for (double shift_magnitude = initial_magnitude;
       shift_magnitude <= parameters_->max_shift_length + 1e-6;
       shift_magnitude += parameters_->trailer_lateral_search_resolution) {
    for (double extra_return = 0.0;
         extra_return <= parameters_->trailer_max_extra_return_distance + 1e-6;
         extra_return += parameters_->trailer_return_search_resolution) {
      const auto elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
          .count();
      if (elapsed_ms > parameters_->trailer_max_planning_time_ms) {
        failure_reason = InfeasibleReason::COMPUTATION_TIMEOUT;
        return std::nullopt;
      }

      const double ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
      const auto longitudinal_feasibility = checkFeasibility(
        target, direction * shift_magnitude, *parameters_, ego_speed);
      if (longitudinal_feasibility.reason != InfeasibleReason::NONE) {
        // A larger lateral candidate can require a longer jerk transition. Do not
        // evaluate or select a trailer path whose shift ends at/after the obstacle.
        failure_reason = longitudinal_feasibility.reason;
        continue;
      }

      const auto lines = buildShiftLines(target, direction * shift_magnitude, extra_return);
      auto candidate_shifter = path_shifter_;
      candidate_shifter.setShiftLines(lines);
      ShiftedPath candidate;
      if (!candidate_shifter.generate(&candidate) || candidate.path.points.empty()) {
        failure_reason = InfeasibleReason::PATH_GENERATION_FAILED;
        continue;
      }
      setOrientation(&candidate.path);
      failure_reason = validateArticulatedPath(candidate.path);
      if (failure_reason == InfeasibleReason::NONE) {
        selected_lines = lines;
        return candidate;
      }
    }
  }
  return std::nullopt;
}

BehaviorModuleOutput SimpleAvoidanceModule::stopForInfeasiblePath(
  const InfeasibleReason reason, const PassThroughDebugInfo & debug_info) const
{
  auto output = make_safe_stop_output();
  if (
    reason == InfeasibleReason::ROAD_BOUNDARY || reason == InfeasibleReason::BOUNDARY_UNAVAILABLE ||
    reason == InfeasibleReason::FOOTPRINT_OUT_OF_BOUNDARY) {
    output = getPreviousModuleOutput();
    const auto fallback_path = output.path.points.empty() ? reference_path_ : output.path;
    output.path =
      make_safe_stop_path(PathWithLaneId{}, fallback_path, *planner_data_->self_odometry);
    output.reference_path = reference_path_.points.empty() ? output.path : reference_path_;
  }
  debug_data_.last_reason = reason;
  logPassThroughDetails(
    getLogger(), *clock_, reason, debug_info, *parameters_,
    planner_data_->parameters.vehicle_width * 0.5);
  return output;
}

bool SimpleAvoidanceModule::isLateralExecutionIncomplete() const
{
  if (!planner_data_ || reference_path_.points.empty()) {
    return true;
  }

  const bool has_valid_previous_shifted_path =
    prev_output_.path.points.size() >= 2 &&
    prev_output_.shift_length.size() == prev_output_.path.points.size();
  if (!has_valid_previous_shifted_path) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] lateral tracking diagnostics: distance_to_shift_start=not_available "
      "commitment_lead_distance=%.2fm expected_current_shift=not_available "
      "actual_lateral_offset=%.2fm lateral_tracking_error=not_available lifecycle_state=%s "
      "reason=previous_path_invalid",
      std::max(0.0, parameters_->commitment_distance_before_shift_start),
      getEgoLateralOffsetToReference(), toString(lifecycle_state_));
    return true;
  }

  const double expected_current_shift =
    getClosestShiftLength(prev_output_, getEgoPose().position);
  const double actual_lateral_offset = getEgoLateralOffsetToReference();
  const double lateral_tracking_error =
    calcLateralTrackingError(expected_current_shift, actual_lateral_offset);
  const auto shift_lines = path_shifter_.getShiftLines();
  const double distance_to_shift_start = shift_lines.empty()
                                           ? std::numeric_limits<double>::quiet_NaN()
                                           : autoware::motion_utils::calcSignedArcLength(
                                               reference_path_.points, getEgoPose().position,
                                               shift_lines.front().start.position);
  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 1000,
    "[SIMPLE_AVOIDANCE] lateral tracking diagnostics: distance_to_shift_start=%.2fm "
    "commitment_lead_distance=%.2fm expected_current_shift=%.2fm actual_lateral_offset=%.2fm "
    "lateral_tracking_error=%.2fm lifecycle_state=%s",
    distance_to_shift_start, parameters_->commitment_distance_before_shift_start,
    expected_current_shift, actual_lateral_offset, lateral_tracking_error,
    toString(lifecycle_state_));
  return isLateralExecutionLagging(
    expected_current_shift, actual_lateral_offset, parameters_->lateral_execution_threshold);
}

BehaviorModuleOutput SimpleAvoidanceModule::stopBeforeTarget(
  const AvoidanceTarget & target, const InfeasibleReason reason,
  const PassThroughDebugInfo & debug_info) const
{
  auto output = getPreviousModuleOutput();
  if (hasReusablePreviousPath()) {
    const auto previous_path_reason = validateVehicleRoadBoundary(prev_output_.path);
    if (previous_path_reason != InfeasibleReason::NONE) {
      return stopForInfeasiblePath(previous_path_reason, debug_info);
    }
    output.path = prev_output_.path;
  }
  if (output.path.points.empty()) {
    output.path = reference_path_;
  }
  if (output.path.points.empty()) {
    output = make_safe_stop_output();
  }

  if (!output.path.points.empty()) {
    const auto & vehicle_parameters = planner_data_->parameters;
    const double base_link_to_front = std::max(
      {0.0, vehicle_parameters.base_link2front,
       vehicle_parameters.wheel_base + vehicle_parameters.front_overhang,
       vehicle_parameters.vehicle_info.max_longitudinal_offset_m});
    const double stop_distance = target.longitudinal_distance - target.object_half_length -
                                 base_link_to_front - parameters_->lateral_margin;

    std::optional<size_t> stop_index;
    if (stop_distance > 0.0) {
      stop_index = autoware::motion_utils::insertStopPoint(
        planner_data_->self_odometry->pose.pose, stop_distance, output.path.points);
    }
    const size_t ego_index = planner_data_->findEgoIndex(output.path.points);
    const size_t first_stop_index = stop_index.value_or(ego_index);
    for (size_t i = std::min(first_stop_index, output.path.points.size());
         i < output.path.points.size(); ++i) {
      output.path.points.at(i).point.longitudinal_velocity_mps = 0.0;
    }
    if (output.path.points.size() == 1) {
      output.path.points.front().point.longitudinal_velocity_mps = 0.0;
    }
  }

  if (!reference_path_.points.empty()) {
    output.reference_path = reference_path_;
  }
  const double ego_half_width = planner_data_->parameters.vehicle_width * 0.5;
  const auto shift = calcShiftLength(target, *parameters_, ego_half_width);
  const double actual_lateral_offset = getEgoLateralOffsetToReference();
  const bool has_valid_previous_shifted_path =
    prev_output_.path.points.size() >= 2 &&
    prev_output_.shift_length.size() == prev_output_.path.points.size();
  const double expected_current_shift = has_valid_previous_shifted_path
                                          ? getClosestShiftLength(prev_output_, getEgoPose().position)
                                          : std::numeric_limits<double>::quiet_NaN();
  const double lateral_tracking_error = has_valid_previous_shifted_path
                                          ? calcLateralTrackingError(
                                              expected_current_shift, actual_lateral_offset)
                                          : std::numeric_limits<double>::quiet_NaN();
  debug_data_.last_reason = reason;
  logPassThroughDetails(getLogger(), *clock_, reason, debug_info, *parameters_, ego_half_width);
  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 1000,
    "[SIMPLE_AVOIDANCE] safety stop before target uuid=%s reason=%s target_lon=%.2fm "
    "target_half_length=%.2fm planned_final_shift=%.2fm expected_current_shift=%.2fm "
    "actual_lateral_offset=%.2fm lateral_tracking_error=%.2fm shift_calc_reason=%s "
    "lifecycle_state=%s",
    target.uuid.c_str(), toString(reason), target.longitudinal_distance, target.object_half_length,
    shift.shift_length, expected_current_shift, actual_lateral_offset, lateral_tracking_error,
    toString(shift.reason), toString(lifecycle_state_));
  return output;
}

double SimpleAvoidanceModule::getEgoLateralOffsetToReference() const
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

bool SimpleAvoidanceModule::isCommittedOrReturning() const
{
  return lifecycle_state_ == AvoidanceLifecycleState::COMMITTED ||
         lifecycle_state_ == AvoidanceLifecycleState::RETURNING;
}

std::optional<ShiftedPath> SimpleAvoidanceModule::generateEgoAlignedReturnPath()
{
  if (reference_path_.points.size() < 2) {
    return std::nullopt;
  }

  const double actual_offset = getEgoLateralOffsetToReference();
  PathShifter recovery_shifter;
  recovery_shifter.setPath(reference_path_);

  if (std::abs(actual_offset) > parameters_->lateral_execution_threshold) {
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

    // PathShifter derives each line from the accumulated end shifts; start_shift_length is only
    // descriptive. Rebuild the measured offset over the already-travelled part of the path, then
    // connect it back to zero ahead of ego. This places the new path on the physical vehicle at
    // start_idx without needing access to PathShifter's private base_offset_.
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
    recovery_shifter.setShiftLines({measured_offset, return_shift});
    ego_aligned_return_active_ = true;
  } else {
    ego_aligned_return_active_ = false;
  }

  ShiftedPath recovery_path;
  if (!recovery_shifter.generate(&recovery_path) || recovery_path.path.points.empty()) {
    return std::nullopt;
  }
  setOrientation(&recovery_path.path);
  path_shifter_ = recovery_shifter;
  return recovery_path;
}

std::optional<ShiftedPath> SimpleAvoidanceModule::generateEgoAlignedAvoidancePath(
  const AvoidanceTarget & target, const double shift_length, ShiftLineArray & selected_lines)
{
  if (reference_path_.points.size() < 2) {
    return std::nullopt;
  }

  const double actual_offset = getEgoLateralOffsetToReference();
  selected_lines = buildShiftLines(target, shift_length);
  selected_lines.front().start_shift_length = actual_offset;

  PathShifter ego_aligned_shifter;
  ego_aligned_shifter.setPath(reference_path_);
  if (std::abs(actual_offset) > parameters_->lateral_execution_threshold) {
    const size_t ego_idx = planner_data_->findEgoIndex(reference_path_.points);
    if (ego_idx <= 1) {
      return std::nullopt;
    }

    // Reconstruct the already-travelled lateral offset so the new shift begins at the measured
    // vehicle pose instead of PathShifter's stale planned base offset.
    ShiftLine measured_offset;
    measured_offset.start_shift_length = 0.0;
    measured_offset.end_shift_length = actual_offset;
    measured_offset.start_idx = 0;
    measured_offset.end_idx = ego_idx;
    measured_offset.start = reference_path_.points.front().point.pose;
    measured_offset.end = reference_path_.points.at(ego_idx).point.pose;
    selected_lines.insert(selected_lines.begin(), measured_offset);
  }
  ego_aligned_shifter.setShiftLines(selected_lines);

  ShiftedPath shifted_path;
  if (!ego_aligned_shifter.generate(&shifted_path) || shifted_path.path.points.empty()) {
    return std::nullopt;
  }
  setOrientation(&shifted_path.path);
  path_shifter_ = ego_aligned_shifter;
  return shifted_path;
}

BehaviorModuleOutput SimpleAvoidanceModule::continueCommittedPath(
  const PassThroughDebugInfo & debug_info)
{
  const double actual_offset = getEgoLateralOffsetToReference();
  const bool planned_return_is_exhausted =
    path_shifter_.getShiftLines().empty() &&
    std::abs(path_shifter_.getBaseOffset()) <= parameters_->lateral_execution_threshold;

  ShiftedPath shifted_path;
  const auto previous_path_shifter = path_shifter_;
  if (
    ego_aligned_return_active_ ||
    (planned_return_is_exhausted &&
     std::abs(actual_offset) > parameters_->lateral_execution_threshold)) {
    const auto recovery_path = generateEgoAlignedReturnPath();
    if (!recovery_path.has_value()) {
      return handlePathGenerationFailure(debug_info);
    }
    shifted_path = *recovery_path;
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] extending return path from measured ego offset=%.2fm to avoid a "
      "trajectory jump",
      actual_offset);
  } else if (
    !path_shifter_.generate(&shifted_path) || shifted_path.path.points.empty() ||
    !isGeneratedPathContinuous(shifted_path)) {
    return handlePathGenerationFailure(debug_info);
  } else {
    setOrientation(&shifted_path.path);
  }

  const auto boundary_reason = validateVehicleRoadBoundary(shifted_path.path);
  if (boundary_reason != InfeasibleReason::NONE) {
    path_shifter_ = previous_path_shifter;
    return stopForInfeasiblePath(boundary_reason, debug_info);
  }

  prev_output_ = shifted_path;
  path_generation_failure_started_.reset();
  debug_data_.last_reason = InfeasibleReason::NONE;
  debug_data_.path_shifter = std::make_shared<PathShifter>(path_shifter_);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);
  return adjustDrivableArea(shifted_path);
}

bool SimpleAvoidanceModule::isGeneratedPathContinuous(const ShiftedPath & path) const
{
  if (prev_output_.path.points.empty() || path.path.points.empty()) {
    return prev_output_.path.points.empty();
  }
  const auto & ego_position = getEgoPose().position;
  const double previous_shift = getClosestShiftLength(prev_output_, ego_position);
  const double generated_shift = getClosestShiftLength(path, ego_position);
  constexpr double max_lateral_jump = 0.1;
  return std::abs(previous_shift - generated_shift) <= max_lateral_jump;
}

bool SimpleAvoidanceModule::isCommitmentDetected() const
{
  const auto shift_lines = path_shifter_.getShiftLines();
  if (shift_lines.empty() || reference_path_.points.empty()) {
    return false;
  }
  const size_t ego_idx = planner_data_->findEgoIndex(reference_path_.points);
  const double ego_shift = getClosestShiftLength(prev_output_, getEgoPose().position);
  const double distance_to_shift_start = autoware::motion_utils::calcSignedArcLength(
    reference_path_.points, getEgoPose().position, shift_lines.front().start.position);
  const double commitment_lead_distance =
    std::max(0.0, parameters_->commitment_distance_before_shift_start);
  const double actual_lateral_offset = getEgoLateralOffsetToReference();
  const double lateral_tracking_error = calcLateralTrackingError(ego_shift, actual_lateral_offset);
  const bool within_commitment_window =
    isWithinCommitmentWindow(distance_to_shift_start, commitment_lead_distance);
  const bool committed =
    within_commitment_window || ego_idx >= shift_lines.front().start_idx || isEgoOnShiftLine() ||
    std::abs(ego_shift) > parameters_->lateral_execution_threshold ||
    std::abs(path_shifter_.getBaseOffset()) > parameters_->lateral_execution_threshold;
  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 1000,
    "[SIMPLE_AVOIDANCE] commitment diagnostics: distance_to_shift_start=%.2fm "
    "commitment_lead_distance=%.2fm expected_current_shift=%.2fm actual_lateral_offset=%.2fm "
    "lateral_tracking_error=%.2fm lifecycle_state=%s committed=%d",
    distance_to_shift_start, commitment_lead_distance, ego_shift, actual_lateral_offset,
    lateral_tracking_error, toString(lifecycle_state_), committed);
  return committed;
}

bool SimpleAvoidanceModule::hasReusablePreviousPath() const
{
  if (prev_output_.path.points.size() < 2) {
    return false;
  }
  const auto & ego_position = getEgoPose().position;
  const size_t ego_idx = planner_data_->findEgoIndex(prev_output_.path.points);
  if (ego_idx >= prev_output_.path.points.size() - 1) {
    return false;
  }
  constexpr double minimum_forward_coverage = 1.0;
  return autoware::motion_utils::calcSignedArcLength(
           prev_output_.path.points, ego_position,
           prev_output_.path.points.back().point.pose.position) >= minimum_forward_coverage;
}

BehaviorModuleOutput SimpleAvoidanceModule::make_safe_stop_output() const
{
  auto output = getPreviousModuleOutput();
  const auto fallback_path = output.path.points.empty() ? reference_path_ : output.path;
  output.path =
    make_safe_stop_path(prev_output_.path, fallback_path, *planner_data_->self_odometry);
  if (!reference_path_.points.empty()) {
    output.reference_path = reference_path_;
  } else if (output.reference_path.points.empty()) {
    output.reference_path = output.path;
  }
  return output;
}

BehaviorModuleOutput SimpleAvoidanceModule::handlePathGenerationFailure(
  const PassThroughDebugInfo & debug_info)
{
  const auto now = clock_->now();
  if (!path_generation_failure_started_.has_value()) {
    path_generation_failure_started_ = now;
  }

  AvoidanceLifecycleObservation observation;
  observation.state = lifecycle_state_;
  observation.generation_succeeded = false;
  observation.has_continuous_previous_path = hasReusablePreviousPath();
  observation.failure_duration = (now - *path_generation_failure_started_).seconds();
  const auto decision = decideAvoidanceLifecycle(
    observation, parameters_->path_generation_failure_timeout,
    parameters_->completion_stable_count);
  lifecycle_state_ = decision.next_state;

  if (decision.action == AvoidanceLifecycleAction::CANCEL_CANDIDATE) {
    path_shifter_.setShiftLines({});
    prev_output_ = ShiftedPath{};
    active_target_.reset();
    path_generation_failure_started_.reset();
    // A candidate generation failure means the module cannot prove a safe trajectory. Keep the
    // geometry valid for downstream consumers, but remove the previous forward command instead of
    // passing through the obstacle at the upstream speed.
    auto output = make_safe_stop_output();
    debug_data_.last_reason = InfeasibleReason::PATH_GENERATION_FAILED;
    logPassThroughDetails(
      getLogger(), *clock_, InfeasibleReason::PATH_GENERATION_FAILED, debug_info, *parameters_,
      planner_data_->parameters.vehicle_width * 0.5);
    return output;
  }
  if (decision.action == AvoidanceLifecycleAction::KEEP_LAST_VALID_PATH) {
    debug_data_.last_reason = InfeasibleReason::PATH_GENERATION_FAILED;
    return adjustDrivableArea(prev_output_);
  }
  if (decision.action == AvoidanceLifecycleAction::INSERT_FEASIBLE_STOP) {
    auto output = adjustDrivableArea(prev_output_);
    // Conservative fixed fallback because the common planner data exposes acceleration but no
    // longitudinal jerk limit. Keep this local rather than adding another lifecycle parameter.
    constexpr double conservative_longitudinal_jerk = 1.0;
    const auto stop_pose = utils::insert_feasible_stop_point(
      output.path, planner_data_, planner_data_->parameters.min_acc, conservative_longitudinal_jerk,
      "simple avoidance path generation failure");
    if (!stop_pose.has_value()) {
      const size_t ego_index = planner_data_->findEgoIndex(output.path.points);
      for (size_t i = ego_index; i < output.path.points.size(); ++i) {
        output.path.points.at(i).point.longitudinal_velocity_mps = 0.0;
      }
    }
    debug_data_.last_reason = InfeasibleReason::PATH_GENERATION_FAILED;
    return output;
  }
  if (decision.action == AvoidanceLifecycleAction::PublishSafeStop) {
    auto output = make_safe_stop_output();
    debug_data_.last_reason = InfeasibleReason::PATH_GENERATION_FAILED;
    logPassThroughDetails(
      getLogger(), *clock_, InfeasibleReason::PATH_GENERATION_FAILED, debug_info, *parameters_,
      planner_data_->parameters.vehicle_width * 0.5);
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 1000,
      "[SIMPLE_AVOIDANCE] publishing non-empty safe-stop fallback: points=%zu",
      output.path.points.size());
    return output;
  }
  debug_data_.last_reason = InfeasibleReason::PATH_GENERATION_FAILED;
  logPassThroughDetails(
    getLogger(), *clock_, InfeasibleReason::PATH_GENERATION_FAILED, debug_info, *parameters_,
    planner_data_->parameters.vehicle_width * 0.5);
  // Defensive fallback for future lifecycle actions: this module must never violate the common
  // SceneModuleInterface contract by publishing an empty path.
  return make_safe_stop_output();
}

BehaviorModuleOutput SimpleAvoidanceModule::passThrough(
  const InfeasibleReason reason, const PassThroughDebugInfo & debug_info) const
{
  debug_data_.last_reason = reason;
  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;
  logPassThroughDetails(getLogger(), *clock_, reason, debug_info, *parameters_, ego_half_width);

  auto output = getPreviousModuleOutput();
  if (output.path.points.empty() && !reference_path_.points.empty()) {
    output.path = reference_path_;
    output.reference_path = reference_path_;
  }
  if (output.path.points.empty()) {
    output = make_safe_stop_output();
  }
  return output;
}

BehaviorModuleOutput SimpleAvoidanceModule::adjustDrivableArea(const ShiftedPath & path) const
{
  if (path.path.points.empty()) {
    return make_safe_stop_output();
  }

  BehaviorModuleOutput out;
  const auto & p = planner_data_->parameters;
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  // A simple-avoidance shift is required to stay inside the original lanelet. Expanding the
  // drivable area by the shift length would make the later path optimizer treat an adjacent lane
  // or shoulder as valid, reintroducing the exact boundary violation checked above.
  const double left_offset = 0.0;
  const double right_offset = 0.0;

  auto output_path = path.path;
  const size_t current_seg_idx = planner_data_->findEgoSegmentIndex(output_path.points);
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  output_path.points = autoware::motion_utils::cropPoints(
    output_path.points, current_pose.position, current_seg_idx, p.forward_path_length,
    p.backward_path_length + p.input_path_interval);
  if (output_path.points.empty()) {
    return make_safe_stop_output();
  }

  const auto drivable_lanes = utils::generateDrivableLanes(current_lanelets_);
  const auto shorten_lanes = utils::cutOverlappedLanes(output_path, drivable_lanes);
  if (output_path.points.empty()) {
    return make_safe_stop_output();
  }
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
  // Keep a monotonic callback-gap signal in the log.  ProcessingTimeTree only
  // reports completed callbacks, so it cannot explain a long interval in which
  // no tree is published.  This is observation-only and does not alter timing
  // or lifecycle decisions.
  const auto timing_now = std::chrono::steady_clock::now();
  static auto previous_plan_entry = timing_now;
  static uint64_t plan_cycle = 0;
  const auto entry_gap_ms = std::chrono::duration<double, std::milli>(
                              timing_now - previous_plan_entry)
                              .count();
  previous_plan_entry = timing_now;
  ++plan_cycle;
  RCLCPP_INFO(
    getLogger(), "[SIMPLE_AVOIDANCE_TIMING] cycle=%llu entry_gap_ms=%.3f state=%s",
    static_cast<unsigned long long>(plan_cycle), entry_gap_ms, toString(lifecycle_state_));

  PassThroughDebugInfo debug_info;
  debug_info.reference_path_points = reference_path_.points.size();

  if (lifecycle_state_ == AvoidanceLifecycleState::STOPPING) {
    return handlePathGenerationFailure(debug_info);
  }

  if (reference_path_.points.size() < 2) {
    debug_info.no_target = diagnoseNoTarget();
    return passThrough(InfeasibleReason::NO_TARGET, debug_info);
  }

  if (lifecycle_state_ == AvoidanceLifecycleState::CANDIDATE && isCommitmentDetected()) {
    lifecycle_state_ = AvoidanceLifecycleState::COMMITTED;
  }

  const auto target = lifecycle_state_ == AvoidanceLifecycleState::RETURNING
                        ? std::optional<AvoidanceTarget>{}
                        : getActiveTargetOrHeldTarget();
  if (!target.has_value()) {
    if (lifecycle_state_ == AvoidanceLifecycleState::CANDIDATE) {
      path_shifter_.setShiftLines({});
      prev_output_ = ShiftedPath{};
      lifecycle_state_ = AvoidanceLifecycleState::IDLE;
      debug_info.no_target = diagnoseNoTarget();
      return passThrough(InfeasibleReason::NO_TARGET, debug_info);
    }
    if (isCommittedOrReturning()) {
      lifecycle_state_ = AvoidanceLifecycleState::RETURNING;
      auto output = continueCommittedPath(debug_info);
      if (parameters_->publish_debug_marker) {
        setDebugMarkersVisualization();
      }
      return output;
    }
    debug_info.no_target = diagnoseNoTarget();
    lifecycle_state_ = AvoidanceLifecycleState::IDLE;
    return passThrough(InfeasibleReason::NO_TARGET, debug_info);
  }

  active_target_ = target;
  debug_data_.target = target;
  debug_info.target = target;

  const double ego_half_width = planner_data_->parameters.vehicle_width / 2.0;
  const auto shift_result = calcShiftLength(*target, *parameters_, ego_half_width);
  debug_info.shift = shift_result;
  if (shift_result.reason != InfeasibleReason::NONE) {
    if (isCommittedOrReturning()) {
      if (isLateralExecutionIncomplete()) {
        return stopBeforeTarget(*target, InfeasibleReason::LATERAL_EXECUTION_LAG, debug_info);
      }
      return continueCommittedPath(debug_info);
    }
    return stopBeforeTarget(*target, shift_result.reason, debug_info);
  }

  const auto ego_speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  const auto feasibility_result =
    checkFeasibility(*target, shift_result.shift_length, *parameters_, ego_speed);
  debug_info.feasibility = feasibility_result;
  if (feasibility_result.reason != InfeasibleReason::NONE) {
    if (isCommittedOrReturning()) {
      if (isLateralExecutionIncomplete()) {
        return stopBeforeTarget(*target, InfeasibleReason::LATERAL_EXECUTION_LAG, debug_info);
      }
      return continueCommittedPath(debug_info);
    }
    return stopBeforeTarget(*target, feasibility_result.reason, debug_info);
  }

  ShiftedPath shifted_path;
  ShiftLineArray shift_lines;
  const auto previous_path_shifter = path_shifter_;
  if (active_trailer_configuration_.geometries.empty()) {
    const double planned_base_offset = path_shifter_.getBaseOffset();
    const double actual_ego_offset = getEgoLateralOffsetToReference();
    const double base_offset_error = std::abs(planned_base_offset - actual_ego_offset);
    if (base_offset_error > parameters_->lateral_execution_threshold) {
      const auto ego_aligned_path =
        generateEgoAlignedAvoidancePath(*target, shift_result.shift_length, shift_lines);
      if (!ego_aligned_path.has_value()) {
        return handlePathGenerationFailure(debug_info);
      }
      shifted_path = *ego_aligned_path;
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 1000,
        "[SIMPLE_AVOIDANCE] realigned stale planned base offset to measured ego pose: "
        "base_offset=%.2fm actual_ego_offset=%.2fm",
        planned_base_offset, actual_ego_offset);
    } else {
      shift_lines = buildShiftLines(*target, shift_result.shift_length);
      path_shifter_.setShiftLines(shift_lines);
      if (
        !path_shifter_.generate(&shifted_path) || shifted_path.path.points.empty() ||
        !isGeneratedPathContinuous(shifted_path)) {
        return handlePathGenerationFailure(debug_info);
      }
      setOrientation(&shifted_path.path);
    }
  } else {
    InfeasibleReason trailer_failure_reason{InfeasibleReason::TRAILER_COLLISION};
    const auto trailer_path = generateTrailerAwarePath(
      *target, shift_result.shift_length, shift_lines, trailer_failure_reason);
    if (!trailer_path.has_value()) {
      return stopForInfeasiblePath(trailer_failure_reason, debug_info);
    }
    shifted_path = *trailer_path;
    path_shifter_.setShiftLines(shift_lines);
  }
  const auto boundary_reason = validateVehicleRoadBoundary(shifted_path.path);
  if (boundary_reason != InfeasibleReason::NONE) {
    path_shifter_ = previous_path_shifter;
    return stopForInfeasiblePath(boundary_reason, debug_info);
  }
  debug_info.shift_lines_count = shift_lines.size();

  prev_output_ = shifted_path;
  ego_aligned_return_active_ = false;
  lifecycle_state_ = isCommitmentDetected() ? AvoidanceLifecycleState::COMMITTED
                                            : AvoidanceLifecycleState::CANDIDATE;
  path_generation_failure_started_.reset();
  debug_data_.last_reason = InfeasibleReason::NONE;
  debug_data_.path_shifter = std::make_shared<PathShifter>(path_shifter_);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  const double lon_margin =
    feasibility_result.dist_to_obstacle - feasibility_result.dist_to_shift_end;
  // dist_to_shift_end: longitudinal distance needed before the obstacle front edge
  // dist_to_obstacle: available longitudinal space before obstacle front edge
  // lon_margin: remaining longitudinal clearance after shift completes
  RCLCPP_INFO_THROTTLE(
    getLogger(), *clock_, 2000,
    "[SIMPLE_AVOIDANCE] avoidance path generated shift=%.2f target_lon=%.2f target_lat=%.2f "
    "required_clearance=%.2f required_before_front=%.2f dist_to_avoid_start=%.2f "
    "transition=%.2f jerk_distance=%.2f ego_speed=%.2f dist_to_shift_end=%.2f "
    "dist_to_obstacle=%.2f lon_margin=%.2f",
    shift_result.shift_length, target->longitudinal_distance, target->lateral_offset,
    shift_result.required_clearance, feasibility_result.required_start_distance_before_front,
    feasibility_result.dist_to_avoid_start, feasibility_result.transition_distance,
    feasibility_result.jerk_distance, feasibility_result.ego_speed,
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

  ShiftedPath shifted_path;
  if (active_trailer_configuration_.geometries.empty()) {
    auto path_shifter_local = path_shifter_;
    path_shifter_local.setShiftLines(buildShiftLines(*active_target_, shift_result.shift_length));
    path_shifter_local.generate(&shifted_path);
    setOrientation(&shifted_path.path);
  } else {
    ShiftLineArray selected_lines;
    InfeasibleReason failure_reason{InfeasibleReason::TRAILER_COLLISION};
    const auto trailer_path = generateTrailerAwarePath(
      *active_target_, shift_result.shift_length, selected_lines, failure_reason);
    if (!trailer_path.has_value()) {
      return CandidateOutput(getPreviousModuleOutput().path);
    }
    shifted_path = *trailer_path;
  }
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

  return extendBackwardPath(reference_path_, original_path, getEgoPose().position, backward_length);
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
