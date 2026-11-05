// Copyright 2026 BYD
// SPDX-License-Identifier: Apache-2.0
#include "opposite_lane.hpp"

#include "borrowing_math.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <tf2/utils.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <functional>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace autoware::behavior_path_planner
{
namespace
{
using Polygon = autoware_utils::Polygon2d;
using MultiPolygon = boost::geometry::model::multi_polygon<Polygon>;
using autoware_utils::calc_distance2d;
using autoware_utils::calc_offset_pose;

Polygon rectangle(const Pose & pose, double front, double rear, double half_width)
{
  Polygon polygon;
  for (const auto & corner : std::vector<std::pair<double, double>>{
         {front, half_width}, {front, -half_width}, {-rear, -half_width}, {-rear, half_width}}) {
    const auto p = calc_offset_pose(pose, corner.first, corner.second, 0.0).position;
    polygon.outer().emplace_back(p.x, p.y);
  }
  boost::geometry::correct(polygon);
  return polygon;
}

MultiPolygon laneArea(const lanelet::ConstLanelets & lanes)
{
  MultiPolygon area;
  for (const auto & lane : lanes) {
    Polygon polygon;
    for (const auto & point : lane.polygon2d().basicPolygon()) {
      polygon.outer().emplace_back(point.x(), point.y());
    }
    boost::geometry::correct(polygon);
    if (area.empty()) {
      area.push_back(polygon);
    } else {
      MultiPolygon merged;
      boost::geometry::union_(area, polygon, merged);
      area = std::move(merged);
    }
  }
  return area;
}

bool inside(const Polygon & footprint, const MultiPolygon & area)
{
  // Polygon difference catches edge crossings and internal holes, unlike corner-only checks.
  MultiPolygon outside;
  boost::geometry::difference(footprint, area, outside);
  return !area.empty() && std::abs(boost::geometry::area(outside)) < 1e-6;
}

bool allowedBoundary(const lanelet::ConstLineString3d & boundary)
{
  const auto type = boundary.attributeOr("type", std::string{});
  // Unknown boundary encodings are not evidence that a physical separator is traversable.
  return type == "line_thin" || type == "line_thick" || type == "virtual";
}

bool road(const lanelet::ConstLanelet & lane)
{
  return lane.attributeOr("subtype", std::string{}) == "road" &&
         lane.attributeOr("turn_direction", std::string{"none"}) == "none" &&
         lane.attributeOr("intersection_area", std::string{"0"}) == "0";
}

bool connects(const lanelet::ConstLanelet & previous, const lanelet::ConstLanelet & next)
{
  const auto distance = [](const auto & a, const auto & b) {
    return std::hypot(a.x() - b.x(), a.y() - b.y());
  };
  return previous.id() == next.id() ||
         (distance(previous.leftBound().back(), next.leftBound().front()) < 0.1 &&
          distance(previous.rightBound().back(), next.rightBound().front()) < 0.1);
}

bool finitePose(const Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(norm) && norm > 0.5 && norm < 1.5;
}

// Include motion in every prediction, even if the measured initial velocity is zero.
std::optional<double> objectMotionBound(const PredictedObject & object)
{
  if (!finitePose(object.kinematics.initial_pose_with_covariance.pose)) return std::nullopt;
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) ||
      !std::isfinite(twist.angular.z) || !std::isfinite(object.shape.dimensions.x) ||
      !std::isfinite(object.shape.dimensions.y)) return std::nullopt;
  double radius = std::hypot(object.shape.dimensions.x, object.shape.dimensions.y) * 0.5;
  for (const auto & point : object.shape.footprint.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return std::nullopt;
    radius = std::max(radius, std::hypot(static_cast<double>(point.x), static_cast<double>(point.y)));
  }
  double speed = std::hypot(twist.linear.x, twist.linear.y) + std::abs(twist.angular.z) * radius;
  for (const auto & prediction : object.kinematics.predicted_paths) {
    const double step = rclcpp::Duration(prediction.time_step).seconds();
    for (size_t i = 0; i < prediction.path.size(); ++i) {
      if (!finitePose(prediction.path[i])) return std::nullopt;
      if (i && step > 0.0) {
        const double rotation = std::abs(autoware_utils::normalize_radian(
          tf2::getYaw(prediction.path[i].orientation) - tf2::getYaw(prediction.path[i - 1].orientation)));
        speed = std::max(speed,
          (calc_distance2d(prediction.path[i - 1], prediction.path[i]) + rotation * radius) / step);
      }
    }
  }
  return std::isfinite(speed) ? std::optional<double>{speed} : std::nullopt;
}

Pose extrapolate(const PredictedObject & object, const Pose & start, double time)
{
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  // Object twist is expressed in its local frame, including lateral velocity.
  const double angle = twist.angular.z * time;
  double x = twist.linear.x * time;
  double y = twist.linear.y * time;
  if (std::abs(twist.angular.z) > 1e-6) {
    x = (twist.linear.x * std::sin(angle) + twist.linear.y * (std::cos(angle) - 1.0)) /
        twist.angular.z;
    y = (twist.linear.x * (1.0 - std::cos(angle)) + twist.linear.y * std::sin(angle)) /
        twist.angular.z;
  }
  auto pose = calc_offset_pose(start, x, y, 0.0);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, tf2::getYaw(start.orientation) + angle);
  pose.orientation = tf2::toMsg(rotation);
  return pose;
}

Pose predictedPose(
  const PredictedObject & object, const autoware_perception_msgs::msg::PredictedPath & path,
  double time)
{
  const double step = rclcpp::Duration(path.time_step).seconds();
  if (path.path.empty() || step <= 0.0) {
    return extrapolate(object, object.kinematics.initial_pose_with_covariance.pose, time);
  }
  const double index = std::max(0.0, time) / step;
  const size_t first = static_cast<size_t>(index);
  if (first + 1 >= path.path.size()) {
    return extrapolate(object, path.path.back(), std::max(0.0, time - step * (path.path.size() - 1)));
  }
  return autoware_utils::calc_interpolated_pose(path.path[first], path.path[first + 1], index - first);
}

Polygon objectEnvelope(const PredictedObject & object, const Pose & pose, double lon, double lat)
{
  // Bound every shape (including arbitrary footprint polygons) in object coordinates.
  double front = std::max(0.0, static_cast<double>(object.shape.dimensions.x) * 0.5);
  double rear = front;
  double width = std::max(0.0, static_cast<double>(object.shape.dimensions.y) * 0.5);
  for (const auto & point : object.shape.footprint.points) {
    front = std::max(front, static_cast<double>(point.x));
    rear = std::max(rear, -static_cast<double>(point.x));
    width = std::max(width, std::abs(static_cast<double>(point.y)));
  }
  if (object.shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    width = std::max(width, front);
  }
  return rectangle(pose, front + lon, rear + lon, width + lat);
}

std::vector<double> arcLengths(const PathWithLaneId & path)
{
  std::vector<double> arc(path.points.size(), 0.0);
  for (size_t i = 1; i < arc.size(); ++i) {
    arc[i] = arc[i - 1] + calc_distance2d(path.points[i - 1].point.pose, path.points[i].point.pose);
  }
  return arc;
}

const char * phaseName(BorrowingPhase phase)
{
  switch (phase) {
    case BorrowingPhase::WAITING_APPROVAL: return "waiting_approval";
    case BorrowingPhase::BORROWING: return "borrowing";
    case BorrowingPhase::PASSING: return "passing";
    case BorrowingPhase::RETURNING: return "returning";
    case BorrowingPhase::STOPPING: return "stopping";
    case BorrowingPhase::COMPLETE: return "complete";
  }
  return "unknown";
}
}  // namespace

OppositeLaneBorrowing::OppositeLaneBorrowing(
  rclcpp::Node & node, std::shared_ptr<AvoidanceByLCParameters> parameters,
  std::shared_ptr<LaneChangeParameters> lane_change_parameters)
: parameters_(std::move(parameters)), lc_parameters_(std::move(lane_change_parameters)),
  clock_(node.get_clock()), logger_(node.get_logger().get_child("opposite_lane"))
{
  // A received empty array explicitly means no trailers. Silence is not interpreted as no trailer.
  trailer_sub_ = node.create_subscription<byd_vehicle_msgs::msg::TrailerConfiguration>(
    parameters_->opposite_lane.trailer_configuration_topic,
    rclcpp::QoS(1).reliable().transient_local(),
    [this](const byd_vehicle_msgs::msg::TrailerConfiguration & message) { configure(message); });
}

void OppositeLaneBorrowing::configure(const byd_vehicle_msgs::msg::TrailerConfiguration & message)
{
  const std::lock_guard<std::mutex> lock(configuration_mutex_);
  ResolvedTrailerConfiguration next;
  next.types = message.trailer_types;
  bool valid = true;
  for (const auto & type : next.types) {
    const auto geometry = parameters_->opposite_lane.trailer_types.find(type);
    if (geometry == parameters_->opposite_lane.trailer_types.end() ||
        !isValidTrailerGeometry(geometry->second)) {
      valid = false;
      break;
    }
    next.geometries.push_back(geometry->second);
  }
  if (!configuration_received_ || valid != configuration_valid_ || next.types != configuration_.types) {
    ++configuration_revision_;
  }
  configuration_received_ = true;
  configuration_valid_ = valid;
  configuration_ = std::move(next);
}

void OppositeLaneBorrowing::reset()
{
  requested_ = safe_ = committed_ = recovery_ = context_changed_ = false;
  path_ = {};
  reference_ = {};
  reference_velocity_limits_.clear();
  lines_.clear();
  source_.clear();
  opposite_.clear();
  drivable_lanes_.clear();
  actual_history_.clear();
  target_.reset();
  route_id_.reset();
  clear_since_.reset();
  conflict_seen_ = false;
  completion_count_ = 0;
  phase_ = BorrowingPhase::WAITING_APPROVAL;
  reason_.clear();
  conflict_object_.clear();
  stop_pose_.reset();
  last_diagnostic_phase_.reset();
}

double OppositeLaneBorrowing::trainLength() const
{
  double length = std::max(
    data_->parameters.vehicle_info.rear_overhang_m,
    parameters_->opposite_lane.tractor_rear_axle_to_hitch);
  for (const auto & geometry : active_configuration_.geometries) {
    length += geometry.front_hitch_to_axle +
              std::max(geometry.axle_to_body_rear, geometry.axle_to_rear_hitch);
  }
  return length;
}

void OppositeLaneBorrowing::update(
  const std::shared_ptr<const PlannerData> & data, const BehaviorModuleOutput & upstream,
  const std::optional<ObjectData> & target, bool prefer_same_direction)
{
  const auto started = std::chrono::steady_clock::now();
  struct OnExit {
    std::function<void()> report;
    ~OnExit() { report(); }
  } report{[this, started]() { logDiagnostics(started); }};
  data_ = data;
  upstream_ = upstream;
  safe_ = false;
  stop_pose_.reset();
  const auto & p = parameters_->opposite_lane;
  deadline_ = started + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double, std::milli>(p.max_planning_time_ms));
  if (data_ && data_->self_odometry && !finitePose(data_->self_odometry->pose.pose)) {
    reason_ = "invalid_odometry";
    return;
  }
  if (data_ && data_->self_odometry && data_->self_odometry->twist.twist.linear.x < -0.05) {
    observed_history_.clear();
    reason_ = "reverse_motion_unsupported";
    return;
  }
  if (data_ && data_->self_odometry) {
    const auto pose = data_->self_odometry->pose.pose;
    if (!observed_history_.empty() && calc_distance2d(observed_history_.back(), pose) > 2.0) {
      observed_history_.clear();
    }
    if (observed_history_.empty() || calc_distance2d(observed_history_.back(), pose) >= 0.05) {
      observed_history_.push_back(pose);
      if (observed_history_.size() > 4000) observed_history_.erase(observed_history_.begin());
    }
  }
  if (!committed_ && (!p.enabled || prefer_same_direction || !target)) {
    reset();
    return;
  }
  if (!data_ || !data_->self_odometry || !data_->route_handler || upstream.path.points.size() < 2) {
    reason_ = "missing_planner_data";
    return;
  }
  const auto now = clock_->now();
  const auto fresh = [&](const auto & stamp) {
    const double age = (now - rclcpp::Time(stamp, now.get_clock_type())).seconds();
    return age >= -0.1 && age <= p.data_timeout;
  };
  const bool fresh_data = data_->dynamic_object && fresh(data_->dynamic_object->header.stamp) &&
                          fresh(data_->self_odometry->header.stamp);
  bool valid_configuration;
  size_t revision;
  ResolvedTrailerConfiguration configuration;
  {
    const std::lock_guard<std::mutex> lock(configuration_mutex_);
    valid_configuration = configuration_received_ && configuration_valid_;
    revision = configuration_revision_;
    configuration = configuration_;
  }
  const auto route = data_->prev_route_id
    ? autoware_utils_uuid::to_hex_string(*data_->prev_route_id) : std::string{};
  if (committed_) {
    context_changed_ = context_changed_ || !p.enabled || revision != active_revision_ ||
                       !route_id_ || *route_id_ != route || !valid_configuration;
    if (revision != active_revision_ && valid_configuration) {
      active_revision_ = revision;
      active_configuration_ = std::move(configuration);
    }
    if (!valid_configuration) {
      reason_ = "invalid_trailer_configuration_during_execution";
    } else if (!fresh_data) {
      reason_ = "stale_perception_or_odometry";
    } else if (!updateActualHistory()) {
      reason_ = "execution_discontinuity";
    } else {
      // Keep the committed geometry, but respect new upstream speed limits and stop requests.
      for (auto & point : path_.path.points) {
        const auto index = autoware::motion_utils::findNearestIndex(reference_.points,
          point.point.pose.position);
        const auto upstream_index = autoware::motion_utils::findNearestIndex(upstream.path.points,
          reference_.points[index].point.pose.position);
        if (index < reference_velocity_limits_.size()) {
          point.point.longitudinal_velocity_mps = std::min<double>(
            reference_velocity_limits_[index],
            upstream.path.points[upstream_index].point.longitudinal_velocity_mps);
        }
      }
      safe_ = validate(path_, recovery_);
      if (!safe_ && tryReturn()) safe_ = true;
      if (context_changed_ && !recovery_) {
        safe_ = tryReturn();
        if (!safe_) reason_ = "context_changed_no_safe_return";
      }
    }
  } else {
    requested_ = true;  // Unsafe or unavailable candidates still request a stop before the target.
    if (!target_ || target_->object.object_id != target->object.object_id) {
      RCLCPP_INFO(logger_,
        "[OPPOSITE_BORROWING] active target locked uuid=%s lon=%.2fm lat=%.2fm",
        autoware_utils_uuid::to_hex_string(target->object.object_id).c_str(),
        target->longitudinal, target->to_centerline);
    }
    target_ = target;
    left_ = target->direction == Direction::RIGHT;
    phase_ = BorrowingPhase::WAITING_APPROVAL;
    if (!fresh_data || !valid_configuration || route.empty()) {
      path_ = {};
      reason_ = !fresh_data ? "stale_perception_or_odometry" :
                !valid_configuration ? "trailer_configuration_unavailable" : "route_unavailable";
      return;
    }
    active_revision_ = revision;
    active_configuration_ = std::move(configuration);
    route_id_ = route;
    reference_ = utils::resamplePathWithSpline(upstream.path, 0.25);
    source_ = utils::getCurrentLanesFromPath(reference_, data_);
    opposite_.clear();
    drivable_lanes_.clear();
    lines_.clear();
    path_ = {};
    actual_history_ = observed_history_;
    if (!active_configuration_.geometries.empty()) {
      double history_length = 0.0;
      for (size_t i = 1; i < actual_history_.size(); ++i) {
        history_length += calc_distance2d(actual_history_[i - 1], actual_history_[i]);
      }
      if (history_length < 2.0 * trainLength()) {
        reason_ = "insufficient_measured_history_for_trailers";
        return;
      }
    }
    if (source_.empty() || reference_.points.size() < 2) {
      reason_ = "source_lane_unavailable";
      return;
    }
    if (std::abs(data_->self_odometry->twist.twist.linear.x) > p.max_speed + 0.05) {
      reason_ = "reduce_speed_before_borrowing";
      return;
    }
    safe_ = buildCorridor() && buildPath() && validate(path_);
    reference_velocity_limits_.clear();
    for (const auto & point : reference_.points) {
      reference_velocity_limits_.push_back(std::min<double>(
        point.point.longitudinal_velocity_mps, p.max_speed));
    }
  }
  if (!safe_) {
    conflict_seen_ = true;
    clear_since_.reset();
  } else if (conflict_seen_) {
    if (!clear_since_) clear_since_ = now;
    if ((now - *clear_since_).seconds() < p.clearance_hold_time) {
      safe_ = false;
      reason_ = "clearance_hold";
    } else {
      conflict_seen_ = false;
    }
  }
  if (committed_) updatePhase();

}

void OppositeLaneBorrowing::logDiagnostics(std::chrono::steady_clock::time_point started)
{
  if (!requested_ && !committed_) return;
  double clearance = std::numeric_limits<double>::quiet_NaN();
  int outside = -1;
  int returned = -1;
  if (committed_ && data_ && data_->dynamic_object && !actual_history_.empty() && !source_.empty()) {
    const auto train = predictArticulatedPath(actual_history_, active_configuration_.geometries,
      parameters_->opposite_lane.tractor_rear_axle_to_hitch);
    if (!train.poses.empty()) {
      outside = 0;
      returned = 1;
      for (const auto & body : footprints(train.poses.back())) {
        outside |= !contained(body);
        returned &= contained(body, true);
        for (const auto & object : data_->dynamic_object->objects) {
          if (!objectMotionBound(object)) continue;
          const auto envelope = objectEnvelope(object, object.kinematics.initial_pose_with_covariance.pose, 0.0, 0.0);
          const double distance = boost::geometry::distance(body, envelope);
          clearance = std::isnan(clearance) ? distance : std::min(clearance, distance);
        }
      }
    }
  }
  const double elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  std::ostringstream message;
  message << "[OPPOSITE_BORROWING] phase=" << phaseName(phase_) << " safe=" << safe_
          << " reason=" << (reason_.empty() ? "none" : reason_)
          << " conflict=" << (conflict_object_.empty() ? "none" : conflict_object_)
          << " source=" << (source_.empty() ? 0 : source_.front().id())
          << " opposite=" << (opposite_.empty() ? 0 : opposite_.front().id())
          << " return_x=" << (lines_.empty() ? 0.0 : lines_.back().end.position.x)
          << " return_y=" << (lines_.empty() ? 0.0 : lines_.back().end.position.y)
          << " actual_min_clearance_m=" << clearance
          << " actual_outside_corridor=" << outside << " actual_returned=" << returned
          << " planning_ms=" << elapsed;
  if (!last_diagnostic_phase_ || *last_diagnostic_phase_ != phase_) {
    RCLCPP_INFO(logger_, "%s", message.str().c_str());
  } else {
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "%s", message.str().c_str());
  }
  last_diagnostic_phase_ = phase_;
}

bool OppositeLaneBorrowing::buildCorridor()
{
  // Only the lane sections covering the maneuver are used by buildPath/validate. Preserve the
  // upstream order and require the inverted opposite lanes to connect in that same order.
  const auto & handler = *data_->route_handler;
  for (const auto & source : source_) {
    DrivableLanes drivable;
    drivable.left_lane = source;
    drivable.right_lane = source;
    const auto boundary = left_ ? source.leftBound() : source.rightBound();
    const auto candidates = left_ ? handler.getLeftOppositeLanelets(source)
                                  : handler.getRightOppositeLanelets(source);
    std::optional<lanelet::ConstLanelet> selected;
    if (road(source) && allowedBoundary(boundary)) {
      for (const auto & lane : candidates) {
        if (!road(lane)) continue;
        const auto aligned = lane.invert();
        if (!opposite_.empty() && !connects(opposite_.back().invert(), aligned)) continue;
        if (selected && selected->id() != lane.id()) {
          reason_ = "ambiguous_opposite_topology";
          return false;
        }
        selected = lane;
      }
    }
    if (selected) {
      if (opposite_.empty() || opposite_.back().id() != selected->id()) {
        opposite_.push_back(*selected);
      }
      if (left_) drivable.left_lane = selected->invert();
      else drivable.right_lane = selected->invert();
    }
    drivable_lanes_.push_back(drivable);
  }
  if (opposite_.empty()) {
    reason_ = "no_continuous_opposite_road";
    return false;
  }
  return true;
}

bool OppositeLaneBorrowing::buildPath()
{
  const auto & p = parameters_->opposite_lane;
  const auto & ego = data_->self_odometry->pose.pose;
  const auto & vehicle = data_->parameters.vehicle_info;
  if (vehicle.wheel_base_m <= 0.0 || vehicle.max_steer_angle_rad <= 0.0 ||
      vehicle.vehicle_width_m <= 0.0 || vehicle.max_longitudinal_offset_m <= 0.0) {
    reason_ = "vehicle_geometry_unavailable";
    return false;
  }
  const auto arc = arcLengths(reference_);
  const size_t ego_index = data_->findEgoIndex(reference_.points);
  const auto indexAt = [&](double distance) {
    return static_cast<size_t>(std::lower_bound(arc.begin(), arc.end(), distance) - arc.begin());
  };
  lanelet::ConstLanelets geometry;
  for (const auto & lane : opposite_) geometry.push_back(lane.invert());
  // Arc distance is signed in the aligned lane's frame; negate to shift from the source to it.
  const double shift = -lanelet::utils::getArcCoordinates(geometry, ego).distance;
  if (!std::isfinite(shift) || std::abs(shift) < 0.1 || (shift > 0.0) != left_) {
    reason_ = "invalid_opposite_offset";
    return false;
  }
  const auto & trajectory = lc_parameters_->trajectory;
  const double jerk = trajectory.lateral_jerk;
  const double acceleration = trajectory.lat_acc_map.base_vel.empty()
    ? 0.2 : trajectory.lat_acc_map.find(p.max_speed).second;
  if (jerk <= 0.0 || acceleration <= 0.0) {
    reason_ = "invalid_lateral_limits";
    return false;
  }
  // Four jerk phases: d = j*T^3/32. Also enforce a conservative lateral acceleration bound.
  const double duration = std::max(
    std::cbrt(32.0 * std::abs(shift) / jerk),
    std::sqrt(8.0 * std::abs(shift) / acceleration));
  const double shift_distance = std::max(p.min_shift_distance, p.max_speed * duration);
  const double obstacle_arc = autoware::motion_utils::calcSignedArcLength(
    reference_.points, reference_.points.front().point.pose.position,
    target_->object.kinematics.initial_pose_with_covariance.pose.position);
  const double object_front = obstacle_arc - target_->length * 0.5;
  const double object_rear = obstacle_arc + target_->length * 0.5;
  const double borrow_start = arc[ego_index] + p.prepare_distance;
  const double borrow_end = borrow_start + shift_distance;
  const double return_start = object_rear + trainLength() + p.return_distance_after_object;
  const double return_end = return_start + shift_distance;
  // Allow a straight settling section for the last trailer before declaring completion.
  const double finish = return_end + trainLength() + 1.0;
  if (borrow_end + data_->parameters.vehicle_info.max_longitudinal_offset_m +
        p.longitudinal_margin >= object_front || finish >= arc.back()) {
    reason_ = "insufficient_borrow_or_return_distance";
    return false;
  }
  const auto graphs = data_->route_handler->getOverallGraphPtr();
  if (!graphs) {
    reason_ = "routing_graph_unavailable";
    return false;
  }
  for (const auto & lanes : {source_, geometry}) {
    const double intersection = utils::getDistanceToNextIntersection(ego, lanes);
    const double crosswalk = utils::getDistanceToCrosswalk(
      ego, lanes, *graphs);
    if (std::min(intersection, crosswalk) <= finish - arc[ego_index] +
          data_->parameters.vehicle_info.max_longitudinal_offset_m) {
      reason_ = "regulatory_element_in_borrowing_interval";
      return false;
    }
  }
  const size_t finish_index = indexAt(finish);
  reference_.points.resize(finish_index + 1);
  const auto makeLine = [&](double start, double end, double start_shift, double end_shift) {
    ShiftLine line;
    line.start_idx = indexAt(start);
    line.end_idx = indexAt(end);
    line.start = reference_.points.at(line.start_idx).point.pose;
    line.end = reference_.points.at(line.end_idx).point.pose;
    line.start_shift_length = start_shift;
    line.end_shift_length = end_shift;
    return line;
  };
  lines_ = {makeLine(borrow_start, borrow_end, 0.0, shift),
            makeLine(return_start, return_end, shift, 0.0)};
  PathShifter shifter;
  shifter.setPath(reference_);
  shifter.setVelocity(p.max_speed);
  shifter.setLateralAccelerationLimit(acceleration);
  shifter.setLongitudinalAcceleration(0.0);
  shifter.setShiftLines(lines_);
  if (!shifter.generate(&path_)) {
    reason_ = "path_generation_failed";
    return false;
  }
  for (auto & point : path_.path.points) {
    point.point.longitudinal_velocity_mps = std::min<double>(
      point.point.longitudinal_velocity_mps, p.max_speed);
  }
  return true;
}

std::vector<Polygon> OppositeLaneBorrowing::footprints(const ArticulatedPose & pose) const
{
  const auto & vehicle = data_->parameters.vehicle_info;
  const double margin = parameters_->opposite_lane.road_boundary_margin;
  std::vector<Polygon> result{rectangle(
    pose.tractor, vehicle.max_longitudinal_offset_m + margin,
    vehicle.rear_overhang_m + margin, vehicle.vehicle_width_m * 0.5 + margin)};
  for (size_t i = 0; i < pose.trailers.size(); ++i) {
    auto geometry = active_configuration_.geometries.at(i);
    geometry.width += 2.0 * margin;
    geometry.axle_to_body_front += margin;
    geometry.axle_to_body_rear += margin;
    result.push_back(createTrailerFootprint(pose.trailers[i], geometry));
  }
  return result;
}

bool OppositeLaneBorrowing::contained(const Polygon & footprint, bool source_only) const
{
  auto lanes = source_;
  if (!source_only) lanes.insert(lanes.end(), opposite_.begin(), opposite_.end());
  return inside(footprint, laneArea(lanes));
}

ArticulatedPath OppositeLaneBorrowing::articulated(const PathWithLaneId & path, size_t start) const
{
  std::vector<Pose> poses = actual_history_;
  if (poses.empty()) {
    // Initial path includes the original lane behind the ego, to seed the trailer chain.
    for (size_t i = 0; i < start; ++i) poses.push_back(path.points[i].point.pose);
  }
  const size_t prefix = poses.size();
  for (size_t i = start; i < path.points.size(); ++i) poses.push_back(path.points[i].point.pose);
  auto result = predictArticulatedPath(
    poses, active_configuration_.geometries,
    parameters_->opposite_lane.tractor_rear_axle_to_hitch);
  if (prefix <= result.poses.size()) {
    result.poses.erase(result.poses.begin(), result.poses.begin() + prefix);
  }
  return result;
}

bool OppositeLaneBorrowing::updateActualHistory()
{
  const auto pose = data_->self_odometry->pose.pose;
  if (!actual_history_.empty()) {
    const double distance = calc_distance2d(actual_history_.back(), pose);
    if (distance > 2.0) return false;
    if (distance < 0.05) return true;
  }
  actual_history_.push_back(pose);
  return true;
}

bool OppositeLaneBorrowing::validate(const ShiftedPath & path, bool recovery)
{
  reason_.clear();
  conflict_object_.clear();
  if (path.path.points.size() < 2 || path.shift_length.size() != path.path.points.size()) {
    reason_ = "invalid_path";
    return false;
  }
  for (size_t i = 0; i < path.path.points.size(); ++i) {
    if (!finitePose(path.path.points[i].point.pose) ||
        !std::isfinite(path.path.points[i].point.longitudinal_velocity_mps) ||
        !std::isfinite(path.shift_length[i])) {
      reason_ = "nonfinite_path";
      return false;
    }
  }
  std::vector<double> object_speeds;
  for (const auto & object : data_->dynamic_object->objects) {
    const auto speed_bound = objectMotionBound(object);
    if (!speed_bound) {
      reason_ = "invalid_object_prediction";
      return false;
    }
    object_speeds.push_back(*speed_bound);
  }
  const double perception_age = std::max(0.0, (clock_->now() - rclcpp::Time(
    data_->dynamic_object->header.stamp, clock_->get_clock_type())).seconds());
  const size_t start = data_->findEgoIndex(path.path.points);
  const auto & ego = data_->self_odometry->pose.pose;
  if (calc_distance2d(path.path.points[start].point.pose, ego) > 0.5 ||
      std::abs(autoware_utils::normalize_radian(tf2::getYaw(ego.orientation) -
        tf2::getYaw(path.path.points[start].point.pose.orientation))) > 0.2) {
    reason_ = "execution_discontinuity";
    return false;
  }
  const auto & p = parameters_->opposite_lane;
  const auto arcs = arcLengths(path.path);
  std::vector<double> distances;
  std::vector<double> limits;
  for (size_t i = start; i < arcs.size(); ++i) {
    distances.push_back(arcs[i] - arcs[start]);
    limits.push_back(std::min<double>(p.max_speed, path.path.points[i].point.longitudinal_velocity_mps));
  }
  const auto & trajectory = lc_parameters_->trajectory;
  const double speed = std::abs(data_->self_odometry->twist.twist.linear.x);
  if (speed > p.max_speed + 0.05) {
    reason_ = "borrowing_speed_exceeded";
    return false;
  }
  const auto & vehicle = data_->parameters.vehicle_info;
  if (vehicle.wheel_base_m > 0.0 && vehicle.max_steer_angle_rad > 0.0) {
    const double lateral_acceleration = trajectory.lat_acc_map.base_vel.empty()
      ? 0.2 : trajectory.lat_acc_map.find(p.max_speed).second;
    const double curvature_limit = std::min(
      std::tan(vehicle.max_steer_angle_rad) / vehicle.wheel_base_m,
      lateral_acceleration / (p.max_speed * p.max_speed));
    std::optional<double> previous_curvature;
    for (size_t i = std::max<size_t>(1, start); i + 1 < path.path.points.size(); ++i) {
      const auto & a = path.path.points[i - 1].point.pose.position;
      const auto & b = path.path.points[i].point.pose.position;
      const auto & c = path.path.points[i + 1].point.pose.position;
      const double ab = std::hypot(b.x - a.x, b.y - a.y);
      const double bc = std::hypot(c.x - b.x, c.y - b.y);
      const double ac = std::hypot(c.x - a.x, c.y - a.y);
      if (ab * bc * ac < 1e-8) {
        reason_ = "degenerate_path_geometry";
        return false;
      }
      const double curvature = 2.0 * ((b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x)) / (ab * bc * ac);
      if (!std::isfinite(curvature) || std::abs(curvature) > curvature_limit * 1.05 ||
          (previous_curvature && std::abs(curvature - *previous_curvature) / ab *
            std::pow(p.max_speed, 3) > trajectory.lateral_jerk * 1.05)) {
        reason_ = "kinematic_limit";
        return false;
      }
      previous_curvature = curvature;
    }
  }
  const double acceleration = trajectory.max_longitudinal_acc;
  const double deceleration = -trajectory.min_longitudinal_acc;
  const auto earliest = borrowing::arrivalTimes(distances, limits, speed, acceleration, deceleration, 1.0);
  const auto latest = borrowing::arrivalTimes(
    distances, limits, speed, acceleration * 0.5, deceleration, 1.0);
  if (!borrowing::coversReturn(latest, p.min_time_gap, p.max_prediction_horizon) || earliest.empty()) {
    reason_ = "prediction_does_not_cover_complete_return";
    return false;
  }
  const auto train = articulated(path.path, start);
  if (!train.articulation_valid || train.poses.size() != distances.size()) {
    reason_ = "articulation_limit";
    return false;
  }
  auto lanes = source_;
  lanes.insert(lanes.end(), opposite_.begin(), opposite_.end());
  const auto corridor = laneArea(lanes);
  const auto source_area = laneArea(source_);
  // Query crossing relations using raw lane directions: inverted lanes need not be nodes in
  // the routing graph. Check these polygons against every body in the generated trajectory.
  lanelet::ConstLanelets crosswalks;
  if (data_->route_handler && data_->route_handler->getOverallGraphPtr()) {
    const auto graphs = data_->route_handler->getOverallGraphPtr();
    std::unordered_set<lanelet::Id> added;
    for (const auto & lane : lanes) {
      for (const auto & crosswalk : graphs->conflictingInGraph(lane, 1)) {
        if (added.insert(crosswalk.id()).second) crosswalks.push_back(crosswalk);
      }
    }
  }
  const auto crosswalk_area = laneArea(crosswalks);
  std::vector<Polygon> previous;
  for (size_t i = 0; i < train.poses.size(); ++i) {
    if (deadline_ && std::chrono::steady_clock::now() > *deadline_) {
      reason_ = "planning_budget_exceeded";
      return false;
    }
    const auto current = footprints(train.poses[i]);
    if (std::abs(path.shift_length[start + i]) > 0.05) {
      const auto ref_idx = autoware::motion_utils::findNearestIndex(
        reference_.points, train.poses[i].tractor.position);
      const auto & position = reference_.points[ref_idx].point.pose.position;
      bool authorized_section = false;
      for (const auto & lanes : drivable_lanes_) {
        const auto & source = left_ ? lanes.right_lane : lanes.left_lane;
        if (lanes.left_lane.id() != lanes.right_lane.id() &&
            lanelet::geometry::inside(source, lanelet::BasicPoint2d{position.x, position.y})) {
          authorized_section = true;
        }
      }
      if (!authorized_section) {
        reason_ = "discontinuous_or_prohibited_borrowing_section";
        return false;
      }
    }
    for (size_t body = 0; body < current.size(); ++body) {
      Polygon swept = current[body];
      if (!previous.empty()) {
        boost::geometry::model::multi_point<autoware_utils::Point2d> corners;
        for (const auto & point : previous[body].outer()) corners.push_back(point);
        for (const auto & point : current[body].outer()) corners.push_back(point);
        boost::geometry::convex_hull(corners, swept);
      }
      if (!crosswalk_area.empty() && !boost::geometry::disjoint(swept, crosswalk_area)) {
        reason_ = "crosswalk_in_borrowing_interval";
        return false;
      }
      if (!inside(swept, corridor)) {
        reason_ = "footprint_out_of_corridor";
        return false;
      }
      if (i + 1 == train.poses.size() && !inside(current[body], source_area)) {
        reason_ = "last_trailer_not_returned";
        return false;
      }
      if (target_ && std::none_of(
          data_->dynamic_object->objects.begin(), data_->dynamic_object->objects.end(),
          [&](const auto & object) { return object.object_id == target_->object.object_id; }) &&
          !boost::geometry::disjoint(swept, objectEnvelope(
            target_->object, target_->getPose(), p.longitudinal_margin, p.lateral_margin))) {
        reason_ = "held_target_collision";
        return false;
      }
      // Check all objects and all predictions. No current-lane, class, heading or speed filter.
      for (size_t object_index = 0; object_index < data_->dynamic_object->objects.size(); ++object_index) {
        const auto & object = data_->dynamic_object->objects[object_index];
        const double object_speed = object_speeds[object_index];
        const double first_time = std::max(0.0, earliest[i ? i - 1 : 0] - p.min_time_gap);
        const double last_time = latest[i] + p.min_time_gap;
        const double step = p.time_resolution;
        for (double time = first_time; ; time = std::min(time + step, last_time)) {
          // Inflate by travel between samples and uncertainty during prediction extrapolation.
          const double padding = object_speed * step + (object_speed > 0.1 ? 0.1 * time : 0.0);
          const auto collides = [&](const Pose & predicted) {
            if (!finitePose(predicted)) return true;
            return !boost::geometry::disjoint(swept, objectEnvelope(
              object, predicted, p.longitudinal_margin + padding, p.lateral_margin + padding));
          };
          bool collision = collides(extrapolate(
            object, object.kinematics.initial_pose_with_covariance.pose, time + perception_age));
          for (const auto & prediction : object.kinematics.predicted_paths) {
            collision = collision || collides(predictedPose(object, prediction, time + perception_age));
          }
          if (collision) {
            reason_ = body ? "trailer_collision" : "vehicle_collision";
            conflict_object_ = autoware_utils_uuid::to_hex_string(object.object_id);
            return false;
          }
          if (object_speed < 1e-6 || time >= last_time) break;
        }
      }
    }
    previous = current;
  }
  if (deadline_ && std::chrono::steady_clock::now() > *deadline_) {
    reason_ = "planning_budget_exceeded";
    return false;
  }
  // Recovery is permitted before passing the target if the whole path is collision-free.
  static_cast<void>(recovery);
  return true;
}

bool OppositeLaneBorrowing::tryReturn()
{
  if (reference_.points.size() < 2 || actual_history_.empty()) return false;
  const auto & p = parameters_->opposite_lane;
  const auto & ego = data_->self_odometry->pose.pose;
  const auto arc = arcLengths(reference_);
  const size_t ego_index = data_->findEgoIndex(reference_.points);
  const double offset = autoware_utils::calc_lateral_deviation(
    reference_.points[ego_index].point.pose, ego.position);
  const auto & trajectory = lc_parameters_->trajectory;
  const double acceleration = trajectory.lat_acc_map.base_vel.empty()
    ? 0.2 : trajectory.lat_acc_map.find(p.max_speed).second;
  const double duration = std::max(
    std::cbrt(32.0 * std::abs(offset) / trajectory.lateral_jerk),
    std::sqrt(8.0 * std::abs(offset) / acceleration));
  const double min_distance = std::max(p.min_shift_distance, p.max_speed * duration);
  const auto saved_reason = reason_;
  for (double extra = 0.0; extra <= 20.0; extra += 1.0) {
    if (deadline_ && std::chrono::steady_clock::now() > *deadline_) break;
    const double end_arc = arc[ego_index] + min_distance + extra;
    const auto end_it = std::lower_bound(arc.begin(), arc.end(), end_arc);
    const auto finish_it = std::lower_bound(
      arc.begin(), arc.end(), end_arc + trainLength() + 1.0);
    if (finish_it == arc.end() || end_it == arc.end()) break;
    ShiftLine line;
    line.start = reference_.points[ego_index].point.pose;
    line.end = reference_.points[std::distance(arc.begin(), end_it)].point.pose;
    line.start_shift_length = offset;
    line.end_shift_length = 0.0;
    PathShifter shifter;
    auto reference = reference_;
    reference.points.resize(std::distance(arc.begin(), finish_it) + 1);
    shifter.setPath(reference);
    shifter.setVelocity(p.max_speed);
    shifter.setLateralAccelerationLimit(acceleration);
    if (ego_index <= 1) continue;
    // PathShifter does not use start_shift_length as its base offset. Seed the measured
    // offset with a line wholly behind the ego before adding the actual returning line.
    ShiftLine seed;
    seed.start = reference.points.front().point.pose;
    seed.end = reference.points[ego_index].point.pose;
    seed.end_shift_length = offset;
    shifter.setShiftLines({seed, line});
    ShiftedPath candidate;
    if (!shifter.generate(&candidate)) continue;
    for (auto & point : candidate.path.points) {
      point.point.longitudinal_velocity_mps = std::min<double>(
        point.point.longitudinal_velocity_mps, p.max_speed);
    }
    // Rebuilding from the measured offset must also preserve the measured tangent.
    const auto idx = data_->findEgoIndex(candidate.path.points);
    if (calc_distance2d(candidate.path.points[idx].point.pose, ego) > 0.25 ||
        std::abs(autoware_utils::normalize_radian(
          tf2::getYaw(candidate.path.points[idx].point.pose.orientation) -
          tf2::getYaw(ego.orientation))) > lc_parameters_->th_finish_judge_yaw_diff) continue;
    if (!validate(candidate, true)) continue;
    path_ = std::move(candidate);
    lines_ = shifter.getShiftLines();
    recovery_ = true;
    reason_ = "safe_recovery_return";
    return true;
  }
  reason_ = saved_reason + ":no_safe_early_return";
  return false;
}

void OppositeLaneBorrowing::updatePhase()
{
  if (!committed_) return;
  if (!safe_) {
    phase_ = BorrowingPhase::STOPPING;
    completion_count_ = 0;
    return;
  }
  const size_t index = data_->findEgoIndex(path_.path.points);
  const auto & ego = data_->self_odometry->pose.pose;
  bool returned = !lines_.empty() && autoware::motion_utils::calcSignedArcLength(
    path_.path.points, lines_.back().end.position, ego.position) >= 0.0;
  const auto current = predictArticulatedPath(
    actual_history_, active_configuration_.geometries,
    parameters_->opposite_lane.tractor_rear_axle_to_hitch);
  if (current.poses.empty() || !current.articulation_valid) returned = false;
  else for (const auto & footprint : footprints(current.poses.back())) {
    returned = returned && contained(footprint, true);
  }
  const double yaw_error = std::abs(autoware_utils::normalize_radian(
    tf2::getYaw(ego.orientation) - tf2::getYaw(path_.path.points[index].point.pose.orientation)));
  if (returned && yaw_error < lc_parameters_->th_finish_judge_yaw_diff) {
    if (++completion_count_ >= 3) phase_ = BorrowingPhase::COMPLETE;
    return;
  }
  completion_count_ = 0;
  if (recovery_ || (lines_.size() > 1 && autoware::motion_utils::calcSignedArcLength(
      path_.path.points, lines_.back().start.position, ego.position) >= 0.0)) {
    phase_ = BorrowingPhase::RETURNING;
  } else if (lines_.size() > 1 && autoware::motion_utils::calcSignedArcLength(
      path_.path.points, lines_.front().end.position, ego.position) >= 0.0) {
    phase_ = BorrowingPhase::PASSING;
  } else {
    phase_ = BorrowingPhase::BORROWING;
  }
}

std::string OppositeLaneBorrowing::approvalKey() const
{
  return (route_id_ ? *route_id_ : "missing_route") + ":" + std::to_string(active_revision_) +
    ":" + (left_ ? "left:" : "right:") +
    (target_ ? autoware_utils_uuid::to_hex_string(target_->object.object_id) : "no_target");
}

CandidateOutput OppositeLaneBorrowing::candidate() const
{
  CandidateOutput result;
  result.path_candidate = path_.path;
  if (!data_ || !data_->self_odometry || !finitePose(data_->self_odometry->pose.pose) ||
      lines_.empty() || path_.path.points.empty()) return result;
  const auto & position = data_->self_odometry->pose.pose.position;
  result.lateral_shift = lines_.front().end_shift_length;
  result.start_distance_to_path_change = autoware::motion_utils::calcSignedArcLength(
    path_.path.points, position, lines_.front().start.position);
  result.finish_distance_to_path_change = autoware::motion_utils::calcSignedArcLength(
    path_.path.points, position, path_.path.points.back().point.pose.position);
  return result;
}

void OppositeLaneBorrowing::stop(PathWithLaneId & path, bool waiting)
{
  if (path.points.empty()) return;
  if (!data_ || !data_->self_odometry || !finitePose(data_->self_odometry->pose.pose) ||
      !std::isfinite(data_->self_odometry->twist.twist.linear.x) ||
      data_->self_odometry->twist.twist.linear.x < -0.05) {
    for (auto & point : path.points) point.point.longitudinal_velocity_mps = 0.0;
    stop_pose_ = path.points.front().point.pose;
    return;
  }
  const auto arc = arcLengths(path);
  const size_t ego_index = data_->findEgoIndex(path.points);
  const auto & p = parameters_->opposite_lane;
  double stop_arc = arc[ego_index];
  if (waiting && target_) {
    stop_arc += std::max(0.0, autoware::motion_utils::calcSignedArcLength(
      path.points, data_->self_odometry->pose.pose.position,
      target_->object.kinematics.initial_pose_with_covariance.pose.position) -
      target_->length * 0.5 - data_->parameters.vehicle_info.max_longitudinal_offset_m -
      p.longitudinal_margin);
    if (!lines_.empty()) {
      stop_arc = std::min(stop_arc, arc[ego_index] + std::max(0.0,
        autoware::motion_utils::calcSignedArcLength(
          path.points, data_->self_odometry->pose.pose.position, lines_.front().start.position) - 1.0));
    }
  } else {
    const double speed = std::abs(data_->self_odometry->twist.twist.linear.x);
    stop_arc += speed * speed / (2.0 * -lc_parameters_->trajectory.min_longitudinal_acc);
  }
  stop_arc = std::min(stop_arc, arc.back());
  const double ego_speed = std::abs(data_->self_odometry->twist.twist.linear.x);
  const double deceleration = -lc_parameters_->trajectory.min_longitudinal_acc;
  for (size_t i = ego_index; i < path.points.size(); ++i) {
    if (waiting) {
      const double approach_speed = std::max(p.max_speed, std::sqrt(std::max(0.0,
        ego_speed * ego_speed - 2.0 * deceleration * (arc[i] - arc[ego_index]))));
      path.points[i].point.longitudinal_velocity_mps = std::min<double>(
        path.points[i].point.longitudinal_velocity_mps, approach_speed);
    }
    path.points[i].point.longitudinal_velocity_mps = std::min<double>(
      path.points[i].point.longitudinal_velocity_mps,
      borrowing::brakingVelocity(stop_arc - arc[i], -lc_parameters_->trajectory.min_longitudinal_acc));
    if (arc[i] >= stop_arc && !stop_pose_) stop_pose_ = path.points[i].point.pose;
  }
  if (!stop_pose_) stop_pose_ = path.points.back().point.pose;
}

BehaviorModuleOutput OppositeLaneBorrowing::output(bool waiting)
{
  BehaviorModuleOutput out = upstream_;
  if (waiting || !committed_) {
    if (!waiting && safe_) {
      committed_ = true;
      phase_ = BorrowingPhase::BORROWING;
      updateActualHistory();
    } else {
      stop(out.path, true);
      return out;
    }
  }
  // Keep the committed geometry on every failure, including route/perception changes.
  out.path = path_.path;
  out.reference_path = reference_;
  DrivableAreaInfo area;
  area.drivable_lanes = utils::cutOverlappedLanes(out.path, drivable_lanes_);
  area.is_already_expanded = true;
  out.drivable_area_info = area;
  for (auto & point : out.path.points) {
    for (const auto & lane : opposite_) {
      if (lanelet::geometry::inside(lane, lanelet::BasicPoint2d{
            point.point.pose.position.x, point.point.pose.position.y})) {
        if (std::find(point.lane_ids.begin(), point.lane_ids.end(), lane.id()) == point.lane_ids.end()) {
          point.lane_ids.push_back(lane.id());
        }
      }
    }
  }
  if (!data_ || !data_->self_odometry) {
    for (auto & point : out.path.points) point.point.longitudinal_velocity_mps = 0.0;
    return out;
  }
  if (!safe_) stop(out.path, false);
  if (!lines_.empty()) {
    const bool returning = phase_ == BorrowingPhase::RETURNING || recovery_;
    const auto & line = returning ? lines_.back() : lines_.front();
    auto [signal, ignored] = data_->getBehaviorTurnSignalInfo(
      path_, line, source_, 0.0, true, false);
    static_cast<void>(ignored);
    signal.turn_signal.command = (returning ? !left_ : left_)
      ? autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT
      : autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT;
    out.turn_signal_info = signal;
  }
  return out;
}
}  // namespace autoware::behavior_path_planner
