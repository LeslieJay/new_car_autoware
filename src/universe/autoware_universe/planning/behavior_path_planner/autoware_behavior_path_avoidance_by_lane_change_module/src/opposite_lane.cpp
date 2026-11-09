// Copyright 2026 BYD
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
#include <boost/geometry/index/rtree.hpp>
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
std::optional<double> objectMotionBound(
  const PredictedObject & object, const bool include_predictions = true)
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
  if (!include_predictions) return std::isfinite(speed) ? std::optional<double>{speed} : std::nullopt;
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

double postReturnDistance(
  const double train_length, const OppositeLaneParameters & parameters,
  const LaneChangeParameters & lane_change)
{
  // A short path end becomes a downstream goal stop before the return is completed.
  // Keep the existing finish buffer available for pose settling and module handover.
  return train_length + 1.0 + (parameters.campus_agv ?
    std::max(0.0, lane_change.lane_change_finish_judge_buffer) : 0.0);
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
  // Use the validated static configuration until a vehicle configuration arrives.
  byd_vehicle_msgs::msg::TrailerConfiguration initial_configuration;
  initial_configuration.trailer_types.assign(
    parameters_->opposite_lane.trailer_ordered_types.begin(),
    parameters_->opposite_lane.trailer_ordered_types.begin() + parameters_->opposite_lane.trailer_count);
  configure(initial_configuration);
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
  reference_ = PathWithLaneId{};
  reference_velocity_limits_.clear();
  lines_.clear();
  source_.clear();
  opposite_.clear();
  drivable_lanes_.clear();
  actual_history_.clear();
  last_actual_stamp_s_.reset();
  target_.reset();
  associated_target_id_.clear();
  route_id_.reset();
  clear_since_.reset();
  conflict_seen_ = false;
  completion_count_ = 0;
  return_extension_conflict_.clear();
  original_return_end_arc_ = 0.0;
  extended_return_end_arc_ = 0.0;
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

bool OppositeLaneBorrowing::isStaticObject(const PredictedObject & object) const
{
  if (!parameters_->opposite_lane.campus_agv) return false;
  using C = autoware_perception_msgs::msg::ObjectClassification;
  for (const auto & classification : object.classification) {
    if (classification.label == C::PEDESTRIAN || classification.label == C::BICYCLE ||
        classification.label == C::MOTORCYCLE) return false;
  }
  const auto id = autoware_utils_uuid::to_hex_string(object.object_id);
  return std::any_of(object_tracks_.begin(), object_tracks_.end(), [&](const auto & track) {
    return track.id == id && track.stationary;
  });
}

void OppositeLaneBorrowing::updateObjectHistory()
{
  const auto & p = parameters_->opposite_lane;
  if (!p.campus_agv || !data_->dynamic_object) return;
  const double now = rclcpp::Time(data_->dynamic_object->header.stamp).seconds();
  if (now < last_object_stamp_) object_tracks_.clear();
  last_object_stamp_ = now;
  std::vector<bool> used(object_tracks_.size(), false);
  for (const auto & object : data_->dynamic_object->objects) {
    const auto & pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & twist = object.kinematics.initial_twist_with_covariance.twist.linear;
    if (!finitePose(pose) || !std::isfinite(twist.x) || !std::isfinite(twist.y)) continue;
    const auto id = autoware_utils_uuid::to_hex_string(object.object_id);
    const double speed = std::hypot(twist.x, twist.y);
    size_t match = object_tracks_.size();
    double best = 1.5;
    for (size_t i = 0; i < object_tracks_.size(); ++i) {
      if (used[i] || now < object_tracks_[i].last_seen ||
          now - object_tracks_[i].last_seen > 1.0) continue;
      const double distance = calc_distance2d(pose, object_tracks_[i].pose);
      if (distance < best || (object_tracks_[i].id == id && distance < 3.0)) {
        match = i;
        best = distance;
        if (object_tracks_[i].id == id) break;
      }
    }
    if (match == object_tracks_.size()) {
      object_tracks_.push_back({pose, pose, id, speed, now, now, false});
      used.push_back(true);
      continue;
    }
    auto & track = object_tracks_[match];
    used[match] = true;
    if (now <= track.last_seen) continue;
    track.filtered_speed = 0.5 * track.filtered_speed + 0.5 * speed;
    if (speed > p.static_speed_threshold || track.filtered_speed > p.static_speed_threshold ||
        calc_distance2d(pose, track.stationary_anchor) >
          p.static_speed_threshold * (now - track.stationary_since) + 0.3) {
      track.stationary_since = now;
      track.stationary_anchor = pose;
    }
    track.stationary = now - track.stationary_since >= p.static_observation_time &&
      speed <= p.static_speed_threshold;
    track.pose = pose;
    track.id = id;
    track.last_seen = now;
  }
  object_tracks_.erase(std::remove_if(object_tracks_.begin(), object_tracks_.end(),
    [&](const auto & track) { return now - track.last_seen > 1.0; }), object_tracks_.end());
}

void OppositeLaneBorrowing::applyCandidateVelocity(ShiftedPath & path) const
{
  const auto & p = parameters_->opposite_lane;
  const auto arc = arcLengths(path.path);
  const auto ego = data_->findEgoIndex(path.path.points);
  const double speed = std::abs(data_->self_odometry->twist.twist.linear.x);
  for (size_t i = 0; i < path.path.points.size(); ++i) {
    double limit = candidate_speed_;
    if (p.campus_agv && data_->external_limit_max_velocity) {
      const double external = data_->external_limit_max_velocity->max_velocity;
      limit = std::isfinite(external) ? std::min(limit, std::max(0.0, external)) : 0.0;
    }
    if (p.campus_agv && limit > 0.0) {
      limit = std::max(limit, std::sqrt(std::max(0.0, speed * speed +
        2.0 * lc_parameters_->trajectory.min_longitudinal_acc * std::max(0.0, arc[i] - arc[ego]))));
    }
    path.path.points[i].point.longitudinal_velocity_mps = std::min<double>(
      path.path.points[i].point.longitudinal_velocity_mps, std::min(p.max_speed, limit));
  }
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
  updateObjectHistory();
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
  if (p.campus_agv && !configuration.geometries.empty()) {
    reason_ = "campus_requires_unhitched_vehicle";
    return;
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
      applyCandidateVelocity(path_);
      safe_ = validate(path_, recovery_);
      if (!safe_ && (reason_ == "vehicle_collision" || reason_ == "trailer_collision") &&
          lines_.size() > 1 && conflict_path_index_ >=
            autoware::motion_utils::findNearestIndex(path_.path.points, lines_.back().start.position) &&
          (!target_ || conflict_object_ !=
            autoware_utils_uuid::to_hex_string(target_->object.object_id)) &&
          extendReturn()) {
        safe_ = true;
      }
      if (!safe_ && !p.campus_agv && tryReturn()) safe_ = true;
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
    const bool associated = p.campus_agv && target_ &&
      calc_distance2d(target_->getPose(), target->getPose()) < 2.0;
    target_ = target;
    if (!associated) {
      associated_target_id_ = autoware_utils_uuid::to_hex_string(target->object.object_id);
      left_ = target->direction == Direction::RIGHT;
      next_speed_index_ = 0;
    }
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
    last_actual_stamp_s_.reset();
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
    if (p.campus_agv) {
      // Prefer the connected borrowed road, retaining the side across target ID changes.
      if (!buildCorridor()) {
        opposite_.clear();
        drivable_lanes_.clear();
        left_ = !left_;
        buildCorridor();
      }
      const auto count = p.candidate_speeds.size();
      for (size_t attempts = 0; attempts < count && !opposite_.empty(); ++attempts) {
        if (deadline_ && std::chrono::steady_clock::now() >= *deadline_) {
          reason_ = "planning_budget_exceeded";
          break;
        }
        double speed_cap = p.max_speed;
        if (data_->external_limit_max_velocity) {
          const double external = data_->external_limit_max_velocity->max_velocity;
          if (!std::isfinite(external) || external <= 0.0) {
            reason_ = "external_stop_request";
            break;
          }
          speed_cap = std::min(speed_cap, external);
        }
        candidate_speed_ = std::min(speed_cap, p.candidate_speeds[next_speed_index_]);
        safe_ = buildPath() && validate(path_);
        if (safe_) { next_speed_index_ = 0; break; }
        next_speed_index_ = (next_speed_index_ + 1) % count;
      }
    } else {
      candidate_speed_ = p.max_speed;
      safe_ = buildCorridor() && buildPath() && validate(path_);
    }
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
          << " return_extension_conflict=" <<
            (return_extension_conflict_.empty() ? "none" : return_extension_conflict_)
          << " profile=" << (parameters_->opposite_lane.campus_agv ? "campus_agv" : "default")
          << " candidate_speed=" << candidate_speed_ << " required_arrival_time=" << required_arrival_time_
          << " horizon=" << parameters_->opposite_lane.max_prediction_horizon
          << " original_return_s=" << original_return_end_arc_
          << " planned_return_s=" << extended_return_end_arc_
          << " source=" << (source_.empty() ? 0 : source_.front().id())
          << " opposite=" << (opposite_.empty() ? 0 : opposite_.front().id())
          << " return_x=" << (lines_.empty() ? 0.0 : lines_.back().end.position.x)
          << " return_y=" << (lines_.empty() ? 0.0 : lines_.back().end.position.y)
          << " actual_min_clearance_m=" << clearance
          << " actual_outside_corridor=" << outside << " actual_returned=" << returned
          << " planning_ms=" << elapsed;
  if (reason_ == "stale_perception_or_odometry") {
    const auto now = clock_->now();
    if (data_ && data_->dynamic_object) {
      message << " object_age_s=" <<
        (now - rclcpp::Time(data_->dynamic_object->header.stamp, now.get_clock_type())).seconds();
    }
    if (data_ && data_->self_odometry) {
      message << " odometry_age_s=" <<
        (now - rclcpp::Time(data_->self_odometry->header.stamp, now.get_clock_type())).seconds();
    }
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 5000, "%s", message.str().c_str());
    last_diagnostic_phase_ = phase_;
    return;
  }
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
  const auto & p = parameters_->opposite_lane;
  const auto & handler = *data_->route_handler;
  for (const auto & source : source_) {
    DrivableLanes drivable;
    drivable.left_lane = source;
    drivable.right_lane = source;
    const auto boundary = left_ ? source.leftBound() : source.rightBound();
    const auto candidates = left_ ? handler.getLeftOppositeLanelets(source)
                                  : handler.getRightOppositeLanelets(source);
    std::optional<lanelet::ConstLanelet> selected;
    if ((p.campus_agv ? source.attributeOr("subtype", std::string{}) == "road" : road(source)) &&
        allowedBoundary(boundary)) {
      for (const auto & lane : candidates) {
        if (p.campus_agv ? lane.attributeOr("subtype", std::string{}) != "road" : !road(lane)) continue;
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
    ? 0.2 : trajectory.lat_acc_map.find(candidate_speed_).second;
  if (jerk <= 0.0 || acceleration <= 0.0) {
    reason_ = "invalid_lateral_limits";
    return false;
  }
  // Four jerk phases: d = j*T^3/32. Also enforce a conservative lateral acceleration bound.
  const double duration = std::max(
    std::cbrt(32.0 * std::abs(shift) / jerk),
    std::sqrt(8.0 * std::abs(shift) / acceleration));
  const double shift_distance = std::max(p.min_shift_distance, candidate_speed_ * duration);
  const double obstacle_arc = autoware::motion_utils::calcSignedArcLength(
    reference_.points, reference_.points.front().point.pose.position,
    target_->object.kinematics.initial_pose_with_covariance.pose.position);
  const double object_front = obstacle_arc - target_->length * 0.5;
  const double object_rear = obstacle_arc + target_->length * 0.5;
  const double current_speed = std::abs(data_->self_odometry->twist.twist.linear.x);
  const double braking_distance = p.campus_agv ? std::max(0.0,
    (current_speed * current_speed - candidate_speed_ * candidate_speed_) /
      (-2.0 * trajectory.min_longitudinal_acc)) : 0.0;
  const double borrow_start = arc[ego_index] + p.prepare_distance + braking_distance;
  const double borrow_end = borrow_start + shift_distance;
  const double return_start = object_rear + trainLength() + p.return_distance_after_object;
  const double return_end = return_start + shift_distance;
  // Allow a straight settling section for the last trailer before declaring completion.
  const double finish = return_end + postReturnDistance(trainLength(), p, *lc_parameters_);
  if (borrow_end + data_->parameters.vehicle_info.max_longitudinal_offset_m +
        p.longitudinal_margin >= object_front || finish >= arc.back()) {
    reason_ = "insufficient_borrow_or_return_distance";
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[OPPOSITE_BORROWING] insufficient_distance borrow_end=%.2f object_front=%.2f finish=%.2f reference_end=%.2f",
      borrow_end, object_front, finish, arc.back());
    return false;
  }
  const auto graphs = data_->route_handler->getOverallGraphPtr();
  if (!graphs) {
    reason_ = "routing_graph_unavailable";
    return false;
  }
  if (!p.campus_agv) for (const auto & lanes : {source_, geometry}) {
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
  original_return_end_arc_ = return_end;
  extended_return_end_arc_ = return_end;
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
  shifter.setVelocity(candidate_speed_);
  shifter.setLateralAccelerationLimit(acceleration);
  shifter.setLongitudinalAcceleration(0.0);
  shifter.setShiftLines(lines_);
  if (!shifter.generate(&path_)) {
    reason_ = "path_generation_failed";
    return false;
  }
  path_.path.points.resize(finish_index + 1);
  path_.shift_length.resize(finish_index + 1);
  applyCandidateVelocity(path_);
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
  const double stamp = rclcpp::Time(data_->self_odometry->header.stamp).seconds();
  const auto previous_stamp = last_actual_stamp_s_;
  last_actual_stamp_s_ = stamp;
  if (!actual_history_.empty()) {
    const double distance = calc_distance2d(actual_history_.back(), pose);
    double reachable = 2.0;
    if (parameters_->opposite_lane.campus_agv && active_configuration_.geometries.empty() &&
        previous_stamp) {
      const double elapsed = stamp - *previous_stamp;
      if (elapsed < 0.0) return false;
      // An unhitched body has no articulation state to integrate between samples.
      // Bound progress by observation time and configured speed instead of cycle spacing.
      reachable = 0.5 + parameters_->opposite_lane.max_speed * elapsed;
    }
    if (distance > reachable) return false;
    if (distance < 0.05) return true;
  }
  actual_history_.push_back(pose);
  return true;
}

bool OppositeLaneBorrowing::validate(const ShiftedPath & path, bool recovery)
{
  reason_.clear();
  conflict_object_.clear();
  conflict_path_index_ = 0;
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
  std::vector<bool> static_objects;
  std::vector<Polygon> static_envelopes;
  bool has_dynamic_objects = false;
  for (const auto & object : data_->dynamic_object->objects) {
    const bool stationary = isStaticObject(object);
    const auto speed_bound = objectMotionBound(object, !stationary);
    if (!speed_bound) {
      reason_ = "invalid_object_prediction";
      return false;
    }
    object_speeds.push_back(*speed_bound);
    static_objects.push_back(stationary);
    has_dynamic_objects = has_dynamic_objects || !stationary;
    static_envelopes.push_back(objectEnvelope(object,
      object.kinematics.initial_pose_with_covariance.pose,
      parameters_->opposite_lane.longitudinal_margin, parameters_->opposite_lane.lateral_margin));
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
      ? 0.2 : trajectory.lat_acc_map.find(candidate_speed_).second;
    const double curvature_limit = std::min(
      std::tan(vehicle.max_steer_angle_rad) / vehicle.wheel_base_m,
      lateral_acceleration / (candidate_speed_ * candidate_speed_));
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
            std::pow(candidate_speed_, 3) > trajectory.lateral_jerk * 1.05)) {
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
  required_arrival_time_ = latest.empty() ? std::numeric_limits<double>::infinity() : latest.back() + p.min_time_gap;
  if (p.campus_agv && (latest.empty() || earliest.empty())) {
    reason_ = "arrival_time_unreachable";
    return false;
  }
  if ((!p.campus_agv || has_dynamic_objects) &&
      (!borrowing::coversReturn(latest, p.min_time_gap, p.max_prediction_horizon) || earliest.empty())) {
    reason_ = "prediction_does_not_cover_complete_return";
    return false;
  }
  using Box = boost::geometry::model::box<autoware_utils::Point2d>;
  using IndexedEnvelope = std::pair<Box, size_t>;
  boost::geometry::index::rtree<IndexedEnvelope, boost::geometry::index::quadratic<16>> static_index;
  std::vector<size_t> dynamic_indices;
  for (size_t i = 0; i < static_objects.size(); ++i) {
    if (static_objects[i]) {
      Box box;
      boost::geometry::envelope(static_envelopes[i], box);
      static_index.insert({box, i});
    } else {
      dynamic_indices.push_back(i);
    }
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
        boost::geometry::clear(swept);
        boost::geometry::convex_hull(corners, swept);
      }
      if (!p.campus_agv && !crosswalk_area.empty() && !boost::geometry::disjoint(swept, crosswalk_area)) {
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
      Box swept_box;
      boost::geometry::envelope(swept, swept_box);
      std::vector<IndexedEnvelope> nearby_static;
      static_index.query(boost::geometry::index::intersects(swept_box), std::back_inserter(nearby_static));
      for (const auto & entry : nearby_static) {
        if (!boost::geometry::disjoint(swept, static_envelopes[entry.second])) {
          reason_ = body ? "trailer_collision" : "vehicle_collision";
          conflict_object_ = autoware_utils_uuid::to_hex_string(data_->dynamic_object->objects[entry.second].object_id);
          conflict_path_index_ = start + i;
          return false;
        }
      }
      // Dynamic objects retain all predictions, including objects initially outside the corridor.
      for (const size_t object_index : dynamic_indices) {
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
            conflict_path_index_ = start + i;
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
    ? 0.2 : trajectory.lat_acc_map.find(candidate_speed_).second;
  const double duration = std::max(
    std::cbrt(32.0 * std::abs(offset) / trajectory.lateral_jerk),
    std::sqrt(8.0 * std::abs(offset) / acceleration));
  const double min_distance = std::max(p.min_shift_distance, candidate_speed_ * duration);
  if (extended_return_end_arc_ > original_return_end_arc_ + 1e-3) return false;
  const auto saved_reason = reason_;
  for (double extra = 0.0; extra <= 20.0; extra += 1.0) {
    if (deadline_ && std::chrono::steady_clock::now() > *deadline_) break;
    const double end_arc = arc[ego_index] + min_distance + extra;
    if (!return_extension_conflict_.empty() && end_arc < extended_return_end_arc_) continue;
    const auto end_it = std::lower_bound(arc.begin(), arc.end(), end_arc);
    const auto finish_it = std::lower_bound(
      arc.begin(), arc.end(), end_arc + postReturnDistance(trainLength(), p, *lc_parameters_));
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
    shifter.setVelocity(candidate_speed_);
    shifter.setLateralAccelerationLimit(acceleration);
    if (ego_index <= 1) continue;
    // PathShifter does not use start_shift_length as its base offset. Seed the measured
    // offset with a line wholly behind the ego before adding the actual returning line.
    ShiftLine seed;
    seed.start_idx = 0;
    seed.end_idx = ego_index;
    seed.start = reference.points.front().point.pose;
    seed.end = reference.points[ego_index].point.pose;
    seed.end_shift_length = offset;
    shifter.setShiftLines({seed, line});
    ShiftedPath candidate;
    if (!shifter.generate(&candidate)) continue;
    applyCandidateVelocity(candidate);
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

bool OppositeLaneBorrowing::extendContinuousReturn(double clear_s, const std::string & conflict)
{
  const auto & p = parameters_->opposite_lane;
  const auto arc = arcLengths(reference_);
  const auto & ego = data_->self_odometry->pose.pose;
  const size_t ego_index = data_->findEgoIndex(reference_.points);
  const double ego_s = autoware::motion_utils::calcSignedArcLength(
    reference_.points, reference_.points.front().point.pose.position, ego.position);
  const double offset = autoware_utils::calc_lateral_deviation(
    reference_.points[ego_index].point.pose, ego.position);
  const double slope = std::tan(autoware_utils::normalize_radian(
    tf2::getYaw(ego.orientation) - tf2::getYaw(reference_.points[ego_index].point.pose.orientation)));
  const size_t committed_index = data_->findEgoIndex(path_.path.points);
  double curvature = 0.0;
  if (committed_index > 0 && committed_index + 1 < path_.path.points.size()) {
    const auto & a = path_.path.points[committed_index - 1].point.pose;
    const auto & b = path_.path.points[committed_index + 1].point.pose;
    const double distance = calc_distance2d(a, b);
    if (distance > 1e-3) curvature = autoware_utils::normalize_radian(
      tf2::getYaw(b.orientation) - tf2::getYaw(a.orientation)) / distance;
  }
  const double second = curvature * std::pow(1.0 + slope * slope, 1.5);
  const double lower = std::max(extended_return_end_arc_, ego_s + p.min_shift_distance);
  const double upper = original_return_end_arc_ + p.max_return_extension_distance;
  const std::string saved = reason_;
  std::string failure = "extension_distance_insufficient";
  // Continue the search across cycles instead of retrying the same short candidates on timeout.
  if (continuous_search_conflict_ != conflict) {
    continuous_search_conflict_ = conflict;
    continuous_search_index_ = 0;
  }
  const size_t steps = upper > lower ? static_cast<size_t>(std::floor(
    (upper - lower + 1e-6) / p.return_extension_step)) : 0;
  const size_t candidates = 3 * steps;
  for (size_t attempt = 0; attempt < candidates; ++attempt) {
      const size_t cursor = continuous_search_index_ % candidates;
      const double end_s = lower + (cursor / 3 + 1) * p.return_extension_step;
      const int mode = cursor % 3;
      if (deadline_ && std::chrono::steady_clock::now() >= *deadline_) {
        reason_ = saved + ":no_safe_delayed_return:planning_budget_exceeded";
        return false;
      }
      continuous_search_index_ = (cursor + 1) % candidates;
      const auto finish = std::lower_bound(arc.begin(), arc.end(), end_s + postReturnDistance(trainLength(), p, *lc_parameters_));
      if (finish == arc.end()) continue;
      const double hold_end = std::max(ego_s + p.min_shift_distance, clear_s);
      if (mode && hold_end + p.min_shift_distance >= end_s) continue;
      const double hold_offset = mode == 2 ? lines_.front().end_shift_length : offset;
      const borrowing::LateralSegment first(offset, slope, second,
        mode ? hold_offset : 0.0, (mode ? hold_end : end_s) - ego_s);
      const borrowing::LateralSegment last(hold_offset, 0.0, 0.0, 0.0,
        std::max(1e-3, end_s - hold_end));
      ShiftedPath candidate;
      candidate.path = reference_;
      candidate.path.points.resize(std::distance(arc.begin(), finish) + 1);
      candidate.shift_length.resize(candidate.path.points.size());
      for (size_t i = 0; i < candidate.path.points.size(); ++i) {
        double lateral = offset;
        double derivative = slope;
        if (arc[i] >= ego_s) {
          const bool final_segment = mode && arc[i] > hold_end;
          lateral = final_segment ? last.value(arc[i] - hold_end) : first.value(arc[i] - ego_s);
          derivative = final_segment ? last.slope(arc[i] - hold_end) : first.slope(arc[i] - ego_s);
        }
        auto & pose = candidate.path.points[i].point.pose;
        pose = calc_offset_pose(reference_.points[i].point.pose, 0.0, lateral, 0.0);
        pose.orientation = autoware_utils::create_quaternion_from_yaw(
          tf2::getYaw(reference_.points[i].point.pose.orientation) + std::atan(derivative));
        candidate.shift_length[i] = lateral;
      }
      // Keep the committed prefix; match the measured tangent/curvature immediately behind ego
      // as well, since three-point curvature validation includes the preceding sample.
      for (size_t i = 0; i < ego_index && i < path_.path.points.size(); ++i) {
        candidate.path.points[i] = path_.path.points[i];
        if (i + 2 >= ego_index) {
          const double distance = std::max(1e-3, ego_s - arc[i]);
          auto & pose = candidate.path.points[i].point.pose;
          pose = calc_offset_pose(ego, -distance, 0.5 * curvature * distance * distance, 0.0);
          pose.orientation = autoware_utils::create_quaternion_from_yaw(
            tf2::getYaw(ego.orientation) - curvature * distance);
        }
      }
      candidate.path.points[ego_index].point.pose = ego;
      applyCandidateVelocity(candidate);
      if (!validate(candidate)) { failure = reason_; continue; }
      auto returning = lines_.back();
      returning.start_idx = ego_index;
      returning.start = ego;
      returning.start_shift_length = offset;
      returning.end_idx = std::distance(arc.begin(), std::lower_bound(arc.begin(), arc.end(), end_s));
      returning.end = reference_.points[returning.end_idx].point.pose;
      returning.end_shift_length = 0.0;
      path_ = std::move(candidate);
      lines_.back() = returning;
      extended_return_end_arc_ = end_s;
      return_extension_conflict_ = conflict;
      recovery_ = false;
      reason_ = "return_extended_for_obstacle";
      continuous_search_index_ = 0;
      return true;
  }
  reason_ = saved + ":no_safe_delayed_return:" + failure;
  return false;
}

bool OppositeLaneBorrowing::extendReturn()
{
  if (reference_.points.size() < 2 || lines_.size() < 2 || actual_history_.empty()) return false;
  const auto & p = parameters_->opposite_lane;
  const auto saved_reason = reason_;
  const auto conflict = conflict_object_;
  const auto original_conflict_index = conflict_path_index_;
  bool accepted = false;
  struct RestoreConflict {
    std::function<void()> restore;
    ~RestoreConflict() { restore(); }
  } restore_conflict{[&]() {
    if (!accepted) {
      conflict_object_ = conflict;
      conflict_path_index_ = original_conflict_index;
    }
  }};
  std::string candidate_failure_reason;
  const auto arc = arcLengths(reference_);
  const size_t ego_index = data_->findEgoIndex(reference_.points);
  const auto & ego = data_->self_odometry->pose.pose;
  const auto obstacle = std::find_if(
    data_->dynamic_object->objects.begin(), data_->dynamic_object->objects.end(),
    [&](const auto & object) {
      return autoware_utils_uuid::to_hex_string(object.object_id) == conflict;
    });
  // Candidate generation need not wait for static classification. Until the observation
  // window is complete, validate() still applies the full temporal prediction checks.
  const auto motion = obstacle == data_->dynamic_object->objects.end()
    ? std::optional<double>{} : objectMotionBound(*obstacle, !p.campus_agv);
  if (!motion || *motion > (p.campus_agv ? p.static_speed_threshold : 0.1)) {
    reason_ = saved_reason + ":secondary_object_not_static";
    return false;
  }
  const double obstacle_s = autoware::motion_utils::calcSignedArcLength(
    reference_.points, reference_.points.front().point.pose.position,
    obstacle->kinematics.initial_pose_with_covariance.pose.position);
  const double clear_s = obstacle_s + obstacle->shape.dimensions.x * 0.5 +
    trainLength() + p.longitudinal_margin;
  const double initial_return_start_s = autoware::motion_utils::calcSignedArcLength(
    reference_.points, reference_.points.front().point.pose.position,
    lines_.back().start.position);
  const double return_length = extended_return_end_arc_ - initial_return_start_s;
  if (return_length <= 0.0) return false;
  // Before the return starts, retain the committed outgoing shift and move the
  // complete return maneuver beyond the newly detected object.
  if (arc[ego_index] < initial_return_start_s - 0.25 && !recovery_) {
    for (double extra = p.return_extension_step;
         extra <= p.max_return_extension_distance + 1e-6;
         extra += p.return_extension_step) {
      if (deadline_ && std::chrono::steady_clock::now() > *deadline_) break;
      const double start_s = std::max(initial_return_start_s + extra, clear_s);
      const double end_s = start_s + return_length;
      if (end_s > original_return_end_arc_ + p.max_return_extension_distance + 1e-6) break;
      const auto start_it = std::lower_bound(arc.begin(), arc.end(), start_s);
      const auto end_it = std::lower_bound(arc.begin(), arc.end(), end_s);
      const auto finish_it = std::lower_bound(
        arc.begin(), arc.end(), end_s + postReturnDistance(trainLength(), p, *lc_parameters_));
      if (start_it == arc.end() || end_it == arc.end() || finish_it == arc.end()) break;
      auto reference = reference_;
      reference.points.resize(std::distance(arc.begin(), finish_it) + 1);
      auto returning = lines_.back();
      returning.start_idx = std::distance(arc.begin(), start_it);
      returning.end_idx = std::distance(arc.begin(), end_it);
      returning.start = reference.points[returning.start_idx].point.pose;
      returning.end = reference.points[returning.end_idx].point.pose;
      PathShifter shifter;
      shifter.setPath(reference);
      shifter.setVelocity(candidate_speed_);
      shifter.setLateralAccelerationLimit(
        lc_parameters_->trajectory.lat_acc_map.base_vel.empty() ? 0.2 :
          lc_parameters_->trajectory.lat_acc_map.find(candidate_speed_).second);
      shifter.setShiftLines({lines_.front(), returning});
      ShiftedPath candidate;
      if (!shifter.generate(&candidate)) continue;
      applyCandidateVelocity(candidate);
      if (!validate(candidate)) {
        candidate_failure_reason = reason_;
        continue;
      }
      path_ = std::move(candidate);
      lines_.back() = returning;
      return_extension_conflict_ = conflict;
      extended_return_end_arc_ = end_s;
      reason_ = "return_extended_for_obstacle";
      accepted = true;
      return true;
    }
  }
  if (p.campus_agv) {
    accepted = extendContinuousReturn(clear_s, conflict);
    return accepted;
  }
  const double offset = autoware_utils::calc_lateral_deviation(
    reference_.points[ego_index].point.pose, ego.position);
  if (std::abs(offset) < 0.5) return false;
  const auto & trajectory = lc_parameters_->trajectory;
  const double acceleration = trajectory.lat_acc_map.base_vel.empty()
    ? 0.2 : trajectory.lat_acc_map.find(candidate_speed_).second;
  const double duration = std::max(
    std::cbrt(32.0 * std::abs(offset) / trajectory.lateral_jerk),
    std::sqrt(8.0 * std::abs(offset) / acceleration));
  const double min_distance = std::max(p.min_shift_distance, candidate_speed_ * duration);
  const auto end_base = std::max(extended_return_end_arc_, arc[ego_index] + min_distance);
  for (double extra = p.return_extension_step;
       extra <= p.max_return_extension_distance + 1e-6;
       extra += p.return_extension_step) {
    if (deadline_ && std::chrono::steady_clock::now() > *deadline_) break;
    const double end_arc = end_base + extra;
    if (end_arc > original_return_end_arc_ + p.max_return_extension_distance + 1e-6) {
      candidate_failure_reason = "extension_limit";
      break;
    }
    const auto end_it = std::lower_bound(arc.begin(), arc.end(), end_arc);
    const auto finish_it = std::lower_bound(arc.begin(), arc.end(), end_arc + postReturnDistance(trainLength(), p, *lc_parameters_));
    if (end_it == arc.end() || finish_it == arc.end() || ego_index <= 1) {
      candidate_failure_reason = "reference_distance_insufficient";
      break;
    }
    const size_t end_index = std::distance(arc.begin(), end_it);
    const size_t finish_index = std::distance(arc.begin(), finish_it);
    auto reference = reference_;
    reference.points.resize(finish_index + 1);
    ShiftLine seed;
    seed.start = reference.points.front().point.pose;
    seed.end = reference.points[ego_index].point.pose;
    seed.end_shift_length = offset;
    ShiftLine returning;
    returning.start_idx = ego_index;
    returning.end_idx = end_index;
    returning.start = reference.points[ego_index].point.pose;
    returning.end = reference.points[end_index].point.pose;
    returning.start_shift_length = offset;
    returning.end_shift_length = 0.0;
    PathShifter shifter;
    shifter.setPath(reference);
    shifter.setVelocity(candidate_speed_);
    shifter.setLateralAccelerationLimit(acceleration);
    shifter.setShiftLines({seed, returning});
    ShiftedPath candidate;
    if (!shifter.generate(&candidate)) {
      candidate_failure_reason = "path_shifter_failed";
      continue;
    }
    applyCandidateVelocity(candidate);
    const auto idx = data_->findEgoIndex(candidate.path.points);
    if (calc_distance2d(candidate.path.points[idx].point.pose, ego) > 0.25) {
      candidate_failure_reason = "position_discontinuity";
      continue;
    }
    if (std::abs(autoware_utils::normalize_radian(
          tf2::getYaw(candidate.path.points[idx].point.pose.orientation) -
          tf2::getYaw(ego.orientation))) > lc_parameters_->th_finish_judge_yaw_diff) {
      candidate_failure_reason = "heading_discontinuity";
      continue;
    }
    reason_ = saved_reason;
    conflict_object_ = conflict;
    if (!validate(candidate)) {
      candidate_failure_reason = reason_;
      continue;
    }
    path_ = std::move(candidate);
    lines_.back() = returning;
    recovery_ = false;
    return_extension_conflict_ = conflict;
    extended_return_end_arc_ = end_arc;
    reason_ = "return_extended_for_obstacle";
    accepted = true;
    return true;
  }
  reason_ = saved_reason + ":no_safe_delayed_return" +
    (candidate_failure_reason.empty() ? "" : ":" + candidate_failure_reason);
  conflict_object_ = conflict;
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
    (parameters_->opposite_lane.campus_agv && !associated_target_id_.empty() ? associated_target_id_ :
      (target_ ? autoware_utils_uuid::to_hex_string(target_->object.object_id) : "no_target"));
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
  if (!committed_) {
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
  if (!safe_ || waiting) stop(out.path, false);
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
