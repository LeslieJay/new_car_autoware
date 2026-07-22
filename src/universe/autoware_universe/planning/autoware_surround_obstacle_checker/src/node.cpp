// Copyright 2020 Tier IV, Inc.
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

#include "node.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware::surround_obstacle_checker
{
namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_utils::create_point;
using autoware_utils::pose2transform;

SurroundObstacleCheckerNode::SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("surround_obstacle_checker_node", node_options)
{
  label_map_ = {
    {ObjectClassification::UNKNOWN, "unknown"}, {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"}, {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"}, {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"}, {ObjectClassification::PEDESTRIAN, "pedestrian"}};
  // Parameters
  {
    param_listener_ = std::make_shared<surround_obstacle_checker_node::ParamListener>(
      this->get_node_parameters_interface());

    logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  }

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  // Publishers
  pub_clear_velocity_limit_ = this->create_publisher<VelocityLimitClearCommand>(
    "~/output/velocity_limit_clear_command", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_ = this->create_publisher<VelocityLimit>(
    "~/output/max_velocity", rclcpp::QoS{1}.transient_local());
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);
  pub_status_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    "~/output/status", rclcpp::QoS{1}.transient_local());
  cli_set_stop_ =
    this->create_client<tier4_control_msgs::srv::SetStop>("/control/vehicle_cmd_gate/set_stop");

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&SurroundObstacleCheckerNode::onTimer, this));

  // Stop Checker
  vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);

  // Debug
  odometry_ptr_ = std::make_shared<nav_msgs::msg::Odometry>();
  {
    const auto param = param_listener_->get_params();
    const auto check_distances = getCheckDistances(param.debug_footprint_label);
    debug_ptr_ = std::make_shared<SurroundObstacleCheckerDebugNode>(
      vehicle_info_, param.debug_footprint_label, check_distances.at(0), check_distances.at(1),
      check_distances.at(2), param.surround_check_hysteresis_distance, odometry_ptr_->pose.pose,
      this->get_clock(), *this);
    odometry_ptr_.reset();
    if (!param.request_command_gate_stop) {
      publishReady();
    }
  }
}

std::array<double, 3> SurroundObstacleCheckerNode::getCheckDistances(
  const std::string & str_label) const
{
  const auto param = param_listener_->get_params();
  const auto & obstacle_param = param.obstacle_types_map.at(str_label);
  return {
    obstacle_param.surround_check_front_distance, obstacle_param.surround_check_side_distance,
    obstacle_param.surround_check_back_distance};
}

bool SurroundObstacleCheckerNode::getUseDynamicObject() const
{
  const auto param = param_listener_->get_params();
  bool use_dynamic_object = false;
  for (const auto & label_pair : label_map_) {
    use_dynamic_object |= param.object_types_map.at(label_pair.second).enable_check;
  }
  return use_dynamic_object;
}

void SurroundObstacleCheckerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic();

  const auto now = this->now();
  const auto odometry = sub_odometry_.take_data();
  const auto pointcloud = sub_pointcloud_.take_data();
  const auto objects = sub_dynamic_objects_.take_data();
  if (odometry && odometry != odometry_ptr_) {
    odometry_ptr_ = odometry;
    last_odometry_time_ = now;
  }
  if (pointcloud && pointcloud != pointcloud_ptr_) {
    pointcloud_ptr_ = pointcloud;
    last_pointcloud_time_ = now;
  }
  if (objects && objects != object_ptr_) {
    object_ptr_ = objects;
    last_object_time_ = now;
  }

  const auto param = param_listener_->get_params();
  const auto use_dynamic_object = getUseDynamicObject();
  const bool input_unsafe =
    param.fail_safe_on_data_timeout &&
    isInputUnsafe(
    use_dynamic_object, param.pointcloud.enable_check, last_odometry_time_, last_object_time_,
    last_pointcloud_time_, param.data_timeout_sec);

  if (!odometry_ptr_ && !param.fail_safe_on_data_timeout) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current velocity...");
    return;
  }

  if (param.publish_debug_footprints && odometry_ptr_) {
    debug_ptr_->publishFootprints();
  }

  if (param.pointcloud.enable_check && !pointcloud_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for pointcloud info...");
  }

  if (use_dynamic_object && !object_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
  }

  if (!param.pointcloud.enable_check && !use_dynamic_object) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "Surround obstacle check is disabled for all dynamic object types and for pointcloud check.");
  }

  const auto nearest_obstacle =
    input_unsafe || !odometry_ptr_ ? std::optional<StopObstacle>{} : getNearestObstacle();
  const auto is_vehicle_stopped = odometry_ptr_ && vehicle_stop_checker_->isVehicleStopped();

  constexpr double epsilon = 1e-3;
  const double threshold =
    state_ == State::PASS ? epsilon : param.surround_check_hysteresis_distance;
  const bool is_obstacle_found =
    input_unsafe || (nearest_obstacle && nearest_obstacle->nearest_distance < threshold);
  const State previous_state = state_;
  bool is_stop_required = false;
  std::tie(is_stop_required, last_obstacle_found_time_) = isStopRequired(
    is_obstacle_found, is_vehicle_stopped, state_, last_obstacle_found_time_,
    param.state_clear_time, param.stop_only_when_stopped);
  state_ = is_stop_required ? State::STOP : State::PASS;

  if (previous_state == State::PASS && state_ == State::STOP) {
    VelocityLimit velocity_limit;
    velocity_limit.stamp = now;
    velocity_limit.max_velocity = 0.0;
    velocity_limit.use_constraints = false;
    velocity_limit.sender = param.stop_request_source;
    pub_velocity_limit_->publish(velocity_limit);
    RCLCPP_WARN(get_logger(), "stop requested because a surrounding hazard was detected.");
  } else if (previous_state == State::STOP && state_ == State::PASS) {
    VelocityLimitClearCommand clear_command;
    clear_command.stamp = now;
    clear_command.command = true;
    clear_command.sender = param.stop_request_source;
    pub_clear_velocity_limit_->publish(clear_command);
  }

  requestCommandGateStop(state_ == State::STOP);

  std::string reason = "clear";
  if (input_unsafe) {
    const auto is_stale = [&now, &param](const std::optional<rclcpp::Time> & stamp) {
        return !stamp || (now - *stamp).seconds() > param.data_timeout_sec;
      };
    if (is_stale(last_odometry_time_)) {
      reason = "odometry_timeout";
    } else if (use_dynamic_object && is_stale(last_object_time_)) {
      reason = "objects_timeout";
    } else {
      reason = "pointcloud_timeout";
    }
  } else if (nearest_obstacle && is_obstacle_found) {
    reason = nearest_obstacle->label;
  } else if (state_ == State::STOP) {
    reason = "clearance_hysteresis";
  }
  publishStatus(reason, nearest_obstacle);

  if (nearest_obstacle.has_value()) {
    debug_ptr_->pushStopObstacle(nearest_obstacle);
  }

  if (state_ == State::STOP && odometry_ptr_) {
    debug_ptr_->pushPose(odometry_ptr_->pose.pose, PoseType::NoStart);
  }

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);

  debug_ptr_->publish();
}

int SurroundObstacleCheckerNode::getObjectLabel(const PredictedObject & object) const
{
  if (object.classification.empty()) {
    return ObjectClassification::UNKNOWN;
  }
  const int label = object.classification.front().label;
  return label_map_.count(label) == 0 ? ObjectClassification::UNKNOWN : label;
}

bool SurroundObstacleCheckerNode::isInputUnsafe(
  const bool use_dynamic_objects, const bool pointcloud_enabled,
  const std::optional<rclcpp::Time> & last_odometry_time,
  const std::optional<rclcpp::Time> & last_object_time,
  const std::optional<rclcpp::Time> & last_pointcloud_time, const double timeout_sec) const
{
  const auto now = this->now();
  const auto is_stale = [&now, timeout_sec](const std::optional<rclcpp::Time> & stamp) {
      return !stamp || (now - *stamp).seconds() > timeout_sec;
    };
  return is_stale(last_odometry_time) || (use_dynamic_objects && is_stale(last_object_time)) ||
         (pointcloud_enabled && is_stale(last_pointcloud_time));
}

void SurroundObstacleCheckerNode::requestCommandGateStop(const bool stop)
{
  const auto param = param_listener_->get_params();
  if (!param.request_command_gate_stop) {
    publishReady();
    return;
  }
  const auto now = this->now();
  if (stop_request_pending_) {
    if (stop_request_time_ && (now - *stop_request_time_).seconds() <= param.data_timeout_sec) {
      return;
    }
    stop_request_pending_ = false;
    ++stop_request_sequence_;
    RCLCPP_ERROR(get_logger(), "vehicle command gate stop request timed out; retrying");
  }
  if (acknowledged_stop_request_ == stop || !cli_set_stop_->service_is_ready()) {
    return;
  }

  auto request = std::make_shared<tier4_control_msgs::srv::SetStop::Request>();
  request->stop = stop;
  request->request_source = param.stop_request_source;
  stop_request_pending_ = true;
  stop_request_time_ = now;
  const uint64_t sequence = ++stop_request_sequence_;
  cli_set_stop_->async_send_request(
    request,
    [this, stop, sequence](rclcpp::Client<tier4_control_msgs::srv::SetStop>::SharedFuture future) {
      if (sequence != stop_request_sequence_) {
        return;
      }
      stop_request_pending_ = false;
      tier4_control_msgs::srv::SetStop::Response::SharedPtr response;
      try {
        response = future.get();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "vehicle command gate stop request failed: %s", error.what());
        return;
      }
      if (!response->status.success) {
        RCLCPP_ERROR(
          get_logger(), "vehicle command gate rejected the stop request: %s",
          response->status.message.c_str());
        return;
      }
      acknowledged_stop_request_ = stop;
      publishReady();
    });
}

void SurroundObstacleCheckerNode::publishReady()
{
  if (!pub_ready_) {
    pub_ready_ = this->create_publisher<std_msgs::msg::Bool>(
      "~/output/ready", rclcpp::QoS{1}.transient_local());
  }
  std_msgs::msg::Bool ready;
  ready.data = true;
  pub_ready_->publish(ready);
}

void SurroundObstacleCheckerNode::publishStatus(
  const std::string & reason, const std::optional<StopObstacle> & obstacle)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(this->get_fully_qualified_name()) + ": surround obstacle checker";
  status.hardware_id = "none";
  status.level = state_ == State::PASS ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN;
  if (reason.find("timeout") != std::string::npos) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  status.message = reason;
  const auto add_value = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(item);
    };
  add_value("state", state_ == State::STOP ? "STOP" : "PASS");
  const auto param = param_listener_->get_params();
  add_value(
    "command_gate_acknowledged",
    !param.request_command_gate_stop ? "disabled" :
    acknowledged_stop_request_.has_value() ? (*acknowledged_stop_request_ ? "stop" : "clear") :
    "pending");
  const auto now = this->now();
  const auto add_age = [&add_value, &now](
    const std::string & key, const std::optional<rclcpp::Time> & stamp) {
      add_value(key, stamp ? std::to_string(std::max(0.0, (now - *stamp).seconds())) : "missing");
    };
  add_age("odometry_age_sec", last_odometry_time_);
  add_age("objects_age_sec", last_object_time_);
  add_age("pointcloud_age_sec", last_pointcloud_time_);
  if (obstacle) {
    add_value("nearest_distance", std::to_string(obstacle->nearest_distance));
  }
  pub_status_->publish(status);
}

std::optional<StopObstacle> SurroundObstacleCheckerNode::getNearestObstacle() const
{
  const auto nearest_pointcloud = getNearestObstacleByPointCloud();
  const auto nearest_object = getNearestObstacleByDynamicObject();
  if (!nearest_pointcloud.has_value() && !nearest_object.has_value()) {
    return {};
  }

  if (!nearest_pointcloud.has_value()) {
    return nearest_object;
  }

  if (!nearest_object.has_value()) {
    return nearest_pointcloud;
  }

  return nearest_pointcloud.value().nearest_distance < nearest_object.value().nearest_distance ?
         nearest_pointcloud :
         nearest_object;
}

std::optional<StopObstacle> SurroundObstacleCheckerNode::getNearestObstacleByPointCloud() const
{
  const auto param = param_listener_->get_params();

  if (!param.pointcloud.enable_check || !pointcloud_ptr_) {
    return std::nullopt;
  }

  if (pointcloud_ptr_->data.empty()) {
    return std::nullopt;
  }

  const auto transform_stamped =
    getTransform("base_link", pointcloud_ptr_->header.frame_id, pointcloud_ptr_->header.stamp, 0.5);

  if (!transform_stamped.has_value()) {
    return std::nullopt;
  }

  Eigen::Affine3f isometry =
    tf2::transformToEigen(transform_stamped.value().transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(*pointcloud_ptr_, transformed_pointcloud);
  autoware_utils::transform_pointcloud(transformed_pointcloud, transformed_pointcloud, isometry);

  const auto & pointcloud_param = param.obstacle_types_map.at("pointcloud");
  const double front_margin = pointcloud_param.surround_check_front_distance;
  const double side_margin = pointcloud_param.surround_check_side_distance;
  const double back_margin = pointcloud_param.surround_check_back_distance;
  const double base_to_front = vehicle_info_.max_longitudinal_offset_m + front_margin;
  const double base_to_rear = vehicle_info_.rear_overhang_m + back_margin;
  const double width = vehicle_info_.vehicle_width_m + side_margin * 2;
  const auto base_link_origin = []() {
      geometry_msgs::msg::Pose p;
      p.position.x = 0.0;
      p.position.y = 0.0;
      p.position.z = 0.0;
      p.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      return p;
    }();
  const auto ego_polygon =
    autoware_utils::to_footprint(base_link_origin, base_to_front, base_to_rear, width);

  // distance comparison on base_link frame
  geometry_msgs::msg::Point nearest_point_base_link;
  double minimum_distance = std::numeric_limits<double>::max();
  bool was_minimum_distance_updated = false;
  for (const auto & p : transformed_pointcloud) {
    Point2d boost_point(p.x, p.y);

    const auto distance_to_object = bg::distance(ego_polygon, boost_point);

    if (distance_to_object < minimum_distance) {
      nearest_point_base_link = create_point(p.x, p.y, p.z);
      minimum_distance = distance_to_object;
      was_minimum_distance_updated = true;
    }
  }

  if (was_minimum_distance_updated) {
    // transform the nearest point from base_link to map frame
    const auto & pose = odometry_ptr_->pose.pose;
    const auto nearest_point_map =
      autoware::universe_utils::transformPoint(nearest_point_base_link, pose);

    StopObstacle obstacle;
    obstacle.is_point_cloud = true;
    obstacle.nearest_distance = minimum_distance;
    obstacle.nearest_point = nearest_point_map;
    obstacle.uuid = UUID();  // Default UUID
    obstacle.label = "pointcloud";
    return obstacle;
  }
  return std::nullopt;
}

std::optional<StopObstacle> SurroundObstacleCheckerNode::getNearestObstacleByDynamicObject() const
{
  if (!object_ptr_ || !getUseDynamicObject()) {
    return std::nullopt;
  }

  const auto param = param_listener_->get_params();

  // TODO(murooka) check computation cost
  PredictedObject nearest_object;
  double minimum_distance = std::numeric_limits<double>::max();
  bool was_minimum_distance_updated = false;
  for (const auto & object : object_ptr_->objects) {
    const int label = getObjectLabel(object);
    const auto & str_label = label_map_.at(label);

    if (!param.object_types_map.at(str_label).enable_check) {
      continue;
    }
    const auto & object_param = param.obstacle_types_map.at(str_label);
    const double front_margin = object_param.surround_check_front_distance;
    const double side_margin = object_param.surround_check_side_distance;
    const double back_margin = object_param.surround_check_back_distance;
    const double base_to_front = vehicle_info_.max_longitudinal_offset_m + front_margin;
    const double base_to_rear = vehicle_info_.rear_overhang_m + back_margin;
    const double width = vehicle_info_.vehicle_width_m + side_margin * 2;
    const auto ego_polygon =
      autoware_utils::to_footprint(odometry_ptr_->pose.pose, base_to_front, base_to_rear, width);

    const auto object_polygon = autoware_utils::to_polygon2d(object);

    const auto distance_to_object = bg::distance(ego_polygon, object_polygon);

    if (distance_to_object < minimum_distance) {
      nearest_object = object;
      minimum_distance = distance_to_object;
      was_minimum_distance_updated = true;
    }
  }

  if (was_minimum_distance_updated) {
    const auto & object_position =
      nearest_object.kinematics.initial_pose_with_covariance.pose.position;
    StopObstacle obstacle;
    obstacle.is_point_cloud = false;
    obstacle.nearest_distance = minimum_distance;
    obstacle.nearest_point = object_position;
    obstacle.uuid = nearest_object.object_id;
    obstacle.label = label_map_.at(getObjectLabel(nearest_object));
    return obstacle;
  }
  return std::nullopt;
}

std::optional<geometry_msgs::msg::TransformStamped> SurroundObstacleCheckerNode::getTransform(
  const std::string & source, const std::string & target, const rclcpp::Time & stamp,
  double duration_sec) const
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  try {
    transform_stamped =
      tf_buffer_.lookupTransform(source, target, stamp, tf2::durationFromSec(duration_sec));
  } catch (const tf2::TransformException & ex) {
    return {};
  }

  return transform_stamped;
}

auto SurroundObstacleCheckerNode::isStopRequired(
  const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
  const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold,
  const bool stop_only_when_stopped) const -> std::pair<bool, std::optional<rclcpp::Time>>
{
  if (stop_only_when_stopped && !is_vehicle_stopped) {
    return std::make_pair(false, std::nullopt);
  }

  if (is_obstacle_found) {
    return std::make_pair(true, this->now());
  }

  if (state != State::STOP) {
    return std::make_pair(false, std::nullopt);
  }

  // Keep stop state
  if (last_obstacle_found_time.has_value()) {
    const auto elapsed_time = this->now() - last_obstacle_found_time.value();
    if (elapsed_time.seconds() <= time_threshold) {
      return std::make_pair(true, last_obstacle_found_time.value());
    }
  }

  return std::make_pair(false, std::nullopt);
}

}  // namespace autoware::surround_obstacle_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::surround_obstacle_checker::SurroundObstacleCheckerNode)
