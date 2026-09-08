// Copyright 2026 BYD.
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

#include "agv_high_precision_reverse_controller/high_precision_reverse_node.hpp"

#include <autoware/freespace_planning_algorithms/reeds_shepp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <utility>

namespace agv_high_precision_reverse_controller
{

using autoware::freespace_planning_algorithms::ReedsSheppStateSpace;

namespace
{
ControllerParameters declareControllerParameters(rclcpp::Node & node)
{
  ControllerParameters p;
  p.wheel_base = node.declare_parameter<double>("vehicle.wheel_base", p.wheel_base);
  p.max_steering_angle =
    node.declare_parameter<double>("lateral.max_steering_angle", p.max_steering_angle);
  p.max_steering_rate =
    node.declare_parameter<double>("lateral.max_steering_rate", p.max_steering_rate);
  p.steering_offset = node.declare_parameter<double>("lateral.steering_offset", p.steering_offset);
  p.lateral_gain = node.declare_parameter<double>("lateral.lateral_gain", p.lateral_gain);
  p.heading_gain = node.declare_parameter<double>("lateral.heading_gain", p.heading_gain);
  p.softening_speed = node.declare_parameter<double>("lateral.softening_speed", p.softening_speed);
  p.max_reverse_speed =
    node.declare_parameter<double>("longitudinal.max_reverse_speed", p.max_reverse_speed);
  p.min_creep_speed =
    node.declare_parameter<double>("longitudinal.min_creep_speed", p.min_creep_speed);
  p.comfortable_deceleration = node.declare_parameter<double>(
    "longitudinal.comfortable_deceleration", p.comfortable_deceleration);
  p.stop_margin = node.declare_parameter<double>("longitudinal.stop_margin", p.stop_margin);
  p.final_approach_distance = node.declare_parameter<double>(
    "longitudinal.final_approach_distance", p.final_approach_distance);
  p.max_acceleration =
    node.declare_parameter<double>("longitudinal.max_acceleration", p.max_acceleration);
  p.max_deceleration =
    node.declare_parameter<double>("longitudinal.max_deceleration", p.max_deceleration);
  p.speed_kp = node.declare_parameter<double>("longitudinal.speed_kp", p.speed_kp);
  p.speed_ki = node.declare_parameter<double>("longitudinal.speed_ki", p.speed_ki);
  p.integral_limit =
    node.declare_parameter<double>("longitudinal.integral_limit", p.integral_limit);
  p.goal_distance_tolerance =
    node.declare_parameter<double>("goal.distance_tolerance", p.goal_distance_tolerance);
  p.goal_yaw_tolerance = node.declare_parameter<double>("goal.yaw_tolerance", p.goal_yaw_tolerance);
  p.stop_speed_tolerance =
    node.declare_parameter<double>("goal.stop_speed_tolerance", p.stop_speed_tolerance);
  p.overshoot_tolerance =
    node.declare_parameter<double>("safety.overshoot_tolerance", p.overshoot_tolerance);
  p.max_lateral_error =
    node.declare_parameter<double>("safety.max_lateral_error", p.max_lateral_error);
  p.max_heading_error =
    node.declare_parameter<double>("safety.max_heading_error", p.max_heading_error);
  return p;
}
}  // namespace

HighPrecisionReverseNode::HighPrecisionReverseNode(const rclcpp::NodeOptions & options)
: Node("agv_high_precision_reverse_controller", options),
  controller_(declareControllerParameters(*this)),
  last_odometry_receipt_(0, 0, get_clock()->get_clock_type()),
  last_observability_publish_(0, 0, get_clock()->get_clock_type())
{
  control_rate_ = declare_parameter<double>("control_rate", control_rate_);
  trajectory_publish_rate_ =
    declare_parameter<double>("trajectory_publish_rate", trajectory_publish_rate_);
  odometry_timeout_ = declare_parameter<double>("safety.odometry_timeout", odometry_timeout_);
  turning_radius_ = declare_parameter<double>("planning.turning_radius", turning_radius_);
  path_resolution_ = declare_parameter<double>("planning.path_resolution", path_resolution_);
  min_path_length_ = declare_parameter<double>("planning.min_path_length", min_path_length_);
  max_path_length_ = declare_parameter<double>("planning.max_path_length", max_path_length_);
  required_settled_cycles_ = static_cast<std::size_t>(
    std::max<std::int64_t>(1, declare_parameter<std::int64_t>("goal.required_settled_cycles", 10)));
  handoff_retry_interval_ =
    declare_parameter<double>("mode_handoff.retry_interval", handoff_retry_interval_);
  max_handoff_retries_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("mode_handoff.max_retries", 40)));

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::SensorDataQoS(),
    std::bind(&HighPrecisionReverseNode::onOdometry, this, std::placeholders::_1));
  rear_warning_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "~/input/rear_warning_level", 1,
    std::bind(&HighPrecisionReverseNode::onRearWarning, this, std::placeholders::_1));
  trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", rclcpp::QoS(1).transient_local());
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/path_markers", 1);
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/output/goal", rclcpp::QoS(1).transient_local().reliable());
  control_pub_ = create_publisher<autoware_control_msgs::msg::Control>("~/output/control_cmd", 1);
  gear_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>("~/output/gear_cmd", 1);
  indicators_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "~/output/turn_indicators_cmd", 1);
  hazards_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard_lights_cmd", 1);
  debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/debug/tracking", 1);
  state_pub_ =
    create_publisher<std_msgs::msg::UInt8>("~/output/state", rclcpp::QoS(1).transient_local());

  set_goal_service_ = create_service<SetGoalPose>(
    "~/set_goal_pose",
    std::bind(
      &HighPrecisionReverseNode::onSetGoal, this, std::placeholders::_1, std::placeholders::_2));
  replan_service_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_planning",
    std::bind(
      &HighPrecisionReverseNode::onReplan, this, std::placeholders::_1, std::placeholders::_2));
  cancel_service_ = create_service<std_srvs::srv::Trigger>(
    "~/cancel",
    std::bind(
      &HighPrecisionReverseNode::onCancel, this, std::placeholders::_1, std::placeholders::_2));

  set_pause_client_ =
    create_client<tier4_control_msgs::srv::SetPause>("/control/vehicle_cmd_gate/set_pause");
  clear_route_client_ =
    create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/api/routing/clear_route");
  change_to_local_client_ =
    create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_local");
  change_to_autonomous_client_ =
    create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous");
  enable_control_client_ =
    create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/enable_autoware_control");
  engage_client_ =
    create_client<tier4_external_api_msgs::srv::Engage>("/api/autoware/set/engage");

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_rate_));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&HighPrecisionReverseNode::onTimer, this));
  changeState(State::IDLE, "ready");
  RCLCPP_INFO(
    get_logger(), "High precision reverse controller ready (%.0f Hz, %.3f m path resolution)",
    control_rate_, path_resolution_);
}

void HighPrecisionReverseNode::onTimer()
{
  processModeHandoff();
  if (state_ == State::IDLE) {
    return;
  }
  if (state_ == State::TRACKING && handoff_phase_ != HandoffPhase::REVERSING) {
    publishStop(true);
    return;
  }
  if (!odometryIsFresh()) {
    if (state_ == State::TRACKING) {
      changeState(State::SAFETY_HOLD, "odometry timeout");
    }
    publishStop(true);
    return;
  }
  if (state_ == State::GOAL_REACHED || state_ == State::ABORTED) {
    publishStop(false);
    publishObservability();
    return;
  }
  if (rear_warning_level_ >= 2U) {
    if (state_ != State::SAFETY_HOLD) {
      changeState(State::SAFETY_HOLD, "rear obstacle warning");
    }
    publishStop(true);
    return;
  }
  if (state_ == State::SAFETY_HOLD) {
    changeState(State::TRACKING, "safety inputs recovered");
  }

  VehicleState vehicle;
  vehicle.x = odometry_->pose.pose.position.x;
  vehicle.y = odometry_->pose.pose.position.y;
  vehicle.yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  vehicle.speed = odometry_->twist.twist.linear.x;
  const auto result = controller_.compute(vehicle, 1.0 / std::max(1.0, control_rate_));
  if (!result.valid || result.tracking_error) {
    changeState(State::ABORTED, result.reason);
    publishStop(false);
    publishObservability(&result);
    beginReturnToAutonomous();
    return;
  }

  if (result.goal_reached) {
    ++settled_cycles_;
    publishStop(true);
    if (settled_cycles_ >= required_settled_cycles_) {
      changeState(State::GOAL_REACHED, "pose and velocity settled");
      beginReturnToAutonomous();
    }
  } else {
    settled_cycles_ = 0U;
    publishCommand(result.steering_angle, result.acceleration, result.target_speed);
    publishGear(true);
    publishSignals(true, result.target_speed <= 0.001);
  }
  publishObservability(&result);
}

void HighPrecisionReverseNode::onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  odometry_ = std::move(message);
  last_odometry_receipt_ = now();
}

void HighPrecisionReverseNode::onRearWarning(std_msgs::msg::UInt8::ConstSharedPtr message)
{
  rear_warning_level_ = message->data;
}

void HighPrecisionReverseNode::onSetGoal(
  const std::shared_ptr<SetGoalPose::Request> request,
  std::shared_ptr<SetGoalPose::Response> response)
{
  goal_ = request->goal_pose;
  goal_.header.stamp = now();
  if (goal_.header.frame_id.empty()) {
    goal_.header.frame_id = "map";
  }
  has_goal_ = true;
  goal_pub_->publish(goal_);
  std::string reason;
  response->success = plan(reason);
  response->message = reason;
  response->path_points_num = static_cast<std::uint32_t>(controller_.path().size());
  if (!response->success) {
    changeState(State::ABORTED, reason);
    publishStop(false);
  } else {
    handoff_retry_count_ = 0U;
    handoff_retry_after_ = {};
    handoff_phase_ = HandoffPhase::PAUSE_FOR_REVERSE;
  }
}

void HighPrecisionReverseNode::processModeHandoff()
{
  if (std::chrono::steady_clock::now() < handoff_retry_after_) {
    return;
  }
  switch (handoff_phase_) {
    case HandoffPhase::PAUSE_FOR_REVERSE: {
        if (!set_pause_client_->service_is_ready()) {
          return;
        }
        auto request = std::make_shared<tier4_control_msgs::srv::SetPause::Request>();
        request->pause = true;
        handoff_phase_ = HandoffPhase::WAIT_PAUSE_FOR_REVERSE;
        set_pause_client_->async_send_request(
          request,
          std::bind(
            &HighPrecisionReverseNode::onPauseForReverseResponse, this, std::placeholders::_1));
        break;
      }
    case HandoffPhase::CLEAR_FORWARD_ROUTE: {
        if (!clear_route_client_->service_is_ready()) {
          return;
        }
        handoff_phase_ = HandoffPhase::WAIT_CLEAR_FORWARD_ROUTE;
        clear_route_client_->async_send_request(
          std::make_shared<autoware_adapi_v1_msgs::srv::ClearRoute::Request>(),
          std::bind(
            &HighPrecisionReverseNode::onClearForwardRouteResponse, this,
            std::placeholders::_1));
        break;
      }
    case HandoffPhase::CHANGE_TO_LOCAL: {
        if (!change_to_local_client_->service_is_ready()) {
          return;
        }
        handoff_phase_ = HandoffPhase::WAIT_CHANGE_TO_LOCAL;
        change_to_local_client_->async_send_request(
          std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>(),
          std::bind(
            &HighPrecisionReverseNode::onChangeToLocalResponse, this, std::placeholders::_1));
        break;
      }
    case HandoffPhase::ENABLE_CONTROL: {
        if (!enable_control_client_->service_is_ready()) {
          return;
        }
        handoff_phase_ = HandoffPhase::WAIT_ENABLE_CONTROL;
        enable_control_client_->async_send_request(
          std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>(),
          std::bind(
            &HighPrecisionReverseNode::onEnableControlResponse, this, std::placeholders::_1));
        break;
      }
    case HandoffPhase::ENGAGE: {
        if (!engage_client_->service_is_ready()) {
          return;
        }
        auto request = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
        request->engage = true;
        handoff_phase_ = HandoffPhase::WAIT_ENGAGE;
        engage_client_->async_send_request(
          request,
          std::bind(&HighPrecisionReverseNode::onEngageResponse, this, std::placeholders::_1));
        break;
      }
    case HandoffPhase::UNPAUSE_REVERSE: {
        if (!set_pause_client_->service_is_ready()) {
          return;
        }
        auto request = std::make_shared<tier4_control_msgs::srv::SetPause::Request>();
        request->pause = false;
        handoff_phase_ = HandoffPhase::WAIT_UNPAUSE_REVERSE;
        set_pause_client_->async_send_request(
          request,
          std::bind(
            &HighPrecisionReverseNode::onUnpauseReverseResponse, this, std::placeholders::_1));
        break;
      }
    case HandoffPhase::PAUSE_FOR_AUTO: {
        if (!set_pause_client_->service_is_ready()) {
          return;
        }
        auto request = std::make_shared<tier4_control_msgs::srv::SetPause::Request>();
        request->pause = true;
        handoff_phase_ = HandoffPhase::WAIT_PAUSE_FOR_AUTO;
        set_pause_client_->async_send_request(
          request,
          std::bind(
            &HighPrecisionReverseNode::onPauseForAutoResponse, this,
            std::placeholders::_1));
        break;
      }
    case HandoffPhase::CHANGE_TO_AUTONOMOUS: {
        if (!change_to_autonomous_client_->service_is_ready()) {
          return;
        }
        handoff_phase_ = HandoffPhase::WAIT_CHANGE_TO_AUTONOMOUS;
        change_to_autonomous_client_->async_send_request(
          std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>(),
          std::bind(
            &HighPrecisionReverseNode::onChangeToAutonomousResponse, this,
            std::placeholders::_1));
        break;
      }
    case HandoffPhase::UNPAUSE_AUTO: {
        if (!set_pause_client_->service_is_ready()) {
          return;
        }
        auto request = std::make_shared<tier4_control_msgs::srv::SetPause::Request>();
        request->pause = false;
        handoff_phase_ = HandoffPhase::WAIT_UNPAUSE_AUTO;
        set_pause_client_->async_send_request(
          request,
          std::bind(&HighPrecisionReverseNode::onUnpauseAutoResponse, this, std::placeholders::_1));
        break;
      }
    default:
      break;
  }
}

void HighPrecisionReverseNode::beginReturnToAutonomous()
{
  if (handoff_phase_ == HandoffPhase::REVERSING) {
    handoff_retry_count_ = 0U;
    handoff_retry_after_ = {};
    handoff_phase_ = HandoffPhase::PAUSE_FOR_AUTO;
  }
}

void HighPrecisionReverseNode::retryModeHandoff(
  HandoffPhase phase, const std::string & reason)
{
  ++handoff_retry_count_;
  if (handoff_retry_count_ >= max_handoff_retries_) {
    failModeHandoff(reason + " (retry limit reached)");
    return;
  }
  handoff_retry_after_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(handoff_retry_interval_));
  handoff_phase_ = phase;
  RCLCPP_WARN(
    get_logger(), "%s; retry %zu/%zu", reason.c_str(), handoff_retry_count_,
    max_handoff_retries_);
}

void HighPrecisionReverseNode::failModeHandoff(const std::string & reason)
{
  handoff_phase_ = HandoffPhase::NONE;
  changeState(State::ABORTED, reason);
  publishStop(false);
  RCLCPP_ERROR(get_logger(), "Automatic mode handoff failed: %s", reason.c_str());
}

void HighPrecisionReverseNode::onPauseForReverseResponse(
  rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future)
{
  const auto response = future.get();
  if (!response->status.success) {
    failModeHandoff("failed to pause command gate: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::CLEAR_FORWARD_ROUTE;
}

void HighPrecisionReverseNode::onClearForwardRouteResponse(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedFuture future)
{
  const auto response = future.get();
  if (
    !response->status.success &&
    response->status.code != autoware_adapi_v1_msgs::msg::ResponseStatus::NO_EFFECT)
  {
    failModeHandoff("failed to clear previous forward route: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::CHANGE_TO_LOCAL;
}

void HighPrecisionReverseNode::onChangeToLocalResponse(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future)
{
  const auto response = future.get();
  if (!response->status.success) {
    retryModeHandoff(
      HandoffPhase::CHANGE_TO_LOCAL,
      "failed to change to LOCAL: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::ENABLE_CONTROL;
}

void HighPrecisionReverseNode::onEnableControlResponse(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future)
{
  const auto response = future.get();
  if (!response->status.success) {
    retryModeHandoff(
      HandoffPhase::ENABLE_CONTROL,
      "failed to enable Autoware control: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::ENGAGE;
}

void HighPrecisionReverseNode::onEngageResponse(
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture future)
{
  const auto response = future.get();
  if (
    response->status.code != tier4_external_api_msgs::msg::ResponseStatus::SUCCESS &&
    response->status.code != tier4_external_api_msgs::msg::ResponseStatus::IGNORED)
  {
    failModeHandoff("failed to engage: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::UNPAUSE_REVERSE;
}

void HighPrecisionReverseNode::onUnpauseReverseResponse(
  rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future)
{
  const auto response = future.get();
  if (!response->status.success) {
    failModeHandoff("failed to unpause reverse control: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::REVERSING;
  RCLCPP_INFO(get_logger(), "Automatic reverse control handoff completed");
}

void HighPrecisionReverseNode::onPauseForAutoResponse(
  rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future)
{
  const auto response = future.get();
  if (!response->status.success) {
    failModeHandoff("failed to pause before AUTO handoff: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::CHANGE_TO_AUTONOMOUS;
}

void HighPrecisionReverseNode::onChangeToAutonomousResponse(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future)
{
  const auto response = future.get();
  if (!response->status.success) {
    retryModeHandoff(
      HandoffPhase::CHANGE_TO_AUTONOMOUS,
      "failed to return to AUTONOMOUS: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::UNPAUSE_AUTO;
}

void HighPrecisionReverseNode::onUnpauseAutoResponse(
  rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future)
{
  const auto response = future.get();
  if (!response->status.success) {
    failModeHandoff("failed to unpause AUTONOMOUS control: " + response->status.message);
    return;
  }
  handoff_phase_ = HandoffPhase::NONE;
  has_goal_ = false;
  controller_.clearPath();
  trajectory_.points.clear();
  RCLCPP_INFO(get_logger(), "Returned to AUTONOMOUS; ready for the next forward goal");
}

void HighPrecisionReverseNode::onReplan(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::string reason;
  response->success = plan(reason);
  response->message = reason;
  if (!response->success) {
    changeState(State::ABORTED, reason);
    publishStop(false);
  }
}

void HighPrecisionReverseNode::onCancel(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  has_goal_ = false;
  const bool reverse_control_was_active = handoff_phase_ == HandoffPhase::REVERSING;
  settled_cycles_ = 0U;
  controller_.clearPath();
  trajectory_.points.clear();
  changeState(State::IDLE, "cancelled");
  publishStop(false);
  if (reverse_control_was_active) {
    handoff_phase_ = HandoffPhase::PAUSE_FOR_AUTO;
  } else {
    handoff_phase_ = HandoffPhase::NONE;
  }
  response->success = true;
  response->message = "reverse operation cancelled";
}

bool HighPrecisionReverseNode::plan(std::string & reason)
{
  // Never continue an old trajectory while validating or planning a new request.
  controller_.clearPath();
  trajectory_.points.clear();
  if (!has_goal_) {
    reason = "no goal pose";
    return false;
  }
  if (!odometryIsFresh()) {
    reason = "fresh odometry is required";
    return false;
  }
  if (
    !goal_.header.frame_id.empty() && !odometry_->header.frame_id.empty() &&
    goal_.header.frame_id != odometry_->header.frame_id)
  {
    reason = "goal and odometry frame_id must match";
    return false;
  }

  auto path = generateReversePath();
  if (path.empty()) {
    reason = "no bounded-curvature reverse-only path";
    return false;
  }
  if (!controller_.setPath(std::move(path), &reason)) {
    return false;
  }
  trajectory_ = makeTrajectory();
  trajectory_pub_->publish(trajectory_);
  settled_cycles_ = 0U;
  changeState(State::TRACKING, "new path accepted");
  publishObservability();
  reason = "reverse-only path planned; tracking started";
  return true;
}

std::vector<PathPoint> HighPrecisionReverseNode::generateReversePath() const
{
  const ReedsSheppStateSpace::StateXYT start{
    odometry_->pose.pose.position.x, odometry_->pose.pose.position.y,
    tf2::getYaw(odometry_->pose.pose.orientation)};
  const ReedsSheppStateSpace::StateXYT goal{
    goal_.pose.position.x, goal_.pose.position.y, tf2::getYaw(goal_.pose.orientation)};
  ReedsSheppStateSpace state_space(std::max(0.10, turning_radius_));
  auto rs_path = state_space.reedsShepp(start, goal);
  const double length = rs_path.length();
  if (!std::isfinite(length) || length < min_path_length_ || length > max_path_length_) {
    return {};
  }

  const std::size_t steps = std::max<std::size_t>(
    2U, static_cast<std::size_t>(std::ceil(length / std::max(0.005, path_resolution_))) + 1U);
  std::vector<PathPoint> path;
  path.reserve(steps);
  for (std::size_t i = 0; i < steps; ++i) {
    const auto sample = state_space.interpolate(
      start, rs_path, length * static_cast<double>(i) / static_cast<double>(steps - 1U));
    path.push_back(PathPoint{sample.x, sample.y, normalizeAngle(sample.yaw), 0.0, 0.0});
  }
  return path;
}

autoware_planning_msgs::msg::Trajectory HighPrecisionReverseNode::makeTrajectory() const
{
  autoware_planning_msgs::msg::Trajectory message;
  message.header.stamp = now();
  message.header.frame_id = goal_.header.frame_id;
  for (const auto & point : controller_.path()) {
    autoware_planning_msgs::msg::TrajectoryPoint output;
    output.pose.position.x = point.x;
    output.pose.position.y = point.y;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, point.yaw);
    output.pose.orientation = tf2::toMsg(quaternion);
    const double remaining = controller_.path().back().s - point.s;
    const auto & parameters = controller_.parameters();
    output.longitudinal_velocity_mps = static_cast<float>(-std::min(
        parameters.max_reverse_speed,
        std::sqrt(std::max(0.0, 2.0 * parameters.comfortable_deceleration * remaining))));
    output.front_wheel_angle_rad =
      static_cast<float>(std::atan(parameters.wheel_base * point.curvature));
    message.points.push_back(output);
  }
  if (!message.points.empty()) {
    message.points.back().longitudinal_velocity_mps = 0.0F;
  }
  return message;
}

void HighPrecisionReverseNode::publishObservability(const TrackingResult * result)
{
  const auto stamp = now();
  if (
    (stamp - last_observability_publish_).seconds() <
    1.0 / std::max(0.1, trajectory_publish_rate_))
  {
    return;
  }
  last_observability_publish_ = stamp;
  if (!trajectory_.points.empty()) {
    trajectory_.header.stamp = stamp;
    trajectory_pub_->publish(trajectory_);
  }

  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker line;
  line.header.stamp = stamp;
  line.header.frame_id = goal_.header.frame_id;
  line.ns = "high_precision_reverse_path";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.035;
  line.color.r = 0.1F;
  line.color.g = 0.8F;
  line.color.b = 1.0F;
  line.color.a = 1.0F;
  for (const auto & point : controller_.path()) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0.08;
    line.points.push_back(p);
  }
  array.markers.push_back(line);
  markers_pub_->publish(array);

  if (result) {
    std_msgs::msg::Float64MultiArray debug;
    // [progress_s, remaining_s, lateral_error, heading_error, goal_distance,
    //  goal_yaw_error, steering_command, target_speed]
    debug.data = {result->projected_s, result->remaining_s, result->lateral_error,
      result->heading_error, result->goal_distance, result->goal_yaw_error,
      result->steering_angle, result->target_speed};
    debug_pub_->publish(debug);
  }
}

void HighPrecisionReverseNode::publishCommand(double steering, double acceleration, double speed)
{
  last_steering_command_ = steering;
  autoware_control_msgs::msg::Control command;
  command.stamp = now();
  command.lateral.steering_tire_angle = static_cast<float>(steering);
  command.lateral.steering_tire_rotation_rate = 0.0F;
  command.longitudinal.velocity = static_cast<float>(std::max(0.0, speed));
  command.longitudinal.acceleration = static_cast<float>(acceleration);
  command.longitudinal.jerk = 0.0F;
  control_pub_->publish(command);
}

void HighPrecisionReverseNode::publishGear(bool reverse)
{
  autoware_vehicle_msgs::msg::GearCommand command;
  command.stamp = now();
  command.command = reverse ? autoware_vehicle_msgs::msg::GearCommand::REVERSE :
    autoware_vehicle_msgs::msg::GearCommand::PARK;
  gear_pub_->publish(command);
}

void HighPrecisionReverseNode::publishSignals(bool reversing, bool stopped)
{
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand indicators;
  indicators.stamp = now();
  indicators.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;
  indicators_pub_->publish(indicators);
  autoware_vehicle_msgs::msg::HazardLightsCommand hazards;
  hazards.stamp = now();
  hazards.command = (reversing || stopped) ?
    autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE :
    autoware_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
  hazards_pub_->publish(hazards);
}

void HighPrecisionReverseNode::publishStop(bool keep_reverse_gear)
{
  const double steering = keep_reverse_gear ? last_steering_command_ : 0.0;
  publishCommand(steering, -controller_.parameters().max_deceleration, 0.0);
  publishGear(keep_reverse_gear);
  publishSignals(keep_reverse_gear, true);
}

void HighPrecisionReverseNode::changeState(State state, const std::string & reason)
{
  if (state_ != state) {
    RCLCPP_INFO(
      get_logger(), "state %u -> %u: %s", static_cast<unsigned>(state_),
      static_cast<unsigned>(state), reason.c_str());
  }
  state_ = state;
  std_msgs::msg::UInt8 message;
  message.data = static_cast<std::uint8_t>(state_);
  state_pub_->publish(message);
}

bool HighPrecisionReverseNode::odometryIsFresh() const
{
  return odometry_ && (now() - last_odometry_receipt_).seconds() <= odometry_timeout_;
}

double HighPrecisionReverseNode::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace agv_high_precision_reverse_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_high_precision_reverse_controller::HighPrecisionReverseNode>());
  rclcpp::shutdown();
  return 0;
}
