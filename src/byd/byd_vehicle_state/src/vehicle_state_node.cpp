// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "byd_vehicle_state/vehicle_state_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace byd_vehicle_state
{
namespace
{
double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

VehicleStateNode::VehicleStateNode(const rclcpp::NodeOptions & options)
: Node("byd_vehicle_state", options)
{
  update_rate_ = declare_parameter<double>("update_rate", 10.0);
  arrive_distance_th_ = declare_parameter<double>("arrive_distance_threshold", 1.0);
  arrive_yaw_th_ = declare_parameter<double>("arrive_yaw_threshold", 1.0472);
  arrive_speed_th_ = declare_parameter<double>("arrive_speed_threshold", 0.05);
  arrive_hold_time_ = declare_parameter<double>("arrive_hold_time", 1.0);
  arrived_to_unset_timeout_ = declare_parameter<double>("arrived_to_unset_timeout", 2.0);
  use_route_state_for_forward_ = declare_parameter<bool>("use_route_state_for_forward", true);

  // 前进 goal 发布端(rviz / autoware_auto_server)为 VOLATILE，必须匹配，否则收不到
  const auto forward_goal_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  // 倒车 goal 由 reverse_parking_planner 以 transient_local 发布
  const auto reverse_goal_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

  forward_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/input/forward_goal", forward_goal_qos,
    std::bind(&VehicleStateNode::onForwardGoal, this, std::placeholders::_1));

  reverse_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/input/reverse_goal", reverse_goal_qos,
    std::bind(&VehicleStateNode::onReverseGoal, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 10,
    std::bind(&VehicleStateNode::onOdometry, this, std::placeholders::_1));

  if (use_route_state_for_forward_) {
    route_state_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
      "~/input/route_state", rclcpp::QoS(1).transient_local(),
      std::bind(&VehicleStateNode::onRouteState, this, std::placeholders::_1));
  }

  operation_mode_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "~/input/operation_mode", rclcpp::QoS(1).transient_local(),
    std::bind(&VehicleStateNode::onOperationMode, this, std::placeholders::_1));

  // 从 LaneletRoute 取真实 goal_pose，用于距离到达判定（AD API 设点时 PoseStamped 可能缺失）
  lanelet_route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/lanelet_route", rclcpp::QoS(1).transient_local(),
    std::bind(&VehicleStateNode::onLaneletRoute, this, std::placeholders::_1));

  state_pub_ = create_publisher<autoware_system_msgs::msg::AutowareState>("~/output/state", 1);
  state_name_pub_ = create_publisher<std_msgs::msg::String>("~/output/state_name", 1);

  const auto period = std::chrono::duration<double>(1.0 / std::max(update_rate_, 1.0));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&VehicleStateNode::onTimer, this));

  RCLCPP_INFO(
    get_logger(),
    "byd_vehicle_state ready: dist=%.3f yaw=%.3f speed=%.3f hold=%.1fs",
    arrive_distance_th_, arrive_yaw_th_, arrive_speed_th_, arrive_hold_time_);
}

void VehicleStateNode::onForwardGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  setActiveGoal(*msg, GoalMode::Forward);
}

void VehicleStateNode::onReverseGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  setActiveGoal(*msg, GoalMode::Reverse);
}

void VehicleStateNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odom_ = msg;
}

void VehicleStateNode::onRouteState(
  const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg)
{
  const uint8_t prev = route_state_;
  route_state_ = msg->state;

  // AD API / SetRoutePoints 设点时不一定发 PoseStamped goal；用路由 SET 进入前进任务
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  if (
    route_state_ == RouteState::SET && prev != RouteState::SET &&
    goal_mode_ == GoalMode::None)
  {
    goal_mode_ = GoalMode::Forward;
    arrived_condition_met_ = false;
    arrived_since_.reset();
    arrived_published_since_.reset();
    if (!active_goal_) {
      geometry_msgs::msg::PoseStamped placeholder;
      placeholder.header.stamp = now();
      placeholder.header.frame_id = "map";
      active_goal_ = placeholder;
      has_valid_goal_pose_ = false;
    }
    RCLCPP_INFO(get_logger(), "Route SET -> enter FORWARD mission (via /api/routing/state)");
  }
}

void VehicleStateNode::onOperationMode(
  const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg)
{
  operation_mode_ = msg->mode;
  is_autoware_control_enabled_ = msg->is_autoware_control_enabled;
  operation_mode_received_ = true;
}

void VehicleStateNode::onLaneletRoute(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  if (goal_mode_ == GoalMode::Reverse) {
    return;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header = msg->header;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = "map";
  }
  goal.pose = msg->goal_pose;

  const bool first = !has_valid_goal_pose_;
  active_goal_ = goal;
  has_valid_goal_pose_ = true;
  goal_mode_ = GoalMode::Forward;
  if (first) {
    arrived_condition_met_ = false;
    arrived_since_.reset();
    arrived_published_since_.reset();
    RCLCPP_INFO(
      get_logger(), "Route goal pose: (%.2f, %.2f) yaw=%.2f deg",
      goal.pose.position.x, goal.pose.position.y,
      tf2::getYaw(goal.pose.orientation) * 180.0 / M_PI);
  }
}

void VehicleStateNode::setActiveGoal(
  const geometry_msgs::msg::PoseStamped & goal, GoalMode mode)
{
  active_goal_ = goal;
  has_valid_goal_pose_ = true;
  goal_mode_ = mode;
  arrived_condition_met_ = false;
  arrived_since_.reset();
  arrived_published_since_.reset();

  const char * mode_str = (mode == GoalMode::Forward) ? "FORWARD" : "REVERSE";
  RCLCPP_INFO(
    get_logger(), "Active goal [%s]: (%.2f, %.2f) yaw=%.2f deg",
    mode_str, goal.pose.position.x, goal.pose.position.y,
    tf2::getYaw(goal.pose.orientation) * 180.0 / M_PI);
}

bool VehicleStateNode::hasActiveMission() const
{
  using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
  if (goal_mode_ == GoalMode::Reverse && active_goal_.has_value()) {
    return true;
  }
  if (
    use_route_state_for_forward_ &&
    (route_state_ == RouteState::SET || route_state_ == RouteState::CHANGING))
  {
    return true;
  }
  return active_goal_.has_value() && goal_mode_ != GoalMode::None;
}

bool VehicleStateNode::isDrivingEngaged() const
{
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  // 对齐 Autoware：只有非 STOP 且 Autoware 控车时才算 Driving
  if (!operation_mode_received_) {
    return false;
  }
  return operation_mode_ != OperationModeState::STOP && is_autoware_control_enabled_;
}

bool VehicleStateNode::isArrivedAtGoal() const
{
  if (!current_odom_) {
    return false;
  }

  // Autoware 官方路由到达信号（阈值很严，横向常要 ≤2cm）
  if (
    use_route_state_for_forward_ && goal_mode_ != GoalMode::Reverse &&
    route_state_ == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED)
  {
    return true;
  }

  if (!active_goal_ || !has_valid_goal_pose_) {
    return false;
  }

  const auto & pose = current_odom_->pose.pose;
  const double dx = pose.position.x - active_goal_->pose.position.x;
  const double dy = pose.position.y - active_goal_->pose.position.y;
  const double dist = std::hypot(dx, dy);
  const double yaw_diff = std::abs(
    normalizeAngle(tf2::getYaw(pose.orientation) - tf2::getYaw(active_goal_->pose.orientation)));
  const double speed = std::hypot(
    current_odom_->twist.twist.linear.x, current_odom_->twist.twist.linear.y);

  return dist < arrive_distance_th_ && yaw_diff < arrive_yaw_th_ && speed < arrive_speed_th_;
}

uint8_t VehicleStateNode::computeState() const
{
  using AutowareState = autoware_system_msgs::msg::AutowareState;

  if (!current_odom_) {
    return AutowareState::INITIALIZING;
  }
  if (arrived_published_since_.has_value()) {
    return AutowareState::ARRIVED_GOAL;
  }
  if (!hasActiveMission()) {
    return AutowareState::WAITING_FOR_ROUTE;
  }
  // 有任务但还在 STOP / 未 engage：WaitingForEngage（速度为 0 是正常的）
  if (!isDrivingEngaged()) {
    return AutowareState::WAITING_FOR_ENGAGE;
  }
  return AutowareState::DRIVING;
}

std::string VehicleStateNode::stateToString(uint8_t state)
{
  using AutowareState = autoware_system_msgs::msg::AutowareState;
  switch (state) {
    case AutowareState::INITIALIZING:
      return "Initializing";
    case AutowareState::WAITING_FOR_ROUTE:
      return "WaitingForRoute";
    case AutowareState::PLANNING:
      return "Planning";
    case AutowareState::WAITING_FOR_ENGAGE:
      return "WaitingForEngage";
    case AutowareState::DRIVING:
      return "Driving";
    case AutowareState::ARRIVED_GOAL:
      return "ArrivedGoal";
    case AutowareState::FINALIZING:
      return "Finalizing";
    default:
      return "Unknown(" + std::to_string(state) + ")";
  }
}

void VehicleStateNode::onTimer()
{
  const auto now_t = now();

  if (hasActiveMission() && isArrivedAtGoal()) {
    if (!arrived_condition_met_) {
      arrived_condition_met_ = true;
      arrived_since_ = now_t;
      RCLCPP_INFO(get_logger(), "Arrival condition met, starting hold timer (%.1fs)", arrive_hold_time_);
    } else if (
      arrived_since_.has_value() &&
      (now_t - arrived_since_.value()).seconds() >= arrive_hold_time_ &&
      !arrived_published_since_.has_value())
    {
      arrived_published_since_ = now_t;
      RCLCPP_INFO(get_logger(), "Goal arrived (mode=%u)", static_cast<unsigned>(goal_mode_));
    }
  } else if (hasActiveMission() && !arrived_published_since_.has_value()) {
    // 到达条件丢失且尚未对外宣布到达，重置保持计时
    if (arrived_condition_met_) {
      RCLCPP_WARN(get_logger(), "Arrival condition lost, resetting hold timer");
    }
    arrived_condition_met_ = false;
    arrived_since_.reset();
  }

  // 到达后保持一段时间，再清 goal，回到 WaitingForRoute（对齐 Autoware 行为）
  if (
    arrived_published_since_.has_value() &&
    (now_t - arrived_published_since_.value()).seconds() >= arrived_to_unset_timeout_)
  {
    RCLCPP_INFO(get_logger(), "Clear active goal after ARRIVED timeout");
    active_goal_.reset();
    has_valid_goal_pose_ = false;
    goal_mode_ = GoalMode::None;
    arrived_condition_met_ = false;
    arrived_since_.reset();
    arrived_published_since_.reset();
  }

  const uint8_t state = computeState();
  if (state != prev_state_) {
    RCLCPP_INFO(
      get_logger(), "VehicleState: %s => %s",
      stateToString(prev_state_).c_str(), stateToString(state).c_str());
    prev_state_ = state;
  }

  autoware_system_msgs::msg::AutowareState state_msg;
  state_msg.stamp = now_t;
  state_msg.state = state;
  state_pub_->publish(state_msg);

  std_msgs::msg::String name_msg;
  name_msg.data = stateToString(state);
  state_name_pub_->publish(name_msg);
}

}  // namespace byd_vehicle_state

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(byd_vehicle_state::VehicleStateNode)
