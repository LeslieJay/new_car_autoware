// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "reverse_parking_controller/reverse_parking_controller_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace reverse_parking_controller
{

ReverseParkingControllerNode::ReverseParkingControllerNode(const rclcpp::NodeOptions & options)
: Node("reverse_parking_controller", options)
{
  // ==================== 声明参数 ====================
  // 控制频率
  control_rate_ = declare_parameter<double>("control_rate", 30.0);

  // Pure Pursuit 参数
  lookahead_distance_ = declare_parameter<double>("lookahead_distance", 1.0);
  min_lookahead_distance_ = declare_parameter<double>("min_lookahead_distance", 0.5);
  lookahead_ratio_ = declare_parameter<double>("lookahead_ratio", 2.0);
  wheel_base_ = declare_parameter<double>("wheel_base", 1.0);

  // PID 纵向控制参数
  kp_ = declare_parameter<double>("pid.kp", 1.0);
  ki_ = declare_parameter<double>("pid.ki", 0.1);
  kd_ = declare_parameter<double>("pid.kd", 0.05);
  max_acceleration_ = declare_parameter<double>("max_acceleration", 1.0);
  max_deceleration_ = declare_parameter<double>("max_deceleration", -2.0);
  pid_integral_max_ = declare_parameter<double>("pid.integral_max", 5.0);

  // 到达判定参数
  goal_distance_threshold_ = declare_parameter<double>("goal_distance_threshold", 0.3);
  goal_yaw_threshold_ = declare_parameter<double>("goal_yaw_threshold", 0.1);
  stop_velocity_threshold_ = declare_parameter<double>("stop_velocity_threshold", 0.05);

  // 转向限幅
  max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.6);

  // AGV充电对接优化参数
  reverse_lookahead_distance_ = declare_parameter<double>("reverse_lookahead_distance", 0.5);
  stanley_gain_ = declare_parameter<double>("stanley_gain", 1.5);
  final_approach_distance_ = declare_parameter<double>("final_approach_distance", 1.0);
  final_approach_speed_ = declare_parameter<double>("final_approach_speed", 0.1);

  // ==================== TF ====================
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ==================== 订阅者 ====================
  // 订阅倒车规划器输出的轨迹
  traj_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", 1,
    std::bind(&ReverseParkingControllerNode::onTrajectory, this, std::placeholders::_1));

  // 订阅里程计
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&ReverseParkingControllerNode::onOdometry, this, std::placeholders::_1));

  // ==================== 发布者（输出到 vehicle_cmd_gate 的 input/external 话题） ====================
  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", 1);

  gear_cmd_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
    "~/output/gear_cmd", 1);

  turn_indicator_cmd_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "~/output/turn_indicators_cmd", 1);

  hazard_light_cmd_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard_lights_cmd", 1);

  // ==================== 调试发布者 ====================
  debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/markers", 1);

  // ==================== 定时器 ====================
  prev_time_ = now();
  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ReverseParkingControllerNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "Reverse Parking Controller initialized");
  RCLCPP_INFO(get_logger(), "  - Control rate: %.1f Hz", control_rate_);
  RCLCPP_INFO(get_logger(), "  - Lookahead distance: %.2f m", lookahead_distance_);
  RCLCPP_INFO(get_logger(), "  - Wheel base: %.2f m", wheel_base_);
  RCLCPP_INFO(get_logger(), "  - PID gains: kp=%.2f, ki=%.2f, kd=%.2f", kp_, ki_, kd_);
  RCLCPP_INFO(get_logger(), "  - Reverse lookahead: %.2f m", reverse_lookahead_distance_);
  RCLCPP_INFO(get_logger(), "  - Stanley gain: %.2f", stanley_gain_);
  RCLCPP_INFO(get_logger(), "  - Final approach: dist=%.2f m, speed=%.2f m/s",
              final_approach_distance_, final_approach_speed_);
}

// ============================================================================
// 回调函数
// ============================================================================

void ReverseParkingControllerNode::onTimer()
{
  // 检查数据是否就绪
  if (!current_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }

  if (!current_trajectory_ || current_trajectory_->points.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for trajectory...");
    // 没有轨迹时，发布停车指令
    publishControlCmd(0.0, max_deceleration_, 0.0);
    publishGearCmd(false);
    publishIndicatorCmds(false, true);
    return;
  }

  const auto & current_pose = current_odom_->pose.pose;
  const double current_velocity = current_odom_->twist.twist.linear.x;

  // 检查是否到达终点
  if (isGoalReached(current_pose, *current_trajectory_)) {
    if (!is_goal_reached_) {
      RCLCPP_INFO(get_logger(), "Goal reached! Stopping.");
      is_goal_reached_ = true;
    }
    // 发布停车指令
    publishControlCmd(0.0, max_deceleration_, 0.0);
    publishGearCmd(false);
    publishIndicatorCmds(false, true);
    return;
  }

  is_goal_reached_ = false;
  is_active_ = true;

  // 找到最近轨迹点
  const size_t nearest_idx = findNearestIndex(current_pose, *current_trajectory_);
  const auto & nearest_pt = current_trajectory_->points[nearest_idx];

  // 判断当前轨迹段是否为倒车（速度为负）
  const bool is_reverse = (nearest_pt.longitudinal_velocity_mps < 0.0);
  double target_velocity = nearest_pt.longitudinal_velocity_mps;

  // ========= 档位切换安全逻辑 =========
  // 切换方向前必须先停车，避免变速箱冲击
  if (is_reverse != prev_is_reverse_ && std::abs(current_velocity) > stop_velocity_threshold_) {
    publishControlCmd(0.0, max_deceleration_, 0.0);
    publishGearCmd(prev_is_reverse_);
    publishIndicatorCmds(prev_is_reverse_, true);
    return;
  }
  prev_is_reverse_ = is_reverse;

  // ========= 基于到终点距离的速度调制 =========
  // 在最终接近阶段逐步降速，确保安全精确对接
  const double distance_to_goal = calcDistanceToGoal(current_pose);
  if (distance_to_goal < final_approach_distance_) {
    double approach_ratio = distance_to_goal / final_approach_distance_;
    double max_speed = final_approach_speed_ +
      approach_ratio * (std::abs(target_velocity) - final_approach_speed_);
    max_speed = std::max(max_speed, final_approach_speed_);
    if (std::abs(target_velocity) > max_speed) {
      target_velocity = is_reverse ? -max_speed : max_speed;
    }
  }

  // ========= 自适应前视距离 =========
  // 倒车时使用更短的前视距离以提高跟踪精度
  const double base_lookahead = is_reverse ? reverse_lookahead_distance_ : lookahead_distance_;
  const double adaptive_lookahead =
    std::max(min_lookahead_distance_,
             lookahead_ratio_ * std::abs(current_velocity));
  const double effective_lookahead = std::max(adaptive_lookahead, base_lookahead);

  // 找到前视目标点
  const size_t lookahead_idx =
    findLookaheadIndex(current_pose, *current_trajectory_, nearest_idx, effective_lookahead);

  // ---- 横向控制：Pure Pursuit 计算转向角 ----
  double steering_angle =
    calcSteeringAngle(current_pose, *current_trajectory_, lookahead_idx, is_reverse);

  // ========= 最终接近横向误差修正（Stanley风格）=========
  // 在接近充电位时叠加横向偏移修正，提高对接精度
  if (distance_to_goal < final_approach_distance_) {
    const double path_yaw = tf2::getYaw(nearest_pt.pose.orientation);
    const double dx_cte = current_pose.position.x - nearest_pt.pose.position.x;
    const double dy_cte = current_pose.position.y - nearest_pt.pose.position.y;
    // 横向误差：车辆偏离路径的垂直距离
    double cte = -std::sin(path_yaw) * dx_cte + std::cos(path_yaw) * dy_cte;
    double cte_correction = -stanley_gain_ * cte;
    if (is_reverse) {
      cte_correction = -cte_correction;
    }
    steering_angle += cte_correction;
    steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
  }

  // ---- 纵向控制：PID 计算加速度 ----
  const double acceleration = calcAcceleration(target_velocity, current_velocity);

  // ---- 发布控制指令 ----
  publishControlCmd(steering_angle, acceleration, target_velocity);
  publishGearCmd(is_reverse);
  publishIndicatorCmds(is_reverse, std::abs(current_velocity) < stop_velocity_threshold_);

  // ---- 发布调试可视化 ----
  publishDebugMarkers(current_pose, nearest_idx, lookahead_idx);
}

void ReverseParkingControllerNode::onTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  current_trajectory_ = msg;
  is_goal_reached_ = false;

  // 重置PID状态
  pid_error_integral_ = 0.0;
  pid_prev_error_ = 0.0;

  RCLCPP_INFO(get_logger(), "Received trajectory with %zu points", msg->points.size());
}

void ReverseParkingControllerNode::onOdometry(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odom_ = msg;
}

// ============================================================================
// 控制算法
// ============================================================================

double ReverseParkingControllerNode::calcSteeringAngle(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  size_t lookahead_idx, bool is_reverse) const
{
  const auto & target_pt = trajectory.points[lookahead_idx];

  // 计算目标点在车体坐标系下的位置
  const double dx = target_pt.pose.position.x - current_pose.position.x;
  const double dy = target_pt.pose.position.y - current_pose.position.y;
  const double current_yaw = tf2::getYaw(current_pose.orientation);

  // 转换到车体坐标系
  const double local_x = std::cos(-current_yaw) * dx - std::sin(-current_yaw) * dy;
  const double local_y = std::sin(-current_yaw) * dx + std::cos(-current_yaw) * dy;

  // 计算到目标点的距离
  const double distance = std::sqrt(local_x * local_x + local_y * local_y);
  if (distance < 1e-6) {
    return 0.0;
  }

  // Pure Pursuit 公式: delta = atan(2 * L * sin(alpha) / Ld)
  // 其中 alpha 是目标方向与车头方向的夹角
  double alpha;
  if (is_reverse) {
    // 倒车时，参考后轴方向（反向）
    alpha = std::atan2(-local_y, -local_x);
  } else {
    alpha = std::atan2(local_y, local_x);
  }

  double steering = std::atan2(2.0 * wheel_base_ * std::sin(alpha), distance);

  // 倒车时转向角反向
  if (is_reverse) {
    steering = -steering;
  }

  // 限幅
  steering = std::clamp(steering, -max_steering_angle_, max_steering_angle_);

  return steering;
}

double ReverseParkingControllerNode::calcAcceleration(
  double target_velocity, double current_velocity)
{
  const rclcpp::Time current_time = now();
  const double dt = (current_time - prev_time_).seconds();
  prev_time_ = current_time;

  if (dt <= 0.0 || dt > 1.0) {
    return 0.0;
  }

  // 速度误差（考虑倒车时的方向）
  const double error = target_velocity - current_velocity;

  // PID 计算
  pid_error_integral_ += error * dt;
  pid_error_integral_ = std::clamp(pid_error_integral_, -pid_integral_max_, pid_integral_max_);

  const double error_derivative = (error - pid_prev_error_) / dt;
  pid_prev_error_ = error;

  double acceleration = kp_ * error + ki_ * pid_error_integral_ + kd_ * error_derivative;

  // 限幅
  acceleration = std::clamp(acceleration, max_deceleration_, max_acceleration_);

  return acceleration;
}

size_t ReverseParkingControllerNode::findNearestIndex(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double dx = trajectory.points[i].pose.position.x - current_pose.position.x;
    const double dy = trajectory.points[i].pose.position.y - current_pose.position.y;
    const double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }

  return nearest_idx;
}

size_t ReverseParkingControllerNode::findLookaheadIndex(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  size_t nearest_idx, double lookahead_distance) const
{
  double accumulated_dist = 0.0;
  size_t lookahead_idx = nearest_idx;

  for (size_t i = nearest_idx; i + 1 < trajectory.points.size(); ++i) {
    const double dx =
      trajectory.points[i + 1].pose.position.x - trajectory.points[i].pose.position.x;
    const double dy =
      trajectory.points[i + 1].pose.position.y - trajectory.points[i].pose.position.y;
    accumulated_dist += std::sqrt(dx * dx + dy * dy);
    lookahead_idx = i + 1;

    if (accumulated_dist >= lookahead_distance) {
      break;
    }
  }

  // 如果前视距离未达到，检查直线距离确保目标点足够远
  const double dx = trajectory.points[lookahead_idx].pose.position.x - current_pose.position.x;
  const double dy = trajectory.points[lookahead_idx].pose.position.y - current_pose.position.y;
  const double direct_dist = std::sqrt(dx * dx + dy * dy);

  // 如果直线距离太近而且不是终点，尝试更远的点
  if (direct_dist < min_lookahead_distance_ && lookahead_idx + 1 < trajectory.points.size()) {
    lookahead_idx = std::min(lookahead_idx + 1, trajectory.points.size() - 1);
  }

  return lookahead_idx;
}

bool ReverseParkingControllerNode::isGoalReached(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory) const
{
  if (trajectory.points.empty()) {
    return false;
  }

  const auto & goal = trajectory.points.back();
  const double dx = goal.pose.position.x - current_pose.position.x;
  const double dy = goal.pose.position.y - current_pose.position.y;
  const double distance = std::sqrt(dx * dx + dy * dy);

  const double current_yaw = tf2::getYaw(current_pose.orientation);
  const double goal_yaw = tf2::getYaw(goal.pose.orientation);
  const double yaw_diff = std::abs(normalizeAngle(current_yaw - goal_yaw));

  return (distance < goal_distance_threshold_ && yaw_diff < goal_yaw_threshold_);
}

// ============================================================================
// 发布函数
// ============================================================================

void ReverseParkingControllerNode::publishControlCmd(
  double steering_angle, double acceleration, double target_velocity)
{
  autoware_control_msgs::msg::Control cmd;
  cmd.stamp = now();

  // 横向控制
  cmd.lateral.steering_tire_angle = static_cast<float>(steering_angle);
  cmd.lateral.steering_tire_rotation_rate = 0.0f;

  // 纵向控制
  cmd.longitudinal.velocity = static_cast<float>(std::abs(target_velocity));
  cmd.longitudinal.acceleration = static_cast<float>(acceleration);
  cmd.longitudinal.jerk = 0.0f;

  control_cmd_pub_->publish(cmd);
}

void ReverseParkingControllerNode::publishGearCmd(bool is_reverse)
{
  autoware_vehicle_msgs::msg::GearCommand gear_cmd;
  gear_cmd.stamp = now();

  if (is_goal_reached_) {
    gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::PARK;
  } else if (is_reverse) {
    gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::REVERSE;
  } else {
    gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
  }

  gear_cmd_pub_->publish(gear_cmd);
}

void ReverseParkingControllerNode::publishIndicatorCmds(bool is_reverse, bool is_stopped)
{
  // 转向灯：倒车时无需开转向灯（根据实际需求可调整）
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand turn_cmd;
  turn_cmd.stamp = now();
  turn_cmd.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;
  turn_indicator_cmd_pub_->publish(turn_cmd);

  // 危险灯：倒车过程中开启危险警示灯
  autoware_vehicle_msgs::msg::HazardLightsCommand hazard_cmd;
  hazard_cmd.stamp = now();
  if (is_reverse || is_stopped) {
    hazard_cmd.command = autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
  } else {
    hazard_cmd.command = autoware_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
  }
  hazard_light_cmd_pub_->publish(hazard_cmd);
}

void ReverseParkingControllerNode::publishDebugMarkers(
  const geometry_msgs::msg::Pose & current_pose,
  size_t nearest_idx, size_t lookahead_idx)
{
  if (!current_trajectory_ || current_trajectory_->points.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  // 最近点标记
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "nearest_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = current_trajectory_->points[nearest_idx].pose;
    marker.pose.position.z += 0.5;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.markers.push_back(marker);
  }

  // 前视目标点标记
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "lookahead_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = current_trajectory_->points[lookahead_idx].pose;
    marker.pose.position.z += 0.5;
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.markers.push_back(marker);
  }

  // 车辆到前视点的连线
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.ns = "lookahead_line";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.8;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    geometry_msgs::msg::Point p1;
    p1.x = current_pose.position.x;
    p1.y = current_pose.position.y;
    p1.z = current_pose.position.z + 0.3;
    marker.points.push_back(p1);

    geometry_msgs::msg::Point p2;
    p2.x = current_trajectory_->points[lookahead_idx].pose.position.x;
    p2.y = current_trajectory_->points[lookahead_idx].pose.position.y;
    p2.z = current_trajectory_->points[lookahead_idx].pose.position.z + 0.3;
    marker.points.push_back(p2);

    markers.markers.push_back(marker);
  }

  debug_marker_pub_->publish(markers);
}

double ReverseParkingControllerNode::calcDistanceToGoal(
  const geometry_msgs::msg::Pose & current_pose) const
{
  if (!current_trajectory_ || current_trajectory_->points.empty()) {
    return std::numeric_limits<double>::max();
  }
  const auto & goal = current_trajectory_->points.back();
  const double dx = goal.pose.position.x - current_pose.position.x;
  const double dy = goal.pose.position.y - current_pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ReverseParkingControllerNode::normalizeAngle(double angle) const
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace reverse_parking_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(reverse_parking_controller::ReverseParkingControllerNode)
