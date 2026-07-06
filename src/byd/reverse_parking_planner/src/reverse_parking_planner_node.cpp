// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "reverse_parking_planner/reverse_parking_planner_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace reverse_parking_planner
{

ReverseParkingPlannerNode::ReverseParkingPlannerNode(const rclcpp::NodeOptions & options)
: Node("reverse_parking_planner", options)
{
  // 规划参数
  wheel_base_ = declare_parameter<double>("wheel_base", 1.0);
  path_resolution_ = declare_parameter<double>("path_resolution", 0.1);
  velocity_reverse_ = declare_parameter<double>("velocity_reverse", -0.3);
  publish_rate_ = declare_parameter<double>("publish_rate", 10.0);  // Trajectory/Marker 发布频率
  velocity_creep_ = declare_parameter<double>("velocity_creep", 0.1);
  decel_distance_ = declare_parameter<double>("decel_distance", 1.5);
  max_straight_lateral_error_ = declare_parameter<double>("max_straight_lateral_error", 0.3);
  max_straight_yaw_error_ = declare_parameter<double>("max_straight_yaw_error", 0.35);
  min_reverse_distance_ = declare_parameter<double>("min_reverse_distance", 0.2);
  max_reverse_driving_distance_ = declare_parameter<double>("max_reverse_driving_distance", 10.0);

  // 控制参数
  control_rate_ = declare_parameter<double>("control_rate", 30.0);
  lookahead_distance_ = declare_parameter<double>("lookahead_distance", 1.0);
  min_lookahead_distance_ = declare_parameter<double>("min_lookahead_distance", 0.5);
  lookahead_ratio_ = declare_parameter<double>("lookahead_ratio", 2.0);
  kp_ = declare_parameter<double>("pid.kp", 1.0);
  ki_ = declare_parameter<double>("pid.ki", 0.1);
  kd_ = declare_parameter<double>("pid.kd", 0.05);
  max_acceleration_ = declare_parameter<double>("max_acceleration", 1.0);
  max_deceleration_ = declare_parameter<double>("max_deceleration", -2.0);
  pid_integral_max_ = declare_parameter<double>("pid.integral_max", 5.0);
  goal_distance_threshold_ = declare_parameter<double>("goal_distance_threshold", 0.3);
  goal_yaw_threshold_ = declare_parameter<double>("goal_yaw_threshold", 0.1);
  stop_velocity_threshold_ = declare_parameter<double>("stop_velocity_threshold", 0.05);
  max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.6);
  reverse_lookahead_distance_ = declare_parameter<double>("reverse_lookahead_distance", 0.5);
  stanley_gain_ = declare_parameter<double>("stanley_gain", 1.5);
  final_approach_distance_ = declare_parameter<double>("final_approach_distance", 1.0);
  final_approach_speed_ = declare_parameter<double>("final_approach_speed", 0.1);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1,
    std::bind(&ReverseParkingPlannerNode::onOdometry, this, std::placeholders::_1));

  traj_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/output/trajectory", 1);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/output/path_markers", 1);
  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", 1);
  gear_cmd_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>(
    "~/output/gear_cmd", 1);
  turn_indicator_cmd_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
    "~/output/turn_indicators_cmd", 1);
  hazard_light_cmd_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>(
    "~/output/hazard_lights_cmd", 1);
  debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/markers", 1);

  set_goal_srv_ = create_service<SetGoalPose>(
    "~/set_goal_pose",
    std::bind(&ReverseParkingPlannerNode::onSetGoalPose, this,
              std::placeholders::_1, std::placeholders::_2));

  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_planning",
    std::bind(&ReverseParkingPlannerNode::onTriggerPlanning, this,
              std::placeholders::_1, std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ReverseParkingPlannerNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "Reverse Parking Unified Node initialized");
  RCLCPP_INFO(get_logger(), "  - Path resolution: %.2f m", path_resolution_);
  RCLCPP_INFO(get_logger(), "  - Reverse velocity: %.2f m/s", velocity_reverse_);
  RCLCPP_INFO(get_logger(), "  - Control rate: %.1f Hz", control_rate_);
  RCLCPP_INFO(get_logger(), "  - Trajectory publish rate: %.1f Hz", publish_rate_);
  if (max_reverse_driving_distance_ > 0.0) {
    RCLCPP_INFO(
      get_logger(), "  - Max reverse driving distance: %.2f m", max_reverse_driving_distance_);
  } else {
    RCLCPP_INFO(get_logger(), "  - Max reverse driving distance: disabled");
  }
}

void ReverseParkingPlannerNode::onTimer()
{
  if (!current_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }
  if (current_trajectory_ && !current_path_.empty()) {
    const auto current_wall_time = std::chrono::steady_clock::now();
    const double publish_period = (publish_rate_ > 0.0) ? (1.0 / publish_rate_) : 0.0;
    const bool should_publish =
      publish_period <= 0.0 || !last_traj_publish_initialized_ ||
      std::chrono::duration<double>(current_wall_time - last_traj_publish_wall_time_).count() >=
      publish_period;
    if (should_publish) {
      current_trajectory_->header.stamp = now();
      traj_pub_->publish(*current_trajectory_);
      last_traj_publish_wall_time_ = current_wall_time;
      last_traj_publish_initialized_ = true;
    }
    publishVisualization(current_path_);
  }

  runControlLoop();
}

void ReverseParkingPlannerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odom_ = msg;
}

void ReverseParkingPlannerNode::onSetGoalPose(
  const std::shared_ptr<SetGoalPose::Request> request,
  std::shared_ptr<SetGoalPose::Response> response)
{
  goal_pose_ = request->goal_pose;
  has_goal_ = true;
  
  RCLCPP_INFO(get_logger(), "Service: Received goal pose: (%.2f, %.2f, %.2f)",
              goal_pose_.pose.position.x,
              goal_pose_.pose.position.y,
              tf2::getYaw(goal_pose_.pose.orientation));
  
  if (!current_odom_) {
    response->success = false;
    response->message = "No odometry available, goal saved but planning deferred";
    response->path_points_num = 0;
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }
  
  if (planPath()) {
    response->success = true;
    response->message = "Straight reverse path planned successfully";
    response->path_points_num = static_cast<uint32_t>(current_path_.size());
    RCLCPP_INFO(get_logger(), "Path planned successfully with %zu points", current_path_.size());
  } else {
    response->success = false;
    response->message = "Failed to plan straight reverse path to goal";
    response->path_points_num = 0;
    RCLCPP_WARN(get_logger(), "Failed to plan straight reverse path");
  }
}

void ReverseParkingPlannerNode::onTriggerPlanning(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!current_odom_) {
    response->success = false;
    response->message = "No odometry available";
    return;
  }
  
  if (!has_goal_) {
    response->success = false;
    response->message = "No goal pose set";
    return;
  }
  
  if (planPath()) {
    response->success = true;
    response->message = "Path planned successfully with " +
                        std::to_string(current_path_.size()) + " points";
  } else {
    response->success = false;
    response->message = "Failed to plan path";
  }
}

bool ReverseParkingPlannerNode::planPath()
{
  if (!current_odom_ || !has_goal_) {
    return false;
  }

  auto path_points = generateStraightReversePath();
  if (path_points.empty()) {
    current_path_.clear();
    current_trajectory_.reset();
    return false;
  }

  current_path_ = path_points;
  current_trajectory_ = std::make_shared<autoware_planning_msgs::msg::Trajectory>(
    convertToTrajectory(current_path_));
  pid_error_integral_ = 0.0;
  pid_prev_error_ = 0.0;
  prev_control_time_initialized_ = false;
  is_goal_reached_ = false;
  is_distance_limit_reached_ = false;
  reverse_driven_distance_ = 0.0;
  prev_reverse_odom_initialized_ = false;

  RCLCPP_INFO(get_logger(), "Straight reverse path planned with %zu points", current_path_.size());
  return true;
}

std::vector<ReverseParkingPlannerNode::PathPoint> ReverseParkingPlannerNode::generateStraightReversePath() const
{
  const double x0 = current_odom_->pose.pose.position.x;
  const double y0 = current_odom_->pose.pose.position.y;
  const double yaw0 = tf2::getYaw(current_odom_->pose.pose.orientation);
  const double xg = goal_pose_.pose.position.x;
  const double yg = goal_pose_.pose.position.y;
  const double yawg = tf2::getYaw(goal_pose_.pose.orientation);

  const double dx = xg - x0;
  const double dy = yg - y0;
  const double total_dist = std::hypot(dx, dy);
  if (total_dist < min_reverse_distance_) {
    RCLCPP_WARN(get_logger(), "Goal too close for reverse planning (dist=%.3f m)", total_dist);
    return {};
  }

  const double reverse_dir_x = -std::cos(yawg);
  const double reverse_dir_y = -std::sin(yawg);
  const double longitudinal = dx * reverse_dir_x + dy * reverse_dir_y;
  const double lateral = std::abs(-reverse_dir_y * dx + reverse_dir_x * dy);
  const double yaw_error = std::abs(normalizeAngle(yaw0 - yawg));

  if (longitudinal <= 0.0) {
    RCLCPP_WARN(get_logger(), "Straight reverse rejected: goal is not behind vehicle heading");
    return {};
  }
  if (lateral > max_straight_lateral_error_) {
    RCLCPP_WARN(
      get_logger(), "Straight reverse rejected: lateral error %.2f > %.2f",
      lateral, max_straight_lateral_error_);
    return {};
  }
  if (yaw_error > max_straight_yaw_error_) {
    RCLCPP_WARN(
      get_logger(), "Straight reverse rejected: yaw error %.2f rad > %.2f rad",
      yaw_error, max_straight_yaw_error_);
    return {};
  }

  const int steps = std::max(2, static_cast<int>(std::ceil(total_dist / path_resolution_)) + 1);
  const double yaw_delta = normalizeAngle(yawg - yaw0);
  std::vector<PathPoint> path_points;
  path_points.reserve(static_cast<size_t>(steps));

  for (int i = 0; i < steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps - 1);
    PathPoint pt;
    pt.x = x0 + t * dx;
    pt.y = y0 + t * dy;
    pt.yaw = yaw0 + t * yaw_delta;
    pt.is_reverse = true;
    pt.curvature = 0.0;
    path_points.push_back(pt);
  }

  return path_points;
}

autoware_planning_msgs::msg::Trajectory ReverseParkingPlannerNode::convertToTrajectory(
  const std::vector<PathPoint> & path_points) const
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.header.frame_id = "map";
  
  if (path_points.empty()) return trajectory;
  
  const size_t n = path_points.size();
  
  // 预计算累计距离（用于速度规划）
  std::vector<double> cumulative_dist(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    double dx = path_points[i].x - path_points[i-1].x;
    double dy = path_points[i].y - path_points[i-1].y;
    cumulative_dist[i] = cumulative_dist[i-1] + std::sqrt(dx*dx + dy*dy);
  }
  double total_dist = cumulative_dist.back();
  
  for (size_t i = 0; i < n; ++i) {
    const auto & pt = path_points[i];
    autoware_planning_msgs::msg::TrajectoryPoint traj_pt;
    traj_pt.pose.position.x = pt.x;
    traj_pt.pose.position.y = pt.y;
    traj_pt.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, pt.yaw);
    traj_pt.pose.orientation = tf2::toMsg(q);
    
    double base_velocity = pt.is_reverse ? velocity_reverse_ : std::abs(velocity_reverse_);
    double velocity_scale = 1.0;

    double dist_to_end = total_dist - cumulative_dist[i];
    if (dist_to_end < decel_distance_ && decel_distance_ > 0.0) {
      double ratio = dist_to_end / decel_distance_;
      double creep_ratio = (base_velocity != 0.0) ?
        std::abs(velocity_creep_ / base_velocity) : 0.0;
      double goal_scale = creep_ratio + (1.0 - creep_ratio) * std::sqrt(ratio);
      velocity_scale = std::min(velocity_scale, goal_scale);
    }

    if (i == n - 1) {
      velocity_scale = 0.0;
    }
    
    traj_pt.longitudinal_velocity_mps = base_velocity * velocity_scale;
    traj_pt.lateral_velocity_mps = 0.0;
    traj_pt.acceleration_mps2 = 0.0;
    traj_pt.heading_rate_rps = 0.0;
    
    traj_pt.front_wheel_angle_rad = static_cast<float>(
      std::atan(wheel_base_ * pt.curvature));
    
    trajectory.points.push_back(traj_pt);
  }
  
  return trajectory;
}

void ReverseParkingPlannerNode::publishVisualization(const std::vector<PathPoint> & path_points)
{
  auto markers = createPathMarkers(path_points);
  marker_pub_->publish(markers);
}

visualization_msgs::msg::MarkerArray ReverseParkingPlannerNode::createPathMarkers(
  const std::vector<PathPoint> & path_points) const
{
  visualization_msgs::msg::MarkerArray markers;
  
  // 路径线
  visualization_msgs::msg::Marker line_marker;
  line_marker.header.stamp = now();
  line_marker.header.frame_id = "map";
  line_marker.ns = "path_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.05;
  line_marker.color.a = 1.0;
  
  for (const auto & pt : path_points) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.1;
    line_marker.points.push_back(p);
    
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    if (pt.is_reverse) {
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
    } else {
      color.r = 0.0;
      color.g = 1.0;
      color.b = 0.0;
    }
    line_marker.colors.push_back(color);
  }
  
  markers.markers.push_back(line_marker);
  
  int arrow_id = 0;
  for (size_t i = 0; i < path_points.size(); i += 10) {
    const auto & pt = path_points[i];
    
    visualization_msgs::msg::Marker arrow;
    arrow.header.stamp = now();
    arrow.header.frame_id = "map";
    arrow.ns = "path_arrows";
    arrow.id = arrow_id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    
    arrow.pose.position.x = pt.x;
    arrow.pose.position.y = pt.y;
    arrow.pose.position.z = 0.1;
    
    tf2::Quaternion q;
    double display_yaw = pt.is_reverse ? pt.yaw + M_PI : pt.yaw;
    q.setRPY(0, 0, display_yaw);
    arrow.pose.orientation = tf2::toMsg(q);
    
    arrow.scale.x = 0.3;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    
    arrow.color.a = 1.0;
    if (pt.is_reverse) {
      arrow.color.r = 1.0;
      arrow.color.g = 0.3;
      arrow.color.b = 0.0;
    } else {
      arrow.color.r = 0.0;
      arrow.color.g = 0.8;
      arrow.color.b = 0.2;
    }
    
    markers.markers.push_back(arrow);
  }
  
  // 起点标记
  visualization_msgs::msg::Marker start_marker;
  start_marker.header.stamp = now();
  start_marker.header.frame_id = "map";
  start_marker.ns = "start_goal";
  start_marker.id = 0;
  start_marker.type = visualization_msgs::msg::Marker::SPHERE;
  start_marker.action = visualization_msgs::msg::Marker::ADD;
  start_marker.pose.position.x = path_points.front().x;
  start_marker.pose.position.y = path_points.front().y;
  start_marker.pose.position.z = 0.2;
  start_marker.scale.x = 0.3;
  start_marker.scale.y = 0.3;
  start_marker.scale.z = 0.3;
  start_marker.color.r = 0.0;
  start_marker.color.g = 1.0;
  start_marker.color.b = 0.0;
  start_marker.color.a = 1.0;
  markers.markers.push_back(start_marker);
  
  // 终点标记
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.stamp = now();
  goal_marker.header.frame_id = "map";
  goal_marker.ns = "start_goal";
  goal_marker.id = 1;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = path_points.back().x;
  goal_marker.pose.position.y = path_points.back().y;
  goal_marker.pose.position.z = 0.2;
  goal_marker.scale.x = 0.3;
  goal_marker.scale.y = 0.3;
  goal_marker.scale.z = 0.3;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 1.0;
  markers.markers.push_back(goal_marker);
  
  return markers;
}

void ReverseParkingPlannerNode::runControlLoop()
{
  if (!current_odom_ || !current_trajectory_ || current_trajectory_->points.empty()) {
    return;
  }

  const auto & current_pose = current_odom_->pose.pose;

  if (!is_goal_reached_ && !is_distance_limit_reached_) {
    if (prev_reverse_odom_initialized_) {
      const double dx = current_pose.position.x - prev_reverse_odom_x_;
      const double dy = current_pose.position.y - prev_reverse_odom_y_;
      reverse_driven_distance_ += std::hypot(dx, dy);
    }
    prev_reverse_odom_x_ = current_pose.position.x;
    prev_reverse_odom_y_ = current_pose.position.y;
    prev_reverse_odom_initialized_ = true;

    if (max_reverse_driving_distance_ > 0.0 &&
      reverse_driven_distance_ >= max_reverse_driving_distance_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Reverse driving distance limit reached (%.2f m >= %.2f m). Forcing stop.",
        reverse_driven_distance_, max_reverse_driving_distance_);
      is_distance_limit_reached_ = true;
    }
  }

  if (!is_goal_reached_ && !is_distance_limit_reached_ &&
    isGoalReached(current_pose, *current_trajectory_))
  {
    RCLCPP_INFO(get_logger(), "Goal reached! Holding stop command.");
    is_goal_reached_ = true;
  }

  if (is_goal_reached_ || is_distance_limit_reached_) {
    // vehicle_cmd_gate 会锁存最后一次 external control_cmd；必须持续发布 vel=0，
    // 否则 gate 仍输出倒车时的恒定速度，导致终点抖动。
    publishControlCmd(0.0, max_deceleration_, 0.0);
    publishGearCmd(false);
    publishIndicatorCmds(false, true);
    return;
  }

  const double constant_velocity = std::abs(velocity_reverse_);
  publishControlCmd(0.0, 0.0, constant_velocity);
  publishGearCmd(true);
  publishIndicatorCmds(true, false);
}

double ReverseParkingPlannerNode::calcSteeringAngle(
  const geometry_msgs::msg::Pose & current_pose,
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  size_t lookahead_idx, bool is_reverse) const
{
  const auto & target_pt = trajectory.points[lookahead_idx];
  const double dx = target_pt.pose.position.x - current_pose.position.x;
  const double dy = target_pt.pose.position.y - current_pose.position.y;
  const double current_yaw = tf2::getYaw(current_pose.orientation);

  const double local_x = std::cos(-current_yaw) * dx - std::sin(-current_yaw) * dy;
  const double local_y = std::sin(-current_yaw) * dx + std::cos(-current_yaw) * dy;
  const double distance = std::sqrt(local_x * local_x + local_y * local_y);
  if (distance < 1e-6) {
    return 0.0;
  }

  double alpha = is_reverse ? std::atan2(-local_y, -local_x) : std::atan2(local_y, local_x);
  double steering = std::atan2(2.0 * wheel_base_ * std::sin(alpha), distance);
  if (is_reverse) {
    steering = -steering;
  }
  return std::clamp(steering, -max_steering_angle_, max_steering_angle_);
}

double ReverseParkingPlannerNode::calcAcceleration(double target_velocity, double current_velocity)
{
  const auto current_time = std::chrono::steady_clock::now();
  if (!prev_control_time_initialized_) {
    prev_control_time_ = current_time;
    prev_control_time_initialized_ = true;
    return 0.0;
  }

  const double dt =
    std::chrono::duration<double>(current_time - prev_control_time_).count();
  prev_control_time_ = current_time;

  if (dt <= 0.0 || dt > 1.0) {
    return 0.0;
  }

  const double error = target_velocity - current_velocity;
  pid_error_integral_ += error * dt;
  pid_error_integral_ = std::clamp(pid_error_integral_, -pid_integral_max_, pid_integral_max_);
  const double error_derivative = (error - pid_prev_error_) / dt;
  pid_prev_error_ = error;

  double acceleration = kp_ * error + ki_ * pid_error_integral_ + kd_ * error_derivative;
  return std::clamp(acceleration, max_deceleration_, max_acceleration_);
}

size_t ReverseParkingPlannerNode::findNearestIndex(
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

size_t ReverseParkingPlannerNode::findLookaheadIndex(
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

  const double dx = trajectory.points[lookahead_idx].pose.position.x - current_pose.position.x;
  const double dy = trajectory.points[lookahead_idx].pose.position.y - current_pose.position.y;
  const double direct_dist = std::sqrt(dx * dx + dy * dy);
  if (direct_dist < min_lookahead_distance_ && lookahead_idx + 1 < trajectory.points.size()) {
    lookahead_idx = std::min(lookahead_idx + 1, trajectory.points.size() - 1);
  }
  return lookahead_idx;
}

bool ReverseParkingPlannerNode::isGoalReached(
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

void ReverseParkingPlannerNode::publishControlCmd(
  double steering_angle, double acceleration, double target_velocity)
{
  autoware_control_msgs::msg::Control cmd;
  cmd.stamp = now();
  cmd.lateral.steering_tire_angle = static_cast<float>(steering_angle);
  cmd.lateral.steering_tire_rotation_rate = 0.0f;
  cmd.longitudinal.velocity = static_cast<float>(std::abs(target_velocity));
  cmd.longitudinal.acceleration = static_cast<float>(acceleration);
  cmd.longitudinal.jerk = 0.0f;
  control_cmd_pub_->publish(cmd);
}

void ReverseParkingPlannerNode::publishGearCmd(bool is_reverse)
{
  autoware_vehicle_msgs::msg::GearCommand gear_cmd;
  gear_cmd.stamp = now();

  if (is_goal_reached_ || is_distance_limit_reached_) {
    gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::PARK;
  } else if (is_reverse) {
    gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::REVERSE;
  } else {
    gear_cmd.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
  }

  gear_cmd_pub_->publish(gear_cmd);
}

void ReverseParkingPlannerNode::publishIndicatorCmds(bool is_reverse, bool is_stopped)
{
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand turn_cmd;
  turn_cmd.stamp = now();
  turn_cmd.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;
  turn_indicator_cmd_pub_->publish(turn_cmd);

  autoware_vehicle_msgs::msg::HazardLightsCommand hazard_cmd;
  hazard_cmd.stamp = now();
  if (is_reverse || is_stopped) {
    hazard_cmd.command = autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
  } else {
    hazard_cmd.command = autoware_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
  }
  hazard_light_cmd_pub_->publish(hazard_cmd);
}

void ReverseParkingPlannerNode::publishDebugMarkers(
  const geometry_msgs::msg::Pose & current_pose,
  size_t nearest_idx, size_t lookahead_idx)
{
  if (!current_trajectory_ || current_trajectory_->points.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker nearest_marker;
  nearest_marker.header.stamp = now();
  nearest_marker.header.frame_id = "map";
  nearest_marker.ns = "nearest_point";
  nearest_marker.id = 0;
  nearest_marker.type = visualization_msgs::msg::Marker::SPHERE;
  nearest_marker.action = visualization_msgs::msg::Marker::ADD;
  nearest_marker.pose = current_trajectory_->points[nearest_idx].pose;
  nearest_marker.pose.position.z += 0.5;
  nearest_marker.scale.x = 0.3;
  nearest_marker.scale.y = 0.3;
  nearest_marker.scale.z = 0.3;
  nearest_marker.color.r = 1.0;
  nearest_marker.color.g = 1.0;
  nearest_marker.color.b = 0.0;
  nearest_marker.color.a = 1.0;
  nearest_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  markers.markers.push_back(nearest_marker);

  visualization_msgs::msg::Marker lookahead_marker;
  lookahead_marker.header.stamp = now();
  lookahead_marker.header.frame_id = "map";
  lookahead_marker.ns = "lookahead_point";
  lookahead_marker.id = 0;
  lookahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
  lookahead_marker.action = visualization_msgs::msg::Marker::ADD;
  lookahead_marker.pose = current_trajectory_->points[lookahead_idx].pose;
  lookahead_marker.pose.position.z += 0.5;
  lookahead_marker.scale.x = 0.4;
  lookahead_marker.scale.y = 0.4;
  lookahead_marker.scale.z = 0.4;
  lookahead_marker.color.r = 0.0;
  lookahead_marker.color.g = 0.0;
  lookahead_marker.color.b = 1.0;
  lookahead_marker.color.a = 1.0;
  lookahead_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  markers.markers.push_back(lookahead_marker);

  visualization_msgs::msg::Marker line_marker;
  line_marker.header.stamp = now();
  line_marker.header.frame_id = "map";
  line_marker.ns = "lookahead_line";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.05;
  line_marker.color.r = 0.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 1.0;
  line_marker.color.a = 0.8;
  line_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
  geometry_msgs::msg::Point p1;
  p1.x = current_pose.position.x;
  p1.y = current_pose.position.y;
  p1.z = current_pose.position.z + 0.3;
  line_marker.points.push_back(p1);
  geometry_msgs::msg::Point p2;
  p2.x = current_trajectory_->points[lookahead_idx].pose.position.x;
  p2.y = current_trajectory_->points[lookahead_idx].pose.position.y;
  p2.z = current_trajectory_->points[lookahead_idx].pose.position.z + 0.3;
  line_marker.points.push_back(p2);
  markers.markers.push_back(line_marker);

  debug_marker_pub_->publish(markers);
}

double ReverseParkingPlannerNode::calcDistanceToGoal(const geometry_msgs::msg::Pose & current_pose) const
{
  if (!current_trajectory_ || current_trajectory_->points.empty()) {
    return std::numeric_limits<double>::max();
  }
  const auto & goal = current_trajectory_->points.back();
  const double dx = goal.pose.position.x - current_pose.position.x;
  const double dy = goal.pose.position.y - current_pose.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ReverseParkingPlannerNode::normalizeAngle(double angle) const
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace reverse_parking_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(reverse_parking_planner::ReverseParkingPlannerNode)
