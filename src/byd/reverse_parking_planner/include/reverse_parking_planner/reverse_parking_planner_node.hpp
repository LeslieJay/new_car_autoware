// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef REVERSE_PARKING_PLANNER__REVERSE_PARKING_PLANNER_NODE_HPP_
#define REVERSE_PARKING_PLANNER__REVERSE_PARKING_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "reverse_parking_planner/srv/set_goal_pose.hpp"

#include <chrono>
#include <memory>
#include <vector>

namespace reverse_parking_planner
{

using SetGoalPose = reverse_parking_planner::srv::SetGoalPose;

/**
 * @brief 倒车停车统一节点（规划 + 控制）
 *
 * 功能：
 * 1. 通过服务接收目标位姿并生成直线倒车轨迹
 * 2. 执行 Pure Pursuit + PID 路径跟踪
 * 3. 发布控制指令到 vehicle_cmd_gate 外部输入
 */
class ReverseParkingPlannerNode : public rclcpp::Node
{
public:
  explicit ReverseParkingPlannerNode(const rclcpp::NodeOptions & options);

private:
  struct PathPoint
  {
    double x{};
    double y{};
    double yaw{};
    bool is_reverse{true};
    double curvature{};
  };

  // 规划与控制回调函数
  void onTimer();
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // 服务回调：设置目标位姿并触发规划
  void onSetGoalPose(
    const std::shared_ptr<SetGoalPose::Request> request,
    std::shared_ptr<SetGoalPose::Response> response);

  // 服务回调：仅触发重新规划（使用已有目标）
  void onTriggerPlanning(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  
  // 规划函数
  bool planPath();
  autoware_planning_msgs::msg::Trajectory convertToTrajectory(const std::vector<PathPoint> & path_points)
    const;
  std::vector<PathPoint> generateStraightReversePath() const;
  
  // 可视化
  void publishVisualization(const std::vector<PathPoint> & path_points);
  visualization_msgs::msg::MarkerArray createPathMarkers(const std::vector<PathPoint> & path_points) const;
  
  // 控制函数
  void runControlLoop();
  double calcSteeringAngle(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    size_t lookahead_idx, bool is_reverse) const;
  double calcAcceleration(double target_velocity, double current_velocity);
  size_t findNearestIndex(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory) const;
  size_t findLookaheadIndex(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    size_t nearest_idx, double lookahead_distance) const;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory) const;
  void publishControlCmd(double steering_angle, double acceleration, double target_velocity);
  void publishGearCmd(bool is_reverse);
  void publishIndicatorCmds(bool is_reverse, bool is_stopped);
  void publishDebugMarkers(
    const geometry_msgs::msg::Pose & current_pose,
    size_t nearest_idx, size_t lookahead_idx);
  double calcDistanceToGoal(const geometry_msgs::msg::Pose & current_pose) const;

  double normalizeAngle(double angle) const;

  // ROS通信
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicator_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_light_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
  rclcpp::Service<SetGoalPose>::SharedPtr set_goal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 规划状态
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr current_trajectory_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_goal_{false};
  std::vector<PathPoint> current_path_;

  // 控制状态
  bool is_goal_reached_{false};
  bool is_distance_limit_reached_{false};
  double reverse_driven_distance_{0.0};
  double prev_reverse_odom_x_{0.0};
  double prev_reverse_odom_y_{0.0};
  bool prev_reverse_odom_initialized_{false};
  bool prev_is_reverse_{false};
  double pid_error_integral_{0.0};
  double pid_prev_error_{0.0};
  std::chrono::steady_clock::time_point prev_control_time_;
  bool prev_control_time_initialized_{false};
  std::chrono::steady_clock::time_point last_traj_publish_wall_time_;
  bool last_traj_publish_initialized_{false};

  // 规划参数
  double wheel_base_;
  double path_resolution_;
  double velocity_reverse_;
  double publish_rate_;
  double velocity_creep_;
  double decel_distance_;

  // 直线倒车准入条件
  double max_straight_lateral_error_;
  double max_straight_yaw_error_;
  double min_reverse_distance_;
  double max_reverse_driving_distance_;

  // 控制参数
  double control_rate_;
  double lookahead_distance_;
  double min_lookahead_distance_;
  double lookahead_ratio_;
  double kp_;
  double ki_;
  double kd_;
  double max_acceleration_;
  double max_deceleration_;
  double pid_integral_max_;
  double goal_distance_threshold_;
  double goal_yaw_threshold_;
  double stop_velocity_threshold_;
  double max_steering_angle_;
  double reverse_lookahead_distance_;
  double stanley_gain_;
  double final_approach_distance_;
  double final_approach_speed_;
};

}  // namespace reverse_parking_planner

#endif  // REVERSE_PARKING_PLANNER__REVERSE_PARKING_PLANNER_NODE_HPP_
