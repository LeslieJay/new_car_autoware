// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef REVERSE_PARKING_CONTROLLER__REVERSE_PARKING_CONTROLLER_NODE_HPP_
#define REVERSE_PARKING_CONTROLLER__REVERSE_PARKING_CONTROLLER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace reverse_parking_controller
{

/**
 * @brief 倒车停车路径跟踪控制器
 *
 * 功能：
 * 1. 订阅倒车规划器输出的轨迹 (autoware_planning_msgs::msg::Trajectory)
 * 2. 使用 Pure Pursuit + PID 进行路径跟踪
 * 3. 输出控制指令到 vehicle_cmd_gate 的 input/external 话题
 *    - control_cmd (转向 + 纵向控制)
 *    - gear_cmd (前进/倒车档位)
 *    - turn_indicators_cmd (转向灯)
 *    - hazard_lights_cmd (危险警示灯)
 */
class ReverseParkingControllerNode : public rclcpp::Node
{
public:
  explicit ReverseParkingControllerNode(const rclcpp::NodeOptions & options);

private:
  // ---- 回调函数 ----
  void onTimer();
  void onTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  // ---- 控制算法 ----
  /// Pure Pursuit 横向控制：计算前轮转角
  double calcSteeringAngle(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    size_t lookahead_idx, bool is_reverse) const;

  /// PID 纵向控制：计算加速度指令
  double calcAcceleration(double target_velocity, double current_velocity);

  /// 找到轨迹上距当前位置最近的索引
  size_t findNearestIndex(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory) const;

  /// 找到前视距离内的目标点索引
  size_t findLookaheadIndex(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    size_t nearest_idx, double lookahead_distance) const;

  /// 判断是否到达终点
  bool isGoalReached(
    const geometry_msgs::msg::Pose & current_pose,
    const autoware_planning_msgs::msg::Trajectory & trajectory) const;

  /// 发布控制指令
  void publishControlCmd(
    double steering_angle, double acceleration, double target_velocity);

  /// 发布档位指令
  void publishGearCmd(bool is_reverse);

  /// 发布转向灯和危险灯指令
  void publishIndicatorCmds(bool is_reverse, bool is_stopped);

  /// 发布调试可视化
  void publishDebugMarkers(
    const geometry_msgs::msg::Pose & current_pose,
    size_t nearest_idx, size_t lookahead_idx);

  /// 角度归一化到 [-pi, pi]
  double normalizeAngle(double angle) const;

  /// 计算当前位置到轨迹终点的距离
  double calcDistanceToGoal(const geometry_msgs::msg::Pose & current_pose) const;

  // ---- 订阅者 ----
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // ---- 发布者（发送到 vehicle_cmd_gate 的 input/external）----
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr
    turn_indicator_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr
    hazard_light_cmd_pub_;

  // ---- 调试发布者 ----
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  // ---- 定时器 ----
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- TF ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- 状态 ----
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr current_trajectory_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom_;
  bool is_goal_reached_{false};
  bool is_active_{false};
  bool prev_is_reverse_{false};  // 上一拍的行驶方向，用于档位切换安全逻辑

  // ---- PID 纵向控制器状态 ----
  double pid_error_integral_{0.0};
  double pid_prev_error_{0.0};
  rclcpp::Time prev_time_;

  // ---- 参数 ----
  // 控制频率
  double control_rate_;          // Hz

  // Pure Pursuit 参数
  double lookahead_distance_;    // 前视距离 [m]
  double min_lookahead_distance_;// 最小前视距离 [m]
  double lookahead_ratio_;       // 前视距离与速度比例系数
  double wheel_base_;            // 轴距 [m]

  // PID 纵向控制参数
  double kp_;                    // 比例增益
  double ki_;                    // 积分增益
  double kd_;                    // 微分增益
  double max_acceleration_;      // 最大加速度 [m/s^2]
  double max_deceleration_;      // 最大减速度 [m/s^2]
  double pid_integral_max_;      // 积分上限

  // 到达判定阈值
  double goal_distance_threshold_;  // 到达终点距离阈值 [m]
  double goal_yaw_threshold_;       // 到达终点航向阈值 [rad]
  double stop_velocity_threshold_;  // 停车速度阈值 [m/s]

  // 转向限幅
  double max_steering_angle_;    // 最大转向角 [rad]

  // AGV充电对接优化参数
  double reverse_lookahead_distance_;  // 倒车前视距离 [m] (更短以提高精度)
  double stanley_gain_;                // Stanley横向误差修正增益
  double final_approach_distance_;     // 最终接近距离 [m]
  double final_approach_speed_;        // 最终接近速度 [m/s]
};

}  // namespace reverse_parking_controller

#endif  // REVERSE_PARKING_CONTROLLER__REVERSE_PARKING_CONTROLLER_NODE_HPP_
