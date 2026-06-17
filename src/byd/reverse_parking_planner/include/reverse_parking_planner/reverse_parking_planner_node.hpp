// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef REVERSE_PARKING_PLANNER__REVERSE_PARKING_PLANNER_NODE_HPP_
#define REVERSE_PARKING_PLANNER__REVERSE_PARKING_PLANNER_NODE_HPP_

#include "reverse_parking_planner/reeds_shepp.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "reverse_parking_planner/srv/set_goal_pose.hpp"

#include <memory>
#include <optional>

namespace reverse_parking_planner
{

using SetGoalPose = reverse_parking_planner::srv::SetGoalPose;

/**
 * @brief 倒车停车规划器节点
 * 
 * 功能：
 * 1. 通过服务接收目标停车位姿并自动规划路径
 * 2. 使用Reeds-Shepp曲线规划包含倒车的路径
 * 3. 发布轨迹供控制器跟踪
 */
class ReverseParkingPlannerNode : public rclcpp::Node
{
public:
  explicit ReverseParkingPlannerNode(const rclcpp::NodeOptions & options);

private:
  // 回调函数
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
  autoware_planning_msgs::msg::Trajectory convertToTrajectory(
    const std::vector<PathPoint>& path_points) const;
  
  // 可视化
  void publishVisualization(const std::vector<PathPoint>& path_points);
  visualization_msgs::msg::MarkerArray createPathMarkers(
    const std::vector<PathPoint>& path_points) const;
  
  // 工具函数
  double normalizeAngle(double angle) const;
  std::optional<geometry_msgs::msg::Pose> getCurrentPose() const;

  // ROS通信
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<SetGoalPose>::SharedPtr set_goal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 规划器
  std::unique_ptr<ReedsSheppPlanner> planner_;
  
  // 状态
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  bool has_goal_{false};
  bool planning_triggered_{false};
  std::vector<PathPoint> current_path_;

  // 参数
  double wheel_base_;
  double min_turning_radius_;
  double vehicle_length_;
  double vehicle_width_;
  double path_resolution_;
  double velocity_forward_;
  double velocity_reverse_;
  double publish_rate_;
  bool enable_reverse_only_;

  // AGV充电对接优化参数
  double final_approach_distance_;    // 最终直线倒车接近距离 [m]
  double velocity_creep_;             // 蠕行速度（最终接近段最低速度）[m/s]
  double decel_distance_;             // 终点减速区距离 [m]
  double transition_decel_distance_;  // 方向切换减速区距离 [m]
};

}  // namespace reverse_parking_planner

#endif  // REVERSE_PARKING_PLANNER__REVERSE_PARKING_PLANNER_NODE_HPP_
