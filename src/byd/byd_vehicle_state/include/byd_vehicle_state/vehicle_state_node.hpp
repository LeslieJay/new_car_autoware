// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef BYD_VEHICLE_STATE__VEHICLE_STATE_NODE_HPP_
#define BYD_VEHICLE_STATE__VEHICLE_STATE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <optional>
#include <string>

namespace byd_vehicle_state
{

/**
 * @brief 综合前进/倒车 goal，发布与 /autoware/state 同类型的车辆状态
 *
 * 输入:
 *  - 前进 goal: /planning/mission_planning/goal
 *  - 倒车 goal: /planning/parking/goal（由 reverse_parking_planner 发布）
 *  - 定位: /localization/kinematic_state
 *  - 可选: /api/routing/state（前进到达的辅助信号）
 *  - /api/operation_mode/state（区分 WaitingForEngage / Driving）
 *
 * 输出:
 *  - /byd/autoware/state (autoware_system_msgs/AutowareState)
 *  - /byd/autoware/state_name (std_msgs/String，便于调试)
 */
class VehicleStateNode : public rclcpp::Node
{
public:
  explicit VehicleStateNode(const rclcpp::NodeOptions & options);

private:
  enum class GoalMode : uint8_t
  {
    None = 0,
    Forward = 1,
    Reverse = 2,
  };

  void onForwardGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onReverseGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onRouteState(const autoware_adapi_v1_msgs::msg::RouteState::ConstSharedPtr msg);
  void onOperationMode(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void onLaneletRoute(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void onTimer();

  void setActiveGoal(const geometry_msgs::msg::PoseStamped & goal, GoalMode mode);
  bool hasActiveMission() const;
  bool isDrivingEngaged() const;
  bool isArrivedAtGoal() const;
  uint8_t computeState() const;
  static std::string stateToString(uint8_t state);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr forward_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reverse_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    operation_mode_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr lanelet_route_sub_;

  rclcpp::Publisher<autoware_system_msgs::msg::AutowareState>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_name_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::ConstSharedPtr current_odom_;
  std::optional<geometry_msgs::msg::PoseStamped> active_goal_;
  bool has_valid_goal_pose_{false};
  GoalMode goal_mode_{GoalMode::None};
  bool use_route_state_for_forward_{true};
  uint8_t route_state_{autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN};
  uint8_t operation_mode_{autoware_adapi_v1_msgs::msg::OperationModeState::UNKNOWN};
  bool is_autoware_control_enabled_{false};
  bool operation_mode_received_{false};

  double arrive_distance_th_{0.1};
  double arrive_yaw_th_{0.1};
  double arrive_speed_th_{0.05};
  double arrive_hold_time_{1.0};
  double arrived_to_unset_timeout_{2.0};
  double update_rate_{10.0};

  bool arrived_condition_met_{false};
  std::optional<rclcpp::Time> arrived_since_;
  std::optional<rclcpp::Time> arrived_published_since_;
  uint8_t prev_state_{autoware_system_msgs::msg::AutowareState::INITIALIZING};
};

}  // namespace byd_vehicle_state

#endif  // BYD_VEHICLE_STATE__VEHICLE_STATE_NODE_HPP_
