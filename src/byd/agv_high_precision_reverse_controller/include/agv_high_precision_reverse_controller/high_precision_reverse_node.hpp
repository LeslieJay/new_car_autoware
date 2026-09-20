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

#ifndef AGV_HIGH_PRECISION_REVERSE_CONTROLLER__HIGH_PRECISION_REVERSE_NODE_HPP_
#define AGV_HIGH_PRECISION_REVERSE_CONTROLLER__HIGH_PRECISION_REVERSE_NODE_HPP_

#include "agv_high_precision_reverse_controller/precision_reverse_controller.hpp"
#include "reverse_parking_planner/srv/set_goal_pose.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_control_msgs/srv/set_pause.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace agv_high_precision_reverse_controller
{

class HighPrecisionReverseNode : public rclcpp::Node
{
public:
  explicit HighPrecisionReverseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using SetGoalPose = reverse_parking_planner::srv::SetGoalPose;

  enum class State : std::uint8_t
  {
    IDLE = 0,
    TRACKING = 1,
    SAFETY_HOLD = 2,
    GOAL_REACHED = 3,
    ABORTED = 4
  };

  enum class HandoffPhase : std::uint8_t
  {
    NONE,
    PAUSE_FOR_REVERSE,
    WAIT_PAUSE_FOR_REVERSE,
    CLEAR_FORWARD_ROUTE,
    WAIT_CLEAR_FORWARD_ROUTE,
    CHANGE_TO_LOCAL,
    WAIT_CHANGE_TO_LOCAL,
    ENABLE_CONTROL,
    WAIT_ENABLE_CONTROL,
    ENGAGE,
    WAIT_ENGAGE,
    UNPAUSE_REVERSE,
    WAIT_UNPAUSE_REVERSE,
    REVERSING,
    PAUSE_FOR_AUTO,
    WAIT_PAUSE_FOR_AUTO,
    CLEAR_FORWARD_ROUTE_BEFORE_AUTO,
    WAIT_CLEAR_FORWARD_ROUTE_BEFORE_AUTO,
    CHANGE_TO_AUTONOMOUS,
    WAIT_CHANGE_TO_AUTONOMOUS,
    UNPAUSE_AUTO,
    WAIT_UNPAUSE_AUTO
  };

  void onTimer();
  void onOdometry(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void onRearWarning(std_msgs::msg::UInt8::ConstSharedPtr message);
  void onSetGoal(
    const std::shared_ptr<SetGoalPose::Request> request,
    std::shared_ptr<SetGoalPose::Response> response);
  void onReplan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onCancel(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void processModeHandoff();
  void beginReturnToAutonomous();
  void retryModeHandoff(HandoffPhase phase, const std::string & reason);
  void failModeHandoff(const std::string & reason);
  void onPauseForReverseResponse(
    rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future);
  void onClearForwardRouteResponse(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedFuture future);
  void onChangeToLocalResponse(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future);
  void onEnableControlResponse(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future);
  void onEngageResponse(rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture future);
  void onUnpauseReverseResponse(
    rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future);
  void onPauseForAutoResponse(
    rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future);
  void onChangeToAutonomousResponse(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future);
  void onUnpauseAutoResponse(
    rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedFuture future);

  bool plan(std::string & reason);
  std::vector<PathPoint> generateReversePath() const;
  autoware_planning_msgs::msg::Trajectory makeTrajectory() const;
  void publishObservability(const TrackingResult * result = nullptr);
  void publishCommand(double steering, double acceleration, double speed);
  void publishGear(bool reverse);
  void publishSignals(bool reversing, bool stopped);
  void publishStop(bool keep_reverse_gear);
  void changeState(State state, const std::string & reason);
  bool odometryIsFresh() const;
  static double normalizeAngle(double angle);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr rear_warning_sub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr indicators_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazards_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::Service<SetGoalPose>::SharedPtr set_goal_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_service_;
  rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedPtr set_pause_client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr clear_route_client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    change_to_local_client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    change_to_autonomous_client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    enable_control_client_;
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr engage_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  PrecisionReverseController controller_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  geometry_msgs::msg::PoseStamped goal_;
  autoware_planning_msgs::msg::Trajectory trajectory_;
  rclcpp::Time last_odometry_receipt_;
  rclcpp::Time last_observability_publish_;
  State state_{State::IDLE};
  HandoffPhase handoff_phase_{HandoffPhase::NONE};
  std::uint8_t rear_warning_level_{0U};
  std::size_t settled_cycles_{0U};
  bool has_goal_{false};
  double last_steering_command_{0.0};

  double control_rate_{50.0};
  double trajectory_publish_rate_{10.0};
  double odometry_timeout_{0.20};
  double turning_radius_{2.0};
  double path_resolution_{0.02};
  double min_path_length_{0.10};
  double max_path_length_{15.0};
  std::size_t required_settled_cycles_{10U};
  std::size_t handoff_retry_count_{0U};
  std::size_t max_handoff_retries_{40U};
  double handoff_retry_interval_{0.5};
  std::chrono::steady_clock::time_point handoff_retry_after_{};
};

}  // namespace agv_high_precision_reverse_controller

#endif  // AGV_HIGH_PRECISION_REVERSE_CONTROLLER__HIGH_PRECISION_REVERSE_NODE_HPP_
