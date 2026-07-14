// Copyright 2026 BYD.

#ifndef AGV_SIMPLE_REVERSE_PLANNER__AGV_SIMPLE_REVERSE_PLANNER_NODE_HPP_
#define AGV_SIMPLE_REVERSE_PLANNER__AGV_SIMPLE_REVERSE_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <autoware/freespace_planning_algorithms/reeds_shepp.hpp>

#include "reverse_parking_planner/srv/set_goal_pose.hpp"

#include <memory>
#include <vector>

namespace agv_simple_reverse_planner
{

using SetGoalPose = reverse_parking_planner::srv::SetGoalPose;

class AgvSimpleReversePlannerNode : public rclcpp::Node
{
public:
  explicit AgvSimpleReversePlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class State : uint8_t
  {
    IDLE = 0,
    PLANNED = 1,
    REVERSING = 2,
    CAUTION_HOLD = 3,
    GOAL_REACHED = 4,
    DIST_LIMIT = 5,
    ABORT = 6
  };

  struct PathPoint
  {
    double x{};
    double y{};
    double yaw{};
  };

  static constexpr uint8_t kRearWarningSafe = 0;
  static constexpr uint8_t kRearWarningWarning = 1;
  static constexpr uint8_t kRearWarningCaution = 2;
  static constexpr uint8_t kRearWarningStop = 3;

  void onTimer();
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onRearWarningLevel(const std_msgs::msg::UInt8::ConstSharedPtr msg);
  void onSetGoalPose(
    const std::shared_ptr<SetGoalPose::Request> request,
    std::shared_ptr<SetGoalPose::Response> response);
  void onTriggerPlanning(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  bool planPath();
  std::vector<PathPoint> generateRsReversePath() const;
  autoware_planning_msgs::msg::Trajectory convertToTrajectory(const std::vector<PathPoint> & path_points)
    const;

  void runControlLoop();
  bool isGoalReached(const geometry_msgs::msg::Pose & current_pose) const;
  double computeTargetSpeed(double distance_to_goal) const;
  double computeSteering(const geometry_msgs::msg::Pose & current_pose) const;
  double computeAcceleration(double target_speed, double current_speed) const;
  size_t findNearestPathIndex(const geometry_msgs::msg::Pose & current_pose) const;
  size_t findLookaheadPathIndex(size_t nearest_idx, double lookahead_distance) const;
  bool isForwardSegment(const PathPoint & from, const PathPoint & to) const;
  double normalizeAngle(double angle) const;
  void updateReverseDistance(const geometry_msgs::msg::Pose & current_pose);
  void setState(State state, const std::string & reason);

  void publishTrajectoryAndMarkers();
  void publishControlCmd(double steering_angle, double acceleration, double target_velocity);
  void publishGearCmd(bool is_reverse);
  void publishIndicatorCmds(bool is_reverse, bool is_stopped);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr rear_warning_level_sub_;

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicator_cmd_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_light_cmd_pub_;

  rclcpp::Service<SetGoalPose>::SharedPtr set_goal_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::ConstSharedPtr current_odom_;
  autoware_planning_msgs::msg::Trajectory::SharedPtr current_trajectory_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  std::vector<PathPoint> current_path_;

  State state_{State::IDLE};
  bool has_goal_{false};
  uint8_t rear_warning_level_{kRearWarningSafe};
  double reverse_driven_distance_{0.0};
  double prev_reverse_odom_x_{0.0};
  double prev_reverse_odom_y_{0.0};
  bool prev_reverse_odom_initialized_{false};
  rclcpp::Time last_publish_time_;
  bool has_last_publish_time_{false};

  double control_rate_{30.0};
  double publish_rate_{10.0};

  double wheel_base_{1.01};
  double path_resolution_{0.05};
  double velocity_reverse_{-1.0};
  double velocity_creep_{0.05};
  double decel_distance_{1.5};
  double final_approach_distance_{1.0};
  double final_approach_speed_{0.05};

  double max_straight_lateral_error_{1.0};
  double max_straight_yaw_error_{1.0};
  double min_reverse_distance_{0.2};
  double max_reverse_driving_distance_{0.0};
  double rs_turning_radius_{2.0};
  double rs_step_size_{0.1};
  bool rs_reverse_only_{true};
  double rs_lookahead_distance_{0.7};

  double goal_distance_threshold_{0.05};
  double goal_yaw_threshold_{0.03};
  double stop_velocity_threshold_{0.02};

  double max_steering_angle_{0.6};
  double lat_kp_{0.4};
  double yaw_kp_{0.8};
  double max_acceleration_{0.5};
  double max_deceleration_{-1.0};
  double speed_kp_{1.5};
};

}  // namespace agv_simple_reverse_planner

#endif  // AGV_SIMPLE_REVERSE_PLANNER__AGV_SIMPLE_REVERSE_PLANNER_NODE_HPP_
