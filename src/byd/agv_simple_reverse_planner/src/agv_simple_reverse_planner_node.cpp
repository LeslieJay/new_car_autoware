// Copyright 2026 BYD.

#include "agv_simple_reverse_planner/agv_simple_reverse_planner_node.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace agv_simple_reverse_planner
{

using autoware::freespace_planning_algorithms::ReedsSheppStateSpace;

AgvSimpleReversePlannerNode::AgvSimpleReversePlannerNode(const rclcpp::NodeOptions & options)
: Node("reverse_parking_planner", options)
{
  control_rate_ = declare_parameter<double>("control_rate", control_rate_);
  publish_rate_ = declare_parameter<double>("publish_rate", publish_rate_);

  wheel_base_ = declare_parameter<double>("wheel_base", wheel_base_);
  path_resolution_ = declare_parameter<double>("path_resolution", path_resolution_);
  velocity_reverse_ = declare_parameter<double>("velocity_reverse", velocity_reverse_);
  velocity_creep_ = declare_parameter<double>("velocity_creep", velocity_creep_);
  decel_distance_ = declare_parameter<double>("decel_distance", decel_distance_);
  final_approach_distance_ = declare_parameter<double>("final_approach_distance", final_approach_distance_);
  final_approach_speed_ = declare_parameter<double>("final_approach_speed", final_approach_speed_);

  max_straight_lateral_error_ =
    declare_parameter<double>("max_straight_lateral_error", max_straight_lateral_error_);
  max_straight_yaw_error_ = declare_parameter<double>("max_straight_yaw_error", max_straight_yaw_error_);
  min_reverse_distance_ = declare_parameter<double>("min_reverse_distance", min_reverse_distance_);
  max_reverse_driving_distance_ =
    declare_parameter<double>("max_reverse_driving_distance", max_reverse_driving_distance_);
  rs_turning_radius_ = declare_parameter<double>("rs_turning_radius", rs_turning_radius_);
  rs_step_size_ = declare_parameter<double>("rs_step_size", rs_step_size_);
  rs_reverse_only_ = declare_parameter<bool>("rs_reverse_only", rs_reverse_only_);
  rs_lookahead_distance_ = declare_parameter<double>("rs_lookahead_distance", rs_lookahead_distance_);

  goal_distance_threshold_ = declare_parameter<double>("goal_distance_threshold", goal_distance_threshold_);
  goal_yaw_threshold_ = declare_parameter<double>("goal_yaw_threshold", goal_yaw_threshold_);
  stop_velocity_threshold_ = declare_parameter<double>("stop_velocity_threshold", stop_velocity_threshold_);

  max_steering_angle_ = declare_parameter<double>("max_steering_angle", max_steering_angle_);
  lat_kp_ = declare_parameter<double>("lat_kp", lat_kp_);
  yaw_kp_ = declare_parameter<double>("yaw_kp", yaw_kp_);
  max_acceleration_ = declare_parameter<double>("max_acceleration", max_acceleration_);
  max_deceleration_ = declare_parameter<double>("max_deceleration", max_deceleration_);
  speed_kp_ = declare_parameter<double>("pid.kp", speed_kp_);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1, std::bind(&AgvSimpleReversePlannerNode::onOdometry, this, std::placeholders::_1));
  rear_warning_level_sub_ = create_subscription<std_msgs::msg::UInt8>(
    "~/input/rear_warning_level", 1,
    std::bind(&AgvSimpleReversePlannerNode::onRearWarningLevel, this, std::placeholders::_1));

  traj_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/path_markers", 1);
  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/output/goal", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>("~/output/control_cmd", 1);
  gear_cmd_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearCommand>("~/output/gear_cmd", 1);
  turn_indicator_cmd_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  hazard_light_cmd_pub_ =
    create_publisher<autoware_vehicle_msgs::msg::HazardLightsCommand>("~/output/hazard_lights_cmd", 1);

  set_goal_srv_ = create_service<SetGoalPose>(
    "~/set_goal_pose",
    std::bind(
      &AgvSimpleReversePlannerNode::onSetGoalPose, this, std::placeholders::_1, std::placeholders::_2));
  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/trigger_planning",
    std::bind(
      &AgvSimpleReversePlannerNode::onTriggerPlanning, this, std::placeholders::_1,
      std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_, 1.0));
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&AgvSimpleReversePlannerNode::onTimer, this));

  RCLCPP_INFO(get_logger(), "agv_simple_reverse_planner started");
  RCLCPP_INFO(get_logger(), "control_rate=%.1f, reverse_vel=%.2f", control_rate_, velocity_reverse_);
  RCLCPP_INFO(
    get_logger(), "rs_turning_radius=%.2f, rs_step_size=%.2f, rs_reverse_only=%s",
    rs_turning_radius_, rs_step_size_, rs_reverse_only_ ? "true" : "false");
}

void AgvSimpleReversePlannerNode::onTimer()
{
  if (!current_odom_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }

  publishTrajectoryAndMarkers();
  runControlLoop();
}

void AgvSimpleReversePlannerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odom_ = msg;
}

void AgvSimpleReversePlannerNode::onRearWarningLevel(const std_msgs::msg::UInt8::ConstSharedPtr msg)
{
  rear_warning_level_ = msg->data;
}

void AgvSimpleReversePlannerNode::onSetGoalPose(
  const std::shared_ptr<SetGoalPose::Request> request, std::shared_ptr<SetGoalPose::Response> response)
{
  goal_pose_ = request->goal_pose;
  if (goal_pose_.header.stamp.sec == 0 && goal_pose_.header.stamp.nanosec == 0) {
    goal_pose_.header.stamp = now();
  }
  if (goal_pose_.header.frame_id.empty()) {
    goal_pose_.header.frame_id = "map";
  }
  has_goal_ = true;
  goal_pub_->publish(goal_pose_);

  if (!current_odom_) {
    setState(State::PLANNED, "goal accepted, waiting odometry");
    response->success = false;
    response->message = "No odometry available, goal saved";
    response->path_points_num = 0;
    return;
  }

  if (!planPath()) {
    setState(State::ABORT, "failed to plan RS reverse path");
    response->success = false;
    response->message = "Failed to plan path";
    response->path_points_num = 0;
    return;
  }

  response->success = true;
  response->message = "Path planned";
  response->path_points_num = static_cast<uint32_t>(current_path_.size());
}

void AgvSimpleReversePlannerNode::onTriggerPlanning(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!has_goal_) {
    response->success = false;
    response->message = "No goal pose set";
    return;
  }
  if (!current_odom_) {
    response->success = false;
    response->message = "No odometry available";
    return;
  }
  if (!planPath()) {
    setState(State::ABORT, "trigger planning failed");
    response->success = false;
    response->message = "Failed to plan path";
    return;
  }
  response->success = true;
  response->message = "Path planned";
}

bool AgvSimpleReversePlannerNode::planPath()
{
  if (!current_odom_ || !has_goal_) {
    return false;
  }

  auto path_points = generateRsReversePath();
  if (path_points.empty()) {
    current_path_.clear();
    current_trajectory_.reset();
    return false;
  }

  current_path_ = std::move(path_points);
  current_trajectory_ = std::make_shared<autoware_planning_msgs::msg::Trajectory>(
    convertToTrajectory(current_path_));

  reverse_driven_distance_ = 0.0;
  prev_reverse_odom_initialized_ = false;
  setState(State::PLANNED, "path ready");
  return true;
}

std::vector<AgvSimpleReversePlannerNode::PathPoint> AgvSimpleReversePlannerNode::generateRsReversePath()
  const
{
  const ReedsSheppStateSpace::StateXYT s0{
    current_odom_->pose.pose.position.x,
    current_odom_->pose.pose.position.y,
    tf2::getYaw(current_odom_->pose.pose.orientation)};
  const ReedsSheppStateSpace::StateXYT s1{
    goal_pose_.pose.position.x,
    goal_pose_.pose.position.y,
    tf2::getYaw(goal_pose_.pose.orientation)};

  ReedsSheppStateSpace rs(std::max(0.1, rs_turning_radius_));
  auto path = rs.reedsShepp(s0, s1);
  const double total_dist = path.length();
  if (!std::isfinite(total_dist) || total_dist < min_reverse_distance_) {
    RCLCPP_WARN(get_logger(), "RS path invalid/too short: %.3f m", total_dist);
    return {};
  }

  const int steps =
    std::max(2, static_cast<int>(std::ceil(total_dist / std::max(0.02, rs_step_size_))) + 1);
  std::vector<PathPoint> path_points;
  path_points.reserve(static_cast<size_t>(steps));

  for (int i = 0; i < steps; ++i) {
    const double seg = total_dist * static_cast<double>(i) / static_cast<double>(steps - 1);
    const auto s = rs.interpolate(s0, path, seg);
    PathPoint pt;
    pt.x = s.x;
    pt.y = s.y;
    pt.yaw = normalizeAngle(s.yaw);
    path_points.push_back(pt);
  }

  if (rs_reverse_only_) {
    for (size_t i = 1; i < path_points.size(); ++i) {
      if (isForwardSegment(path_points[i - 1], path_points[i])) {
        RCLCPP_WARN(get_logger(), "RS path contains forward segment, reject due to rs_reverse_only");
        return {};
      }
    }
  }

  return path_points;
}

autoware_planning_msgs::msg::Trajectory AgvSimpleReversePlannerNode::convertToTrajectory(
  const std::vector<PathPoint> & path_points) const
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header.stamp = now();
  trajectory.header.frame_id = "map";
  if (path_points.empty()) {
    return trajectory;
  }

  std::vector<double> cumulative(path_points.size(), 0.0);
  for (size_t i = 1; i < path_points.size(); ++i) {
    const double dx = path_points[i].x - path_points[i - 1].x;
    const double dy = path_points[i].y - path_points[i - 1].y;
    cumulative[i] = cumulative[i - 1] + std::hypot(dx, dy);
  }
  const double total = cumulative.back();

  for (size_t i = 0; i < path_points.size(); ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint traj_pt;
    traj_pt.pose.position.x = path_points[i].x;
    traj_pt.pose.position.y = path_points[i].y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, path_points[i].yaw);
    traj_pt.pose.orientation = tf2::toMsg(q);

    const double remain = std::max(0.0, total - cumulative[i]);
    double speed = std::abs(velocity_reverse_);
    if (decel_distance_ > 0.0 && remain < decel_distance_) {
      const double ratio = remain / decel_distance_;
      const double floor_speed = std::max(0.0, std::abs(velocity_creep_));
      speed = std::max(floor_speed, speed * std::sqrt(ratio));
    }
    if (i + 1 == path_points.size()) {
      speed = 0.0;
    }

    traj_pt.longitudinal_velocity_mps = -static_cast<float>(speed);
    traj_pt.lateral_velocity_mps = 0.0;
    traj_pt.acceleration_mps2 = 0.0;
    traj_pt.heading_rate_rps = 0.0;

    const size_t prev_idx = (i == 0) ? i : (i - 1);
    const size_t next_idx = (i + 1 < path_points.size()) ? (i + 1) : i;
    double steering = 0.0;
    if (next_idx != prev_idx) {
      const double ddx = path_points[next_idx].x - path_points[prev_idx].x;
      const double ddy = path_points[next_idx].y - path_points[prev_idx].y;
      const double ds = std::hypot(ddx, ddy);
      if (ds > 1e-3) {
        const double dyaw = normalizeAngle(path_points[next_idx].yaw - path_points[prev_idx].yaw);
        const double curvature = dyaw / ds;
        steering = std::atan(wheel_base_ * curvature);
      }
    }
    traj_pt.front_wheel_angle_rad = static_cast<float>(std::clamp(
      steering, -max_steering_angle_, max_steering_angle_));
    trajectory.points.push_back(traj_pt);
  }
  return trajectory;
}

void AgvSimpleReversePlannerNode::runControlLoop()
{
  if (!current_odom_ || !current_trajectory_ || current_trajectory_->points.empty()) {
    return;
  }
  const auto & pose = current_odom_->pose.pose;
  updateReverseDistance(pose);
  if (state_ == State::PLANNED) {
    setState(State::REVERSING, "start reversing");
  }

  const bool caution_hold = rear_warning_level_ >= kRearWarningCaution;
  const bool dist_limit_hit =
    max_reverse_driving_distance_ > 0.0 && reverse_driven_distance_ >= max_reverse_driving_distance_;
  const bool goal_hit = isGoalReached(pose);

  if (goal_hit) {
    setState(State::GOAL_REACHED, "goal reached");
  } else if (dist_limit_hit) {
    setState(State::DIST_LIMIT, "distance limit reached");
  } else if (caution_hold) {
    setState(State::CAUTION_HOLD, "rear warning hold");
  } else if (state_ == State::CAUTION_HOLD || state_ == State::PLANNED || state_ == State::ABORT) {
    setState(State::REVERSING, "resume reversing");
  }

  if (state_ == State::GOAL_REACHED || state_ == State::DIST_LIMIT || state_ == State::ABORT) {
    publishControlCmd(0.0, max_deceleration_, 0.0);
    publishGearCmd(false);
    publishIndicatorCmds(false, true);
    return;
  }

  if (state_ == State::CAUTION_HOLD) {
    publishControlCmd(0.0, max_deceleration_, 0.0);
    publishGearCmd(true);
    publishIndicatorCmds(true, true);
    return;
  }

  const double distance_to_goal = std::hypot(
    goal_pose_.pose.position.x - pose.position.x, goal_pose_.pose.position.y - pose.position.y);
  const double target_speed = computeTargetSpeed(distance_to_goal);
  const double steering = computeSteering(pose);
  const double current_speed = std::abs(current_odom_->twist.twist.linear.x);
  const double accel = computeAcceleration(target_speed, current_speed);
  publishControlCmd(steering, accel, target_speed);
  publishGearCmd(true);
  publishIndicatorCmds(true, target_speed < stop_velocity_threshold_);
}

bool AgvSimpleReversePlannerNode::isGoalReached(const geometry_msgs::msg::Pose & current_pose) const
{
  const double dx = goal_pose_.pose.position.x - current_pose.position.x;
  const double dy = goal_pose_.pose.position.y - current_pose.position.y;
  const double dist = std::hypot(dx, dy);
  const double yaw_error = std::abs(
    normalizeAngle(tf2::getYaw(current_pose.orientation) - tf2::getYaw(goal_pose_.pose.orientation)));
  return dist < goal_distance_threshold_ && yaw_error < goal_yaw_threshold_;
}

double AgvSimpleReversePlannerNode::computeTargetSpeed(double distance_to_goal) const
{
  const double base_speed = std::abs(velocity_reverse_);
  if (distance_to_goal < goal_distance_threshold_) {
    return 0.0;
  }
  if (distance_to_goal < final_approach_distance_ && final_approach_distance_ > 0.0) {
    const double ratio = distance_to_goal / final_approach_distance_;
    return std::max(std::abs(final_approach_speed_), base_speed * ratio);
  }
  return base_speed;
}

double AgvSimpleReversePlannerNode::computeSteering(const geometry_msgs::msg::Pose & current_pose) const
{
  if (current_path_.size() < 2) {
    return 0.0;
  }

  const size_t nearest_idx = findNearestPathIndex(current_pose);
  const size_t lookahead_idx = findLookaheadPathIndex(nearest_idx, rs_lookahead_distance_);
  const auto & target = current_path_[lookahead_idx];

  const double dx = target.x - current_pose.position.x;
  const double dy = target.y - current_pose.position.y;
  const double current_yaw = tf2::getYaw(current_pose.orientation);
  const double local_x = std::cos(-current_yaw) * dx - std::sin(-current_yaw) * dy;
  const double local_y = std::sin(-current_yaw) * dx + std::cos(-current_yaw) * dy;
  const double lookahead = std::hypot(local_x, local_y);
  if (lookahead < 1e-3) {
    return 0.0;
  }

  const double alpha = std::atan2(-local_y, -local_x);
  double pure_pursuit_steer = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead);
  pure_pursuit_steer = -pure_pursuit_steer;

  const double lateral_error = local_y;
  const double yaw_error = normalizeAngle(current_yaw - target.yaw);
  const double feedback_steer = -(lat_kp_ * lateral_error + yaw_kp_ * yaw_error);

  return std::clamp(
    pure_pursuit_steer + feedback_steer, -max_steering_angle_, max_steering_angle_);
}

double AgvSimpleReversePlannerNode::computeAcceleration(double target_speed, double current_speed) const
{
  if (target_speed <= stop_velocity_threshold_) {
    return max_deceleration_;
  }
  const double accel = speed_kp_ * (target_speed - current_speed);
  return std::clamp(accel, max_deceleration_, max_acceleration_);
}

size_t AgvSimpleReversePlannerNode::findNearestPathIndex(const geometry_msgs::msg::Pose & current_pose) const
{
  double min_dist = std::numeric_limits<double>::max();
  size_t nearest_idx = 0;
  for (size_t i = 0; i < current_path_.size(); ++i) {
    const double dx = current_path_[i].x - current_pose.position.x;
    const double dy = current_path_[i].y - current_pose.position.y;
    const double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

size_t AgvSimpleReversePlannerNode::findLookaheadPathIndex(
  size_t nearest_idx, double lookahead_distance) const
{
  if (current_path_.empty()) {
    return 0;
  }
  const double clamped_lookahead = std::max(0.05, lookahead_distance);
  double accumulated = 0.0;
  size_t lookahead_idx = nearest_idx;
  for (size_t i = nearest_idx; i + 1 < current_path_.size(); ++i) {
    const double dx = current_path_[i + 1].x - current_path_[i].x;
    const double dy = current_path_[i + 1].y - current_path_[i].y;
    accumulated += std::hypot(dx, dy);
    lookahead_idx = i + 1;
    if (accumulated >= clamped_lookahead) {
      break;
    }
  }
  return lookahead_idx;
}

bool AgvSimpleReversePlannerNode::isForwardSegment(const PathPoint & from, const PathPoint & to) const
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-4) {
    return false;
  }
  const double heading_x = std::cos(from.yaw);
  const double heading_y = std::sin(from.yaw);
  const double signed_motion = dx * heading_x + dy * heading_y;
  return signed_motion > 1e-4;
}

double AgvSimpleReversePlannerNode::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

void AgvSimpleReversePlannerNode::updateReverseDistance(const geometry_msgs::msg::Pose & current_pose)
{
  if (!prev_reverse_odom_initialized_) {
    prev_reverse_odom_x_ = current_pose.position.x;
    prev_reverse_odom_y_ = current_pose.position.y;
    prev_reverse_odom_initialized_ = true;
    return;
  }
  const double dx = current_pose.position.x - prev_reverse_odom_x_;
  const double dy = current_pose.position.y - prev_reverse_odom_y_;
  reverse_driven_distance_ += std::hypot(dx, dy);
  prev_reverse_odom_x_ = current_pose.position.x;
  prev_reverse_odom_y_ = current_pose.position.y;
}

void AgvSimpleReversePlannerNode::setState(State state, const std::string & reason)
{
  if (state_ == state) {
    return;
  }
  state_ = state;
  RCLCPP_INFO(get_logger(), "state -> %u: %s", static_cast<unsigned>(state_), reason.c_str());
}

void AgvSimpleReversePlannerNode::publishTrajectoryAndMarkers()
{
  if (!current_trajectory_ || current_path_.empty()) {
    return;
  }

  const auto stamp = now();
  const bool should_publish =
    !has_last_publish_time_ ||
    (publish_rate_ <= 0.0) ||
    ((stamp - last_publish_time_).seconds() >= (1.0 / std::max(0.1, publish_rate_)));
  if (!should_publish) {
    return;
  }
  has_last_publish_time_ = true;
  last_publish_time_ = stamp;

  current_trajectory_->header.stamp = stamp;
  traj_pub_->publish(*current_trajectory_);

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = "map";
  marker.ns = "simple_reverse_path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 0.2;
  marker.color.g = 0.9;
  marker.color.b = 0.2;
  for (const auto & pt : current_path_) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.1;
    marker.points.push_back(p);
  }
  marker_array.markers.push_back(marker);
  marker_pub_->publish(marker_array);
}

void AgvSimpleReversePlannerNode::publishControlCmd(
  double steering_angle, double acceleration, double target_velocity)
{
  autoware_control_msgs::msg::Control cmd;
  cmd.stamp = now();
  cmd.lateral.steering_tire_angle = static_cast<float>(steering_angle);
  cmd.lateral.steering_tire_rotation_rate = 0.0F;
  cmd.longitudinal.velocity = static_cast<float>(std::max(0.0, target_velocity));
  cmd.longitudinal.acceleration = static_cast<float>(acceleration);
  cmd.longitudinal.jerk = 0.0F;
  control_cmd_pub_->publish(cmd);
}

void AgvSimpleReversePlannerNode::publishGearCmd(bool is_reverse)
{
  autoware_vehicle_msgs::msg::GearCommand cmd;
  cmd.stamp = now();
  if (state_ == State::GOAL_REACHED || state_ == State::DIST_LIMIT || state_ == State::ABORT) {
    cmd.command = autoware_vehicle_msgs::msg::GearCommand::PARK;
  } else if (is_reverse) {
    cmd.command = autoware_vehicle_msgs::msg::GearCommand::REVERSE;
  } else {
    cmd.command = autoware_vehicle_msgs::msg::GearCommand::DRIVE;
  }
  gear_cmd_pub_->publish(cmd);
}

void AgvSimpleReversePlannerNode::publishIndicatorCmds(bool is_reverse, bool is_stopped)
{
  autoware_vehicle_msgs::msg::TurnIndicatorsCommand turn_cmd;
  turn_cmd.stamp = now();
  turn_cmd.command = autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND;
  turn_indicator_cmd_pub_->publish(turn_cmd);

  autoware_vehicle_msgs::msg::HazardLightsCommand hazard_cmd;
  hazard_cmd.stamp = now();
  hazard_cmd.command =
    (is_reverse || is_stopped) ? autoware_vehicle_msgs::msg::HazardLightsCommand::ENABLE
                               : autoware_vehicle_msgs::msg::HazardLightsCommand::DISABLE;
  hazard_light_cmd_pub_->publish(hazard_cmd);
}

}  // namespace agv_simple_reverse_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agv_simple_reverse_planner::AgvSimpleReversePlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
