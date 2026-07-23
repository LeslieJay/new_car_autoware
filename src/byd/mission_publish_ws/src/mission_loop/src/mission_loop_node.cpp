#include "mission_loop/arrival_gate.hpp"
#include "mission_loop/arrival_recorder.hpp"

#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace autoware
{
namespace mission_loop
{

struct MissionPoint
{
  std::string name;
  double x;
  double y;
  double z;
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
  double wait_time;
};

class MissionLoopNode : public rclcpp::Node
{
public:
  explicit MissionLoopNode(const rclcpp::NodeOptions & options)
  : Node("mission_loop", options)
  {
    declareParameters();

    loadParameters();
    loadMissionPoints();

    if (mission_points_.empty()) {
      RCLCPP_FATAL(get_logger(), "No mission points loaded!");
      rclcpp::shutdown();
      return;
    }

    recorder_ = std::make_unique<ArrivalRecorder>(arrival_pose_output_dir_);

    auto qos = rclcpp::QoS(1).transient_local().reliable();

    goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
      "/planning/mission_planning/goal", qos);


    localization_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", 10,
      std::bind(&MissionLoopNode::localizationCallback, this, std::placeholders::_1));

    autoware_state_sub_ =
      create_subscription<autoware_system_msgs::msg::AutowareState>(
      "/byd/autoware/state", rclcpp::QoS{10},
      std::bind(&MissionLoopNode::autowareStateCallback, this, std::placeholders::_1));


    timer_ = create_wall_timer(
      200ms, std::bind(&MissionLoopNode::timerCallback, this));

    publishGoal();
    RCLCPP_INFO(get_logger(), "Mission loop node started");
  }

private:
  enum class State
  {
    DRIVING,
    WAIT
  };

  /* ---------------- parameters ---------------- */

  void declareParameters()
  {
    declare_parameter<std::string>(
      "arrival_pose_output_dir", "/home/nvidia/autoware/log/mission_arrivals");
    declare_parameter<std::vector<std::string>>("points", std::vector<std::string>{});
  }

  void loadParameters()
  {
    get_parameter("arrival_pose_output_dir", arrival_pose_output_dir_);
  }

  void loadMissionPoints()
  {
    std::vector<std::string> names;
    get_parameter("points", names);

    for (const auto & name : names) {

      // ---- declare ----
      declare_parameter<double>("points." + name + ".x");
      declare_parameter<double>("points." + name + ".y");
      declare_parameter<double>("points." + name + ".z");        // ← 新增
      declare_parameter<double>("points." + name + ".orientation_x");
      declare_parameter<double>("points." + name + ".orientation_y");
      declare_parameter<double>("points." + name + ".orientation_z");
      declare_parameter<double>("points." + name + ".orientation_w");
      declare_parameter<double>("points." + name + ".wait_time");

      MissionPoint p;
      p.name = name;

      get_parameter("points." + name + ".x", p.x);
      get_parameter("points." + name + ".y", p.y);
      get_parameter("points." + name + ".z", p.z);               // ← 新增
      get_parameter("points." + name + ".orientation_x", p.orientation_x);
      get_parameter("points." + name + ".orientation_y", p.orientation_y);
      get_parameter("points." + name + ".orientation_z", p.orientation_z);
      get_parameter("points." + name + ".orientation_w", p.orientation_w);
      get_parameter("points." + name + ".wait_time", p.wait_time);

      mission_points_.push_back(p);

      RCLCPP_INFO(
        get_logger(),
        "Load point %s (%.2f, %.2f, %.2f), wait=%.1f",
        p.name.c_str(), p.x, p.y, p.z, p.wait_time);
    }
  }

  /* ---------------- ROS callbacks ---------------- */

  void localizationCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_localization_ = *msg;
  }

  void autowareStateCallback(
    const autoware_system_msgs::msg::AutowareState::SharedPtr msg)
  {
    arrival_gate_.observe(msg->state);
  }


  void timerCallback()
  {
    if (state_ == State::DRIVING) {
      if (arrival_gate_.shouldHandleArrival(latest_localization_.has_value())) {
        recorder_->append(
          mission_points_[current_idx_].name, now(), latest_localization_.value());
        arrival_gate_.markHandled();
        wait_start_time_ = now();
        state_ = State::WAIT;
        RCLCPP_INFO(
          get_logger(), "Arrived point %s; localization appended to %s",
          mission_points_[current_idx_].name.c_str(), recorder_->filePath().c_str());
      } else if (arrival_gate_.isArrivalObserved() && !latest_localization_.has_value()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Arrival state received but localization is unavailable");
      }
    } else if (state_ == State::WAIT) {
      double wait_elapsed =
        (now() - wait_start_time_).seconds();

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[WAIT] goal=%s  wait_elapsed=%.2f / %.1f s",
        mission_points_[current_idx_].name.c_str(),
        wait_elapsed, mission_points_[current_idx_].wait_time);

      if (wait_elapsed >
        mission_points_[current_idx_].wait_time)
      {
        RCLCPP_INFO(
          get_logger(),
          "[WAIT] Wait done. Advancing from %s → %s",
          mission_points_[current_idx_].name.c_str(),
          mission_points_[(current_idx_ + 1) % mission_points_.size()].name.c_str());
        current_idx_ = (current_idx_ + 1) % mission_points_.size();
        publishGoal();
      }
    }
  }

  /* ---------------- logic ---------------- */

  void publishGoal()
  {
    auto goal = makeGoal(mission_points_[current_idx_]);
    goal.header.stamp = now();
    goal_pub_->publish(goal);

    RCLCPP_INFO(
      get_logger(),
      "Publish goal %s (%.2f, %.2f)",
      mission_points_[current_idx_].name.c_str(),
      mission_points_[current_idx_].x,
      mission_points_[current_idx_].y);

    arrival_gate_.reset();
    state_ = State::DRIVING;
  }

  geometry_msgs::msg::PoseStamped makeGoal(const MissionPoint & p) const
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";

    goal.pose.position.x = p.x;
    goal.pose.position.y = p.y;
    goal.pose.position.z = p.z;   // ← 新增

    goal.pose.orientation.x = p.orientation_x;
    goal.pose.orientation.y = p.orientation_y;
    goal.pose.orientation.z = p.orientation_z;
    goal.pose.orientation.w = p.orientation_w;

    return goal;
  }


  /* ---------------- ROS objects ---------------- */

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr
    autoware_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;


  /* ---------------- data ---------------- */

  std::vector<MissionPoint> mission_points_;
  size_t current_idx_{0};

  std::optional<nav_msgs::msg::Odometry> latest_localization_;
  ArrivalGate arrival_gate_;
  std::unique_ptr<ArrivalRecorder> recorder_;
  std::string arrival_pose_output_dir_;

  State state_{State::DRIVING};

  rclcpp::Time wait_start_time_;
};

}  // namespace mission_loop
}  // namespace autoware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_loop::MissionLoopNode)
