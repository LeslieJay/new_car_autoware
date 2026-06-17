#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <optional>
#include <chrono>

using namespace std::chrono_literals;

namespace autoware {
namespace mission_loop {

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

    auto qos = rclcpp::QoS(1).transient_local().reliable();

    goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(
        "/planning/mission_planning/goal", qos);
    

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", 10,
        std::bind(&MissionLoopNode::stateCallback, this, std::placeholders::_1));
    

    timer_ = create_wall_timer(
      200ms, std::bind(&MissionLoopNode::timerCallback, this));

    if (mission_points_.empty()) {
      RCLCPP_FATAL(get_logger(), "No mission points loaded!");
      rclcpp::shutdown();
      return;
    }

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
    declare_parameter<double>("arrive_distance_threshold", 0.8);
    declare_parameter<double>("arrive_speed_threshold", 0.05);
    declare_parameter<double>("arrive_time_threshold", 2.0);

    declare_parameter<std::vector<std::string>>("points", {});
  }

  void loadParameters()
  {
    get_parameter("arrive_distance_threshold", arrive_distance_th_);
    get_parameter("arrive_speed_threshold", arrive_speed_th_);
    get_parameter("arrive_time_threshold", arrive_time_th_);
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

  void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    current_speed_ = std::abs(msg->twist.twist.linear.x);
  }
  

  void timerCallback()
  {
    if (!current_pose_.has_value()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "[timerCallback] current_pose_ not available yet, waiting...");
      return;
    }

    const double dist = distanceToGoal();
    const double spd  = current_speed_;

    if (state_ == State::DRIVING) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[DRIVING] goal=%s  dist=%.3f m (thr=%.2f)  speed=%.4f m/s (thr=%.3f)  arrived=%s",
        mission_points_[current_idx_].name.c_str(),
        dist, arrive_distance_th_, spd, arrive_speed_th_,
        (dist < arrive_distance_th_ && spd < arrive_speed_th_) ? "YES" : "NO");

      if (isArrived()) {
        if (!arrived_start_time_) {
          arrived_start_time_ = now();
          RCLCPP_INFO(get_logger(),
            "[DRIVING] Arrived condition first met at goal=%s, starting %.1fs timer",
            mission_points_[current_idx_].name.c_str(), arrive_time_th_);
        }

        double elapsed =
          (now() - arrived_start_time_.value()).seconds();

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "[DRIVING] Arrived hold timer: %.2f / %.1f s",
          elapsed, arrive_time_th_);

        if (elapsed > arrive_time_th_) {
          RCLCPP_INFO(
            get_logger(), "Arrived point %s",
            mission_points_[current_idx_].name.c_str());

          wait_start_time_ = now();
          state_ = State::WAIT;
        }
      } else {
        if (arrived_start_time_) {
          RCLCPP_WARN(get_logger(),
            "[DRIVING] Arrived condition LOST for goal=%s  dist=%.3f  speed=%.4f — resetting timer",
            mission_points_[current_idx_].name.c_str(), dist, spd);
        }
        arrived_start_time_.reset();
      }
    }

    else if (state_ == State::WAIT) {
      double wait_elapsed =
        (now() - wait_start_time_).seconds();

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "[WAIT] goal=%s  wait_elapsed=%.2f / %.1f s",
        mission_points_[current_idx_].name.c_str(),
        wait_elapsed, mission_points_[current_idx_].wait_time);

      if (wait_elapsed >
          mission_points_[current_idx_].wait_time) {
        RCLCPP_INFO(get_logger(),
          "[WAIT] Wait done. Advancing from %s → %s",
          mission_points_[current_idx_].name.c_str(),
          mission_points_[(current_idx_ + 1) % mission_points_.size()].name.c_str());
        current_idx_ = (current_idx_ + 1) % mission_points_.size();
        publishGoal();
      }
    }
  }

  /* ---------------- logic ---------------- */

  bool isArrived() const
  {
    return distanceToGoal() < arrive_distance_th_ &&
           current_speed_ < arrive_speed_th_;
  }

  double distanceToGoal() const
  {
    const auto & g = mission_points_[current_idx_];
    double dx = current_pose_->position.x - g.x;
    double dy = current_pose_->position.y - g.y;
    return std::sqrt(dx * dx + dy * dy);
  }

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

    arrived_start_time_.reset();
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;


  /* ---------------- data ---------------- */

  std::vector<MissionPoint> mission_points_;
  size_t current_idx_{0};

  std::optional<geometry_msgs::msg::Pose> current_pose_;
  double current_speed_{0.0};

  State state_{State::DRIVING};

  std::optional<rclcpp::Time> arrived_start_time_;
  rclcpp::Time wait_start_time_;

  /* ---------------- thresholds ---------------- */

  double arrive_distance_th_{0.8};
  double arrive_speed_th_{0.05};
  double arrive_time_th_{2.0};
};

}  // namespace mission_loop
}  // namespace autoware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_loop::MissionLoopNode)

