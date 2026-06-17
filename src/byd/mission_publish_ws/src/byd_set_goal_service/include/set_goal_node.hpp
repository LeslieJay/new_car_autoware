#ifndef BYD_SET_GOAL_SERVICE__SET_GOAL_NODE_HPP_
#define BYD_SET_GOAL_SERVICE__SET_GOAL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <memory>
#include <vector>
#include <map>

struct Goal
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

class SetGoalNode : public rclcpp::Node
{
public:
  SetGoalNode();

private:
  // Routing state constants
  static constexpr uint8_t ROUTE_STATE_UNKNOWN = 0;
  static constexpr uint8_t ROUTE_STATE_UNSET = 1;
  static constexpr uint8_t ROUTE_STATE_SET = 2;
  static constexpr uint8_t ROUTE_STATE_ARRIVED = 3;
  static constexpr uint8_t ROUTE_STATE_CHANGING = 4;

  // Methods
  void load_goals(const std::vector<std::string> & goal_names);
  void route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
  void on_goal_arrived();
  void on_wait_complete();
  void clear_route_and_send_next();
  void send_current_goal();
  void clear_route_before_send_current();
  void clear_route_response_for_send_callback(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedFuture future);
  void do_send_current_goal();
  void send_goal(
    double x, double y, double z,
    double ox, double oy, double oz, double ow,
    const std::string & frame_id, bool allow_goal_modification);
  void goal_response_callback(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedFuture future);

  // Member variables
  std::vector<Goal> goals_;
  size_t current_goal_index_;
  bool is_waiting_;
  rclcpp::TimerBase::SharedPtr wait_timer_;
  uint8_t route_state_;
  bool goal_sent_;
  bool pending_send_goal_;  // wait for UNSET state before sending next goal

  // ROS2 clients
  rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedPtr client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr clear_route_client_;

  // ROS2 subscription
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_sub_;
};

#endif  // byd_set_goal_service__SET_GOAL_NODE_HPP_
