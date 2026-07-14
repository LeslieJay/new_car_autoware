#ifndef MISSION_PAUSE_NODE_HPP_
#define MISSION_PAUSE_NODE_HPP_

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_control_msgs/msg/is_paused.hpp>
#include <tier4_control_msgs/srv/set_pause.hpp>

class MissionPauseNode : public rclcpp::Node
{
public:
  MissionPauseNode();

private:
  static constexpr uint16_t ROUTE_STATE_SET =
    autoware_adapi_v1_msgs::msg::RouteState::SET;

  void route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
  void is_paused_callback(const tier4_control_msgs::msg::IsPaused::SharedPtr msg);

  void on_pause(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void on_resume(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  bool call_set_pause(bool pause, std::string & error_message);

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
  rclcpp::Client<tier4_control_msgs::srv::SetPause>::SharedPtr set_pause_client_;

  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<tier4_control_msgs::msg::IsPaused>::SharedPtr is_paused_sub_;

  uint16_t route_state_{autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN};
  bool is_paused_{false};
  bool route_state_received_{false};
  bool is_paused_received_{false};
};

#endif  // MISSION_PAUSE_NODE_HPP_
