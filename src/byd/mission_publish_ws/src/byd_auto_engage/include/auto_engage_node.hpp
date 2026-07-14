#ifndef BYD_AUTO_ENGAGE__AUTO_ENGAGE_NODE_HPP_
#define BYD_AUTO_ENGAGE__AUTO_ENGAGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

class AutoEngageNode : public rclcpp::Node
{
public:
  AutoEngageNode();

private:
  static constexpr uint8_t ROUTE_STATE_SET = 2;
  static constexpr uint8_t MODE_AUTONOMOUS = 2;
  static constexpr uint8_t MODE_LOCAL = 3;

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void reverse_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
  void operation_mode_callback(
    const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg);
  void on_retry_timer();
  void try_auto_engage();
  void try_switch_to_local();

  void call_change_to_autonomous();
  void call_change_to_local();
  void call_enable_autoware_control();
  void call_enable_autoware_control_for_local();

  void on_change_to_autonomous_response(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future);
  void on_change_to_local_response(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future);
  void on_enable_autoware_control_response(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future);
  void on_enable_autoware_control_for_local_response(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future);

  bool enabled_;
  bool require_autonomous_available_;
  bool goal_pending_;
  bool local_mode_pending_;
  bool auto_engage_in_progress_;
  bool local_mode_in_progress_;
  uint8_t route_state_;
  uint8_t operation_mode_;
  bool is_autoware_control_enabled_;
  bool is_autonomous_mode_available_;
  double retry_period_sec_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reverse_goal_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    operation_mode_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    change_to_autonomous_client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    change_to_local_client_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    enable_autoware_control_client_;
};

#endif  // BYD_AUTO_ENGAGE__AUTO_ENGAGE_NODE_HPP_
