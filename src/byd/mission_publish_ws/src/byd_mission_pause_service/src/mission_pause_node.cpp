#include "mission_pause_node.hpp"

#include <chrono>

MissionPauseNode::MissionPauseNode()
: Node("mission_pause_node")
{
  const auto qos_transient = rclcpp::QoS(1).transient_local();

  route_state_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", qos_transient,
    std::bind(&MissionPauseNode::route_state_callback, this, std::placeholders::_1));

  is_paused_sub_ = create_subscription<tier4_control_msgs::msg::IsPaused>(
    "/control/vehicle_cmd_gate/is_paused", qos_transient,
    std::bind(&MissionPauseNode::is_paused_callback, this, std::placeholders::_1));

  set_pause_client_ = create_client<tier4_control_msgs::srv::SetPause>(
    "/control/vehicle_cmd_gate/set_pause");

  pause_srv_ = create_service<std_srvs::srv::Trigger>(
    "/byd/mission/pause",
    std::bind(&MissionPauseNode::on_pause, this, std::placeholders::_1, std::placeholders::_2));

  resume_srv_ = create_service<std_srvs::srv::Trigger>(
    "/byd/mission/resume",
    std::bind(&MissionPauseNode::on_resume, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "Mission pause node started. Services: /byd/mission/pause, /byd/mission/resume");
}

void MissionPauseNode::route_state_callback(
  const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
  route_state_received_ = true;
  route_state_ = msg->state;
}

void MissionPauseNode::is_paused_callback(const tier4_control_msgs::msg::IsPaused::SharedPtr msg)
{
  is_paused_received_ = true;
  is_paused_ = msg->data;
}

bool MissionPauseNode::call_set_pause(bool pause, std::string & error_message)
{
  if (!set_pause_client_->wait_for_service(std::chrono::seconds(3))) {
    error_message = "Service /control/vehicle_cmd_gate/set_pause is not available";
    return false;
  }

  auto request = std::make_shared<tier4_control_msgs::srv::SetPause::Request>();
  request->pause = pause;

  auto future = set_pause_client_->async_send_request(request);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    error_message = "set_pause service call timed out";
    return false;
  }

  const auto response = future.get();
  if (!response->status.success) {
    error_message = response->status.message.empty() ? "set_pause failed" : response->status.message;
    return false;
  }

  is_paused_ = pause;
  return true;
}

void MissionPauseNode::on_pause(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (route_state_received_ && route_state_ != ROUTE_STATE_SET) {
    response->success = false;
    response->message = "No active route, cannot pause";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (is_paused_received_ && is_paused_) {
    response->success = true;
    response->message = "Already paused";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return;
  }

  std::string error_message;
  if (!call_set_pause(true, error_message)) {
    response->success = false;
    response->message = error_message;
    RCLCPP_ERROR(get_logger(), "Pause failed: %s", error_message.c_str());
    return;
  }

  response->success = true;
  response->message = "Mission paused";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void MissionPauseNode::on_resume(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (is_paused_received_ && !is_paused_) {
    response->success = true;
    response->message = "Not paused";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return;
  }

  std::string error_message;
  if (!call_set_pause(false, error_message)) {
    response->success = false;
    response->message = error_message;
    RCLCPP_ERROR(get_logger(), "Resume failed: %s", error_message.c_str());
    return;
  }

  response->success = true;
  response->message = "Mission resumed, continuing original route";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionPauseNode>());
  rclcpp::shutdown();
  return 0;
}
