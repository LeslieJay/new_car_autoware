#include "auto_engage_node.hpp"

AutoEngageNode::AutoEngageNode()
: Node("auto_engage_node"),
  enabled_(true),
  goal_pending_(false),
  auto_engage_in_progress_(false),
  route_state_(0),
  operation_mode_(0),
  is_autoware_control_enabled_(false),
  is_autonomous_mode_available_(false)
{
  declare_parameter<bool>("enabled", true);
  enabled_ = get_parameter("enabled").as_bool();

  const auto qos_transient = rclcpp::QoS(1).transient_local();

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/planning/mission_planning/goal", qos_transient,
    std::bind(&AutoEngageNode::goal_callback, this, std::placeholders::_1));

  route_state_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", qos_transient,
    std::bind(&AutoEngageNode::route_state_callback, this, std::placeholders::_1));

  operation_mode_sub_ = create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "/api/operation_mode/state", qos_transient,
    std::bind(&AutoEngageNode::operation_mode_callback, this, std::placeholders::_1));

  change_to_autonomous_client_ =
    create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
      "/api/operation_mode/change_to_autonomous");

  enable_autoware_control_client_ =
    create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
      "/api/operation_mode/enable_autoware_control");

  RCLCPP_INFO(
    get_logger(),
    "Auto engage node started (enabled=%s). Waiting for goal on "
    "/planning/mission_planning/goal and route on /api/routing/state",
    enabled_ ? "true" : "false");
}

void AutoEngageNode::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!enabled_) {
    return;
  }

  goal_pending_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Goal received: (%.2f, %.2f), waiting for route SET...",
    msg->pose.position.x, msg->pose.position.y);

  try_auto_engage();
}

void AutoEngageNode::route_state_callback(
  const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
  const uint8_t prev_state = route_state_;
  route_state_ = msg->state;

  if (prev_state != route_state_) {
    RCLCPP_INFO(get_logger(), "Route state changed: %u -> %u", prev_state, route_state_);
  }

  if (route_state_ == ROUTE_STATE_SET) {
    try_auto_engage();
  } else if (route_state_ != ROUTE_STATE_SET) {
    goal_pending_ = false;
    auto_engage_in_progress_ = false;
  }
}

void AutoEngageNode::operation_mode_callback(
  const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg)
{
  const bool prev_available = is_autonomous_mode_available_;
  operation_mode_ = msg->mode;
  is_autoware_control_enabled_ = msg->is_autoware_control_enabled;
  is_autonomous_mode_available_ = msg->is_autonomous_mode_available;

  if (!prev_available && is_autonomous_mode_available_) {
    RCLCPP_INFO(get_logger(), "Autonomous mode is now available, retrying auto engage...");
    try_auto_engage();
  }
}

void AutoEngageNode::try_auto_engage()
{
  if (!enabled_ || !goal_pending_ || route_state_ != ROUTE_STATE_SET || auto_engage_in_progress_) {
    return;
  }

  if (operation_mode_ == MODE_AUTONOMOUS && is_autoware_control_enabled_) {
    RCLCPP_INFO(get_logger(), "Already in AUTONOMOUS mode with autoware control enabled");
    goal_pending_ = false;
    return;
  }

  if (!is_autonomous_mode_available_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Waiting for autonomous mode to become available (planning/diagnostics not ready yet)...");
    return;
  }

  auto_engage_in_progress_ = true;
  call_change_to_autonomous();
}

void AutoEngageNode::call_change_to_autonomous()
{
  if (!change_to_autonomous_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service /api/operation_mode/change_to_autonomous is not ready");
    auto_engage_in_progress_ = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "Calling /api/operation_mode/change_to_autonomous ...");
  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
  change_to_autonomous_client_->async_send_request(
    request,
    std::bind(&AutoEngageNode::on_change_to_autonomous_response, this, std::placeholders::_1));
}

void AutoEngageNode::call_enable_autoware_control()
{
  if (!enable_autoware_control_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service /api/operation_mode/enable_autoware_control is not ready");
    auto_engage_in_progress_ = false;
    return;
  }

  RCLCPP_INFO(get_logger(), "Calling /api/operation_mode/enable_autoware_control ...");
  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
  enable_autoware_control_client_->async_send_request(
    request,
    std::bind(&AutoEngageNode::on_enable_autoware_control_response, this, std::placeholders::_1));
}

void AutoEngageNode::on_change_to_autonomous_response(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future)
{
  try {
    const auto response = future.get();
    if (response->status.success) {
      RCLCPP_INFO(get_logger(), "change_to_autonomous succeeded");
      call_enable_autoware_control();
    } else {
      RCLCPP_WARN(
        get_logger(), "change_to_autonomous failed: %s (will retry when mode becomes available)",
        response->status.message.c_str());
      auto_engage_in_progress_ = false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "change_to_autonomous service call failed: %s", e.what());
    auto_engage_in_progress_ = false;
  }
}

void AutoEngageNode::on_enable_autoware_control_response(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedFuture future)
{
  try {
    const auto response = future.get();
    if (response->status.success) {
      RCLCPP_INFO(get_logger(), "enable_autoware_control succeeded, auto engage complete");
      goal_pending_ = false;
    } else {
      RCLCPP_ERROR(
        get_logger(), "enable_autoware_control failed: %s", response->status.message.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "enable_autoware_control service call failed: %s", e.what());
  }
  auto_engage_in_progress_ = false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoEngageNode>());
  rclcpp::shutdown();
  return 0;
}
