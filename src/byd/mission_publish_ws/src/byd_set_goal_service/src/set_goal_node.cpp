/**
 * ROS2 C++ Node for setting multiple route points via /api/routing/set_route_points service.
 *
 * This node calls the Autoware AD API service to set route points for autonomous navigation.
 * Supports multiple goals with sequential execution and arrival detection.
 */

#include "set_goal_node.hpp"

SetGoalNode::SetGoalNode()
: rclcpp::Node("set_goal_node"),
  current_goal_index_(0),
  is_waiting_(false),
  route_state_(ROUTE_STATE_UNKNOWN),
  goal_sent_(false),
  pending_send_goal_(false)
{
  // Declare parameters
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<bool>("allow_goal_modification", true);
  this->declare_parameter<bool>("loop", false);
  this->declare_parameter<std::vector<std::string>>("goal_names", std::vector<std::string>{"A", "B"});

  // Get goal names
  auto goal_names = this->get_parameter("goal_names").as_string_array();

  // Declare parameters for each goal
  for (const auto & name : goal_names) {
    this->declare_parameter<double>("goals." + name + ".x", 0.0);
    this->declare_parameter<double>("goals." + name + ".y", 0.0);
    this->declare_parameter<double>("goals." + name + ".z", 0.0);
    this->declare_parameter<double>("goals." + name + ".orientation_x", 0.0);
    this->declare_parameter<double>("goals." + name + ".orientation_y", 0.0);
    this->declare_parameter<double>("goals." + name + ".orientation_z", 0.0);
    this->declare_parameter<double>("goals." + name + ".orientation_w", 1.0);
    this->declare_parameter<double>("goals." + name + ".wait_time", 5.0);
  }

  // Load goals from parameters
  load_goals(goal_names);

  // Create service clients
  client_ = this->create_client<autoware_adapi_v1_msgs::srv::SetRoutePoints>(
    "/api/routing/set_route_points");
  clear_route_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>(
    "/api/routing/clear_route");

  // Subscribe to routing state
  route_state_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", 10,
    std::bind(&SetGoalNode::route_state_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Waiting for services...");

  // Wait for services to be available
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(), "set_route_points service not available, waiting...");
  }
  while (!clear_route_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(), "clear_route service not available, waiting...");
  }

  RCLCPP_INFO(this->get_logger(), "All services are available!");

  // Log loaded goals
  std::string goal_names_str_list;
  for (const auto & goal : goals_) {
    if (!goal_names_str_list.empty()) {
      goal_names_str_list += ", ";
    }
    goal_names_str_list += goal.name;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Loaded %lu goals: %s",
    goals_.size(),
    goal_names_str_list.c_str());

  // Send first goal
  send_current_goal();
}

void SetGoalNode::load_goals(const std::vector<std::string> & goal_names)
{
  for (const auto & name : goal_names) {
    Goal goal;
    goal.name = name;
    goal.x = this->get_parameter("goals." + name + ".x").as_double();
    goal.y = this->get_parameter("goals." + name + ".y").as_double();
    goal.z = this->get_parameter("goals." + name + ".z").as_double();
    goal.orientation_x = this->get_parameter("goals." + name + ".orientation_x").as_double();
    goal.orientation_y = this->get_parameter("goals." + name + ".orientation_y").as_double();
    goal.orientation_z = this->get_parameter("goals." + name + ".orientation_z").as_double();
    goal.orientation_w = this->get_parameter("goals." + name + ".orientation_w").as_double();
    goal.wait_time = this->get_parameter("goals." + name + ".wait_time").as_double();

    goals_.push_back(goal);
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded goal %s: (%.2f, %.2f), wait_time=%.1fs",
      name.c_str(),
      goal.x,
      goal.y,
      goal.wait_time);
  }
}

void SetGoalNode::route_state_callback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
  uint8_t prev_state = route_state_;
  route_state_ = msg->state;

  const std::map<uint8_t, std::string> state_names = {
    {ROUTE_STATE_UNKNOWN, "UNKNOWN"},
    {ROUTE_STATE_UNSET, "UNSET"},
    {ROUTE_STATE_SET, "SET"},
    {ROUTE_STATE_ARRIVED, "ARRIVED"},
    {ROUTE_STATE_CHANGING, "CHANGING"}
  };

  if (prev_state != route_state_) {
    auto prev_state_name = state_names.find(prev_state) != state_names.end() ?
      state_names.at(prev_state) : "?";
    auto curr_state_name = state_names.find(route_state_) != state_names.end() ?
      state_names.at(route_state_) : "?";

    RCLCPP_INFO(
      this->get_logger(),
      "Route state changed: %s -> %s",
      prev_state_name.c_str(),
      curr_state_name.c_str());
  }

  // If waiting for UNSET state to send next goal, trigger it now
  if (route_state_ == ROUTE_STATE_UNSET && pending_send_goal_) {
    pending_send_goal_ = false;
    do_send_current_goal();
    return;
  }

  // Check if arrived at current goal
  if (route_state_ == ROUTE_STATE_ARRIVED && goal_sent_ && !is_waiting_) {
    on_goal_arrived();
  }
}

void SetGoalNode::on_goal_arrived()
{
  if (current_goal_index_ >= goals_.size()) {
    RCLCPP_INFO(this->get_logger(), "Invalid goal index");
    return;
  }

  const auto & goal = goals_[current_goal_index_];
  double wait_time = goal.wait_time;

  RCLCPP_INFO(
    this->get_logger(),
    "Arrived at goal %s! Waiting %.1f seconds before next goal...",
    goal.name.c_str(),
    wait_time);

  is_waiting_ = true;
  goal_sent_ = false;

  // Start wait timer
  wait_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(wait_time * 1000)),
    std::bind(&SetGoalNode::on_wait_complete, this));
}

void SetGoalNode::on_wait_complete()
{
  // Cancel timer
  if (wait_timer_) {
    wait_timer_->cancel();
    wait_timer_ = nullptr;
  }

  is_waiting_ = false;

  // Move to next goal
  current_goal_index_++;
  bool loop = this->get_parameter("loop").as_bool();

  if (current_goal_index_ >= goals_.size()) {
    if (loop) {
      current_goal_index_ = 0;
      RCLCPP_INFO(this->get_logger(), "Looping back to first goal...");
    } else {
      RCLCPP_INFO(this->get_logger(), "All goals completed!");
      return;
    }
  }

  // Clear route before sending next goal
  clear_route_and_send_next();
}

void SetGoalNode::clear_route_and_send_next()
{
  RCLCPP_INFO(this->get_logger(), "Moving to next goal...");
  send_current_goal();
}

void SetGoalNode::send_current_goal()
{
  if (current_goal_index_ >= goals_.size()) {
    RCLCPP_INFO(this->get_logger(), "No more goals to send.");
    return;
  }

  // Clear route before sending any goal
  clear_route_before_send_current();
}

void SetGoalNode::clear_route_before_send_current()
{
  RCLCPP_INFO(this->get_logger(), "Clearing route before sending goal...");
  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ClearRoute::Request>();
  
  auto future = clear_route_client_->async_send_request(
    request,
    std::bind(
      &SetGoalNode::clear_route_response_for_send_callback, this,
      std::placeholders::_1));
}

void SetGoalNode::clear_route_response_for_send_callback(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedFuture future)
{
  try {
    auto response = future.get();
    if (response->status.success) {
      RCLCPP_INFO(
        this->get_logger(),
        "Route cleared successfully! Waiting for UNSET state before sending goal...");
      // Don't send immediately — wait for route state to become UNSET
      pending_send_goal_ = true;
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to clear route: %s, but proceeding anyway",
        response->status.message.c_str());
      // Fallback: send directly if clear failed
      do_send_current_goal();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Clear route service call failed: %s, but proceeding anyway",
      e.what());
    do_send_current_goal();
  }
}

void SetGoalNode::do_send_current_goal()
{
  if (current_goal_index_ >= goals_.size()) {
    RCLCPP_INFO(this->get_logger(), "No more goals to send.");
    return;
  }

  const auto & goal = goals_[current_goal_index_];
  std::string frame_id = this->get_parameter("frame_id").as_string();
  bool allow_goal_modification = this->get_parameter("allow_goal_modification").as_bool();

  RCLCPP_INFO(
    this->get_logger(),
    "Sending goal %lu/%lu: %s",
    current_goal_index_ + 1,
    goals_.size(),
    goal.name.c_str());

  send_goal(
    goal.x, goal.y, goal.z,
    goal.orientation_x, goal.orientation_y,
    goal.orientation_z, goal.orientation_w,
    frame_id, allow_goal_modification);
}

void SetGoalNode::send_goal(
  double x, double y, double z,
  double ox, double oy, double oz, double ow,
  const std::string & frame_id, bool allow_goal_modification)
{
  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::SetRoutePoints::Request>();

  // Set header
  request->header.frame_id = frame_id;
  request->header.stamp = this->get_clock()->now();

  // Set route option
  request->option.allow_goal_modification = allow_goal_modification;

  // Set goal pose
  request->goal.position.x = x;
  request->goal.position.y = y;
  request->goal.position.z = z;
  request->goal.orientation.x = ox;
  request->goal.orientation.y = oy;
  request->goal.orientation.z = oz;
  request->goal.orientation.w = ow;

  // waypoints is empty (direct navigation to goal)
  request->waypoints = {};

  RCLCPP_INFO(
    this->get_logger(),
    "Sending goal: position=(%.2f, %.2f, %.2f), orientation=(%.4f, %.4f, %.4f, %.4f)",
    x, y, z, ox, oy, oz, ow);

  // Send request asynchronously
  auto future = client_->async_send_request(
    request,
    std::bind(&SetGoalNode::goal_response_callback, this, std::placeholders::_1));
}

void SetGoalNode::goal_response_callback(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoutePoints>::SharedFuture future)
{
  try {
    auto response = future.get();
    if (response->status.success) {
      RCLCPP_INFO(this->get_logger(), "Goal set successfully!");
      goal_sent_ = true;
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to set goal: %s",
        response->status.message.c_str());
      goal_sent_ = false;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    goal_sent_ = false;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetGoalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
