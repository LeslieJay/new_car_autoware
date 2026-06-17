/**
 * ROS2 C++ Lifecycle Node for initializing pose via /api/localization/initialize service.
 *
 * This node calls the Autoware AD API service to initialize the vehicle's pose.
 * Supports setting initial pose from configuration parameters.
 * Lifecycle node that stays active after initialization.
 */

#include "initialize_pose_node.hpp"
#include <lifecycle_msgs/msg/transition.hpp>

InitializePoseNode::InitializePoseNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("initialize_pose_node", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<double>("x", 0.0);
  this->declare_parameter<double>("y", 0.0);
  this->declare_parameter<double>("z", 0.0);
  this->declare_parameter<double>("orientation_x", 0.0);
  this->declare_parameter<double>("orientation_y", 0.0);
  this->declare_parameter<double>("orientation_z", 0.0);
  this->declare_parameter<double>("orientation_w", 1.0);
  this->declare_parameter<bool>("auto_initialize", false);

  RCLCPP_INFO(this->get_logger(), "Lifecycle node initialized");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InitializePoseNode::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "on_configure() is called from state %s.",
    state.label().c_str());

  // Create service client
  client_ = this->create_client<autoware_adapi_v1_msgs::srv::InitializeLocalization>(
    "/api/localization/initialize");

  RCLCPP_INFO(this->get_logger(), "Waiting for /api/localization/initialize service...");

  // Wait for service to be available
  int retry_count = 0;
  while (!client_->wait_for_service(std::chrono::seconds(1)) && retry_count < 10) {
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting... (attempt %d/10)", ++retry_count);
  }

  if (retry_count >= 10) {
    RCLCPP_ERROR(this->get_logger(), "Service not available after 10 attempts");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "Service is available!");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InitializePoseNode::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "on_activate() is called from state %s.",
    state.label().c_str());

  // Get parameters
  bool auto_initialize = this->get_parameter("auto_initialize").as_bool();

  // Send initial pose
  if (auto_initialize) {
    send_auto_initialize();
  } else {
    send_initialize_pose();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InitializePoseNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "on_deactivate() is called from state %s.",
    state.label().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
InitializePoseNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    this->get_logger(),
    "on_cleanup() is called from state %s.",
    state.label().c_str());

  client_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void InitializePoseNode::send_initialize_pose()
{
  // Get pose parameters
  std::string frame_id = this->get_parameter("frame_id").as_string();
  double x = this->get_parameter("x").as_double();
  double y = this->get_parameter("y").as_double();
  double z = this->get_parameter("z").as_double();
  double ox = this->get_parameter("orientation_x").as_double();
  double oy = this->get_parameter("orientation_y").as_double();
  double oz = this->get_parameter("orientation_z").as_double();
  double ow = this->get_parameter("orientation_w").as_double();

  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::InitializeLocalization::Request>();

  // Create pose with covariance
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.frame_id = frame_id;
  pose_msg.header.stamp = this->get_clock()->now();

  pose_msg.pose.pose.position.x = x;
  pose_msg.pose.pose.position.y = y;
  pose_msg.pose.pose.position.z = z;
  pose_msg.pose.pose.orientation.x = ox;
  pose_msg.pose.pose.orientation.y = oy;
  pose_msg.pose.pose.orientation.z = oz;
  pose_msg.pose.pose.orientation.w = ow;

  // Set default covariance (6x6 matrix)
  // Position covariance (x, y, z)
  pose_msg.pose.covariance[0] = 1.0;    // x
  pose_msg.pose.covariance[7] = 1.0;    // y
  pose_msg.pose.covariance[14] = 0.01;  // z
  // Orientation covariance (roll, pitch, yaw)
  pose_msg.pose.covariance[21] = 0.01;  // roll
  pose_msg.pose.covariance[28] = 0.01;  // pitch
  pose_msg.pose.covariance[35] = 10.0;  // yaw

  request->pose.push_back(pose_msg);

  RCLCPP_INFO(
    this->get_logger(),
    "Sending initial pose: position=(%.2f, %.2f, %.2f), orientation=(%.4f, %.4f, %.4f, %.4f)",
    x, y, z, ox, oy, oz, ow);

  // Send request asynchronously
  auto future = client_->async_send_request(
    request,
    std::bind(&InitializePoseNode::initialize_response_callback, this, std::placeholders::_1));
}

void InitializePoseNode::send_auto_initialize()
{
  auto request = std::make_shared<autoware_adapi_v1_msgs::srv::InitializeLocalization::Request>();
  // Empty pose list means auto-initialize using GNSS
  request->pose = {};

  RCLCPP_INFO(this->get_logger(), "Sending auto-initialize request (using GNSS position)...");

  // Send request asynchronously
  auto future = client_->async_send_request(
    request,
    std::bind(&InitializePoseNode::initialize_response_callback, this, std::placeholders::_1));
}

void InitializePoseNode::initialize_response_callback(
  rclcpp::Client<autoware_adapi_v1_msgs::srv::InitializeLocalization>::SharedFuture future)
{
  try {
    auto response = future.get();
    if (response->status.success) {
      RCLCPP_INFO(this->get_logger(), "Pose initialized successfully!");
    } else {
      auto error_code = response->status.code;
      auto error_message = response->status.message;
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to initialize pose: code=%d, message=%s",
        error_code,
        error_message.c_str());
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
  }

  // Keep the node active - do not shutdown
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<InitializePoseNode>();

  // Manually iterate through lifecycle states
  // This simulates the state machine transitions
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // Transition from UNCONFIGURED to CONFIGURED
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  
  // Transition from CONFIGURED to ACTIVE
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  // Spin the executor (this will keep the node running)
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
