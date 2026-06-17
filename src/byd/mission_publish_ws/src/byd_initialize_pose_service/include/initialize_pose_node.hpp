#ifndef BYD_INITIALIZE_POSE_SERVICE__INITIALIZE_POSE_NODE_HPP_
#define BYD_INITIALIZE_POSE_SERVICE__INITIALIZE_POSE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>

class InitializePoseNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit InitializePoseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state);

private:
  void send_initialize_pose();
  void send_auto_initialize();
  void initialize_response_callback(
    rclcpp::Client<autoware_adapi_v1_msgs::srv::InitializeLocalization>::SharedFuture future);

  rclcpp::Client<autoware_adapi_v1_msgs::srv::InitializeLocalization>::SharedPtr client_;
};

#endif  // BYD_INITIALIZE_POSE_SERVICE__INITIALIZE_POSE_NODE_HPP_
