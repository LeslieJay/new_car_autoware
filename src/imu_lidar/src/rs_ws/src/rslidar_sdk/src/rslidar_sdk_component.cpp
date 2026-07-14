/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved
*********************************************************************************************************************/

#include "manager/node_manager.hpp"

#include <rs_driver/macro/version.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace robosense
{
namespace lidar
{

class RslidarSdkNode : public rclcpp::Node
{
public:
  explicit RslidarSdkNode(const rclcpp::NodeOptions & options)
  : Node("rslidar_sdk_node", options)
  {
    RCLCPP_INFO(
      get_logger(), "RSLidar_SDK Version: v%d.%d.%d", RSLIDAR_VERSION_MAJOR, RSLIDAR_VERSION_MINOR,
      RSLIDAR_VERSION_PATCH);

    // Defer driver init so shared_from_this() is valid after construction.
    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(0),
      std::bind(&RslidarSdkNode::initDriver, this));
  }

  ~RslidarSdkNode() override
  {
    if (init_timer_) {
      init_timer_->cancel();
    }
    if (node_manager_) {
      node_manager_->stop();
      node_manager_.reset();
    }
  }

private:
  void initDriver()
  {
    if (init_timer_) {
      init_timer_->cancel();
      init_timer_.reset();
    }

    std::string config_path = declare_parameter<std::string>("config_path", "");
    if (config_path.empty()) {
      config_path = std::string(PROJECT_PATH) + "/config/config.yaml";
    }

    YAML::Node config;
    try {
      config = YAML::LoadFile(config_path);
      RCLCPP_INFO(get_logger(), "Config loaded from PATH: %s", config_path.c_str());
    } catch (...) {
      RCLCPP_ERROR(
        get_logger(),
        "The format of config file %s is wrong. Please check (e.g. indentation).",
        config_path.c_str());
      return;
    }

    node_manager_ = std::make_shared<NodeManager>();
    node_manager_->setRosNode(shared_from_this());
    node_manager_->init(config);
    node_manager_->start();

    RCLCPP_INFO(get_logger(), "RoboSense-LiDAR-Driver is running.....");
  }

  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<NodeManager> node_manager_;
};

}  // namespace lidar
}  // namespace robosense

RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RslidarSdkNode)
