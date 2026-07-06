#ifndef GOAL_SERVICE_SERVER_HPP
#define GOAL_SERVICE_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// 修改头文件包含方式
#include "ref_slam_interface/srv/use_trajectory.hpp"
#include "agv_config.h"

// 使用正确的命名空间
using UseTrajectory = ref_slam_interface::srv::UseTrajectory;

class GoalServiceServer : public rclcpp::Node
{
public:
  GoalServiceServer();
  ~GoalServiceServer() = default;

private:
  void handle_goal_request(
    const std::shared_ptr<UseTrajectory::Request> request,
    std::shared_ptr<UseTrajectory::Response> response);

  rclcpp::Service<UseTrajectory>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
};

#endif // GOAL_SERVICE_SERVER_HPP