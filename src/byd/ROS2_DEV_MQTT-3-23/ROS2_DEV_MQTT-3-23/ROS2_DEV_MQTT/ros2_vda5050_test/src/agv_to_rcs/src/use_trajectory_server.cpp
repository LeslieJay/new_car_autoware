// 原版服务用于轨迹验证，这个是自己写的，暂时没用上
#include "use_trajectory_server.h"

GoalServiceServer::GoalServiceServer() : Node("goal_service_server")
{
  service_ = this->create_service<UseTrajectory>(
    // 与客户端初始化的服务名匹配
    "use_trajectory",
    // 回调函数
    std::bind(&GoalServiceServer::handle_goal_request, this, std::placeholders::_1, std::placeholders::_2)
  );

  // 创建发布者（发布目标点到/autoware/goal话题）
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/planning/mission_planning/goal", 10);

  RCLCPP_INFO(this->get_logger(), "Goal service server started. Waiting for requests...");
}

void GoalServiceServer::handle_goal_request(
  const std::shared_ptr<UseTrajectory::Request> request,
  std::shared_ptr<UseTrajectory::Response> response)
{
  // 将请求中的目标点发布到/autoware/goal话题
  auto goal_msg = geometry_msgs::msg::PoseStamped();
  goal_msg.header.stamp = this->now();
  goal_msg.header.frame_id = "map";
  goal_msg.pose = request->goal_pose.pose;  // 直接使用请求中的Pose

  goal_pub_->publish(goal_msg);

  // 服务响应（空响应，根据服务定义要求）
  // 如果服务定义要求响应，这里可以填充数据；否则返回空
  (void)response; // 避免未使用警告
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalServiceServer>());
  rclcpp::shutdown();
  return 0;
}