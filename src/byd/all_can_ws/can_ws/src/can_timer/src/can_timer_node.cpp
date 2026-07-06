#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"

using namespace std::chrono_literals;

class TimerPublisherNode : public rclcpp::Node
{
public:
  TimerPublisherNode()
  : Node("can_timer_publisher_node"), count_(0)
  {
    // 创建发布者，发布std_msgs::msg::String类型消息，话题名为"topic_name"，队列大小10
    velocity_publisher_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
    steering_publisher_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
    // 创建定时器，周期为100ms，定时回调函数为timer_callback
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TimerPublisherNode::timer_callback, this));
  }

private:
  void timer_callback()
  {
    

    autoware_vehicle_msgs::msg::VelocityReport velocity_info;
    velocity_info.header.frame_id = "base_link";
    velocity_info.header.stamp = this->get_clock()->now();
    velocity_info.longitudinal_velocity = 0;
    velocity_info.lateral_velocity = 0;
    velocity_info.heading_rate = 0;
    velocity_publisher_->publish(velocity_info);

    autoware_vehicle_msgs::msg::SteeringReport steering_info;
    steering_info.steering_tire_angle = 0;
    steering_info.stamp = this->get_clock()->now();
    steering_publisher_->publish(steering_info);

    RCLCPP_INFO(this->get_logger(), "Publishing:  ");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_publisher_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimerPublisherNode>());
  rclcpp::shutdown();
  return 0;
}