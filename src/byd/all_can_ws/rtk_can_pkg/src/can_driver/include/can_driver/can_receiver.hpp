#ifndef CAN_DRIVER__CAN_RECEIVER_HPP_
#define CAN_DRIVER__CAN_RECEIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include "can_driver/can_frame.hpp"

constexpr double INT_POS = 1e7;
constexpr double INT_IMU = 1e2;
constexpr double INT_ATTI = 1e2;
constexpr double acc_IMU = 1e3;

namespace can_driver
{

class CanReceiver
{
public:
  explicit CanReceiver(std::shared_ptr<rclcpp::Node> node);
  ~CanReceiver();

  bool isRunning() const;
  void stop();
  void receiveTask(int handle, const std::string & interface_name);
  void publishNavSatFixTask();
  void publishGnssInsTask();

private:
  bool initRawLogging();
  void writeRawFrame(const CanFrame & frame, const rclcpp::Time & stamp);

  std::shared_ptr<rclcpp::Node> node_;
  std::atomic<bool> running_{false};

  std::ofstream raw_log_file_;
  std::atomic<int> current_log_lines_{0};
  std::mutex raw_log_mutex_;
  static constexpr int MAX_LOG_LINES = 100000;
  static constexpr int TRIM_LINES = 50000;
  std::string raw_log_basename_;

  std::deque<CanFrame> nav_queue_;
  std::mutex nav_queue_mutex_;
  std::condition_variable nav_cv_;

  std::deque<CanFrame> gnss_queue_;
  std::mutex gnss_queue_mutex_;
  std::condition_variable gnss_cv_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_NavSatFix_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr
    rtk_GnssInsOrientationStamped_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    rtk_velocity_twist_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    rtk_velocity_vector_publisher_;
};

}  // namespace can_driver

#endif  // CAN_DRIVER__CAN_RECEIVER_HPP_
