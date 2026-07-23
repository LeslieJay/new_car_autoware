#ifndef MISSION_LOOP__ARRIVAL_RECORDER_HPP_
#define MISSION_LOOP__ARRIVAL_RECORDER_HPP_

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include <filesystem>
#include <fstream>
#include <string>

namespace autoware::mission_loop
{

class ArrivalRecorder
{
public:
  explicit ArrivalRecorder(const std::filesystem::path & output_directory);
  ~ArrivalRecorder();

  void append(
    const std::string & goal_name, const rclcpp::Time & arrival_time,
    const nav_msgs::msg::Odometry & localization);

  const std::filesystem::path & filePath() const;

private:
  std::filesystem::path file_path_;
  std::ofstream stream_;
};

}  // namespace autoware::mission_loop

#endif  // MISSION_LOOP__ARRIVAL_RECORDER_HPP_
