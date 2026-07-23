#include "mission_loop/arrival_recorder.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace autoware::mission_loop
{
namespace
{

std::string makeFilename()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);

  std::ostringstream name;
  name << "mission_arrivals_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".csv";
  return name.str();
}

std::string formatTime(const int64_t nanoseconds)
{
  const auto seconds = nanoseconds / 1000000000;
  const auto remainder = nanoseconds % 1000000000;
  std::ostringstream value;
  value << seconds << '.' << std::setw(9) << std::setfill('0') << remainder;
  return value.str();
}

std::string formatTime(const builtin_interfaces::msg::Time & time)
{
  std::ostringstream value;
  value << time.sec << '.' << std::setw(9) << std::setfill('0') << time.nanosec;
  return value.str();
}

}  // namespace

ArrivalRecorder::ArrivalRecorder(const std::filesystem::path & output_directory)
{
  std::filesystem::create_directories(output_directory);
  file_path_ = output_directory / makeFilename();
  stream_.open(file_path_);
  if (!stream_.is_open()) {
    throw std::runtime_error("Failed to open arrival file: " + file_path_.string());
  }
  stream_ << "goal_name,arrival_time_sec,localization_time_sec,x,y,z,qx,qy,qz,qw\n";
  stream_.flush();
  if (!stream_) {
    throw std::runtime_error("Failed to write arrival file header: " + file_path_.string());
  }
}

ArrivalRecorder::~ArrivalRecorder()
{
  if (stream_.is_open()) {
    stream_.close();
  }
}

void ArrivalRecorder::append(
  const std::string & goal_name, const rclcpp::Time & arrival_time,
  const nav_msgs::msg::Odometry & localization)
{
  const auto & position = localization.pose.pose.position;
  const auto & orientation = localization.pose.pose.orientation;
  stream_ << goal_name << ',' << formatTime(arrival_time.nanoseconds()) << ','
          << formatTime(localization.header.stamp) << ',' << std::setprecision(17) << position.x
          << ',' << position.y << ',' << position.z << ',' << orientation.x << ','
          << orientation.y << ',' << orientation.z << ',' << orientation.w << '\n';
  stream_.flush();
  if (!stream_) {
    throw std::runtime_error("Failed to write arrival file: " + file_path_.string());
  }
}

const std::filesystem::path & ArrivalRecorder::filePath() const
{
  return file_path_;
}

}  // namespace autoware::mission_loop
