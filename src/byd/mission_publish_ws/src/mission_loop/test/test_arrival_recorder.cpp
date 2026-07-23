#include "mission_loop/arrival_recorder.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{
std::vector<std::string> readLines(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::vector<std::string> lines;
  for (std::string line; std::getline(input, line); ) {
    lines.push_back(line);
  }
  return lines;
}
}  // namespace

TEST(ArrivalRecorder, AppendsMultipleGoalsToOneFile)
{
  const auto directory =
    std::filesystem::temp_directory_path() / "mission_loop_arrival_recorder_test";
  std::filesystem::remove_all(directory);

  std::filesystem::path file;
  {
    autoware::mission_loop::ArrivalRecorder recorder(directory);
    nav_msgs::msg::Odometry localization;
    localization.header.stamp.sec = 20;
    localization.header.stamp.nanosec = 500000000;
    localization.pose.pose.position.x = 1.0;
    localization.pose.pose.position.y = 2.0;
    localization.pose.pose.position.z = 3.0;
    localization.pose.pose.orientation.w = 1.0;

    recorder.append("A", rclcpp::Time(10, 250000000), localization);
    localization.pose.pose.position.x = 4.0;
    recorder.append("B", rclcpp::Time(11, 0), localization);
    file = recorder.filePath();
  }

  const auto lines = readLines(file);
  ASSERT_EQ(lines.size(), 3U);
  EXPECT_EQ(
    lines[0],
    "goal_name,arrival_time_sec,localization_time_sec,x,y,z,qx,qy,qz,qw");
  EXPECT_EQ(lines[1], "A,10.250000000,20.500000000,1,2,3,0,0,0,1");
  EXPECT_EQ(lines[2], "B,11.000000000,20.500000000,4,2,3,0,0,0,1");

  std::filesystem::remove_all(directory);
}
