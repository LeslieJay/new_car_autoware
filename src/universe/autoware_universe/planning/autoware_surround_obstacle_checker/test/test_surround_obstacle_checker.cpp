// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../src/node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <optional>
#include <utility>

namespace autoware::surround_obstacle_checker
{
auto generateTestTargetNode() -> std::shared_ptr<SurroundObstacleCheckerNode>
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  auto node_options = rclcpp::NodeOptions{};
  const auto autoware_test_utils_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");

  autoware::test_utils::updateNodeOptions(
    node_options,
    {autoware_test_utils_dir + "/config/test_common.param.yaml",
      autoware_test_utils_dir + "/config/test_nearest_search.param.yaml",
      autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml",
      ament_index_cpp::get_package_share_directory("autoware_surround_obstacle_checker") +
      "/config/surround_obstacle_checker.param.yaml"});

  return std::make_shared<SurroundObstacleCheckerNode>(node_options);
}

class SurroundObstacleCheckerNodeTest : public ::testing::Test
{
public:
  SurroundObstacleCheckerNodeTest()
  : node_{generateTestTargetNode()} {}
  ~SurroundObstacleCheckerNodeTest() override
  {
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  auto isStopRequired(
    const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
    const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold,
    const bool stop_only_when_stopped = true) const -> std::pair<bool, std::optional<rclcpp::Time>>
  {
    return node_->isStopRequired(
      is_obstacle_found, is_vehicle_stopped, state, last_obstacle_found_time, time_threshold,
      stop_only_when_stopped);
  }

  int getObjectLabel(const PredictedObject & object) const {return node_->getObjectLabel(object);}

  bool isInputUnsafe(
    const bool use_dynamic_objects, const bool pointcloud_enabled,
    const std::optional<rclcpp::Time> & last_odometry_time,
    const std::optional<rclcpp::Time> & last_object_time,
    const std::optional<rclcpp::Time> & last_pointcloud_time, const double timeout_sec) const
  {
    return node_->isInputUnsafe(
      use_dynamic_objects, pointcloud_enabled, last_odometry_time, last_object_time,
      last_pointcloud_time, timeout_sec);
  }

  rclcpp::Time now() const {return node_->now();}

private:
  std::shared_ptr<SurroundObstacleCheckerNode> node_;
};

TEST_F(SurroundObstacleCheckerNodeTest, isStopRequired)
{
  const auto LAST_STOP_TIME = rclcpp::Clock{RCL_ROS_TIME}.now();

  using namespace std::literals::chrono_literals;
  rclcpp::sleep_for(500ms);

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, false, State::STOP, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::PASS, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(true, true, State::STOP, LAST_STOP_TIME, THRESHOLD);

    ASSERT_TRUE(stop_time.has_value());

    const auto time_diff = rclcpp::Clock{RCL_ROS_TIME}.now() - stop_time.value();

    EXPECT_TRUE(is_stop);
    EXPECT_NEAR(time_diff.seconds(), 0.0, 1e-3);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::STOP, LAST_STOP_TIME, THRESHOLD);

    ASSERT_TRUE(stop_time.has_value());

    const auto time_diff = rclcpp::Clock{RCL_ROS_TIME}.now() - stop_time.value();

    EXPECT_TRUE(is_stop);
    EXPECT_NEAR(time_diff.seconds(), 0.5, 1e-3);
  }

  {
    constexpr double THRESHOLD = 0.25;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::STOP, LAST_STOP_TIME, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  {
    constexpr double THRESHOLD = 1.0;
    const auto [is_stop, stop_time] =
      isStopRequired(false, true, State::STOP, std::nullopt, THRESHOLD);
    EXPECT_FALSE(is_stop);
    EXPECT_EQ(stop_time, std::nullopt);
  }

  rclcpp::shutdown();
}

TEST_F(SurroundObstacleCheckerNodeTest, StopsMovingVehicleWhenConfigured)
{
  const auto [is_stop, stop_time] =
    isStopRequired(true, false, State::PASS, std::nullopt, 2.0, false);

  EXPECT_TRUE(is_stop);
  EXPECT_TRUE(stop_time.has_value());
}

TEST_F(SurroundObstacleCheckerNodeTest, KeepsLegacyStoppedVehicleGuardByDefault)
{
  const auto [is_stop, stop_time] = isStopRequired(true, false, State::PASS, std::nullopt, 2.0);

  EXPECT_FALSE(is_stop);
  EXPECT_FALSE(stop_time.has_value());
}

TEST_F(SurroundObstacleCheckerNodeTest, EmptyClassificationIsUnknown)
{
  PredictedObject object;
  EXPECT_EQ(getObjectLabel(object), autoware_perception_msgs::msg::ObjectClassification::UNKNOWN);
}

TEST_F(SurroundObstacleCheckerNodeTest, StaleRequiredInputIsUnsafe)
{
  const auto current_time = now();
  const auto fresh = current_time - rclcpp::Duration::from_seconds(0.1);
  const auto stale = current_time - rclcpp::Duration::from_seconds(0.6);

  EXPECT_FALSE(isInputUnsafe(true, false, fresh, fresh, std::nullopt, 0.5));
  EXPECT_TRUE(isInputUnsafe(true, false, fresh, stale, std::nullopt, 0.5));
  EXPECT_TRUE(isInputUnsafe(true, false, std::nullopt, fresh, std::nullopt, 0.5));
  EXPECT_TRUE(isInputUnsafe(false, true, fresh, std::nullopt, stale, 0.5));
}
}  // namespace autoware::surround_obstacle_checker
