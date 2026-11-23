// Copyright 2026 BYD
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

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace lon = autoware::motion::control::pid_longitudinal_controller;
namespace follower = autoware::motion::control::trajectory_follower;

class SteeringTrackingHold : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void initialize(double hold_velocity)
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "--params-file", TEST_PID_PARAM_PATH});
    options.parameter_overrides(
      {{"steer_convergence_hold_velocity", hold_velocity},
       {"wheel_radius", 0.2165},
       {"wheel_width", 0.125},
       {"wheel_base", 1.008},
       {"wheel_tread", 0.835},
       {"front_overhang", 0.546},
       {"rear_overhang", 0.676},
       {"left_overhang", 0.235},
       {"right_overhang", 0.235},
       {"vehicle_height", 2.4},
       {"max_steer_angle", 1.0}});
    node = std::make_shared<rclcpp::Node>("test_steering_tracking_hold", options);
    node->declare_parameter("ctrl_period", 0.02);
    node->declare_parameter("ego_nearest_dist_threshold", 3.0);
    node->declare_parameter("ego_nearest_yaw_threshold", 1.046);
    updater = std::make_shared<diagnostic_updater::Updater>(node);
    controller = std::make_unique<lon::PidLongitudinalController>(*node, updater);
    input.current_odometry.pose.pose.orientation.w = 1.0;
    input.current_odometry.twist.twist.linear.x = 0.3;
    input.current_operation_mode.mode = 2;
    input.current_operation_mode.is_autoware_control_enabled = true;
    for (int i = 0; i <= 100; ++i) {
      autoware_planning_msgs::msg::TrajectoryPoint point;
      point.pose.position.x = i * 0.25;
      point.pose.orientation.w = 1.0;
      point.longitudinal_velocity_mps = i == 100 ? 0.0 : 1.0;
      input.current_trajectory.points.push_back(point);
    }
    controller->sync(follower::LateralSyncData{true});
    for (int i = 0; i < 3; ++i) controller->run(input);
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<diagnostic_updater::Updater> updater;
  std::unique_ptr<follower::LongitudinalControllerBase> controller;
  follower::InputData input;
};

TEST_F(SteeringTrackingHold, HoldsMovingRestartUntilWheelTracksAgain)
{
  initialize(1.1);
  ASSERT_GT(controller->run(input).control_cmd.velocity, 0.0);
  controller->sync(follower::LateralSyncData{false});
  const auto held = controller->run(input);
  EXPECT_DOUBLE_EQ(held.control_cmd.velocity, 0.0);
  EXPECT_LT(held.control_cmd.acceleration, 0.0);
  ASSERT_EQ(held.control_cmd_horizon.controls.size(), 1U);
  EXPECT_DOUBLE_EQ(held.control_cmd_horizon.controls.front().velocity, 0.0);
  controller->sync(follower::LateralSyncData{true});
  EXPECT_DOUBLE_EQ(controller->run(input).control_cmd.velocity, 0.0);
  EXPECT_DOUBLE_EQ(controller->run(input).control_cmd.velocity, 0.0);
  EXPECT_GT(controller->run(input).control_cmd.velocity, 0.0);
}

TEST_F(SteeringTrackingHold, DefaultKeepsExistingMovingBehavior)
{
  initialize(0.0);
  controller->sync(follower::LateralSyncData{false});
  EXPECT_GT(controller->run(input).control_cmd.velocity, 0.0);
}

TEST_F(SteeringTrackingHold, DoesNotApplyLowSpeedHoldAboveItsOperatingRange)
{
  initialize(1.1);
  input.current_odometry.twist.twist.linear.x = 1.5;
  controller->sync(follower::LateralSyncData{false});
  EXPECT_GT(controller->run(input).control_cmd.velocity, 0.0);
}

// Closed-loop velocity-interface model: finite steering nonresponse, followed by recovery.
// This isolates the controller interlock; it does not replace full planning/chassis simulation.
TEST_F(SteeringTrackingHold, FiniteSteeringFaultBrakesThenRecoversInClosedLoop)
{
  initialize(1.1);
  constexpr double dt = 0.02;
  constexpr double fault_duration = 2.1;
  double velocity = 0.3;
  double distance_during_fault = 0.0;
  double distance_after_recovery = 0.0;
  for (int step = 0; step < 350; ++step) {
    const bool fault_active = step * dt < fault_duration;
    controller->sync(follower::LateralSyncData{!fault_active});
    input.current_odometry.twist.twist.linear.x = velocity;
    const auto command = controller->run(input).control_cmd;
    if (fault_active) {
      EXPECT_DOUBLE_EQ(command.velocity, 0.0);
      EXPECT_LT(command.acceleration, 0.0);
    }
    // Measured velocity follows its command with a 0.15 s first-order response.
    velocity += (command.velocity - velocity) * dt / 0.15;
    if (fault_active) {
      distance_during_fault += velocity * dt;
    } else {
      distance_after_recovery += velocity * dt;
    }
    input.current_odometry.pose.pose.position.x += velocity * dt;
  }
  EXPECT_LT(distance_during_fault, 0.05);
  EXPECT_GT(distance_after_recovery, 3.0);
  EXPECT_GT(velocity, 0.9);
}

TEST_F(SteeringTrackingHold, PersistentSteeringFaultNeverReleasesLowSpeedHold)
{
  initialize(1.1);
  double velocity = 0.3;
  for (int step = 0; step < 500; ++step) {
    controller->sync(follower::LateralSyncData{false});
    input.current_odometry.twist.twist.linear.x = velocity;
    const auto command = controller->run(input).control_cmd;
    EXPECT_DOUBLE_EQ(command.velocity, 0.0);
    velocity += (command.velocity - velocity) * 0.02 / 0.15;
    input.current_odometry.pose.pose.position.x += velocity * 0.02;
  }
  EXPECT_LT(velocity, 0.001);
}
