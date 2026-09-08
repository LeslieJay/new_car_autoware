// Copyright 2026 BYD.
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

#include "agv_high_precision_reverse_controller/precision_reverse_controller.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

namespace agv_high_precision_reverse_controller
{

namespace
{
std::vector<PathPoint> straightPath()
{
  return {{0.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, {-2.0, 0.0, 0.0}};
}
}  // namespace

TEST(PrecisionReverseController, RejectsForwardPath)
{
  PrecisionReverseController controller;
  std::string reason;
  EXPECT_FALSE(controller.setPath({{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}, &reason));
  EXPECT_EQ(reason, "path contains a forward segment");
}

TEST(PrecisionReverseController, ClearPathInvalidatesOldCommand)
{
  PrecisionReverseController controller;
  ASSERT_TRUE(controller.setPath(straightPath()));
  EXPECT_TRUE(controller.compute({0.0, 0.0, 0.0, 0.0}, 0.02).valid);
  controller.clearPath();
  EXPECT_FALSE(controller.compute({0.0, 0.0, 0.0, 0.0}, 0.02).valid);
}

TEST(PrecisionReverseController, ProjectsContinuouslyAndUsesRemainingArcLength)
{
  PrecisionReverseController controller;
  ASSERT_TRUE(controller.setPath(straightPath()));
  const auto result = controller.compute({-0.75, 0.0, 0.0, -0.1}, 0.02);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.projected_s, 0.75, 1.0e-9);
  EXPECT_NEAR(result.remaining_s, 1.25, 1.0e-9);
  EXPECT_NEAR(result.lateral_error, 0.0, 1.0e-9);
}

TEST(PrecisionReverseController, CorrectsLeftLateralErrorWithRightSteering)
{
  ControllerParameters parameters;
  parameters.max_steering_rate = 100.0;
  PrecisionReverseController controller(parameters);
  ASSERT_TRUE(controller.setPath(straightPath()));
  const auto result = controller.compute({-0.5, 0.10, 0.0, -0.1}, 0.02);
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.lateral_error, 0.0);
  EXPECT_LT(result.steering_angle, 0.0);
}

TEST(PrecisionReverseController, BrakingProfileSlowsNearGoal)
{
  ControllerParameters parameters;
  parameters.max_acceleration = 100.0;
  parameters.max_deceleration = 100.0;
  PrecisionReverseController far_controller(parameters);
  PrecisionReverseController near_controller(parameters);
  ASSERT_TRUE(far_controller.setPath(straightPath()));
  ASSERT_TRUE(near_controller.setPath(straightPath()));
  const auto far = far_controller.compute({-0.2, 0.0, 0.0, 0.0}, 0.02);
  const auto near = near_controller.compute({-1.95, 0.0, 0.0, 0.0}, 0.02);
  ASSERT_TRUE(far.valid);
  ASSERT_TRUE(near.valid);
  EXPECT_GT(far.target_speed, near.target_speed);
  EXPECT_GE(near.target_speed, parameters.min_creep_speed);
}

TEST(PrecisionReverseController, RequiresPoseAndSpeedToDeclareGoal)
{
  PrecisionReverseController controller;
  ASSERT_TRUE(controller.setPath(straightPath()));
  EXPECT_FALSE(controller.compute({-2.0, 0.0, 0.0, -0.1}, 0.02).goal_reached);
  EXPECT_TRUE(controller.compute({-2.0, 0.0, 0.0, 0.0}, 0.02).goal_reached);
}

TEST(PrecisionReverseController, StopsAfterOvershootingGoal)
{
  PrecisionReverseController controller;
  ASSERT_TRUE(controller.setPath(straightPath()));
  const auto result = controller.compute({-2.10, 0.0, 0.0, 0.0}, 0.02);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.tracking_error);
  EXPECT_EQ(result.target_speed, 0.0);
}

TEST(PrecisionReverseController, ClosedLoopStraightReverseConvergesFromOffset)
{
  PrecisionReverseController controller;
  ASSERT_TRUE(controller.setPath(straightPath()));
  VehicleState vehicle{0.0, 0.10, 0.0, 0.0};
  TrackingResult result;
  constexpr double dt = 0.02;
  for (std::size_t cycle = 0; cycle < 1500U; ++cycle) {
    result = controller.compute(vehicle, dt);
    ASSERT_TRUE(result.valid);
    ASSERT_FALSE(result.tracking_error);
    if (result.goal_reached) {
      break;
    }
    vehicle.speed = -result.target_speed;
    vehicle.x += vehicle.speed * std::cos(vehicle.yaw) * dt;
    vehicle.y += vehicle.speed * std::sin(vehicle.yaw) * dt;
    vehicle.yaw +=
      vehicle.speed / controller.parameters().wheel_base * std::tan(result.steering_angle) * dt;
  }
  vehicle.speed = 0.0;
  result = controller.compute(vehicle, dt);
  EXPECT_TRUE(result.goal_reached);
  EXPECT_LT(result.goal_distance, controller.parameters().goal_distance_tolerance);
  EXPECT_LT(result.goal_yaw_error, controller.parameters().goal_yaw_tolerance);
}

}  // namespace agv_high_precision_reverse_controller
