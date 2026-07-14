// Copyright 2025 BYD
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

#include "autoware/behavior_path_simple_avoidance_module/utils.hpp"

#include <gtest/gtest.h>

namespace autoware::behavior_path_planner
{

class SimpleAvoidanceUtilsTest : public ::testing::Test
{
protected:
  SimpleAvoidanceParameters defaultParameters()
  {
    SimpleAvoidanceParameters p;
    p.lateral_margin = 0.8;
    p.max_shift_length = 3.0;
    p.min_prepare_distance = 1.0;
    p.min_shifting_distance = 2.0;
    p.shifting_lateral_jerk = 0.5;
    p.min_shifting_speed = 0.3;
    return p;
  }

  AvoidanceTarget makeTarget(
    const double lateral_offset, const double longitudinal_distance,
    const double object_half_width = 0.5, const double object_half_length = 1.0)
  {
    AvoidanceTarget target;
    target.lateral_offset = lateral_offset;
    target.longitudinal_distance = longitudinal_distance;
    target.object_half_width = object_half_width;
    target.object_half_length = object_half_length;
    return target;
  }
};

TEST_F(SimpleAvoidanceUtilsTest, CalcShiftLengthWithinMaxShift)
{
  const auto params = defaultParameters();
  constexpr double ego_half_width = 0.9;
  // near_edge=0.0 + ego_hw=0.9 + margin=0.8 = 1.7 < max_shift 3.0
  const auto target = makeTarget(0.5, 20.0, 0.5);

  const auto result = calcShiftLength(target, params, ego_half_width);

  EXPECT_EQ(result.reason, InfeasibleReason::NONE);
  EXPECT_NEAR(result.required_clearance, 1.7, 1e-6);
  EXPECT_NEAR(result.shift_length, -1.7, 1e-6);
  EXPECT_NEAR(result.remaining_gap, 0.0, 1e-6);
}

TEST_F(SimpleAvoidanceUtilsTest, CalcShiftLengthNoRoomWhenRequiredExceedsMaxShift)
{
  const auto params = defaultParameters();
  constexpr double ego_half_width = 0.9;
  // near_edge=2.0 + ego_hw=0.9 + margin=0.8 = 3.7 > max_shift 3.0
  const auto target = makeTarget(2.5, 20.0, 0.5);
  const double required = 2.0 + ego_half_width + params.lateral_margin;
  ASSERT_GT(required, params.max_shift_length);

  const auto result = calcShiftLength(target, params, ego_half_width);

  EXPECT_EQ(result.reason, InfeasibleReason::NO_ROOM);
  EXPECT_NEAR(result.shift_length, -params.max_shift_length, 1e-6);
  EXPECT_NEAR(result.remaining_gap, required - params.max_shift_length, 1e-6);
  EXPECT_GT(result.remaining_gap, 0.0);
}

TEST_F(SimpleAvoidanceUtilsTest, CalcShiftLengthNoRoomInFormerFalsePassBand)
{
  const auto params = defaultParameters();
  constexpr double ego_half_width = 0.9;
  // near_edge=1.5 + ego_hw=0.9 + margin=0.8 = 3.2 > max_shift 3.0
  const auto target = makeTarget(2.0, 20.0, 0.5);

  const auto result = calcShiftLength(target, params, ego_half_width);

  EXPECT_EQ(result.reason, InfeasibleReason::NO_ROOM);
  EXPECT_NEAR(result.remaining_gap, 0.2, 1e-6);
}

TEST_F(SimpleAvoidanceUtilsTest, CalcShiftLengthNegativeLateralOffsetShiftsLeft)
{
  const auto params = defaultParameters();
  constexpr double ego_half_width = 0.9;
  // near_edge=0.0 + ego_hw=0.9 + margin=0.8 = 1.7 < max_shift 3.0
  const auto target = makeTarget(-0.5, 20.0, 0.5);

  const auto result = calcShiftLength(target, params, ego_half_width);

  EXPECT_EQ(result.reason, InfeasibleReason::NONE);
  EXPECT_GT(result.shift_length, 0.0);
}

TEST_F(SimpleAvoidanceUtilsTest, CalcShiftLengthWideObjectEncroachingPath)
{
  const auto params = defaultParameters();
  constexpr double ego_half_width = 0.47;
  // near_edge=0.09 + ego_hw=0.47 + margin=0.8 = 1.36 < max_shift 3.0
  const auto target = makeTarget(-2.91, 49.0, 3.0);

  const auto result = calcShiftLength(target, params, ego_half_width);

  EXPECT_EQ(result.reason, InfeasibleReason::NONE);
  EXPECT_NEAR(result.required_clearance, 1.36, 1e-6);
  EXPECT_NEAR(result.shift_length, 1.36, 1e-6);
}

TEST_F(SimpleAvoidanceUtilsTest, CheckFeasibilitySufficientDistance)
{
  const auto params = defaultParameters();
  const auto target = makeTarget(0.5, 20.0, 0.5, 1.0);
  constexpr double ego_speed = 1.0;
  constexpr double shift_length = -2.0;

  const auto result = checkFeasibility(target, shift_length, params, ego_speed);

  EXPECT_EQ(result.reason, InfeasibleReason::NONE);
  EXPECT_GT(result.dist_to_obstacle, result.dist_to_shift_end);
}

TEST_F(SimpleAvoidanceUtilsTest, CheckFeasibilityInsufficientDistance)
{
  const auto params = defaultParameters();
  const auto target = makeTarget(0.5, 5.0, 0.5, 1.0);
  constexpr double ego_speed = 1.0;
  constexpr double shift_length = -2.0;

  const auto result = checkFeasibility(target, shift_length, params, ego_speed);

  EXPECT_EQ(result.reason, InfeasibleReason::INSUFFICIENT_DISTANCE);
  EXPECT_GT(result.dist_to_shift_end, result.dist_to_obstacle);
}

TEST_F(SimpleAvoidanceUtilsTest, CheckFeasibilityCloseObjectAtLowSpeed)
{
  const auto params = defaultParameters();
  // lon=9.22m, obj_hl=2.5m, shift=1.36m, ego_speed=0.48m/s
  const auto target = makeTarget(-2.91, 9.22, 3.0, 2.5);
  constexpr double ego_speed = 0.48;
  constexpr double shift_length = 1.36;

  const auto result = checkFeasibility(target, shift_length, params, ego_speed);

  EXPECT_EQ(result.reason, InfeasibleReason::NONE);
  EXPECT_LE(result.dist_to_shift_end, result.dist_to_obstacle);
}

}  // namespace autoware::behavior_path_planner
