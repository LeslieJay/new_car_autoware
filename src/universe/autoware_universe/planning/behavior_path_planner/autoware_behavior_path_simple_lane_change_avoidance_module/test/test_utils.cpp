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

#include "autoware/behavior_path_simple_lane_change_avoidance_module/utils.hpp"

#include <gtest/gtest.h>

namespace autoware::behavior_path_planner
{

class SimpleLCAvoidanceUtilsTest : public ::testing::Test
{
protected:
  SimpleLCAvoidanceParameters defaultParameters()
  {
    SimpleLCAvoidanceParameters p;
    p.lateral_margin = 0.3;
    p.min_prepare_distance = 3.0;
    p.min_shifting_distance = 5.0;
    p.shifting_lateral_jerk = 0.2;
    p.min_shifting_speed = 1.0;
    return p;
  }

  LCAvoidanceTarget makeTarget(
    const double lateral_offset, const double longitudinal_distance,
    const double object_half_length = 1.0)
  {
    LCAvoidanceTarget target;
    target.lateral_offset = lateral_offset;
    target.longitudinal_distance = longitudinal_distance;
    target.object_half_length = object_half_length;
    target.direction = getAvoidanceDirection(lateral_offset);
    return target;
  }
};

TEST_F(SimpleLCAvoidanceUtilsTest, GetAvoidanceDirectionObjectOnLeft)
{
  EXPECT_EQ(getAvoidanceDirection(1.0), LCAvoidanceDirection::RIGHT);
  EXPECT_EQ(getAvoidanceDirection(0.0), LCAvoidanceDirection::RIGHT);
}

TEST_F(SimpleLCAvoidanceUtilsTest, GetAvoidanceDirectionObjectOnRight)
{
  EXPECT_EQ(getAvoidanceDirection(-1.0), LCAvoidanceDirection::LEFT);
}

TEST_F(SimpleLCAvoidanceUtilsTest, ApplyLaneShiftMarginPositive)
{
  EXPECT_NEAR(applyLaneShiftMargin(3.0, 0.3), 3.3, 1e-6);
}

TEST_F(SimpleLCAvoidanceUtilsTest, ApplyLaneShiftMarginNegative)
{
  EXPECT_NEAR(applyLaneShiftMargin(-3.0, 0.3), -3.3, 1e-6);
}

TEST_F(SimpleLCAvoidanceUtilsTest, CalcLaneShiftLengthFollowsPathShifterSignConvention)
{
  constexpr double current_lane_distance = 0.0;
  constexpr double left_lane_distance_from_ego = -3.5;
  constexpr double right_lane_distance_from_ego = 3.5;
  constexpr double lateral_margin = 0.3;

  EXPECT_NEAR(
    calcLaneShiftLength(current_lane_distance, left_lane_distance_from_ego, lateral_margin), 3.8,
    1e-6);
  EXPECT_NEAR(
    calcLaneShiftLength(current_lane_distance, right_lane_distance_from_ego, lateral_margin), -3.8,
    1e-6);
}

TEST_F(SimpleLCAvoidanceUtilsTest, InitializeManeuverOnlyWhenNoShiftLinesExist)
{
  ShiftLineArray no_shift_lines;
  ShiftLineArray active_shift_lines(1);

  EXPECT_TRUE(shouldInitializeManeuver(no_shift_lines));
  EXPECT_FALSE(shouldInitializeManeuver(active_shift_lines));
}

TEST_F(SimpleLCAvoidanceUtilsTest, CompleteManeuverOnlyAfterTargetAndShiftLinesAreGone)
{
  ShiftLineArray no_shift_lines;
  ShiftLineArray active_shift_lines(1);

  EXPECT_FALSE(canCompleteManeuver(false, active_shift_lines, 0.0, 0.05));
  EXPECT_FALSE(canCompleteManeuver(true, no_shift_lines, 0.0, 0.05));
  EXPECT_FALSE(canCompleteManeuver(false, no_shift_lines, 0.1, 0.05));
  EXPECT_TRUE(canCompleteManeuver(false, no_shift_lines, 0.0, 0.05));
}

TEST_F(SimpleLCAvoidanceUtilsTest, CheckFeasibilitySufficientDistance)
{
  const auto params = defaultParameters();
  const auto target = makeTarget(1.0, 20.0);
  constexpr double ego_speed = 1.0;
  constexpr double shift_length = -3.5;

  const auto result = checkFeasibility(target, shift_length, params, ego_speed);

  EXPECT_EQ(result.reason, InfeasibleReason::NONE);
  EXPECT_GT(result.dist_to_obstacle, result.dist_to_shift_end);
}

TEST_F(SimpleLCAvoidanceUtilsTest, CheckFeasibilityInsufficientDistance)
{
  const auto params = defaultParameters();
  const auto target = makeTarget(1.0, 5.0);
  constexpr double ego_speed = 1.0;
  constexpr double shift_length = -3.5;

  const auto result = checkFeasibility(target, shift_length, params, ego_speed);

  EXPECT_EQ(result.reason, InfeasibleReason::INSUFFICIENT_DISTANCE);
  EXPECT_GT(result.dist_to_shift_end, result.dist_to_obstacle);
}

}  // namespace autoware::behavior_path_planner
