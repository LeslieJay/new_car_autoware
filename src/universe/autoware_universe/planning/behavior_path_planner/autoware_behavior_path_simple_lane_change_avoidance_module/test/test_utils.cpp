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

#include <limits>

namespace autoware::behavior_path_planner
{
namespace
{
PathWithLaneId makePath(const std::initializer_list<double> x_positions)
{
  PathWithLaneId path;
  for (const auto x : x_positions) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = x;
    point.point.pose.orientation.w = 1.0;
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

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
  constexpr double left_lane_distance_from_ego = -3.5;
  constexpr double right_lane_distance_from_ego = 3.5;
  constexpr double lateral_margin = 0.3;

  EXPECT_NEAR(
    calcLaneShiftLength(left_lane_distance_from_ego, lateral_margin), 3.8, 1e-6);
  EXPECT_NEAR(
    calcLaneShiftLength(right_lane_distance_from_ego, lateral_margin), -3.8, 1e-6);
}

TEST_F(SimpleLCAvoidanceUtilsTest, CalcLaneShiftLengthDoesNotApplyExistingUpstreamShiftTwice)
{
  // The upstream simple_avoidance path is already shifted 1.67 m toward the left lane.  The
  // adjacent lane center is therefore only 1.83 m left of the path that this module will shift.
  // Adding the full lane-center separation again would produce 4.30 m and leave the drivable area.
  constexpr double adjacent_lane_distance_from_upstream_path = -1.83;
  constexpr double lateral_margin = 0.3;

  EXPECT_NEAR(
    calcLaneShiftLength(adjacent_lane_distance_from_upstream_path, lateral_margin),
    2.13, 1e-6);
}

TEST_F(SimpleLCAvoidanceUtilsTest, LimitLaneShiftLengthPreservesDirection)
{
  EXPECT_NEAR(limitLaneShiftLength(4.3, 3.0), 3.0, 1e-6);
  EXPECT_NEAR(limitLaneShiftLength(-4.3, 3.0), -3.0, 1e-6);
  EXPECT_NEAR(limitLaneShiftLength(2.0, 3.0), 2.0, 1e-6);
}

TEST_F(SimpleLCAvoidanceUtilsTest, RequiredShiftMustFitHardSafetyLimit)
{
  EXPECT_TRUE(isShiftLengthWithinLimit(4.5, 4.5));
  EXPECT_TRUE(isShiftLengthWithinLimit(-4.5, 4.5));
  EXPECT_FALSE(isShiftLengthWithinLimit(4.5001, 4.5));
  EXPECT_FALSE(isShiftLengthWithinLimit(3.0, 0.0));
  EXPECT_FALSE(isShiftLengthWithinLimit(std::numeric_limits<double>::quiet_NaN(), 4.5));
}

TEST_F(SimpleLCAvoidanceUtilsTest, InvalidFeasibilityParametersAreRejected)
{
  auto parameters = defaultParameters();
  parameters.shifting_lateral_jerk = 0.0;
  EXPECT_FALSE(areFeasibilityParametersValid(parameters));

  parameters = defaultParameters();
  parameters.max_forward_distance = parameters.min_forward_distance - 1.0;
  EXPECT_FALSE(areFeasibilityParametersValid(parameters));

  parameters = defaultParameters();
  parameters.max_shift_length = 4.5;
  EXPECT_TRUE(areFeasibilityParametersValid(parameters));
}

TEST_F(SimpleLCAvoidanceUtilsTest, CompletionRequiresStableSafeGeometry)
{
  LCAvoidanceCompletionStatus status;
  status.is_active_target_passed = true;
  status.lateral_execution_threshold = 0.3;

  EXPECT_FALSE(canCompleteManeuver(status, 2, 3));
  EXPECT_TRUE(canCompleteManeuver(status, 3, 3));

  status.base_offset = 0.31;
  EXPECT_FALSE(canCompleteManeuver(status, 3, 3));
}

TEST_F(SimpleLCAvoidanceUtilsTest, ShiftLineGeometryRejectsCollapsedAndOutOfRangeLines)
{
  ShiftLine line;
  line.start_idx = 1;
  line.end_idx = 3;
  line.start_shift_length = 0.0;
  line.end_shift_length = 3.0;
  EXPECT_TRUE(isValidShiftLineGeometry({line}, 5));

  line.end_idx = 2;
  EXPECT_FALSE(isValidShiftLineGeometry({line}, 5));
  line.end_idx = 5;
  EXPECT_FALSE(isValidShiftLineGeometry({line}, 5));
}

TEST_F(SimpleLCAvoidanceUtilsTest, ShiftOverHardLimitIsReportedAsNoRoom)
{
  auto parameters = defaultParameters();
  parameters.max_shift_length = 4.5;
  const auto result = checkFeasibility(makeTarget(1.0, 30.0), 4.6, parameters, 1.0);
  EXPECT_EQ(result.reason, InfeasibleReason::NO_ROOM);
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

  EXPECT_FALSE(canCompleteManeuver(false, active_shift_lines, 0.0, 0.0, 0.05));
  EXPECT_FALSE(canCompleteManeuver(true, no_shift_lines, 0.0, 0.0, 0.05));
  EXPECT_FALSE(canCompleteManeuver(false, no_shift_lines, 0.1, 0.0, 0.05));
  EXPECT_FALSE(canCompleteManeuver(false, no_shift_lines, 0.0, 0.1, 0.05));
  EXPECT_TRUE(canCompleteManeuver(false, no_shift_lines, 0.0, 0.0, 0.05));
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

TEST_F(SimpleLCAvoidanceUtilsTest, ExtendBackwardPathUsesPreviousReferenceHistory)
{
  const auto previous_path = makePath({-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0});
  const auto current_path = makePath({0.0, 1.0, 2.0, 3.0});
  geometry_msgs::msg::Point ego_position;
  ego_position.x = 0.0;

  const auto extended = extendBackwardPath(previous_path, current_path, ego_position, 2.0);

  ASSERT_EQ(extended.points.size(), 6U);
  EXPECT_DOUBLE_EQ(extended.points.at(0).point.pose.position.x, -2.0);
  EXPECT_DOUBLE_EQ(extended.points.at(1).point.pose.position.x, -1.0);
  EXPECT_DOUBLE_EQ(extended.points.at(2).point.pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(extended.points.back().point.pose.position.x, 3.0);
}

}  // namespace autoware::behavior_path_planner
