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

#include <autoware/behavior_path_goal_planner_module/util.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

PathWithLaneId makeTurningApproach()
{
  PathWithLaneId path;
  // Quarter circle followed by a straight goal approach. A straight fixture cannot detect
  // loss of the intermediate points that describe a turn.
  for (int i = 0; i <= 20; ++i) {
    const double theta = static_cast<double>(i) * M_PI_2 / 20.0;
    autoware_internal_planning_msgs::msg::PathPointWithLaneId point;
    point.point.pose.position.x = 10.0 * std::sin(theta);
    point.point.pose.position.y = 10.0 * (1.0 - std::cos(theta));
    point.point.pose.orientation.z = std::sin(theta / 2.0);
    point.point.pose.orientation.w = std::cos(theta / 2.0);
    point.point.longitudinal_velocity_mps = 2.0F;
    point.lane_ids = {1};
    path.points.push_back(point);
  }
  for (int i = 1; i <= 20; ++i) {
    auto point = path.points.back();
    point.point.pose.position.y = 10.0 + static_cast<double>(i);
    point.lane_ids = {2};
    path.points.push_back(point);
  }
  path.points.back().point.longitudinal_velocity_mps = 0.0F;
  return path;
}
}  // namespace

TEST(FixedGoalResampling, PreservesTheTurnBeforeTheTerminalSection)
{
  const auto input = makeTurningApproach();
  GoalPlannerParameters::FixedGoal params;
  params.goal_resample_interval = 0.05;
  const auto output =
    goal_planner_utils::applyPreciseFixedGoalStop(input, input.points.back().point.pose, params);

  ASSERT_GE(output.points.size(), 2U);
  for (const auto & original : input.points) {
    if (original.point.pose.position.y >= 27.0) continue;
    const auto found =
      std::find_if(output.points.begin(), output.points.end(), [&](const auto & p) {
        return std::hypot(
                 p.point.pose.position.x - original.point.pose.position.x,
                 p.point.pose.position.y - original.point.pose.position.y) < 1e-8;
      });
    ASSERT_TRUE(found != output.points.end())
      << "A turn point before the terminal section was lost";
    EXPECT_EQ(found->lane_ids, original.lane_ids);
    EXPECT_FLOAT_EQ(
      found->point.longitudinal_velocity_mps, original.point.longitudinal_velocity_mps);
  }
}
TEST(FixedGoalResampling, DensifiesTheTerminalSectionAtTheRequestedInterval)
{
  const auto input = makeTurningApproach();
  GoalPlannerParameters::FixedGoal params;
  params.goal_resample_interval = 0.05;
  const auto output =
    goal_planner_utils::applyPreciseFixedGoalStop(input, input.points.back().point.pose, params);
  for (size_t i = 1; i < output.points.size(); ++i) {
    const auto & a = output.points[i - 1].point.pose.position;
    const auto & b = output.points[i].point.pose.position;
    if (a.y >= 27.0) {
      EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), 0.050001);
    }
  }
  EXPECT_DOUBLE_EQ(output.points.back().point.pose.position.y, 30.0);
  EXPECT_FLOAT_EQ(output.points.back().point.longitudinal_velocity_mps, 0.0F);
}

TEST(FixedGoalResampling, RefinementDoesNotReassignUpstreamLaneIds)
{
  PathWithLaneId input;
  for (int i = 0; i <= 5; ++i) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = 2.0 * i;
    p.point.pose.orientation.w = 1.0;
    p.point.longitudinal_velocity_mps = 2.0F;
    p.lane_ids = {i <= 3 ? 1 : 2};
    input.points.push_back(p);
  }
  const auto get_lane = [](int64_t id) -> lanelet::ConstLanelet {
    const double begin = id == 1 ? -1.0 : 7.0;
    const double end = id == 1 ? 7.0 : 12.0;
    const lanelet::LineString3d left(
      100 + id,
      {lanelet::Point3d(200 + id, begin, 2.0, 0.0), lanelet::Point3d(300 + id, end, 2.0, 0.0)});
    const lanelet::LineString3d right(
      400 + id,
      {lanelet::Point3d(500 + id, begin, -2.0, 0.0), lanelet::Point3d(600 + id, end, -2.0, 0.0)});
    return lanelet::Lanelet(id, left, right);
  };
  const auto output =
    utils::refinePathForGoal(3.0, M_PI_2, 0.5, input, input.points.back().point.pose, 2, get_lane);
  ASSERT_GT(output.points.size(), input.points.size());
  for (const auto & p : output.points) {
    if (p.point.pose.position.x < 6.0) {
      EXPECT_EQ(p.lane_ids, std::vector<int64_t>{1}) << p.point.pose.position.x;
    }
  }
}
}  // namespace autoware::behavior_path_planner
