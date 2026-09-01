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

#include "autoware/behavior_path_goal_planner_module/default_fixed_goal_planner.hpp"

#include <autoware/route_handler/route_handler.hpp>

#include <gtest/gtest.h>

#include <memory>

namespace autoware::behavior_path_planner
{
namespace
{
class TestableDefaultFixedGoalPlanner : public DefaultFixedGoalPlanner
{
public:
  TestableDefaultFixedGoalPlanner() : DefaultFixedGoalPlanner{GoalPlannerParameters{}} {}

  using DefaultFixedGoalPlanner::extractLaneletsFromPath;
};
}  // namespace

TEST(DefaultFixedGoalPlanner, EmptyLaneIdsAreRejectedWithoutThrowing)
{
  TestableDefaultFixedGoalPlanner planner;
  auto planner_data = std::make_shared<PlannerData>();
  planner_data->route_handler = std::make_shared<route_handler::RouteHandler>();

  PathWithLaneId safe_stop_path;
  safe_stop_path.points.emplace_back();

  lanelet::ConstLanelets lanelets;
  ASSERT_NO_THROW(lanelets = planner.extractLaneletsFromPath(safe_stop_path, planner_data));
  EXPECT_TRUE(lanelets.empty());
}

}  // namespace autoware::behavior_path_planner
