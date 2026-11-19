// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_goal_planner_module/util.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <memory>

namespace autoware::behavior_path_planner
{
using Point2d = autoware_utils::Point2d;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

DefaultFixedGoalPlanner::DefaultFixedGoalPlanner(const GoalPlannerParameters & parameters)
: parameters_{parameters}
{
}

BehaviorModuleOutput DefaultFixedGoalPlanner::plan(
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  BehaviorModuleOutput output =
    // use planner previous module reference path
    getPreviousModuleOutput();
  const PathWithLaneId smoothed_path = modifyPathForSmoothGoalConnection(output.path, planner_data);

  const Pose goal = planner_data->route_handler->getGoalPose();
  const auto precise_path =
    goal_planner_utils::applyPreciseFixedGoalStop(smoothed_path, goal, parameters_.fixed_goal);
  // Validate the final transformed geometry, rather than a path that is subsequently changed.
  if (!isPathValid(precise_path, goal, planner_data)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("default_fixed_goal_planner"),
      "Cannot connect a valid path to the requested goal without moving it.");
    if (!output.path.points.empty()) {
      output.path.points.back().point.longitudinal_velocity_mps = 0.0;
    }
    return output;
  }

  output.path = precise_path;
  output.reference_path = getPreviousModuleOutput().reference_path;
  return output;
}

lanelet::ConstLanelets DefaultFixedGoalPlanner::extractLaneletsFromPath(
  const PathWithLaneId & refined_path,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto & rh = planner_data->route_handler;
  lanelet::ConstLanelets refined_path_lanelets;
  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    const auto & path_point = refined_path.points.at(i);
    // A safety fallback from an upstream scene module can contain valid stop geometry before its
    // lane association is available. Treat it as an invalid refinement candidate instead of
    // terminating the whole behavior-path-planner component container with std::out_of_range.
    if (path_point.lane_ids.empty()) {
      continue;
    }
    const int64_t lane_id = path_point.lane_ids.front();
    lanelet::ConstLanelet lanelet = rh->getLaneletsFromId(lane_id);
    bool is_unique =
      std::find(refined_path_lanelets.begin(), refined_path_lanelets.end(), lanelet) ==
      refined_path_lanelets.end();
    if (is_unique) {
      refined_path_lanelets.push_back(lanelet);
    }
  }
  return refined_path_lanelets;
}

bool DefaultFixedGoalPlanner::isPathValid(
  const PathWithLaneId & refined_path, const Pose & goal,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  if (refined_path.points.size() < 2) {
    return false;
  }
  const auto & end = refined_path.points.back().point.pose;
  constexpr double position_tolerance = 1e-3;
  constexpr double heading_tolerance = 1e-3;
  if (
    std::hypot(end.position.x - goal.position.x, end.position.y - goal.position.y) >
      position_tolerance ||
    std::abs(autoware_utils::normalize_radian(
      tf2::getYaw(end.orientation) - tf2::getYaw(goal.orientation))) > heading_tolerance) {
    return false;
  }
  const auto lanelets = extractLaneletsFromPath(refined_path, planner_data);
  // std::any_of detects whether any point lies outside lanelets
  bool has_points_outside_lanelet = std::any_of(
    refined_path.points.begin(), refined_path.points.end(),
    [&lanelets](const auto & refined_path_point) {
      return !utils::isInLanelets(refined_path_point.point.pose, lanelets);
    });
  return !has_points_outside_lanelet;
}

PathWithLaneId DefaultFixedGoalPlanner::modifyPathForSmoothGoalConnection(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto goal = planner_data->route_handler->getGoalPose();
  const auto goal_lane_id = planner_data->route_handler->getGoalLaneId();

  // A short fixed-goal request must not reconnect geometry farther back than the hand-off.
  // Keep the first half of the requested approach available for continuous tracking.
  double goal_search_radius = std::min(
    planner_data->parameters.refine_goal_search_radius_range,
    0.5 * parameters_.fixed_goal.request_length);
  const double output_path_interval{planner_data->parameters.output_path_interval};
  // TODO(shen): define in the parameter
  constexpr double range_reduce_by{1.0};  // set a reasonable value, 10% - 20% of the
                                          // refine_goal_search_radius_range is recommended
  bool is_valid_path{false};
  autoware_internal_planning_msgs::msg::PathWithLaneId refined_path;
  while (goal_search_radius >= 0 && !is_valid_path) {
    refined_path = utils::refinePathForGoal(
      goal_search_radius, M_PI * 0.5, output_path_interval, path, goal, goal_lane_id,
      [&](int64_t lane_id) -> lanelet::ConstLanelet {
        return planner_data->route_handler->getLaneletMapPtr()->laneletLayer.get(lane_id);
      });
    if (isPathValid(refined_path, goal, planner_data)) {
      is_valid_path = true;
    }
    goal_search_radius -= range_reduce_by;
  }
  return refined_path;
}

}  // namespace autoware::behavior_path_planner
