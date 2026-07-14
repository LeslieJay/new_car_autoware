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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIMPLE_LANE_CHANGE_AVOIDANCE_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIMPLE_LANE_CHANGE_AVOIDANCE_MODULE__UTILS_HPP_

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_simple_lane_change_avoidance_module/data_structs.hpp"

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

void setOrientation(PathWithLaneId * path);

double getClosestShiftLength(
  const ShiftedPath & shifted_path, const geometry_msgs::msg::Point & ego_point);

LCAvoidanceDirection getAvoidanceDirection(const double lateral_offset);

double applyLaneShiftMargin(const double raw_shift_length, const double lateral_margin);

FeasibilityResult checkFeasibility(
  const LCAvoidanceTarget & target, const double shift_length,
  const SimpleLCAvoidanceParameters & parameters, const double ego_speed);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIMPLE_LANE_CHANGE_AVOIDANCE_MODULE__UTILS_HPP_
