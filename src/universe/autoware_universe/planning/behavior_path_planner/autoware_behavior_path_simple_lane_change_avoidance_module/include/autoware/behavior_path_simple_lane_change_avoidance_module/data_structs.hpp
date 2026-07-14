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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIMPLE_LANE_CHANGE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIMPLE_LANE_CHANGE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <optional>
#include <string>

namespace autoware::behavior_path_planner
{

enum class LCAvoidanceDirection { LEFT, RIGHT };

enum class InfeasibleReason {
  NONE,
  NO_TARGET,
  NO_ADJACENT_LANE,
  INSUFFICIENT_DISTANCE,
  PATH_GENERATION_FAILED,
};

inline const char * toString(const InfeasibleReason reason)
{
  switch (reason) {
    case InfeasibleReason::NONE:
      return "none";
    case InfeasibleReason::NO_TARGET:
      return "no_target";
    case InfeasibleReason::NO_ADJACENT_LANE:
      return "no_adjacent_lane";
    case InfeasibleReason::INSUFFICIENT_DISTANCE:
      return "infeasible_distance";
    case InfeasibleReason::PATH_GENERATION_FAILED:
      return "path_generation_failed";
  }
  return "unknown";
}

struct SimpleLCAvoidanceParameters
{
  double th_moving_speed{0.5};
  double min_forward_distance{1.0};
  double max_forward_distance{50.0};
  double lateral_margin{0.3};
  double min_prepare_distance{3.0};
  double min_shifting_distance{5.0};
  double shifting_lateral_jerk{0.2};
  double min_shifting_speed{1.0};
  double return_distance_after_object{5.0};
  bool publish_debug_marker{true};
};

struct LCAvoidanceTarget
{
  geometry_msgs::msg::Pose pose{};
  double longitudinal_distance{0.0};
  double lateral_offset{0.0};
  double object_half_width{0.0};
  double object_half_length{0.0};
  std::string uuid;
  LCAvoidanceDirection direction{LCAvoidanceDirection::LEFT};
};

struct AdjacentLaneResult
{
  bool found{false};
  lanelet::ConstLanelet lanelet{};
  lanelet::ConstLanelets lanelet_sequence{};
  LCAvoidanceDirection direction{LCAvoidanceDirection::LEFT};
};

struct LaneShiftResult
{
  double shift_length{0.0};
  InfeasibleReason reason{InfeasibleReason::NONE};
  AdjacentLaneResult adjacent_lane{};
};

struct FeasibilityResult
{
  InfeasibleReason reason{InfeasibleReason::NONE};
  double dist_to_shift_end{0.0};
  double dist_to_obstacle{0.0};
  double jerk_distance{0.0};
  double ego_speed{0.0};
  double min_prepare_distance{0.0};
  double min_shifting_distance{0.0};
};

struct SimpleLCAvoidanceDebugData
{
  std::shared_ptr<PathShifter> path_shifter{};
  std::optional<LCAvoidanceTarget> target{};
  InfeasibleReason last_reason{InfeasibleReason::NONE};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIMPLE_LANE_CHANGE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
