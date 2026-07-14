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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <optional>
#include <string>

namespace autoware::behavior_path_planner
{

enum class InfeasibleReason {
  NONE,
  NO_TARGET,
  NO_ROOM,
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
    case InfeasibleReason::NO_ROOM:
      return "infeasible_no_room";
    case InfeasibleReason::INSUFFICIENT_DISTANCE:
      return "infeasible_distance";
    case InfeasibleReason::PATH_GENERATION_FAILED:
      return "path_generation_failed";
  }
  return "unknown";
}

struct SimpleAvoidanceParameters
{
  double th_moving_speed{0.5};
  double min_forward_distance{0.5};
  double max_forward_distance{60.0};
  double lateral_margin{0.4};
  double max_shift_length{4.0};
  double min_prepare_distance{0.5};
  double min_shifting_distance{1.0};
  double shifting_lateral_jerk{0.8};
  double min_shifting_speed{0.2};
  double return_distance_after_object{2.0};
  bool publish_debug_marker{false};
};

struct AvoidanceTarget
{
  geometry_msgs::msg::Pose pose{};
  double longitudinal_distance{0.0};
  double lateral_offset{0.0};
  double object_half_width{0.0};
  double object_half_length{0.0};
  std::string uuid;
};

enum class TargetRejectReason { MOVING, OUT_OF_LANE, LONGITUDINAL, NO_OVERLAP };

inline const char * toString(const TargetRejectReason reason)
{
  switch (reason) {
    case TargetRejectReason::MOVING:
      return "moving";
    case TargetRejectReason::OUT_OF_LANE:
      return "out_of_lane";
    case TargetRejectReason::LONGITUDINAL:
      return "longitudinal_range";
    case TargetRejectReason::NO_OVERLAP:
      return "no_overlap";
  }
  return "unknown";
}

enum class NoTargetPrecondition {
  NONE,
  NO_DYNAMIC_OBJECT,
  EMPTY_REFERENCE_PATH,
  EMPTY_CURRENT_LANELETS,
};

inline const char * toString(const NoTargetPrecondition precondition)
{
  switch (precondition) {
    case NoTargetPrecondition::NONE:
      return "none";
    case NoTargetPrecondition::NO_DYNAMIC_OBJECT:
      return "no_dynamic_object";
    case NoTargetPrecondition::EMPTY_REFERENCE_PATH:
      return "empty_reference_path";
    case NoTargetPrecondition::EMPTY_CURRENT_LANELETS:
      return "empty_current_lanelets";
  }
  return "unknown";
}

struct NoTargetDiagnosis
{
  NoTargetPrecondition precondition{NoTargetPrecondition::NONE};
  size_t total_objects{0};
  size_t rejected_moving{0};
  size_t rejected_out_of_lane{0};
  size_t rejected_longitudinal{0};
  size_t rejected_no_overlap{0};
  bool has_nearest_rejection{false};
  TargetRejectReason nearest_reject_reason{TargetRejectReason::MOVING};
  std::string nearest_uuid;
  double nearest_speed{0.0};
  double nearest_lon{0.0};
  double nearest_lat{0.0};
  double nearest_overlap{0.0};
  double nearest_obj_x{0.0};
  double nearest_obj_y{0.0};
  double nearest_threshold{0.0};
  double nearest_shortfall{0.0};
};

struct ShiftLengthResult
{
  double shift_length{0.0};
  InfeasibleReason reason{InfeasibleReason::NONE};
  double required_clearance{0.0};
  double requested_shift_length{0.0};
  double remaining_gap{0.0};
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

struct PassThroughDebugInfo
{
  std::optional<AvoidanceTarget> target;
  std::optional<NoTargetDiagnosis> no_target;
  std::optional<ShiftLengthResult> shift;
  std::optional<FeasibilityResult> feasibility;
  size_t reference_path_points{0};
  size_t shift_lines_count{0};
};

struct SimpleAvoidanceDebugData
{
  std::shared_ptr<PathShifter> path_shifter{};
  std::optional<AvoidanceTarget> target{};
  InfeasibleReason last_reason{InfeasibleReason::NONE};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__DATA_STRUCTS_HPP_
