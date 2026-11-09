// Copyright 2022 TIER IV, Inc.
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
#ifndef DATA_STRUCTS_HPP_
#define DATA_STRUCTS_HPP_

#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware/behavior_path_planner_common/utils/articulated_vehicle/trailer_geometry.hpp"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::AvoidanceParameters;

struct OppositeLaneParameters
{
  bool enabled{false};
  bool campus_agv{false};
  double attention_distance{50.0};
  double static_speed_threshold{0.4};
  double static_observation_time{1.0};
  std::vector<double> candidate_speeds{2.0, 1.5, 1.0, 0.5};
  double max_speed{2.0};
  double max_prediction_horizon{30.0};
  double time_resolution{0.5};
  double min_time_gap{3.0};
  double longitudinal_margin{2.0};
  double lateral_margin{0.5};
  double clearance_hold_time{1.0};
  double return_distance_after_object{5.0};
  double data_timeout{0.5};
  double max_planning_time_ms{50.0};
  double prepare_distance{3.0};
  double min_shift_distance{5.0};
  double max_return_extension_distance{20.0};
  double return_extension_step{1.0};
  double road_boundary_margin{0.1};
  double tractor_rear_axle_to_hitch{0.6};
  std::string trailer_configuration_topic{"/vehicle/status/trailer_configuration"};
  int64_t trailer_count{0};
  std::vector<std::string> trailer_ordered_types{"default"};
  std::unordered_map<std::string, TrailerGeometry> trailer_types;
};

struct AvoidanceByLCParameters : public AvoidanceParameters
{
  OppositeLaneParameters opposite_lane;

  // execute only when the target object longitudinal distance is larger than this param.
  double execute_object_longitudinal_margin{0.0};

  // execute only when lane change end point is before the object.
  bool execute_only_when_lane_change_finish_before_object{false};

  explicit AvoidanceByLCParameters(const AvoidanceParameters & param) : AvoidanceParameters(param)
  {
  }
};
}  // namespace autoware::behavior_path_planner

#endif  // DATA_STRUCTS_HPP_
