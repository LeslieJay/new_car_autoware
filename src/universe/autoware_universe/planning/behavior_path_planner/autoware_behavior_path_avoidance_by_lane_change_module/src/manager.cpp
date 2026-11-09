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

#include "manager.hpp"

#include "autoware/behavior_path_static_obstacle_avoidance_module/parameter_helper.hpp"
#include "autoware_utils/ros/parameter.hpp"
#include "autoware_utils/ros/update_param.hpp"
#include "data_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::behavior_path_planner::getParameter;
using autoware::behavior_path_planner::ObjectParameter;

void AvoidanceByLaneChangeModuleManager::init(rclcpp::Node * node)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_utils::get_or_declare_parameter;

  // init manager interface
  initInterface(node, {"left", "right"});

  // init lane change manager
  LaneChangeModuleManager::initParams(node);

  const auto avoidance_params = getParameter(node);
  AvoidanceByLCParameters p(avoidance_params);

  // unique parameters
  {
    const std::string ns = "avoidance_by_lane_change.";
    p.execute_object_longitudinal_margin =
      get_or_declare_parameter<double>(*node, ns + "execute_object_longitudinal_margin");
    p.execute_only_when_lane_change_finish_before_object = get_or_declare_parameter<bool>(
      *node, ns + "execute_only_when_lane_change_finish_before_object");
  }

  // general params
  {
    const std::string ns = "avoidance.";
    p.resample_interval_for_planning =
      get_or_declare_parameter<double>(*node, ns + "resample_interval_for_planning");
    p.resample_interval_for_output =
      get_or_declare_parameter<double>(*node, ns + "resample_interval_for_output");
  }

  // target object
  {
    const auto get_object_param = [&](std::string && ns) {
      ObjectParameter param{};
      param.moving_speed_threshold =
        get_or_declare_parameter<double>(*node, ns + "th_moving_speed");
      param.moving_time_threshold = get_or_declare_parameter<double>(*node, ns + "th_moving_time");
      param.max_expand_ratio = get_or_declare_parameter<double>(*node, ns + "max_expand_ratio");
      param.envelope_buffer_margin =
        get_or_declare_parameter<double>(*node, ns + "envelope_buffer_margin");
      param.lateral_soft_margin =
        get_or_declare_parameter<double>(*node, ns + "lateral_margin.soft_margin");
      param.lateral_hard_margin =
        get_or_declare_parameter<double>(*node, ns + "lateral_margin.hard_margin");
      param.lateral_hard_margin_for_parked_vehicle = get_or_declare_parameter<double>(
        *node, ns + "lateral_margin.hard_margin_for_parked_vehicle");
      return param;
    };

    const std::string ns = "avoidance_by_lane_change.target_object.";
    p.object_parameters.emplace(
      ObjectClassification::MOTORCYCLE, get_object_param(ns + "motorcycle."));
    p.object_parameters.emplace(ObjectClassification::CAR, get_object_param(ns + "car."));
    p.object_parameters.emplace(ObjectClassification::TRUCK, get_object_param(ns + "truck."));
    p.object_parameters.emplace(ObjectClassification::TRAILER, get_object_param(ns + "trailer."));
    p.object_parameters.emplace(ObjectClassification::BUS, get_object_param(ns + "bus."));
    p.object_parameters.emplace(
      ObjectClassification::PEDESTRIAN, get_object_param(ns + "pedestrian."));
    p.object_parameters.emplace(ObjectClassification::BICYCLE, get_object_param(ns + "bicycle."));
    p.object_parameters.emplace(ObjectClassification::UNKNOWN, get_object_param(ns + "unknown."));

    p.lower_distance_for_polygon_expansion =
      get_or_declare_parameter<double>(*node, ns + "lower_distance_for_polygon_expansion");
    p.upper_distance_for_polygon_expansion =
      get_or_declare_parameter<double>(*node, ns + "upper_distance_for_polygon_expansion");
  }

  // target filtering
  {
    const auto set_target_flag = [&](const uint8_t & object_type, const std::string & ns) {
      if (p.object_parameters.count(object_type) == 0) {
        return;
      }
      p.object_parameters.at(object_type).is_avoidance_target =
        get_or_declare_parameter<bool>(*node, ns);
    };

    const std::string ns = "avoidance.target_filtering.";
    set_target_flag(ObjectClassification::CAR, ns + "target_type.car");
    set_target_flag(ObjectClassification::TRUCK, ns + "target_type.truck");
    set_target_flag(ObjectClassification::TRAILER, ns + "target_type.trailer");
    set_target_flag(ObjectClassification::BUS, ns + "target_type.bus");
    set_target_flag(ObjectClassification::PEDESTRIAN, ns + "target_type.pedestrian");
    set_target_flag(ObjectClassification::BICYCLE, ns + "target_type.bicycle");
    set_target_flag(ObjectClassification::MOTORCYCLE, ns + "target_type.motorcycle");
    set_target_flag(ObjectClassification::UNKNOWN, ns + "target_type.unknown");

    p.object_check_goal_distance =
      get_or_declare_parameter<double>(*node, ns + "object_check_goal_distance");
    p.object_last_seen_threshold =
      get_or_declare_parameter<double>(*node, ns + "max_compensation_time");
  }

  {
    const std::string ns = "avoidance.target_filtering.parked_vehicle.";
    p.object_check_shiftable_ratio =
      get_or_declare_parameter<double>(*node, ns + "th_shiftable_ratio");
    p.object_check_min_road_shoulder_width =
      get_or_declare_parameter<double>(*node, ns + "min_road_shoulder_width");
  }

  {
    const std::string ns = "avoidance.target_filtering.avoidance_for_ambiguous_vehicle.";
    p.policy_ambiguous_vehicle = get_or_declare_parameter<std::string>(*node, ns + "policy");
    p.wait_and_see_target_behaviors = get_or_declare_parameter<std::vector<std::string>>(
      *node, ns + "wait_and_see.target_behaviors");
    p.wait_and_see_th_closest_distance =
      get_or_declare_parameter<double>(*node, ns + "wait_and_see.th_closest_distance");
    p.time_threshold_for_ambiguous_vehicle =
      get_or_declare_parameter<double>(*node, ns + "condition.th_stopped_time");
    p.distance_threshold_for_ambiguous_vehicle =
      get_or_declare_parameter<double>(*node, ns + "condition.th_moving_distance");
    p.object_ignore_section_traffic_light_in_front_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.traffic_light.front_distance");
    p.object_ignore_section_crosswalk_in_front_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.crosswalk.front_distance");
    p.object_ignore_section_crosswalk_behind_distance =
      get_or_declare_parameter<double>(*node, ns + "ignore_area.crosswalk.behind_distance");
  }

  // avoidance maneuver (longitudinal)
  {
    const std::string ns = "avoidance.avoidance.longitudinal.";
    p.min_prepare_time = get_or_declare_parameter<double>(*node, ns + "min_prepare_time");
    p.max_prepare_time = get_or_declare_parameter<double>(*node, ns + "max_prepare_time");
    p.min_prepare_distance = get_or_declare_parameter<double>(*node, ns + "min_prepare_distance");
    p.min_slow_down_speed = get_or_declare_parameter<double>(*node, ns + "min_slow_down_speed");
    p.buf_slow_down_speed = get_or_declare_parameter<double>(*node, ns + "buf_slow_down_speed");
    p.nominal_avoidance_speed =
      get_or_declare_parameter<double>(*node, ns + "nominal_avoidance_speed");
  }

  {
    const std::string ns = "avoidance.target_filtering.detection_area.";
    p.use_static_detection_area = get_or_declare_parameter<bool>(*node, ns + "static");
    p.object_check_min_forward_distance =
      get_or_declare_parameter<double>(*node, ns + "min_forward_distance");
    p.object_check_max_forward_distance =
      get_or_declare_parameter<double>(*node, ns + "max_forward_distance");
    p.object_check_backward_distance =
      get_or_declare_parameter<double>(*node, ns + "backward_distance");
  }

  // safety check
  {
    const std::string ns = "avoidance.safety_check.";
    p.hysteresis_factor_expand_rate =
      get_or_declare_parameter<double>(*node, ns + "hysteresis_factor_expand_rate");
  }

  // Opposite borrowing is opt-in; defaults also allow older parameter files to load.
  {
    const std::string ns = "avoidance_by_lane_change.";
    auto & o = p.opposite_lane;
    const auto parameter_with_default = [node](const std::string & name, auto fallback) {
      using T = decltype(fallback);
      return node->has_parameter(name) ? node->get_parameter(name).get_value<T>()
                                       : node->declare_parameter<T>(name, fallback);
    };
    o.enabled = parameter_with_default(ns + "enable_opposite_lane_borrowing", false);
    const auto number = [&](const std::string & name, double fallback, bool allow_zero = false) {
      const double value = parameter_with_default(ns + name, fallback);
      if (!std::isfinite(value) || (allow_zero ? value < 0.0 : value <= 0.0)) {
        throw std::invalid_argument(
          ns + name + " must be finite and " + (allow_zero ? "nonnegative" : "positive"));
      }
      return value;
    };
    o.campus_agv = parameter_with_default(ns + "campus_agv", false);
    o.attention_distance = number("opposite_lane.attention_distance", 50.0);
    o.static_speed_threshold = number("opposite_lane.static_speed_threshold", 0.4);
    o.static_observation_time = number("opposite_lane.static_observation_time", 1.0);
    o.candidate_speeds = parameter_with_default(
      ns + "opposite_lane.candidate_speeds", std::vector<double>{2.0, 1.5, 1.0, 0.5});
    if (o.candidate_speeds.empty() || !std::all_of(
        o.candidate_speeds.begin(), o.candidate_speeds.end(),
        [](double speed) { return std::isfinite(speed) && speed > 0.0 && speed <= 2.0; })) {
      throw std::invalid_argument("campus candidate speeds must be in (0, 2] m/s");
    }
    std::sort(o.candidate_speeds.begin(), o.candidate_speeds.end(), std::greater<double>{});
    o.max_speed = number("opposite_lane.max_speed", 2.0);
    o.max_prediction_horizon = number("opposite_lane.max_prediction_horizon", 30.0);
    o.time_resolution = number("opposite_lane.time_resolution", 0.5);
    o.min_time_gap = number("opposite_lane.min_time_gap", 3.0);
    o.longitudinal_margin = number("opposite_lane.longitudinal_margin", 2.0);
    o.lateral_margin = number("opposite_lane.lateral_margin", 0.5);
    o.clearance_hold_time = number("opposite_lane.clearance_hold_time", 1.0, true);
    o.return_distance_after_object = number("opposite_lane.return_distance_after_object", 5.0);
    o.data_timeout = number("opposite_lane.data_timeout", 0.5);
    o.max_planning_time_ms = number("opposite_lane.max_planning_time_ms", 50.0);
    o.prepare_distance = number("opposite_lane.prepare_distance", 3.0);
    o.min_shift_distance = number("opposite_lane.min_shift_distance", 5.0);
    o.max_return_extension_distance = number("opposite_lane.max_return_extension_distance", 20.0);
    o.return_extension_step = number("opposite_lane.return_extension_step", 1.0);
    if (o.return_extension_step > o.max_return_extension_distance ||
        o.max_return_extension_distance > 100.0) {
      throw std::invalid_argument("return extension step must not exceed a maximum extension of 100m");
    }
    o.road_boundary_margin = number("opposite_lane.road_boundary_margin", 0.1);
    o.tractor_rear_axle_to_hitch = number("trailer.tractor_rear_axle_to_hitch", 0.6, true);
    o.trailer_configuration_topic = parameter_with_default(
      ns + "trailer.configuration_topic", std::string{"/vehicle/status/trailer_configuration"});
    o.trailer_count = parameter_with_default(ns + "trailer.count", int64_t{0});
    o.trailer_ordered_types = parameter_with_default(
      ns + "trailer.ordered_types", std::vector<std::string>{"default"});
    if (o.trailer_count < 0 ||
        static_cast<size_t>(o.trailer_count) > o.trailer_ordered_types.size()) {
      throw std::invalid_argument("trailer.count must be between 0 and trailer.ordered_types size");
    }
    const auto types =
      parameter_with_default(ns + "trailer.type_names", std::vector<std::string>{"default"});
    for (const auto & type : types) {
      const auto prefix = "trailer.types." + type + ".";
      TrailerGeometry geometry;
      geometry.type = type;
      geometry.width = number(prefix + "width", 1.305);
      geometry.axle_to_body_front = number(prefix + "axle_to_body_front", 1.0);
      geometry.axle_to_body_rear = number(prefix + "axle_to_body_rear", 0.7);
      geometry.front_hitch_to_axle = number(prefix + "front_hitch_to_axle", 0.9);
      geometry.axle_to_rear_hitch = number(prefix + "axle_to_rear_hitch", 0.6, true);
      geometry.max_articulation_angle_rad =
        number(prefix + "max_articulation_angle_deg", 45.0) * 0.017453292519943295;
      if (!isValidTrailerGeometry(geometry)) {
        throw std::invalid_argument("invalid opposite-borrowing trailer geometry: " + type);
      }
      o.trailer_types.emplace(type, geometry);
    }
    for (int64_t i = 0; i < o.trailer_count; ++i) {
      if (o.trailer_types.count(o.trailer_ordered_types.at(i)) == 0) {
        throw std::invalid_argument("unknown configured trailer type: " + o.trailer_ordered_types.at(i));
      }
    }
    if (o.time_resolution < 0.05 || o.time_resolution > 0.5 ||
        o.max_prediction_horizon <= o.min_time_gap) {
      throw std::invalid_argument("opposite prediction requires resolution in [0.05, 0.5]s and horizon > gap");
    }
  }

  if (p.opposite_lane.enabled &&
      (!std::isfinite(parameters_->trajectory.lateral_jerk) ||
       parameters_->trajectory.lateral_jerk <= 0.0 ||
       !std::isfinite(parameters_->trajectory.max_longitudinal_acc) ||
       parameters_->trajectory.max_longitudinal_acc <= 0.0 ||
       !std::isfinite(parameters_->trajectory.min_longitudinal_acc) ||
       parameters_->trajectory.min_longitudinal_acc >= 0.0)) {
    throw std::invalid_argument("opposite borrowing requires positive jerk/acceleration and negative deceleration");
  }
  avoidance_parameters_ = std::make_shared<AvoidanceByLCParameters>(p);
}

SMIPtr AvoidanceByLaneChangeModuleManager::createNewSceneModuleInstance()
{
  return std::make_unique<AvoidanceByLaneChangeInterface>(
    name_, *node_, parameters_, avoidance_parameters_, rtc_interface_ptr_map_,
    objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_);
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::AvoidanceByLaneChangeModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
