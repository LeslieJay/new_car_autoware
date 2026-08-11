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

#include "autoware/behavior_path_simple_avoidance_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <cmath>
#include <functional>

namespace autoware::behavior_path_planner
{

void SimpleAvoidanceModuleManager::init(rclcpp::Node * node)
{
  initInterface(node, {});

  SimpleAvoidanceParameters p{};
  const std::string ns = "simple_avoidance.";
  // These are fallback defaults. Runtime tuning normally comes from
  // config/simple_avoidance.param.yaml.
  p.th_moving_speed = node->declare_parameter<double>(ns + "th_moving_speed", 0.5);
  p.min_forward_distance = node->declare_parameter<double>(ns + "min_forward_distance", 0.5);
  p.max_forward_distance = node->declare_parameter<double>(ns + "max_forward_distance", 60.0);
  p.lateral_margin = node->declare_parameter<double>(ns + "lateral_margin", 0.4);
  p.max_shift_length = node->declare_parameter<double>(ns + "max_shift_length", 4.0);
  p.min_prepare_distance = node->declare_parameter<double>(ns + "min_prepare_distance", 2.0);
  p.min_shifting_distance = node->declare_parameter<double>(ns + "min_shifting_distance", 4.0);
  p.shifting_lateral_jerk = node->declare_parameter<double>(ns + "shifting_lateral_jerk", 0.5);
  p.min_shifting_speed = node->declare_parameter<double>(ns + "min_shifting_speed", 1.0);
  p.return_distance_after_object =
    node->declare_parameter<double>(ns + "return_distance_after_object", 3.0);
  p.target_lost_time_threshold =
    node->declare_parameter<double>(ns + "target_lost_time_threshold", 1.0);
  p.target_hold_lateral_hysteresis =
    node->declare_parameter<double>(ns + "target_hold_lateral_hysteresis", 0.3);
  p.lateral_execution_threshold =
    node->declare_parameter<double>(ns + "lateral_execution_threshold", 0.05);
  p.trailer_configuration_topic = node->declare_parameter<std::string>(
    ns + "trailer.configuration_topic", "/vehicle/status/trailer_configuration");
  p.tractor_rear_axle_to_hitch =
    node->declare_parameter<double>(ns + "trailer.tractor_rear_axle_to_hitch", 0.6);
  p.trailer_footprint_sampling_interval =
    node->declare_parameter<double>(ns + "trailer.footprint_sampling_interval", 0.5);
  p.trailer_lateral_search_resolution =
    node->declare_parameter<double>(ns + "trailer.lateral_search_resolution", 0.1);
  p.trailer_return_search_resolution =
    node->declare_parameter<double>(ns + "trailer.return_search_resolution", 0.5);
  p.trailer_max_extra_return_distance =
    node->declare_parameter<double>(ns + "trailer.max_extra_return_distance", 20.0);
  p.trailer_max_planning_time_ms =
    node->declare_parameter<double>(ns + "trailer.max_planning_time_ms", 20.0);
  p.trailer_stationary_speed_threshold =
    node->declare_parameter<double>(ns + "trailer.stationary_speed_threshold", 0.05);

  const auto trailer_type_names =
    node->declare_parameter<std::vector<std::string>>(ns + "trailer.type_names", {"default"});
  constexpr double degree_to_radian = 0.017453292519943295;
  for (const auto & type : trailer_type_names) {
    const auto type_ns = ns + "trailer.types." + type + ".";
    TrailerGeometry geometry;
    geometry.type = type;
    geometry.width = node->declare_parameter<double>(type_ns + "width", 1.305);
    geometry.axle_to_body_front =
      node->declare_parameter<double>(type_ns + "axle_to_body_front", 1.0);
    geometry.axle_to_body_rear =
      node->declare_parameter<double>(type_ns + "axle_to_body_rear", 0.7);
    geometry.front_hitch_to_axle =
      node->declare_parameter<double>(type_ns + "front_hitch_to_axle", 0.9);
    geometry.axle_to_rear_hitch =
      node->declare_parameter<double>(type_ns + "axle_to_rear_hitch", 0.6);
    geometry.max_articulation_angle_rad =
      node->declare_parameter<double>(type_ns + "max_articulation_angle_deg", 45.0) *
      degree_to_radian;
    if (!isValidTrailerGeometry(geometry)) {
      throw std::invalid_argument("invalid trailer geometry for type: " + type);
    }
    p.trailer_types.emplace(type, geometry);
  }
  p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker", true);

  parameters_ = std::make_shared<SimpleAvoidanceParameters>(p);
  trailer_configuration_store_ = std::make_shared<TrailerConfigurationStore>();
  trailer_configuration_store_->update({}, parameters_->trailer_types);
  trailer_configuration_sub_ =
    node->create_subscription<byd_vehicle_msgs::msg::TrailerConfiguration>(
      p.trailer_configuration_topic, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
        &SimpleAvoidanceModuleManager::onTrailerConfiguration, this, std::placeholders::_1));
}

void SimpleAvoidanceModuleManager::onTrailerConfiguration(
  const byd_vehicle_msgs::msg::TrailerConfiguration & message)
{
  const auto current = trailer_configuration_store_->snapshot();
  if (message.trailer_types != current.types && planner_data_ && planner_data_->self_odometry) {
    const double speed = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
    if (speed > parameters_->trailer_stationary_speed_threshold) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Reject trailer configuration change while moving: speed=%.3fm/s threshold=%.3fm/s", speed,
        parameters_->trailer_stationary_speed_threshold);
      return;
    }
  }

  if (!trailer_configuration_store_->update(message.trailer_types, parameters_->trailer_types)) {
    RCLCPP_ERROR(
      node_->get_logger(), "Reject trailer configuration containing unknown or invalid type");
    return;
  }
  RCLCPP_INFO(
    node_->get_logger(), "Accepted trailer configuration: count=%zu", message.trailer_types.size());
}

void SimpleAvoidanceModuleManager::updateModuleParams(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto p = parameters_;
  const std::string ns = "simple_avoidance.";
  update_param(parameters, ns + "th_moving_speed", p->th_moving_speed);
  update_param(parameters, ns + "min_forward_distance", p->min_forward_distance);
  update_param(parameters, ns + "max_forward_distance", p->max_forward_distance);
  update_param(parameters, ns + "lateral_margin", p->lateral_margin);
  update_param(parameters, ns + "max_shift_length", p->max_shift_length);
  update_param(parameters, ns + "min_prepare_distance", p->min_prepare_distance);
  update_param(parameters, ns + "min_shifting_distance", p->min_shifting_distance);
  update_param(parameters, ns + "shifting_lateral_jerk", p->shifting_lateral_jerk);
  update_param(parameters, ns + "min_shifting_speed", p->min_shifting_speed);
  update_param(parameters, ns + "return_distance_after_object", p->return_distance_after_object);
  update_param(parameters, ns + "target_lost_time_threshold", p->target_lost_time_threshold);
  update_param(
    parameters, ns + "target_hold_lateral_hysteresis", p->target_hold_lateral_hysteresis);
  update_param(parameters, ns + "lateral_execution_threshold", p->lateral_execution_threshold);
  update_param(
    parameters, ns + "trailer.tractor_rear_axle_to_hitch", p->tractor_rear_axle_to_hitch);
  update_param(
    parameters, ns + "trailer.footprint_sampling_interval", p->trailer_footprint_sampling_interval);
  update_param(
    parameters, ns + "trailer.lateral_search_resolution", p->trailer_lateral_search_resolution);
  update_param(
    parameters, ns + "trailer.return_search_resolution", p->trailer_return_search_resolution);
  update_param(
    parameters, ns + "trailer.max_extra_return_distance", p->trailer_max_extra_return_distance);
  update_param(parameters, ns + "trailer.max_planning_time_ms", p->trailer_max_planning_time_ms);
  update_param(
    parameters, ns + "trailer.stationary_speed_threshold", p->trailer_stationary_speed_threshold);
  update_param(parameters, ns + "publish_debug_marker", p->publish_debug_marker);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) {
      observer.lock()->updateModuleParams(p);
    }
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SimpleAvoidanceModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
