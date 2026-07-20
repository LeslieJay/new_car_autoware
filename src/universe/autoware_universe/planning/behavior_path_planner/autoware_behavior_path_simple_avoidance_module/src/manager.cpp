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
  p.publish_debug_marker = node->declare_parameter<bool>(ns + "publish_debug_marker", true);

  parameters_ = std::make_shared<SimpleAvoidanceParameters>(p);
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
