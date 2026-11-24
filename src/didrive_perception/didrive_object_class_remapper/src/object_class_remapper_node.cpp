// Copyright 2026 BYD Company Limited. All rights reserved.
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

#include "didrive_object_class_remapper/object_class_remapper_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <utility>

namespace didrive::object_class_remapper
{

uint8_t remapLabel(
  uint8_t input_label, bool remap_vehicle_to_car, bool remap_twowheeler_to_car,
  bool fallback_to_unknown)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  switch (input_label) {
    case ObjectClassification::CAR:
      return ObjectClassification::CAR;

    case ObjectClassification::TRUCK:
    case ObjectClassification::BUS:
    case ObjectClassification::TRAILER:
      return remap_vehicle_to_car ? ObjectClassification::CAR : input_label;

    case ObjectClassification::MOTORCYCLE:
    case ObjectClassification::BICYCLE:
      return remap_twowheeler_to_car ? ObjectClassification::CAR : input_label;

    case ObjectClassification::PEDESTRIAN:
      return ObjectClassification::PEDESTRIAN;

    case ObjectClassification::UNKNOWN:
      return ObjectClassification::UNKNOWN;

    default:
      return fallback_to_unknown ? ObjectClassification::UNKNOWN : input_label;
  }
}

void remapPredictedObjects(
  autoware_perception_msgs::msg::PredictedObjects & msg, bool remap_vehicle_to_car,
  bool remap_twowheeler_to_car, bool fallback_to_unknown)
{
  for (auto & object : msg.objects) {
    if (object.classification.empty()) {
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      classification.probability = 1.0f;
      object.classification.push_back(classification);
      continue;
    }

    // Merge probabilities for identical remapped labels
    std::map<uint8_t, float> label_prob_map;
    for (const auto & classification : object.classification) {
      const uint8_t mapped_label =
        remapLabel(classification.label, remap_vehicle_to_car, remap_twowheeler_to_car, fallback_to_unknown);
      label_prob_map[mapped_label] += classification.probability;
    }

    object.classification.clear();
    for (const auto & [label, prob] : label_prob_map) {
      autoware_perception_msgs::msg::ObjectClassification classification;
      classification.label = label;
      classification.probability = std::min(prob, 1.0f);
      object.classification.push_back(classification);
    }

    // Sort descending by probability
    std::sort(
      object.classification.begin(), object.classification.end(),
      [](const auto & a, const auto & b) { return a.probability > b.probability; });
  }
}

ObjectClassRemapperNode::ObjectClassRemapperNode(const rclcpp::NodeOptions & options)
: Node("object_class_remapper_node", options)
{
  remap_vehicle_to_car_ = declare_parameter<bool>("remap_vehicle_to_car", false);
  remap_twowheeler_to_car_ = declare_parameter<bool>("remap_twowheeler_to_car", true);
  fallback_to_unknown_ = declare_parameter<bool>("fallback_to_unknown", true);

  sub_objects_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ObjectClassRemapperNode::objectsCallback, this, std::placeholders::_1));

  pub_objects_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "~/output/objects", rclcpp::QoS{1});
}

void ObjectClassRemapperNode::objectsCallback(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  auto output_msg = std::make_unique<autoware_perception_msgs::msg::PredictedObjects>(*msg);
  remapPredictedObjects(*output_msg, remap_vehicle_to_car_, remap_twowheeler_to_car_, fallback_to_unknown_);
  pub_objects_->publish(std::move(output_msg));
}

}  // namespace didrive::object_class_remapper

RCLCPP_COMPONENTS_REGISTER_NODE(didrive::object_class_remapper::ObjectClassRemapperNode)
