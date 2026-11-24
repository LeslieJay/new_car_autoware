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

#ifndef DIDRIVE_OBJECT_CLASS_REMAPPER__OBJECT_CLASS_REMAPPER_NODE_HPP_
#define DIDRIVE_OBJECT_CLASS_REMAPPER__OBJECT_CLASS_REMAPPER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <memory>

namespace didrive::object_class_remapper
{

uint8_t remapLabel(
  uint8_t input_label, bool remap_vehicle_to_car = false, bool remap_twowheeler_to_car = true,
  bool fallback_to_unknown = true);

void remapPredictedObjects(
  autoware_perception_msgs::msg::PredictedObjects & msg, bool remap_vehicle_to_car = false,
  bool remap_twowheeler_to_car = true, bool fallback_to_unknown = true);

class ObjectClassRemapperNode : public rclcpp::Node
{
public:
  explicit ObjectClassRemapperNode(const rclcpp::NodeOptions & options);

private:
  void objectsCallback(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);

  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr pub_objects_;

  bool remap_vehicle_to_car_{false};
  bool remap_twowheeler_to_car_{true};
  bool fallback_to_unknown_{true};
};

}  // namespace didrive::object_class_remapper

#endif  // DIDRIVE_OBJECT_CLASS_REMAPPER__OBJECT_CLASS_REMAPPER_NODE_HPP_
