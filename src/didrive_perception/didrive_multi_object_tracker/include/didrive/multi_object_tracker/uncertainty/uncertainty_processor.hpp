// Copyright 2024 TIER IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee

#ifndef DIDRIVE__MULTI_OBJECT_TRACKER__UNCERTAINTY__UNCERTAINTY_PROCESSOR_HPP_
#define DIDRIVE__MULTI_OBJECT_TRACKER__UNCERTAINTY__UNCERTAINTY_PROCESSOR_HPP_

#include "didrive/multi_object_tracker/object_model/object_model.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>

namespace didrive::multi_object_tracker
{

namespace uncertainty
{

using didrive::multi_object_tracker::object_model::ObjectModel;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;

ObjectModel decodeObjectModel(const ObjectClassification & object_class);

DetectedObjects modelUncertainty(const DetectedObjects & detected_objects);

object_model::StateCovariance covarianceFromObjectClass(
  const DetectedObject & detected_object, const ObjectClassification & object_class);

void normalizeUncertainty(DetectedObjects & detected_objects);

}  // namespace uncertainty

}  // namespace didrive::multi_object_tracker

#endif  // DIDRIVE__MULTI_OBJECT_TRACKER__UNCERTAINTY__UNCERTAINTY_PROCESSOR_HPP_
