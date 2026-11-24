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

#include <gtest/gtest.h>

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using didrive::object_class_remapper::remapLabel;
using didrive::object_class_remapper::remapPredictedObjects;

TEST(ObjectClassRemapperTest, RemapLabelDefault)
{
  // Vehicles and two-wheelers -> CAR
  EXPECT_EQ(remapLabel(ObjectClassification::CAR), ObjectClassification::CAR);
  EXPECT_EQ(remapLabel(ObjectClassification::TRUCK), ObjectClassification::CAR);
  EXPECT_EQ(remapLabel(ObjectClassification::BUS), ObjectClassification::CAR);
  EXPECT_EQ(remapLabel(ObjectClassification::TRAILER), ObjectClassification::CAR);
  EXPECT_EQ(remapLabel(ObjectClassification::MOTORCYCLE), ObjectClassification::CAR);
  EXPECT_EQ(remapLabel(ObjectClassification::BICYCLE), ObjectClassification::CAR);

  // Pedestrian -> PEDESTRIAN
  EXPECT_EQ(remapLabel(ObjectClassification::PEDESTRIAN), ObjectClassification::PEDESTRIAN);

  // Unknown & others -> UNKNOWN
  EXPECT_EQ(remapLabel(ObjectClassification::UNKNOWN), ObjectClassification::UNKNOWN);
  EXPECT_EQ(remapLabel(ObjectClassification::ANIMAL), ObjectClassification::UNKNOWN);
  EXPECT_EQ(remapLabel(ObjectClassification::HAZARD), ObjectClassification::UNKNOWN);
}

TEST(ObjectClassRemapperTest, RemapPredictedObjectsProbMerge)
{
  PredictedObjects msg;
  PredictedObject obj;

  ObjectClassification c1;
  c1.label = ObjectClassification::CAR;
  c1.probability = 0.5f;

  ObjectClassification c2;
  c2.label = ObjectClassification::TRUCK;
  c2.probability = 0.3f;

  ObjectClassification c3;
  c3.label = ObjectClassification::PEDESTRIAN;
  c3.probability = 0.2f;

  obj.classification = {c1, c2, c3};
  msg.objects.push_back(obj);

  remapPredictedObjects(msg, true, true, true);

  ASSERT_EQ(msg.objects.size(), 1u);
  const auto & classifications = msg.objects[0].classification;
  ASSERT_EQ(classifications.size(), 2u);

  // CAR + TRUCK merged -> 0.8
  EXPECT_EQ(classifications[0].label, ObjectClassification::CAR);
  EXPECT_FLOAT_EQ(classifications[0].probability, 0.8f);

  // PEDESTRIAN -> 0.2
  EXPECT_EQ(classifications[1].label, ObjectClassification::PEDESTRIAN);
  EXPECT_FLOAT_EQ(classifications[1].probability, 0.2f);
}

TEST(ObjectClassRemapperTest, RemapPredictedObjectsTwoWheelerFlag)
{
  PredictedObjects msg;
  PredictedObject obj;

  ObjectClassification c;
  c.label = ObjectClassification::BICYCLE;
  c.probability = 0.9f;

  obj.classification = {c};
  msg.objects.push_back(obj);

  // If remap_twowheeler_to_car is false, BICYCLE should not be remapped to CAR
  remapPredictedObjects(msg, true, false, true);

  ASSERT_EQ(msg.objects.size(), 1u);
  EXPECT_EQ(msg.objects[0].classification[0].label, ObjectClassification::BICYCLE);
}
