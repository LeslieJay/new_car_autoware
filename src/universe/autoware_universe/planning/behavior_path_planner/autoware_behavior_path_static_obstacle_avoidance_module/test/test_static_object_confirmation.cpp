// Copyright 2026 TIER IV, Inc.
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

#include "autoware/behavior_path_static_obstacle_avoidance_module/static_object_confirmation.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace autoware::behavior_path_planner
{
namespace
{
using Tracker = StaticObjectConfirmation;

Tracker::Observation observation(const double stamp)
{
  return {"car", stamp, 10.0, -1.2, 0.0, 0.4};
}

void confirm(Tracker & tracker)
{
  EXPECT_FALSE(tracker.update({observation(10.0)}, 10.0).at("car"));
  EXPECT_FALSE(tracker.update({observation(10.1)}, 10.1).at("car"));
  EXPECT_TRUE(tracker.update({observation(10.21)}, 10.21).at("car"));
}

TEST(StaticObjectConfirmation, ConfirmsBeforeLegacyAmbiguousVehicleDelay)
{
  Tracker tracker{Tracker::Parameters{}};
  confirm(tracker);
}

TEST(StaticObjectConfirmation, ReprocessedSingleFrameNeverConfirms)
{
  Tracker tracker{Tracker::Parameters{}};
  const auto frame = observation(10.0);
  EXPECT_FALSE(tracker.update({frame}, 10.0).at("car"));
  EXPECT_FALSE(tracker.update({frame}, 10.1).at("car"));
  EXPECT_FALSE(tracker.update({frame}, 10.2).at("car"));
  EXPECT_FALSE(tracker.update({frame}, 10.5).at("car"));
}

TEST(StaticObjectConfirmation, PositionMotionResetsEvenWhenReportedSpeedIsZero)
{
  Tracker tracker{Tracker::Parameters{}};
  confirm(tracker);
  auto frame = observation(10.31);
  frame.x += 0.2;
  EXPECT_FALSE(tracker.update({frame}, 10.31).at("car"));
}

TEST(StaticObjectConfirmation, SpeedThresholdAndNonfiniteValuesInvalidateImmediately)
{
  Tracker tracker{Tracker::Parameters{}};
  confirm(tracker);
  auto frame = observation(10.31);
  frame.speed = frame.speed_threshold;
  EXPECT_FALSE(tracker.update({frame}, 10.31).at("car"));
  frame = observation(10.41);
  EXPECT_FALSE(tracker.update({frame}, 10.41).at("car"));
  frame = observation(10.51);
  frame.speed = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(tracker.update({frame}, 10.51).at("car"));
}

TEST(StaticObjectConfirmation, MissingDetectionAndNewUuidDoNotInheritConfirmation)
{
  Tracker tracker{Tracker::Parameters{}};
  confirm(tracker);
  EXPECT_TRUE(tracker.update({}, 10.25).empty());
  EXPECT_FALSE(tracker.update({observation(10.31)}, 10.31).at("car"));
  auto frame = observation(10.41);
  frame.uuid = "another_car";
  EXPECT_FALSE(tracker.update({frame}, 10.41).at("another_car"));
}

TEST(StaticObjectConfirmation, GapsClockRegressionAndFutureDataResetEvidence)
{
  Tracker tracker{Tracker::Parameters{}};
  confirm(tracker);
  EXPECT_FALSE(tracker.update({observation(11.0)}, 11.0).at("car"));
  EXPECT_FALSE(tracker.update({observation(10.9)}, 10.9).at("car"));
  EXPECT_FALSE(tracker.update({observation(12.0)}, 11.0).at("car"));
}

TEST(StaticObjectConfirmation, StaleDataCannotRetainConfirmation)
{
  Tracker tracker{Tracker::Parameters{}};
  confirm(tracker);
  EXPECT_FALSE(tracker.update({observation(10.21)}, 11.0).at("car"));
  EXPECT_FALSE(tracker.update({observation(11.0)}, 11.0).at("car"));
}

TEST(StaticObjectConfirmation, DisabledModeLeavesLegacyFilteringInCharge)
{
  Tracker::Parameters parameters;
  parameters.enabled = false;
  Tracker tracker{parameters};
  EXPECT_TRUE(tracker.update({observation(10.0)}, 10.0).empty());
}

TEST(StaticObjectConfirmation, RejectsSingleFrameAndInvalidThresholdConfigurations)
{
  Tracker::Parameters parameters;
  parameters.minimum_observations = 1;
  EXPECT_THROW(Tracker{parameters}, std::invalid_argument);
  parameters = Tracker::Parameters{};
  parameters.observation_time = 0.0;
  EXPECT_THROW(Tracker{parameters}, std::invalid_argument);
}
}  // namespace
}  // namespace autoware::behavior_path_planner
