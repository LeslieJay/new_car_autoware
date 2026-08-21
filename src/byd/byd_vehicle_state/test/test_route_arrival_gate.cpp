// Copyright 2026 BYD
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

#include "byd_vehicle_state/route_arrival_gate.hpp"

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <gtest/gtest.h>

using autoware_adapi_v1_msgs::msg::RouteState;

TEST(RouteArrivalGate, RejectsStaleArrivalUntilNewRouteStarts)
{
  byd_vehicle_state::RouteArrivalGate gate;

  gate.reset(RouteState::ARRIVED);
  gate.observe(RouteState::ARRIVED);
  EXPECT_FALSE(gate.accepts(RouteState::ARRIVED));

  gate.observe(RouteState::CHANGING);
  EXPECT_FALSE(gate.accepts(RouteState::CHANGING));

  gate.observe(RouteState::ARRIVED);
  EXPECT_TRUE(gate.accepts(RouteState::ARRIVED));
}

TEST(RouteArrivalGate, ResetDisarmsPreviousRoute)
{
  byd_vehicle_state::RouteArrivalGate gate;

  gate.observe(RouteState::SET);
  EXPECT_TRUE(gate.accepts(RouteState::ARRIVED));

  gate.reset(RouteState::ARRIVED);
  EXPECT_FALSE(gate.accepts(RouteState::ARRIVED));
}

TEST(RouteArrivalGate, AcceptsRouteThatStartedBeforeGoalCallback)
{
  byd_vehicle_state::RouteArrivalGate gate;

  gate.reset(RouteState::SET);
  EXPECT_TRUE(gate.accepts(RouteState::ARRIVED));
}
