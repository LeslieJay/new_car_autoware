// Copyright 2026 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "didrive/front_collision_warning/risk_evaluator.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace didrive::front_collision_warning
{

TEST(RiskEvaluator, DetectsVehicleClosingInsideFrontCorridor)
{
  Parameters parameters;
  parameters.ego_front_offset = 1.554;
  parameters.ego_half_width = 0.6525;

  const VehicleObservation vehicle{
    7.0, 0.0, 0.0, 4.0, 1.8, -2.0, 0.0};

  const auto risk = evaluateRisk({vehicle}, 0.0, parameters);

  ASSERT_TRUE(risk.has_value());
  EXPECT_NEAR(risk->gap, 3.446, 1e-3);
  EXPECT_NEAR(risk->closing_speed, 2.0, 1e-6);
  EXPECT_NEAR(risk->ttc, 1.723, 1e-3);
}

TEST(RiskEvaluator, IgnoresVehiclesThatAreNotClosingOrOutsideCorridor)
{
  Parameters parameters;
  parameters.ego_front_offset = 1.5;
  parameters.ego_half_width = 0.65;

  const VehicleObservation static_vehicle{5.0, 0.0, 0.0, 4.0, 1.8, 0.0, 0.0};
  const VehicleObservation adjacent_vehicle{5.0, 3.0, 0.0, 4.0, 1.8, -2.0, 0.0};
  const VehicleObservation retreating_vehicle{5.0, 0.0, 0.0, 4.0, 1.8, 2.0, 0.0};

  EXPECT_FALSE(evaluateRisk({static_vehicle}, 0.0, parameters));
  EXPECT_FALSE(evaluateRisk({adjacent_vehicle}, 0.0, parameters));
  EXPECT_FALSE(evaluateRisk({retreating_vehicle}, 0.0, parameters));
}

TEST(RiskEvaluator, UsesCriticalGapEvenWhenTtcIsLong)
{
  Parameters parameters;
  parameters.ego_front_offset = 1.5;
  parameters.ego_half_width = 0.65;
  const VehicleObservation slowly_closing{4.4, 0.0, 0.0, 4.0, 1.8, -0.25, 0.0};

  const auto risk = evaluateRisk({slowly_closing}, 0.0, parameters);

  ASSERT_TRUE(risk);
  EXPECT_NEAR(risk->gap, 0.9, 1e-6);
  EXPECT_GT(risk->ttc, parameters.max_ttc);
}

TEST(RiskEvaluator, RotatesObjectLocalVelocityIntoBaseFrame)
{
  Parameters parameters;
  parameters.ego_front_offset = 1.5;
  parameters.ego_half_width = 0.65;
  const VehicleObservation oncoming{
    6.0, 0.0, 3.141592653589793, 4.0, 1.8, 2.0, 0.0};

  const auto risk = evaluateRisk({oncoming}, 0.0, parameters);

  ASSERT_TRUE(risk);
  EXPECT_NEAR(risk->closing_speed, 2.0, 1e-6);
}

TEST(RiskEvaluator, RejectsInvalidObservations)
{
  Parameters parameters;
  parameters.ego_front_offset = 1.5;
  parameters.ego_half_width = 0.65;
  const VehicleObservation zero_length{5.0, 0.0, 0.0, 0.0, 1.8, -2.0, 0.0};
  const VehicleObservation not_finite{
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 4.0, 1.8, -2.0, 0.0};

  EXPECT_FALSE(evaluateRisk({zero_length}, 0.0, parameters));
  EXPECT_FALSE(evaluateRisk({not_finite}, 0.0, parameters));
}

TEST(WarningStateMachine, AppliesStopHoldAndOnOffBuffers)
{
  Parameters parameters;
  WarningStateMachine state(parameters);

  EXPECT_FALSE(state.update(0.0, true, true, true, true));
  EXPECT_FALSE(state.update(0.5, true, true, true, true));
  EXPECT_FALSE(state.update(0.79, true, true, true, true));
  EXPECT_TRUE(state.update(0.80, true, true, true, true));
  EXPECT_TRUE(state.update(1.0, true, true, true, false));
  EXPECT_TRUE(state.update(1.49, true, true, true, false));
  EXPECT_FALSE(state.update(1.50, true, true, true, false));
}

TEST(WarningStateMachine, ClearsImmediatelyWhenPrerequisitesFail)
{
  Parameters parameters;
  parameters.ego_stop_hold_time = 0.0;
  parameters.on_time_buffer = 0.0;
  WarningStateMachine state(parameters);

  EXPECT_TRUE(state.update(1.0, true, true, true, true));
  EXPECT_FALSE(state.update(1.1, false, true, true, true));
  EXPECT_TRUE(state.update(2.0, true, true, true, true));
  EXPECT_FALSE(state.update(2.1, true, true, false, true));
  EXPECT_TRUE(state.update(3.0, true, true, true, true));
  EXPECT_FALSE(state.update(2.0, true, true, true, true));
}

}  // namespace didrive::front_collision_warning
