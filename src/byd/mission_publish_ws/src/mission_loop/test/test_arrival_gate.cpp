#include "mission_loop/arrival_gate.hpp"

#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <gtest/gtest.h>

using autoware_system_msgs::msg::AutowareState;

TEST(ArrivalGate, RequiresRearmLocalizationAndOneShotHandling)
{
  autoware::mission_loop::ArrivalGate gate;

  gate.reset();
  gate.observe(AutowareState::ARRIVED_GOAL);
  EXPECT_TRUE(gate.isArrivalObserved());
  EXPECT_FALSE(gate.shouldHandleArrival(true));

  gate.observe(AutowareState::DRIVING);
  gate.observe(AutowareState::ARRIVED_GOAL);
  EXPECT_FALSE(gate.shouldHandleArrival(false));
  EXPECT_TRUE(gate.shouldHandleArrival(true));

  gate.markHandled();
  EXPECT_FALSE(gate.shouldHandleArrival(true));
}
