#include "mission_loop/stop_gate.hpp"

#include <gtest/gtest.h>

TEST(StopGate, RequiresContinuousStopDuration)
{
  autoware::mission_loop::StopGate gate(0.05, 1.0);

  gate.observe(0.04, 10.0);
  EXPECT_FALSE(gate.isSatisfied(10.9));
  EXPECT_TRUE(gate.isSatisfied(11.0));

  gate.observe(0.06, 11.1);
  EXPECT_FALSE(gate.isSatisfied(12.1));

  gate.observe(0.0, 12.2);
  EXPECT_FALSE(gate.isSatisfied(13.1));
  EXPECT_TRUE(gate.isSatisfied(13.2));
}
