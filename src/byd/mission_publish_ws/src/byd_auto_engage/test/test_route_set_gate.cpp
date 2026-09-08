#include "route_set_gate.hpp"

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>

#include <gtest/gtest.h>

using autoware_adapi_v1_msgs::msg::RouteState;

TEST(RouteSetGate, RejectsSetStateLeftOverFromPreviousGoal)
{
  RouteSetGate gate;

  gate.on_goal(RouteState::SET);
  gate.observe(RouteState::SET);

  EXPECT_FALSE(gate.ready());
}

TEST(RouteSetGate, AcceptsSetObservedAfterCurrentGoal)
{
  RouteSetGate gate;

  gate.on_goal(RouteState::SET);
  gate.observe(RouteState::ARRIVED);
  EXPECT_FALSE(gate.ready());

  gate.observe(RouteState::SET);

  EXPECT_TRUE(gate.ready());
}

TEST(RouteSetGate, AcceptsSetAfterGoalArrivesDuringRouteTransition)
{
  RouteSetGate gate;

  gate.on_goal(RouteState::CHANGING);
  gate.observe(RouteState::SET);

  EXPECT_TRUE(gate.ready());
}
