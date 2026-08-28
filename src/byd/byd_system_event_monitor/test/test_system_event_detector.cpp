#include "byd_system_event_monitor/system_event_detector.hpp"

#include <gtest/gtest.h>

using byd_system_event_monitor::AbnormalStopDetector;
using byd_system_event_monitor::AbnormalStopInput;
using byd_system_event_monitor::AbnormalStopParameters;
using byd_system_event_monitor::ModeTransitionDetector;
using byd_system_event_monitor::ModeTransitionParameters;

TEST(AbnormalStopDetector, TriggersOnceAfterFiveSeconds) {
  AbnormalStopDetector detector(AbnormalStopParameters{});
  AbnormalStopInput input{0, true, true, false, 20.0, false, 0.05};
  EXPECT_FALSE(detector.update(input));
  input.now_ns = 4'999'999'999LL;
  EXPECT_FALSE(detector.update(input));
  input.now_ns = 5'000'000'000LL;
  EXPECT_TRUE(detector.update(input));
  input.now_ns = 8'000'000'000LL;
  EXPECT_FALSE(detector.update(input));
}

TEST(AbnormalStopDetector, PlannedStopAndGoalSuppressDetection) {
  AbnormalStopDetector detector(AbnormalStopParameters{});
  AbnormalStopInput input{0, true, true, false, 20.0, true, 0.0};
  EXPECT_FALSE(detector.update(input));
  input.planned_stop = false;
  input.now_ns = 6'000'000'000LL;
  input.arrived_goal = true;
  EXPECT_FALSE(detector.update(input));
  input.arrived_goal = false;
  input.goal_distance_m = 1.5;
  input.now_ns = 12'000'000'000LL;
  EXPECT_FALSE(detector.update(input));
}

TEST(AbnormalStopDetector, RearmsAfterMovingForTwoSeconds) {
  AbnormalStopDetector detector(AbnormalStopParameters{});
  AbnormalStopInput input{0, true, true, false, 20.0, false, 0.0};
  detector.update(input);
  input.now_ns = 5'000'000'000LL;
  ASSERT_TRUE(detector.update(input));
  input.speed_mps = 0.4;
  input.now_ns = 6'000'000'000LL;
  detector.update(input);
  input.now_ns = 8'000'000'000LL;
  detector.update(input);
  input.speed_mps = 0.0;
  detector.update(input);
  input.now_ns = 13'000'000'000LL;
  EXPECT_TRUE(detector.update(input));
}

TEST(AbnormalStopDetector, WaitsForGraceAfterPlannedStopClears) {
  AbnormalStopDetector detector(AbnormalStopParameters{});
  AbnormalStopInput input{0, true, true, false, 20.0, true, 0.0};
  detector.update(input);
  input.planned_stop = false;
  input.now_ns = 1'000'000'000LL;
  detector.update(input);
  input.now_ns = 2'999'999'999LL;
  EXPECT_FALSE(detector.update(input));
  input.now_ns = 3'000'000'000LL;
  EXPECT_FALSE(detector.update(input));
  input.now_ns = 8'000'000'000LL;
  EXPECT_TRUE(detector.update(input));
}

TEST(AbnormalStopDetector, StaleInputsNeverTrigger) {
  AbnormalStopDetector detector(AbnormalStopParameters{});
  AbnormalStopInput input{0, false, true, false, 20.0, false, 0.0};
  detector.update(input);
  input.now_ns = 60'000'000'000LL;
  EXPECT_FALSE(detector.update(input));
}

TEST(ModeTransitionDetector, DetectsStableAutonomousToManual) {
  ModeTransitionParameters parameters;
  ModeTransitionDetector detector(parameters);
  EXPECT_FALSE(detector.update(0, parameters.autonomous));
  EXPECT_FALSE(detector.update(100'000'000LL, parameters.manual));
  EXPECT_FALSE(detector.update(599'999'999LL, parameters.manual));
  EXPECT_TRUE(detector.update(600'000'000LL, parameters.manual));
  EXPECT_FALSE(detector.update(700'000'000LL, parameters.manual));
}

TEST(ModeTransitionDetector, AllowsIntermediateModesWithinWindow) {
  ModeTransitionParameters parameters;
  ModeTransitionDetector detector(parameters);
  detector.update(0, parameters.autonomous);
  detector.update(500'000'000LL, parameters.no_command);
  detector.update(1'000'000'000LL, parameters.autonomous_steer_only);
  detector.update(1'400'000'000LL, parameters.manual);
  EXPECT_TRUE(detector.update(1'900'000'000LL, parameters.manual));
}

TEST(ModeTransitionDetector, StartupManualAndLateManualDoNotTrigger) {
  ModeTransitionParameters parameters;
  ModeTransitionDetector detector(parameters);
  detector.update(0, parameters.manual);
  EXPECT_FALSE(detector.update(1'000'000'000LL, parameters.manual));
  detector.update(2'000'000'000LL, parameters.autonomous);
  detector.update(2'100'000'000LL, parameters.no_command);
  detector.update(4'500'000'001LL, parameters.manual);
  EXPECT_FALSE(detector.update(5'100'000'001LL, parameters.manual));
}

TEST(ModeTransitionDetector, ManualMayStabilizeAfterTransitionWindow) {
  ModeTransitionParameters parameters;
  ModeTransitionDetector detector(parameters);
  detector.update(0, parameters.autonomous);
  detector.update(1'900'000'000LL, parameters.manual);
  EXPECT_TRUE(detector.update(2'400'000'000LL, parameters.manual));
}
