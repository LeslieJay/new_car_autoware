#include "byd_event_rosbag_recorder/event_recorder_core.hpp"

#include <gtest/gtest.h>

using byd_event_rosbag_recorder::CaptureWindow;
using byd_event_rosbag_recorder::DiagnosticTransitionFilter;
using byd_event_rosbag_recorder::TriggerPolicy;
using byd_event_rosbag_recorder::TriggerPolicyParameters;
using byd_event_rosbag_recorder::TriggerSource;

TEST(CaptureWindow, MergesTriggersAndCapsDuration) {
  CaptureWindow window(30'000'000'000LL, 30'000'000'000LL, 600'000'000'000LL);
  window.trigger(100'000'000'000LL);
  EXPECT_EQ(window.start_ns(), 70'000'000'000LL);
  EXPECT_EQ(window.end_ns(), 130'000'000'000LL);
  EXPECT_EQ(window.trigger_count(), 1U);

  window.trigger(120'000'000'000LL);
  EXPECT_EQ(window.start_ns(), 70'000'000'000LL);
  EXPECT_EQ(window.end_ns(), 150'000'000'000LL);
  EXPECT_EQ(window.trigger_count(), 2U);

  window.trigger(900'000'000'000LL);
  EXPECT_EQ(window.end_ns(), 670'000'000'000LL);
}

TEST(CaptureWindow, CapsInitialWindowFromPreTriggerStart) {
  CaptureWindow window(30'000'000'000LL, 30'000'000'000LL, 40'000'000'000LL);
  window.trigger(100'000'000'000LL);
  EXPECT_EQ(window.start_ns(), 70'000'000'000LL);
  EXPECT_EQ(window.end_ns(), 110'000'000'000LL);
}

TEST(DiagnosticTransitionFilter, RearmsOnlyAfterRecovery) {
  DiagnosticTransitionFilter filter(2, {".*"}, {"^event_rosbag_recorder"});
  EXPECT_TRUE(filter.should_trigger("lidar", 2));
  EXPECT_FALSE(filter.should_trigger("lidar", 2));
  EXPECT_FALSE(filter.should_trigger("event_rosbag_recorder: disk", 2));
  EXPECT_FALSE(filter.is_abnormal("event_rosbag_recorder: disk", 2));
  EXPECT_FALSE(filter.should_trigger("lidar", 0));
  EXPECT_TRUE(filter.should_trigger("lidar", 3));
}

TEST(DiagnosticTransitionFilter, RepeatsWhileAbnormalWhenTransitionsDisabled) {
  DiagnosticTransitionFilter filter(2, {".*"}, {});
  EXPECT_TRUE(filter.should_trigger("lidar", 2, false));
  EXPECT_TRUE(filter.should_trigger("lidar", 2, false));
  EXPECT_FALSE(filter.should_trigger("lidar", 1, false));
}

TEST(TopicSelection, RequiredEvidenceTopicsCannotBeExcluded) {
  const std::vector<std::regex> excluded{std::regex(".*")};
  EXPECT_TRUE(byd_event_rosbag_recorder::topic_selected("/diagnostics", {}, {},
                                                        excluded));
  EXPECT_TRUE(byd_event_rosbag_recorder::topic_selected("/system/event_trigger",
                                                        {}, {}, excluded));
  EXPECT_FALSE(byd_event_rosbag_recorder::topic_selected(
      "/debug/noise", {"/debug/noise"}, {}, excluded));
}

TEST(TriggerPolicy, DefaultsToSystemEventMonitorEventsOnly) {
  const TriggerPolicy policy;

  EXPECT_TRUE(policy.accepts(TriggerSource::EVENT_TOPIC, "abnormal_stop"));
  EXPECT_TRUE(
      policy.accepts(TriggerSource::EVENT_TOPIC, "autonomous_to_manual"));
  EXPECT_FALSE(policy.accepts(TriggerSource::EVENT_TOPIC, "manual_test"));
  EXPECT_FALSE(policy.accepts(TriggerSource::DIAGNOSTICS, "lidar_failure"));
  EXPECT_FALSE(policy.accepts(TriggerSource::SERVICE, "abnormal_stop"));
}

TEST(TriggerPolicy, ParametersCanEnableOtherTriggerSources) {
  TriggerPolicyParameters parameters;
  parameters.event_topic_enabled = false;
  parameters.diagnostics_enabled = true;
  parameters.service_enabled = true;
  parameters.allowed_event_types = {"abnormal_stop", "manual_test"};
  const TriggerPolicy policy(parameters);

  EXPECT_TRUE(policy.accepts(TriggerSource::DIAGNOSTICS, "lidar_failure"));
  EXPECT_TRUE(policy.accepts(TriggerSource::SERVICE, "manual_test"));
  EXPECT_FALSE(policy.accepts(TriggerSource::SERVICE, "unlisted_event"));
  EXPECT_FALSE(policy.accepts(TriggerSource::EVENT_TOPIC, "abnormal_stop"));
}
