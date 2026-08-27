#include "byd_event_rosbag_recorder/event_recorder_core.hpp"

#include <gtest/gtest.h>

using byd_event_rosbag_recorder::CaptureWindow;
using byd_event_rosbag_recorder::DiagnosticTransitionFilter;

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

TEST(DiagnosticTransitionFilter, RearmsOnlyAfterRecovery) {
  DiagnosticTransitionFilter filter(2, {".*"}, {"^event_rosbag_recorder"});
  EXPECT_TRUE(filter.should_trigger("lidar", 2));
  EXPECT_FALSE(filter.should_trigger("lidar", 2));
  EXPECT_FALSE(filter.should_trigger("event_rosbag_recorder: disk", 2));
  EXPECT_FALSE(filter.is_abnormal("event_rosbag_recorder: disk", 2));
  EXPECT_FALSE(filter.should_trigger("lidar", 0));
  EXPECT_TRUE(filter.should_trigger("lidar", 3));
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
