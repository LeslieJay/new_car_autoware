#include "byd_event_rosbag_recorder/event_recorder_core.hpp"

#include <algorithm>
#include <stdexcept>

namespace byd_event_rosbag_recorder {

CaptureWindow::CaptureWindow(int64_t pre_ns, int64_t post_ns,
                             int64_t max_duration_ns)
    : pre_ns_(pre_ns), post_ns_(post_ns), max_duration_ns_(max_duration_ns) {
  if (pre_ns < 0 || post_ns < 0 || max_duration_ns <= 0) {
    throw std::invalid_argument(
        "capture durations must be non-negative and cap must be positive");
  }
}

void CaptureWindow::trigger(int64_t time_ns) {
  if (!active_) {
    active_ = true;
    first_trigger_ns_ = time_ns;
    start_ns_ = time_ns - pre_ns_;
    end_ns_ = time_ns + post_ns_;
    trigger_count_ = 1;
    return;
  }
  ++trigger_count_;
  end_ns_ = std::min(time_ns + post_ns_, first_trigger_ns_ + max_duration_ns_);
}

bool CaptureWindow::active() const { return active_; }
bool CaptureWindow::expired(int64_t now_ns) const {
  return active_ && now_ns >= end_ns_;
}

void CaptureWindow::reset() {
  active_ = false;
  first_trigger_ns_ = 0;
  start_ns_ = 0;
  end_ns_ = 0;
  trigger_count_ = 0;
}

int64_t CaptureWindow::start_ns() const { return start_ns_; }
int64_t CaptureWindow::end_ns() const { return end_ns_; }
uint32_t CaptureWindow::trigger_count() const { return trigger_count_; }

DiagnosticTransitionFilter::DiagnosticTransitionFilter(
    uint8_t min_level, const std::vector<std::string> &includes,
    const std::vector<std::string> &excludes)
    : min_level_(min_level) {
  for (const auto &expression : includes) {
    includes_.emplace_back(expression);
  }
  for (const auto &expression : excludes) {
    excludes_.emplace_back(expression);
  }
}

bool DiagnosticTransitionFilter::matches(const std::string &name) const {
  const bool included =
      includes_.empty() ||
      std::any_of(includes_.begin(), includes_.end(),
                  [&name](const auto &expression) {
                    return std::regex_search(name, expression);
                  });
  const bool excluded = std::any_of(
      excludes_.begin(), excludes_.end(), [&name](const auto &expression) {
        return std::regex_search(name, expression);
      });
  return included && !excluded;
}

bool DiagnosticTransitionFilter::should_trigger(const std::string &name,
                                                uint8_t level) {
  const auto previous = last_levels_.find(name);
  const bool was_abnormal =
      previous != last_levels_.end() && previous->second >= min_level_;
  last_levels_[name] = level;
  return matches(name) && level >= min_level_ && !was_abnormal;
}

bool DiagnosticTransitionFilter::is_abnormal(const std::string &name,
                                             uint8_t level) const {
  return matches(name) && level >= min_level_;
}

bool topic_selected(const std::string &topic,
                    const std::vector<std::string> &exact_names,
                    const std::vector<std::regex> &includes,
                    const std::vector<std::regex> &excludes) {
  const bool exact = std::find(exact_names.begin(), exact_names.end(), topic) !=
                     exact_names.end();
  const bool regex_match = std::any_of(
      includes.begin(), includes.end(), [&topic](const auto &expression) {
        return std::regex_search(topic, expression);
      });
  const bool excluded = std::any_of(
      excludes.begin(), excludes.end(), [&topic](const auto &expression) {
        return std::regex_search(topic, expression);
      });
  const bool required =
      topic == "/system/event_trigger" || topic == "/diagnostics";
  return required || ((exact || regex_match) && !excluded);
}

} // namespace byd_event_rosbag_recorder
