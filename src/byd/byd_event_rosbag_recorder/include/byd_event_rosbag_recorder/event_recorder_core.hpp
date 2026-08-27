#ifndef BYD_EVENT_ROSBAG_RECORDER__EVENT_RECORDER_CORE_HPP_
#define BYD_EVENT_ROSBAG_RECORDER__EVENT_RECORDER_CORE_HPP_

#include <cstdint>
#include <regex>
#include <string>
#include <unordered_map>
#include <vector>

namespace byd_event_rosbag_recorder {

class CaptureWindow {
public:
  CaptureWindow(int64_t pre_ns, int64_t post_ns, int64_t max_duration_ns);

  void trigger(int64_t time_ns);
  bool active() const;
  bool expired(int64_t now_ns) const;
  void reset();
  int64_t start_ns() const;
  int64_t end_ns() const;
  uint32_t trigger_count() const;

private:
  int64_t pre_ns_;
  int64_t post_ns_;
  int64_t max_duration_ns_;
  int64_t first_trigger_ns_{0};
  int64_t start_ns_{0};
  int64_t end_ns_{0};
  uint32_t trigger_count_{0};
  bool active_{false};
};

class DiagnosticTransitionFilter {
public:
  DiagnosticTransitionFilter(uint8_t min_level,
                             const std::vector<std::string> &includes,
                             const std::vector<std::string> &excludes);

  bool should_trigger(const std::string &name, uint8_t level);
  bool is_abnormal(const std::string &name, uint8_t level) const;

private:
  bool matches(const std::string &name) const;

  uint8_t min_level_;
  std::vector<std::regex> includes_;
  std::vector<std::regex> excludes_;
  std::unordered_map<std::string, uint8_t> last_levels_;
};

bool topic_selected(const std::string &topic,
                    const std::vector<std::string> &exact_names,
                    const std::vector<std::regex> &includes,
                    const std::vector<std::regex> &excludes);

} // namespace byd_event_rosbag_recorder

#endif // BYD_EVENT_ROSBAG_RECORDER__EVENT_RECORDER_CORE_HPP_
