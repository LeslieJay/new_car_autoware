#ifndef BYD_SYSTEM_EVENT_MONITOR__SYSTEM_EVENT_DETECTOR_HPP_
#define BYD_SYSTEM_EVENT_MONITOR__SYSTEM_EVENT_DETECTOR_HPP_

#include <cstdint>
#include <optional>
#include <vector>

namespace byd_system_event_monitor {

struct AbnormalStopInput {
  int64_t now_ns{};
  bool inputs_fresh{};
  bool autonomous{};
  bool arrived_goal{};
  double goal_distance_m{};
  bool planned_stop{};
  double speed_mps{};
};

struct AbnormalStopParameters {
  double stop_speed_mps{0.1};
  double rearm_speed_mps{0.3};
  double goal_distance_m{2.0};
  int64_t hold_ns{5'000'000'000LL};
  int64_t rearm_ns{2'000'000'000LL};
  int64_t planned_stop_grace_ns{2'000'000'000LL};
};

class AbnormalStopDetector {
public:
  explicit AbnormalStopDetector(AbnormalStopParameters parameters);
  bool update(const AbnormalStopInput &input);
  void reset();

private:
  AbnormalStopParameters parameters_;
  bool armed_{true};
  bool last_planned_stop_{false};
  int64_t planned_stop_grace_until_ns_{0};
  std::optional<int64_t> stopped_since_ns_;
  std::optional<int64_t> moving_since_ns_;
};

struct ModeTransitionParameters {
  int64_t manual_stable_ns{500'000'000LL};
  int64_t transition_window_ns{2'000'000'000LL};
  uint8_t no_command{0};
  uint8_t autonomous{1};
  uint8_t autonomous_steer_only{2};
  uint8_t autonomous_velocity_only{3};
  uint8_t manual{4};
};

class ModeTransitionDetector {
public:
  explicit ModeTransitionDetector(ModeTransitionParameters parameters);
  bool update(int64_t now_ns, uint8_t mode);
  void reset();
  const std::vector<uint8_t> &transition_sequence() const;

private:
  bool is_intermediate(uint8_t mode) const;

  ModeTransitionParameters parameters_;
  bool observed_autonomous_{false};
  bool emitted_{false};
  std::optional<int64_t> last_autonomous_ns_;
  std::optional<int64_t> manual_since_ns_;
  std::vector<uint8_t> sequence_;
};

} // namespace byd_system_event_monitor

#endif // BYD_SYSTEM_EVENT_MONITOR__SYSTEM_EVENT_DETECTOR_HPP_
