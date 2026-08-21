#ifndef MISSION_LOOP__STOP_GATE_HPP_
#define MISSION_LOOP__STOP_GATE_HPP_

#include <optional>

namespace autoware::mission_loop
{

class StopGate
{
public:
  StopGate(const double max_speed_mps, const double required_duration_s)
  : max_speed_mps_(max_speed_mps), required_duration_s_(required_duration_s)
  {
  }

  void observe(const double speed_mps, const double now_s)
  {
    if (speed_mps > max_speed_mps_) {
      stopped_since_s_.reset();
    } else if (!stopped_since_s_.has_value()) {
      stopped_since_s_ = now_s;
    }
  }

  bool isSatisfied(const double now_s) const
  {
    return stopped_since_s_.has_value() &&
           now_s - stopped_since_s_.value() >= required_duration_s_;
  }

  void reset() {stopped_since_s_.reset();}

private:
  double max_speed_mps_;
  double required_duration_s_;
  std::optional<double> stopped_since_s_;
};

}  // namespace autoware::mission_loop

#endif  // MISSION_LOOP__STOP_GATE_HPP_
