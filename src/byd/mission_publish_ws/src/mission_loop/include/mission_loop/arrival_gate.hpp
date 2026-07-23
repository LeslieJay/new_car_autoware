#ifndef MISSION_LOOP__ARRIVAL_GATE_HPP_
#define MISSION_LOOP__ARRIVAL_GATE_HPP_

#include <autoware_system_msgs/msg/autoware_state.hpp>

#include <cstdint>

namespace autoware::mission_loop
{

class ArrivalGate
{
public:
  void reset()
  {
    armed_ = false;
    handled_ = false;
    state_ = 0;
  }

  void observe(const uint8_t autoware_state)
  {
    state_ = autoware_state;
    if (state_ != autoware_system_msgs::msg::AutowareState::ARRIVED_GOAL) {
      armed_ = true;
    }
  }

  bool shouldHandleArrival(const bool has_localization) const
  {
    return armed_ && !handled_ && has_localization && isArrivalObserved();
  }

  bool isArrivalObserved() const
  {
    return state_ == autoware_system_msgs::msg::AutowareState::ARRIVED_GOAL;
  }

  void markHandled()
  {
    handled_ = true;
  }

private:
  bool armed_{false};
  bool handled_{false};
  uint8_t state_{0};
};

}  // namespace autoware::mission_loop

#endif  // MISSION_LOOP__ARRIVAL_GATE_HPP_
