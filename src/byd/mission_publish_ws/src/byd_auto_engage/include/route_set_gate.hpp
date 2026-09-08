#ifndef BYD_AUTO_ENGAGE__ROUTE_SET_GATE_HPP_
#define BYD_AUTO_ENGAGE__ROUTE_SET_GATE_HPP_

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>

#include <cstdint>

class RouteSetGate
{
public:
  // A SET state already present when the goal arrives belongs to the previous route.
  void on_goal(const uint8_t current_route_state)
  {
    using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
    ready_ = false;
    observed_non_set_ = current_route_state != RouteState::SET;
  }

  void observe(const uint8_t route_state)
  {
    using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
    if (route_state != RouteState::SET) {
      observed_non_set_ = true;
    } else if (observed_non_set_) {
      ready_ = true;
    }
  }

  bool ready() const { return ready_; }

private:
  bool observed_non_set_{false};
  bool ready_{false};
};

#endif  // BYD_AUTO_ENGAGE__ROUTE_SET_GATE_HPP_
