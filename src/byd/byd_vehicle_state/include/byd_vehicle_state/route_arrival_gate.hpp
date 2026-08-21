// Copyright 2026 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BYD_VEHICLE_STATE__ROUTE_ARRIVAL_GATE_HPP_
#define BYD_VEHICLE_STATE__ROUTE_ARRIVAL_GATE_HPP_

#include <autoware_adapi_v1_msgs/msg/route_state.hpp>

#include <cstdint>

namespace byd_vehicle_state
{

class RouteArrivalGate
{
public:
  void reset(const uint8_t current_route_state)
  {
    armed_ = false;
    observe(current_route_state);
  }

  void observe(const uint8_t route_state)
  {
    using RouteState = autoware_adapi_v1_msgs::msg::RouteState;
    if (route_state == RouteState::SET || route_state == RouteState::CHANGING) {
      armed_ = true;
    }
  }

  bool accepts(const uint8_t route_state) const
  {
    return armed_ && route_state == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED;
  }

private:
  bool armed_{false};
};

}  // namespace byd_vehicle_state

#endif  // BYD_VEHICLE_STATE__ROUTE_ARRIVAL_GATE_HPP_
