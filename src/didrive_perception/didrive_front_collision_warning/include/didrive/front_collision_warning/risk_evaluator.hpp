// Copyright 2026 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DIDRIVE__FRONT_COLLISION_WARNING__RISK_EVALUATOR_HPP_
#define DIDRIVE__FRONT_COLLISION_WARNING__RISK_EVALUATOR_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace didrive::front_collision_warning
{

struct Parameters
{
  double ego_front_offset{0.0};
  double ego_half_width{0.0};
  double max_longitudinal_gap{8.0};
  double critical_longitudinal_gap{1.5};
  double min_closing_speed{0.2};
  double max_ttc{3.0};
  double lateral_margin{0.5};
  double ego_stop_hold_time{0.5};
  double on_time_buffer{0.3};
  double off_time_buffer{0.5};
};

struct VehicleObservation
{
  double x;
  double y;
  double yaw;
  double length;
  double width;
  double velocity_x;
  double velocity_y;
};

struct Risk
{
  std::size_t index;
  double gap;
  double closing_speed;
  double ttc;
};

std::optional<Risk> evaluateRisk(
  const std::vector<VehicleObservation> & vehicles, double ego_velocity,
  const Parameters & parameters);

class WarningStateMachine
{
public:
  explicit WarningStateMachine(const Parameters & parameters);

  bool update(
    double now, bool autonomous, bool ego_stopped, bool inputs_fresh, bool raw_risk);
  void reset();

private:
  Parameters parameters_;
  std::optional<double> stopped_since_;
  std::optional<double> risk_since_;
  std::optional<double> clear_since_;
  std::optional<double> last_update_;
  bool active_{false};
};

}  // namespace didrive::front_collision_warning

#endif  // DIDRIVE__FRONT_COLLISION_WARNING__RISK_EVALUATOR_HPP_
