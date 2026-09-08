// Copyright 2026 BYD.
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

#ifndef AGV_HIGH_PRECISION_REVERSE_CONTROLLER__PRECISION_REVERSE_CONTROLLER_HPP_
#define AGV_HIGH_PRECISION_REVERSE_CONTROLLER__PRECISION_REVERSE_CONTROLLER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace agv_high_precision_reverse_controller
{

struct PathPoint
{
  double x{};
  double y{};
  double yaw{};
  double s{};
  double curvature{};
};

struct VehicleState
{
  double x{};
  double y{};
  double yaw{};
  double speed{};  // Signed vehicle longitudinal speed; reverse is negative.
};

struct ControllerParameters
{
  double wheel_base{1.01};
  double max_steering_angle{0.60};
  double max_steering_rate{0.50};
  double steering_offset{0.0};
  double lateral_gain{1.20};
  double heading_gain{1.80};
  double softening_speed{0.15};

  double max_reverse_speed{0.35};
  double min_creep_speed{0.035};
  double comfortable_deceleration{0.25};
  double stop_margin{0.015};
  double final_approach_distance{0.60};
  double max_acceleration{0.30};
  double max_deceleration{0.50};
  double speed_kp{1.5};
  double speed_ki{0.20};
  double integral_limit{0.50};

  double goal_distance_tolerance{0.025};
  double goal_yaw_tolerance{0.020};
  double stop_speed_tolerance{0.025};
  double overshoot_tolerance{0.040};
  double max_lateral_error{0.40};
  double max_heading_error{0.60};
};

struct TrackingResult
{
  bool valid{false};
  bool goal_reached{false};
  bool tracking_error{false};
  std::string reason;
  double steering_angle{};
  double target_speed{};  // Positive magnitude; gear selects reverse.
  double acceleration{};
  double projected_s{};
  double remaining_s{};
  double lateral_error{};
  double heading_error{};
  double goal_distance{};
  double goal_yaw_error{};
  std::size_t segment_index{};
};

/// Pure, stateful reverse tracking module. The ROS node is only an adapter around this interface.
class PrecisionReverseController
{
public:
  explicit PrecisionReverseController(ControllerParameters parameters = {});

  bool setPath(std::vector<PathPoint> path, std::string * reason = nullptr);
  TrackingResult compute(const VehicleState & state, double dt);
  void reset();
  void clearPath();

  const std::vector<PathPoint> & path() const {return path_;}
  const ControllerParameters & parameters() const {return parameters_;}

private:
  struct Projection
  {
    bool valid{false};
    std::size_t segment_index{};
    double ratio{};
    double x{};
    double y{};
    double yaw{};
    double curvature{};
    double s{};
    double squared_distance{};
  };

  Projection project(const VehicleState & state) const;
  static double normalizeAngle(double angle);
  static double interpolateAngle(double from, double to, double ratio);

  ControllerParameters parameters_;
  std::vector<PathPoint> path_;
  std::size_t last_segment_index_{0};
  double last_projected_s_{0.0};
  double last_steering_{0.0};
  double last_target_speed_{0.0};
  double speed_integral_{0.0};
};

}  // namespace agv_high_precision_reverse_controller

#endif  // AGV_HIGH_PRECISION_REVERSE_CONTROLLER__PRECISION_REVERSE_CONTROLLER_HPP_
