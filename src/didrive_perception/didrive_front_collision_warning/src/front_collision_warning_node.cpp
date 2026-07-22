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

#include "didrive/front_collision_warning/risk_evaluator.hpp"

#include <autoware_control_msgs/msg/safety_state.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace didrive::front_collision_warning
{

using autoware_control_msgs::msg::SafetyState;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_vehicle_msgs::msg::ControlModeReport;
using autoware_vehicle_msgs::msg::VelocityReport;

class FrontCollisionWarningNode : public rclcpp::Node
{
public:
  FrontCollisionWarningNode()
  : Node("front_collision_warning")
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    parameters_.ego_front_offset = vehicle_info.max_longitudinal_offset_m;
    parameters_.ego_half_width = vehicle_info.vehicle_width_m * 0.5;
    parameters_.max_longitudinal_gap = declare_parameter<double>("max_longitudinal_gap", 8.0);
    parameters_.critical_longitudinal_gap =
      declare_parameter<double>("critical_longitudinal_gap", 1.5);
    parameters_.min_closing_speed = declare_parameter<double>("min_closing_speed", 0.2);
    parameters_.max_ttc = declare_parameter<double>("max_ttc", 3.0);
    parameters_.lateral_margin = declare_parameter<double>("lateral_margin", 0.5);
    parameters_.ego_stop_hold_time = declare_parameter<double>("ego_stop_hold_time", 0.5);
    parameters_.on_time_buffer = declare_parameter<double>("on_time_buffer", 0.3);
    parameters_.off_time_buffer = declare_parameter<double>("off_time_buffer", 0.5);
    stopped_speed_threshold_ = declare_parameter<double>("stopped_speed_threshold", 0.1);
    input_timeout_ = declare_parameter<double>("input_timeout", 0.5);
    warning_publish_rate_ = declare_parameter<double>("warning_publish_rate", 5.0);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    validateParameters();
    state_machine_ = std::make_unique<WarningStateMachine>(parameters_);

    objects_sub_ = create_subscription<TrackedObjects>(
      "~/input/objects", rclcpp::QoS{1},
      [this](TrackedObjects::ConstSharedPtr msg) {
        objects_ = std::move(msg);
        objects_received_at_ = now();
      });
    velocity_sub_ = create_subscription<VelocityReport>(
      "~/input/velocity", rclcpp::QoS{1},
      [this](VelocityReport::ConstSharedPtr msg) {
        velocity_ = std::move(msg);
        velocity_received_at_ = now();
      });
    control_mode_sub_ = create_subscription<ControlModeReport>(
      "~/input/control_mode", rclcpp::QoS{1},
      [this](ControlModeReport::ConstSharedPtr msg) {
        control_mode_ = std::move(msg);
        control_mode_received_at_ = now();
      });
    warning_pub_ = create_publisher<SafetyState>("~/output/state", rclcpp::QoS{1});
    timer_ = create_wall_timer(std::chrono::milliseconds{100}, [this]() {onTimer();});
  }

private:
  struct TargetDetails
  {
    Risk risk;
    std::string uuid;
  };

  void validateParameters() const
  {
    if (
      parameters_.max_longitudinal_gap <= 0.0 ||
      parameters_.critical_longitudinal_gap < 0.0 || parameters_.min_closing_speed <= 0.0 ||
      parameters_.max_ttc <= 0.0 || parameters_.lateral_margin < 0.0 ||
      parameters_.ego_stop_hold_time < 0.0 || parameters_.on_time_buffer < 0.0 ||
      parameters_.off_time_buffer < 0.0 || stopped_speed_threshold_ < 0.0 ||
      input_timeout_ <= 0.0 || warning_publish_rate_ <= 0.0)
    {
      throw std::invalid_argument("front collision warning parameters are out of range");
    }
  }

  static bool isVehicle(const TrackedObject & object)
  {
    if (object.classification.empty()) {
      return false;
    }
    const auto best = std::max_element(
      object.classification.begin(), object.classification.end(),
      [](const auto & lhs, const auto & rhs) {return lhs.probability < rhs.probability;});
    return best->label == ObjectClassification::CAR || best->label == ObjectClassification::TRUCK ||
           best->label == ObjectClassification::BUS ||
           best->label == ObjectClassification::TRAILER;
  }

  std::optional<TargetDetails> findRisk() const
  {
    if (!objects_ || objects_->header.frame_id != base_frame_) {
      return std::nullopt;
    }

    std::vector<VehicleObservation> observations;
    std::vector<std::string> uuids;
    observations.reserve(objects_->objects.size());
    uuids.reserve(objects_->objects.size());
    for (const auto & object : objects_->objects) {
      if (!isVehicle(object)) {
        continue;
      }
      const auto & pose = object.kinematics.pose_with_covariance.pose;
      const auto & twist = object.kinematics.twist_with_covariance.twist;
      observations.push_back(
        VehicleObservation{
          pose.position.x, pose.position.y, tf2::getYaw(pose.orientation),
          object.shape.dimensions.x, object.shape.dimensions.y, twist.linear.x, twist.linear.y});
      uuids.push_back(autoware_utils_uuid::to_hex_string(object.object_id));
    }

    const auto risk = evaluateRisk(observations, velocity_->longitudinal_velocity, parameters_);
    if (!risk) {
      return std::nullopt;
    }
    return TargetDetails{*risk, uuids.at(risk->index)};
  }

  bool isFresh(const std::optional<rclcpp::Time> & received_at, const rclcpp::Time & stamp) const
  {
    if (!received_at) {
      return false;
    }
    const double age = (stamp - *received_at).seconds();
    return age >= 0.0 && age <= input_timeout_;
  }

  void publishState(const uint8_t state, const double distance)
  {
    SafetyState message;
    message.current_state = state;
    message.distance = static_cast<float>(distance);
    warning_pub_->publish(message);
  }

  void onTimer()
  {
    const auto stamp = now();
    const bool inputs_fresh = objects_ && velocity_ && control_mode_ &&
      isFresh(objects_received_at_, stamp) && isFresh(velocity_received_at_, stamp) &&
      isFresh(control_mode_received_at_, stamp);
    const bool autonomous =
      inputs_fresh && control_mode_->mode == ControlModeReport::AUTONOMOUS;
    const bool stopped =
      inputs_fresh && std::abs(velocity_->longitudinal_velocity) <= stopped_speed_threshold_;

    std::optional<TargetDetails> target;
    if (inputs_fresh && autonomous && stopped) {
      target = findRisk();
      if (target) {
        last_target_ = target;
      }
    }

    const bool was_active = active_;
    active_ = state_machine_->update(
      stamp.seconds(), autonomous, stopped, inputs_fresh, target.has_value());

    if (!published_initial_state_) {
      publishState(SafetyState::NORMAL, -1.0);
      published_initial_state_ = true;
    }
    if (active_) {
      const double publish_period = 1.0 / warning_publish_rate_;
      if (!last_warning_publish_ || (stamp - *last_warning_publish_).seconds() >= publish_period) {
        publishState(SafetyState::EMERGENCY, last_target_ ? last_target_->risk.gap : -1.0);
        last_warning_publish_ = stamp;
      }
      if (!was_active && last_target_) {
        RCLCPP_WARN(
          get_logger(), "front collision warning ON: object=%s gap=%.2f m closing=%.2f m/s ttc=%.2f s",
          last_target_->uuid.c_str(), last_target_->risk.gap, last_target_->risk.closing_speed,
          last_target_->risk.ttc);
      }
    } else if (was_active) {
      publishState(SafetyState::NORMAL, -1.0);
      RCLCPP_INFO(get_logger(), "front collision warning OFF");
      last_warning_publish_.reset();
      last_target_.reset();
    }
  }

  Parameters parameters_;
  double stopped_speed_threshold_;
  double input_timeout_;
  double warning_publish_rate_;
  std::string base_frame_;
  std::unique_ptr<WarningStateMachine> state_machine_;

  TrackedObjects::ConstSharedPtr objects_;
  VelocityReport::ConstSharedPtr velocity_;
  ControlModeReport::ConstSharedPtr control_mode_;
  std::optional<rclcpp::Time> objects_received_at_;
  std::optional<rclcpp::Time> velocity_received_at_;
  std::optional<rclcpp::Time> control_mode_received_at_;
  std::optional<rclcpp::Time> last_warning_publish_;
  std::optional<TargetDetails> last_target_;
  bool published_initial_state_{false};
  bool active_{false};

  rclcpp::Subscription<TrackedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr control_mode_sub_;
  rclcpp::Publisher<SafetyState>::SharedPtr warning_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace didrive::front_collision_warning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<didrive::front_collision_warning::FrontCollisionWarningNode>());
  rclcpp::shutdown();
  return 0;
}
