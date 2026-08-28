#include "byd_system_event_monitor/system_event_detector.hpp"

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/planning_factor_array.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <byd_vehicle_msgs/msg/event_trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace byd_system_event_monitor {

template <typename Message> struct TimedMessage {
  typename Message::ConstSharedPtr value;
  int64_t received_ns{};

  bool fresh(int64_t now_ns, int64_t limit_ns) const {
    return value && now_ns >= received_ns && now_ns - received_ns <= limit_ns;
  }
};

struct FactorState {
  std::string topic;
  TimedMessage<autoware_internal_planning_msgs::msg::PlanningFactorArray>
      message;
  rclcpp::Subscription<
      autoware_internal_planning_msgs::msg::PlanningFactorArray>::SharedPtr
      subscription;
};

class SystemEventMonitorNode : public rclcpp::Node {
public:
  SystemEventMonitorNode()
      : Node("system_event_monitor_node"),
        stop_detector_(make_stop_parameters()),
        mode_detector_(make_mode_parameters()),
        velocity_fresh_ns_(seconds_to_ns(
            declare_parameter("freshness.velocity_seconds", 0.5))),
        mode_fresh_ns_(seconds_to_ns(
            declare_parameter("freshness.control_mode_seconds", 1.0))),
        state_fresh_ns_(seconds_to_ns(
            declare_parameter("freshness.autoware_state_seconds", 2.0))),
        localization_fresh_ns_(seconds_to_ns(
            declare_parameter("freshness.localization_seconds", 0.5))),
        trajectory_fresh_ns_(seconds_to_ns(
            declare_parameter("freshness.trajectory_seconds", 1.0))),
        factor_fresh_ns_(seconds_to_ns(
            declare_parameter("freshness.planning_factor_seconds", 1.0))),
        stop_reason_fresh_ns_(seconds_to_ns(
            declare_parameter("freshness.stop_reason_seconds", 1.0))),
        diagnostic_error_after_ns_(seconds_to_ns(
            declare_parameter("diagnostics.error_after_seconds", 5.0))) {
    const auto velocity_topic =
        declare_parameter("topics.velocity", "/vehicle/status/velocity_status");
    const auto mode_topic = declare_parameter("topics.control_mode",
                                              "/vehicle/status/control_mode");
    const auto state_topic =
        declare_parameter("topics.autoware_state", "/byd/autoware/state");
    const auto localization_topic = declare_parameter(
        "topics.localization", "/localization/kinematic_state");
    const auto trajectory_topic =
        declare_parameter("topics.trajectory", "/planning/trajectory");
    const auto stop_reason_topic =
        declare_parameter("topics.stop_reasons",
                          "/planning/scenario_planning/status/stop_reasons");
    const auto pedestrian_topic = declare_parameter(
        "topics.pedestrian_stop_status", "/byd/pedestrian_safety_stop/status");

    event_pub_ = create_publisher<byd_vehicle_msgs::msg::EventTrigger>(
        "/system/event_trigger", rclcpp::QoS(10).reliable());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);

    velocity_sub_ =
        create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
            velocity_topic, rclcpp::SensorDataQoS(),
            [this](const auto msg) { store(velocity_, msg); });
    mode_sub_ =
        create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
            mode_topic, 10, [this](const auto msg) { store(mode_, msg); });
    state_sub_ = create_subscription<autoware_system_msgs::msg::AutowareState>(
        state_topic, 10,
        [this](const auto msg) { store(autoware_state_, msg); });
    localization_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        localization_topic, rclcpp::SensorDataQoS(),
        [this](const auto msg) { store(localization_, msg); });
    trajectory_sub_ =
        create_subscription<autoware_planning_msgs::msg::Trajectory>(
            trajectory_topic, 10,
            [this](const auto msg) { store(trajectory_, msg); });
    stop_reason_sub_ =
        create_subscription<tier4_planning_msgs::msg::StopReasonArray>(
            stop_reason_topic, 10,
            [this](const auto msg) { store(stop_reasons_, msg); });
    pedestrian_sub_ =
        create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
            pedestrian_topic, rclcpp::QoS(1).transient_local(),
            [this](const auto msg) { store(pedestrian_status_, msg); });

    const auto factor_topics = declare_parameter<std::vector<std::string>>(
        "topics.planning_factors", default_factor_topics());
    factor_states_.reserve(factor_topics.size());
    for (const auto &topic : factor_topics) {
      factor_states_.push_back(FactorState{topic, {}, nullptr});
    }
    for (auto &factor : factor_states_) {
      auto *state = &factor;
      factor.subscription = create_subscription<
          autoware_internal_planning_msgs::msg::PlanningFactorArray>(
          factor.topic, 10,
          [this, state](const auto msg) { store(state->message, msg); });
    }

    evaluation_timer_ = create_wall_timer(100ms, [this]() { evaluate(); });
  }

private:
  static int64_t seconds_to_ns(double seconds) {
    return static_cast<int64_t>(seconds * 1'000'000'000.0);
  }

  AbnormalStopParameters make_stop_parameters() {
    AbnormalStopParameters parameters;
    parameters.stop_speed_mps =
        declare_parameter("abnormal_stop.speed_threshold_mps", 0.1);
    parameters.rearm_speed_mps =
        declare_parameter("abnormal_stop.rearm_speed_mps", 0.3);
    parameters.goal_distance_m =
        declare_parameter("abnormal_stop.goal_distance_m", 2.0);
    parameters.hold_ns =
        seconds_to_ns(declare_parameter("abnormal_stop.hold_seconds", 5.0));
    parameters.rearm_ns =
        seconds_to_ns(declare_parameter("abnormal_stop.rearm_seconds", 2.0));
    parameters.planned_stop_grace_ns = seconds_to_ns(
        declare_parameter("abnormal_stop.planned_stop_grace_seconds", 2.0));
    return parameters;
  }

  ModeTransitionParameters make_mode_parameters() {
    ModeTransitionParameters parameters;
    parameters.manual_stable_ns = seconds_to_ns(
        declare_parameter("mode_transition.manual_stable_seconds", 0.5));
    parameters.transition_window_ns = seconds_to_ns(
        declare_parameter("mode_transition.transition_window_seconds", 2.0));
    return parameters;
  }

  static std::vector<std::string> default_factor_topics() {
    return {"/planning/planning_factors/traffic_light",
            "/planning/planning_factors/stop_line",
            "/planning/planning_factors/crosswalk",
            "/planning/planning_factors/intersection",
            "/planning/planning_factors/detection_area",
            "/planning/planning_factors/dynamic_obstacle_stop",
            "/planning/planning_factors/obstacle_stop",
            "/planning/planning_factors/obstacle_stop_planner",
            "/planning/planning_factors/surround_obstacle_checker",
            "/planning/planning_factors/virtual_traffic_light"};
  }

  template <typename Message>
  void store(TimedMessage<Message> &target,
             typename Message::ConstSharedPtr message) {
    target.value = std::move(message);
    target.received_ns = now().nanoseconds();
  }

  bool planned_stop_active(int64_t now_ns, std::string &reason) const {
    if (stop_reasons_.fresh(now_ns, stop_reason_fresh_ns_) &&
        !stop_reasons_.value->stop_reasons.empty()) {
      reason = stop_reasons_.value->stop_reasons.front().reason;
      return true;
    }
    if (pedestrian_status_.fresh(now_ns, factor_fresh_ns_)) {
      const auto state =
          std::find_if(pedestrian_status_.value->values.begin(),
                       pedestrian_status_.value->values.end(),
                       [](const auto &value) { return value.key == "state"; });
      if (state != pedestrian_status_.value->values.end() &&
          state->value == "STOP") {
        reason = "pedestrian_safety_stop";
        return true;
      }
    }
    for (const auto &factor_state : factor_states_) {
      if (!factor_state.message.fresh(now_ns, factor_fresh_ns_)) {
        continue;
      }
      for (const auto &factor : factor_state.message.value->factors) {
        if (factor.behavior ==
            autoware_internal_planning_msgs::msg::PlanningFactor::STOP) {
          reason = factor.module.empty() ? factor_state.topic : factor.module;
          return true;
        }
      }
    }
    reason.clear();
    return false;
  }

  double goal_distance() const {
    const auto &position = localization_.value->pose.pose.position;
    const auto &goal = trajectory_.value->points.back().pose.position;
    return std::hypot(goal.x - position.x, goal.y - position.y);
  }

  void evaluate() {
    const auto now_ns = now().nanoseconds();
    if (last_evaluation_ns_ && now_ns < *last_evaluation_ns_) {
      stop_detector_.reset();
      mode_detector_.reset();
      last_error_ = "ROS clock moved backwards; detector state reset";
    }
    last_evaluation_ns_ = now_ns;

    const bool mode_fresh = mode_.fresh(now_ns, mode_fresh_ns_);
    if (mode_fresh && mode_detector_.update(now_ns, mode_.value->mode)) {
      publish_mode_event(now_ns);
    }

    std::vector<std::string> missing;
    if (!velocity_.fresh(now_ns, velocity_fresh_ns_)) {
      missing.push_back("velocity");
    }
    if (!mode_fresh) {
      missing.push_back("control_mode");
    }
    if (!autoware_state_.fresh(now_ns, state_fresh_ns_)) {
      missing.push_back("autoware_state");
    }
    if (!localization_.fresh(now_ns, localization_fresh_ns_)) {
      missing.push_back("localization");
    }
    if (!trajectory_.fresh(now_ns, trajectory_fresh_ns_) ||
        (trajectory_.value && trajectory_.value->points.empty())) {
      missing.push_back("trajectory");
    }
    if (!stop_reasons_.fresh(now_ns, stop_reason_fresh_ns_)) {
      missing.push_back("stop_reasons");
    }

    std::string stop_reason;
    const bool planned_stop = planned_stop_active(now_ns, stop_reason);
    const bool inputs_fresh = missing.empty();
    if (inputs_fresh) {
      const auto speed =
          static_cast<double>(velocity_.value->longitudinal_velocity);
      const auto distance = goal_distance();
      const bool autonomous =
          mode_.value->mode ==
          autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
      const bool arrived =
          autoware_state_.value->state ==
          autoware_system_msgs::msg::AutowareState::ARRIVED_GOAL;
      if (stop_detector_.update(AbnormalStopInput{now_ns, true, autonomous,
                                                  arrived, distance,
                                                  planned_stop, speed})) {
        publish_abnormal_stop_event(now_ns, speed, distance, stop_reason);
      }
    } else {
      stop_detector_.update(
          AbnormalStopInput{now_ns, false, false, false, 0.0, false, 0.0});
    }
    publish_diagnostics(now_ns, missing);
  }

  std::string next_event_id(int64_t now_ns) {
    std::ostringstream stream;
    stream << std::hex << now_ns << "-" << ++event_sequence_;
    return stream.str();
  }

  void publish_event(int64_t now_ns, const std::string &type, uint8_t severity,
                     const std::string &description) {
    byd_vehicle_msgs::msg::EventTrigger event;
    event.header.stamp = rclcpp::Time(now_ns).to_msg();
    event.event_id = next_event_id(now_ns);
    event.event_type = type;
    event.severity = severity;
    event.description = description;
    event_pub_->publish(event);
  }

  void publish_abnormal_stop_event(int64_t now_ns, double speed,
                                   double distance,
                                   const std::string &stop_reason) {
    std::ostringstream description;
    description << std::fixed << std::setprecision(3) << "speed_mps=" << speed
                << ", goal_distance_m=" << distance
                << ", control_mode=" << static_cast<unsigned>(mode_.value->mode)
                << ", autoware_state="
                << static_cast<unsigned>(autoware_state_.value->state)
                << ", planned_stop_reason="
                << (stop_reason.empty() ? "none" : stop_reason)
                << ", velocity_age_s="
                << static_cast<double>(now_ns - velocity_.received_ns) / 1e9
                << ", trajectory_age_s="
                << static_cast<double>(now_ns - trajectory_.received_ns) / 1e9
                << ", stop_reason_age_s="
                << static_cast<double>(now_ns - stop_reasons_.received_ns) /
                       1e9;
    publish_event(now_ns, "abnormal_stop",
                  byd_vehicle_msgs::msg::EventTrigger::SEVERITY_ERROR,
                  description.str());
  }

  void publish_mode_event(int64_t now_ns) {
    std::ostringstream description;
    description << "mode_sequence=";
    const auto &sequence = mode_detector_.transition_sequence();
    for (std::size_t index = 0; index < sequence.size(); ++index) {
      if (index != 0) {
        description << "->";
      }
      description << static_cast<unsigned>(sequence[index]);
    }
    if (velocity_.fresh(now_ns, velocity_fresh_ns_)) {
      description << ", speed_mps=" << velocity_.value->longitudinal_velocity;
    }
    if (autoware_state_.fresh(now_ns, state_fresh_ns_)) {
      description << ", autoware_state="
                  << static_cast<unsigned>(autoware_state_.value->state);
    }
    publish_event(now_ns, "autonomous_to_manual",
                  byd_vehicle_msgs::msg::EventTrigger::SEVERITY_WARN,
                  description.str());
  }

  void publish_diagnostics(int64_t now_ns,
                           const std::vector<std::string> &missing) {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = rclcpp::Time(now_ns).to_msg();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "system_event_monitor";
    status.hardware_id = "byd_system_events";
    status.level = missing.empty()
                       ? diagnostic_msgs::msg::DiagnosticStatus::OK
                       : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message =
        missing.empty() ? "monitoring" : "abnormal-stop detection suspended";
    if (missing.empty()) {
      inputs_missing_since_ns_.reset();
    } else {
      if (!inputs_missing_since_ns_) {
        inputs_missing_since_ns_ = now_ns;
      }
      if (now_ns - *inputs_missing_since_ns_ >= diagnostic_error_after_ns_) {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = "required inputs remain missing or stale";
      }
    }
    diagnostic_msgs::msg::KeyValue missing_value;
    missing_value.key = "missing_or_stale_inputs";
    for (std::size_t index = 0; index < missing.size(); ++index) {
      if (index != 0) {
        missing_value.value += ",";
      }
      missing_value.value += missing[index];
    }
    status.values.push_back(missing_value);
    if (!last_error_.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = last_error_;
      last_error_.clear();
    }
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  AbnormalStopDetector stop_detector_;
  ModeTransitionDetector mode_detector_;
  const int64_t velocity_fresh_ns_;
  const int64_t mode_fresh_ns_;
  const int64_t state_fresh_ns_;
  const int64_t localization_fresh_ns_;
  const int64_t trajectory_fresh_ns_;
  const int64_t factor_fresh_ns_;
  const int64_t stop_reason_fresh_ns_;
  const int64_t diagnostic_error_after_ns_;

  TimedMessage<autoware_vehicle_msgs::msg::VelocityReport> velocity_;
  TimedMessage<autoware_vehicle_msgs::msg::ControlModeReport> mode_;
  TimedMessage<autoware_system_msgs::msg::AutowareState> autoware_state_;
  TimedMessage<nav_msgs::msg::Odometry> localization_;
  TimedMessage<autoware_planning_msgs::msg::Trajectory> trajectory_;
  TimedMessage<tier4_planning_msgs::msg::StopReasonArray> stop_reasons_;
  TimedMessage<diagnostic_msgs::msg::DiagnosticStatus> pedestrian_status_;
  std::vector<FactorState> factor_states_;
  std::optional<int64_t> last_evaluation_ns_;
  std::optional<int64_t> inputs_missing_since_ns_;
  uint64_t event_sequence_{0};
  std::string last_error_;

  rclcpp::Publisher<byd_vehicle_msgs::msg::EventTrigger>::SharedPtr event_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
      velocity_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
      mode_sub_;
  rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr
      state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
      trajectory_sub_;
  rclcpp::Subscription<tier4_planning_msgs::msg::StopReasonArray>::SharedPtr
      stop_reason_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr
      pedestrian_sub_;
  rclcpp::TimerBase::SharedPtr evaluation_timer_;
};

} // namespace byd_system_event_monitor

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<byd_system_event_monitor::SystemEventMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
