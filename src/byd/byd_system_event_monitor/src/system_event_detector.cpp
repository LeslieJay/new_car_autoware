#include "byd_system_event_monitor/system_event_detector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace byd_system_event_monitor {

AbnormalStopDetector::AbnormalStopDetector(AbnormalStopParameters parameters)
    : parameters_(std::move(parameters)) {
  if (parameters_.stop_speed_mps < 0.0 ||
      parameters_.rearm_speed_mps <= parameters_.stop_speed_mps ||
      parameters_.goal_distance_m < 0.0 || parameters_.hold_ns <= 0 ||
      parameters_.rearm_ns <= 0 || parameters_.planned_stop_grace_ns < 0) {
    throw std::invalid_argument("invalid abnormal-stop detector parameters");
  }
}

bool AbnormalStopDetector::update(const AbnormalStopInput &input) {
  if (last_planned_stop_ && !input.planned_stop) {
    planned_stop_grace_until_ns_ =
        input.now_ns + parameters_.planned_stop_grace_ns;
  }
  last_planned_stop_ = input.planned_stop;

  if (!input.autonomous || input.arrived_goal || input.planned_stop) {
    armed_ = true;
    stopped_since_ns_.reset();
    moving_since_ns_.reset();
    return false;
  }
  if (!input.inputs_fresh ||
      input.goal_distance_m <= parameters_.goal_distance_m ||
      input.now_ns < planned_stop_grace_until_ns_) {
    stopped_since_ns_.reset();
    moving_since_ns_.reset();
    return false;
  }

  if (std::abs(input.speed_mps) > parameters_.rearm_speed_mps) {
    stopped_since_ns_.reset();
    if (!moving_since_ns_) {
      moving_since_ns_ = input.now_ns;
    }
    if (!armed_ && input.now_ns - *moving_since_ns_ >= parameters_.rearm_ns) {
      armed_ = true;
    }
    return false;
  }
  moving_since_ns_.reset();
  if (std::abs(input.speed_mps) > parameters_.stop_speed_mps) {
    stopped_since_ns_.reset();
    return false;
  }
  if (!stopped_since_ns_) {
    stopped_since_ns_ = input.now_ns;
    return false;
  }
  if (armed_ && input.now_ns - *stopped_since_ns_ >= parameters_.hold_ns) {
    armed_ = false;
    return true;
  }
  return false;
}

void AbnormalStopDetector::reset() {
  armed_ = true;
  last_planned_stop_ = false;
  planned_stop_grace_until_ns_ = 0;
  stopped_since_ns_.reset();
  moving_since_ns_.reset();
}

ModeTransitionDetector::ModeTransitionDetector(
    ModeTransitionParameters parameters)
    : parameters_(std::move(parameters)) {
  if (parameters_.manual_stable_ns <= 0 ||
      parameters_.transition_window_ns <= 0) {
    throw std::invalid_argument("invalid mode-transition detector parameters");
  }
}

bool ModeTransitionDetector::is_intermediate(uint8_t mode) const {
  return mode == parameters_.no_command ||
         mode == parameters_.autonomous_steer_only ||
         mode == parameters_.autonomous_velocity_only;
}

bool ModeTransitionDetector::update(int64_t now_ns, uint8_t mode) {
  if (mode == parameters_.autonomous) {
    observed_autonomous_ = true;
    emitted_ = false;
    last_autonomous_ns_ = now_ns;
    manual_since_ns_.reset();
    sequence_.clear();
    sequence_.push_back(mode);
    return false;
  }
  if (!observed_autonomous_ || !last_autonomous_ns_) {
    return false;
  }
  if (sequence_.empty() || sequence_.back() != mode) {
    sequence_.push_back(mode);
  }
  if (!manual_since_ns_ &&
      now_ns - *last_autonomous_ns_ > parameters_.transition_window_ns) {
    observed_autonomous_ = false;
    manual_since_ns_.reset();
    return false;
  }
  if (is_intermediate(mode)) {
    manual_since_ns_.reset();
    return false;
  }
  if (mode != parameters_.manual || emitted_) {
    observed_autonomous_ = false;
    manual_since_ns_.reset();
    return false;
  }
  if (!manual_since_ns_) {
    if (now_ns - *last_autonomous_ns_ > parameters_.transition_window_ns) {
      observed_autonomous_ = false;
      return false;
    }
    manual_since_ns_ = now_ns;
    return false;
  }
  if (now_ns - *manual_since_ns_ >= parameters_.manual_stable_ns) {
    emitted_ = true;
    observed_autonomous_ = false;
    return true;
  }
  return false;
}

void ModeTransitionDetector::reset() {
  observed_autonomous_ = false;
  emitted_ = false;
  last_autonomous_ns_.reset();
  manual_since_ns_.reset();
  sequence_.clear();
}

const std::vector<uint8_t> &
ModeTransitionDetector::transition_sequence() const {
  return sequence_;
}

} // namespace byd_system_event_monitor
