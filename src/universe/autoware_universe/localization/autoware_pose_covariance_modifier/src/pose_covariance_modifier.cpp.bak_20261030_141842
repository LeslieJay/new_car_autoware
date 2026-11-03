// Copyright 2024 The Autoware Foundation
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

#include "include/pose_covariance_modifier.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace autoware::pose_covariance_modifier
{
using PoseSource = PoseCovarianceModifierNode::PoseSource;

PoseCovarianceModifierNode::PoseCovarianceModifierNode(const rclcpp::NodeOptions & node_options)
: Node("PoseCovarianceModifierNode", node_options),
  gnss_pose_received_time_last_(this->now()),
  pose_source_(PoseSource::NDT),
  candidate_pose_source_(PoseSource::NDT),
  candidate_pose_source_count_(0)
{
  // parameters
  threshold_gnss_stddev_yaw_deg_max_ =
    this->declare_parameter<double>("threshold_gnss_stddev_yaw_deg_max", 10.0);
  threshold_gnss_stddev_z_max_ = this->declare_parameter<double>("threshold_gnss_stddev_z_max", 0.5);
  threshold_gnss_stddev_xy_bound_lower_ =
    this->declare_parameter<double>("threshold_gnss_stddev_xy_bound_lower", 0.1);
  threshold_gnss_stddev_xy_bound_upper_ =
    this->declare_parameter<double>("threshold_gnss_stddev_xy_bound_upper", 0.25);
  ndt_std_dev_bound_lower_ = this->declare_parameter<double>("ndt_std_dev_bound_lower", 0.14);
  ndt_std_dev_bound_upper_ = this->declare_parameter<double>("ndt_std_dev_bound_upper", 0.30);
  gnss_pose_timeout_sec_ = this->declare_parameter<double>("gnss_pose_timeout_sec", 1.0);
  switch_to_ndt_count_threshold_ = this->declare_parameter<int>("switch_to_ndt_count_threshold", 5);
  switch_to_gnss_count_threshold_ = this->declare_parameter<int>("switch_to_gnss_count_threshold", 20);
  gnss_ndt_position_difference_threshold_ =
    this->declare_parameter<double>("gnss_ndt_position_difference_threshold", 0.3);
  gnss_ndt_yaw_difference_threshold_deg_ =
    this->declare_parameter<double>("gnss_ndt_yaw_difference_threshold_deg", 3.0);
  use_ndt_orientation_with_gnss_position_ =
    this->declare_parameter<bool>("use_ndt_orientation_with_gnss_position", false);
  debug_mode_ = this->declare_parameter<bool>("enable_debug_topics", true);

  // Auto attitude alignment parameters
  enable_auto_attitude_switch_ =
    this->declare_parameter<bool>("attitude_alignment.enable_auto_attitude_switch", true);
  min_align_velocity_mps_ =
    this->declare_parameter<double>("attitude_alignment.min_align_velocity_mps", 0.8);
  max_align_yaw_diff_deg_ =
    this->declare_parameter<double>("attitude_alignment.max_align_yaw_diff_deg", 2.5);
  max_align_pos_diff_m_ =
    this->declare_parameter<double>("attitude_alignment.max_align_pos_diff_m", 0.8);
  required_stable_duration_sec_ =
    this->declare_parameter<double>("attitude_alignment.required_stable_duration_sec", 1.5);
  max_angular_velocity_radps_ =
    this->declare_parameter<double>("attitude_alignment.max_angular_velocity_radps", 0.05);
  smooth_transition_duration_sec_ =
    this->declare_parameter<double>("attitude_alignment.smooth_transition_duration_sec", 1.0);
  fast_fallback_yaw_diff_deg_ =
    this->declare_parameter<double>("attitude_alignment.fast_fallback_yaw_diff_deg", 5.0);

  if (switch_to_ndt_count_threshold_ < 1 || switch_to_gnss_count_threshold_ < 1) {
    throw std::invalid_argument("Pose source switch count thresholds must be at least 1");
  }
  if (
    gnss_ndt_position_difference_threshold_ < 0.0 || gnss_ndt_yaw_difference_threshold_deg_ < 0.0) {
    throw std::invalid_argument("GNSS/NDT consistency thresholds must not be negative");
  }

  // subscribers
  sub_gnss_pose_with_cov_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "input_gnss_pose_with_cov_topic", 10,
      std::bind(
        &PoseCovarianceModifierNode::callback_gnss_pose_with_cov, this, std::placeholders::_1));

  sub_ndt_pose_with_cov_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "input_ndt_pose_with_cov_topic", 10,
    std::bind(
      &PoseCovarianceModifierNode::callback_ndt_pose_with_cov, this, std::placeholders::_1));

  sub_twist_with_cov_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input_twist_with_cov_topic", 10,
    std::bind(
      &PoseCovarianceModifierNode::callback_twist_with_cov, this, std::placeholders::_1));

  // publishers
  pub_pose_with_covariance_stamped_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "output_pose_with_covariance_topic", 10);

  pub_str_pose_source_ = this->create_publisher<std_msgs::msg::String>("~/selected_pose_type", 10);

  if (debug_mode_) {
    pub_double_ndt_position_stddev_ =
      this->create_publisher<std_msgs::msg::Float64>("~/debug/ndt_position_stddev", 10);
    pub_double_gnss_position_stddev_ =
      this->create_publisher<std_msgs::msg::Float64>("~/debug/gnss_position_stddev", 10);
  }
}

void PoseCovarianceModifierNode::callback_twist_with_cov(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & msg_twist_in)
{
  current_twist_ = msg_twist_in;
}

void PoseCovarianceModifierNode::callback_gnss_pose_with_cov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg_pose_with_cov_in)
{
  if (msg_pose_with_cov_in->header.stamp.sec <= 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received GNSS pose with invalid timestamp (sec <= 0: %d); ignoring and forcing fallback to NDT",
      msg_pose_with_cov_in->header.stamp.sec);
    if (pose_source_ != PoseSource::NDT) {
      pose_source_ = PoseSource::NDT;
      candidate_pose_source_ = PoseSource::NDT;
      candidate_pose_source_count_ = 0;
      publish_pose_type(pose_source_);
    }
    return;
  }

  // will be used to check if GNSS pose has timed out in the NDT pose callback
  gnss_pose_received_time_last_ = this->now();

  // if the pose source is not GNSS, it will be used to calculate the NDT covariance in the NDT pose
  // callback
  gnss_pose_with_cov_last_ = msg_pose_with_cov_in;

  const double gnss_pose_yaw_stddev_deg =
    std::sqrt(msg_pose_with_cov_in->pose.covariance[YAW_POS_IDX_]) * 180 / M_PI;

  const double gnss_pose_stddev_z = std::sqrt(msg_pose_with_cov_in->pose.covariance[Z_POS_IDX_]);

  const double gnss_pose_stddev_xy =
    (std::sqrt(msg_pose_with_cov_in->pose.covariance[X_POS_IDX_]) +
     std::sqrt(msg_pose_with_cov_in->pose.covariance[Y_POS_IDX_])) /
    2;

  if (debug_mode_) {
    std_msgs::msg::Float64 msg_double;
    msg_double.data = gnss_pose_stddev_xy;
    pub_double_gnss_position_stddev_->publish(msg_double);
  }

  // When auto attitude switch is enabled, publishing is synchronized from the NDT callback
  // for hybrid, transition, and fallback states.
  if (enable_auto_attitude_switch_) {
    if (pose_source_ == PoseSource::GNSS && !in_transition_) {
      // In steady full RTK mode, publish at GNSS rate as well
      pub_pose_with_covariance_stamped_->publish(*msg_pose_with_cov_in);
      publish_pose_type(pose_source_);
    }
    return;
  }

  // Legacy manual mode handling
  const auto requested_pose_source =
    pose_source_from_gnss_stddev(gnss_pose_yaw_stddev_deg, gnss_pose_stddev_z, gnss_pose_stddev_xy);
  update_pose_source(requested_pose_source);
  publish_pose_type(pose_source_);

  if (use_ndt_orientation_with_gnss_position_ && pose_source_ != PoseSource::NDT) {
    return;
  }

  if (pose_source_ == PoseSource::NDT) {
    return;
  }

  pub_pose_with_covariance_stamped_->publish(*msg_pose_with_cov_in);
}

void PoseCovarianceModifierNode::callback_ndt_pose_with_cov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg_pose_with_cov_in)
{
  ndt_pose_with_cov_last_ = msg_pose_with_cov_in;

  const bool gnss_timed_out = gnss_pose_has_timed_out(gnss_pose_received_time_last_);
  const bool gnss_stamp_invalid =
    !gnss_pose_with_cov_last_ || (gnss_pose_with_cov_last_->header.stamp.sec <= 0);

  // =========================================================================
  // Mode A: Auto Attitude Switch & Lifecycle State Machine (New Proposal A)
  // =========================================================================
  if (enable_auto_attitude_switch_) {
    double gnss_pos_stddev_xy = 999.0;
    if (gnss_pose_with_cov_last_) {
      gnss_pos_stddev_xy =
        (std::sqrt(gnss_pose_with_cov_last_->pose.covariance[X_POS_IDX_]) +
         std::sqrt(gnss_pose_with_cov_last_->pose.covariance[Y_POS_IDX_])) /
        2.0;
    }
    const bool gnss_pos_usable = (gnss_pos_stddev_xy <= threshold_gnss_stddev_xy_bound_upper_);

    // 1. If GNSS is timed out, stamp is invalid, or position is inaccurate, fallback to NDT
    if (gnss_timed_out || gnss_stamp_invalid || !gnss_pos_usable) {
      if (pose_source_ != PoseSource::NDT) {
        RCLCPP_WARN(
          get_logger(), "GNSS unusable (timed_out=%d, invalid_stamp=%d, stddev_xy=%.3f); fallback to NDT",
          gnss_timed_out, gnss_stamp_invalid, gnss_pos_stddev_xy);
      }
      pose_source_ = PoseSource::NDT;
      attitude_converged_ = false;
      in_transition_ = false;
      publish_pose_type(pose_source_);
      pub_pose_with_covariance_stamped_->publish(*msg_pose_with_cov_in);
      return;
    }

    // 2. GNSS position is usable. Check attitude convergence if not yet converged.
    if (!attitude_converged_) {
      if (check_attitude_convergence()) {
        attitude_converged_ = true;
        in_transition_ = true;
        transition_start_time_ = this->now();
        transition_start_orientation_ = msg_pose_with_cov_in->pose.pose.orientation;
        pose_source_ = PoseSource::TRANSITION_SMOOTHING;
        publish_pose_type(pose_source_);
        RCLCPP_INFO(
          get_logger(),
          "RTK dual-antenna attitude converged! Starting %.1fs smooth SLERP transition to RTK attitude.",
          smooth_transition_duration_sec_);
      } else {
        // Stage 1 & 2: AUTO_HYBRID (RTK Position + NDT 360° Orientation)
        pose_source_ = PoseSource::AUTO_HYBRID;
        publish_pose_type(pose_source_);
        const auto hybrid_pose =
          make_gnss_position_ndt_orientation_pose(*gnss_pose_with_cov_last_, *msg_pose_with_cov_in);
        pub_pose_with_covariance_stamped_->publish(hybrid_pose);
        return;
      }
    }

    // 3. Attitude converged: handle transition or steady Full RTK
    if (attitude_converged_) {
      if (in_transition_) {
        const double elapsed = (this->now() - transition_start_time_).seconds();
        const double ratio = std::clamp(elapsed / std::max(smooth_transition_duration_sec_, 0.001), 0.0, 1.0);

        if (ratio >= 1.0) {
          in_transition_ = false;
          pose_source_ = PoseSource::GNSS;
          publish_pose_type(pose_source_);
          RCLCPP_INFO(get_logger(), "Smooth transition complete. Now running on FULL RTK pose.");
          pub_pose_with_covariance_stamped_->publish(*gnss_pose_with_cov_last_);
          return;
        }

        pose_source_ = PoseSource::TRANSITION_SMOOTHING;
        publish_pose_type(pose_source_);
        const auto interp_pose =
          make_interpolated_orientation_pose(*gnss_pose_with_cov_last_, *msg_pose_with_cov_in, ratio);
        pub_pose_with_covariance_stamped_->publish(interp_pose);
        return;
      }

      // Stage 3: Steady FULL_RTK. Perform shadow health monitoring against NDT.
      if (pose_source_ == PoseSource::GNSS) {
        double pos_diff = 0.0;
        double yaw_diff_deg = 0.0;
        gnss_and_ndt_are_consistent(
          *gnss_pose_with_cov_last_, *msg_pose_with_cov_in, pos_diff, yaw_diff_deg);

        // Fast fallback check: if RTK attitude jumps or deviates significantly from NDT
        if (yaw_diff_deg > fast_fallback_yaw_diff_deg_ ||
            pos_diff > gnss_ndt_position_difference_threshold_ * 2.5) {
          fallback_anomaly_count_++;
          if (fallback_anomaly_count_ >= 2) {
            RCLCPP_WARN(
              get_logger(),
              "Fast fallback: RTK deviated from shadow NDT (yaw_diff=%.2f deg, pos_diff=%.2f m). "
              "Falling back to AUTO_HYBRID.",
              yaw_diff_deg, pos_diff);
            attitude_converged_ = false;
            in_transition_ = false;
            is_attitude_stable_ = false;
            fallback_anomaly_count_ = 0;
            pose_source_ = PoseSource::AUTO_HYBRID;
            publish_pose_type(pose_source_);
            const auto hybrid_pose =
              make_gnss_position_ndt_orientation_pose(*gnss_pose_with_cov_last_, *msg_pose_with_cov_in);
            pub_pose_with_covariance_stamped_->publish(hybrid_pose);
            return;
          }
        } else {
          fallback_anomaly_count_ = 0;
        }

        // In FULL_RTK, output is handled via GNSS or forwarded here
        return;
      }
    }
  }

  // =========================================================================
  // Mode B: Legacy Manual Handling (when enable_auto_attitude_switch == false)
  // =========================================================================
  if ((gnss_timed_out || gnss_stamp_invalid) && pose_source_ != PoseSource::NDT) {
    RCLCPP_WARN(
      get_logger(), "GNSS pose %s; switching immediately from GNSS to NDT",
      gnss_timed_out ? "timed out" : "timestamp invalid");
    pose_source_ = PoseSource::NDT;
    candidate_pose_source_ = PoseSource::NDT;
    candidate_pose_source_count_ = 0;
    publish_pose_type(pose_source_);
  }

  if (
    use_ndt_orientation_with_gnss_position_ && pose_source_ != PoseSource::NDT &&
    gnss_pose_with_cov_last_ && !gnss_stamp_invalid && !gnss_timed_out) {
    const auto hybrid_pose =
      make_gnss_position_ndt_orientation_pose(*gnss_pose_with_cov_last_, *msg_pose_with_cov_in);
    pub_pose_with_covariance_stamped_->publish(hybrid_pose);
    return;
  }

  if (pose_source_ == PoseSource::GNSS) {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped msg_pose_with_cov_out;
  if (gnss_timed_out || gnss_stamp_invalid || pose_source_ == PoseSource::NDT) {
    msg_pose_with_cov_out = *msg_pose_with_cov_in;
  } else if (pose_source_ == PoseSource::GNSS_NDT) {
    auto ndt_pose_with_cov_updated = *msg_pose_with_cov_in;
    ndt_pose_with_cov_updated.pose.covariance =
      update_ndt_covariances_from_gnss(msg_pose_with_cov_in->pose.covariance);
    msg_pose_with_cov_out = ndt_pose_with_cov_updated;
  }

  pub_pose_with_covariance_stamped_->publish(msg_pose_with_cov_out);

  if (debug_mode_) {
    std_msgs::msg::Float64 msg_double;
    msg_double.data = (std::sqrt(msg_pose_with_cov_out.pose.covariance[X_POS_IDX_]) +
                       std::sqrt(msg_pose_with_cov_out.pose.covariance[Y_POS_IDX_])) /
                      2.0;
    pub_double_ndt_position_stddev_->publish(msg_double);
  }
}

bool PoseCovarianceModifierNode::check_attitude_convergence()
{
  if (!gnss_pose_with_cov_last_ || !ndt_pose_with_cov_last_) {
    is_attitude_stable_ = false;
    return false;
  }

  // 1. Velocity and straight-motion checks
  if (!current_twist_) {
    is_attitude_stable_ = false;
    return false;
  }

  const double linear_v = current_twist_->twist.twist.linear.x;
  const double angular_wz = std::abs(current_twist_->twist.twist.angular.z);

  // Must be driving above minimum alignment velocity
  if (linear_v < min_align_velocity_mps_) {
    is_attitude_stable_ = false;
    return false;
  }

  // Must not be turning sharply (prevents attitude handover during curve maneuvers)
  if (angular_wz > max_angular_velocity_radps_) {
    is_attitude_stable_ = false;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[AttitudeAlign] Vehicle is turning (angular_wz: %.3f rad/s > %.3f rad/s), waiting for straight motion.",
      angular_wz, max_angular_velocity_radps_);
    return false;
  }

  // 2. Yaw difference between RTK orientation and NDT orientation
  const auto quaternion_to_yaw = [](const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };

  const double gnss_yaw = quaternion_to_yaw(gnss_pose_with_cov_last_->pose.pose.orientation);
  const double ndt_yaw = quaternion_to_yaw(ndt_pose_with_cov_last_->pose.pose.orientation);
  const double yaw_diff = std::abs(std::atan2(std::sin(gnss_yaw - ndt_yaw), std::cos(gnss_yaw - ndt_yaw)));
  const double yaw_diff_deg = yaw_diff * 180.0 / M_PI;

  if (yaw_diff_deg > max_align_yaw_diff_deg_) {
    is_attitude_stable_ = false;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[AttitudeAlign] Yaw diff (%.2f deg) > threshold (%.2f deg)", yaw_diff_deg, max_align_yaw_diff_deg_);
    return false;
  }

  // 3. Position consistency check
  const double dx = gnss_pose_with_cov_last_->pose.pose.position.x - ndt_pose_with_cov_last_->pose.pose.position.x;
  const double dy = gnss_pose_with_cov_last_->pose.pose.position.y - ndt_pose_with_cov_last_->pose.pose.position.y;
  const double pos_diff = std::hypot(dx, dy);

  if (pos_diff > max_align_pos_diff_m_) {
    is_attitude_stable_ = false;
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[AttitudeAlign] Position diff (%.2fm) > threshold (%.2fm)", pos_diff, max_align_pos_diff_m_);
    return false;
  }

  // 4. Stable duration evaluation
  const auto now = this->now();
  if (!is_attitude_stable_) {
    is_attitude_stable_ = true;
    attitude_stable_start_time_ = now;
    RCLCPP_INFO(
      get_logger(),
      "[AttitudeAlign] Straight motion verified (v=%.2fm/s, yaw_diff=%.2f deg, pos_diff=%.2fm)! Timing stabilization...",
      linear_v, yaw_diff_deg, pos_diff);
    return false;
  }

  const double stable_duration = (now - attitude_stable_start_time_).seconds();
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 500,
    "[AttitudeAlign] Stabilizing... (%.1f / %.1f s)", stable_duration, required_stable_duration_sec_);
  return stable_duration >= required_stable_duration_sec_;
}

bool PoseCovarianceModifierNode::gnss_pose_has_timed_out(
  const rclcpp::Time & gnss_pose_received_time_last)
{
  auto duration = this->now() - gnss_pose_received_time_last;
  if (duration.seconds() > gnss_pose_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "GNSS pose has timed out");
    return true;
  }
  return false;
}

PoseSource PoseCovarianceModifierNode::pose_source_from_gnss_stddev(
  const double gnss_pose_yaw_stddev_deg, const double gnss_pose_stddev_z,
  const double gnss_pose_stddev_xy) const
{
  if (
    gnss_pose_yaw_stddev_deg > threshold_gnss_stddev_yaw_deg_max_ ||
    gnss_pose_stddev_z > threshold_gnss_stddev_z_max_) {
    return PoseSource::NDT;
  }
  if (gnss_pose_stddev_xy <= threshold_gnss_stddev_xy_bound_lower_) {
    return PoseSource::GNSS;
  }
  if (gnss_pose_stddev_xy <= threshold_gnss_stddev_xy_bound_upper_) {
    return PoseSource::GNSS_NDT;
  }
  return PoseSource::NDT;
}

void PoseCovarianceModifierNode::update_pose_source(const PoseSource requested_pose_source)
{
  if (requested_pose_source == pose_source_) {
    candidate_pose_source_ = requested_pose_source;
    candidate_pose_source_count_ = 0;
    return;
  }

  if (requested_pose_source != candidate_pose_source_) {
    candidate_pose_source_ = requested_pose_source;
    candidate_pose_source_count_ = 0;
  }
  ++candidate_pose_source_count_;

  const bool uses_more_gnss =
    static_cast<int>(requested_pose_source) < static_cast<int>(pose_source_);
  const auto count_threshold = static_cast<std::size_t>(
    uses_more_gnss ? switch_to_gnss_count_threshold_ : switch_to_ndt_count_threshold_);
  if (candidate_pose_source_count_ < count_threshold) {
    return;
  }

  if (uses_more_gnss && gnss_pose_with_cov_last_ && ndt_pose_with_cov_last_) {
    double position_difference = 0.0;
    double yaw_difference_deg = 0.0;
    if (!gnss_and_ndt_are_consistent(
          *gnss_pose_with_cov_last_, *ndt_pose_with_cov_last_, position_difference,
          yaw_difference_deg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "GNSS recovery rejected: GNSS/NDT difference is %.3f m and %.3f deg "
        "(limits: %.3f m, %.3f deg)",
        position_difference, yaw_difference_deg, gnss_ndt_position_difference_threshold_,
        gnss_ndt_yaw_difference_threshold_deg_);
      candidate_pose_source_count_ = 0;
      return;
    }
  }

  if (!uses_more_gnss && gnss_pose_with_cov_last_ && ndt_pose_with_cov_last_) {
    double position_difference = 0.0;
    double yaw_difference_deg = 0.0;
    if (!gnss_and_ndt_are_consistent(
          *gnss_pose_with_cov_last_, *ndt_pose_with_cov_last_, position_difference,
          yaw_difference_deg)) {
      RCLCPP_WARN(
        get_logger(),
        "Switching to a safer NDT source while GNSS/NDT differ by %.3f m and %.3f deg",
        position_difference, yaw_difference_deg);
    }
  }

  const auto previous_pose_source = pose_source_;
  pose_source_ = requested_pose_source;
  candidate_pose_source_count_ = 0;
  RCLCPP_INFO(
    get_logger(), "Pose source switched from %d to %d", static_cast<int>(previous_pose_source),
    static_cast<int>(pose_source_));
}

bool PoseCovarianceModifierNode::gnss_and_ndt_are_consistent(
  const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
  const geometry_msgs::msg::PoseWithCovarianceStamped & ndt_pose, double & position_difference,
  double & yaw_difference_deg) const
{
  if (gnss_pose.header.stamp.sec <= 0 || ndt_pose.header.stamp.sec <= 0) {
    return false;
  }

  const double time_diff_sec =
    std::abs((rclcpp::Time(gnss_pose.header.stamp) - rclcpp::Time(ndt_pose.header.stamp)).seconds());
  if (time_diff_sec > gnss_pose_timeout_sec_) {
    return false;
  }

  const double dx = gnss_pose.pose.pose.position.x - ndt_pose.pose.pose.position.x;
  const double dy = gnss_pose.pose.pose.position.y - ndt_pose.pose.pose.position.y;
  position_difference = std::hypot(dx, dy);

  const auto quaternion_to_yaw = [](const geometry_msgs::msg::Quaternion & q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  };
  const double gnss_yaw = quaternion_to_yaw(gnss_pose.pose.pose.orientation);
  const double ndt_yaw = quaternion_to_yaw(ndt_pose.pose.pose.orientation);
  const double yaw_difference =
    std::abs(std::atan2(std::sin(gnss_yaw - ndt_yaw), std::cos(gnss_yaw - ndt_yaw)));
  yaw_difference_deg = yaw_difference * 180.0 / M_PI;

  return position_difference <= gnss_ndt_position_difference_threshold_ &&
         yaw_difference_deg <= gnss_ndt_yaw_difference_threshold_deg_;
}

std::array<double, 36> PoseCovarianceModifierNode::update_ndt_covariances_from_gnss(
  const std::array<double, 36> & ndt_covariance_in)
{
  if (!gnss_pose_with_cov_last_ || gnss_pose_with_cov_last_->header.stamp.sec <= 0) {
    return ndt_covariance_in;
  }

  auto lerp_range_to_range = [](double x, double x_min, double x_max, double y_min, double y_max) {
    const double input_normalized = (x - x_min) / (x_max - x_min);
    return autoware::interpolation::lerp(y_min, y_max, input_normalized);
  };

  auto ndt_variance_from_gnss_variance = [&](double ndt_variance, double gnss_variance) {
    double ndt_stddev = std::sqrt(ndt_variance);
    if (ndt_stddev > ndt_std_dev_bound_upper_ || ndt_stddev < ndt_std_dev_bound_lower_) {
      RCLCPP_ERROR(
        get_logger(),
        "Input variance of NDT exceeds bound values. Variance values of NDT were not modified. "
        "Check your bound values for NDT stddev.");
      return ndt_variance;
    }
    const double gnss_std_dev = std::sqrt(gnss_variance);
    const double interpolated_std_dev = lerp_range_to_range(
      gnss_std_dev, threshold_gnss_stddev_xy_bound_lower_, threshold_gnss_stddev_xy_bound_upper_,
      ndt_std_dev_bound_lower_, ndt_std_dev_bound_upper_);

    const double reversed_std_dev =
      ndt_std_dev_bound_lower_ + ndt_std_dev_bound_upper_ - interpolated_std_dev;

    const double interpolated_variance = std::pow(reversed_std_dev, 2);
    return (std::max(interpolated_variance, std::pow(ndt_std_dev_bound_lower_, 2)));
  };

  std::array<double, 36> ndt_covariance = ndt_covariance_in;
  std::array<int, 3> indices = {X_POS_IDX_, Y_POS_IDX_, Z_POS_IDX_};
  for (int idx : indices) {
    ndt_covariance[idx] = ndt_variance_from_gnss_variance(
      ndt_covariance_in[idx], gnss_pose_with_cov_last_->pose.covariance[idx]);
  }

  return ndt_covariance;
}

geometry_msgs::msg::PoseWithCovarianceStamped
PoseCovarianceModifierNode::make_gnss_position_ndt_orientation_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
  const geometry_msgs::msg::PoseWithCovarianceStamped & ndt_pose) const
{
  auto hybrid_pose = gnss_pose;

  hybrid_pose.header = ndt_pose.header;
  hybrid_pose.pose.pose.orientation = ndt_pose.pose.pose.orientation;

  constexpr std::size_t kPoseDimensions = 6;
  constexpr std::size_t kOrientationStart = 3;
  for (std::size_t row = 0; row < kPoseDimensions; ++row) {
    for (std::size_t col = 0; col < kPoseDimensions; ++col) {
      const auto index = row * kPoseDimensions + col;
      const bool row_is_orientation = row >= kOrientationStart;
      const bool col_is_orientation = col >= kOrientationStart;
      if (row_is_orientation && col_is_orientation) {
        hybrid_pose.pose.covariance[index] = ndt_pose.pose.covariance[index];
      } else if (row_is_orientation != col_is_orientation) {
        hybrid_pose.pose.covariance[index] = 0.0;
      }
    }
  }

  return hybrid_pose;
}

geometry_msgs::msg::PoseWithCovarianceStamped
PoseCovarianceModifierNode::make_interpolated_orientation_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
  const geometry_msgs::msg::PoseWithCovarianceStamped & ndt_pose,
  const double ratio) const
{
  auto out_pose = gnss_pose;
  out_pose.header = ndt_pose.header;

  tf2::Quaternion q_start(
    transition_start_orientation_.x, transition_start_orientation_.y,
    transition_start_orientation_.z, transition_start_orientation_.w);
  tf2::Quaternion q_end(
    gnss_pose.pose.pose.orientation.x, gnss_pose.pose.pose.orientation.y,
    gnss_pose.pose.pose.orientation.z, gnss_pose.pose.pose.orientation.w);

  if (q_start.length2() > 1e-6 && q_end.length2() > 1e-6) {
    q_start.normalize();
    q_end.normalize();
    const tf2::Quaternion q_interp = q_start.slerp(q_end, ratio);
    out_pose.pose.pose.orientation = tf2::toMsg(q_interp);
  } else {
    out_pose.pose.pose.orientation = ndt_pose.pose.pose.orientation;
  }

  constexpr std::size_t kPoseDimensions = 6;
  constexpr std::size_t kOrientationStart = 3;
  for (std::size_t row = 0; row < kPoseDimensions; ++row) {
    for (std::size_t col = 0; col < kPoseDimensions; ++col) {
      const auto index = row * kPoseDimensions + col;
      const bool row_is_orientation = row >= kOrientationStart;
      const bool col_is_orientation = col >= kOrientationStart;
      if (row_is_orientation && col_is_orientation) {
        out_pose.pose.covariance[index] =
          (1.0 - ratio) * ndt_pose.pose.covariance[index] + ratio * gnss_pose.pose.covariance[index];
      } else if (row_is_orientation != col_is_orientation) {
        out_pose.pose.covariance[index] = 0.0;
      }
    }
  }

  return out_pose;
}

void PoseCovarianceModifierNode::publish_pose_type(const PoseSource & pose_source)
{
  std_msgs::msg::String selected_pose_type;
  switch (pose_source) {
    case PoseSource::GNSS:
      selected_pose_type.data = "GNSS";
      break;
    case PoseSource::GNSS_NDT:
      selected_pose_type.data = "GNSS_NDT";
      break;
    case PoseSource::NDT:
      selected_pose_type.data = "NDT";
      break;
    case PoseSource::AUTO_HYBRID:
      selected_pose_type.data = "AUTO_HYBRID";
      break;
    case PoseSource::TRANSITION_SMOOTHING:
      selected_pose_type.data = "TRANSITION_SMOOTHING";
      break;
    default:
      selected_pose_type.data = "NOT_DEFINED";
      break;
  }
  pub_str_pose_source_->publish(selected_pose_type);
}

}  // namespace autoware::pose_covariance_modifier

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pose_covariance_modifier::PoseCovarianceModifierNode)
