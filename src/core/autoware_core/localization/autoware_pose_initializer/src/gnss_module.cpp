// Copyright 2022 The Autoware Contributors
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

#include "gnss_module.hpp"

#include <autoware/component_interface_specs/localization.hpp>

#include <autoware_adapi_v1_msgs/msg/response_status.hpp>

#include <cmath>
#include <memory>

namespace autoware::pose_initializer
{
GnssModule::GnssModule(rclcpp::Node * node)
: fitter_(node),
  clock_(node->get_clock()),
  logger_(node->get_logger()),
  timeout_(node->declare_parameter<double>("gnss_pose_timeout")),
  max_position_variance_(node->declare_parameter<double>("max_gnss_position_variance", 0.02)),
  max_fixed_frame_gap_sec_(node->declare_parameter<double>("max_fixed_frame_gap_sec", 0.5)),
  min_consecutive_fixed_count_(node->declare_parameter<int>("min_consecutive_fixed_count", 5))
{
  sub_gnss_pose_ = node->create_subscription<PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1, std::bind(&GnssModule::on_pose, this, std::placeholders::_1));
}

void GnssModule::on_pose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  pose_ = msg;
  const auto now = clock_->now();
  if (has_last_pose_received_time_ &&
      (now - last_pose_received_time_).seconds() > max_fixed_frame_gap_sec_) {
    consecutive_fixed_count_ = 0;
  }
  last_pose_received_time_ = now;
  has_last_pose_received_time_ = true;

  if (msg && std::isfinite(msg->pose.covariance[0]) &&
      std::isfinite(msg->pose.covariance[7]) &&
      msg->pose.covariance[0] >= 0.0 && msg->pose.covariance[7] >= 0.0 &&
      msg->pose.covariance[0] <= max_position_variance_ &&
      msg->pose.covariance[7] <= max_position_variance_) {
    consecutive_fixed_count_++;
  } else {
    consecutive_fixed_count_ = 0;
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped GnssModule::get_pose()
{
  using Initialize = autoware::component_interface_specs::localization::Initialize;

  if (!pose_) {
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = Initialize::Service::Response::ERROR_GNSS;
    respose_status.message = "The GNSS pose has not arrived.";
    throw respose_status;
  }

  const auto elapsed = clock_->now() - rclcpp::Time(pose_->header.stamp);
  if (elapsed.seconds() > timeout_) {
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = Initialize::Service::Response::ERROR_GNSS;
    respose_status.message = "The GNSS pose is out of date.";
    throw respose_status;
  }

  if (consecutive_fixed_count_ < min_consecutive_fixed_count_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "GNSS pose initialization rejected: consecutive fixed frames (%d) < required (%d). Waiting for stable signal.",
      consecutive_fixed_count_, min_consecutive_fixed_count_);
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = Initialize::Service::Response::ERROR_GNSS;
    respose_status.message = "The GNSS pose is not stable (waiting for consecutive fixed frames).";
    throw respose_status;
  }

  // Check position covariance:
  if (pose_->pose.covariance[0] > max_position_variance_ ||
      pose_->pose.covariance[7] > max_position_variance_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "GNSS pose initialization rejected: position variance (%.4f) > threshold (%.4f). RTK is not FIXED (status 4).",
      pose_->pose.covariance[0], max_position_variance_);
    autoware_adapi_v1_msgs::msg::ResponseStatus respose_status;
    respose_status.success = false;
    respose_status.code = Initialize::Service::Response::ERROR_GNSS;
    respose_status.message = "The GNSS pose is not RTK fixed (variance is too high).";
    throw respose_status;
  }


  PoseWithCovarianceStamped pose = *pose_;
  const auto fitted = fitter_.fit(pose.pose.pose.position, pose.header.frame_id);
  if (fitted) {
    pose.pose.pose.position = fitted.value();
  }
  return pose;
}
}  // namespace autoware::pose_initializer
