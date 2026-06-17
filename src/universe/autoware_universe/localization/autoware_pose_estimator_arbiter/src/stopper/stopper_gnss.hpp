// Copyright 2023 Autoware Foundation
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

#ifndef STOPPER__STOPPER_GNSS_HPP_
#define STOPPER__STOPPER_GNSS_HPP_
#include "stopper/base_stopper.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <memory>

namespace autoware::pose_estimator_arbiter::stopper
{
class StopperGnss : public BaseStopper
{
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  explicit StopperGnss(
    rclcpp::Node * node, const std::shared_ptr<const SharedData> & shared_data)
  : BaseStopper(node, shared_data)
  {
    gnss_is_enabled_ = true;
    pub_pose_ = node->create_publisher<PoseCovStamped>("~/output/gnss/pose_with_covariance", 5);

    // Register callback
    shared_data_->gnss_output_pose_cov.register_callback(
      [this](PoseCovStamped::ConstSharedPtr msg) -> void {
        if (gnss_is_enabled_) {
          pub_pose_->publish(*msg);
        }
      });
  }

  void set_enable(bool enabled) override { gnss_is_enabled_ = enabled; }

private:
  bool gnss_is_enabled_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_;
};
}  // namespace autoware::pose_estimator_arbiter::stopper

#endif  // STOPPER__STOPPER_GNSS_HPP_
