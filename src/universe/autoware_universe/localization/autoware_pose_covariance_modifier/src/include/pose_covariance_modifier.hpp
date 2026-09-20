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
#ifndef POSE_COVARIANCE_MODIFIER_HPP_
#define POSE_COVARIANCE_MODIFIER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cstddef>

namespace autoware::pose_covariance_modifier
{
class PoseCovarianceModifierNode : public rclcpp::Node
{
public:
  explicit PoseCovarianceModifierNode(const rclcpp::NodeOptions & node_options);

  enum class PoseSource {
    NDT = 0,
    GNSS_NDT = 1,
    GNSS = 2,
    AUTO_HYBRID = 3,
    TRANSITION_SMOOTHING = 4,
  };

private:
  // covariance matrix indexes
  const int X_POS_IDX_ = 0;
  const int Y_POS_IDX_ = 7;
  const int Z_POS_IDX_ = 14;
  const int YAW_POS_IDX_ = 35;

  // parameters
  double threshold_gnss_stddev_yaw_deg_max_;
  double threshold_gnss_stddev_z_max_;
  double threshold_gnss_stddev_xy_bound_lower_;
  double threshold_gnss_stddev_xy_bound_upper_;
  double ndt_std_dev_bound_lower_;
  double ndt_std_dev_bound_upper_;
  double gnss_pose_timeout_sec_;
  int switch_to_ndt_count_threshold_;
  int switch_to_gnss_count_threshold_;
  double gnss_ndt_position_difference_threshold_;
  double gnss_ndt_yaw_difference_threshold_deg_;
  bool use_ndt_orientation_with_gnss_position_;
  bool debug_mode_;

  // Auto attitude alignment parameters
  bool enable_auto_attitude_switch_;
  double min_align_velocity_mps_;
  double max_align_yaw_diff_deg_;
  double max_align_pos_diff_m_;
  double required_stable_duration_sec_;
  double max_angular_velocity_radps_;
  double smooth_transition_duration_sec_;
  double fast_fallback_yaw_diff_deg_;

  // State variables
  rclcpp::Time gnss_pose_received_time_last_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr gnss_pose_with_cov_last_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr ndt_pose_with_cov_last_;
  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr current_twist_;

  PoseSource pose_source_;
  PoseSource candidate_pose_source_;
  std::size_t candidate_pose_source_count_;

  // Attitude convergence & transition tracking
  bool attitude_converged_{false};
  rclcpp::Time attitude_stable_start_time_{0, 0, RCL_ROS_TIME};
  bool is_attitude_stable_{false};

  bool in_transition_{false};
  rclcpp::Time transition_start_time_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Quaternion transition_start_orientation_;

  std::size_t fallback_anomaly_count_{0};

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_gnss_pose_with_cov_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    sub_ndt_pose_with_cov_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_pose_with_covariance_stamped_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_str_pose_source_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_double_ndt_position_stddev_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_double_gnss_position_stddev_;

  void callback_gnss_pose_with_cov(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg_pose_with_cov_in);

  void callback_ndt_pose_with_cov(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & msg_pose_with_cov_in);

  void callback_twist_with_cov(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & msg_twist_in);

  bool gnss_pose_has_timed_out(const rclcpp::Time & gnss_pose_received_time_last);

  PoseSource pose_source_from_gnss_stddev(
    double gnss_pose_yaw_stddev_deg, double gnss_pose_stddev_z, double gnss_pose_stddev_xy) const;

  void update_pose_source(PoseSource requested_pose_source);

  bool gnss_and_ndt_are_consistent(
    const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
    const geometry_msgs::msg::PoseWithCovarianceStamped & ndt_pose, double & position_difference,
    double & yaw_difference_deg) const;

  bool check_attitude_convergence();

  std::array<double, 36> update_ndt_covariances_from_gnss(
    const std::array<double, 36> & ndt_covariance_in);

  geometry_msgs::msg::PoseWithCovarianceStamped make_gnss_position_ndt_orientation_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
    const geometry_msgs::msg::PoseWithCovarianceStamped & ndt_pose) const;

  geometry_msgs::msg::PoseWithCovarianceStamped make_interpolated_orientation_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
    const geometry_msgs::msg::PoseWithCovarianceStamped & ndt_pose, double ratio) const;

  void publish_pose_type(const PoseSource & pose_source);
};

}  // namespace autoware::pose_covariance_modifier

#endif  // POSE_COVARIANCE_MODIFIER_HPP_
