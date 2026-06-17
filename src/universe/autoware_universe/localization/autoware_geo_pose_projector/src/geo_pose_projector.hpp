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

#ifndef GEO_POSE_PROJECTOR_HPP_
#define GEO_POSE_PROJECTOR_HPP_

#include <autoware/component_interface_specs_universe/map.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geographic_msgs/msg/geo_pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <optional>
#include <string>

namespace autoware::geo_pose_projector
{
class GeoPoseProjector : public rclcpp::Node
{
private:
  using GeoPoseWithCovariance = geographic_msgs::msg::GeoPoseWithCovarianceStamped;
  using PoseWithCovariance = geometry_msgs::msg::PoseWithCovarianceStamped;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using GnssInsOrientation = autoware_sensing_msgs::msg::GnssInsOrientationStamped;
  using MapProjectorInfo = autoware::component_interface_specs_universe::map::MapProjectorInfo;

public:
  explicit GeoPoseProjector(const rclcpp::NodeOptions & options);

private:
  void on_nav_sat_fix(const NavSatFix::ConstSharedPtr msg);
  void on_gnss_ins_orientation(const GnssInsOrientation::ConstSharedPtr msg);
  void try_publish_geo_pose();
  void set_covariance_by_status(
    PoseWithCovariance & pose, const int8_t nav_sat_status);

  autoware::component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr
    sub_map_projector_info_;
  rclcpp::Subscription<NavSatFix>::SharedPtr nav_sat_fix_sub_;
  rclcpp::Subscription<GnssInsOrientation>::SharedPtr gnss_ins_orientation_sub_;
  rclcpp::Publisher<PoseWithCovariance>::SharedPtr pose_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::optional<MapProjectorInfo::Message> projector_info_ = std::nullopt;

  // Latest data buffers for fusion
  std::optional<NavSatFix> latest_nav_sat_fix_;
  std::optional<GnssInsOrientation> latest_orientation_;

  const bool publish_tf_;

  std::string parent_frame_;
  std::string child_frame_;

  // Covariance parameters for different GNSS states
  // Mapped to RTK box status: 4=fixed, 5=float, 2=DGNSS, 6=DR, 1=single, 0=invalid
  double position_variance_fixed_;    // status=4: RTK fixed solution
  double position_variance_float_;    // status=5: RTK float solution
  double position_variance_low_;      // status=2: DGNSS / status=6: DR dead reckoning
  double position_variance_no_fix_;   // status=0: invalid / status=1: single point
  double orientation_variance_;       // Orientation variance (rad^2)
};
}  // namespace autoware::geo_pose_projector

#endif  // GEO_POSE_PROJECTOR_HPP_
