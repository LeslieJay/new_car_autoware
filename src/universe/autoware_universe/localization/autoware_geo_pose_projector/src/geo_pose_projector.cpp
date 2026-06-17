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

#include "geo_pose_projector.hpp"

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>

#include <memory>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <string>

namespace autoware::geo_pose_projector
{
GeoPoseProjector::GeoPoseProjector(const rclcpp::NodeOptions & options)
: rclcpp::Node("geo_pose_projector", options), publish_tf_(declare_parameter<bool>("publish_tf"))
{
  // Subscribe to map_projector_info topic
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(
    sub_map_projector_info_,
    [this](const MapProjectorInfo::Message::ConstSharedPtr msg) { projector_info_ = *msg; });

  // Subscribe to NavSatFix (RTK position with status)
  // Use BEST_EFFORT QoS to match sensor driver publishers
  const auto sensor_qos = rclcpp::QoS(10).best_effort();
  nav_sat_fix_sub_ = create_subscription<NavSatFix>(
    "input_nav_sat_fix", sensor_qos,
    [this](const NavSatFix::ConstSharedPtr msg) { on_nav_sat_fix(msg); });

  // Subscribe to GnssInsOrientationStamped (orientation from positioning device)
  gnss_ins_orientation_sub_ = create_subscription<GnssInsOrientation>(
    "input_gnss_ins_orientation", sensor_qos,
    [this](const GnssInsOrientation::ConstSharedPtr msg) {
      on_gnss_ins_orientation(msg);
    });

  // Publish pose topic
  pose_pub_ = create_publisher<PoseWithCovariance>("output_pose", 10);

  // Declare covariance parameters for different GNSS states
  position_variance_fixed_ = declare_parameter<double>("position_variance_fixed", 0.01);  // 1cm
  position_variance_float_ = declare_parameter<double>("position_variance_float", 0.05);  // 5cm
  position_variance_low_ = declare_parameter<double>("position_variance_low", 0.5);       // 50cm
  position_variance_no_fix_ = declare_parameter<double>("position_variance_no_fix", 10.0); // 10m
  orientation_variance_ = declare_parameter<double>("orientation_variance", 0.01);        // rad^2

  // Publish tf
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    parent_frame_ = declare_parameter<std::string>("parent_frame");
    child_frame_ = declare_parameter<std::string>("child_frame");
  }
}

void GeoPoseProjector::on_nav_sat_fix(const NavSatFix::ConstSharedPtr msg)
{
  latest_nav_sat_fix_ = *msg;
  try_publish_geo_pose();
}

void GeoPoseProjector::on_gnss_ins_orientation(const GnssInsOrientation::ConstSharedPtr msg)
{
  latest_orientation_ = *msg;
  try_publish_geo_pose();
}

void GeoPoseProjector::try_publish_geo_pose()
{
  // Wait for both inputs
  if (!latest_nav_sat_fix_.has_value() || !latest_orientation_.has_value()) {
    return;
  }

  if (!projector_info_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000 /* ms */, "map_projector_info is not received yet.");
    return;
  }

  const auto & nav_sat = latest_nav_sat_fix_.value();
  const auto & orientation = latest_orientation_.value();

  // Reject invalid coordinates (NaN/Inf can occur during RTK initialization)
  if (
    !std::isfinite(nav_sat.latitude) || !std::isfinite(nav_sat.longitude) ||
    !std::isfinite(nav_sat.altitude)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000 /* ms */,
      "NavSatFix contains NaN/Inf values (lat=%.6f lon=%.6f alt=%.2f), skipping.",
      nav_sat.latitude, nav_sat.longitude, nav_sat.altitude);
    return;
  }

  // Create GeoPose message internally
  geographic_msgs::msg::GeoPoint gps_point;
  gps_point.latitude = nav_sat.latitude;
  gps_point.longitude = nav_sat.longitude;
  gps_point.altitude = nav_sat.altitude;

  // Project to local coordinates
  geometry_msgs::msg::Point position =
    autoware::geography_utils::project_forward(gps_point, projector_info_.value());
  position.z = autoware::geography_utils::convert_height(
    position.z, gps_point.latitude, gps_point.longitude, MapProjectorInfo::Message::WGS84,
    projector_info_.value().vertical_datum);

  // Convert to PoseWithCovariance
  PoseWithCovariance projected_pose;
  
  // Use the newer timestamp
  const auto nav_sat_time = rclcpp::Time(nav_sat.header.stamp);
  const auto orientation_time = rclcpp::Time(orientation.header.stamp);
  const auto max_time = (nav_sat_time > orientation_time) ? nav_sat_time : orientation_time;
  
  projected_pose.header.stamp = max_time;
  projected_pose.header.frame_id = "map";  // Output is in map frame after projection
  projected_pose.pose.pose.position = position;

  // Extract orientation from GnssInsOrientationStamped
  // Check if quaternion is valid
  if (orientation.orientation.orientation.w != 0.0 || orientation.orientation.orientation.x != 0.0 ||
      orientation.orientation.orientation.y != 0.0 || orientation.orientation.orientation.z != 0.0) {
    projected_pose.pose.pose.orientation = orientation.orientation.orientation;
  } else {
    // If no valid quaternion, use identity
    projected_pose.pose.pose.orientation.x = 0.0;
    projected_pose.pose.pose.orientation.y = 0.0;
    projected_pose.pose.pose.orientation.z = 0.0;
    projected_pose.pose.pose.orientation.w = 1.0;
  }

  // Set covariance based on RTK status
  set_covariance_by_status(projected_pose, nav_sat.status.status);

  pose_pub_->publish(projected_pose);

  // Publish tf
  if (publish_tf_) {
    tf2::Transform transform;
    transform.setOrigin(
      tf2::Vector3(
        projected_pose.pose.pose.position.x, projected_pose.pose.pose.position.y,
        projected_pose.pose.pose.position.z));
    const auto localization_quat = tf2::Quaternion(
      projected_pose.pose.pose.orientation.x, projected_pose.pose.pose.orientation.y,
      projected_pose.pose.pose.orientation.z, projected_pose.pose.pose.orientation.w);
    transform.setRotation(localization_quat);

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header = projected_pose.header;
    transform_stamped.header.frame_id = parent_frame_;
    transform_stamped.child_frame_id = child_frame_;
    transform_stamped.transform = tf2::toMsg(transform);
    tf_broadcaster_->sendTransform(transform_stamped);
  }
}

void GeoPoseProjector::set_covariance_by_status(
  PoseWithCovariance & pose, const int8_t nav_sat_status)
{
  // Select variance based on GNSS status
  double position_variance = position_variance_no_fix_;

  // RTK box status definition:
  //   0 = invalid (no fix)
  //   1 = single point
  //   2 = DGNSS
  //   4 = RTK fixed solution  (high accuracy, CEP95 < 10cm)
  //   5 = RTK float solution  (medium accuracy)
  //   6 = DR dead reckoning   (IMU+wheel speed extrapolation, no GNSS correction)
  //
  // NOTE: These values do NOT match ROS2 NavSatStatus constants:
  //   STATUS_NO_FIX=-1, STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
  // We therefore match against raw integer values, not ROS2 named constants.
  switch (nav_sat_status) {
    case 4:  // RTK fixed solution - highest accuracy (CEP95 < 10cm)
      position_variance = position_variance_fixed_;
      break;
    case 5:  // RTK float solution - medium accuracy
      position_variance = position_variance_float_;
      break;
    case 2:  // DGNSS - low accuracy
      position_variance = position_variance_low_;
      break;
    case 6:  // DR dead reckoning - treat as low accuracy (no GNSS correction)
      position_variance = position_variance_low_;
      break;
    case 1:  // Single point - very low accuracy
    case 0:  // Invalid / no fix
    default:
      position_variance = position_variance_no_fix_;
      break;
  }

  // Set diagonal covariance matrix (variance for x, y, z)
  // Covariance matrix layout:
  // [x, y, z, roll, pitch, yaw] -> indices: 0, 7, 14, 21, 28, 35
  pose.pose.covariance[0] = position_variance;   // x variance
  pose.pose.covariance[7] = position_variance;   // y variance
  pose.pose.covariance[14] = position_variance;  // z variance

  // Set orientation variance
  pose.pose.covariance[21] = orientation_variance_;  // roll variance (rad^2)
  pose.pose.covariance[28] = orientation_variance_;  // pitch variance (rad^2)
  pose.pose.covariance[35] = orientation_variance_;  // yaw variance (rad^2)
}
}  // namespace autoware::geo_pose_projector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::geo_pose_projector::GeoPoseProjector)
