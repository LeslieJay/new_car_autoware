// Copyright 2025 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "autoware/behavior_path_simple_avoidance_module/trailer_geometry.hpp"

#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace autoware::behavior_path_planner
{
namespace
{
double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Pose makePose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

geometry_msgs::msg::Point offsetPoint(
  const geometry_msgs::msg::Point & origin, const double yaw, const double longitudinal)
{
  auto point = origin;
  point.x += longitudinal * std::cos(yaw);
  point.y += longitudinal * std::sin(yaw);
  return point;
}
}  // namespace

bool isValidTrailerGeometry(const TrailerGeometry & geometry)
{
  constexpr double half_pi = 1.5707963267948966;
  return geometry.width > 0.0 && geometry.axle_to_body_front > 0.0 &&
         geometry.axle_to_body_rear > 0.0 && geometry.front_hitch_to_axle > 0.0 &&
         geometry.axle_to_rear_hitch >= 0.0 && geometry.max_articulation_angle_rad > 0.0 &&
         geometry.max_articulation_angle_rad <= half_pi;
}

bool TrailerConfigurationStore::update(
  const std::vector<std::string> & types,
  const std::unordered_map<std::string, TrailerGeometry> & configured_types)
{
  ResolvedTrailerConfiguration resolved;
  resolved.types = types;
  resolved.geometries.reserve(types.size());
  for (const auto & type : types) {
    const auto geometry = configured_types.find(type);
    if (geometry == configured_types.end() || !isValidTrailerGeometry(geometry->second)) {
      return false;
    }
    resolved.geometries.push_back(geometry->second);
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::move(resolved);
  return true;
}

ResolvedTrailerConfiguration TrailerConfigurationStore::snapshot() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

ArticulatedPath predictArticulatedPath(
  const std::vector<geometry_msgs::msg::Pose> & tractor_path,
  const std::vector<TrailerGeometry> & trailers, const double tractor_rear_axle_to_hitch)
{
  ArticulatedPath output;
  if (tractor_path.empty()) {
    return output;
  }

  output.poses.reserve(tractor_path.size());
  std::vector<geometry_msgs::msg::Pose> previous_trailer_poses;

  for (size_t path_index = 0; path_index < tractor_path.size(); ++path_index) {
    const auto & tractor_pose = tractor_path.at(path_index);
    ArticulatedPose articulated_pose;
    articulated_pose.tractor = tractor_pose;
    articulated_pose.trailers.reserve(trailers.size());

    double parent_yaw = tf2::getYaw(tractor_pose.orientation);
    auto upstream_hitch =
      offsetPoint(tractor_pose.position, parent_yaw, -tractor_rear_axle_to_hitch);

    for (size_t trailer_index = 0; trailer_index < trailers.size(); ++trailer_index) {
      const auto & geometry = trailers.at(trailer_index);
      double trailer_yaw = parent_yaw;
      if (!previous_trailer_poses.empty()) {
        const auto & previous_axle = previous_trailer_poses.at(trailer_index).position;
        const double dx = upstream_hitch.x - previous_axle.x;
        const double dy = upstream_hitch.y - previous_axle.y;
        if (std::hypot(dx, dy) > 1e-6) {
          trailer_yaw = std::atan2(dy, dx);
        } else {
          trailer_yaw = tf2::getYaw(previous_trailer_poses.at(trailer_index).orientation);
        }
      }

      const auto axle = offsetPoint(upstream_hitch, trailer_yaw, -geometry.front_hitch_to_axle);
      auto trailer_pose = makePose(axle.x, axle.y, trailer_yaw);
      const double articulation = std::abs(normalizeAngle(parent_yaw - trailer_yaw));
      articulated_pose.max_articulation_angle_rad =
        std::max(articulated_pose.max_articulation_angle_rad, articulation);
      articulated_pose.trailers.push_back(trailer_pose);

      if (articulation > geometry.max_articulation_angle_rad && output.articulation_valid) {
        output.articulation_valid = false;
        output.invalid_path_index = path_index;
        output.invalid_trailer_index = trailer_index;
      }

      upstream_hitch = offsetPoint(axle, trailer_yaw, -geometry.axle_to_rear_hitch);
      parent_yaw = trailer_yaw;
    }

    previous_trailer_poses = articulated_pose.trailers;
    output.poses.push_back(std::move(articulated_pose));
  }

  return output;
}

autoware_utils::Polygon2d createTrailerFootprint(
  const geometry_msgs::msg::Pose & axle_pose, const TrailerGeometry & geometry)
{
  const double yaw = tf2::getYaw(axle_pose.orientation);
  const double half_width = geometry.width * 0.5;
  const std::array<std::pair<double, double>, 4> local_corners{{
    {geometry.axle_to_body_front, half_width},
    {geometry.axle_to_body_front, -half_width},
    {-geometry.axle_to_body_rear, -half_width},
    {-geometry.axle_to_body_rear, half_width},
  }};

  autoware_utils::Polygon2d footprint;
  for (const auto & [longitudinal, lateral] : local_corners) {
    const double x = axle_pose.position.x + longitudinal * std::cos(yaw) - lateral * std::sin(yaw);
    const double y = axle_pose.position.y + longitudinal * std::sin(yaw) + lateral * std::cos(yaw);
    footprint.outer().emplace_back(x, y);
  }
  boost::geometry::correct(footprint);
  return footprint;
}

}  // namespace autoware::behavior_path_planner
