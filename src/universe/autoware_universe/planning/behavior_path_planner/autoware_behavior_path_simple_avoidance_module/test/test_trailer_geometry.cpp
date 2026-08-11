// Copyright 2025 BYD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "autoware/behavior_path_simple_avoidance_module/trailer_geometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
namespace
{
TrailerGeometry makeGeometry()
{
  TrailerGeometry geometry;
  geometry.type = "default";
  geometry.width = 1.2;
  geometry.axle_to_body_front = 1.0;
  geometry.axle_to_body_rear = 0.7;
  geometry.front_hitch_to_axle = 0.9;
  geometry.axle_to_rear_hitch = 0.6;
  geometry.max_articulation_angle_rad = M_PI_4;
  return geometry;
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
}  // namespace

TEST(TrailerGeometryTest, RejectsInvalidDimensions)
{
  auto geometry = makeGeometry();
  EXPECT_TRUE(isValidTrailerGeometry(geometry));
  geometry.width = 0.0;
  EXPECT_FALSE(isValidTrailerGeometry(geometry));
}

TEST(TrailerGeometryTest, ConfigurationStoreRetainsLastValidConfiguration)
{
  TrailerConfigurationStore store;
  const std::unordered_map<std::string, TrailerGeometry> configured{{"default", makeGeometry()}};

  ASSERT_TRUE(store.update({"default", "default"}, configured));
  EXPECT_FALSE(store.update({"unknown"}, configured));

  const auto snapshot = store.snapshot();
  ASSERT_EQ(snapshot.types.size(), 2U);
  EXPECT_EQ(snapshot.types.front(), "default");
}

TEST(TrailerGeometryTest, KeepsMultipleTrailersAlignedOnStraightPath)
{
  const auto geometry = makeGeometry();
  std::vector<geometry_msgs::msg::Pose> path;
  for (size_t i = 0; i < 10; ++i) {
    path.push_back(makePose(static_cast<double>(i) * 0.5, 0.0, 0.0));
  }

  const auto result = predictArticulatedPath(path, {geometry, geometry}, 0.6);

  ASSERT_TRUE(result.articulation_valid);
  ASSERT_EQ(result.poses.size(), path.size());
  ASSERT_EQ(result.poses.back().trailers.size(), 2U);
  EXPECT_NEAR(result.poses.back().trailers.at(0).position.y, 0.0, 1e-6);
  EXPECT_NEAR(result.poses.back().trailers.at(1).position.y, 0.0, 1e-6);
}

TEST(TrailerGeometryTest, TrailerCutsInsideCurvedTractorPath)
{
  const auto geometry = makeGeometry();
  std::vector<geometry_msgs::msg::Pose> path;
  constexpr double radius = 5.0;
  for (size_t i = 0; i < 30; ++i) {
    const double angle = static_cast<double>(i) * 0.03;
    path.push_back(makePose(radius * std::sin(angle), radius * (1.0 - std::cos(angle)), angle));
  }

  const auto result = predictArticulatedPath(path, {geometry}, 0.6);

  ASSERT_TRUE(result.articulation_valid);
  const auto & tractor = result.poses.back().tractor.position;
  const auto & trailer = result.poses.back().trailers.front().position;
  const double tractor_radius = std::hypot(tractor.x, tractor.y - radius);
  const double trailer_radius = std::hypot(trailer.x, trailer.y - radius);
  EXPECT_LT(trailer_radius, tractor_radius);
}

TEST(TrailerGeometryTest, CreatesFootprintWithConfiguredDimensions)
{
  const auto geometry = makeGeometry();
  const auto footprint = createTrailerFootprint(makePose(0.0, 0.0, 0.0), geometry);

  ASSERT_EQ(footprint.outer().size(), 5U);
  EXPECT_NEAR(footprint.outer().at(0).x(), geometry.axle_to_body_front, 1e-6);
  EXPECT_NEAR(std::abs(footprint.outer().at(0).y()), geometry.width * 0.5, 1e-6);
}

}  // namespace autoware::behavior_path_planner
