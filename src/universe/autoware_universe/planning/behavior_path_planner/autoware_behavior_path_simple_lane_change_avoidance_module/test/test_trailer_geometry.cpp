// Copyright 2026 BYD
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

#include "autoware/behavior_path_planner_common/utils/articulated_vehicle/trailer_geometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::behavior_path_planner
{
namespace
{
TrailerGeometry makeTrailer()
{
  TrailerGeometry geometry;
  geometry.type = "default";
  geometry.width = 1.3;
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

TEST(SimpleLaneChangeAvoidanceTrailerTest, ValidatesConfiguredGeometry)
{
  const auto geometry = makeTrailer();
  EXPECT_TRUE(isValidTrailerGeometry(geometry));

  auto invalid = geometry;
  invalid.width = 0.0;
  EXPECT_FALSE(isValidTrailerGeometry(invalid));
}

TEST(SimpleLaneChangeAvoidanceTrailerTest, KeepsTrailerAlignedOnStraightPath)
{
  const auto geometry = makeTrailer();
  std::vector<geometry_msgs::msg::Pose> path;
  for (size_t i = 0; i < 10; ++i) {
    path.push_back(makePose(static_cast<double>(i), 0.0, 0.0));
  }

  const auto result = predictArticulatedPath(path, {geometry}, 0.6);

  ASSERT_TRUE(result.articulation_valid);
  ASSERT_EQ(result.poses.size(), path.size());
  ASSERT_EQ(result.poses.back().trailers.size(), 1U);
  EXPECT_NEAR(result.poses.back().trailers.front().position.y, 0.0, 1e-6);
}

TEST(SimpleLaneChangeAvoidanceTrailerTest, RejectsExcessiveArticulation)
{
  auto geometry = makeTrailer();
  geometry.max_articulation_angle_rad = 0.05;
  std::vector<geometry_msgs::msg::Pose> path{
    makePose(0.0, 0.0, 0.0), makePose(0.2, 0.2, M_PI_2), makePose(0.2, 0.4, M_PI_2)};

  const auto result = predictArticulatedPath(path, {geometry}, 0.6);

  EXPECT_FALSE(result.articulation_valid);
  EXPECT_EQ(result.invalid_trailer_index, 0U);
}

}  // namespace autoware::behavior_path_planner
