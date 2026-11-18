// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE__ROUTE_HANDLER__GOAL_POSE_REFINEMENT_HPP_
#define AUTOWARE__ROUTE_HANDLER__GOAL_POSE_REFINEMENT_HPP_

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/distance.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <cmath>
#include <limits>

namespace autoware::route_handler
{

// Keep route admission and fixed-goal path generation on the same goal pose.
inline geometry_msgs::msg::Pose refineGoalPose(
  const geometry_msgs::msg::Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  const auto segment = autoware::experimental::lanelet2_utils::get_closest_segment(
    goal_lanelet.centerline(), lanelet_point.basicPoint());
  if (segment.empty()) {
    return goal;
  }

  const auto p1 = segment.front().basicPoint();
  const auto p2 = segment.back().basicPoint();
  const auto direction_vector = (p2 - p1).normalized();
  const auto p1_to_goal = lanelet_point.basicPoint() - p1;
  const auto refined_point = p1 + direction_vector * direction_vector.dot(p1_to_goal);

  geometry_msgs::msg::Pose refined_goal;
  refined_goal.position.x = refined_point.x();
  refined_goal.position.y = refined_point.y();
  refined_goal.position.z = refined_point.z();
  const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
  refined_goal.orientation.z = std::sin(yaw / 2.0);
  refined_goal.orientation.w = std::cos(yaw / 2.0);
  return refined_goal;
}

}  // namespace autoware::route_handler

#endif  // AUTOWARE__ROUTE_HANDLER__GOAL_POSE_REFINEMENT_HPP_
