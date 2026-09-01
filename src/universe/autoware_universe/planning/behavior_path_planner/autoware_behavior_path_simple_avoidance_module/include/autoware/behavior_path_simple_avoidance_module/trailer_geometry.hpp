// Copyright 2025 BYD
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__TRAILER_GEOMETRY_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__TRAILER_GEOMETRY_HPP_

#include <autoware_utils/geometry/boost_geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{

struct TrailerGeometry
{
  std::string type;
  double width{0.0};
  double axle_to_body_front{0.0};
  double axle_to_body_rear{0.0};
  double front_hitch_to_axle{0.0};
  double axle_to_rear_hitch{0.0};
  double max_articulation_angle_rad{0.0};
};

struct ArticulatedPose
{
  geometry_msgs::msg::Pose tractor;
  std::vector<geometry_msgs::msg::Pose> trailers;
  double max_articulation_angle_rad{0.0};
};

struct ArticulatedPath
{
  std::vector<ArticulatedPose> poses;
  bool articulation_valid{true};
  size_t invalid_path_index{0};
  size_t invalid_trailer_index{0};
};

struct ResolvedTrailerConfiguration
{
  std::vector<std::string> types;
  std::vector<TrailerGeometry> geometries;
};

class TrailerConfigurationStore
{
public:
  bool update(
    const std::vector<std::string> & types,
    const std::unordered_map<std::string, TrailerGeometry> & configured_types);
  ResolvedTrailerConfiguration snapshot() const;

private:
  mutable std::mutex mutex_;
  ResolvedTrailerConfiguration latest_;
};

bool isValidTrailerGeometry(const TrailerGeometry & geometry);

ArticulatedPath predictArticulatedPath(
  const std::vector<geometry_msgs::msg::Pose> & tractor_path,
  const std::vector<TrailerGeometry> & trailers, double tractor_rear_axle_to_hitch);

autoware_utils::Polygon2d createTrailerFootprint(
  const geometry_msgs::msg::Pose & axle_pose, const TrailerGeometry & geometry);

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_SIMPLE_AVOIDANCE_MODULE__TRAILER_GEOMETRY_HPP_
