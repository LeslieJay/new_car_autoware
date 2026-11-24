// Copyright 2018 Autoware Foundation. All rights reserved.
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

#ifndef AUTOWARE__SHAPE_ESTIMATION__MODEL__CONVEX_HULL_HPP_
#define AUTOWARE__SHAPE_ESTIMATION__MODEL__CONVEX_HULL_HPP_

#include "autoware/shape_estimation/model/model_interface.hpp"

namespace autoware::shape_estimation
{
namespace model
{

struct ConcaveFootprintParams
{
  bool use_concave_footprint{false};
  float footprint_resolution{0.1f};
  float footprint_connection_radius{0.5f};
  float footprint_min_extent{6.0f};
  float footprint_min_area_reduction{0.15f};
};

class ConvexHullShapeModel : public ShapeEstimationModelInterface
{
public:
  ConvexHullShapeModel() = default;
  explicit ConvexHullShapeModel(const ConcaveFootprintParams & params) : params_(params) {}

  ~ConvexHullShapeModel() = default;

  bool estimate(
    const pcl::PointCloud<pcl::PointXYZ> & cluster,
    autoware_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output) override;

private:
  ConcaveFootprintParams params_{};
};

}  // namespace model
}  // namespace autoware::shape_estimation

#endif  // AUTOWARE__SHAPE_ESTIMATION__MODEL__CONVEX_HULL_HPP_
