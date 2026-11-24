// Copyright 2026 Autoware Foundation. All rights reserved.
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

#include "autoware/shape_estimation/model/convex_hull.hpp"

#include <opencv2/imgproc.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace
{
using autoware::shape_estimation::model::ConcaveFootprintParams;
using autoware::shape_estimation::model::ConvexHullShapeModel;

pcl::PointCloud<pcl::PointXYZ> makeCloud(bool with_top_edge)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // Bottom horizontal edge: y = 0, x in [0, 10]
  for (int i = 0; i <= 100; ++i) {
    cloud.emplace_back(i * 0.1f, 0.0f, 1.0f);
    if (with_top_edge) {
      cloud.emplace_back(i * 0.1f, 4.0f, 1.0f);
    }
  }
  // Left vertical edge: x = 0, y in (0, 4)
  // Right vertical edge: x = 10, y in (0, 4)
  for (int i = 1; i < 40; ++i) {
    cloud.emplace_back(0.0f, i * 0.1f, 1.0f);
    cloud.emplace_back(10.0f, i * 0.1f, 1.0f);
  }
  return cloud;
}

std::vector<cv::Point2f> getFootprintPoints(
  const autoware_perception_msgs::msg::Shape & shape,
  const geometry_msgs::msg::Pose & pose)
{
  std::vector<cv::Point2f> points;
  for (const auto & p : shape.footprint.points) {
    points.emplace_back(p.x + pose.position.x, p.y + pose.position.y);
  }
  return points;
}
}  // namespace

TEST(ConcaveFootprint, RetainsTheOpeningOfALongUShapedCluster)
{
  const auto cloud = makeCloud(false);  // U-shaped (top edge open)

  // 1. Without concave footprint (standard convex hull)
  ConcaveFootprintParams convex_params;
  convex_params.use_concave_footprint = false;
  ConvexHullShapeModel convex_model(convex_params);

  autoware_perception_msgs::msg::Shape convex_shape;
  geometry_msgs::msg::Pose convex_pose;
  EXPECT_TRUE(convex_model.estimate(cloud, convex_shape, convex_pose));

  // 2. With concave footprint enabled
  ConcaveFootprintParams concave_params;
  concave_params.use_concave_footprint = true;
  concave_params.footprint_resolution = 0.1f;
  concave_params.footprint_connection_radius = 0.5f;
  concave_params.footprint_min_extent = 6.0f;
  concave_params.footprint_min_area_reduction = 0.15f;
  ConvexHullShapeModel concave_model(concave_params);

  autoware_perception_msgs::msg::Shape concave_shape;
  geometry_msgs::msg::Pose concave_pose;
  EXPECT_TRUE(concave_model.estimate(cloud, concave_shape, concave_pose));

  const auto old_polygon = getFootprintPoints(convex_shape, convex_pose);
  const auto new_polygon = getFootprintPoints(concave_shape, concave_pose);

  // Center opening point (5.0, 2.0) should be INSIDE convex hull, but OUTSIDE concave hull
  EXPECT_GT(cv::pointPolygonTest(old_polygon, cv::Point2f(5.0f, 2.0f), false), 0);
  EXPECT_LT(cv::pointPolygonTest(new_polygon, cv::Point2f(5.0f, 2.0f), false), 0);

  // The concave area should be at least 15% smaller than the convex area
  EXPECT_LT(cv::contourArea(new_polygon), 0.85 * cv::contourArea(old_polygon));
}

TEST(ConcaveFootprint, KeepsAClosedLargeObjectOccupied)
{
  const auto cloud = makeCloud(true);  // Closed rectangle

  ConcaveFootprintParams concave_params;
  concave_params.use_concave_footprint = true;
  concave_params.footprint_resolution = 0.1f;
  concave_params.footprint_connection_radius = 0.5f;
  concave_params.footprint_min_extent = 6.0f;
  concave_params.footprint_min_area_reduction = 0.15f;
  ConvexHullShapeModel concave_model(concave_params);

  autoware_perception_msgs::msg::Shape concave_shape;
  geometry_msgs::msg::Pose concave_pose;
  EXPECT_TRUE(concave_model.estimate(cloud, concave_shape, concave_pose));

  const auto polygon = getFootprintPoints(concave_shape, concave_pose);
  // Inside center point (5.0, 2.0) should remain occupied
  EXPECT_GT(cv::pointPolygonTest(polygon, cv::Point2f(5.0f, 2.0f), false), 0);
}

TEST(ConcaveFootprint, FallsBackToConvexHullForSmallObjects)
{
  pcl::PointCloud<pcl::PointXYZ> small_cloud;
  // Small U-shape with extent = 3m (< 6m min_extent)
  for (int i = 0; i <= 30; ++i) {
    small_cloud.emplace_back(i * 0.1f, 0.0f, 1.0f);
  }
  for (int i = 1; i < 20; ++i) {
    small_cloud.emplace_back(0.0f, i * 0.1f, 1.0f);
    small_cloud.emplace_back(3.0f, i * 0.1f, 1.0f);
  }

  ConcaveFootprintParams concave_params;
  concave_params.use_concave_footprint = true;
  concave_params.footprint_min_extent = 6.0f;  // threshold > 3.0m
  ConvexHullShapeModel concave_model(concave_params);

  autoware_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Pose pose;
  EXPECT_TRUE(concave_model.estimate(small_cloud, shape, pose));

  const auto polygon = getFootprintPoints(shape, pose);
  // Center point (1.5, 1.0) should still be INSIDE because of fallback to convex hull
  EXPECT_GT(cv::pointPolygonTest(polygon, cv::Point2f(1.5f, 1.0f), false), 0);
}
