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

#include "autoware/shape_estimation/model/convex_hull.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::shape_estimation
{
namespace
{
// Keep the result inside the original convex hull: the raster is used only to
// remove unsupported space, never to enlarge an obstacle beyond its input points.
bool makeConcaveFootprint(
  const std::vector<cv::Point> & points_mm, const std::vector<cv::Point> & hull_mm,
  float resolution, float connection_radius, float min_extent,
  float min_area_reduction, std::vector<cv::Point> & footprint_mm)
{
  if (
    points_mm.size() < 3 || hull_mm.size() < 3 || resolution <= 0.0f ||
    connection_radius < 0.0f || min_area_reduction < 0.0f ||
    min_area_reduction >= 1.0f) {
    return false;
  }

  const cv::Rect bounds = cv::boundingRect(hull_mm);
  if (
    std::max(bounds.width, bounds.height) < min_extent * 1000.0f ||
    cv::contourArea(hull_mm) < 3.0e6) {
    return false;
  }

  const double scale = 1000.0 * resolution;
  const int padding = static_cast<int>(std::ceil(connection_radius / resolution)) + 2;
  const int width = static_cast<int>(std::ceil(bounds.width / scale)) + 2 * padding + 1;
  const int height = static_cast<int>(std::ceil(bounds.height / scale)) + 2 * padding + 1;
  // Bound the cost for unexpectedly large or malformed clusters.
  if (width <= 0 || height <= 0 || static_cast<int64_t>(width) * height > 2000000) {
    return false;
  }

  const double origin_x = bounds.x - padding * scale;
  const double origin_y = bounds.y - padding * scale;
  const auto pixel = [&](const cv::Point & p) {
    return cv::Point(
      static_cast<int>(std::round((p.x - origin_x) / scale)),
      static_cast<int>(std::round((p.y - origin_y) / scale)));
  };
  cv::Mat occupied = cv::Mat::zeros(height, width, CV_8UC1);
  for (const auto & point : points_mm) {
    const auto p = pixel(point);
    occupied.at<uint8_t>(p.y, p.x) = 255;
  }
  const int radius = static_cast<int>(std::ceil(connection_radius / resolution));
  if (radius > 0) {
    const auto kernel = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(2 * radius + 1, 2 * radius + 1));
    cv::dilate(occupied, occupied, kernel);
  }

  std::vector<cv::Point> hull_pixels;
  hull_pixels.reserve(hull_mm.size());
  for (const auto & point : hull_mm) hull_pixels.push_back(pixel(point));
  cv::Mat hull_mask = cv::Mat::zeros(height, width, CV_8UC1);
  cv::fillConvexPoly(hull_mask, hull_pixels, cv::Scalar(255));
  cv::bitwise_and(occupied, hull_mask, occupied);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(occupied, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // A single polygon cannot represent disconnected components. Preserve the
  // original hull rather than silently dropping a component.
  if (contours.size() != 1) return false;

  std::vector<cv::Point> simplified;
  cv::approxPolyDP(contours.front(), simplified, 0.1 / resolution, true);
  if (simplified.size() < 3 || simplified.size() > 200) return false;
  const double footprint_area = cv::contourArea(simplified) * scale * scale;
  if (
    footprint_area <= 0.0 ||
    footprint_area > cv::contourArea(hull_mm) * (1.0 - min_area_reduction)) {
    return false;
  }

  footprint_mm.clear();
  footprint_mm.reserve(simplified.size());
  for (const auto & p : simplified) {
    footprint_mm.emplace_back(
      static_cast<int>(std::round(origin_x + p.x * scale)),
      static_cast<int>(std::round(origin_y + p.y * scale)));
  }
  return true;
}
}  // namespace

namespace model
{

bool ConvexHullShapeModel::estimate(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  // calc centroid point for convex hull height(z)
  pcl::PointXYZ centroid;
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (const auto & pcl_point : cluster) {
    centroid.x += pcl_point.x;
    centroid.y += pcl_point.y;
    centroid.z += pcl_point.z;
  }
  centroid.x = centroid.x / static_cast<double>(cluster.size());
  centroid.y = centroid.y / static_cast<double>(cluster.size());
  centroid.z = centroid.z / static_cast<double>(cluster.size());

  // calc min and max z for convex hull height(z)
  float min_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  float max_z = cluster.empty() ? 0.0 : cluster.at(0).z;
  for (const auto & point : cluster) {
    min_z = std::min(point.z, min_z);
    max_z = std::max(point.z, max_z);
  }

  std::vector<cv::Point> v_pointcloud;
  std::vector<cv::Point> v_polygon_points;
  for (size_t i = 0; i < cluster.size(); ++i) {
    v_pointcloud.push_back(
      cv::Point((cluster.at(i).x - centroid.x) * 1000.0, (cluster.at(i).y - centroid.y) * 1000.0));
  }
  cv::convexHull(v_pointcloud, v_polygon_points);
  if (params_.use_concave_footprint) {
    std::vector<cv::Point> concave_footprint;
    if (makeConcaveFootprint(
          v_pointcloud, v_polygon_points, params_.footprint_resolution,
          params_.footprint_connection_radius, params_.footprint_min_extent,
          params_.footprint_min_area_reduction, concave_footprint)) {
      v_polygon_points = std::move(concave_footprint);
    }
  }

  pcl::PointXYZ polygon_centroid;
  polygon_centroid.x = 0;
  polygon_centroid.y = 0;
  for (size_t i = 0; i < v_polygon_points.size(); ++i) {
    polygon_centroid.x += static_cast<double>(v_polygon_points.at(i).x) / 1000.0;
    polygon_centroid.y += static_cast<double>(v_polygon_points.at(i).y) / 1000.0;
  }
  polygon_centroid.x = polygon_centroid.x / static_cast<double>(v_polygon_points.size());
  polygon_centroid.y = polygon_centroid.y / static_cast<double>(v_polygon_points.size());

  for (size_t i = 0; i < v_polygon_points.size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<double>(v_polygon_points.at(i).x) / 1000.0 - polygon_centroid.x;
    point.y = static_cast<double>(v_polygon_points.at(i).y) / 1000.0 - polygon_centroid.y;
    point.z = 0.0;
    shape_output.footprint.points.push_back(point);
  }

  constexpr float ep = 0.001;
  shape_output.type = autoware_perception_msgs::msg::Shape::POLYGON;
  shape_output.dimensions.x = 0.0;
  shape_output.dimensions.y = 0.0;
  shape_output.dimensions.z = std::max((max_z - min_z), ep);
  pose_output.position.x = centroid.x + polygon_centroid.x;
  pose_output.position.y = centroid.y + polygon_centroid.y;
  pose_output.position.z = min_z + shape_output.dimensions.z * 0.5;
  pose_output.orientation.x = 0;
  pose_output.orientation.y = 0;
  pose_output.orientation.z = 0;
  pose_output.orientation.w = 1;
  return true;
}

}  // namespace model
}  // namespace autoware::shape_estimation
