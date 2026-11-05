#include "didrive/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <opencv2/imgproc.hpp>
#include <gtest/gtest.h>

#include <cstring>
#include <vector>

namespace {
sensor_msgs::msg::PointCloud2 makeCloud(bool with_top_edge) {
  std::vector<float> xyz;
  const auto add = [&](float x, float y) {
    xyz.insert(xyz.end(), {x, y, 1.0f});
  };
  for (int i = 0; i <= 100; ++i) {
    add(i * 0.1f, 0.0f);
    if (with_top_edge) add(i * 0.1f, 4.0f);
  }
  for (int i = 1; i < 40; ++i) {
    add(0.0f, i * 0.1f);
    add(10.0f, i * 0.1f);
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.width = xyz.size() / 3;
  cloud.point_step = 12;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.is_dense = true;
  cloud.fields.resize(3);
  for (int i = 0; i < 3; ++i) {
    cloud.fields[i].name = i == 0 ? "x" : i == 1 ? "y" : "z";
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }
  cloud.data.resize(xyz.size() * sizeof(float));
  std::memcpy(cloud.data.data(), xyz.data(), cloud.data.size());
  return cloud;
}

std::vector<cv::Point2f> footprint(
    const autoware_perception_msgs::msg::DetectedObject& object) {
  std::vector<cv::Point2f> points;
  const auto& position = object.kinematics.pose_with_covariance.pose.position;
  for (const auto& p : object.shape.footprint.points) {
    points.emplace_back(p.x + position.x, p.y + position.y);
  }
  return points;
}

autoware_perception_msgs::msg::DetectedObjects detect(bool with_top_edge, bool concave) {
  didrive::euclidean_cluster::VoxelGridBasedEuclideanCluster cluster(
      true, 1, 50000, 1.0f, 0.2f, 1);
  cluster.setConcaveFootprint(concave, 0.1f, 0.5f, 6.0f, 0.15f);
  autoware_perception_msgs::msg::DetectedObjects output;
  const auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(
      makeCloud(with_top_edge));
  EXPECT_TRUE(cluster.cluster(cloud, output));
  return output;
}
}  // namespace

TEST(ConcaveFootprint, RetainsTheOpeningOfALongUShapedCluster) {
  const auto old_output = detect(false, false);
  const auto new_output = detect(false, true);
  ASSERT_EQ(old_output.objects.size(), 1u);
  ASSERT_EQ(new_output.objects.size(), 1u);
  const auto old_polygon = footprint(old_output.objects.front());
  const auto new_polygon = footprint(new_output.objects.front());
  EXPECT_GT(cv::pointPolygonTest(old_polygon, cv::Point2f(5.0f, 2.0f), false), 0);
  EXPECT_LT(cv::pointPolygonTest(new_polygon, cv::Point2f(5.0f, 2.0f), false), 0);
  EXPECT_LT(cv::contourArea(new_polygon), 0.8 * cv::contourArea(old_polygon));
}

TEST(ConcaveFootprint, KeepsAClosedLargeObjectOccupied) {
  const auto output = detect(true, true);
  ASSERT_EQ(output.objects.size(), 1u);
  EXPECT_GT(
      cv::pointPolygonTest(footprint(output.objects.front()),
                           cv::Point2f(5.0f, 2.0f), false),
      0);
}
