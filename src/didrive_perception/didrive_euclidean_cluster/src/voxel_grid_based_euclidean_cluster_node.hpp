// Copyright 2020 Tier IV, Inc.
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

#pragma once

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include "didrive/euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

namespace didrive::euclidean_cluster {
class VoxelGridBasedEuclideanClusterNode : public rclcpp::Node {
 public:
  explicit VoxelGridBasedEuclideanClusterNode(
      const rclcpp::NodeOptions& options);

 private:
  void onPointCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void crop_box_filter(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input,
      sensor_msgs::msg::PointCloud2::SharedPtr& output);
  void publishCropBoxPolygon(const std_msgs::msg::Header& header);
  CropBoxParam crop_params_;
  bool use_crop_filter_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;
  //   rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::
  //       SharedPtr cluster_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
      cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      crop_box_polygon_pub_;

  std::shared_ptr<VoxelGridBasedEuclideanCluster> cluster_;
  std::unique_ptr<
      autoware::universe_utils::StopWatch<std::chrono::milliseconds>>
      stop_watch_ptr_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;
};

}  // namespace didrive::euclidean_cluster
