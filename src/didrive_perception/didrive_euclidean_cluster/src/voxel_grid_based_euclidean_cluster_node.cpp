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

#include "voxel_grid_based_euclidean_cluster_node.hpp"

#include <vector>

#include "didrive/euclidean_cluster/utils.hpp"

namespace didrive::euclidean_cluster {
VoxelGridBasedEuclideanClusterNode::VoxelGridBasedEuclideanClusterNode(
    const rclcpp::NodeOptions& options)
    : Node("voxel_grid_based_euclidean_cluster_node", options) {
  const bool use_height = this->declare_parameter("use_height", false);
  const int min_cluster_size = this->declare_parameter("min_cluster_size", 1);
  const int max_cluster_size = this->declare_parameter("max_cluster_size", 500);
  const float tolerance = this->declare_parameter("tolerance", 1.0);
  const float voxel_leaf_size = this->declare_parameter("voxel_leaf_size", 0.5);
  const int min_points_number_per_voxel =
      this->declare_parameter("min_points_number_per_voxel", 3);
  // define low height filter box
  crop_params_.max_x = this->declare_parameter("max_x", 200.0);
  crop_params_.min_x = this->declare_parameter("min_x", -200.0);
  crop_params_.max_y = this->declare_parameter("max_y", 200.0);
  crop_params_.min_y = this->declare_parameter("min_y", -200.0);
  crop_params_.max_z = this->declare_parameter("max_z", 2.0);
  crop_params_.min_z = this->declare_parameter("min_z", -10.0);
  crop_params_.negative = this->declare_parameter("negative", false);
  use_crop_filter_ = this->declare_parameter("use_crop_filter", true);

  cluster_ = std::make_shared<VoxelGridBasedEuclideanCluster>(
      use_height, min_cluster_size, max_cluster_size, tolerance,
      voxel_leaf_size, min_points_number_per_voxel);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&VoxelGridBasedEuclideanClusterNode::onPointCloud, this, _1));

  // cluster_pub_ = this->create_publisher<
  //     tier4_perception_msgs::msg::DetectedObjectsWithFeature>("output",
  //                                                             rclcpp::QoS{1});

  cluster_pub_ =
      this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
          "output", rclcpp::QoS{1});
          
  rclcpp::PublisherOptions pub_options;
  pub_options.qos_overriding_options =
      rclcpp::QosOverridingOptions::with_default_policies();
  crop_box_polygon_pub_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          "~/crop_box_polygon", 10, pub_options);
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "debug/clusters", 1);
  stop_watch_ptr_ = std::make_unique<
      autoware::universe_utils::StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<autoware::universe_utils::DebugPublisher>(
      this, "voxel_grid_based_euclidean_cluster");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
}

void VoxelGridBasedEuclideanClusterNode::publishCropBoxPolygon(
    const std_msgs::msg::Header& header) {
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = crop_params_.max_x;
  const double x2 = crop_params_.min_x;
  const double x3 = crop_params_.min_x;
  const double x4 = crop_params_.max_x;

  const double y1 = crop_params_.max_y;
  const double y2 = crop_params_.max_y;
  const double y3 = crop_params_.min_y;
  const double y4 = crop_params_.min_y;

  const double z1 = crop_params_.min_z;
  const double z2 = crop_params_.max_z;

  geometry_msgs::msg::PolygonStamped polygon_msg;
  polygon_msg.header = header;
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  crop_box_polygon_pub_->publish(polygon_msg);
}

void VoxelGridBasedEuclideanClusterNode::crop_box_filter(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input,
    sensor_msgs::msg::PointCloud2::SharedPtr& output) {
  int x_offset = input->fields[pcl::getFieldIndex(*input, "x")].offset;
  int y_offset = input->fields[pcl::getFieldIndex(*input, "y")].offset;
  int z_offset = input->fields[pcl::getFieldIndex(*input, "z")].offset;

  output->data.resize(input->data.size());
  size_t output_size = 0;

  int skipped_count = 0;

  for (size_t global_offset = 0;
       global_offset + input->point_step <= input->data.size();
       global_offset += input->point_step) {
    // get point
    const float x =
        *reinterpret_cast<const float*>(&input->data[global_offset + x_offset]);
    const float y =
        *reinterpret_cast<const float*>(&input->data[global_offset + y_offset]);
    const float z =
        *reinterpret_cast<const float*>(&input->data[global_offset + z_offset]);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      skipped_count++;
      continue;
    }

    bool point_is_inside = z > crop_params_.min_z && z < crop_params_.max_z &&
                           y > crop_params_.min_y && y < crop_params_.max_y &&
                           x > crop_params_.min_x && x < crop_params_.max_x;

    if ((!crop_params_.negative && point_is_inside) ||
        (crop_params_.negative && !point_is_inside)) {
      memcpy(&output->data[output_size], &input->data[global_offset],
             input->point_step);

      output_size += input->point_step;
    }
  }

  if (skipped_count > 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "%d points contained NaN values and have been ignored",
                         skipped_count);
  }

  output->data.resize(output_size);

  // Note that tf_input_orig_frame_ is the input frame, while tf_input_frame_ is
  // the frame of the crop box
  output->header.frame_id = input->header.frame_id;
  output->header.stamp = input->header.stamp;
  output->height = 1;
  output->fields = input->fields;
  output->is_bigendian = input->is_bigendian;
  output->point_step = input->point_step;
  output->is_dense = input->is_dense;
  output->width = static_cast<uint32_t>(output->data.size() / output->height /
                                        output->point_step);
  output->row_step =
      static_cast<uint32_t>(output->data.size() / output->height);
  publishCropBoxPolygon(output->header);
}

void VoxelGridBasedEuclideanClusterNode::onPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg) {
  stop_watch_ptr_->toc("processing_time", true);

  // convert ros to pcl
  if (input_msg->data.empty()) {
    // NOTE: prevent pcl log spam
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Empty sensor points!");
  }
  // cluster and build output msg
//   tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  autoware_perception_msgs::msg::DetectedObjects output;
  if (use_crop_filter_) {
    sensor_msgs::msg::PointCloud2::SharedPtr filtered_input_msg =
        std::make_shared<sensor_msgs::msg::PointCloud2>();
    crop_box_filter(input_msg, filtered_input_msg);
    cluster_->cluster(filtered_input_msg, output);
  } else {
    cluster_->cluster(input_msg, output);
  }
  cluster_pub_->publish(output);

  // // build debug msg
  // if (debug_pub_->get_subscription_count() >= 1) {
  //   sensor_msgs::msg::PointCloud2 debug;
  //   convertObjectMsg2SensorMsg(output, debug);
  //   debug_pub_->publish(debug);
  // }
  if (debug_publisher_) {
    const double processing_time_ms =
        stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::nanoseconds(
                (this->get_clock()->now() - output.header.stamp).nanoseconds()))
            .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
        "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

}  // namespace didrive::euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(
    didrive::euclidean_cluster::VoxelGridBasedEuclideanClusterNode)
