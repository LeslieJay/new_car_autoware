/*
 * 可视化节点头文件 *
*/

#ifndef LIDAR_CAMERA_PROJ_NODE_HPP_
#define LIDAR_CAMERA_PROJ_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"

class LidarCameraProjNode : public rclcpp::Node
{
public:
  // init function
  explicit LidarCameraProjNode(const rclcpp::NodeOptions &node_options);

private:
  // lidar and image msg callback function
  void MsgsCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar,
      const sensor_msgs::msg::Image::ConstSharedPtr &image);

  // msgs publisher and subscribers
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_{};
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_{};

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  // parms
  int sync_queue_size_;
  cv::Mat camera_matrix_;
  cv::Mat distCoeff_;
  Eigen::Matrix4f T_lidar_camera_;
  
  int image_width_, image_height_;
  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;
};

#endif // LIDAR_CAMERA_PROJ_NODE_HPP_
