/*
 * lidar camera calibrate node *
*/

#ifndef LIDAR_CAMERA_CALIB_NODE_HPP_
#define LIDAR_CAMERA_CALIB_NODE_HPP_

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
#include "calibration_error_term.h"

#include <image_transport/image_transport.hpp>
#include <deque>
#include <mutex>

class LidarCameraCalibNode : public rclcpp::Node
{
public:
  // init function
  explicit LidarCameraCalibNode(const rclcpp::NodeOptions &node_options);

private:
  // lidar and image msgs callback function
  void MsgsCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar,
      const sensor_msgs::msg::Image::ConstSharedPtr &image);
  // calculate extrinsic 
  void runSolver();
  // detect chessboard corners
  void imageHandler(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);
  // cluster chessboard pointcloud
  void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg);


  // lidar msg subscriber
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_{};
  // image msg subscriber
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_{};
  // message filters
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;
  // pub chessboard pointcloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  // pub pointclud after pass through filter
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_range_pub_;
  // pub chessboard corners detect result
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  // void onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  // void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  // image_transport::Subscriber img_sub_;
  // std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_queue_;
  // std::mutex image_queue_mutex_; // 保护队列的互斥锁

  // message sync queue size
  int sync_queue_size_;
  // camera intrinsic
  cv::Mat camera_matrix_;
  // camera dist coeff
  cv::Mat distCoeff_;
  // chessboard 3D points
  std::vector<cv::Point3f> object_points_;
  // detect chessboard flag
  bool boardDetectedInCam_;
  // chessboard size (m)
  double dx_, dy_;
  // chessboard corner points
  int checkerboard_rows_, checkerboard_cols_;
  // min points on chessboard
  int min_points_on_plane_;
  // chessboard pose in camera coordinate
  Eigen::Vector3d r3_;
  Eigen::Vector3d r3_old_;
  Eigen::Vector3d Nc_;
  // lidar points of chessboard
  std::vector<Eigen::Vector3d> lidar_points_;
  std::vector<std::vector<Eigen::Vector3d>> all_lidar_points_;
  // all chessboard normals
  std::vector<Eigen::Vector3d> all_normals_;
  // result file path
  std::string result_file_, result_rpy_file_;
  // num of different views of chessboard
  int num_views_;
  // image size
  int image_width_, image_height_;
  // pointclouds filter range
  double x_min_, x_max_;
  double y_min_, y_max_;
  double z_min_, z_max_;
  // ransac threshold
  double ransac_threshold_;
  // solver related params
  int no_of_initializations_;
  std::string initializations_file_;
  std::ofstream init_file_stream_;
};

#endif // LIDAR_CAMERA_CALIB_NODE_HPP_
