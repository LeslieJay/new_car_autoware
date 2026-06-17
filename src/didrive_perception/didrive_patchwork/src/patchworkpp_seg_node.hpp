// Patchwork++
#include "patchworkpp/include/patchworkpp.h"

// ROS 2
#include <deque>
#include <mutex>
#include <string>
#include <utility>  // for std::pair

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  // for cv::Mat
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>  // 用于 tf2::transformToEigen
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "autoware/universe_utils/ros/debug_publisher.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

namespace didrive::patchworkpp_ros {

class PatchworkppSegNode : public rclcpp::Node {
 public:
  /// PatchworkppSegNode constructor
  PatchworkppSegNode() = delete;
  explicit PatchworkppSegNode(const rclcpp::NodeOptions &options);

 private:
  /// Register new frame
  void EstimateGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  std::pair<Eigen::MatrixX3f, Eigen::MatrixX3f> FilterWithMask(const Eigen::MatrixX3f &input,
                                                               cv::Mat &mask_img,
                                                               const Eigen::MatrixX3f &K,
                                                               const Eigen::Matrix4f &T_cam_lidar);

 private:
  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_buffer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  mutable std::mutex buffer_mutex_;   // 保护 buffer 的线程安全
  size_t max_image_buffer_size_ = 5;  // 缓存最多 5 帧图像
  bool use_mask_                = false;
  float max_time_diff_          = 0.5;
  Eigen::Matrix4f T_cam_lidar_;
  Eigen::Matrix3f K_;
  float sensor_height_ = 0;
  /// Data publishers.
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dropped_points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;

  /// Patchwork++
  std::unique_ptr<patchwork::PatchWorkpp> Patchworkpp_;

  std::string base_frame_{"base_link"};
  std::string mask_frame_;
  // debug publisher
  std::unique_ptr<autoware::universe_utils::DebugPublisher> processing_time_publisher_;
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
};

}  // namespace didrive::patchworkpp_ros
