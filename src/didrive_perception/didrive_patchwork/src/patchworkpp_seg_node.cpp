#include <memory>
#include <utility>
#include <vector>

// Patchwork++-ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "patchworkpp_seg_node.hpp"

#include "Utils.hpp"

namespace didrive::patchworkpp_ros {

using utils::EigenToPointCloud2;
using utils::GetTimestamps;
using utils::PointCloud2ToEigen;

PatchworkppSegNode::PatchworkppSegNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_seg_node", options) {
  patchwork::Params params;
  base_frame_          = declare_parameter<std::string>("base_frame", base_frame_);
  mask_frame_          = declare_parameter<std::string>("mask_frame", mask_frame_);
  params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
  params.num_iter      = declare_parameter<int>("num_iter", params.num_iter);
  params.num_lpr       = declare_parameter<int>("num_lpr", params.num_lpr);
  params.num_min_pts   = declare_parameter<int>("num_min_pts", params.num_min_pts);
  params.th_seeds      = declare_parameter<double>("th_seeds", params.th_seeds);

  params.th_dist    = declare_parameter<double>("th_dist", params.th_dist);
  params.th_seeds_v = declare_parameter<double>("th_seeds_v", params.th_seeds_v);
  params.th_dist_v  = declare_parameter<double>("th_dist_v", params.th_dist_v);

  params.max_range       = declare_parameter<double>("max_range", params.max_range);
  params.min_range       = declare_parameter<double>("min_range", params.min_range);
  params.uprightness_thr = declare_parameter<double>("uprightness_thr", params.uprightness_thr);

  params.verbose = declare_parameter<bool>("verbose", params.verbose);
  use_mask_      = declare_parameter<bool>("use_mask", use_mask_);
  max_time_diff_ = declare_parameter<double>("max_time_diff", max_time_diff_);
  sensor_height_ = params.sensor_height;
  // 获取参数
  std::vector<double> intrinsics = declare_parameter<std::vector<double>>("camera_intrinsic");
  // 构造 Eigen 矩阵（按行主序赋值）
  K_ << static_cast<float>(intrinsics[0]), static_cast<float>(intrinsics[1]),
      static_cast<float>(intrinsics[2]), static_cast<float>(intrinsics[3]),
      static_cast<float>(intrinsics[4]), static_cast<float>(intrinsics[5]),
      static_cast<float>(intrinsics[6]), static_cast<float>(intrinsics[7]),
      static_cast<float>(intrinsics[8]);

  // ToDo. Support intensity
  params.enable_RNR = false;

  // Construct the main Patchwork++ node
  Patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params);

  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_pointcloud",
      rclcpp::SensorDataQoS(),
      std::bind(&PatchworkppSegNode::EstimateGround, this, std::placeholders::_1));

  // 创建图像订阅者
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "input_image",            // 输入图像 topic
      rclcpp::SensorDataQoS(),  // 适合传感器数据
      std::bind(&PatchworkppSegNode::ImageCallback, this, std::placeholders::_1));

  /*
   * We use the following QoS setting for reliable ground segmentation.
   * If you want to run Patchwork++ in real-time and real-world operation,
   * please change the QoS setting
   */
  //  rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  //   cloud_publisher_  = create_publisher<sensor_msgs::msg::PointCloud2>("/output/cloud", qos);
  ground_publisher_    = create_publisher<sensor_msgs::msg::PointCloud2>("output_ground", qos);
  nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("output_nonground", qos);
  debug_image_pub_ =
      create_publisher<sensor_msgs::msg::Image>("debug/projected_image", rclcpp::QoS{1});
  dropped_points_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("dropped_points", qos);

  // create tf buffer
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Debug publisher
  processing_time_publisher_ =
      std::make_unique<autoware::universe_utils::DebugPublisher>(this, "patchworkpp_seg");
  stop_watch_ptr_ =
      std::make_unique<autoware::universe_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  RCLCPP_INFO(this->get_logger(), "Patchwork++ ROS 2 node initialized");
}

void PatchworkppSegNode::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  std::lock_guard<std::mutex> lock(buffer_mutex_);

  RCLCPP_DEBUG(this->get_logger(),
               "Received image at time %f",
               img_msg->header.stamp.sec + img_msg->header.stamp.nanosec * 1e-9);

  // 存入队列
  // === 关键检查 1：尺寸合理性 ===

  if (img_msg->height == 0 || img_msg->width == 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid image size!");

    return;
  }

  // === 关键检查 2：step 与 width 匹配（mono8 应满足 step >= width）===

  if (img_msg->encoding == "mono8" || img_msg->encoding == "8UC1") {
    if (img_msg->step < img_msg->width) {
      RCLCPP_ERROR(
          this->get_logger(), "Image step (%d) < width (%d)!", img_msg->step, img_msg->width);

      return;
    }

    size_t expected_data_size = img_msg->height * img_msg->step;

    if (img_msg->data.size() != expected_data_size) {
      RCLCPP_ERROR(this->get_logger(),

                   "Data size mismatch: got %zu, expected %zu",

                   img_msg->data.size(),
                   expected_data_size);

      return;
    }
  }

  image_buffer_.push_back(img_msg);

  // 控制缓存大小，删除最老的帧
  while (image_buffer_.size() > max_image_buffer_size_) {
    image_buffer_.pop_front();
  }
}

std::pair<Eigen::MatrixX3f, Eigen::MatrixX3f> PatchworkppSegNode::FilterWithMask(
    const Eigen::MatrixX3f &input,
    cv::Mat &mask_img,
    const Eigen::MatrixX3f &K,
    const Eigen::Matrix4f &T_cam_lidar) {
  int N = input.rows();

  Eigen::MatrixXf points_homo(4, N);
  points_homo.block(0, 0, 3, N) = input.transpose();
  points_homo.row(3).setConstant(1.0f);

  Eigen::MatrixXf points_cam_homo = T_cam_lidar.cast<float>() * points_homo;

  Eigen::VectorXf cam_x = points_cam_homo.row(0);
  Eigen::VectorXf cam_y = points_cam_homo.row(1);
  Eigen::VectorXf cam_z = points_cam_homo.row(2);

  std::vector<Eigen::Vector3f> kept_points;    // 保留的点（非地面）
  std::vector<Eigen::Vector3f> masked_points;  // 被 mask 过滤掉的点（地面）

  kept_points.reserve(N);
  masked_points.reserve(N);

  for (int i = 0; i < N; ++i) {
    bool is_masked = false;

    if (cam_z(i) > 0 && cam_y(i) > (sensor_height_ - 0.2)) {
      float u = (K(0, 0) * cam_x(i)) / cam_z(i) + K(0, 2);
      float v = (K(1, 1) * cam_y(i)) / cam_z(i) + K(1, 2);

      int img_u = static_cast<int>(u);
      int img_v = static_cast<int>(v);

      if (img_u >= 0 && img_u < mask_img.cols && img_v >= 0 && img_v < mask_img.rows) {
        uint8_t mask_value = mask_img.at<uint8_t>(img_v, img_u);
        if (mask_value == 255) {
          is_masked = true;  // 标记为“被过滤”
        }
      }
    }

    if (is_masked) {
      masked_points.emplace_back(input.row(i));
    } else {
      kept_points.emplace_back(input.row(i));
    }
  }

  // 构建 kept 矩阵
  Eigen::MatrixX3f kept(kept_points.size(), 3);
  for (size_t i = 0; i < kept_points.size(); ++i) {
    kept.row(i) = kept_points[i];
  }

  // 构建 masked 矩阵
  Eigen::MatrixX3f masked(masked_points.size(), 3);
  for (size_t i = 0; i < masked_points.size(); ++i) {
    masked.row(i) = masked_points[i];
  }

  RCLCPP_INFO(this->get_logger(),
              "Kept %ld points, masked (filtered out) %ld points using mask",
              kept.rows(),
              masked.rows());

  return std::make_pair(kept, masked);  // {kept, masked}
}

void PatchworkppSegNode::EstimateGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  stop_watch_ptr_->toc("processing_time", true);

  const auto &cloud = patchworkpp_ros::utils::PointCloud2ToEigenMat(msg);
  // Estimate ground
  Patchworkpp_->estimateGround(cloud);

  // Get ground and nonground
  Eigen::MatrixX3f ground    = Patchworkpp_->getGround();
  Eigen::MatrixX3f nonground = Patchworkpp_->getNonground();

  // whether use mask filter no ground points
  if (use_mask_) {
    // --- Step 1: 获取 T_cam_lidar ---
    bool tf_available           = false;
    Eigen::Matrix4f T_cam_lidar = Eigen::Matrix4f::Identity();  // 默认初始化
    try {
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform(mask_frame_,
                                      base_frame_,
                                      msg->header.stamp,                     // 使用点云时间戳
                                      std::chrono::duration<double>(0.05));  // 最多等待 50ms

      // 转换为 Eigen 并赋值
      T_cam_lidar  = tf2::transformToEigen(transform_stamped).matrix().cast<float>();
      tf_available = true;

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "TF lookup failed (%s -> %s): %s",
                  base_frame_.c_str(),
                  mask_frame_.c_str(),
                  ex.what());
    }
    // Step 2: 获取mask 图像
    cv_bridge::CvImagePtr cv_mask_ptr;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (!image_buffer_.empty()) {
        // 直接取最后一帧
        sensor_msgs::msg::Image::ConstSharedPtr target_mask = nullptr;
        target_mask       = image_buffer_.back();  // 使用 shared_ptr 复制，增加引用计数
        double lidar_time = rclcpp::Time(msg->header.stamp).seconds();
        double image_time = rclcpp::Time(target_mask->header.stamp).seconds();
        double time_diff  = std::abs(lidar_time - image_time);
        if (time_diff < max_time_diff_) {
          try {
            cv_mask_ptr = cv_bridge::toCvCopy(target_mask, "mono8");  // 单通道灰度图
          } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
          }
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "too much time diff :%f, lidar: %f, image: %f",
                      time_diff,
                      lidar_time,
                      image_time);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Image buffer is empty. Skipping mask filtering.");
      }
    }

    if (cv_mask_ptr != nullptr && tf_available) {
      cv::Mat &mask_img = cv_mask_ptr->image;  // 值为 0 或 255
      auto [kept, drop] = FilterWithMask(nonground, mask_img, K_, T_cam_lidar);
      nonground         = kept;
      dropped_points_publisher_->publish(
          std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(drop, msg->header)));
    } else {
      RCLCPP_WARN(this->get_logger(), "No synchronized mask found.");
    }
  }

  nonground_publisher_->publish(
      std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(nonground, msg->header)));
  ground_publisher_->publish(
      std::move(patchworkpp_ros::utils::EigenMatToPointCloud2(ground, msg->header)));

  // publish processing time
  processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", stop_watch_ptr_->toc("cyclic_time", true));
  processing_time_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

}  // namespace didrive::patchworkpp_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(didrive::patchworkpp_ros::PatchworkppSegNode)
