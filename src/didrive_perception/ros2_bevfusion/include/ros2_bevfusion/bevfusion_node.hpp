#ifndef NV_BEVFUSION_NODE_HPP_
#define NV_BEVFUSION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "bevfusion/bevfusion.hpp"
#include "common/tensor.hpp"
#include "common/timer.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"

namespace didrive::ros2bevfusion
{

class NVBEVFusionNode : public rclcpp::Node
{
public:
  explicit NVBEVFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NVBEVFusionNode() override;

private:
  // 初始化函数
  void initialize();

  // 参数配置
  void declare_parameters();
  void load_parameters();

  // 模型初始化
  std::shared_ptr<bevfusion::Core> create_core(
    const std::string& model, const std::string& precision);

  // 回调函数
  void sensor_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img0_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img1_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img2_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img3_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img4_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & img5_msg
  );

  // 数据转换
  nv::Tensor convert_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg);
  std::vector<unsigned char*> convert_images(const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & img_msgs);

  // 结果发布
  void publish_detections(const std_msgs::msg::Header & header, const std::vector<bevfusion::head::transbbox::BoundingBox> & bboxes);

  // 内存管理
  void free_image_buffers(std::vector<unsigned char*> & buffers);

  // BEVFusion核心组件
  std::shared_ptr<bevfusion::Core> bev_core_;
  cudaStream_t stream_;

  // 订阅器
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_subs_[6];

  // 时间同步器 (1个激光雷达 + 6个相机)
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image
  >;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // 发布器
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr detection_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // 配置参数
  std::string model_path_;
  std::string precision_;
  float confidence_threshold_;
  bool enable_timer_;
  std::string lidar_frame_id_;
  std::string output_frame_id_;
  float max_time_diff_;
    
  // 坐标变换矩阵
  nv::Tensor camera2lidar_;
  nv::Tensor camera_intrinsics_;
  nv::Tensor lidar2image_;
  nv::Tensor img_aug_matrix_;
};

}  // namespace didrive::ros2bevfusion

#endif  // NV_BEVFUSION_NODE_HPP_
