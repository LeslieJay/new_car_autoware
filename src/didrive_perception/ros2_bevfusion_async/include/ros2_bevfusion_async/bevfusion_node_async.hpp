#ifndef NV_BEVFUSION_NODE_ASYNC_HPP_
#define NV_BEVFUSION_NODE_ASYNC_HPP_

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

#include "bevfusion/bevfusion.hpp"
#include "common/tensor.hpp"
#include "common/timer.hpp"

#include <Eigen/Core>
#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware/object_recognition_utils/object_recognition_utils.hpp>

#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ros2_bevfusion_async/detection_class_remapper.hpp"
#include "ros2_bevfusion_async/utility.hpp"

namespace didrive::ros2_bevfusion_async
{

class NVBEVFusionNodeAsync : public rclcpp::Node
{
public:
  using Matrix4f = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

  explicit NVBEVFusionNodeAsync(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~NVBEVFusionNodeAsync() override;

private:
  // 初始化函数
  void initialize();
  void allocateBuffers();

  // 参数配置
  void declare_parameters();
  void load_parameters();

  // 模型初始化
  std::shared_ptr<bevfusion::Core> create_core(
    const std::string& model, const std::string& precision);
  
  // 订阅回调
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t camera_id);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo & msg, std::size_t camera_id);

  // 数据转换
  nv::Tensor convert_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg);
  std::vector<unsigned char*> convert_images(
    const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & img_msgs,
    const std::vector<float> & camera_masks);

  // det结果转发
  void box3DToDetectedObject(
      const bevfusion::head::transbbox::BoundingBox &box3d,
      const std::vector<std::string> &class_names,
      autoware_perception_msgs::msg::DetectedObject &obj);

  // 内存管理
  void free_image_buffers(std::vector<unsigned char*> & buffers);

  // BEVFusion核心组件
  std::shared_ptr<bevfusion::Core> bev_core_;
  cudaStream_t stream_;

  // 发布器
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;

  // 配置参数
  std::string model_path_;
  std::string precision_;
  float confidence_threshold_;
  bool enable_timer_;
  int num_cameras_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr cloud_sub_{nullptr};
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::ConstSharedPtr> camera_info_subs_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_{
    nullptr};
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  DetectionClassRemapper detection_class_remapper_;
  std::vector<std::string> class_names_;
  std::optional<std::string> lidar_frame_;
  float max_camera_lidar_delay_;
  double lidar_stamp_;

  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> image_msgs_;
  // 固定大小的互斥锁数组
  std::mutex image_mutexes_[6];
  std::vector<float> camera_masks_;
  std::vector<std::optional<sensor_msgs::msg::CameraInfo>> camera_info_msgs_;
  std::vector<std::optional<Matrix4f>> lidar2camera_extrinsics_;

  // 预分配的缓冲区
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> temp_image_msgs_;  // 预分配的图像消息缓冲区
  std::vector<unsigned char*> cpu_image_buffers_;       // CPU图像缓冲区
  std::vector<unsigned char*> gpu_image_buffers_;       // GPU图像缓冲区
  static constexpr size_t IMG_DATA_SIZE = INFER_IMG_WIDTH * INFER_IMG_HEIGHT * 3;
  bool buffers_allocated_ = false;

  bool sensor_fusion_{false};
  bool images_available_{false};
  bool intrinsics_available_{false};
  bool extrinsics_available_{false};
  bool intrinsics_extrinsics_precomputed_{false};

  // 坐标变换矩阵
  nv::Tensor camera2lidar_;
  nv::Tensor camera_intrinsics_;
  nv::Tensor lidar2image_;
  nv::Tensor img_aug_matrix_;

  // debugger
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
    nullptr};
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_pub_{nullptr};
  
};

}  // namespace didrive::ros2_bevfusion_async

#endif  // NV_BEVFUSION_NODE_ASYNC_HPP_
