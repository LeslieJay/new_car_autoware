#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>  // 引入 Eigen 处理矩阵运算
#include <cmath>

namespace autoware {
namespace lidar_transform {

class SideLidarCropBoxNode : public rclcpp::Node
{
public:
  SideLidarCropBoxNode(const rclcpp::NodeOptions & options)
    : Node("side_lidar_cropbox_node", options)
  {
    // 1. 参数声明
    auto input_topic = this->declare_parameter<std::string>("input_topic", "/rslidar_points");
    auto output_topic = this->declare_parameter<std::string>("output_topic", "/output/points_filtered");

    // 坐标变换参数 (RPY 欧拉角)
    TransformX = this->declare_parameter("transform_x", 0.0);
    TransformY = this->declare_parameter("transform_y", 0.0);
    TransformZ = this->declare_parameter("transform_z", 0.0);
    TransformRoll = this->declare_parameter("transform_roll", 3.14159265); // 默认 180 度
    TransformPitch = this->declare_parameter("transform_pitch", 0.0);
    TransformYaw = this->declare_parameter("transform_yaw", 0.0);

    // 裁剪参数
    CropMinX = this->declare_parameter("crop_min_x", -2.0);
    CropMaxX = this->declare_parameter("crop_max_x", 1.0);
    CropMinY = this->declare_parameter("crop_min_y", -1.5);
    CropMaxY = this->declare_parameter("crop_max_y", 1.5);
    CropMinZ = this->declare_parameter("crop_min_z", -0.5);
    CropMaxZ = this->declare_parameter("crop_max_z", 2.0);
    CropRadius = this->declare_parameter("crop_radius", 0.1);
    
    // --- 核心：预计算变换矩阵 ---
    // 使用 Eigen 构建旋转矩阵 (Z-Y-X 顺序: Yaw -> Pitch -> Roll)
    Eigen::AngleAxisf rollAngle(TransformRoll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(TransformPitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(TransformYaw, Eigen::Vector3f::UnitZ());
    
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    affine_transform_ = Eigen::Affine3f::Identity();
    affine_transform_.rotate(q);
    affine_transform_.translation() << TransformX, TransformY, TransformZ;

    crop_radius_sq_ = CropRadius * CropRadius;

    RCLCPP_INFO(this->get_logger(), "Side Lidar CropBox Node Initialized with 3D Transform");
    RCLCPP_INFO(this->get_logger(), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", TransformRoll, TransformPitch, TransformYaw);

    // 2. 订阅与发布
    pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->pointCloudCallback(msg);
        });
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_in); 

    using PointT = autoware::point_types::PointXYZIRC;
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    filtered->reserve(cloud_in->size());

    for (const auto &p : *cloud_in) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

      // A. 裁剪逻辑 (在雷达自身坐标系下)
      bool in_box = (p.x >= CropMinX && p.x <= CropMaxX) &&
                    (p.y >= CropMinY && p.y <= CropMaxY) &&
                    (p.z >= CropMinZ && p.z <= CropMaxZ);

      float range_sq = p.x * p.x + p.y * p.y;
      bool in_radius = range_sq <= crop_radius_sq_;

      if (in_box || in_radius) continue; 

      // B. 核心：执行 3D 坐标变换
      // 将点转换为 Eigen 向量
      Eigen::Vector3f point_in(p.x, p.y, p.z);
      Eigen::Vector3f point_out = affine_transform_ * point_in;

      // C. 填充输出
      PointT pt;
      pt.x = point_out.x();
      pt.y = point_out.y();
      pt.z = point_out.z();
      pt.intensity = static_cast<uint8_t>(p.intensity);
      pt.channel = 0;
      pt.return_type = autoware::point_types::ReturnType::SINGLE_STRONGEST;
      
      filtered->points.push_back(pt);
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.header.frame_id = "base_link";
    output.header.stamp = msg->header.stamp;
    pub_filtered_->publish(output);
  }

  // 变量声明
  double TransformX, TransformY, TransformZ;
  double TransformRoll, TransformPitch, TransformYaw;
  Eigen::Affine3f affine_transform_; // 存储完整的旋转平移矩阵
  
  double CropMinX, CropMaxX, CropMinY, CropMaxY, CropMinZ, CropMaxZ;
  double CropRadius, crop_radius_sq_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
};

}  // namespace lidar_transform
}  // namespace autoware

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_transform::SideLidarCropBoxNode)
