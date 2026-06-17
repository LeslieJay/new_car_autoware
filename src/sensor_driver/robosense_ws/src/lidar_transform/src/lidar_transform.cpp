#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;
  uint8_t padding[6];
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

namespace autoware {
namespace lidar_transform {

class LidarTransformNode : public rclcpp::Node
{
public:
  LidarTransformNode(const rclcpp::NodeOptions & options)
    : Node("points_raw_transform_node", options)
  {
    // 参数声明
    RadiusOutlierFilter = this->declare_parameter("RadiusOutlierFilter", 1.0);
    TransformX = this->declare_parameter("transform_x", 0.0);
    TransformY = this->declare_parameter("transform_y", 0.0);
    TransformZ = this->declare_parameter("transform_z", 0.0);
    TransformRoll = this->declare_parameter("transform_roll", 0.0);
    TransformPitch = this->declare_parameter("transform_pitch", 0.0);
    TransformYaw = this->declare_parameter("transform_yaw", 0.0);



    RCLCPP_INFO(this->get_logger(), "Lidar Transform Parameters:");
    RCLCPP_INFO(this->get_logger(), "  RadiusOutlierFilter: %.2f", RadiusOutlierFilter);
    RCLCPP_INFO(this->get_logger(), "  TransformX: %.2f", TransformX);
    RCLCPP_INFO(this->get_logger(), "  TransformY: %.2f", TransformY);
    RCLCPP_INFO(this->get_logger(), "  TransformZ: %.2f", TransformZ);


    // 创建发布者 - 使用 SensorDataQoS (Best Effort) 以匹配 Autoware 点云订阅者的 QoS
    rclcpp::SensorDataQoS sensor_qos;
    pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/top/outlier_filtered/pointcloud", sensor_qos);
    pub_concatenated_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/concatenated2/pointcloud", sensor_qos);

    // 创建订阅者
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->pointCloudCallback(msg);
        });
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 转换到自定义点类型
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_in(new pcl::PointCloud<PointXYZIRT>);
    pcl::fromROSMsg(*msg, *cloud_in); 

    // 创建目标类型的点云 (PointXYZIRC)
    using PointT = autoware::point_types::PointXYZIRC;
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
    filtered->reserve(cloud_in->size());

    // 手动遍历，逐点复制并应用滤波
    for (const auto &p : *cloud_in) {
      float range = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
      if (range >= RadiusOutlierFilter && !std::isnan(p.z)) {
        PointT point;
        point.x = p.x + TransformX;
        point.y = p.y + TransformY;
        point.z = p.z + TransformZ;
        point.intensity = static_cast<uint8_t>(p.intensity);
        point.channel = static_cast<uint16_t>(p.ring);
        point.return_type = autoware::point_types::ReturnType::SINGLE_STRONGEST;
        filtered->points.push_back(point);
      }
    }

    // 设置点云元数据
    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;

    // 转换为ROS消息
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    
    // 修改frame_id为sensor_kit_base_link
    output.header.frame_id = "base_link";
    output.header.stamp = msg->header.stamp;  // 保持原始时间戳

    // 发布处理后的点云
    pub_filtered_->publish(output);
    pub_concatenated_->publish(output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_, pub_concatenated_;
  double RadiusOutlierFilter, TransformX, TransformY, TransformZ, TransformPitch, TransformRoll, TransformYaw;
};

}  // namespace lidar_transform

}  // namespace autoware
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_transform::LidarTransformNode)
