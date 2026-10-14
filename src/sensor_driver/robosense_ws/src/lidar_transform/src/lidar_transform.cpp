#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <chrono>
#include <cstring>



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
    sensor_qos.keep_last(1);
    pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/sensing/lidar/top/outlier_filtered/pointcloud", sensor_qos);

    // 创建订阅者
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/rslidar_points", sensor_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          this->pointCloudCallback(msg);
        });
  }

private:
  static bool hasField(
    const sensor_msgs::msg::PointCloud2 & cloud, const char * name,
    const uint32_t offset, const uint8_t datatype)
  {
    for (const auto & field : cloud.fields) {
      if (field.name == name && field.offset == offset && field.datatype == datatype &&
          field.count == 1) {
        return true;
      }
    }
    return false;
  }

  static void setField(
    sensor_msgs::msg::PointCloud2 & cloud, const size_t index,
    const char * name, const uint32_t offset, const uint8_t datatype)
  {
    auto & field = cloud.fields[index];
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
  }

  bool convertRawCloud(const sensor_msgs::msg::PointCloud2 & input)
  {
    using PointT = autoware::point_types::PointXYZIRC;
    using Field = sensor_msgs::msg::PointField;
    static_assert(sizeof(PointT) == 16, "PointXYZIRC must match the ROS point layout");
    // The RoboSense driver publishes packed XYZ, intensity, ring and timestamp.
    // Keep the PCL conversion below for other layouts instead of interpreting
    // unknown data as this driver's 26-byte format.
    if (input.is_bigendian || input.point_step != 26 ||
        input.row_step < static_cast<size_t>(input.width) * input.point_step ||
        input.data.size() < static_cast<size_t>(input.row_step) * input.height ||
        !hasField(input, "x", 0, Field::FLOAT32) ||
        !hasField(input, "y", 4, Field::FLOAT32) ||
        !hasField(input, "z", 8, Field::FLOAT32) ||
        !hasField(input, "intensity", 12, Field::FLOAT32) ||
        !hasField(input, "ring", 16, Field::UINT16)) {
      return false;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = input.header;
    output.header.frame_id = "base_link";
    output.height = 1;
    output.is_bigendian = false;
    output.is_dense = true;
    output.point_step = sizeof(PointT);
    output.fields.resize(6);
    setField(output, 0, "x", 0, Field::FLOAT32);
    setField(output, 1, "y", 4, Field::FLOAT32);
    setField(output, 2, "z", 8, Field::FLOAT32);
    setField(output, 3, "intensity", 12, Field::UINT8);
    setField(output, 4, "return_type", 13, Field::UINT8);
    setField(output, 5, "channel", 14, Field::UINT16);
    output.data.resize(static_cast<size_t>(input.width) * input.height * sizeof(PointT));

    size_t output_count = 0;
    for (uint32_t row = 0; row < input.height; ++row) {
      const auto * source = input.data.data() + static_cast<size_t>(row) * input.row_step;
      for (uint32_t col = 0; col < input.width; ++col, source += input.point_step) {
        float x, y, z, intensity;
        uint16_t ring;
        std::memcpy(&x, source, sizeof(x));
        std::memcpy(&y, source + 4, sizeof(y));
        std::memcpy(&z, source + 8, sizeof(z));
        std::memcpy(&intensity, source + 12, sizeof(intensity));
        std::memcpy(&ring, source + 16, sizeof(ring));
        const float range_squared = x * x + y * y + z * z;
        if (!(range_squared >= RadiusOutlierFilter * RadiusOutlierFilter) || std::isnan(z)) {
          continue;
        }
        PointT point;
        point.x = x + TransformX;
        point.y = y + TransformY;
        point.z = z + TransformZ;
        point.intensity = static_cast<uint8_t>(intensity);
        point.return_type = autoware::point_types::ReturnType::SINGLE_STRONGEST;
        point.channel = ring;
        std::memcpy(output.data.data() + output_count * sizeof(PointT), &point, sizeof(PointT));
        ++output_count;
      }
    }
    output.width = output_count;
    output.row_step = output_count * output.point_step;
    output.data.resize(output.row_step);
    const auto conversion_done = std::chrono::steady_clock::now();
    pub_filtered_->publish(std::move(output));
    const auto publish_done = std::chrono::steady_clock::now();
    conversion_duration_ms_ +=
      std::chrono::duration<double, std::milli>(conversion_done - conversion_start_).count();
    publish_duration_ms_ +=
      std::chrono::duration<double, std::milli>(publish_done - conversion_done).count();
    return true;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto start = std::chrono::steady_clock::now();
    conversion_start_ = start;
    if (convertRawCloud(*msg)) {
      reportCallbackTiming(start);
      return;
    }
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
    filtered->is_dense = true;

    // 转换为ROS消息
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered, output);
    output.is_dense = true;
    
    // 修改frame_id为sensor_kit_base_link
    output.header.frame_id = "base_link";
    output.header.stamp = msg->header.stamp;  // 保持原始时间戳

    // 发布处理后的点云
    pub_filtered_->publish(output);
    reportCallbackTiming(start);
  }

  void reportCallbackTiming(const std::chrono::steady_clock::time_point start)
  {
    const auto end = std::chrono::steady_clock::now();
    callback_duration_ms_ += std::chrono::duration<double, std::milli>(end - start).count();
    ++callback_count_;
    if (callback_count_ < 50) {
      return;
    }
    const double elapsed_sec = std::chrono::duration<double>(end - report_start_).count();
    RCLCPP_INFO(
      get_logger(),
      "Top LiDAR transform: input %.2f Hz, callback mean %.2f ms, conversion %.2f ms, publish %.2f ms",
      callback_count_ / elapsed_sec, callback_duration_ms_ / callback_count_,
      conversion_duration_ms_ / callback_count_, publish_duration_ms_ / callback_count_);
    callback_count_ = 0;
    callback_duration_ms_ = 0.0;
    conversion_duration_ms_ = 0.0;
    publish_duration_ms_ = 0.0;
    report_start_ = end;
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_filtered_;
  double RadiusOutlierFilter, TransformX, TransformY, TransformZ, TransformPitch, TransformRoll, TransformYaw;
  std::chrono::steady_clock::time_point report_start_{std::chrono::steady_clock::now()};
  size_t callback_count_{0};
  double callback_duration_ms_{0.0};
  std::chrono::steady_clock::time_point conversion_start_;
  double conversion_duration_ms_{0.0};
  double publish_duration_ms_{0.0};
};

}  // namespace lidar_transform

}  // namespace autoware
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_transform::LidarTransformNode)
