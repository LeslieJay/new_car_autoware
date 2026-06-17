
#include <type_traits>
#define EIGEN_MPL2_ONLY

#include <boost/optional.hpp>

#include <chrono>
#include <unordered_map>
#include <utility>

#include "lidar_camera_proj_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>


LidarCameraProjNode::LidarCameraProjNode(const rclcpp::NodeOptions &node_options)
    : rclcpp::Node("lidar_camera_proj_node", node_options),

      lidar_sub_(this, "input/lidar", rclcpp::QoS{1}.get_rmw_qos_profile()),
      image_sub_(this, "input/camera", rclcpp::QoS{1}.get_rmw_qos_profile())
{

  // 获取参数
  sync_queue_size_ = 10;
  camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
  distCoeff_ = cv::Mat::zeros(8, 1, CV_64F);
  
  x_min_ = declare_parameter<double>("x_min");
  x_max_ = declare_parameter<double>("x_max");
  y_min_ = declare_parameter<double>("y_min");
  y_max_ = declare_parameter<double>("y_max");
  z_min_ = declare_parameter<double>("z_min");
  z_max_ = declare_parameter<double>("z_max");
  const double sync_diff = declare_parameter<double>("sync_diff");

  const auto camera_matrix_vec = this->declare_parameter<std::vector<double>>("camera_matrix");
  const auto camera_dist_vec = this->declare_parameter<std::vector<double>>("dist_coeff");
  const auto T_lidar_camera_vec = this->declare_parameter<std::vector<double>>("T_lidar_camera");
  // 检查 vector 大小是否正确
  if (camera_matrix_vec.size() != 9)
  {
    RCLCPP_ERROR(this->get_logger(), "相机矩阵参数大小不正确，需要 9 个值，但提供了 %zu 个", camera_matrix_vec.size());
    return;
  }
  if (camera_dist_vec.size() < 5)
  { // 畸变系数通常至少有 5 个
    RCLCPP_ERROR(this->get_logger(), "畸变系数参数大小不正确，需要至少 5 个值，但提供了 %zu 个", camera_dist_vec.size());
    return;
  }
  // 检查外参矩阵大小
  if (T_lidar_camera_vec.size() != 16)
  { 
    RCLCPP_ERROR(this->get_logger(), "外参矩阵大小不正确，需要 16 个值，但提供了 %zu 个", T_lidar_camera_vec.size());
    return;
  }


  // 使用 memcpy 直接复制数据
  std::memcpy(camera_matrix_.data, camera_matrix_vec.data(), sizeof(double) * 9);
  std::memcpy(distCoeff_.data, camera_dist_vec.data(), sizeof(double) * 8);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
        T_lidar_camera_(i, j) = T_lidar_camera_vec[i * 4 + j];
    }
  }
  std::cout << "lidar to camera matrix: \n" << T_lidar_camera_ << std::endl;
  std::cout << "distCoeff_: \n" << distCoeff_ << std::endl;
  
  // 创建消息发布器和订阅器
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), lidar_sub_, image_sub_);
  sync_ptr_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_diff)); // 100ms 容差

  sync_ptr_->registerCallback(
      std::bind(&LidarCameraProjNode::MsgsCallback, this, _1, _2));

  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("debug/projected_image", rclcpp::QoS{1});
}


void LidarCameraProjNode::MsgsCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar,
    const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*lidar, *in_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PassThrough<pcl::PointXYZ> pass;

  // 设置 x 轴过滤
  pass.setInputCloud(in_cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(x_min_, x_max_);
  pass.filter(*cloud_filtered);
 
  // 设置 y 轴过滤
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(y_min_, y_max_);
  pass.filter(*cloud_filtered);

  // 设置 z 轴过滤
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min_, z_max_);
  pass.filter(*cloud_filtered);

  cv::Mat image_in;
    // 使用 toCvCopy 创建副本，避免共享问题
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image, 
      sensor_msgs::image_encodings::RGB8  // 使用命名常量，更安全
  );
  image_in = cv_ptr->image;

  // 1. 提取外参：R (旋转向量) 和 t (平移向量)
  // //    假设 T_lidar_camera_ 是 Eigen::Matrix4f 类型的变换矩阵
  // Eigen::Matrix3f R_mat = T_lidar_camera_.block<3,3>(0,0);
  // Eigen::Vector3f t_vec_eigen = T_lidar_camera_.block<3,1>(0,3);

  // // 转为 OpenCV 格式
  // cv::Mat R_mat_cv, rvec, t_vec;
  // cv::eigen2cv(R_mat, R_mat_cv);
  // cv::eigen2cv(t_vec_eigen, t_vec);

  // // 将旋转矩阵转为旋转向量（Rodrigues）
  // cv::Rodrigues(R_mat_cv, rvec);  // rvec 输出为 3x1
  // std::vector<cv::Point3f> points_3d;
  // std::cout <<"RRRRR  " << rvec << std::endl;
  // std::cout <<"TTTTT  " << t_vec << std::endl;

  // // 投影每个点
  // for (const auto& point : cloud_filtered->points) {
  //     if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
  //         continue;

  //   points_3d.emplace_back(point.x, point.y, point.z);
  // }

  // std::vector<cv::Point2f> points_2d;
  // std::cout << "before projectPoints  " << points_3d.size() << std::endl;
  // // 3. 使用 projectPoints 投影（自动处理畸变！）
  // cv::projectPoints(
  //     points_3d,          // 3D 点（LiDAR 坐标系）
  //     rvec,               // 旋转向量
  //     t_vec,              // 平移向量
  //     camera_matrix_,     // 内参矩阵 K
  //     distCoeff_,         // 畸变系数 (k1,k2,p1,p2,k3,...)
  //     points_2d           // 输出：2D 像素坐标（已含畸变）
  // );
  // std::cout << "after projectPoints  " << points_2d.size() << std::endl;
  // for (size_t i = 0; i < points_3d.size(); ++i) {
  //     Eigen::Vector3f pt_in_cam = R_mat * Eigen::Vector3f(points_3d[i].x, points_3d[i].y, points_3d[i].z) + t_vec_eigen;
  //     if (pt_in_cam.z() <= 0) continue;

  //     const auto& uv = points_2d[i];
  //     // 5. 检查是否在图像范围内
  //     if (uv.x >= 0 && uv.x < image_in.cols && uv.y >= 0 && uv.y < image_in.rows) {
  //         // 可选：根据距离着色
  //         cv::circle(image_in, uv, 1, cv::Scalar(0, 255, 0), -1);
  //     }
  // }

  //投影每个点
  for (const auto& point : cloud_filtered->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
          continue;

      // 1. 将点转为齐次坐标
      Eigen::Vector4f point_lidar(point.x, point.y, point.z, 1.0f);

      // 2. 应用外参变换：转到相机坐标系
      Eigen::Vector4f point_cam = T_lidar_camera_ * point_lidar;

      // 3. 检查是否在相机前方
      if (point_cam.z() <= 0)
          continue;  // 在相机后方，跳过

      // 4. 应用内参：投影到图像平面
      float u = camera_matrix_.at<double>(0,0) * point_cam.x() / point_cam.z() + camera_matrix_.at<double>(0,2);
      float v = camera_matrix_.at<double>(1,1) * point_cam.y() / point_cam.z() + camera_matrix_.at<double>(1,2);

      // 5. 检查是否在图像范围内
      if (u >= 0 && u < image_in.cols && v >= 0 && v < image_in.rows) {
          // 可选：根据距离着色
          float depth = point_cam.z();
          int color_val = std::min(255, (int)(255 * (1.0f - std::min(depth, 50.0f) / 50.0f)));
          cv::circle(image_in, cv::Point(u, v), 2, cv::Scalar(0, color_val, 255 - color_val), -1);
      }
    }
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image_in).toImageMsg();
    msg->header.frame_id = image->header.frame_id;           // 可选
    msg->header.stamp = image->header.stamp;          // 时间戳  
    debug_image_pub_->publish(*msg);

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(LidarCameraProjNode)
