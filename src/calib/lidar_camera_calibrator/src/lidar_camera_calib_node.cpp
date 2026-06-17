/*
 * lidar camera extrinsic calculate node
 * 1. sync lidar and camera msgs
 * 2. cluster chessboard pointcloud
 * 3. detect chessboard in image
 * 4. calculate extrinsic
*/

#include <type_traits>
#define EIGEN_MPL2_ONLY

#include <boost/optional.hpp>

#include <chrono>
#include <unordered_map>
#include <utility>

#include "lidar_camera_calib_node.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/covariance.h"


LidarCameraCalibNode::LidarCameraCalibNode(const rclcpp::NodeOptions &node_options)
    : rclcpp::Node("lidar_camera_calib_node", node_options),
      lidar_sub_(this, "input/lidar", rclcpp::QoS{1}.get_rmw_qos_profile()),
      image_sub_(this, "input/camera", rclcpp::QoS{1}.get_rmw_qos_profile())
{

  // 获取参数
  sync_queue_size_ = 10;
  camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
  distCoeff_ = cv::Mat::zeros(8, 1, CV_64F);
  boardDetectedInCam_ = false;
  dx_ = declare_parameter<double>("dx");
  dy_ = declare_parameter<double>("dy");
  checkerboard_rows_ = declare_parameter<int>("checkerboard_rows");
  checkerboard_cols_ = declare_parameter<int>("checkerboard_cols");
  min_points_on_plane_ = declare_parameter<int>("min_points_on_plane");
  num_views_ = declare_parameter<int>("num_views");
  no_of_initializations_ = declare_parameter<int>("no_of_initializations");
  initializations_file_ = declare_parameter<std::string>("initializations_file");
  object_points_.clear();
  for (int i = 0; i < checkerboard_rows_; i++)
    for (int j = 0; j < checkerboard_cols_; j++)
      object_points_.emplace_back(cv::Point3f(i * dx_, j * dy_, 0.0));

  result_file_ = declare_parameter<std::string>("result_file");
  result_rpy_file_ = declare_parameter<std::string>("result_rpy_file");
  x_min_ = declare_parameter<double>("x_min");
  x_max_ = declare_parameter<double>("x_max");
  y_min_ = declare_parameter<double>("y_min");
  y_max_ = declare_parameter<double>("y_max");
  z_min_ = declare_parameter<double>("z_min");
  z_max_ = declare_parameter<double>("z_max");
  const double sync_diff = declare_parameter<double>("sync_diff");
  ransac_threshold_ = declare_parameter<double>("ransac_threshold_");
  const auto camera_matrix_vec = this->declare_parameter<std::vector<double>>("camera_matrix");
  const auto camera_dist_vec = this->declare_parameter<std::vector<double>>("dist_coeff");
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

  // 使用 memcpy 直接复制数据
  std::memcpy(camera_matrix_.data, camera_matrix_vec.data(), sizeof(double) * 9);
  std::memcpy(distCoeff_.data, camera_dist_vec.data(), sizeof(double) * 8);
  std::cout << "camera matrix: \n" << camera_matrix_ << std::endl;
  // 创建消息订阅器和发布器
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), lidar_sub_, image_sub_);
  sync_ptr_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_diff)); // 100ms 容差
  sync_ptr_->registerCallback(
      std::bind(&LidarCameraCalibNode::MsgsCallback, this, _1, _2));
  debug_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("debug/chessboard_pointcloud", rclcpp::QoS{1});
  debug_cloud_range_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("debug/pointcloud_range", rclcpp::QoS{1});
  debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("debug/chessboard_image", rclcpp::QoS{1});

  // // 图像订阅 (使用 image_transport)
  // img_sub_ = image_transport::create_subscription(
  //     this,
  //     "/input/camera",
  //     std::bind(&LidarCameraCalibNode::onImage, this, std::placeholders::_1),
  //     "raw",
  //     rmw_qos_profile_default
  // );

  // // LiDAR 订阅
  // pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  //     "/input/lidar",
  //     10, // QoS history depth
  //     std::bind(&LidarCameraCalibNode::onPointCloud, this, std::placeholders::_1)
  // );
}

/*
 * 在求解外参时添加高斯噪声 *
*/
void addGaussianNoise(Eigen::Matrix4d &transformation)
{
  std::vector<double> data_rot = {0, 0, 0};
  const double mean_rot = 0.0;
  std::default_random_engine generator_rot;
  generator_rot.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::normal_distribution<double> dist(mean_rot, 90);

  // Add Gaussian noise
  for (auto &x : data_rot)
  {
    x = x + dist(generator_rot);
  }

  double roll = data_rot[0] * M_PI / 180;
  double pitch = data_rot[1] * M_PI / 180;
  double yaw = data_rot[2] * M_PI / 180;

  Eigen::Matrix3d m;
  m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  std::vector<double> data_trans = {0, 0, 0};
  const double mean_trans = 0.0;
  std::default_random_engine generator_trans;
  generator_trans.seed(std::chrono::system_clock::now().time_since_epoch().count());
  std::normal_distribution<double> dist_trans(mean_trans, 0.5);

  // Add Gaussian noise
  for (auto &x : data_trans)
  {
    x = x + dist_trans(generator_trans);
  }

  Eigen::Vector3d trans;
  trans(0) = data_trans[0];
  trans(1) = data_trans[1];
  trans(2) = data_trans[2];

  Eigen::Matrix4d trans_noise = Eigen::Matrix4d::Identity();
  trans_noise.block(0, 0, 3, 3) = m;
  trans_noise.block(0, 3, 3, 1) = trans;
  transformation = transformation * trans_noise;
}

/*
 * 点云处理函数，主要包括以下功能
 * 1. 根据设定参数截取一定范围内点云
 * 2. 拟合截取后点云里最大的平面
 * 3. 移除异常值                   
 * 4. 发布debug 点云
*/
void LidarCameraCalibNode::cloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*cloud_msg, *in_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_filtered(new pcl::PointCloud<pcl::PointXYZ>);

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

  /// Plane Segmentation
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_filtered));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
  ransac.setDistanceThreshold(ransac_threshold_);
  ransac.computeModel();
  std::vector<int> inliers_indicies;
  ransac.getInliers(inliers_indicies);
  pcl::copyPointCloud<pcl::PointXYZ>(*cloud_filtered, inliers_indicies, *plane);

  /// Statistical Outlier Removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(plane);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1);
  sor.filter(*plane_filtered);

  /// Store the points lying in the filtered plane in a vector
  lidar_points_.clear();
  for (size_t i = 0; i < plane_filtered->points.size(); i++)
  {
    double X = plane_filtered->points[i].x;
    double Y = plane_filtered->points[i].y;
    double Z = plane_filtered->points[i].z;
    lidar_points_.push_back(Eigen::Vector3d(X, Y, Z));
  }
  RCLCPP_INFO(this->get_logger(), "No of planar_pts: %d", plane_filtered->points.size());
  // 发布chessboard 点云
  sensor_msgs::msg::PointCloud2 out_cloud;
  pcl::toROSMsg(*plane_filtered, out_cloud);
  out_cloud.header.frame_id = cloud_msg->header.frame_id;
  out_cloud.header.stamp = cloud_msg->header.stamp;
  debug_cloud_pub_->publish(out_cloud);
  // 发布直通滤波后点云
  sensor_msgs::msg::PointCloud2 out_cloud_range;
  pcl::toROSMsg(*cloud_filtered, out_cloud_range);
  out_cloud_range.header.frame_id = cloud_msg->header.frame_id;
  out_cloud_range.header.stamp = cloud_msg->header.stamp;
  debug_cloud_range_pub_->publish(out_cloud_range);

}

/*
 * 图像处理函数，主要完成以下功能
 * 1. 检测棋盘格角点
 * 2. 求解棋盘格在相机坐标系的法向量
 * 3. 发布调试图像
*/
void LidarCameraCalibNode::imageHandler(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
  try
  {
    cv::Mat image_in;
    // 使用 toCvCopy 创建副本，避免共享问题
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        image_msg, 
        sensor_msgs::image_encodings::RGB8  // 使用命名常量，更安全
    );
    image_in = cv_ptr->image;
    // 将 RGB 转换为 BGR 以正确显示
    cv::Mat image_bgr;
    cv::cvtColor(image_in, image_bgr, cv::COLOR_RGB2BGR);    

     // 目标尺寸（例如：640x480）
    // const int target_w = image_bgr.cols / 2; // 或保持比例：static_cast<int>(orig_w * target_h / orig_h)
    // const int target_h = image_bgr.rows / 2; // 或保持比例：static_cast<int>(orig_h * target_w / orig_w)

    // cv::Mat image_resized;
    // cv::resize(image_bgr, image_resized, cv::Size(target_w, target_h));

    std::vector<cv::Point2f> image_points, image_points_scaled;
    // 检测棋盘格点
    // boardDetectedInCam_ = cv::findChessboardCorners(image_bgr,
    //                                                 cv::Size(checkerboard_cols_, checkerboard_rows_),
    //                                                 image_points,
    //                                                 cv::CALIB_CB_ADAPTIVE_THRESH +
    //                                                     cv::CALIB_CB_NORMALIZE_IMAGE);

    // 第一步：快速检查是否存在棋盘格
    boardDetectedInCam_ = cv::findChessboardCorners(
        image_bgr,
        cv::Size(checkerboard_cols_, checkerboard_rows_),
        image_points,
        cv::CALIB_CB_FAST_CHECK
    );

    if (boardDetectedInCam_) {
        // 第二步：只在确认存在时进行精细检测
        boardDetectedInCam_ = cv::findChessboardCorners(
            image_bgr,
            cv::Size(checkerboard_cols_, checkerboard_rows_),
            image_points,
            cv::CALIB_CB_ADAPTIVE_THRESH | 
            cv::CALIB_CB_NORMALIZE_IMAGE | 
            cv::CALIB_CB_FILTER_QUADS
        );
    }

    cv::drawChessboardCorners(image_bgr,
                              cv::Size(checkerboard_cols_, checkerboard_rows_),
                              image_points,
                              boardDetectedInCam_);

    // boardDetectedInCam_ = cv::findChessboardCorners(image_resized,
    //                                                 cv::Size(checkerboard_cols_, checkerboard_rows_),
    //                                                 image_points_scaled,
    //                                                 cv::CALIB_CB_ADAPTIVE_THRESH +
    //                                                     cv::CALIB_CB_NORMALIZE_IMAGE);

    // if (boardDetectedInCam_) {
    //     // 计算缩放比例
    //     double scale_x = static_cast<double>(image_bgr.cols) / target_w;
    //     double scale_y = static_cast<double>(image_bgr.rows) / target_h;

    //     // // 映射回原始图像坐标
    //     // image_points.resize(image_points_scaled.size());
    //     for (size_t i = 0; i < image_points_scaled.size(); ++i) {
    //     //     image_points[i] = cv::Point2f(
    //     //         image_points_scaled[i].x * scale_x,
    //     //         image_points_scaled[i].y * scale_y
    //     //     );
    //       std::cout<<"Scaled Point " << i << ": " << image_points_scaled[i].x * scale_x  << ", " << image_points_scaled[i].y * scale_y << std::endl;
    //       std::cout<<"Original Point " << i << ": " << image_points[i] << std::endl;
    //     }
    // }
    // cv::drawChessboardCorners(image_bgr,
    //                           cv::Size(checkerboard_cols_, checkerboard_rows_),
    //                           image_points,
                              // boardDetectedInCam_);
    // 求解棋盘格法向量
    if (image_points.size() == object_points_.size())
    {
      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
      cv::Mat C_R_W = cv::Mat::eye(3, 3, CV_64F);
      Eigen::Matrix3d c_R_w = Eigen::Matrix3d::Identity();
      cv::solvePnP(object_points_, image_points, camera_matrix_, distCoeff_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
      // projected_points.clear();
      std::vector<cv::Point2f> projected_points;
      cv::projectPoints(object_points_, rvec, tvec, camera_matrix_, distCoeff_, projected_points, cv::noArray());
      for (int i = 0; i < projected_points.size(); i++)
      {
        cv::circle(image_bgr, projected_points[i], 16, cv::Scalar(0, 255, 0), 10, cv::LINE_AA, 0);
      }
      cv::Rodrigues(rvec, C_R_W);
      cv::cv2eigen(C_R_W, c_R_w);
      Eigen::Vector3d c_t_w = Eigen::Vector3d(tvec.at<double>(0),
                                              tvec.at<double>(1),
                                              tvec.at<double>(2));

      r3_ = c_R_w.block<3, 1>(0, 2);
      Nc_ = (r3_.dot(c_t_w)) * r3_;
    }
    // 发布调试图片
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_bgr).toImageMsg();
    msg->header.frame_id = image_msg->header.frame_id;           // 可选
    msg->header.stamp = image_msg->header.stamp;          // 时间戳  
    debug_image_pub_->publish(*msg);

  }
  catch (cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.",
                 image_msg->encoding.c_str());
  }
}

/*
 * 求解 lidar到camera 的外参
*/
void LidarCameraCalibNode::runSolver()
{
  // check whether lidar frame and image frame can be used
  if (lidar_points_.size() > min_points_on_plane_ && boardDetectedInCam_)
  {
    // check whether chessboard has enough view change
    if (r3_.dot(r3_old_) < 0.9)
    {
      r3_old_ = r3_;
      all_normals_.push_back(Nc_);
      all_lidar_points_.push_back(lidar_points_);
      assert(all_normals_.size() == all_lidar_points_.size());
      RCLCPP_INFO(this->get_logger(), "Recording View number: %d", all_normals_.size());
      // check whether captured enough views
      if (all_normals_.size() >= num_views_)
      {
        RCLCPP_INFO(this->get_logger(), "Starting optimization...");
        init_file_stream_.open(initializations_file_);
        for (int counter = 0; counter < no_of_initializations_; counter++)
        {
          /// Start Optimization here

          /// Step 1: Initialization
          Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
          // transformation_matrix(0,0) = -0.950833;
          // transformation_matrix(0,1) = 0.188758;
          // transformation_matrix(0,2) = -0.245532;
          // transformation_matrix(1,0) =  0.305056;
          // transformation_matrix(1,1) = 0.433993;
          // transformation_matrix(1,2) = -0.847697;
          // transformation_matrix(2,0) = -0.0534505;
          // transformation_matrix(2,2) = -0.470237;
          // transformation_matrix(2,1) = -0.88092;
          // transformation_matrix(3,0) = 0.0642289;
          // transformation_matrix(3,1) = 0.0726968;
          // transformation_matrix(3,2) = -0.542222;

          addGaussianNoise(transformation_matrix);
          Eigen::Matrix3d Rotn = transformation_matrix.block(0, 0, 3, 3);
          Eigen::Vector3d axis_angle;
          ceres::RotationMatrixToAngleAxis(Rotn.data(), axis_angle.data());

          Eigen::Vector3d Translation = transformation_matrix.block(0, 3, 3, 1);

          Eigen::Vector3d rpy_init = Rotn.eulerAngles(0, 1, 2) * 180 / M_PI;
          Eigen::Vector3d tran_init = transformation_matrix.block(0, 3, 3, 1);

          Eigen::VectorXd R_t(6);
          R_t(0) = axis_angle(0);
          R_t(1) = axis_angle(1);
          R_t(2) = axis_angle(2);
          R_t(3) = Translation(0);
          R_t(4) = Translation(1);
          R_t(5) = Translation(2);
          /// Step2: Defining the Loss function (Can be NULL)
          //                    ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
          //                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
          ceres::LossFunction *loss_function = NULL;

          /// Step 3: Form the Optimization Problem
          ceres::Problem problem;
          problem.AddParameterBlock(R_t.data(), 6);
          for (int i = 0; i < all_normals_.size(); i++)
          {
            Eigen::Vector3d normal_i = all_normals_[i];
            std::vector<Eigen::Vector3d> lidar_points_i = all_lidar_points_[i];
            for (int j = 0; j < lidar_points_i.size(); j++)
            {
              Eigen::Vector3d lidar_point = lidar_points_i[j];
              ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<CalibrationErrorTerm, 1, 6>(new CalibrationErrorTerm(lidar_point, normal_i));
              problem.AddResidualBlock(cost_function, loss_function, R_t.data());
            }
          }

          /// Step 4: Solve it
          ceres::Solver::Options options;
          options.max_num_iterations = 300;
          options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
          options.minimizer_progress_to_stdout = false;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);

          // std::cout << summary.FullReport() << '\n';
          /// Printing and Storing C_T_L in a file
          ceres::AngleAxisToRotationMatrix(R_t.data(), Rotn.data());
          Eigen::MatrixXd C_T_L(3, 4);
          C_T_L.block(0, 0, 3, 3) = Rotn;
          C_T_L.block(0, 3, 3, 1) = Eigen::Vector3d(R_t[3], R_t[4], R_t[5]);
          // std::cout << "RPY = " << Rotn.eulerAngles(0, 1, 2) * 180 / M_PI << std::endl;
          // std::cout << "t = " << C_T_L.block(0, 3, 3, 1) << std::endl;

          init_file_stream_ << rpy_init(0) << "," << rpy_init(1) << "," << rpy_init(2) << ","
                            << tran_init(0) << "," << tran_init(1) << "," << tran_init(2) << "\n";
          init_file_stream_ << Rotn.eulerAngles(0, 1, 2)(0) * 180 / M_PI << "," << Rotn.eulerAngles(0, 1, 2)(1) * 180 / M_PI << "," << Rotn.eulerAngles(0, 1, 2)(2) * 180 / M_PI << ","
                            << R_t[3] << "," << R_t[4] << "," << R_t[5] << "\n";

          /// Step 5: Covariance Estimation
          ceres::Covariance::Options options_cov;
          ceres::Covariance covariance(options_cov);
          std::vector<std::pair<const double *, const double *>> covariance_blocks;
          covariance_blocks.push_back(std::make_pair(R_t.data(), R_t.data()));
          CHECK(covariance.Compute(covariance_blocks, &problem));
          double covariance_xx[6 * 6];
          covariance.GetCovarianceBlock(R_t.data(),
                                        R_t.data(),
                                        covariance_xx);

          Eigen::MatrixXd cov_mat_RotTrans(6, 6);
          cv::Mat cov_mat_cv = cv::Mat(6, 6, CV_64F, &covariance_xx);
          cv::cv2eigen(cov_mat_cv, cov_mat_RotTrans);

          Eigen::MatrixXd cov_mat_TransRot(6, 6);
          cov_mat_TransRot.block(0, 0, 3, 3) = cov_mat_RotTrans.block(3, 3, 3, 3);
          cov_mat_TransRot.block(3, 3, 3, 3) = cov_mat_RotTrans.block(0, 0, 3, 3);
          cov_mat_TransRot.block(0, 3, 3, 3) = cov_mat_RotTrans.block(3, 0, 3, 3);
          cov_mat_TransRot.block(3, 0, 3, 3) = cov_mat_RotTrans.block(0, 3, 3, 3);

          double sigma_xx = sqrt(cov_mat_TransRot(0, 0));
          double sigma_yy = sqrt(cov_mat_TransRot(1, 1));
          double sigma_zz = sqrt(cov_mat_TransRot(2, 2));

          double sigma_rot_xx = sqrt(cov_mat_TransRot(3, 3));
          double sigma_rot_yy = sqrt(cov_mat_TransRot(4, 4));
          double sigma_rot_zz = sqrt(cov_mat_TransRot(5, 5));

          // std::cout << "sigma_xx = " << sigma_xx << "\t"
          //           << "sigma_yy = " << sigma_yy << "\t"
          //           << "sigma_zz = " << sigma_zz << std::endl;

          // std::cout << "sigma_rot_xx = " << sigma_rot_xx * 180 / M_PI << "\t"
          //           << "sigma_rot_yy = " << sigma_rot_yy * 180 / M_PI << "\t"
          //           << "sigma_rot_zz = " << sigma_rot_zz * 180 / M_PI << std::endl;

          std::ofstream results;
          results.open(result_file_);
          results << C_T_L;
          results.close();

          std::ofstream results_rpy;
          results_rpy.open(result_rpy_file_);
          results_rpy << Rotn.eulerAngles(0, 1, 2) * 180 / M_PI << "\n"
                      << C_T_L.block(0, 3, 3, 1);
          results_rpy.close();

          RCLCPP_INFO(this->get_logger(), "No of initialization: %d", counter);
        }
        init_file_stream_.close();
        rclcpp::shutdown();
      }
    }
    // else
    // {
    //   //RCLCPP_WARN(this->get_logger(), "Not enough Rotation, view not recorded");
    // }
  }
  // else
  // {
  //   if (!boardDetectedInCam_)
  //     //RCLCPP_WARN(this->get_logger(), "Checker-board not detected in Image.");
  //   //else
  //   {
  //     //RCLCPP_WARN(this->get_logger(), "Checker Board Detected in Image?: %d, No of LiDAR pts: %d, (Check if this is less than threshold)", boardDetectedInCam_, lidar_points_.size());
  //   }
  // }
}

void LidarCameraCalibNode::MsgsCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar,
    const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
  imageHandler(image);
  cloudHandler(lidar);
  runSolver();
}


// void LidarCameraCalibNode::onPointCloud(
//   const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
// {
//   RCLCPP_INFO(this->get_logger(), "Received point cloud message");
//   cloudHandler(input_msg);
// }

// void LidarCameraCalibNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
// {
//   RCLCPP_INFO(this->get_logger(), "Received image message");
//   imageHandler(msg);
// }

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(LidarCameraCalibNode)
