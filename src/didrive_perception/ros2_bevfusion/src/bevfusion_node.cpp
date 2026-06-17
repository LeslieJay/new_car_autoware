#include <cuda_runtime.h>
#include <dlfcn.h>
#include "ros2_bevfusion/bevfusion_node.hpp"
#include "ros2_bevfusion/utility.hpp"
#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#define PI 3.141592

namespace didrive::ros2bevfusion
{

  // 定义枚举类型
enum ObjectType : uint8_t {
    UNKNOWN = 0,
    CAR = 1,
    TRUCK = 2,
    BUS = 3,
    TRAILER = 4,
    MOTORCYCLE = 5,
    BICYCLE = 6,
    PEDESTRIAN = 7,
    ANIMAL = 8,
    HAZARD = 9,
    OVER_DRIVABLE = 10,
    UNDER_DRIVABLE = 11
};

// 创建从字符串到枚举值的映射
const std::unordered_map<int, ObjectType> CLASS_NAME_TO_ENUM = {
    {0, CAR},
    {1, TRUCK},
    {2, TRUCK},  // 映射到 TRUCK，可根据需求调整
    {3, BUS},
    {4, TRAILER},
    {5, UNKNOWN},              // barrier 视为潜在危险，映射到 HAZARD
    {6, MOTORCYCLE},
    {7, BICYCLE},
    {8, PEDESTRIAN},
    {9, UNKNOWN}          // 锥桶是典型危险物，映射到 HAZARD
};

inline bool isCarLikeVehicle(const uint8_t label)
{
  return label == ObjectType::BUS || label == ObjectType::CAR ||
         label == ObjectType::TRAILER || label == ObjectType::TRUCK;
}

NVBEVFusionNode::NVBEVFusionNode(const rclcpp::NodeOptions & options)
: Node("nv_bevfusion_node", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  // 初始化CUDA流
  checkRuntime(cudaStreamCreate(&stream_));

  // 声明并加载参数
  declare_parameters();
  load_parameters();

  // 加载自定义插件
  dlopen("libcustom_layernorm.so", RTLD_NOW);

  // 初始化BEVFusion核心
  initialize();

  // 创建订阅器 (1个激光雷达 + 6个相机)
  lidar_sub_.subscribe(this, "/sensing/lidar/rslidar_32/rslidar_points");

  image_subs_[0].subscribe(this, "/sensing/camera/camera0/image_rect_color");
  image_subs_[1].subscribe(this, "/sensing/camera/camera1/image_rect_color");
  image_subs_[2].subscribe(this, "/sensing/camera/camera2/image_rect_color");
  image_subs_[3].subscribe(this, "/sensing/camera/camera3/image_rect_color");
  image_subs_[4].subscribe(this, "/sensing/camera/camera4/image_rect_color");
  image_subs_[5].subscribe(this, "/sensing/camera/camera5/image_rect_color");

  // 创建时间同步器
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10),  // 队列大小
    lidar_sub_,
    image_subs_[0], image_subs_[1], image_subs_[2],
    image_subs_[3], image_subs_[4], image_subs_[5]
  );
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_time_diff_)); // 100ms 容差
  sync_->registerCallback(
    std::bind(
      &NVBEVFusionNode::sensor_callback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5, std::placeholders::_6, std::placeholders::_7
    )
  );

  // 创建检测结果发布器
  detection_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "/perception/object_recognition/detection/lidar_dnn/objects", 10
  );

  RCLCPP_INFO(this->get_logger(), "NVBEVFusionNode initialized successfully");
}

NVBEVFusionNode::~NVBEVFusionNode()
{
  if (bev_core_) {
    bev_core_.reset();
  }
  checkRuntime(cudaStreamDestroy(stream_));
  RCLCPP_INFO(this->get_logger(), "NVBEVFusionNode destroyed");
}

void NVBEVFusionNode::declare_parameters()
{
  this->declare_parameter<std::string>("model_path", "model/resnet50");
  this->declare_parameter<std::string>("precision", "fp16");
  this->declare_parameter<float>("confidence_threshold", 0.12f);
  this->declare_parameter<bool>("enable_timer", true);
  this->declare_parameter<float>("max_time_diff", 0.1f);

}

void NVBEVFusionNode::load_parameters()
{
  this->get_parameter("model_path", model_path_);
  this->get_parameter("precision", precision_);
  this->get_parameter("confidence_threshold", confidence_threshold_);
  this->get_parameter("enable_timer", enable_timer_);
  this->get_parameter("max_time_diff", max_time_diff_);


  RCLCPP_INFO_STREAM(this->get_logger(), "Model path: " << model_path_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Precision: " << precision_);
  RCLCPP_INFO_STREAM(this->get_logger(), "Confidence threshold: " << confidence_threshold_);
  RCLCPP_INFO_STREAM(this->get_logger(), "max time diff : " << max_time_diff_);

}

void NVBEVFusionNode::initialize()
{
  RCLCPP_INFO_STREAM(this->get_logger(), "Creating BEVFusion core with " << model_path_ << ", " << precision_);

  // 创建核心实例
  bev_core_ = create_core(model_path_, precision_);
  if (!bev_core_) {
    RCLCPP_FATAL(this->get_logger(), "Failed to create BEVFusion core instance");
    rclcpp::shutdown();
  }

  // 加载坐标变换矩阵 (实际应用中应从TF或参数服务器获取)
  std::vector<int> shape{1, 6, 4, 4};
  camera2lidar_ = nv::Tensor::from_data(camera2lidar_BN, shape,
                nv::DataType::Float32, false, stream_);
  camera_intrinsics_ = nv::Tensor::from_data(camera_intrinsics_BN, shape,
                nv::DataType::Float32, false, stream_);
  lidar2image_ = nv::Tensor::from_data(lidar2image_BN, shape,
                nv::DataType::Float32, false, stream_);
  img_aug_matrix_ = nv::Tensor::from_data(img_aug_BN, shape,
                nv::DataType::Float32, false, stream_);
  camera2lidar_.print();
  camera_intrinsics_.print();
  lidar2image_.print();
  img_aug_matrix_.print();

  // camera2lidar_ = nv::Tensor::load(nv::format("%s/camera2lidar.tensor", "example-data"), false);
  // camera_intrinsics_ = nv::Tensor::load(nv::format("%s/camera_intrinsics.tensor", "example-data"), false);
  // lidar2image_ = nv::Tensor::load(nv::format("%s/lidar2image.tensor", "example-data"), false);
  // img_aug_matrix_ = nv::Tensor::load(nv::format("%s/img_aug_matrix.tensor", "example-data"), false);

  // 更新变换矩阵
  bev_core_->update(
    camera2lidar_.ptr<float>(), 
    camera_intrinsics_.ptr<float>(), 
    lidar2image_.ptr<float>(), 
    img_aug_matrix_.ptr<float>(),
    stream_
  );

  // 启用计时器
  bev_core_->set_timer(enable_timer_);
  bev_core_->print();
}

std::shared_ptr<bevfusion::Core> NVBEVFusionNode::create_core(const std::string& model, const std::string& precision) {

  printf("Create by %s, %s\n", model.c_str(), precision.c_str());
  bevfusion::camera::NormalizationParameter normalization;
  normalization.image_width = 1920;
  normalization.image_height = 1080;
  normalization.output_width = 704;
  normalization.output_height = 256;
  normalization.num_camera = 6;
  normalization.resize_lim = 0.48f;
  normalization.interpolation = bevfusion::camera::Interpolation::Bilinear;

  float mean[3] = {0.485, 0.456, 0.406};
  float std[3] = {0.229, 0.224, 0.225};
  normalization.method = bevfusion::camera::NormMethod::mean_std(mean, std, 1 / 255.0f, 0.0f);

  bevfusion::lidar::VoxelizationParameter voxelization;
  voxelization.min_range = nvtype::Float3(-54.0f, -54.0f, -5.0);
  voxelization.max_range = nvtype::Float3(+54.0f, +54.0f, +3.0);
  voxelization.voxel_size = nvtype::Float3(0.075f, 0.075f, 0.2f);
  voxelization.grid_size =
      voxelization.compute_grid_size(voxelization.max_range, voxelization.min_range, voxelization.voxel_size);
  voxelization.max_points_per_voxel = 10;
  voxelization.max_points = 300000;
  voxelization.max_voxels = 160000;
  voxelization.num_feature = 5;

  bevfusion::lidar::SCNParameter scn;
  scn.voxelization = voxelization;
  scn.model = nv::format("%s/lidar.backbone.xyz.onnx", model.c_str());
  scn.order = bevfusion::lidar::CoordinateOrder::XYZ;

  if (precision == "int8") {
    scn.precision = bevfusion::lidar::Precision::Int8;
  } else {
    scn.precision = bevfusion::lidar::Precision::Float16;
  }

  bevfusion::camera::GeometryParameter geometry;
  geometry.xbound = nvtype::Float3(-54.0f, 54.0f, 0.3f);
  geometry.ybound = nvtype::Float3(-54.0f, 54.0f, 0.3f);
  geometry.zbound = nvtype::Float3(-10.0f, 10.0f, 20.0f);
  geometry.dbound = nvtype::Float3(1.0, 60.0f, 0.5f);
  geometry.image_width = 704;
  geometry.image_height = 256;
  geometry.feat_width = 88;
  geometry.feat_height = 32;
  geometry.num_camera = 6;
  geometry.geometry_dim = nvtype::Int3(360, 360, 80);

  bevfusion::head::transbbox::TransBBoxParameter transbbox;
  transbbox.out_size_factor = 8;
  transbbox.pc_range = {-54.0f, -54.0f};
  transbbox.post_center_range_start = {-61.2, -61.2, -10.0};
  transbbox.post_center_range_end = {61.2, 61.2, 10.0};
  transbbox.voxel_size = {0.075, 0.075};
  transbbox.model = nv::format("%s/build/head.bbox.plan", model.c_str());

  // if you got an inaccurate boundingbox result please turn on the layernormplugin plan.
  // transbbox.model = nv::format("model/%s/build/head.bbox.layernormplugin.plan", model.c_str());
  transbbox.confidence_threshold = confidence_threshold_;
  transbbox.sorted_bboxes = true;

  bevfusion::CoreParameter param;
  param.camera_model = nv::format("%s/build/camera.backbone.plan", model.c_str());
  param.normalize = normalization;
  param.lidar_scn = scn;
  param.geometry = geometry;
  param.transfusion = nv::format("%s/build/fuser.plan", model.c_str());
  param.transbbox = transbbox;
  param.camera_vtransform = nv::format("%s/build/camera.vtransform.plan", model.c_str());
  return bevfusion::create_core(param);
}

void NVBEVFusionNode::sensor_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & img0_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & img1_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & img2_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & img3_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & img4_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & img5_msg
)
{
  RCLCPP_DEBUG(this->get_logger(), "Received synchronized sensor data");

  try {
    // 转换点云数据
    auto lidar_tensor = convert_pointcloud(lidar_msg);

    // 转换图像数据
    std::vector<sensor_msgs::msg::Image::ConstSharedPtr> img_msgs = {
      img0_msg, img1_msg, img2_msg, img3_msg, img4_msg, img5_msg
    };
    auto images = convert_images(img_msgs);

    // 执行BEV融合推理
    auto bboxes = bev_core_->forward(
      (const unsigned char**)images.data(), 
      lidar_tensor.ptr<nvtype::half>(), 
      lidar_tensor.size(0), 
      stream_
    );

    // 发布检测结果
    publish_detections(lidar_msg->header, bboxes);

    // 释放图像缓冲区
    free_image_buffers(images);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error in sensor callback: %s", e.what());
  }
}

nv::Tensor NVBEVFusionNode::convert_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg)
{
  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*lidar_msg, pcl_pc2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ROI_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*lidar_msg, *ROI_cloud);

  half *points = new half[ROI_cloud->points.size() * 5];
  for (int i = 0; i < ROI_cloud->points.size(); i++)
  {
      points[i * 5 + 0] = __internal_float2half(ROI_cloud->points[i].x);
      points[i * 5 + 1] = __internal_float2half(ROI_cloud->points[i].y);
      points[i * 5 + 2] = __internal_float2half(ROI_cloud->points[i].z);
      points[i * 5 + 3] = __internal_float2half(ROI_cloud->points[i].intensity);
      points[i * 5 + 4] = __internal_float2half(0);
  }
  std::vector<int32_t> shape{ROI_cloud->points.size(), 5};
  auto lidar_half = nv::Tensor::from_data_reference(points, shape, nv::DataType::Float16, false);
  return lidar_half;
}

std::vector<unsigned char*> NVBEVFusionNode::convert_images(const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & img_msgs)
{
  std::vector<unsigned char*> images;
  images.reserve(6);

  for (const auto & img_msg : img_msgs) {
    // 转换ROS图像消息到OpenCV格式
    auto cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::RGB8);
    if (!cv_ptr) {
      throw std::runtime_error("Failed to convert image message");
    }

    // 分配GPU内存并复制数据
    unsigned char* img_data;
    size_t img_size = cv_ptr->image.rows * cv_ptr->image.cols * 3;
    checkRuntime(cudaMallocHost(&img_data, img_size));
    memcpy(img_data, cv_ptr->image.data, img_size);
    
    images.push_back(img_data);
  }

  return images;
}

void NVBEVFusionNode::publish_detections(const std_msgs::msg::Header & header, const std::vector<bevfusion::head::transbbox::BoundingBox> & bboxes)
{
  autoware_perception_msgs::msg::DetectedObjects output_msg, map_link_output_msg;
  output_msg.header = header;
  
  for (const auto & det : bboxes) {
    autoware_perception_msgs::msg::DetectedObject object;
    object.kinematics.pose_with_covariance.pose.position.x = det.position.x;
    object.kinematics.pose_with_covariance.pose.position.y = det.position.y;
    object.kinematics.pose_with_covariance.pose.position.z = det.position.z;
    float half_angle = (det.z_rotation + PI / 2 ) * 0.5f;
    float sin_half = sin(half_angle);
    float cos_half = cos(half_angle);
    object.kinematics.pose_with_covariance.pose.orientation.x = 0;
    object.kinematics.pose_with_covariance.pose.orientation.y = 0;
    object.kinematics.pose_with_covariance.pose.orientation.z = sin_half;
    object.kinematics.pose_with_covariance.pose.orientation.w = cos_half;

    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = det.size.l;
    object.shape.dimensions.y = det.size.w;
    object.shape.dimensions.z = det.size.h;
    object.existence_probability = det.score;
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = CLASS_NAME_TO_ENUM.at(det.id);
    if (isCarLikeVehicle(classification.label)) {
      object.kinematics.orientation_availability =
      autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
    }
    classification.probability = 1.0;
    object.classification.emplace_back(classification);
    output_msg.objects.push_back(object);
  }

   if (!autoware::object_recognition_utils::transformObjects(
          output_msg, "base_link", tf_buffer_,
          map_link_output_msg)) 
    return;

  detection_pub_->publish(map_link_output_msg);
}

void NVBEVFusionNode::free_image_buffers(std::vector<unsigned char*> & buffers)
{
  for (auto buf : buffers) {
    checkRuntime(cudaFreeHost(buf));
  }
  buffers.clear();
}

}  // namespace didrive::ros2bevfusion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(didrive::ros2bevfusion::NVBEVFusionNode)
