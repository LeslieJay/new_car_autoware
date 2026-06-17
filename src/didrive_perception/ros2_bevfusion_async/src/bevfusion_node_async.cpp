#include <cuda_runtime.h>
#include <dlfcn.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros2_bevfusion_async/bevfusion_node_async.hpp"

#define PI 3.141592

namespace didrive::ros2_bevfusion_async
{

  using Label = autoware_perception_msgs::msg::ObjectClassification;
  std::uint8_t getSemanticType(const std::string &class_name)
  {
    if (class_name == "CAR")
    {
      return Label::CAR;
    }
    else if (class_name == "TRUCK")
    {
      return Label::TRUCK;
    }
    else if (class_name == "BUS")
    {
      return Label::BUS;
    }
    else if (class_name == "TRAILER")
    {
      return Label::TRAILER;
    }
    else if (class_name == "MOTORCYCLE")
    {
      return Label::MOTORCYCLE;
    }
    else if (class_name == "BICYCLE")
    {
      return Label::BICYCLE;
    }
    else if (class_name == "PEDESTRIAN")
    {
      return Label::PEDESTRIAN;
    }
    else
    { // CONSTRUCTION_VEHICLE, BARRIER, TRAFFIC_CONE
      return Label::UNKNOWN;
    }
  }

  inline geometry_msgs::msg::Point createPoint(const double x, const double y, const double z)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  inline geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
  {
    auto yaw_ros = yaw;
    while (yaw_ros > PI)
      yaw_ros -= 2 * PI;
    while (yaw_ros < -PI)
      yaw_ros += 2 * PI;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_ros);
    return tf2::toMsg(q);
  }

  inline geometry_msgs::msg::Vector3 createTranslation(const double x, const double y, const double z)
  {
    geometry_msgs::msg::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
  }

  NVBEVFusionNodeAsync::NVBEVFusionNodeAsync(const rclcpp::NodeOptions &options)
      : Node("nv_bevfusion_node", options), tf_buffer_(this->get_clock())
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

    const auto allow_remapping_by_area_matrix = this->declare_parameter<std::vector<std::int64_t>>(
        "allow_remapping_by_area_matrix");
    const auto min_area_matrix =
        this->declare_parameter<std::vector<double>>("min_area_matrix");
    const auto max_area_matrix =
        this->declare_parameter<std::vector<double>>("max_area_matrix");
    detection_class_remapper_.setParameters(
        allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

    const auto reliable_qos = rclcpp::QoS(
                                  rclcpp::KeepLast(10))
                                  .reliable()
                                  .durability_volatile();

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    const auto qos = use_sensor_data_qos ? rclcpp::SensorDataQoS{}.keep_last(1) : reliable_qos;

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/input/pointcloud", qos,
        std::bind(&NVBEVFusionNodeAsync::cloudCallback, this, std::placeholders::_1));

    objects_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
        "/output/objects", rclcpp::QoS(1));

    if (sensor_fusion_)
    {
      image_subs_.resize(num_cameras_);
      camera_info_subs_.resize(num_cameras_);
      image_msgs_.resize(num_cameras_);
      camera_info_msgs_.resize(num_cameras_);
      lidar2camera_extrinsics_.resize(num_cameras_);

      for (std::int64_t camera_id = 0; camera_id < num_cameras_; ++camera_id)
      {
        image_subs_[camera_id] = this->create_subscription<sensor_msgs::msg::Image>(
            "/input/image" + std::to_string(camera_id), qos,
            [this, camera_id](const sensor_msgs::msg::Image::ConstSharedPtr msg)
            {
              this->imageCallback(msg, camera_id);
            });

        camera_info_subs_[camera_id] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/input/camera_info" + std::to_string(camera_id), qos,
            [this, camera_id](const sensor_msgs::msg::CameraInfo &msg)
            {
              this->cameraInfoCallback(msg, camera_id);
            });
      }
    }

    published_time_pub_ = std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);

    // initialize debug tool
    {
      using autoware::universe_utils::DebugPublisher;
      using autoware::universe_utils::StopWatch;
      stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
      debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
      stop_watch_ptr_->tic("cyclic");
      stop_watch_ptr_->tic("processing/total");
    }

    // 分配图像缓冲区
    allocateBuffers();

    RCLCPP_INFO(this->get_logger(), "NVBEVFusionNodeAsync initialized successfully");
  }

  NVBEVFusionNodeAsync::~NVBEVFusionNodeAsync()
  {
    if (bev_core_)
    {
      bev_core_.reset();
    }
    // 释放预分配的缓冲区
    for (auto buf : cpu_image_buffers_)
    {
      if (buf != nullptr)
      {
        checkRuntime(cudaFreeHost(buf));
      }
    }
    for (auto buf : gpu_image_buffers_)
    {
      if (buf != nullptr)
      {
        checkRuntime(cudaFree(buf));
      }
    }
    checkRuntime(cudaStreamDestroy(stream_));
    RCLCPP_INFO(this->get_logger(), "NVBEVFusionNodeAsync destroyed");
  }

  void NVBEVFusionNodeAsync::declare_parameters()
  {
    this->declare_parameter<std::string>("model_path", "model/resnet50");
    this->declare_parameter<std::string>("precision", "fp16");
    this->declare_parameter<float>("confidence_threshold", 0.12f);
    this->declare_parameter<bool>("enable_timer", true);

    this->declare_parameter<bool>("sensor_fusion", true);
    this->declare_parameter<float>("max_camera_lidar_delay", 0.12f);
    this->declare_parameter<int>("num_cameras", 6);
    this->declare_parameter<std::vector<std::string>>("class_names", {"CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"});
  }

  void NVBEVFusionNodeAsync::load_parameters()
  {
    this->get_parameter("model_path", model_path_);
    this->get_parameter("precision", precision_);
    this->get_parameter("confidence_threshold", confidence_threshold_);
    this->get_parameter("enable_timer", enable_timer_);

    this->get_parameter("sensor_fusion", sensor_fusion_);
    this->get_parameter("max_camera_lidar_delay", max_camera_lidar_delay_);
    this->get_parameter("num_cameras", num_cameras_);
    this->get_parameter("class_names", class_names_);

    RCLCPP_INFO_STREAM(this->get_logger(), "Model path: " << model_path_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Precision: " << precision_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Confidence threshold: " << confidence_threshold_);
  }

  void NVBEVFusionNodeAsync::initialize()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Creating BEVFusion core with " << model_path_ << ", " << precision_);

    // 创建核心实例
    bev_core_ = create_core(model_path_, precision_);
    if (!bev_core_)
    {
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

    intrinsics_extrinsics_precomputed_ = true;
    intrinsics_available_ = true;
    extrinsics_available_ = true;

    // 更新变换矩阵
    bev_core_->update(
        camera2lidar_.ptr<float>(),
        camera_intrinsics_.ptr<float>(),
        lidar2image_.ptr<float>(),
        img_aug_matrix_.ptr<float>(),
        stream_);

    // 启用计时器
    bev_core_->set_timer(enable_timer_);
    bev_core_->print();
  }

  // 缓冲区分配实现
  void NVBEVFusionNodeAsync::allocateBuffers()
  {
    if (buffers_allocated_ || !sensor_fusion_)
      return;

    // 分配CPU缓冲区和临时消息存储
    cpu_image_buffers_.resize(num_cameras_);
    temp_image_msgs_.resize(num_cameras_);
    camera_masks_.resize(num_cameras_);

    for (size_t i = 0; i < num_cameras_; ++i)
    {
      // 分配锁页内存
      checkRuntime(cudaMallocHost(&cpu_image_buffers_[i], IMG_DATA_SIZE));
      memset(cpu_image_buffers_[i], 0, IMG_DATA_SIZE); // 初始化为黑色图像
    }

    // 分配GPU缓冲区
    gpu_image_buffers_.resize(num_cameras_);
    for (size_t i = 0; i < num_cameras_; ++i)
    {
      checkRuntime(cudaMalloc(&gpu_image_buffers_[i], IMG_DATA_SIZE));
    }

    buffers_allocated_ = true; 
    RCLCPP_INFO_STREAM(this->get_logger(), "Allocated " << num_cameras_ << " image buffers ("
                                                        << (IMG_DATA_SIZE * num_cameras_ / 1024 / 1024) << " MB total)");
  }

  std::shared_ptr<bevfusion::Core> NVBEVFusionNodeAsync::create_core(const std::string &model, const std::string &precision)
  {

    printf("Create by %s, %s\n", model.c_str(), precision.c_str());
    bevfusion::camera::NormalizationParameter normalization;
    normalization.image_width = 1920;
    normalization.image_height = 1080;
    normalization.output_width = 704;
    normalization.output_height = 256;
    normalization.num_camera = 6;
    normalization.resize_lim = 0.4f;
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

    if (precision == "int8")
    {
      scn.precision = bevfusion::lidar::Precision::Int8;
    }
    else
    {
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

  void NVBEVFusionNodeAsync::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
  {
    lidar_frame_ = pc_msg->header.frame_id;
    lidar_stamp_ = rclcpp::Time(pc_msg->header.stamp).seconds();
    //if (sensor_fusion_ && (!extrinsics_available_ || !images_available_ || !intrinsics_available_))
    if (sensor_fusion_ && (!extrinsics_available_ || !intrinsics_available_))

    {
      return;
    }

    for (std::size_t i = 0; i < num_cameras_; ++i)
    {
      std::lock_guard<std::mutex> lock(image_mutexes_[i]); // 读取时加锁
      temp_image_msgs_[i] = image_msgs_[i];
      if (temp_image_msgs_[i] != nullptr)
      {
        double cam_stamp = rclcpp::Time(temp_image_msgs_[i]->header.stamp).seconds();
        if (std::abs(lidar_stamp_ - cam_stamp) >= max_camera_lidar_delay_)
        {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(),
              5000, // 每5秒最多警告一次
              "Camera-LiDAR timestamp mismatch: cam=%.6f, lidar=%.6f (diff > %.3f s)",
              cam_stamp, lidar_stamp_, max_camera_lidar_delay_);
        }
        camera_masks_[i] = (std::abs(lidar_stamp_ - cam_stamp) < max_camera_lidar_delay_) ? 1.0f : 0.0f;
      }
      else
      {
        camera_masks_[i] = 0.0f;
      }
    }

    // if (sensor_fusion_ && !intrinsics_extrinsics_precomputed_) {
    //   std::vector<sensor_msgs::msg::CameraInfo> camera_info_msgs;
    //   std::vector<Matrix4f> lidar2camera_extrinsics;

    //   std::transform(
    //     camera_info_msgs_.begin(), camera_info_msgs_.end(), std::back_inserter(camera_info_msgs),
    //     [](const auto & opt) { return *opt; });

    //   std::transform(
    //     lidar2camera_extrinsics_.begin(), lidar2camera_extrinsics_.end(),
    //     std::back_inserter(lidar2camera_extrinsics), [](const auto & opt) { return *opt; });

    //   detector_ptr_->setIntrinsicsExtrinsics(camera_info_msgs, lidar2camera_extrinsics);
    //   intrinsics_extrinsics_precomputed_ = true;
    // }

    const auto objects_sub_count =
        objects_pub_->get_subscription_count() + objects_pub_->get_intra_process_subscription_count();
    // if (objects_sub_count < 1) {
    //   return;
    // }

    if (stop_watch_ptr_)
    {
      stop_watch_ptr_->toc("processing/total", true);
    }

    // 转换点云数据
    auto lidar_half = convert_pointcloud(pc_msg);
    // 转换图像数据
    auto images = convert_images(temp_image_msgs_, camera_masks_);

    

    // 执行BEV融合推理

    auto det_boxes3d = bev_core_->forward(
        (const unsigned char **)images.data(),
        lidar_half.ptr<nvtype::half>(),
        lidar_half.size(0), stream_);
    auto end_time = std::chrono::high_resolution_clock::now();

    std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
    raw_objects.reserve(det_boxes3d.size());
    for (const auto &box3d : det_boxes3d)
    {
      autoware_perception_msgs::msg::DetectedObject obj;
      box3DToDetectedObject(box3d, class_names_, obj);
      raw_objects.emplace_back(obj);
    }

    autoware_perception_msgs::msg::DetectedObjects output_msg;
    output_msg.header = pc_msg->header;
    output_msg.objects = raw_objects;
    detection_class_remapper_.mapClasses(output_msg);

    if (objects_sub_count > 0)
    {
      objects_pub_->publish(output_msg);
      published_time_pub_->publish_if_subscribed(objects_pub_, output_msg.header.stamp);
    }
  }

  void NVBEVFusionNodeAsync::imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t camera_id)
  {
    // if (std::abs(lidar_stamp_ - rclcpp::Time(msg->header.stamp).seconds()) > max_camera_lidar_delay_) {
    //   return;
    // }
    {
      std::lock_guard<std::mutex> lock(image_mutexes_[camera_id]);
      image_msgs_[camera_id] = msg;
    }

    std::size_t num_valid_images = std::count_if(
        image_msgs_.begin(), image_msgs_.end(),
        [](const auto &image_msg)
        { return image_msg != nullptr; });
    images_available_ = num_valid_images == image_msgs_.size();
  }

  void NVBEVFusionNodeAsync::cameraInfoCallback(
      const sensor_msgs::msg::CameraInfo &msg, std::size_t camera_id)
  {
    camera_info_msgs_[camera_id] = msg;

    // std::size_t num_valid_intrinsics = std::count_if(
    //   camera_info_msgs_.begin(), camera_info_msgs_.end(),
    //   [](const auto & opt) { return opt.has_value(); });

    // intrinsics_available_ = num_valid_intrinsics == camera_info_msgs_.size();

    // if (
    //   lidar2camera_extrinsics_[camera_id].has_value() || !lidar_frame_.has_value() ||
    //   extrinsics_available_) {
    //   return;
    // }

    // try {
    //   geometry_msgs::msg::TransformStamped transform_stamped;
    //   transform_stamped =
    //     tf_buffer_.lookupTransform(msg.header.frame_id, *lidar_frame_, msg.header.stamp);

    //   Eigen::Matrix4f lidar2camera_transform =
    //     tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();

    //   Matrix4f lidar2camera_rowmajor_transform = lidar2camera_transform.eval();
    //   lidar2camera_extrinsics_[camera_id] = lidar2camera_rowmajor_transform;
    // } catch (tf2::TransformException & ex) {
    //   RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    //   return;
    // }

    // std::size_t num_valid_extrinsics = std::count_if(
    //   lidar2camera_extrinsics_.begin(), lidar2camera_extrinsics_.end(),
    //   [](const auto & opt) { return opt.has_value(); });

    // extrinsics_available_ = num_valid_extrinsics == lidar2camera_extrinsics_.size();
  }

  nv::Tensor NVBEVFusionNodeAsync::convert_pointcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &lidar_msg)
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

  std::vector<unsigned char *> NVBEVFusionNodeAsync::convert_images(
      const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> &img_msgs,
      const std::vector<float> &camera_masks)
  {
    if (!buffers_allocated_)
    {
      RCLCPP_ERROR(this->get_logger(), "Image buffers not allocated!");
      return {};
    }

    if (img_msgs.size() != num_cameras_ || camera_masks.size() != num_cameras_)
    {
      RCLCPP_ERROR(this->get_logger(), "Image or mask count mismatch!");
      return {};
    }

    // 处理每个相机的图像数据
    for (size_t i = 0; i < num_cameras_; ++i)
    {
      const auto &img_msg = img_msgs[i];

      // 根据掩码决定是否使用原始图像或黑色图像
      if (camera_masks[i] > 0.0f && img_msg)
      {
        if (img_msg->height != INFER_IMG_HEIGHT || img_msg->width != INFER_IMG_WIDTH)
        {
          RCLCPP_WARN(
              this->get_logger(),
              "Camera %zu image size mismatch! Expected %dx%d, got %dx%d. Using black image.",
              i, INFER_IMG_WIDTH, INFER_IMG_HEIGHT, img_msg->width, img_msg->height);
          memset(cpu_image_buffers_[i], 0, IMG_DATA_SIZE);
        }
        else
        {
          memcpy(cpu_image_buffers_[i], img_msg->data.data(), IMG_DATA_SIZE);
        }
      }
      else
      {
        // 使用黑色图像
        memset(cpu_image_buffers_[i], 0, IMG_DATA_SIZE);
        RCLCPP_DEBUG(this->get_logger(), "Camera %zu masked, using black image", i);
      }

      // 将CPU数据复制到GPU缓冲区（异步传输）
      // checkRuntime(cudaMemcpyAsync(
      //     gpu_image_buffers_[i],
      //     cpu_image_buffers_[i],
      //     IMG_DATA_SIZE,
      //     cudaMemcpyHostToDevice,
      //     stream_));
    }

    // checkRuntime(cudaStreamSynchronize(stream_));

    return cpu_image_buffers_;
  }

  void NVBEVFusionNodeAsync::box3DToDetectedObject(
      const bevfusion::head::transbbox::BoundingBox &box3d, const std::vector<std::string> &class_names,
      autoware_perception_msgs::msg::DetectedObject &obj)
  {
    obj.existence_probability = box3d.score;

    // classification
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.probability = 1.0f;
    if (box3d.id >= 0 && static_cast<std::size_t>(box3d.id) < class_names.size())
    {
      classification.label = getSemanticType(class_names[box3d.id]);
    }
    else
    {
      classification.label = Label::UNKNOWN;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("bevfusion"), "Unexpected label: UNKNOWN is set.");
    }

    if (autoware::object_recognition_utils::isCarLikeVehicle(classification.label))
    {
      obj.kinematics.orientation_availability =
          autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
    }

    obj.classification.emplace_back(classification);

    // pose and shape
    obj.kinematics.pose_with_covariance.pose.position =
        createPoint(box3d.position.x, box3d.position.y, box3d.position.z);
    obj.kinematics.pose_with_covariance.pose.orientation =
        createQuaternionFromYaw(box3d.z_rotation + PI / 2);
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions =
        createTranslation(box3d.size.l, box3d.size.w, box3d.size.h);
  }

  // void NVBEVFusionNodeAsync::publish_detections(const std::vector<bevfusion::head::transbbox::BoundingBox> & bboxes)
  // {
  //   vision_msgs::msg::Detection3DArray detection_msg;
  //   detection_msg.header.stamp = this->now();
  //   detection_msg.header.frame_id = "map";  // 根据实际坐标系修改

  //   for (const auto & det : bboxes) {
  //     vision_msgs::msg::Detection3D det3d;

  //     // 填充检测框位置信息
  //     det3d.bbox.center.position.x = det.position.x;
  //     det3d.bbox.center.position.y = det.position.y;
  //     det3d.bbox.center.position.z = det.position.z;

  //     // 填充检测框尺寸信息
  //     det3d.bbox.size.x = det.size.w;
  //     det3d.bbox.size.y = det.size.l;
  //     det3d.bbox.size.z = det.size.h;

  //     // 填充置信度和类别信息
  //     auto & score = det3d.results.emplace_back();
  //     score.hypothesis.score = det.score;
  //     score.hypothesis.class_id = std::to_string(det.id);
  //     detection_msg.detections.push_back(det3d);
  //   }

  //   detection_pub_->publish(detection_msg);
  //   RCLCPP_DEBUG_STREAM(this->get_logger(), "Published " << detection_msg.detections.size() << " detections");
  // }

  void NVBEVFusionNodeAsync::free_image_buffers(std::vector<unsigned char *> &buffers)
  {
    for (auto buf : buffers)
    {
      checkRuntime(cudaFreeHost(buf));
    }
    buffers.clear();
  }

} // namespace didrive::ros2_bevfusion_async

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(didrive::ros2_bevfusion_async::NVBEVFusionNodeAsync)
