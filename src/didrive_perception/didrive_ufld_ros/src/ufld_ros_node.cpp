#include "ufld_ros/ufld_detector.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <chrono>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class UFLDRosNode : public rclcpp::Node
{
public:
    UFLDRosNode() : Node("ufld_detector")
    {
        // 声明所有在params.yaml中使用的参数
        this->declare_parameter<std::string>("engine_path", "");
        this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
        this->declare_parameter<std::string>("pointcloud_topic", "/lane_detection/points");
        this->declare_parameter<std::string>("visualization_path", "/tmp/lane_visualization.png");
        // this->declare_parameter<std::string>("visualization3d_path", "/tmp/lane_visualization3d.png");
        this->declare_parameter<std::string>("dataset", "CULane");

        this->declare_parameter<int>("target_width", 800);
        this->declare_parameter<int>("target_height", 288);
        this->declare_parameter<int>("griding_num", 200);
        this->declare_parameter<int>("num_lanes", 6);
        this->declare_parameter<int>("num_styles", 15);
        this->declare_parameter<double>("camera_height", 1.95);
        this->declare_parameter<double>("pitch_angle", 5.0);
        this->declare_parameter<double>("max_distance", 40.0);
        this->declare_parameter<int>("resample_points", 40);
        this->declare_parameter<bool>("save_vis_image", false);

        this->declare_parameter<std::vector<double>>("mean", std::vector<double>{0.416956, 0.419838, 0.402038});
        this->declare_parameter<std::vector<double>>("std", std::vector<double>{0.205873, 0.203590, 0.219311});
        this->declare_parameter<std::vector<int64_t>>("row_anchors", std::vector<int64_t>{121, 131, 141, 150, 160, 170, 180, 189, 199, 209, 219, 228, 238, 248, 258, 267, 277, 287});

        // 相机内参需要单独声明
        this->declare_parameter<std::vector<double>>("camera_intrinsics.K",
                                                     std::vector<double>{1907.3244009283, 0.0, 1919.4205556116, 0.0, 1906.8286514328, 1079.8599665278, 0.0, 0.0, 1.0});

        // 加载参数
        loadParameters();

        // 初始化检测器
        if (!initializeDetector())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize detector");
            rclcpp::shutdown();
            return;
        }

        // 创建订阅者和发布者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_, 10,
            std::bind(&UFLDRosNode::imageCallback, this, std::placeholders::_1));

        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10);
        RCLCPP_INFO(this->get_logger(), "UFLD ROS node initialized successfully");
    }

private:
    void loadParameters()
    {
        // 获取基本参数
        engine_path_ = this->get_parameter("engine_path").as_string();
        image_topic_ = this->get_parameter("image_topic").as_string();
        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        visualization_path_ = this->get_parameter("visualization_path").as_string();
        // visualization3d_path_ = this->get_parameter("visualization3d_path").as_string();
        dataset_ = this->get_parameter("dataset").as_string();
        save_vis_image_ = this->get_parameter("save_vis_image").as_bool();
        // 获取数值参数
        target_width_ = this->get_parameter("target_width").as_int();
        target_height_ = this->get_parameter("target_height").as_int();
        griding_num_ = this->get_parameter("griding_num").as_int();
        num_lanes_ = this->get_parameter("num_lanes").as_int();
        num_styles_ = this->get_parameter("num_styles").as_int();
        camera_height_ = this->get_parameter("camera_height").as_double();
        pitch_angle_ = this->get_parameter("pitch_angle").as_double();
        max_distance_ = this->get_parameter("max_distance").as_double();
        resample_points_ = this->get_parameter("resample_points").as_int();
        std::cout << "pitch_angle_:" << pitch_angle_ << std::endl;
        // 获取数组参数
        auto mean_param = this->get_parameter("mean").as_double_array();
        mean_.resize(mean_param.size());
        for (size_t i = 0; i < mean_param.size(); ++i)
        {
            mean_[i] = static_cast<float>(mean_param[i]);
        }

        auto std_param = this->get_parameter("std").as_double_array();
        std_.resize(std_param.size());
        for (size_t i = 0; i < std_param.size(); ++i)
        {
            std_[i] = static_cast<float>(std_param[i]);
        }

        auto row_anchors_param = this->get_parameter("row_anchors").as_integer_array();
        row_anchors_.resize(row_anchors_param.size());
        for (size_t i = 0; i < row_anchors_param.size(); ++i)
        {
            row_anchors_[i] = static_cast<int>(row_anchors_param[i]);
        }

        // 加载相机内参
        loadCameraIntrinsics();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded successfully");
    }

    void loadCameraIntrinsics()
    {
        try
        {
            // 直接从参数中获取相机内参
            auto K_param = this->get_parameter("camera_intrinsics.K").as_double_array();
            camera_K_.clear();
            RCLCPP_INFO(this->get_logger(), "Get camera intrinsics: ");
            for (const auto &value : K_param)
            {
                camera_K_.push_back(static_cast<float>(value));
                std::cout << value << ", " << std::endl;
            }
            RCLCPP_INFO(this->get_logger(), "Camera intrinsics loaded, size: %zu", camera_K_.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load camera intrinsics: %s", e.what());
            // 设置默认值
            camera_K_ = {574.32874025f, 0.0f, 960.18839962f,
                         0.0f, 838.288f, 539.807f,
                         0.0f, 0.0f, 1.0f};
        }
    }

    bool initializeDetector()
    {
        detector_ = std::make_unique<UFLDDetector>();
        return detector_->initialize(
            engine_path_, camera_K_, target_width_, target_height_,
            mean_, std_, griding_num_, num_lanes_, num_styles_,
            camera_height_, pitch_angle_, row_anchors_);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        auto total_start = std::chrono::high_resolution_clock::now();

        try
        {
            // 转换图像格式
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
            cv::Mat image = cv_ptr->image;
            // 处理图像
            auto result = detector_->processImage(image);

            // 发布点云
            publishPointCloud(msg->header, result);

            if (save_vis_image_)
            {
                // 保存可视化结果
                detector_->saveVisualization(image, result, visualization_path_);
            }
            // 保存3D可视化结果
            // detector_->save3DLanesPlot(image, result, visualization3d_path_);
            // 输出计时信息
            auto total_end = std::chrono::high_resolution_clock::now();
            auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start);

            // 在 imageCallback 函数中修改日志输出
            // RCLCPP_DEBUG(this->get_logger(),
            //              "Processing times - Preprocess: %.2fms, Inference: %.2fms, Postprocess: %.2fms, Total: %ldms",
            //              result.preprocess_time.count() * 1000,
            //              result.inference_time.count() * 1000,
            //              result.postprocess_time.count() * 1000,
            //              total_time.count()); // 使用 %ld 格式

            // RCLCPP_DEBUG(this->get_logger(), "Detected %zu lanes", result.lanes_3d.size());
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge error: %s", e.what());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Processing error: %s", e.what());
        }
    }

    void publishPointCloud(const std_msgs::msg::Header &header, const LaneDetectionResult &result)
    {
        auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

        // 设置点云头信息
        cloud_msg->header = header;
        //cloud_msg->header.frame_id = "base_link"; // 根据实际情况调整坐标系

        // 设置点云字段
        cloud_msg->height = 1;

        // 计算总点数
        size_t total_points = 0;
        for (const auto &lane : result.lanes_3d)
        {
            total_points += lane.size();
        }
        cloud_msg->width = total_points;

        // 设置字段
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2Fields(4,
                                      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "r", 1, sensor_msgs::msg::PointField::UINT8);

        // 填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
        float pitch_rad = pitch_angle_ * M_PI / 180.0f;
        const double cos_p = std::cos(pitch_rad);
        const double sin_p = std::sin(pitch_rad);
        for (size_t i = 0; i < result.lanes_3d.size(); ++i)
        {
            const auto &lane = result.lanes_3d[i];
            uint8_t lane_type = (i < result.lane_types.size()) ? result.lane_types[i] : 0;

            for (const auto &point : lane)
            {
                double x_g = point.x;
                double y_g = point.y;
                double z_g = point.z; // 地面点z=0，这里只是为了展示通用性

                // 应用 G -> C 变换
                double x_c = x_g; // X 不变
                double y_c = y_g * cos_p - z_g * sin_p;
                double z_c = y_g * sin_p + z_g * cos_p + camera_height_;
                // 更新点的位置

                *iter_x = point.x;
                *iter_y = point.y;
                *iter_z = point.z;
                *iter_r = lane_type;

                // *iter_x = x_g;
                // *iter_y = y_g + 0.3;
                // *iter_z = z_g - camera_height_;
                // *iter_r = lane_type;
                // RCLCPP_INFO(this->get_logger(), "x:%f", point.x);
                // RCLCPP_INFO(this->get_logger(), "y:%f", point.y);
                // RCLCPP_INFO(this->get_logger(), "z:%f", point.z);
                // RCLCPP_INFO(this->get_logger(), "type_id:%d", lane_type);
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_r;
            }
        }

        // 发布点云
        pointcloud_pub_->publish(std::move(cloud_msg));
        RCLCPP_INFO(this->get_logger(), "Published point cloud with %zu points", total_points);
    }

    // ROS2相关
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    // 检测器
    std::unique_ptr<UFLDDetector> detector_;

    // 参数
    std::string engine_path_;
    std::string camera_intrinsics_path_;
    std::string image_topic_;
    std::string pointcloud_topic_;
    std::string visualization_path_;
    // std::string visualization3d_path_;
    std::string dataset_;
    bool save_vis_image_;

    int target_width_, target_height_;
    int griding_num_, num_lanes_, num_styles_;
    int resample_points_;
    float camera_height_, pitch_angle_, max_distance_;
    std::vector<float> mean_, std_, camera_K_;
    std::vector<int> row_anchors_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<UFLDRosNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception in main: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
