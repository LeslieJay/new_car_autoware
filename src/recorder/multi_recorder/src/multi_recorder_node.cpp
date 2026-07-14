#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Autoware 车辆状态消息
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <filesystem>
#include <regex>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <thread>

// 第二版
namespace fs = std::filesystem;

// ================= 全局状态 (4路共用) =================
struct VehicleState {
    std::string mode = "UNKNOWN";
    std::string gear = "N";
    double speed = 0.0;
    double steer = 0.0;
    std::string turn = "NONE";
};

// 线程安全的全局状态
static VehicleState global_state;
static std::mutex state_mutex;

// ================= 单路录制类 =================
class CameraRecorder {
public:
    CameraRecorder(rclcpp::Node::SharedPtr node, int cam_id, const std::string& topic, 
                   const std::string& data_dir, int width, int height)
        : node_(node), cam_id_(cam_id), width_(width), height_(height), frame_count_(0) 
    {
        // 1. 创建独立存储目录
        output_dir_ = fs::path(data_dir) / ("cam_" + std::to_string(cam_id));
        fs::create_directories(output_dir_);

        // 2. 自动计算下一个保存的视频编号索引
        start_index_ = get_next_file_index();
        RCLCPP_INFO(node_->get_logger(), "Camera %d will resume video indexing from: %05d", cam_id_, start_index_);

        // 3. 构建并启动 GStreamer 管道
        build_and_start_pipeline();

        // ⭐ 4. 优化：为当前相机通道创建独立的、互斥的回调组，防止多路图像在默认队列中死锁
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;

        // 订阅该相机的图像
        // sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        //     topic, 10, std::bind(&CameraRecorder::image_callback, this, std::placeholders::_1), sub_options);

        // ⭐ 核心优化：自定义 QoS 策略
        // 1. 将队列深度设为 1（只保留最新的一帧，防止 10 帧排队占用 46MB 内存）
        rclcpp::QoS qos_profile(1); 
        // 2. 强制使用 BEST_EFFORT（允许丢帧，彻底根治 OOM）
        qos_profile.best_effort();

        // 订阅该相机的图像 (传入自定义的 qos_profile)
        sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            topic, qos_profile, std::bind(&CameraRecorder::image_callback, this, std::placeholders::_1), sub_options);

        RCLCPP_INFO(node_->get_logger(), "Initialized Camera %d on %s (Parallel Worker Allocated)", cam_id_, topic.c_str());
    }

    ~CameraRecorder() {
        if (pipeline_) {
            RCLCPP_INFO(node_->get_logger(), "Sending EOS to Camera %d pipeline...", cam_id_);
            gst_element_send_event(pipeline_, gst_event_new_eos());
            std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待写入完成
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
        }
    }

private:
    int get_next_file_index() {
        int max_idx = -1;
        std::regex pattern("video_(\\d{5})\\.mp4");

        if (fs::exists(output_dir_)) {
            for (const auto& entry : fs::directory_iterator(output_dir_)) {
                if (entry.is_regular_file()) {
                    std::string filename = entry.path().filename().string();
                    std::smatch match;
                    if (std::regex_search(filename, match, pattern)) {
                        int idx = std::stoi(match[1].str());
                        if (idx > max_idx) {
                            max_idx = idx;
                        }
                    }
                }
            }
        }
        return max_idx + 1;
    }

    void build_and_start_pipeline() {
        std::string src_name = "src_" + std::to_string(cam_id_);
        std::string sink_name = "sink_" + std::to_string(cam_id_);

        std::stringstream ss;
        ss << "appsrc name=" << src_name << " is-live=true format=3 do-timestamp=true ! "
           << "video/x-raw, format=I420, width=" << width_ << ", height=" << height_ << ", framerate=10/1 ! "
           << "nvvidconv ! "
           << "video/x-raw(memory:NVMM), format=I420 ! "
           << "nvv4l2h265enc bitrate=3000000 insert-sps-pps=true ! " // 码率微调至 3Mbps，超长跑更稳
           << "h265parse ! "
           << "splitmuxsink name=" << sink_name << " location=" << output_dir_.string() << "/video_%05d.mp4 "
           << "start-index=" << start_index_ << " max-size-time=60000000000 max-files=5000";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(ss.str().c_str(), &error);
        if (error) {
            RCLCPP_ERROR(node_->get_logger(), "GStreamer pipeline error: %s", error->message);
            g_error_free(error);
            return;
        }

        GstElement* appsrc_elem = gst_bin_get_by_name(GST_BIN(pipeline_), src_name.c_str());
        appsrc_ = GST_APP_SRC(appsrc_elem);
        gst_object_unref(appsrc_elem); // 释放 get 的引用

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 1. ROS 图像转 OpenCV 格式
            // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            // cv::Mat cv_img = cv_ptr->image;

            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
            cv::Mat cv_img = cv_ptr->image.clone(); // 仅在局部安全克隆，离开大括号后由 C++ 栈自动高效回收

            // 防御性空帧拦截
            if (cv_img.empty()) {
                RCLCPP_WARN(node_->get_logger(), "Camera %d received an EMPTY image message. Dropping.", cam_id_);
                return;
            }

            // 获取时间字符串
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss_time;
            ss_time << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");

            // 读取安全的全局车辆状态用于 OSD 叠加
            VehicleState state;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                state = global_state;
            }

            // 绘制 OSD 文本
            std::stringstream ss_osd;
            ss_osd << "CAM_" << cam_id_ << " | " << state.mode << " | " 
                   << std::fixed << std::setprecision(0) << state.speed << "km/h | " 
                   << state.gear << " | " << std::setprecision(1) << state.steer << "deg | " 
                   << state.turn << " | " << ss_time.str();

            cv::putText(cv_img, ss_osd.str(), cv::Point(40, height_ - 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

            // 2. CPU 端转为 I420 绕过硬件 Session 限制
            cv::Mat img_yuv;
            cv::cvtColor(cv_img, img_yuv, cv::COLOR_BGR2YUV_I420);
            size_t data_size = img_yuv.total() * img_yuv.elemSize();

            // 3. 封装 GStreamer Buffer 并推送
            GstBuffer* buffer = gst_buffer_new_allocate(nullptr, data_size, nullptr);
            gst_buffer_fill(buffer, 0, img_yuv.data, data_size);

            // 计算时间戳 (10Hz -> 每帧持续 100,000,000 纳秒)
            GstClockTime duration = GST_SECOND / 10;
            GST_BUFFER_PTS(buffer) = frame_count_ * duration;
            GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
            GST_BUFFER_DURATION(buffer) = duration;
            frame_count_++;

            // 推送到 appsrc
            GstFlowReturn ret;
            g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);

            if (ret != GST_FLOW_OK) {
                RCLCPP_WARN(node_->get_logger(), "Cam %d failed to push buffer into appsrc.", cam_id_);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Camera %d callback exception: %s", cam_id_, e.what());
        }
    }

    rclcpp::Node::SharedPtr node_;
    int cam_id_;
    int width_;
    int height_;
    uint64_t frame_count_;
    fs::path output_dir_;
    int start_index_;

    GstElement* pipeline_ = nullptr;
    GstAppSrc* appsrc_ = nullptr;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

// ================= 主控制节点 =================
class MultiRecorderNode : public rclcpp::Node {
public:
    MultiRecorderNode() : Node("multi_recorder_node") {
        // 订阅车端 Autoware 状态话题
        mode_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
            "/vehicle/status/control_mode", 10, std::bind(&MultiRecorderNode::mode_cb, this, std::placeholders::_1));
        
        gear_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::GearReport>(
            "/vehicle/status/gear_status", 10, std::bind(&MultiRecorderNode::gear_cb, this, std::placeholders::_1));

        vel_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10, std::bind(&MultiRecorderNode::vel_cb, this, std::placeholders::_1));

        steer_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
            "/vehicle/status/steering_status", 10, std::bind(&MultiRecorderNode::steer_cb, this, std::placeholders::_1));

        turn_sub_ = this->create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
            "/vehicle/status/turn_indicators_status", 10, std::bind(&MultiRecorderNode::turn_cb, this, std::placeholders::_1));
    }

private:
    void mode_cb(const autoware_vehicle_msgs::msg::ControlModeReport::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        switch (msg->mode) {
            case 1: global_state.mode = "AUTO"; break;
            case 2: global_state.mode = "AUTO_STEER"; break;
            case 3: global_state.mode = "AUTO_VEL"; break;
            case 4: global_state.mode = "MANUAL"; break;
            case 5: global_state.mode = "DISENGAGED"; break;
            case 6: global_state.mode = "NOT_READY"; break;
            default: global_state.mode = "UNKNOWN"; break;
        }
    }

    void gear_cb(const autoware_vehicle_msgs::msg::GearReport::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        switch (msg->report) {
            case 2:  global_state.gear = "D"; break;
            case 20: global_state.gear = "R"; break;
            case 22: global_state.gear = "P"; break;
            default: global_state.gear = "N"; break;
        }
    }

    void vel_cb(const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        global_state.speed = msg->longitudinal_velocity * 3.6; // m/s -> km/h
    }

    void steer_cb(const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        global_state.steer = msg->steering_tire_angle;
    }

    void turn_cb(const autoware_vehicle_msgs::msg::TurnIndicatorsReport::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex);
        switch (msg->report) {
            case 2:  global_state.turn = "LEFT"; break;
            case 3:  global_state.turn = "RIGHT"; break;
            default: global_state.turn = "NONE"; break;
        }
    }

    rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr mode_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vel_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steer_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_sub_;
};

// ================= 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    gst_init(&argc, &argv);

    auto main_node = std::make_shared<MultiRecorderNode>();
    std::string data_dir = "/mnt/driving_recorder";

    // 4路相机配置话题
    struct CamConfig { int id; std::string topic; };
    std::vector<CamConfig> configs = {
        {0, "/camera0/image_raw"},
        {1, "/camera1/image_raw"},
        {2, "/camera2/image_raw"},
        {3, "/camera3/image_raw"}
    };

    // 初始化 4 路独立带线程隔离的 Recorder
    std::vector<std::unique_ptr<CameraRecorder>> recorders;
    for (const auto& cfg : configs) {
        recorders.push_back(std::make_unique<CameraRecorder>(main_node, cfg.id, cfg.topic, data_dir, 1920, 1200));
    }

    RCLCPP_INFO(main_node->get_logger(), "[SUCCESS] 4-channel C++ MultiThreaded parallel recorder is running.");
    
    // ⭐ 终极优化：采用 4 线程物理并行调度器，彻底干掉多路并发排队导致的瞬时过载
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(main_node);
    executor.spin(); 

    // 干净利落释放资源
    recorders.clear(); 
    rclcpp::shutdown();
    return 0;
}