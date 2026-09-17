#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
#include <deque>
#include <memory>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <mutex>
#include <thread>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>

namespace fs = std::filesystem;

// ================= 全局状态 (4路共用) =================
struct VehicleState {
    const char* mode = "UNKNOWN";
    const char* gear = "N";
    double speed = 0.0;
    double steer = 0.0;
    const char* turn = "NONE";
};

// 线程安全的全局状态
static VehicleState global_state;
static std::mutex state_mutex;

// 单文件 Page Cache 驱逐辅助函数
static void evict_single_file_cache(const char* filepath) {
    if (!filepath || filepath[0] == '\0') return;
    int fd = open(filepath, O_RDONLY);
    if (fd >= 0) {
        posix_fadvise(fd, 0, 0, POSIX_FADV_DONTNEED);
        close(fd);
    }
}

// 前置声明
class CameraRecorder;

// ================= GStreamer 总线同步回调函数声明 =================
static GstBusSyncReply on_bus_message_sync(GstBus* /*bus*/, GstMessage* message, gpointer user_data);


// ================= 单路录制类 =================
class CameraRecorder {
public:
    CameraRecorder(rclcpp::Node::SharedPtr node, int cam_id, const std::string& topic, 
                   const std::string& data_dir, int width, int height, size_t max_files = 5000)
        : node_(node), cam_id_(cam_id), width_(width), height_(height),
          max_files_(max_files), frame_count_(0), was_empty_image_(false),
          has_first_frame_(false), last_pts_(0)
    {
        // 1. 创建独立存储目录
        output_dir_ = fs::path(data_dir) / ("cam_" + std::to_string(cam_id));
        fs::create_directories(output_dir_);

        // 2. 初始化已有历史视频文件队列（按时间序排列，防止磁盘爆满）
        init_recorded_files();

        // 3. 预分配图像转换和 OSD 内存，消除每帧高频堆内存申请/释放
        osd_img_.create(height_, width_, CV_8UC3);
        img_yuv_.create(height_ * 3 / 2, width_, CV_8UC1);

        // 4. 构建并启动 GStreamer 管道
        build_and_start_pipeline();

        // 5. 为当前相机通道创建互斥回调组（注册在 node_ 上）
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = callback_group_;

        // 队列深度设为 10，提供平滑缓冲，杜绝因 CPU 瞬间繁忙引起的丢帧
        rclcpp::QoS qos_profile(10); 
        qos_profile.best_effort();

        sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            topic, qos_profile,
            std::bind(&CameraRecorder::image_callback, this, std::placeholders::_1),
            sub_options);

        RCLCPP_INFO(node_->get_logger(),
                    "Initialized Camera %d on %s (Timestamp Naming & Fragment Eviction Active)",
                    cam_id_, topic.c_str());
    }

    ~CameraRecorder() {
        // 先取消订阅，防止析构期间继续进入回调
        sub_.reset();

        if (pipeline_) {
            RCLCPP_INFO(node_->get_logger(), "Sending EOS to Camera %d pipeline...", cam_id_);
            if (appsrc_) {
                gst_app_src_end_of_stream(appsrc_);
            } else {
                gst_element_send_event(pipeline_, gst_event_new_eos());
            }

            // 等待总线 EOS 消息，最长 2 秒，确保尾部索引(moov atom)安全封包
            GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
            if (bus) {
                GstMessage* msg = gst_bus_timed_pop_filtered(
                    bus, 2 * GST_SECOND,
                    static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
                if (msg) {
                    gst_message_unref(msg);
                }
                gst_object_unref(bus);
            }

            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsrc_ = nullptr;
        }
    }

    // 格式化新切片文件名回调（GStreamer 内部会在创建每个新视频文件时触发）
    gchar* format_location(guint /*fragment_id*/) {
        auto now = std::chrono::system_clock::now();
        std::time_t in_time_t = std::chrono::system_clock::to_time_t(now);
        struct tm tm_buf;
        localtime_r(&in_time_t, &tm_buf);

        char time_str[32];
        // 命名格式如：2026_0910_145430
        std::strftime(time_str, sizeof(time_str), "%Y_%m%d_%H%M%S", &tm_buf);

        fs::path target_path = output_dir_ / (std::string(time_str) + ".mp4");

        // 如果同一秒已存在同名文件（如同秒快速重启），自动追加数字后缀避免覆盖
        if (fs::exists(target_path)) {
            int seq = 1;
            while (true) {
                fs::path alt_path = output_dir_ / (std::string(time_str) + "_" + std::to_string(seq) + ".mp4");
                if (!fs::exists(alt_path)) {
                    target_path = alt_path;
                    break;
                }
                seq++;
            }
        }

        // 核心修复：在新切片文件即将创建前，如果已有文件数达到或超过 max_files_，
        // 立即删除最旧的历史文件。确保磁盘上【已完成文件 + 正在写入文件】的总数在任何瞬间都不超过 max_files_
        if (max_files_ > 0) {
            std::lock_guard<std::mutex> lock(files_mutex_);
            while (recorded_files_.size() >= max_files_) {
                std::string oldest = recorded_files_.front();
                recorded_files_.pop_front();
                std::error_code ec;
                if (fs::remove(oldest, ec)) {
                    // RCLCPP_INFO(node_->get_logger(), "Camera %d rotated oldest file: %s",
                    //             cam_id_, oldest.c_str());
                }
            }
            recorded_files_.push_back(target_path.string());
        }

        RCLCPP_INFO(node_->get_logger(), "Camera %d recording started: %s",
                    cam_id_, target_path.filename().c_str());

        // GStreamer 负责释放返回的字符串内存
        return g_strdup(target_path.string().c_str());
    }

    // 切片完成闭合回调
    void on_fragment_closed(const char* /*filepath*/) {
        // Page Cache 驱逐在 on_bus_message_sync 中即时处理
    }

private:
    static gchar* on_format_location_static(GstElement* /*splitmux*/, guint fragment_id, gpointer user_data) {
        auto* recorder = static_cast<CameraRecorder*>(user_data);
        return recorder->format_location(fragment_id);
    }

    void init_recorded_files() {
        std::lock_guard<std::mutex> lock(files_mutex_);
        recorded_files_.clear();
        if (fs::exists(output_dir_)) {
            std::vector<std::string> existing_files;
            for (const auto& entry : fs::directory_iterator(output_dir_)) {
                if (entry.is_regular_file() && entry.path().extension() == ".mp4") {
                    existing_files.push_back(entry.path().string());
                }
            }
            // 时间戳命名 YYYY_MMDD_HHMMSS 在字典序下严格等于时间序
            std::sort(existing_files.begin(), existing_files.end());
            for (const auto& f : existing_files) {
                recorded_files_.push_back(f);
            }

            // 如果启动时已有文件数达到或超过上限，先清理至预留 1 个空位给即将启动的录制切片
            if (max_files_ > 0) {
                while (recorded_files_.size() >= max_files_) {
                    std::string oldest = recorded_files_.front();
                    recorded_files_.pop_front();
                    std::error_code ec;
                    if (fs::remove(oldest, ec)) {
                        RCLCPP_INFO(node_->get_logger(), "Camera %d cleaned existing file: %s",
                                    cam_id_, oldest.c_str());
                    }
                }
            }
        }
    }

    void build_and_start_pipeline() {
        std::string src_name = "src_" + std::to_string(cam_id_);
        std::string sink_name = "sink_" + std::to_string(cam_id_);

        std::stringstream ss;
        ss << "appsrc name=" << src_name << " is-live=true format=3 do-timestamp=false max-bytes=20000000 ! "
           << "video/x-raw, format=I420, width=" << width_ << ", height=" << height_ << ", framerate=10/1 ! "
           << "nvvidconv ! "
           << "video/x-raw(memory:NVMM), format=I420 ! "
           << "nvv4l2h265enc bitrate=3000000 insert-sps-pps=true ! "
           << "h265parse ! "
           << "splitmuxsink name=" << sink_name << " max-size-time=60000000000";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(ss.str().c_str(), &error);
        if (error) {
            RCLCPP_ERROR(node_->get_logger(), "GStreamer pipeline error: %s", error->message);
            g_error_free(error);
            return;
        }

        GstElement* appsrc_elem = gst_bin_get_by_name(GST_BIN(pipeline_), src_name.c_str());
        appsrc_ = GST_APP_SRC(appsrc_elem);
        gst_object_unref(appsrc_elem);

        // 绑定 format-location 动态命名信号
        GstElement* sink_elem = gst_bin_get_by_name(GST_BIN(pipeline_), sink_name.c_str());
        if (sink_elem) {
            g_signal_connect(sink_elem, "format-location", G_CALLBACK(on_format_location_static), this);
            gst_object_unref(sink_elem);
        }

        // 设置 GstBus 同步消息处理句柄捕获 splitmuxsink-fragment-closed 释放缓存并轮转文件
        GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));
        if (bus) {
            gst_bus_set_sync_handler(bus, on_bus_message_sync, this, nullptr);
            gst_object_unref(bus);
        }

        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            bool is_empty = (msg->data.empty() || msg->height == 0 || msg->width == 0);
            if (is_empty) {
                if (!was_empty_image_) {
                    RCLCPP_WARN(node_->get_logger(),
                                "Camera %d entered EMPTY image state (dropping frames until valid data arrives).",
                                cam_id_);
                    was_empty_image_ = true;
                }
                return;
            } else {
                if (was_empty_image_) {
                    RCLCPP_INFO(node_->get_logger(),
                                "Camera %d recovered from EMPTY image state (valid frames resumed).",
                                cam_id_);
                    was_empty_image_ = false;
                }
            }

            if (osd_img_.rows != height_ || osd_img_.cols != width_ || osd_img_.type() != CV_8UC3) {
                osd_img_.create(height_, width_, CV_8UC3);
            }

            // 根据相机真实图像编码高效转换为 BGR 格式（无需 cv_bridge，彻底杜绝 OpenCV ABI 冲突）
            if (msg->encoding == "yuv422" || msg->encoding == "uyvy") {
                cv::Mat raw_img(msg->height, msg->width, CV_8UC2, const_cast<uint8_t*>(msg->data.data()), msg->step);
                cv::cvtColor(raw_img, osd_img_, cv::COLOR_YUV2BGR_UYVY);
            } else if (msg->encoding == "yuv422_yuy2" || msg->encoding == "yuyv") {
                cv::Mat raw_img(msg->height, msg->width, CV_8UC2, const_cast<uint8_t*>(msg->data.data()), msg->step);
                cv::cvtColor(raw_img, osd_img_, cv::COLOR_YUV2BGR_YUYV);
            } else if (msg->encoding == "bgr8") {
                cv::Mat raw_img(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()), msg->step);
                raw_img.copyTo(osd_img_);
            } else if (msg->encoding == "rgb8") {
                cv::Mat raw_img(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(msg->data.data()), msg->step);
                cv::cvtColor(raw_img, osd_img_, cv::COLOR_RGB2BGR);
            } else {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "Camera %d unsupported image encoding: %s", cam_id_, msg->encoding.c_str());
                return;
            }

            // 线程安全的时间格式化
            auto now = std::chrono::system_clock::now();
            std::time_t in_time_t = std::chrono::system_clock::to_time_t(now);
            struct tm tm_buf;
            localtime_r(&in_time_t, &tm_buf);

            char time_str[32];
            std::strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);

            VehicleState state;
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                state = global_state;
            }

            // 栈缓冲区格式化 OSD 字符串，零堆内存分配
            char osd_text[160];
            std::snprintf(osd_text, sizeof(osd_text),
                          "CAM_%d | %s | %.0fkm/h | %s | %.1fdeg | %s | %s",
                          cam_id_, state.mode, state.speed,
                          state.gear, state.steer,
                          state.turn, time_str);

            // 高性能标准折线绘制 OSD，相比 LINE_AA 节省大量 CPU 算力
            cv::putText(osd_img_, osd_text, cv::Point(40, height_ - 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 255, 255), 2, cv::LINE_8);

            if (img_yuv_.rows != height_ * 3 / 2 || img_yuv_.cols != width_) {
                img_yuv_.create(height_ * 3 / 2, width_, CV_8UC1);
            }
            cv::cvtColor(osd_img_, img_yuv_, cv::COLOR_BGR2YUV_I420);
            size_t data_size = img_yuv_.total() * img_yuv_.elemSize();

            GstBuffer* buffer = gst_buffer_new_allocate(nullptr, data_size, nullptr);
            gst_buffer_fill(buffer, 0, img_yuv_.data, data_size);

            // 基于单调物理时钟计算绝对物理 PTS，彻底杜绝丢帧导致的视频快进与时长缩水
            auto frame_now = std::chrono::steady_clock::now();
            if (!has_first_frame_) {
                first_frame_time_ = frame_now;
                has_first_frame_ = true;
                last_pts_ = 0;
            }

            auto elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(frame_now - first_frame_time_).count();
            GstClockTime pts = static_cast<GstClockTime>(elapsed_ns);

            // 严格保障时间戳单调递增，防止极短时间内的微小抖动
            if (pts <= last_pts_ && frame_count_ > 0) {
                pts = last_pts_ + GST_MSECOND;
            }
            last_pts_ = pts;

            GstClockTime duration = GST_SECOND / 10;
            GST_BUFFER_PTS(buffer) = pts;
            GST_BUFFER_DTS(buffer) = pts;
            GST_BUFFER_DURATION(buffer) = duration;
            frame_count_++;

            // 原生 C API 直接推送 buffer，避免信号名称反射开销
            GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buffer);

            if (ret != GST_FLOW_OK) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "Cam %d failed to push buffer into appsrc: %d", cam_id_, ret);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Camera %d callback exception: %s", cam_id_, e.what());
        }
    }

    rclcpp::Node::SharedPtr node_;
    int cam_id_;
    int width_;
    int height_;
    size_t max_files_;
    uint64_t frame_count_;
    bool was_empty_image_ = false;
    bool has_first_frame_ = false;
    std::chrono::steady_clock::time_point first_frame_time_;
    GstClockTime last_pts_ = 0;
    fs::path output_dir_;

    cv::Mat osd_img_;
    cv::Mat img_yuv_;

    std::deque<std::string> recorded_files_;
    std::mutex files_mutex_;

    GstElement* pipeline_ = nullptr;
    GstAppSrc* appsrc_ = nullptr;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

// ================= GStreamer 总线同步回调 =================
static GstBusSyncReply on_bus_message_sync(GstBus* /*bus*/, GstMessage* message, gpointer user_data) {
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ELEMENT) {
        const GstStructure* s = gst_message_get_structure(message);
        if (s && gst_structure_has_name(s, "splitmuxsink-fragment-closed")) {
            const gchar* location = gst_structure_get_string(s, "location");
            if (location) {
                // 1. 精准驱逐刚落盘闭合的单个切片 Page Cache
                evict_single_file_cache(location);

                // 2. 通知当前相机通道更新切片文件队列并进行最旧文件轮转
                if (user_data) {
                    auto* recorder = static_cast<CameraRecorder*>(user_data);
                    recorder->on_fragment_closed(location);
                }
            }
        }
    }
    return GST_BUS_PASS;
}

// ================= 主控制节点 =================
class MultiRecorderNode : public rclcpp::Node {
public:
    MultiRecorderNode() : Node("multi_recorder_node") {
        this->declare_parameter<std::string>("data_dir", "/mnt/driving_recorder");
        this->declare_parameter<int>("max_files", 5000);

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

        clean_timer_ = this->create_wall_timer(
            std::chrono::seconds(60),
            std::bind(&MultiRecorderNode::periodic_clean_cb, this));
    }

    void set_recorders(std::vector<CameraRecorder*> recorders) {
        recorders_ = recorders;
    }

private:
    void periodic_clean_cb() {
        sync();
    }

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
        global_state.speed = msg->longitudinal_velocity * 3.6;
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

    std::vector<CameraRecorder*> recorders_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr mode_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr vel_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steer_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_sub_;
    rclcpp::TimerBase::SharedPtr clean_timer_;
};

// ================= 主函数 =================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    gst_init(&argc, &argv);

    auto main_node = std::make_shared<MultiRecorderNode>();
    std::string data_dir = main_node->get_parameter("data_dir").as_string();
    int max_files = main_node->get_parameter("max_files").as_int();

    struct CamConfig { int id; std::string topic; };
    std::vector<CamConfig> configs = {
        {0, "/camera0/image_raw"},
        {1, "/camera1/image_raw"},
        {2, "/camera2/image_raw"},
        {3, "/camera3/image_raw"}
    };

    std::vector<std::unique_ptr<CameraRecorder>> recorders;
    std::vector<CameraRecorder*> raw_recorders;

    // 所有 CameraRecorder 的 ROS 2 订阅直接挂载在 main_node 上
    for (const auto& cfg : configs) {
        auto rec = std::make_unique<CameraRecorder>(main_node, cfg.id, cfg.topic, data_dir, 1920, 1200, max_files);
        raw_recorders.push_back(rec.get());
        recorders.push_back(std::move(rec));
    }

    main_node->set_recorders(raw_recorders);

    auto now_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    struct tm tm_buf;
    localtime_r(&now_t, &tm_buf);
    char start_time_str[32];
    std::strftime(start_time_str, sizeof(start_time_str), "%Y-%m-%d %H:%M:%S", &tm_buf);

    RCLCPP_INFO(main_node->get_logger(),
                "[SUCCESS] [%s] Multi-channel recorder active. Storage: %s (Max %d files/cam)",
                start_time_str, data_dir.c_str(), max_files);
    
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 5);
    executor.add_node(main_node);
    executor.spin(); 

    recorders.clear(); 
    rclcpp::shutdown();
    return 0;
}