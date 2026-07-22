// src/can_receiver.cpp

#include "can_driver/can_receiver.hpp"
#include "can_driver/can_node.hpp"
#include "can_driver/trailer.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <linux/can/raw.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
inline int16_t to_int16(const uint8_t low, const uint8_t high)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

inline can_frame make_frame(const canid_t id)
{
    can_frame frame{};
    frame.can_id = id;
    frame.can_dlc = 8;
    return frame;
}
}

namespace can_driver
{

    CanReceiver::CanReceiver(std::shared_ptr<rclcpp::Node> node): node_(node), running_(false)
    {
        // 实例通过调用->publish方法发布数据
        steering_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
        velocity_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
        control_mode_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
        gear_report_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::GearReport>(
          "/vehicle/status/gear_status", 10);
        //rtk_NavSatFix_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/sensing/gnss/rtk/nav_sat_fix", 10);
        //rtk_GnssInsOrientationStamped_publisher_ = node_->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>("/autoware_orientation", 10);

        control_subscript_ = node_->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 1, std::bind(&CanReceiver::control_cmd_callback, this, _1));
        agv_state_subscript_ = node_->create_subscription<autoware_system_msgs::msg::AutowareState>(
            "/byd/autoware/state", 1, std::bind(&CanReceiver::agv_state_callback, this, _1));
        battery_publisher_ = node_->create_publisher<ref_slam_interface::msg::BatteryState>("/battery", 10);
        error_publisher_ = node_->create_publisher<vda5050_interfaces::msg::Error>("/error", 10);
        control_cmd_debug_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/can_driver/debug/control_cmd_rx", 10);
        can_cmd_debug_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/can_driver/debug/control_cmd_can", 10);

        gear_subscript_ = node_->create_subscription<autoware_vehicle_msgs::msg::GearCommand>("/control/command/gear_cmd", 1, std::bind(&CanReceiver::gear_cmd_callback, this, _1));
        person_instance_subscript_ = node_->create_subscription<autoware_control_msgs::msg::SafetyState>("/perception/pedestrian_distance_judge/state", 1, std::bind(&CanReceiver::person_instance_callback, this, _1));
        car_instance_subscript_ = node_->create_subscription<autoware_control_msgs::msg::SafetyState>("/perception/vehicel_distance_judge/state", 1, std::bind(&CanReceiver::car_instance_callback, this, _1));
 
        // // 该话题用来汇报转向信息，但是在模拟时，左转时标志位不变，直接使用control_cmd播报转向信息
        // turn_cmd_subscript_ = node_->create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
        //     "/control/command/turn_indicators_cmd", 1, std::bind(&CanReceiver::turn_cmd_callback, this, _1));

        // 创建engage客户端
        engage_client_ = node_->create_client<tier4_external_api_msgs::srv::Engage>("/api/autoware/set/engage");

        if (!engage_client_->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_WARN(node_->get_logger(), "Engage服务暂时不可用");
        }

        // 键盘控制
        // keyboard_subscript_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        //     "cmd_vel", 1, std::bind(&CanReceiver::keyboard_cmd_callback, this, _1));


        node_->declare_parameter("speed_upper_bound", rclcpp::ParameterValue(4200));
        node_->get_parameter("speed_upper_bound", speed_upper_bound_);

        node_->declare_parameter("wheel_distance", 0.5);
        node_->get_parameter("wheel_distance", wheel_distance_);

        node_->declare_parameter("engage_service_wait_timeout", 2.0); // 等待服务可用的超时时间
        node_->declare_parameter("engage_service_call_timeout", 5.0); // 调用服务响应的超时时间
        node_->declare_parameter("voice_frame_period_ms", 200);
        node_->declare_parameter("engage_frame_period_ms", 500);

        node_->get_parameter("engage_service_wait_timeout", engage_srv_wait_timeout_);
        node_->get_parameter("engage_service_call_timeout", engage_srv_call_timeout_);
        node_->get_parameter("voice_frame_period_ms", voice_frame_period_ms_);
        node_->get_parameter("engage_frame_period_ms", engage_frame_period_ms_);
        voice_frame_period_ms_ = std::max(voice_frame_period_ms_, 0);
        engage_frame_period_ms_ = std::max(engage_frame_period_ms_, 0);
        sendSafetyFrameCallback();
        // 创建安全定时器，之后无需任何调用即可定期执行sendSafetyFrameCallback
        safety_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(3000),
            std::bind(&CanReceiver::sendSafetyFrameCallback, this)
        );
        CreateSafetyFrame();
        publishGearStatus(current_gear_report_);
        last_control_time_ = node_->get_clock()->now();  // 新增
        last_voice_frame_time_ = last_control_time_;
        last_engage_frame_time_ = last_control_time_;
        openLogFile();
        initRecording();
    }

    CanReceiver::~CanReceiver()
    {
        // 停止写线程
        record_running_ = false;
        record_cv_.notify_one();
        if (record_write_thread_.joinable()) {
            record_write_thread_.join();
        }
        if (record_file_.is_open()) {
            record_file_.close();
        }
        // 确保剩余日志写入
        if (!log_buffer_.empty()) {
            flushLogBuffer();
        }
        if (log_file_.is_open()) {
            log_file_.close();
        }
        // stop(); // 确保线程停止
        // 注意：socket 的关闭由调用者负责！
        running_ = false;
    }

    bool CanReceiver::isRunning() const
    {
        return running_;
    }
    // 打开日志文件（仅一次），并统计已有行数
    void CanReceiver::openLogFile() {
        // 先以追加模式打开（若不存在则创建）
        log_file_.open(log_file_path_, std::ios::app);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "无法打开日志文件: %s", log_file_path_.c_str());
            return;
        }

        // 统计已存在行数（用临时 ifstream 单独计数）
        std::ifstream ifs(log_file_path_);
        if (ifs.is_open()) {
            log_line_count_ = std::count(std::istreambuf_iterator<char>(ifs),
                                        std::istreambuf_iterator<char>(), '\n');
            ifs.close();
        }
        // 若文件为空或只有一行，确保换行
        if (log_line_count_ == 0) {
            // 写入 CSV 头部（可选）
            log_file_ << "timestamp,velocity_cmd,steer_angle_cmd,speed_can_cmd,angle_can_cmd\n";
            log_file_.flush();
            log_line_count_ = 1; // 表头行也计入
        }
    }

    // 截断文件：保留后 MAX_LOG_LINES/2 行
    void CanReceiver::truncateLogFile() {
        log_file_.close(); // 先关闭当前流

        // 读取所有行
        std::ifstream ifs(log_file_path_);
        std::vector<std::string> lines;
        std::string line;
        while (std::getline(ifs, line)) {
            lines.push_back(line);
        }
        ifs.close();

        // 保留后一半
        size_t keep = MAX_LOG_LINES / 2;
        if (lines.size() > keep) {
            lines.erase(lines.begin(), lines.begin() + (lines.size() - keep));
        }

        // 覆盖写入
        std::ofstream ofs(log_file_path_, std::ios::trunc);
        for (const auto& l : lines) {
            ofs << l << '\n';
        }
        ofs.close();

        // 重新以追加模式打开，更新行计数
        log_file_.open(log_file_path_, std::ios::app);
        log_line_count_ = lines.size();
    }

    // 刷新缓冲区：写入文件，必要时截断
    void CanReceiver::flushLogBuffer() {
        if (log_buffer_.empty() || !log_file_.is_open()) return;

        // 预估写入后总行数，若超过上限则先截断
        if (log_line_count_ + static_cast<int>(log_buffer_.size()) > MAX_LOG_LINES) {
            truncateLogFile();
        }

        // 批量写入
        for (const auto& entry : log_buffer_) {
            log_file_ << entry << '\n';
        }
        log_file_.flush(); // 确保落盘
        log_line_count_ += log_buffer_.size();
        log_buffer_.clear();
    }
    // 计算拖拽挂车的最大转向角
    double calculateTrailerAngle(int trailer_num, double l, double a, double b, double l1) {
        // 参数校验
        if (l <= 0 || a <= 0 || b <= 0 || l1 <= 0) {
            throw std::invalid_argument("所有长度参数必须大于0");
        }
        
        // 计算根号内的值
        double inner_value = trailer_num * (l * l - a * a);
        
        if (inner_value < 0) {
            throw std::invalid_argument("计算错误：l² - a² 不能为负数");
        }
        
        // 计算分子：sqrt(trailer_num * (l² - a²)) + b/2
        double numerator = std::sqrt(inner_value) + b / 2.0;
        
        // 计算分母：l1
        double denominator = l1;
        
        if (denominator == 0) {
            throw std::invalid_argument("牵引车轴距l1不能为0");
        }
        
        // 计算y值
        double y = numerator / denominator;
        
        // 计算 arccot(y) = atan(1/y)
        if (y == 0) {
            throw std::invalid_argument("计算错误：y值为0，1/y无定义");
        }
        
        // 使用atan计算arccot
        double x_rad = std::atan(1.0 / y);
        
        return x_rad;
    }
    void CanReceiver::agv_state_callback(const autoware_system_msgs::msg::AutowareState::ConstSharedPtr msg){
        const auto now = node_->get_clock()->now();
        std::vector<struct can_frame> send_frames;
        struct can_frame v_frame{};
        // 语音播报
        v_frame.can_id = 0x401;
        v_frame.can_dlc = 8;
        v_frame.data[0] = 0;
        v_frame.data[1] = 0;
        v_frame.data[2] = 0;
        v_frame.data[3] = 0;
        v_frame.data[4] = 0;
        v_frame.data[5] = 0;
        v_frame.data[6] = 0;
        v_frame.data[7] = 0;
        if (msg->state == 6){
            // 第4位设为1,打开语音播报
            v_frame.data[0] |= (1 << 4);
            // 语音到达目标点
            v_frame.data[2] = 5;
        }
        const bool should_send_voice =
          (now - last_voice_frame_time_).nanoseconds() >=
          static_cast<int64_t>(voice_frame_period_ms_) * 1000000LL;
        if (should_send_voice) {
            send_frames.push_back(v_frame);
            last_voice_frame_time_ = now;
        }
        // 添加到队列
        if (!send_frames.empty()) {
            send_queue_->push(send_frames);
        }
        // ULONG result = VCI_Transmit(
        //     gDevType, 
        //     gDevIdx, 
        //     0,                      // 0=CAN0
        //     &send_frames[0],     // 【关键】直接传入 vector 的数据指针，等价于 
        //     static_cast<UINT>(send_frames.size()) // 发送帧的数量
        // );
        
        // // 3. 检查返回值
        // if (result == 1) {
        //     RCLCPP_INFO(node_->get_logger(), "Publishing到站语音...  ");
        // } else {
        //     RCLCPP_INFO(node_->get_logger(), "Publishing到站语音失败...  ");
        //     // // 1. 定义一个错误信息结构体变量
        //     // VCI_ERR_INFO errInfo;
        //     // // 可选：调用前先将结构体清零
        //     // memset(&errInfo, 0, sizeof(VCI_ERR_INFO));

        //     // // 2. 立即调用 VCI_ReadErrInfo 获取详细错误
        //     // if (VCI_ReadErrInfo(gDevType, gDevIdx, 0, &errInfo) == 1) {
        //     //     // 成功获取错误信息，解析 errInfo 中的 ErrCode
        //     //     RCLCPP_ERROR(node_->get_logger(), 
        //     //         "VCI_Transmit 失败，错误码: 0x%08X", errInfo.ErrCode);
                
        //     //     // 可以在此处添加更详细的错误码分析逻辑
        //     //     // switch (errInfo.ErrCode) { ... }

        //     // } else {
        //     //     RCLCPP_ERROR(node_->get_logger(), "无法读取 VCI_Transmit 的错误信息。");
        //     // }
        // }
        
    }
    
    /**
     * @brief 设置 CAN 过滤器，只接收指定 ID 的报文
     *
     * @param socket CAN raw socket 文件描述符
     * @param accept_ids 允许接收的 CAN ID 列表
     * @param is_extended 是否为扩展帧（29位 ID）
     * @return true 设置成功
     * @return false 设置失败
     */
    bool CanReceiver::setCanFilter(int socket, const std::vector<uint32_t> &accept_ids, bool is_extended)
    {
        RCLCPP_INFO(node_->get_logger(), " Attempting to set CAN filter on socket %d", socket);

        if (accept_ids.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "CAN filter: no IDs specified → accept all frames");
            return true;
        }

        std::vector<struct can_filter> filters;
        uint32_t mask = is_extended ? CAN_EFF_MASK : CAN_SFF_MASK;
        canid_t flag = is_extended ? CAN_EFF_FLAG : 0;

        RCLCPP_INFO(node_->get_logger(), "Setting filter for %zu %s frame(s):",
                    accept_ids.size(), is_extended ? "extended" : "standard");

        for (uint32_t id : accept_ids)
        {
            struct can_filter filter;
            // id & mask按位与运算，清除ID中超出范围的位，标准帧保留11位，扩展帧保留29位（除了这些位，前面全是0）
            // & 按位与：对应位都是1，结果才是1；否则是0   | 按位或，只要有一个1就为1
            // 与flag做或运算，来保证作为标志位的第31位一定为1
            filter.can_id = (id & mask) | flag;
            filter.can_mask = mask | CAN_EFF_FLAG; // 匹配 ID 和 帧类型
            filters.push_back(filter);

            RCLCPP_INFO(node_->get_logger(), "  Filter: ID=0x%X, Mask=0x%X, Extended=%s",
                        filter.can_id, filter.can_mask, is_extended ? "yes" : "no");
        }

        // 设置过滤socket，只要运行了后面的setsockopt(，以后所有通过这个套接字的数据都会被该规则过滤
        int result = setsockopt(socket, SOL_CAN_RAW, CAN_RAW_FILTER,
                                filters.data(), filters.size() * sizeof(struct can_filter));

        if (result < 0)
        {
            // 🔥 关键：用 RCLCPP_ERROR + strerror(errno)
            RCLCPP_ERROR(node_->get_logger(), " setsockopt(CAN_RAW_FILTER) failed: %s (errno=%d)",
                         std::strerror(errno), errno);
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), " CAN filter applied successfully for %zu IDs.", filters.size());
        return true;
    }
    void CanReceiver::person_instance_callback(const autoware_control_msgs::msg::SafetyState::ConstSharedPtr msg){
        std::vector<struct can_frame> send_frames;
        struct can_frame v_frame{};
        // 语音播报，灯光控制
        v_frame.can_id = 0x401;
        v_frame.can_dlc = 8;
        v_frame.data[0] = 0;
        v_frame.data[1] = 0;
        v_frame.data[2] = 0;
        v_frame.data[3] = 0;
        v_frame.data[4] = 0;
        v_frame.data[5] = 0;
        v_frame.data[6] = 0;
        v_frame.data[7] = 0;
        // 第4位设为1,打开语音播报
        v_frame.data[0] |= (1 << 4);

        // 检测到行人，速度降为0
        int16_t speed_command = 0;
        struct can_frame frame{};

        // 发送控制can指令
        frame.can_id = 0x201;
        frame.can_dlc = 8;
        frame.data[0] = 0;
        frame.data[1] = 0;
        frame.data[2] = 0;
        frame.data[3] = 0;
        frame.data[4] = 0b00001001; // 抱闸
        frame.data[5] = 0;
        frame.data[6] = 0;  
        frame.data[7] = 0;
        // 行人靠近，急停
        if (msg->current_state == 2){
            frame.data[4] = 0b00000001; // 抱闸
            v_frame.data[2] = 9;
        }
        send_frames.push_back(v_frame);
        send_frames.push_back(frame);
        send_queue_->push(send_frames);
        // ULONG result = VCI_Transmit(
        //     gDevType, 
        //     gDevIdx, 
        //     0,                      // 0=CAN0
        //     &send_frames[0],     // 【关键】直接传入 vector 的数据指针，等价于 
        //     static_cast<UINT>(send_frames.size()) // 发送帧的数量
        // );
        
        // // 3. 检查返回值
        // if (result == 1) {
        //     RCLCPP_INFO(node_->get_logger(), "Publishing行人帧...  ");
        // } else {
        //     RCLCPP_INFO(node_->get_logger(), "Publishing行人帧失败...  ");
        // }
        // // send_queue_->push(send_frames);
    } 
    void CanReceiver::car_instance_callback(const autoware_control_msgs::msg::SafetyState::ConstSharedPtr msg){
        RCLCPP_INFO(node_->get_logger(), "接到车辆话题...  ");
        std::vector<struct can_frame> send_frames;
        struct can_frame v_frame{};
        // 语音播报，灯光控制
        v_frame.can_id = 0x401;
        v_frame.can_dlc = 8;
        v_frame.data[0] = 0;
        v_frame.data[1] = 0;
        v_frame.data[2] = 0;
        v_frame.data[3] = 0;
        v_frame.data[4] = 0;
        v_frame.data[5] = 0;
        v_frame.data[6] = 0;
        v_frame.data[7] = 0;
        // 第4位设为1,打开语音播报
        v_frame.data[0] |= (1 << 4);
        // 车辆靠近报警
        if (msg->current_state == 2){
            v_frame.data[2] = 10;
        }
         RCLCPP_INFO(node_->get_logger(), "接到车辆话题...  %d", msg->current_state);
        send_frames.push_back(v_frame);
        send_queue_->push(send_frames);
    } 
    /**
     * @brief 接受报文
     *
     * @param handle socket返回的文件描述符
     * @param interface_name can接口名
     */

void CanReceiver::initRecording() {
    // 1. 检查文件并裁剪
    std::ifstream in_file(record_file_path_);
    std::vector<std::string> lines;
    if (in_file.is_open()) {
        std::string line;
        while (std::getline(in_file, line)) {
            lines.push_back(line);
        }
        in_file.close();

        if (lines.size() > 200000) {
            size_t keep_start = lines.size() / 2;
            lines.erase(lines.begin(), lines.begin() + keep_start);
            RCLCPP_WARN(node_->get_logger(),
                        "0x181 record file trimmed: %zu -> %zu lines",
                        lines.size() + keep_start, lines.size());
        }

        std::ofstream out_file(record_file_path_, std::ios::trunc);
        if (out_file.is_open()) {
            for (const auto &l : lines) {
                out_file << l << '\n';
            }
            out_file.close();
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to write trimmed 0x181 file");
        }
    }

    // 2. 以追加模式打开
    record_file_.open(record_file_path_, std::ios::app);
    if (!record_file_.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open 0x181 record file: %s",
                     record_file_path_.c_str());
    }

    // 3. 启动写线程
    record_running_ = true;
    record_write_thread_ = std::thread(&CanReceiver::writeRecordToFile, this);
}

void CanReceiver::writeRecordToFile() {
    while (record_running_) {
        std::unique_lock<std::mutex> lock(record_queue_mutex_);
        record_cv_.wait(lock, [this]() {
            return !record_queue_.empty() || !record_running_;
        });

        std::queue<std::string> batch;
        batch.swap(record_queue_);
        lock.unlock();

        if (record_file_.is_open()) {
            while (!batch.empty()) {
                record_file_ << batch.front() << std::endl;
                batch.pop();
            }
            record_file_.flush();
        }
    }
}
void CanReceiver::pushRecord(const can_frame &frame, double angle, double speed) {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;

    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%F %T")
       << '.' << std::setfill('0') << std::setw(3) << ms.count() << ','
       << std::hex << "0x" << frame.can_id << ','
       << std::dec << (int)frame.can_dlc << ',';
    for (int i = 0; i < frame.can_dlc; ++i) {
        ss << std::hex << (int)frame.data[i];
        if (i < frame.can_dlc - 1) ss << ' ';
    }
    ss << std::dec << ','
       << std::fixed << std::setprecision(3) << angle << ','
       << std::fixed << std::setprecision(3) << speed;

    {
        std::lock_guard<std::mutex> lock(record_queue_mutex_);
        record_queue_.push(ss.str());
    }
    record_cv_.notify_one();
}
    void CanReceiver::receiveTask(int handle, const std::string &interface_name)
    {
        // 线程运行标志，用于安全停止线程，该监听在独立的线程中无限循环，直到程序停止
        running_ = true;

        RCLCPP_INFO(node_->get_logger(),
                    "Starting receive thread for %s", interface_name.c_str());

        /* 让 socket 支持被信号唤醒（可选，但无害）*/
        int flags = fcntl(handle, F_GETFL, 0);
        fcntl(handle, F_SETFL, flags | O_NONBLOCK);
        // poll()会等待50ms，期间可以被信号中断，如果没有数据，poll()超时返回，继续检查running_标志
        struct pollfd pfd;
        pfd.fd     = handle;
        pfd.events = POLLIN;
        auto power = -1;
        auto total_current = -1;

        struct can_frame frame;
        // 这里需要在canfd和can之间切换，数据结构不同
        // struct canfd_frame frame;
        RCLCPP_INFO(node_->get_logger(),
            "Starting receive thread for %s", interface_name.c_str());
        while (running_ && rclcpp::ok())
        {
            int ret = poll(&pfd, 1, 50); // 50ms 超时
            if (ret <= 0) continue;
    
            int nbytes = read(handle, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                if (errno == EINTR || errno == EAGAIN) continue;
                RCLCPP_ERROR(node_->get_logger(), "[%s] read error: %s",
                             interface_name.c_str(), strerror(errno));
                break;
            }
            if (nbytes == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (frame.can_id == 0x4A1){
                // 小端序，byte1在0前作为高位
                int rawPower = CanReceiver::toDecimal(frame.data[1], frame.data[0]);   // 功率原始值
                // 转换为物理值
                power = rawPower * 0.1 * 1000;       // 充电功率，单位 kW
                double voltage_V = 56;
                total_current = power / 56;
            }
            std::vector<struct can_frame> send_frames;
            // // 电池帧，用来汇报电池电量，充电放电等。
            if (frame.can_id == 0x2A1){
                uint8_t battery = frame.data[4];
                // 1故障，2放电，3充电
                uint8_t battery_state = frame.data[5];
                // 1. 创建消息对象
                auto message = ref_slam_interface::msg::BatteryState();

                // 2. 填充数据（假设你已经获取到了这些 float64 变量）
                message.battery_level     = battery;
                message.battery_status    = battery_state;
                message.total_voltage     = 56;
                message.total_current     = total_current;
                // RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing battery msg, power=" << frame[i].ID);

                if (power != -1){
                    // 3. 发布消息
                    battery_publisher_->publish(message);
                }
                // 低电量报警
                if (battery < 30){
                    struct can_frame v_frame = make_frame(0x401);
                    v_frame.data[2] = 17;
                    send_frames.push_back(v_frame);                            
                }
            }
            // 0x181 检测上报小车实际速度和转向
            if (frame.can_id == 0x181)
            {
                // 记录插销当前状态
                if((frame.data[4] >> 5) & 0x01){
                    hook = 1;
                }
                if((frame.data[4] >> 6) & 0x01){
                    hook = -1;
                }
                if((frame.data[4] >> 7) & 0x01){
                    hook = 2;
                }
                if((frame.data[5] >> 0) & 0x01){
                    hook = -2;
                }
                // 将两个字节的数据合并在一起，来表示一项数据，一个字节8位，16位能表示更大的数字，这种使用方式完全由用户协定
                // 读取can构建数据
                double current_angle = toDecimal(frame.data[1], frame.data[0])*0.01; // 原始单位： 0.01 ° 角度
                double current_speed = toDecimal(frame.data[3], frame.data[2])*0.001; // 原始单位： mm/s
                double heading_rate = (current_speed * tan(current_angle * M_PI / 180.0)) / kWheelbase;
                this->pushRecord(frame, current_angle, current_speed);
                // 一个字节有8个标志位，与操作只保留一个，具体含义完全由用户协定
                int operate_mode = frame.data[4] & 0x01;
                if (agv_info_.operate_mode != operate_mode){
                    agv_info_.operate_mode = operate_mode;  // 车辆操作模式  1：自动  0：人工
                    // 切换模式时， 都要调用服务
                    // this->call_engage_service_async(operate_mode == 1);
                }

                // 自动模式下，时刻播报语音
                // 不需要一个语音帧，就包含全部的语音功能，可以只包含一部分帧，其余部分为空，由其他语音帧包含值，或者把语音帧作为类变量，但是在哪push是个问题，必须是一直在调用的那个函数里，时刻push，其他函数只负责在调用的时候改写这个帧
                // 把构造语音控制帧的push放在receiveTask里没问题，因为时刻都能收到总线的帧，语音帧也可以一直往总线发
                if (agv_info_.operate_mode == 1){
                    voice_frame.can_id = 0x401;
                    // 标识数据帧的长度为8个字节
                    voice_frame.can_dlc = 8;
                    voice_frame.data[0] = 0x00;
                    voice_frame.data[1] = 0x00;
                    voice_frame.data[2] = 1;
                    voice_frame.data[3] = 0x00;
                    voice_frame.data[4] = 0x00;
                    voice_frame.data[5] = 0x00;
                    voice_frame.data[6] = 0x00;
                    voice_frame.data[7] = 0x00;
                    // 添加到队列的功能放在所有帧处理的最后
                }

                
                agv_info_.enable = (frame.data[4] >> 1) & 0x01; // 抱闸状态
                agv_info_.steer_enable = (frame.data[4] >> 2) & 0x01;  // 转向使能
                agv_info_.drive_enable  = (frame.data[4] >> 3) & 0x01;  // 驱动使能

                current_angle_ = current_angle * kPi / 180.0;
                autoware_vehicle_msgs::msg::SteeringReport steering_info;
                steering_info.steering_tire_angle = current_angle_;
                steering_info.stamp = node_->get_clock()->now();
                this->getSteeringPub()->publish(steering_info);

                // 构建velocity话题信息
                autoware_vehicle_msgs::msg::VelocityReport velocity_info;
                velocity_info.header.frame_id = "base_link";
                velocity_info.header.stamp = node_->get_clock()->now();
                velocity_info.longitudinal_velocity = current_speed;
                velocity_info.lateral_velocity = 0;
                velocity_info.heading_rate = heading_rate;
                this->getVelocityPub()->publish(velocity_info);
                
                // 构建control_mode话题信息
                autoware_vehicle_msgs::msg::ControlModeReport control_mode_info;
                control_mode_info.stamp = node_->get_clock()->now();
                control_mode_info.mode = agv_info_.operate_mode == 1 ? 1 : 4; 
                this->getControlModePub()->publish(control_mode_info);
                
                // RCLCPP_INFO(
                //     node_->get_logger(),
                //     "CAN 0x181 raw: [%02X %02X %02X %02X %02X %02X %02X %02X], angle=%.2f deg, rad=%.4f",
                //     frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                //     frame.data[4], frame.data[5], frame.data[6], frame.data[7],
                //     current_angle,
                //     current_angle_
                //     );
                // // 添加到队列的功能放在所有帧处理的最后
                // send_frames.push_back(voice_frame);
                // send_queue_->push(send_frames);
            }
        }


        RCLCPP_INFO(node_->get_logger(), "Receive thread for %s exited.", interface_name.c_str());
    }


    // 监听前进后退档位
    void CanReceiver::publishGearStatus(uint8_t report)
    {
        current_gear_report_ = report;
        autoware_vehicle_msgs::msg::GearReport gear_report;
        gear_report.stamp = node_->get_clock()->now();
        gear_report.report = report;
        gear_report_publisher_->publish(gear_report);
    }

    void CanReceiver::gear_cmd_callback(
      const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg)
    {
        if (msg->command == autoware_vehicle_msgs::msg::GearCommand::REVERSE) {
            gear = -1;
        } else {
            gear = 1;
        }
        publishGearStatus(msg->command);
    }
    // 这是监听autoware发来的控制命令的回调函数，但是进入自动模式时，autoware也不一定就有控制指令发来，所以驾驶模式的语音播报在receiveTask中额外构造帧
    void CanReceiver::control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg){

        const auto now = node_->get_clock()->now();
        if ((now - last_control_cmd_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                "接收到control_cmd话题\t angle: " << msg->lateral.steering_tire_angle << "\t" <<
                                    "velocity: " << msg->longitudinal.velocity);
            last_control_cmd_log_time_ = now;
        }
        {
            geometry_msgs::msg::Twist rx_msg;
            rx_msg.linear.x = msg->longitudinal.velocity;
            rx_msg.angular.z = msg->lateral.steering_tire_angle;
            control_cmd_debug_pub_->publish(rx_msg);
        }
        last_control_time_ = now;   // 新增：记录时间
        // 计算挂车后的最大转向角度
        const int trailer_num = trailer_config["trailer_num"];
        const int l = trailer_config["l"];
        const int a = trailer_config["a"];
        const int b = trailer_config["b"];
        const int l1 = trailer_config["l1"];
        // 结果为弧度
        double max_trailer_angle = trailer_config["max_trailer_angle"];
        if ( trailer_num > 0 ){
            max_trailer_angle = calculateTrailerAngle(trailer_num, l, a, b, l1);
        }
        // RCLCPP_INFO_STREAM(node_->get_logger(), "111111111111max_trailer_angle: " << max_trailer_angle);
        // RCLCPP_INFO_STREAM(node_->get_logger(),
        //     "max_trailer_angle: " << max_trailer_angle);
        if(1.0471975803375244 < max_trailer_angle){
            max_trailer_angle = 1.0471975803375244;
        }
        std::vector<struct can_frame> send_frames;
        struct can_frame frame{};
        
        double velocity = msg->longitudinal.velocity;  // 这里单位为: m/s
        velocity = velocity * gear;
        double angle = msg->lateral.steering_tire_angle; // 这里单位为: 弧度
        
        if (std::fabs(angle) > max_trailer_angle){
            if (angle > 0){
                angle = max_trailer_angle;
            }
            else{
                angle = -max_trailer_angle;
            }
        }

        // 转化为角度并乘100，转化为epec需要的指令格式
        int16_t angle_command = static_cast<int16_t>(100 * angle * 180 / kPi);
        if (angle_command > 6000) {
            angle_command = 6000;
        } else if (angle_command < -6000) {
            angle_command = -6000;
        }

        struct can_frame v_frame = make_frame(0x401);
        // 第4位设为1,打开语音播报
        v_frame.data[0] |= (1 << 4);
        // 左转,角度小于500认为微调，非转向
        if (angle_command < 6001 && angle_command > 500){
            v_frame.data[2] = 2;
            // 左转向灯，第0位设为1
            v_frame.data[0] |= (1 << 0);
            RCLCPP_INFO_STREAM(node_->get_logger(),"左转向:");
        }
        // 右转
        if (angle_command > -6001 && angle_command < -500){
            v_frame.data[2] = 3;
            v_frame.data[0] |= (1 << 1);
            RCLCPP_INFO_STREAM(node_->get_logger(),"右转向.");
        }

        if (velocity < 0){
            // 正在倒车语音
            v_frame.data[2] = 4;
            v_frame.data[3] |= (1 << 1);
        }
        send_frames.push_back(v_frame);

        // 转化为epec需要的指令格式。单位：0.001 m/s,
        int16_t speed_command = velocity * 1000;

        // 基本控制驱动方实行的速度限制，speed_upper_bound_现设置为500，即0.5m/s
        speed_command = std::min(speed_command, (int16_t)speed_upper_bound_);
        speed_command = std::max(speed_command, (int16_t)-speed_upper_bound_);

        // // 舵轮转角超过阈值时, 再次降低车速
        // // TODO: 分阶段降低; 参数化
        // if(std::fabs(angle_command) >= 1000 && std::fabs(speed_command) > 300){
        //     speed_command = speed_command<0?-300:300;
        // }
        if ((now - last_can_cmd_log_time_).seconds() >= 1.0) {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                "实际控制can输出\t angle: " << angle_command << "\t" <<
                                    "velocity: " << speed_command);
            last_can_cmd_log_time_ = now;
        }
        {
            geometry_msgs::msg::Twist can_msg;
            can_msg.linear.x = static_cast<double>(speed_command);
            can_msg.angular.z = static_cast<double>(angle_command);
            can_cmd_debug_pub_->publish(can_msg);
        }
        agv_info_.speed_command = speed_command;
        agv_info_.steer_command = angle_command;
        // 发送控制can指令
        frame.can_id = 0x201;
        frame.can_dlc = 8;
        frame.data[0] = agv_info_.speed_command & 0xff;
        frame.data[1] = agv_info_.speed_command >> 8;
        frame.data[2] = agv_info_.steer_command & 0xff;
        frame.data[3] = agv_info_.steer_command >> 8;
        frame.data[4] = 0b00011011;
        frame.data[5] = 10;
        frame.data[6] = 10;  
        frame.data[7] = 0x00;


        // if (agv_info_.steer_enable == 0 || agv_info_.drive_enable == 0){
        //     RCLCPP_INFO_STREAM(node_->get_logger(),
        //     "转向和驱动使能" << agv_info_.enable);
        //     frame.data[4] = 0b00001011;
        //     frame.data[0] = 0;
        //     frame.data[1] = 0;
        //     frame.data[2] = 0;
        //     frame.data[3] = 0;
        // }
        // if (agv_info_.drive_enable == 1 && agv_info_.speed_command < 1 && agv_info_.enable == 1){
        //     RCLCPP_INFO_STREAM(node_->get_logger(),
        //     "抱闸");
        //     frame.data[4] = 0b00001011;
        // }
        // else if (agv_info_.steer_enable == 1 && agv_info_.drive_enable == 1 && agv_info_.speed_command != 0){
        //     RCLCPP_INFO_STREAM(node_->get_logger(),
        //     "释放抱闸");
        //     frame.data[4] = 0b00011011;
        // }
        
        safe_frame = frame;
        send_frames.push_back(frame);

        // 用于手动切换自动模式后完成切换，这个帧有必要一直发吗？
        // 怀疑是想加语音帧加错了，待验证
        const bool should_send_engage =
          (now - last_engage_frame_time_).nanoseconds() >=
          static_cast<int64_t>(engage_frame_period_ms_) * 1000000LL;
        if (should_send_engage) {
            struct can_frame engage_frame = make_frame(0x301);
            engage_frame.data[2] = 0x02;
            send_frames.push_back(engage_frame);
            last_engage_frame_time_ = now;
        }

        // 添加到队列
        if (!send_frames.empty()) {
            send_queue_->push(send_frames);
        }
        // ULONG result = VCI_Transmit(
        //     gDevType, 
        //     gDevIdx, 
        //     0,                      // 0=CAN0
        //     &send_frames[0],     // 【关键】直接传入 vector 的数据指针，等价于 
        //     static_cast<UINT>(send_frames.size()) // 发送帧的数量
        // );
        
        // // 3. 检查返回值
        // if (result == 1) {
        //     RCLCPP_INFO(node_->get_logger(), "Publishing控制指令...  ");
        // } else {
        //     RCLCPP_INFO(node_->get_logger(), "Publishing控制指令失败...  ");
        // }
        // 日志：时间戳, 原始速度指令, 转向指令, CAN速度指令, CAN转向指令
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6)
            << now.seconds() << ","          // Unix 时间戳（秒.纳秒）
            << velocity << ","
            << msg->lateral.steering_tire_angle << ","
            << speed_command << ","
            << angle_command;
        log_buffer_.push_back(oss.str());

        // 缓冲区满 1000 条时批量写入
        if (log_buffer_.size() >= BUFFER_FLUSH_SIZE) {
            flushLogBuffer();
        }
    }

      void CanReceiver::call_engage_service_async(bool engage){
        RCLCPP_INFO_STREAM(node_->get_logger(),
            "开始调用engage服务...");
        
        // 检查服务是否可用（快速检查）
        if (!engage_client_->wait_for_service(std::chrono::milliseconds(100))) {
            RCLCPP_WARN(node_->get_logger(), "Engage服务暂时不可用");
            agv_info_.operate_mode = engage ? 0 : 1;
            return;
        }
        
        // 创建请求
        auto request = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
        request->engage = engage;

        RCLCPP_INFO_STREAM(node_->get_logger(),
            "Calling engage service with: " << (engage ? "true" : "false"));  // 修复运算符优先级问题

        // 异步发送请求，使用回调处理响应（不会阻塞CAN解析线程）
        engage_client_->async_send_request(
            request,
            [this, engage](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture future) {
                this->handle_engage_response(future, engage);
            }
        );
    }
    
    void CanReceiver::handle_engage_response(rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture future, bool expected_engage) {
        try {
            auto response = future.get();
            if(response->status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS){
                RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Service /api/autoware/set/engage call successed!");
            } else {
                RCLCPP_INFO_STREAM(node_->get_logger(),
                    "Service /api/autoware/set/engage call failed with code: " << 
                    response->status.code << " message: " << response->status.message.c_str());
                agv_info_.operate_mode = expected_engage ? 0 : 1;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(node_->get_logger(),
                "Service /api/autoware/set/engage call exception: " << e.what());
            agv_info_.operate_mode = expected_engage ? 0 : 1;
        }
    }

    void CanReceiver::call_engage_service(bool engage){
        RCLCPP_INFO_STREAM(node_->get_logger(),
            "开始调用engage服务...");
        // 等待服务可用
        if (!engage_client_->wait_for_service(std::chrono::duration<double>(engage_srv_wait_timeout_))){
            RCLCPP_INFO_STREAM(node_->get_logger(),
            "Service /api/autoware/set/engage not available after " << engage_srv_wait_timeout_ <<  "seconds");
            return;
        }

        // 创建请求
        auto request = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
        request->engage = engage;

        RCLCPP_INFO_STREAM(node_->get_logger(),
            "Calling engage service with: " << (engage ? "true" : "false"));

        // 发送异步请求并处理响应
        auto future = engage_client_->async_send_request(request);

        // 处理响应
        auto response = future.get();
        if(response->status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS){
            RCLCPP_INFO_STREAM(node_->get_logger(),
            "Service /api/autoware/set/engage call successed!");
        }else{
            RCLCPP_INFO_STREAM(node_->get_logger(),
            "Service /api/autoware/set/engage call failed with code: " << response->status.code << "message: " << response->status.message.c_str());
        }


        

    }



    // 将 int32_t 转换为 4 字节的大端序十六进制数组
    void CanReceiver::decimalToHexBytes(int32_t value, uint8_t bytes[4])
    {
        bytes[0] = (value >> 24) & 0xFF;
        bytes[1] = (value >> 16) & 0xFF;
        bytes[2] = (value >> 8) & 0xFF;
        bytes[3] = value & 0xFF;
    }

    // 十进制转十六进制字符串（带前缀 0x）
    std::string CanReceiver::decimalToHexString(int32_t value)
    {
        std::stringstream ss;
        ss << "0x" << std::setfill('0') << std::setw(8) << std::hex << (static_cast<uint32_t>(value));
        return ss.str();
    }
    /// @brief 将两个字节转换为一个16位的十进制数。输入:十进制数,表示高八位和低八位
    /// @param high 高8位的整数
    /// @param low 低8位的整数
    /// @return 返回转换后的16位十进制数
    int CanReceiver::toDecimal(int high, int low)
    {
        std::string hi, lo;
        int result;

        // 十进制转化为二进制
        for (int i = 7; i >= 0; i--)
        {
            hi += std::to_string(((high >> i) & 1));
        }

        for (int j = 7; j >= 0; j--)
        {
            lo += std::to_string(((low >> j) & 1));
        }

        std::bitset<16> binaryData(hi + lo);

        if (binaryData[15] == 1)
        {
            binaryData.flip();
            result = std::stoi(binaryData.to_string(), nullptr, 2);
            result = -(result + 1);
        }
        else
        {
            result = std::stoi(binaryData.to_string(), nullptr, 2);
        }

        return result;
    }

    void CanReceiver::CreateSafetyFrame() {
        // 构造安全报文 - 速度为0，转角为0，抱闸
        safe_frame.can_id = 0x201;
        safe_frame.can_dlc = 8;

        safe_frame.data[0] = 0;
        safe_frame.data[1] = 0;
        safe_frame.data[2] = 0;
        safe_frame.data[3] = 0;
        safe_frame.data[4] = 0b10000000; // 抱闸状态
        safe_frame.data[5] = 0;
        safe_frame.data[6] = 0;
        safe_frame.data[7] = 0;
    }

    void CanReceiver::sendSafetyFrameCallback() {
        std::vector<struct can_frame> send_frames;
        struct can_frame voice_frame;
        voice_frame.can_id = 0x401;
        // 标识数据帧的长度为8个字节
        voice_frame.can_dlc = 8;
        voice_frame.data[0] = 0x00;
        // 打开语音播报
        voice_frame.data[0] |= (1 << 4);
        voice_frame.data[1] = 0x00;
        voice_frame.data[2] = 1;
        voice_frame.data[3] = 0x00;
        voice_frame.data[4] = 0x00;
        voice_frame.data[5] = 0x00;
        voice_frame.data[6] = 0x00;
        voice_frame.data[7] = 0x00;
        // 添加到队列的功能放在所有帧处理的最后
        send_frames.push_back(voice_frame);
        send_queue_->push(send_frames);
        RCLCPP_INFO_STREAM(node_->get_logger(),"无人驾驶.");
    }


} // namespace can_driver
// ros2 bag record /autoware_orientation /sensing/gnss/rtk/nav_sat_fix /rtk_imu/data_raw
// ros2 run can_driver can_rtk_node
// candump can1 -t a > can_data.log 2>&1 &
// awk 'BEGIN{FS=":"} /can1  601/{a++} /can1  602/{b++} END{printf "601: %.1f Hz, 602: %.1f Hz\n", a/10, b/10}' <(timeout 10 candump can1)
// ros2 topic pub -r 1 /control/command/control_cmd autoware_control_msgs/msg/Control "
// stamp:
//   sec: 0
//   nanosec: 0
// lateral:
//   steering_tire_angle: 0
//   steering_tire_rotation_rate: 0.0
// longitudinal:
//   velocity: 0.1
//   acceleration: 0.5
//   jerk: 0.0
// "

// ros2 topic pub -1 /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 20}"