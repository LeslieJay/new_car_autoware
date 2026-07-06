// src/can_receiver.cpp

#include "can_driver/can_receiver.hpp"
#include "can_driver/can_node.hpp"
#include "can_driver/trailer.hpp"
#include <rclcpp/rclcpp.hpp>                     // 用于 rclcpp::Publisher
#include <iostream>
#include <autoware_control_msgs/msg/safety_state.hpp>
#include <ctime>          // for struct tm, timegm
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <chrono>
#include <linux/can.h>
#include <linux/can/raw.h> 
extern std::shared_ptr<can_driver::CanSend> send_queue_;

builtin_interfaces::msg::Time toRosTime(int year, int month, int day,
    int hour, int minute, int millisecond)
{
    // 填充 struct tm（注意字段范围和含义）
    struct tm tm {};
    tm.tm_year = year - 1900;          // 年从1900起
    tm.tm_mon  = month - 1;             // 月 0-11
    tm.tm_mday = day;                   // 日 1-31
    tm.tm_hour = hour;                   // 时 0-23
    tm.tm_min  = minute;                  // 分 0-59
    tm.tm_sec  = 0;                       // 秒（假设秒为0）
    tm.tm_isdst = 0;                       // 不考虑夏令时

    // 转换为 UTC 时间戳（秒）
    time_t secs = timegm(&tm);

    // 构造 ROS 2 Time 消息
    builtin_interfaces::msg::Time stamp;
    stamp.sec = secs;
    stamp.nanosec = millisecond * 1000000u;  // 毫秒转纳秒
    return stamp;
}

geometry_msgs::msg::Quaternion attiToQuaternion(const int64_t atti[3])
{
    const double DEG_TO_RAD = M_PI / 180.0;

    // 1. 角度转弧度（注意顺序：Roll, Pitch, Yaw）
    double roll  = static_cast<double>(atti[1]) * DEG_TO_RAD;   // Roll 绕X轴
    double pitch = static_cast<double>(atti[0]) * DEG_TO_RAD;   // Pitch 绕Y轴

    // 2. 处理航向角（假设 Heading 是北顺时针，需转换为 ROS 偏航角）
    double heading_deg = static_cast<double>(atti[2]);
    double yaw_deg = 90.0 - heading_deg;   // 转换为从东逆时针

    // 可选：将 yaw_deg 归一化到 [-180,180]
    while (yaw_deg > 180.0)  yaw_deg -= 360.0;
    while (yaw_deg < -180.0) yaw_deg += 360.0;

    double yaw = yaw_deg * DEG_TO_RAD;

    // 3. 使用 tf2 生成四元数
    tf2::Quaternion tf_q;
    tf_q.setRPY(roll, pitch, yaw);

    // 4. 转换为 ROS 消息类型
    geometry_msgs::msg::Quaternion ros_q;
    ros_q.x = tf_q.x();
    ros_q.y = tf_q.y();
    ros_q.z = tf_q.z();
    ros_q.w = tf_q.w();

    return ros_q;
}
namespace can_driver
{

    CanReceiver::CanReceiver(std::shared_ptr<rclcpp::Node> node): node_(node), running_(false)
    {
        // 实例通过调用->publish方法发布数据
        steering_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
        velocity_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
        control_mode_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
        //rtk_NavSatFix_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("/sensing/gnss/rtk/nav_sat_fix", 10);
        //rtk_GnssInsOrientationStamped_publisher_ = node_->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>("/autoware_orientation", 10);

        control_subscript_ = node_->create_subscription<autoware_control_msgs::msg::Control>(
            "/control/command/control_cmd", 1, std::bind(&CanReceiver::control_cmd_callback, this, _1));
        agv_state_subscript_ = node_->create_subscription<vda5050_interfaces::msg::AGVState>(
            "uagv/v1/BYD/qqa0001/state", 1, std::bind(&CanReceiver::agv_state_callback, this, _1));
        battery_publisher_ = node_->create_publisher<agv_interfaces::msg::BatteryState>("/battery", 10);
        error_publisher_ = node_->create_publisher<vda5050_interfaces::msg::Error>("/error", 10);

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

        node_->get_parameter("engage_service_wait_timeout", engage_srv_wait_timeout_);
        node_->get_parameter("engage_service_call_timeout", engage_srv_call_timeout_);
        
        // 创建安全定时器，之后无需任何调用即可定期执行sendSafetyFrameCallback
        safety_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&CanReceiver::sendSafetyFrameCallback, this)
        );
        CreateSafetyFrame();
    }

    CanReceiver::~CanReceiver()
    {
        // stop(); // 确保线程停止
        // 注意：socket 的关闭由调用者负责！
        running_ = false;
    }

    bool CanReceiver::isRunning() const
    {
        return running_;
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
    void CanReceiver::agv_state_callback(const vda5050_interfaces::msg::AGVState::ConstSharedPtr msg){
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
        if (msg->driving == false){
            // 第4位设为1,打开语音播报
            v_frame.data[0] |= (1 << 4);
            // 语音到达目标点
            v_frame.data[2] = 5;
        }
        send_frames.push_back(v_frame);
        // 添加到队列
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
    } 
    /**
     * @brief 接受报文
     *
     * @param handle socket返回的文件描述符
     * @param interface_name can接口名
     */
//     void CanReceiver::receiveTask(int handle, const std::string &interface_name)
//     {
//         // 线程运行标志，用于安全停止线程，该监听在独立的线程中无限循环，直到程序停止
//         running_ = true;

//         RCLCPP_INFO(node_->get_logger(),
//                     "Starting receive thread for %s", interface_name.c_str());

//         /* 让 socket 支持被信号唤醒（可选，但无害）*/
//         int flags = fcntl(handle, F_GETFL, 0);
//         fcntl(handle, F_SETFL, flags | O_NONBLOCK);
//         // poll()会等待50ms，期间可以被信号中断，如果没有数据，poll()超时返回，继续检查running_标志
//         struct pollfd pfd;
//         pfd.fd     = handle;
//         pfd.events = POLLIN;

//         struct can_frame frame;
//         RCLCPP_INFO(node_->get_logger(),
//             "Starting receive thread for %s", interface_name.c_str());
//         while (running_ && rclcpp::ok())
//         {
//             /* poll 50 ms 超时，可被 Ctrl-C 信号中断，ret则被置为-1 */
//             int ret = poll(&pfd, 1, 50);   // 50 ms
//             if (ret <= 0)
//                 continue;                   // 超时或错误，立即重试

//             /* 此时必定有数据可读 */
//             // poll() 是数据就绪检查器，它确保了当代码执行到 read() 时，数据已经准备就绪，从而避免了无效的读取尝试和复杂的空值处理逻辑。这是Linux/Unix系统中同步IO多路复用的标准模式。
//             // 读取can数据帧，如果出错则为-1，CAN接口不存在为0，把内容存储在frame中，nbytes只是个标志位
//             int nbytes = ::read(handle, &frame, sizeof(frame));
//             if (nbytes < 0)
//             {
//                 if (errno == EINTR || errno == EAGAIN)
//                     continue;               // 被信号或 EAGAIN，继续

//                 RCLCPP_ERROR(node_->get_logger(),
//                             "[%s] read error: %s",
//                             interface_name.c_str(), std::strerror(errno));
//                 break;
//             }
//             else if (nbytes == 0)
//             {
//                 /* 对 RAW socket 基本不会发生，安全起见 sleep 一下 */
//                 std::this_thread::sleep_for(std::chrono::milliseconds(1));
//                 continue;
//             }
// //#ifdef DUBUG
//                 // 打印接受到的报文，注意直接收过滤器中设置的报文
//                 // std::stringstream ss;
//                 // ss << "ID:0x" << std::hex << frame.can_id
//                 //    << " DLC:" << std::dec << (int)frame.can_dlc << " Data:";
//                 // for (int i = 0; i < frame.can_dlc; ++i)
//                 // {
//                 //     ss << " " << std::hex << (int)frame.data[i];
//                 // }
//                 // RCLCPP_INFO(node_->get_logger(), "[%s] %s", interface_name.c_str(), ss.str().c_str());
// //#endif
//                 // 解析rtk数据
//                 // ins_pos_can_t 定义（需紧凑排列）
//                 typedef struct ins_pos_can_t{
//                     uint8_t  valid;          // 1字节
//                     uint8_t  state;          // 1字节
//                     uint16_t year;           // 2字节（小端）
//                     uint8_t  month;          // 1字节
//                     uint8_t  day;            // 1字节
//                     uint8_t  hour;           // 1字节
//                     uint8_t  minute;         // 1字节
//                     uint64_t seconds;        // 
//                     int64_t  pos_ins[3];     // 
//                     int64_t  vel[3];         //
//                     // 可能还有其他未打印字段，总大小64字节
//                 } __attribute__((packed));

//                 typedef struct ins_atti_can_t{
//                     uint8_t  valid;           ///< DR标志: 0=DR无效, 1=DR获取GNSS, 2=DR转导航
//                     uint8_t  state;           ///< 定位状态: 0=无效, 1=单点, 2=DGNSS, 4=固定解, 5=浮点解, 6=DR递推
                    
//                     // UTC 时间
//                     uint16_t year;            ///< 年
//                     uint8_t  month;           ///< 月
//                     uint8_t  day;             ///< 日
//                     uint8_t  hour;            ///< 时
//                     uint8_t  minute;          ///< 分
//                     int64_t  seconds;         ///< 秒 (单位: s * INT_SEC)
                    
//                     // 姿态角
//                     int64_t  atti[3];         ///< [Pitch, Roll, Heading] 单位: deg (缩放: INT_ATTI)
//                                             ///< Pitch: [-90, 90], Roll: [-180, 180], Heading: [0, 360]
                    
//                     // IMU 原始数据
//                     int32_t  gyro_xyz[3];     ///< [Gx, Gy, Gz] 单位: deg/s (缩放: INT_IMU)
//                     int32_t  acc_xyz[3];      ///< [Ax, Ay, Az] 单位: g    (缩放: INT_IMU)
                    
//                     // 注意：若结构体不足64字节，编译器可能会自动填充或需手动添加保留字段
//                 } __attribute__((packed));

//                 // 缩放因子（根据实际协议定义）
//                 constexpr uint32_t CANID_INS_POS = 0x601; // 替换为实际ID
//                 const double INT_POS = 1e8;      // 纬度/经度缩放
//                 const double INT_ALT = 1e8;      // 海拔缩放
//                 const int INT_SEC = 1000;        // 毫秒转秒
//                 if (frame.can_id == 0x601){
//                     RCLCPP_INFO(node_->get_logger(), "正在解析rtk数据601");
//                     ins_pos_can_t dataPos;
//                     std::memcpy(&dataPos, &frame, sizeof(dataPos));  // 字节拷贝
//                     sensor_msgs::msg::NavSatFix msg;
                    
//                     msg.header.stamp = toRosTime( dataPos.year, dataPos.month, dataPos.day,
//                         dataPos.hour, dataPos.minute, dataPos.seconds);
//                     msg.header.frame_id = "base_link";
//                     msg.longitude = static_cast<double>(dataPos.pos_ins[1]) / INT_POS;
//                     msg.latitude = static_cast<double>(dataPos.pos_ins[0]) / INT_POS;
//                     msg.altitude = static_cast<double>(dataPos.pos_ins[2]) / INT_POS;
//                     msg.status.status = dataPos.state;
//                     rtk_NavSatFix_publisher_->publish(msg);
                    
//                     // 打印数据（使用原始代码的格式）
//                     printf("[POS] ID:0x%03X | Valid:%d, State:%d | Time:%04d-%02d-%02d %02d:%02d:%05.2f\n",
//                         frame.can_id,
//                         dataPos.valid,
//                         dataPos.state,
//                         dataPos.year, dataPos.month, dataPos.day,
//                         dataPos.hour, dataPos.minute,
//                         static_cast<double>(dataPos.seconds) / INT_SEC);

//                     printf("      Lat:%.8f, Lon:%.8f, Alt:%.4f (m)\n",
//                         static_cast<double>(dataPos.pos_ins[0]) / INT_POS,
//                         static_cast<double>(dataPos.pos_ins[1]) / INT_POS,
//                         static_cast<double>(dataPos.pos_ins[2]) / INT_POS);

//                     printf("      Ve:%.4f, Vn:%.4f, Vu:%.4f (m/s)\n",
//                         static_cast<double>(dataPos.vel[0]) / INT_POS,
//                         static_cast<double>(dataPos.vel[1]) / INT_POS,
//                         static_cast<double>(dataPos.vel[2]) / INT_POS);
                    
//                 }
//                 if (frame.can_id == 0x602){
//                     RCLCPP_INFO(node_->get_logger(), "正在解析rtk数据602");
//                     ins_atti_can_t dataPos;
//                     std::memcpy(&dataPos, &frame, sizeof(dataPos));  // 字节拷贝
//                     autoware_sensing_msgs::msg::GnssInsOrientationStamped GnssInsOrientationStamped_msg;    

//                     GnssInsOrientationStamped_msg.header.stamp = toRosTime( dataPos.year, dataPos.month, dataPos.day,
//                         dataPos.hour, dataPos.minute, dataPos.seconds);
//                     GnssInsOrientationStamped_msg.header.frame_id = "gnss_link";
//                     GnssInsOrientationStamped_msg.orientation.orientation = attiToQuaternion(dataPos.atti);
//                     rtk_GnssInsOrientationStamped_publisher_->publish(GnssInsOrientationStamped_msg);

//                 }
//                 std::vector<struct can_frame> send_frames;
//                 // 电池帧，用来汇报电池电量，充电放电等。
//                 if (frame.can_id == 0x2A1){
//                     uint8_t battery = frame.data[5];
//                     // 1故障，2放电，3充电
//                     uint8_t battery_state = frame.data[6];
//                     // 低电量报警
//                     if (battery < 20){
//                         struct can_frame v_frame{}; 
//                         v_frame.can_id = 0x401;
//                         v_frame.can_dlc = 8;
                
//                         v_frame.data[0] = 0;
//                         v_frame.data[1] = 0;
//                         v_frame.data[2] = 17;
//                         v_frame.data[3] = 0;
//                         v_frame.data[4] = 0;
//                         v_frame.data[5] = 0;
//                         v_frame.data[6] = 0;
//                         v_frame.data[7] = 0;
//                         send_frames.push_back(v_frame);                            
//                     }
//                 }

//                 if (frame.can_id == 0x201)
//                 {
//                     std::stringstream ss;
//                     // std::hex使输出变成16进制，std::dec切换为十进制格式
//                     ss << "ID:0x" << std::hex << frame.can_id
//                     << " DLC:" << std::dec << (int)frame.can_dlc << " Data:";
//                     // can_dlc数据长度
//                     for (int i = 0; i < frame.can_dlc; ++i)
//                     {
//                         ss << " " << std::hex << (int)frame.data[i];
//                     }
//                     RCLCPP_INFO(node_->get_logger(), "[%s] %s", interface_name.c_str(), ss.str().c_str());
//                 }
//                 // 0x181 检测上报小车实际速度和转向
//                 if (frame.can_id == 0x181)
//                 {
//                     // 将两个字节的数据合并在一起，来表示一项数据，一个字节8位，16位能表示更大的数字，这种使用方式完全由用户协定
//                     // 读取can构建数据
//                     double current_angle = toDecimal(frame.data[1], frame.data[0])*0.01; // 原始单位： 0.01 ° 角度
//                     double current_speed = toDecimal(frame.data[3], frame.data[2])*0.001; // 原始单位： mm/s
//                     double heading_rate = (current_speed * tan(current_angle * M_PI / 180.0)) / WHEELBASE;
//                     // 一个字节有8个标志位，与操作只保留一个，具体含义完全由用户协定
//                     int operate_mode = frame.data[4] & 0x01;
//                     if (agv_info_.operate_mode != operate_mode){
//                         agv_info_.operate_mode = operate_mode;  // 车辆操作模式  1：自动  0：人工
//                         // 切换模式时， 都要调用服务
//                         // this->call_engage_service_async(operate_mode == 1);
//                     }

//                     // 自动模式下，时刻播报语音
//                     // 不需要一个语音帧，就包含全部的语音功能，可以只包含一部分帧，其余部分为空，由其他语音帧包含值，或者把语音帧作为类变量，但是在哪push是个问题，必须是一直在调用的那个函数里，时刻push，其他函数只负责在调用的时候改写这个帧
//                     // 把构造语音控制帧的push放在receiveTask里没问题，因为时刻都能收到总线的帧，语音帧也可以一直往总线发
//                     if (agv_info_.operate_mode == 1){
//                         voice_frame.can_id = 0x401;
//                         // 标识数据帧的长度为8个字节
//                         voice_frame.can_dlc = 8;
//                         voice_frame.data[0] = 0x00;
//                         voice_frame.data[1] = 0x00;
//                         voice_frame.data[2] = 1;
//                         voice_frame.data[3] = 0x00;
//                         voice_frame.data[4] = 0x00;
//                         voice_frame.data[5] = 0x00;
//                         voice_frame.data[6] = 0x00;
//                         voice_frame.data[7] = 0x00;
//                         // 添加到队列的功能放在所有帧处理的最后
//                     }

                    
//                     agv_info_.enable = (frame.data[4] >> 1) & 0x01; // 抱闸状态
//                     agv_info_.steer_enable = (frame.data[4] >> 2) & 0x01;  // 转向使能
//                     agv_info_.drive_enable  = (frame.data[4] >> 3) & 0x01;  // 驱动使能 

//                     current_angle_ = current_angle *PI/180;
//                     autoware_vehicle_msgs::msg::SteeringReport steering_info;
//                     steering_info.steering_tire_angle = current_angle;
//                     steering_info.stamp = node_->get_clock()->now();
//                     this->getSteeringPub()->publish(steering_info);

//                     // 构建velocity话题信息
//                     autoware_vehicle_msgs::msg::VelocityReport velocity_info;
//                     velocity_info.header.frame_id = "base_link";
//                     velocity_info.header.stamp = node_->get_clock()->now();
//                     velocity_info.longitudinal_velocity = current_speed;
//                     velocity_info.lateral_velocity = 0;
//                     velocity_info.heading_rate = heading_rate;
//                     this->getVelocityPub()->publish(velocity_info);
                    
//                     // 构建control_mode话题信息
//                     autoware_vehicle_msgs::msg::ControlModeReport control_mode_info;
//                     control_mode_info.stamp = node_->get_clock()->now();
//                     control_mode_info.mode = agv_info_.operate_mode == 1 ? 1 : 4; 
//                     this->getControlModePub()->publish(control_mode_info);
                    

//                     // 添加到队列的功能放在所有帧处理的最后
//                     send_frames.push_back(voice_frame);
//                     send_queue_->push(send_frames);
//                     RCLCPP_INFO(node_->get_logger(), "Publishing...  ");
//                 }

//                 // 发送固定的导航报文
//                 // std::vector<struct can_frame> send_frames;
//                 // struct can_frame frame{}; 
//                 // frame.can_id = 0x201;
//                 // frame.can_dlc = 8;
//                 // frame.data[0] = agv_info_.speed_command & 0xff;
//                 // frame.data[1] = agv_info_.speed_command >> 8;
//                 // frame.data[2] = agv_info_.steer_command & 0xff;
//                 // frame.data[3] = agv_info_.steer_command >> 8;
//                 // frame.data[4] = 0b00001011; // 默认抱闸状态
//                 // frame.data[5] = 10;
//                 // frame.data[6] = 10;  
//                 // frame.data[7] = 0x00;


//                 // if (agv_info_.steer_enable == 0 || agv_info_.drive_enable == 0){
//                 //     RCLCPP_INFO_STREAM(node_->get_logger(),
//                 //     "转向和驱动使能" << agv_info_.enable);
//                 //     frame.data[4] = 0b00001011;
//                 //     frame.data[0] = 0;
//                 //     frame.data[1] = 0;
//                 //     frame.data[2] = 0;
//                 //     frame.data[3] = 0;
//                 // }
//                 // if (agv_info_.drive_enable == 1 && agv_info_.speed_command < 1 && agv_info_.enable == 1){
//                 //     RCLCPP_INFO_STREAM(node_->get_logger(),
//                 //     "抱闸");
//                 //     frame.data[4] = 0b00001011;
//                 // }
//                 // else if (agv_info_.steer_enable == 1 && agv_info_.drive_enable == 1 && agv_info_.speed_command != 0){
//                 //     RCLCPP_INFO_STREAM(node_->get_logger(),
//                 //     "释放抱闸");
//                 //     frame.data[4] = 0b00011011;
//                 // }
                

//                 // send_frames.push_back(frame);

//                 // frame.can_id = 0x301;
//                 // frame.can_dlc = 8;

//                 // frame.data[0] = 0;
//                 // frame.data[1] = 0;
//                 // frame.data[2] = 0x02;
//                 // frame.data[3] = 0;
//                 // frame.data[4] = 0;
//                 // frame.data[5] = 0;
//                 // frame.data[6] = 0;
//                 // frame.data[7] = 0;

//                 // send_frames.push_back(frame);

//                 // // 添加到队列
//                 // send_queue_->push(send_frames);

// #ifdef DEBUG
//                 // agv的can2,目前工控机没有接受agv的can2，只能接收vcu转发，后续需重新确认电池和托盘的报文，20280814 dxy
//                 // 监听0x1806E59B
//                 if (frame.can_id == 0x1806E59B)
//                 {
//                     battery_info_.charge_allowed = frame.data[4];
//                     switch (battery_info_.charge_allowed)
//                     {
//                     case 0:
//                         RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"), "充电使能状态: " << "允许充电!");
//                         break;
//                     default:
//                         RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"), "充电使能状态 : " << "不允许充电!");
//                         break;
//                     }
//                 }

//                 // 监听0x0C71D0D4 但id是无符号整型 有可能越界
//                 if (frame.can_id == 0xC71D0D4)
//                 {

//                     battery_msg.total_voltage = battery_info_.total_voltage = toDecimal(frame.data[1], frame.data[0]) * 0.015;
//                     battery_msg.total_current = battery_info_.total_current = toDecimal(frame.data[3], frame.data[2]) * 0.05 - 1600;
//                     battery_msg.battery_level = battery_info_.battery_level = frame.data[4] * 0.4;
//                     battery_msg.battery_life = battery_info_.battery_life = frame.data[5] * 0.4;
//                     battery_msg.charge_status = battery_info_.charge_status = frame.data[6] & 0xC0; // 提取最高的两位;

//                     RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"),
//                                        "电池电压 : " << std::dec << battery_info_.total_voltage << " V");
//                     RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"),
//                                        "电池电流 : " << std::dec << battery_info_.total_current << " A");
//                     RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"),
//                                        "电量   : " << std::dec << battery_info_.battery_level << " %");
//                     RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"),
//                                        "电池寿命   : " << std::dec << battery_info_.battery_life << " %");

//                     battery_state_pub_->publish(battery_msg);

//                     switch (battery_info_.charge_status)
//                     {
//                     case 0:
//                         RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"), "电池充电状态  : " << "未充电!");
//                         break;
//                     case 1:
//                         RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"), "电池充电状态  : " << "充电中!");
//                         break;
//                     default:
//                         RCLCPP_INFO_STREAM(rclcpp::get_logger("Read CAN1 Thread"), "电池充电状态  : " << "充电完成!");
//                         break;
//                     }
//                 }


// #endif
         
//         }

//         RCLCPP_INFO(node_->get_logger(), "Receive thread for %s exited.", interface_name.c_str());
//     }


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

            std::vector<struct can_frame> send_frames;
            // // 电池帧，用来汇报电池电量，充电放电等。
            if (frame.can_id == 0x2A1){
                // 电压0.001v
                battery_info_.total_voltage = ToInt16(frame.data[0], frame.data[1]) ;
                // 单位0.1A
                battery_info_.total_current = ToInt16(frame.data[2], frame.data[3])*10;
                battery_info_.battery_level = frame.data[4];
                // 电池没有充电放电许可，全部置为1
                battery_info_.charge_allowed    = 1;
                battery_info_.discharge_allowed = 1;
                uint8_t battery = frame.data[4];
                // 1故障，2放电，3充电
                uint8_t battery_state = frame.data[5];
                // 低电量报警
                if (battery < 20){
                    struct can_frame v_frame{}; 
                    v_frame.can_id = 0x401;
                    v_frame.can_dlc = 8;
            
                    v_frame.data[0] = 0;
                    v_frame.data[1] = 0;
                    v_frame.data[2] = 17;
                    v_frame.data[3] = 0;
                    v_frame.data[4] = 0;
                    v_frame.data[5] = 0;
                    v_frame.data[6] = 0;
                    v_frame.data[7] = 0;
                    send_frames.push_back(v_frame);                            
                }
            }

            if (frame.can_id == 0x201)
            {
                std::stringstream ss;
                // std::hex使输出变成16进制，std::dec切换为十进制格式
                ss << "ID:0x" << std::hex << frame.can_id
                << " DLC:" << std::dec << (int)frame.can_dlc << " Data:";
                // can_dlc数据长度
                for (int i = 0; i < frame.can_dlc; ++i)
                {
                    ss << " " << std::hex << (int)frame.data[i];
                }
                RCLCPP_INFO(node_->get_logger(), "[%s] %s", interface_name.c_str(), ss.str().c_str());
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
                if((frame.data[4] >> 8) & 0x01){
                    hook = -2;
                }
                // 将两个字节的数据合并在一起，来表示一项数据，一个字节8位，16位能表示更大的数字，这种使用方式完全由用户协定
                // 读取can构建数据
                double current_angle = toDecimal(frame.data[1], frame.data[0])*0.01; // 原始单位： 0.01 ° 角度
                double current_speed = toDecimal(frame.data[3], frame.data[2])*0.001; // 原始单位： mm/s
                double heading_rate = (current_speed * tan(current_angle * M_PI / 180.0)) / WHEELBASE;
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

                current_angle_ = current_angle * PI / 180.0;
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
                

                // // 添加到队列的功能放在所有帧处理的最后
                // send_frames.push_back(voice_frame);
                // send_queue_->push(send_frames);
            }
        }


        RCLCPP_INFO(node_->get_logger(), "Receive thread for %s exited.", interface_name.c_str());
    }

    // // 原版键盘控制车辆移动，现在用不上了
    // void CanReceiver::keyboard_cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
    //     // 提取线速度和角速度
    //     double v = msg->linear.x;
    //     double omega = msg->angular.z;
        
    //     double steering_angle = 0.0;
    //     double drive_speed = 0.0;
        
    //     auto request = std::make_shared<agv_interfaces::srv::VoiceControl::Request>();
        
    //     std::vector<struct can_frame> send_frames;
    //     struct can_frame frame{}; 
    //     // 计算舵轮角度和驱动速度
    //     if (std::abs(v) < 0.001 && std::abs(omega) < 0.001)
    //     {
    //         // 停止状态
    //         steering_angle = 0.0;
    //         drive_speed = 0.0;
    //     }
    //     else if (std::abs(v) < 0.001)
    //     {
    //         steering_angle = current_angle_;
    //         double step = 1 * PI / 180;
    //         // 纯旋转
    //         if (omega > 0){
    //             request->voice_code = 2;
    //             voice_client_->async_send_request(request);
    //             steering_angle = current_angle_ + step;
    //             steering_angle = steering_angle > PI/2? PI/2: steering_angle;
    //         }else if(omega < 0){
    //             request->voice_code = 3;
    //             voice_client_->async_send_request(request);
    //             steering_angle = current_angle_ - step;
    //             steering_angle = steering_angle < -PI/2? -PI/2: steering_angle;
    //         }
    //     }
    //     else
    //     {
    //         // 一般运动
    //         steering_angle = std::atan2(wheel_distance_ * omega, v);
    //         drive_speed = v;
            
    //         // 后退
    //         if (180 == steering_angle * 180 / PI){
    //             steering_angle = current_angle_;
                
    //         } // 前进
    //         else if (0 == steering_angle * 180 / PI){
    //             steering_angle = current_angle_;
    //         }
    //         else{
    //             steering_angle = 0;
    //             drive_speed = 0;
    //         }
    //     }

    //     RCLCPP_INFO_STREAM(node_->get_logger(),
    //         "指令处理\t angle: " << steering_angle << "\t" << 
    //                             "velocity: " << drive_speed);
        
    //     // 转化为角度并乘100，转化为epec需要的指令格式
    //     int16_t angle_command = 100 * steering_angle * 180 / PI;
    //     // 转化为epec需要的指令格式。单位：0.01 m/s,
    //     int16_t speed_command = drive_speed * 1000;
    //     speed_command = std::min(speed_command, (int16_t)speed_upper_bound_);
    //     speed_command = std::max(speed_command, (int16_t)-speed_upper_bound_);
    //     // 舵轮转角超过阈值时, 主动降低转动速度
    //     // TODO: 分阶段降低; 参数化
    //     if(std::fabs(angle_command) >= 1000 && std::fabs(speed_command) > 300){
    //         speed_command = speed_command<0?-300:300;
    //     }
    //     // RCLCPP_INFO_STREAM(node_->get_logger(),
    //     //     "实际控制can输出\t angle: " << angle_command << "\t" << 
    //     //                         "velocity: " << speed_command << "\t" << agv_info_.enable);

    //     agv_info_.speed_command = speed_command;
    //     agv_info_.steer_command = angle_command;
    //     // 发送控制can指令
        
    //     frame.can_id = 0x201;
    //     frame.can_dlc = 8;
    //     frame.data[0] = agv_info_.speed_command & 0xff;
    //     frame.data[1] = agv_info_.speed_command >> 8;
    //     frame.data[2] = agv_info_.steer_command & 0xff;
    //     frame.data[3] = agv_info_.steer_command >> 8;
    //     frame.data[4] = 0b00001011; // 默认抱闸状态
    //     frame.data[5] = 10;
    //     frame.data[6] = 10;  
    //     frame.data[7] = 0x00;


    //     if (agv_info_.steer_enable == 0 || agv_info_.drive_enable == 0){
    //         RCLCPP_INFO_STREAM(node_->get_logger(),
    //         "转向和驱动使能" << agv_info_.enable);
    //         frame.data[4] = 0b00001011;
    //         frame.data[0] = 0;
    //         frame.data[1] = 0;
    //         frame.data[2] = 0;
    //         frame.data[3] = 0;
    //     }
    //     if (agv_info_.drive_enable == 1 && agv_info_.speed_command < 1 && agv_info_.enable == 1){
    //         RCLCPP_INFO_STREAM(node_->get_logger(),
    //         "抱闸");
    //         frame.data[4] = 0b00001011;
    //     }
    //     else if (agv_info_.steer_enable == 1 && agv_info_.drive_enable == 1 && agv_info_.speed_command != 0){
    //         RCLCPP_INFO_STREAM(node_->get_logger(),
    //         "释放抱闸");
    //         frame.data[4] = 0b00011011;
    //     }
        

    //     send_frames.push_back(frame);

    //     // 语音播报当前处于自动驾驶状态
    //     frame.can_id = 0x301;
    //     frame.can_dlc = 8;

    //     frame.data[0] = 0;
    //     frame.data[1] = 0;
    //     frame.data[2] = 0x02;
    //     frame.data[3] = 0;
    //     frame.data[4] = 0;
    //     frame.data[5] = 0;
    //     frame.data[6] = 0;
    //     frame.data[7] = 0;

    //     send_frames.push_back(frame);

    //     // 添加到队列
    //     // send_queue_是指针，指向can_send类，push是方法该类自定义的方法，将send_frames放进类变量std::queue<struct ::can_frame> queue_内
    //     send_queue_->push(send_frames);


        
    //     RCLCPP_DEBUG(node_->get_logger(), 
    //                 "收到命令: v=%.2f, ω=%.2f | 转换为: 角度=%.2frad, 速度=%.2fm/s", 
    //                 v, omega, steering_angle, drive_speed);
    // }

    // 监听前进后退档位
    void CanReceiver::gear_cmd_callback(const                                autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg){
        if (msg->command == 20) {
            gear = -1;
        }
        if (msg->command == 22) {
            gear = 1;
        }
    }
    // 这是监听autoware发来的控制命令的回调函数，但是进入自动模式时，autoware也不一定就有控制指令发来，所以驾驶模式的语音播报在receiveTask中额外构造帧
    void CanReceiver::control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg){

        // RCLCPP_INFO_STREAM(node_->get_logger(),
        //     "接收到control_cmd话题\t angle: " << msg->lateral.steering_tire_angle << "\t" << 
        //                         "velocity: " << msg->longitudinal.velocity);

        // 计算挂车后的最大转向角度
        const int trailer_num = trailer_config["trailer_num"];
        const int l = trailer_config["l"];
        const int a = trailer_config["a"];
        const int b = trailer_config["b"];
        const int l1 = trailer_config["l1"];
        double max_trailer_angle = trailer_config["max_trailer_angle"];
        if ( trailer_num > 0 ){
            max_trailer_angle = calculateTrailerAngle(trailer_num, l, a, b, l1);
        }
        
        std::vector<struct can_frame> send_frames;
        struct can_frame frame{};

        double velocity = msg->longitudinal.velocity;  // 这里单位为: m/s
        velocity = velocity * gear;
        double angle = msg->lateral.steering_tire_angle; // 这里单位为: 角度rad
        
        if (std::fabs(angle) > max_trailer_angle){
            if (angle > 0){
                angle = max_trailer_angle;
            }
            else{
                angle = -max_trailer_angle;
            }
        }

        // 转化为角度并乘100，转化为epec需要的指令格式
        int16_t angle_command = 100 * angle * 180 / PI;
        if (angle_command > 9000){
            angle_command = 9000;
        }
        if (angle_command < -9000){
            angle_command = -9000;
        }
        if (angle_command > 6000) {
            angle_command = 6000;
        } else if (angle_command < -6000) {
            angle_command = -6000;
        }

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
        // 左转,角度小于500认为微调，非转向
        if (angle_command < 6001 && angle_command > 500){
            v_frame.data[2] = 2;
            // 左转向灯，第0位设为1
            v_frame.data[0] |= (1 << 0);
        } else{
            v_frame.data[2] = 1;
            // 左转向灯，第0位设为1
            v_frame.data[0] |= (0 << 0);
        }
        // 右转
        if (angle_command > -6001 && angle_command > -500){
            v_frame.data[2] = 3;
            v_frame.data[0] |= (1 << 1);
        } else {
            v_frame.data[2] = 1;
            v_frame.data[0] |= (0 << 1);
        }

        if (velocity < 0){
            // 正在倒车语音
            v_frame.data[2] = 4;
            v_frame.data[3] |= (1 << 1);
        } else {
            v_frame.data[2] = 1;
            v_frame.data[3] |= (0 << 1);
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
        RCLCPP_INFO_STREAM(node_->get_logger(),
            "实际控制can输出\t angle: " << angle_command << "\t" << 
                                "velocity: " << speed_command);
        agv_info_.speed_command = speed_command;

        {
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


        if (agv_info_.steer_enable == 0 || agv_info_.drive_enable == 0){
            RCLCPP_INFO_STREAM(node_->get_logger(),
            "转向和驱动使能" << agv_info_.enable);
            frame.data[4] = 0b00001011;
            frame.data[0] = 0;
            frame.data[1] = 0;
            frame.data[2] = 0;
            frame.data[3] = 0;
        }
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
        struct can_frame frame{}; 

        frame.can_id = 0x301;
        frame.can_dlc = 8;

        frame.data[0] = 0;
        frame.data[1] = 0;
        frame.data[2] = 0x02;
        frame.data[3] = 0;
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = 0;
        frame.data[7] = 0;

        send_frames.push_back(frame);

        // 添加到队列
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
        //     RCLCPP_INFO(node_->get_logger(), "Publishing控制指令...  ");
        // } else {
        //     RCLCPP_INFO(node_->get_logger(), "Publishing控制指令失败...  ");
        // }
        }
        // 测试181
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
            "Calling engage service with: " << engage ? "true" : "false");

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

        // send_frames.push_back(voice_frame);
        send_frames.push_back(safe_frame);
        
        // 使用正确的成员变量名发送
        if (send_queue_) {
            send_queue_->push(send_frames);
            // RCLCPP_INFO(node_->get_logger(), "Publishing安全，自动驾驶语音报文...  ");
            // ULONG result = VCI_Transmit(
            //     gDevType, 
            //     gDevIdx, 
            //     0,                      // 0=CAN0
            //     &send_frames[0],     // 【关键】直接传入 vector 的数据指针，等价于 
            //     static_cast<UINT>(send_frames.size()) // 发送帧的数量
            // );
            
            // // 3. 检查返回值
            // if (result == 1) {
            //     RCLCPP_INFO(node_->get_logger(), "Publishing安全报文...  ");
            // } else {
            //     RCLCPP_INFO(node_->get_logger(), "Publishing安全报文失败...  ");
            // }
            // RCLCPP_WARN(node_->get_logger(), "发送安全控制报文（超时未收到控制消息）");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "send_queue_ 为空，无法发送安全，自动驾驶语音报文报文");
        }

    }


} // namespace can_driver
ros2 bag record /autoware_orientation /sensing/gnss/rtk/nav_sat_fix /rtk_imu/data_raw
ros2 run can_driver can_rtk_node
candump can1 -t a > can_data.log 2>&1 &
awk 'BEGIN{FS=":"} /can1  601/{a++} /can1  602/{b++} END{printf "601: %.1f Hz, 602: %.1f Hz\n", a/10, b/10}' <(timeout 10 candump can1)
ros2 topic pub -r 1 /control/command/control_cmd autoware_control_msgs/msg/Control "
stamp:
  sec: 0
  nanosec: 0
lateral:
  steering_tire_angle: 0.0
  steering_tire_rotation_rate: 0.0
longitudinal:
  velocity: 0.1
  acceleration: 0.5
  jerk: 0.0
"

// ros2 topic pub -1 /control/command/gear_cmd autoware_vehicle_msgs/msg/GearCommand "{stamp: {sec: 0, nanosec: 0}, command: 20}"