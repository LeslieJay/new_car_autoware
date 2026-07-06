// src/can_receiver.cpp

#include "can_driver/can_receiver.hpp"
#include "can_driver/can_node.hpp"
#include "can_driver/trailer.hpp"
#include <rclcpp/rclcpp.hpp>                     // 用于 rclcpp::Publisher
#include <iostream>
#include <ctime>          // for struct tm, timegm
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

// 假设你在一个类成员函数中，拥有 node_ 指针 (rclcpp::Node::SharedPtr)
// 或者你可以传入 node 指针

builtin_interfaces::msg::Time get_current_ros_time(rclcpp::Node::SharedPtr node) {
    // now() 会自动判断是否启用了 use_sim_time 参数
    // 如果启用，它返回 /clock 话题的时间；否则返回系统时间
    rclcpp::Time ros_time = node->now(); 
    
    return ros_time; // rclcpp::Time 可以隐式转换或直接用于 msg 赋值
}
builtin_interfaces::msg::Time toRosTime(
    uint16_t year, uint8_t month, uint8_t day,
    uint8_t hour, uint8_t minute, uint64_t milliseconds)
{
    // 1️⃣ 构建当天 0 点的 tm
    std::tm tm_zero{};
    tm_zero.tm_year = year - 1900;
    tm_zero.tm_mon  = month - 1;
    tm_zero.tm_mday = day;
    tm_zero.tm_hour = 0;
    tm_zero.tm_min  = 0;
    tm_zero.tm_sec  = 0;

    // 2️⃣ 得到当天 0 点的 UTC 秒数
    std::time_t day_start = timegm(&tm_zero);

    // 3️⃣ 计算总秒和纳秒
    std::time_t total_sec = day_start + hour*3600 + minute*60 + milliseconds/10000;
    uint32_t nanosec = (milliseconds % 10000) * 100000ULL;

    // 4️⃣ 填充 ROS 时间戳
    builtin_interfaces::msg::Time ros_time;
    ros_time.sec = static_cast<int32_t>(total_sec);
    ros_time.nanosec = nanosec;
    return ros_time;
}


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
/*
//旧版实现：直接按 setRPY(roll, pitch, yaw_ros) 计算。
//该实现没有按厂家给出的 y -> x -> -z 自轴顺序使用原始姿态定义。
geometry_msgs::msg::Quaternion attiToQuaternion(const int64_t atti[3])
{
    const double DEG_TO_RAD = M_PI / 180.0;

    double roll  = static_cast<double>(atti[1]) / INT_ATTI * DEG_TO_RAD;
    double pitch = static_cast<double>(atti[0]) / INT_ATTI * DEG_TO_RAD;
    double heading_deg = static_cast<double>(atti[2]) / INT_ATTI;
    double yaw_deg = 90.0 - heading_deg;

    // while (yaw_deg > 180.0)  yaw_deg -= 360.0;
    // while (yaw_deg < -180.0) yaw_deg += 360.0;

    tf2::Quaternion tf_q;
    tf_q.setRPY(roll, pitch, yaw_deg * DEG_TO_RAD);

    geometry_msgs::msg::Quaternion ros_q;
    ros_q.x = tf_q.x();
    ros_q.y = tf_q.y();
    ros_q.z = tf_q.z();
    ros_q.w = tf_q.w();
    return ros_q;
}
*/

namespace
{
double quaternionNorm(const double q[4])
{
    return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}
}

geometry_msgs::msg::Quaternion attiToQuaternion(const int64_t atti[3])
{
    const double DEG_TO_RAD = M_PI / 180.0;
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    // 按协议缩放恢复角度，厂家定义的顺序为 [pitch, roll, yaw_north_cw]
    const double pitch_deg = static_cast<double>(atti[0]) / INT_ATTI;
    const double roll_deg = static_cast<double>(atti[1]) / INT_ATTI;
    const double yaw_north_cw_deg = static_cast<double>(atti[2]) / INT_ATTI;
    const double pitch = static_cast<double>(atti[0]) / INT_ATTI * DEG_TO_RAD;
    const double roll = static_cast<double>(atti[1]) / INT_ATTI * DEG_TO_RAD;
    const double yaw_north_cw = static_cast<double>(atti[2]) / INT_ATTI * DEG_TO_RAD;

    RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("atti_convert"),
        steady_clock,
        1000,
        "原始解析欧拉角 pitch=%.8f deg, roll=%.8f deg, yaw_north_cw=%.8f deg",
        pitch_deg,
        roll_deg,
        yaw_north_cw_deg);

    // 先按原始 y -> x -> -z 自轴顺序公式计算 q_vendor，数组顺序为 [w, x, y, z]
    const double sina = std::sin(pitch * 0.5);
    const double sinb = std::sin(roll * 0.5);
    const double sinc = std::sin(yaw_north_cw * 0.5);
    const double cosa = std::cos(pitch * 0.5);
    const double cosb = std::cos(roll * 0.5);
    const double cosc = std::cos(yaw_north_cw * 0.5);

    double q_vendor_array[4];
    q_vendor_array[0] = cosa * cosb * cosc + sina * sinb * sinc;
    q_vendor_array[1] = sina * cosb * cosc + cosa * sinb * sinc;
    q_vendor_array[2] = cosa * sinb * cosc - sina * cosb * sinc;
    q_vendor_array[3] = sina * sinb * cosc - cosa * cosb * sinc;

    const double norm = quaternionNorm(q_vendor_array);
    if (norm > 1e-12) {
        q_vendor_array[0] /= norm;
        q_vendor_array[1] /= norm;
        q_vendor_array[2] /= norm;
        q_vendor_array[3] /= norm;
    }

    tf2::Quaternion q_vendor(
        q_vendor_array[1],
        q_vendor_array[2],
        q_vendor_array[3],
        q_vendor_array[0]);

    // 将“北为0、顺时针”为正的世界参考系转换到 ROS2 ENU 的“东为0、逆时针”为正
    tf2::Quaternion q_ref;
    q_ref.setRotation(tf2::Vector3(0.0, 0.0, 1.0), M_PI_2);

    tf2::Quaternion q_ros = q_ref * q_vendor;
    q_ros.normalize();

    geometry_msgs::msg::Quaternion ros_q;
    ros_q.x = q_ros.x();
    ros_q.y = q_ros.y();
    ros_q.z = q_ros.z();
    ros_q.w = q_ros.w();
    return ros_q;
}
// geometry_msgs::msg::Quaternion attiToQuaternion(const int64_t atti[3])
// {
//     const double DEG_TO_RAD = M_PI / 180.0;

//     // ====================== 输入角度转换 ======================
//     double pitch = static_cast<double>(atti[0]) / INT_ATTI * DEG_TO_RAD;  // 俯仰
//     double roll  = static_cast<double>(atti[1]) / INT_ATTI * DEG_TO_RAD;  // 横滚
//     double yaw   = static_cast<double>(atti[2]) / INT_ATTI * DEG_TO_RAD;  // 航向

//     // ====================== 图片里的标准算法 ======================
//     double cy = cos(yaw * 0.5);
//     double sy = sin(yaw * 0.5);
//     double cr = cos(roll * 0.5);
//     double sr = sin(roll * 0.5);
//     double cp = cos(pitch * 0.5);
//     double sp = sin(pitch * 0.5);

//     // 完全按照你图片公式计算
//     double qw = cp * cr * cy + sp * sr * sy;
//     double qx = sp * cr * cy + cp * sr * sy;
//     double qy = cp * sr * cy - sp * cr * sy;
//     double qz = sp * sr * cy - cp * cr * sy;

//     // 归一化
//     double norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
//     qw /= norm;
//     qx /= norm;
//     qy /= norm;
//     qz /= norm;

//     // 输出 ROS 四元数
//     geometry_msgs::msg::Quaternion ros_q;
//     ros_q.w = qw;
//     ros_q.x = qx;
//     ros_q.y = qy;
//     ros_q.z = qz;

//     // ====================== 日志打印 ======================
//     RCLCPP_INFO(rclcpp::get_logger("atti_convert"), "航向角(yaw)  %f rad", yaw);
//     RCLCPP_INFO(rclcpp::get_logger("atti_convert"), "俯仰角(pitch)%f rad", pitch);
//     RCLCPP_INFO(rclcpp::get_logger("atti_convert"), "横滚角(roll) %f rad", roll);

//     // ====================== 写入文件 ======================
//     std::ofstream ofs;
//     ofs.open("/media/f/nvme_storage/can_ws/atti_quat_log.txt", std::ios::app);

//     if (ofs.is_open())
//     {
//         char time_buf[64];
//         time_t now = time(nullptr);
//         strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", localtime(&now));

//         ofs << "=============================================" << std::endl;
//         ofs << "时间: " << time_buf << std::endl;

//         ofs << "【原始输入】" << std::endl;
//         ofs << "atti[0] = " << atti[0] << std::endl;
//         ofs << "atti[1] = " << atti[1] << std::endl;
//         ofs << "atti[2] = " << atti[2] << std::endl;

//         ofs << "【转换后弧度】" << std::endl;
//         ofs << "pitch = " << pitch << std::endl;
//         ofs << "roll  = " << roll  << std::endl;
//         ofs << "yaw   = " << yaw   << std::endl;

//         ofs << "【输出四元数】" << std::endl;
//         ofs << "w = " << ros_q.w << std::endl;
//         ofs << "x = " << ros_q.x << std::endl;
//         ofs << "y = " << ros_q.y << std::endl;
//         ofs << "z = " << ros_q.z << std::endl;
//         ofs << "=============================================\n" << std::endl;

//         ofs.close();
//     }

//     return ros_q;
// }





namespace can_driver
{

    CanReceiver::CanReceiver(std::shared_ptr<rclcpp::Node> node,std::shared_ptr<CanSend> send_queue)
        : node_(node), running_(false)
    {
        send_queue_ = send_queue;
        // 实例通过调用->publish方法发布数据
        steering_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 10);
        velocity_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 10);
        control_mode_publisher_ = node_->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 10);
        rclcpp::QoS qos(rclcpp::KeepLast(10));  // 高频传感器数据不需要大队列
        qos.best_effort(); // 高速GNSS数据用best_effort
        rtk_NavSatFix_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/sensing/gnss/rtk/nav_sat_fix", qos);
        imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("/rtk_imu/data_raw", qos);

        rtk_GnssInsOrientationStamped_publisher_ = node_->create_publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>("/autoware_orientation", 10);
        
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
    /**
     * @brief 接受报文
     *
     * @param handle socket返回的文件描述符
     * @param interface_name can接口名
     */

void CanReceiver::receiveTask(int handle, const std::string &interface_name)
{
    running_ = true;
    struct canfd_frame frame{};

    while (running_ && rclcpp::ok())
    {
        int nbytes = read(handle, &frame, sizeof(frame));
        if (nbytes <= 0) {
            std::this_thread::sleep_for(1ms);
            continue;
        }

        CanFrame can_frame;
        can_frame.frameId = frame.can_id;
        can_frame.dataLen = frame.len;
        std::memcpy(can_frame.data, frame.data, frame.len);

        std::lock_guard<std::mutex> lock(queue_mutex_);
        frame_queue_.push_back(can_frame); // deque 用 push_back
        cv_.notify_all();
    }
}
// void CanReceiver::publishNavSatFixTask()
// {
//     std::deque<CanFrame> frames_to_publish;
    
//     // 🛡️ 新增：记录上一次成功发布 0x601 的时间戳
//     rclcpp::Time last_pub_time_603(0, 0, RCL_ROS_TIME);
//     // 10ms 的时间间隔限制 (100Hz)
//     const rclcpp::Duration min_interval = rclcpp::Duration::from_seconds(0.010); 

//     while (running_ && rclcpp::ok())
//     {
//         {
//             std::unique_lock<std::mutex> lock(queue_mutex_);
//             cv_.wait(lock, [this]{ return !frame_queue_.empty() || !running_; });
//             if (!running_ && frame_queue_.empty()) break;

//             for (auto it = frame_queue_.begin(); it != frame_queue_.end(); ) {
//                 if (it->frameId == 0x601) {
//                     frames_to_publish.push_back(*it);
//                     it = frame_queue_.erase(it);
//                 } else {
//                     ++it;
//                 }
//             }
//         }

//         if (frames_to_publish.empty()) {
//             continue;
//         }

//         for (auto &frame : frames_to_publish) {
//             if (frame.frameId != 0x601) continue;

//             // 🛡️ 频率控制核心逻辑
//             rclcpp::Time now_time = node_->now();
//             if ((now_time - last_pub_time_603) < min_interval) {
//                 // 如果距离上一帧小于 10ms (换算过来超过 100Hz)，直接抛弃此帧
//                 continue; 
//             }
//             // 满足条件，更新上一次发布的时间
//             last_pub_time_603 = now_time;

//             ins_pos_can_t dataPos;
//             if (frame.dataLen < sizeof(dataPos)) continue;
//             std::memcpy(&dataPos, frame.data, sizeof(dataPos));

//             sensor_msgs::msg::NavSatFix msg;
//             msg.header.stamp = now_time; // 使用判定通过后的时间统一时间戳
//             msg.header.frame_id = "base_link";
//             msg.longitude = static_cast<double>(dataPos.pos_ins[0]) / INT_POS;
//             msg.latitude  = static_cast<double>(dataPos.pos_ins[1]) / INT_POS;
//             msg.altitude  = static_cast<double>(dataPos.pos_ins[2]) / INT_POS;
//             msg.status.status = dataPos.state;
//             if(rtk_NavSatFix_publisher_) rtk_NavSatFix_publisher_->publish(msg);
//         }

//         frames_to_publish.clear();
//     }
// }

// void CanReceiver::publishGnssInsTask()
// {
//     std::deque<CanFrame> frames_to_publish;

//     // 用于控制 1s 打印一次日志
//     rclcpp::Time last_print_time = node_->now();
//     const rclcpp::Duration print_interval = rclcpp::Duration(1, 0); 

//     // 🛡️ 新增：记录上一次成功发布 0x602 的时间戳
//     rclcpp::Time last_pub_time_602(0, 0, RCL_ROS_TIME);
//     // 10ms 的时间间隔限制 (100Hz)
//     const rclcpp::Duration min_interval = rclcpp::Duration::from_seconds(0.010);

//     while (running_ && rclcpp::ok())
//     {
//         {
//             std::unique_lock<std::mutex> lock(queue_mutex_);
//             cv_.wait(lock, [this]{ return !frame_queue_.empty() || !running_; });
//             if (!running_ && frame_queue_.empty()) break;

//             for (auto it = frame_queue_.begin(); it != frame_queue_.end(); ) {
//                 if (it->frameId == 0x602) {
//                     frames_to_publish.push_back(*it);
//                     it = frame_queue_.erase(it);
//                 } else {
//                     ++it;
//                 }
//             }
//         }

//         if (frames_to_publish.empty()) {
//             continue;
//         }

//         for (auto &frame : frames_to_publish) {
//             if (frame.frameId != 0x602) continue;

//             // 🛡️ 频率控制核心逻辑
//             rclcpp::Time now_time = node_->now();
//             if ((now_time - last_pub_time_602) < min_interval) {
//                 // 如果距离上一帧小于 10ms (换算过来超过 100Hz)，直接抛弃此帧
//                 continue; 
//             }
//             // 满足条件，更新上一次发布的时间
//             last_pub_time_602 = now_time;

//             ins_atti_can_t dataPos;
//             if (frame.dataLen < sizeof(dataPos)) continue;
//             std::memcpy(&dataPos, frame.data, sizeof(dataPos));

//             autoware_sensing_msgs::msg::GnssInsOrientationStamped msg;
//             sensor_msgs::msg::Imu imu_msg;
//             msg.header.stamp = now_time;
//             msg.header.frame_id = "imu_link";
//             msg.orientation.orientation = attiToQuaternion(dataPos.atti);

//             // 每 1 秒打印一次姿态状态
//             if (now_time - last_print_time >= print_interval) {
//                 RCLCPP_INFO(node_->get_logger(), " 姿态状态是否可靠 %d ", dataPos.valid);
//                 last_print_time = now_time; 
//             }

//             imu_msg.header.stamp = now_time;
//             imu_msg.header.frame_id = "imu_link";
//             imu_msg.orientation.x = 0.0;
//             imu_msg.orientation.y = 0.0;
//             imu_msg.orientation.z = 0.0;
//             imu_msg.orientation.w = 1.0;
//             imu_msg.orientation_covariance.fill(0.0);
//             imu_msg.orientation_covariance[0] = -1.0;
//             const double DEG_TO_RAD = M_PI / 180.0;
//             imu_msg.angular_velocity.x = static_cast<double>(dataPos.gyro_xyz[0]) / INT_IMU * DEG_TO_RAD;
//             imu_msg.angular_velocity.y = static_cast<double>(dataPos.gyro_xyz[1]) / INT_IMU * DEG_TO_RAD;
//             imu_msg.angular_velocity.z = static_cast<double>(dataPos.gyro_xyz[2]) / INT_IMU * DEG_TO_RAD;
//             imu_msg.linear_acceleration.x = dataPos.acc_xyz[0]/INT_IMU *9.8;
//             imu_msg.linear_acceleration.y = dataPos.acc_xyz[1]/INT_IMU *9.8;
//             imu_msg.linear_acceleration.z = dataPos.acc_xyz[2]/INT_IMU *9.8;
            
//             if(rtk_GnssInsOrientationStamped_publisher_) rtk_GnssInsOrientationStamped_publisher_->publish(msg);
//             if(imu_publisher_) imu_publisher_->publish(imu_msg);
//         }

//         frames_to_publish.clear();
//     }
// }


void CanReceiver::publishNavSatFixTask()
{
    std::deque<CanFrame> frames_to_publish;

    while (running_ && rclcpp::ok())
    {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait(lock, [this]{ return !frame_queue_.empty() || !running_; });
            if (!running_ && frame_queue_.empty()) break;

            // 仅提取并移除 0x601，避免与其他发布线程互相抢空队列。
            for (auto it = frame_queue_.begin(); it != frame_queue_.end(); ) {
                if (it->frameId == 0x601) {
                    frames_to_publish.push_back(*it);
                    it = frame_queue_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (frames_to_publish.empty()) {
            continue;
        }

        for (auto &frame : frames_to_publish) {
            if (frame.frameId != 0x601) continue;
            ins_pos_can_t dataPos;
            // if (frame.dataLen < sizeof(dataPos)) continue;
            std::memcpy(&dataPos, frame.data, sizeof(dataPos));

            sensor_msgs::msg::NavSatFix msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = "base_link";
            msg.longitude = static_cast<double>(dataPos.pos_ins[0]) / INT_POS;
            msg.latitude  = static_cast<double>(dataPos.pos_ins[1]) / INT_POS;
            msg.altitude  = static_cast<double>(dataPos.pos_ins[2]) / INT_POS;
            msg.status.status = dataPos.state;
            if(rtk_NavSatFix_publisher_) rtk_NavSatFix_publisher_->publish(msg);
        }

        frames_to_publish.clear();
    }
}

void CanReceiver::publishGnssInsTask()
{
    std::deque<CanFrame> frames_to_publish;

    // 新增：用于控制 1s 打印一次日志
    rclcpp::Time last_print_time = node_->now();
    const rclcpp::Duration print_interval = rclcpp::Duration(1, 0); // 1 秒

    while (running_ && rclcpp::ok())
    {
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait(lock, [this]{ return !frame_queue_.empty() || !running_; });
            if (!running_ && frame_queue_.empty()) break;

            // 仅提取并移除 0x602，避免与其他发布线程互相抢空队列。
            for (auto it = frame_queue_.begin(); it != frame_queue_.end(); ) {
                if (it->frameId == 0x602) {
                    frames_to_publish.push_back(*it);
                    it = frame_queue_.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (frames_to_publish.empty()) {
            continue;
        }

        for (auto &frame : frames_to_publish) {
            if (frame.frameId != 0x602) continue;
            ins_atti_can_t dataPos;
            if (frame.dataLen < sizeof(dataPos)) continue;
            std::memcpy(&dataPos, frame.data, sizeof(dataPos));

            autoware_sensing_msgs::msg::GnssInsOrientationStamped msg;
            sensor_msgs::msg::Imu imu_msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = "imu_link";
            msg.orientation.orientation = attiToQuaternion(dataPos.atti);

            // 修改：每 1 秒打印一次姿态状态
            rclcpp::Time current_time = node_->now();
            if (current_time - last_print_time >= print_interval) {
                RCLCPP_INFO(node_->get_logger(), " 姿态状态是否可靠 %d ", dataPos.valid);
                last_print_time = current_time; // 更新最后打印时间
            }

            imu_msg.header.stamp = node_->now();
            imu_msg.header.frame_id = "imu_link";
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 1.0;
            imu_msg.orientation_covariance.fill(0.0);
            imu_msg.orientation_covariance[0] = -1.0;
            const double DEG_TO_RAD = M_PI / 180.0;
            imu_msg.angular_velocity.x = static_cast<double>(dataPos.gyro_xyz[0]) / INT_IMU * DEG_TO_RAD;
            imu_msg.angular_velocity.y = static_cast<double>(dataPos.gyro_xyz[1]) / INT_IMU * DEG_TO_RAD;
            imu_msg.angular_velocity.z = static_cast<double>(dataPos.gyro_xyz[2]) / INT_IMU * DEG_TO_RAD;
            imu_msg.linear_acceleration.x = dataPos.acc_xyz[0]/INT_IMU *9.8;
            imu_msg.linear_acceleration.y = dataPos.acc_xyz[1]/INT_IMU *9.8;
            imu_msg.linear_acceleration.z = dataPos.acc_xyz[2]/INT_IMU *9.8;
            if(rtk_GnssInsOrientationStamped_publisher_) rtk_GnssInsOrientationStamped_publisher_->publish(msg);
            if(imu_publisher_) imu_publisher_->publish(imu_msg);
        }


        frames_to_publish.clear();
    }
}






void CanReceiver::setNavSatFixPublisher(const rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr &pub)
{
    rtk_NavSatFix_publisher_ = pub;
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

} // namespace can_driver
