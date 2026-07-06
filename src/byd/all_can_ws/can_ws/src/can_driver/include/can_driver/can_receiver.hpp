/*
 * @Author: du.xiaoying1
 * @Date: 2025-08-12 17:22:59
 * @LastEditors: dxy
 * @LastEditTime: 2025-08-19 15:58:12
 * @FilePath: /qr_agv_0627_r/src/can_driver/include/can_driver/can_receiver.hpp
 * @Description:
 *
 * Copyright (c) 2025 by du.xiaoying1 , All Rights Reserved.
 */
// include/can_driver/can_receiver.hpp
#ifndef CAN_DRIVER__CAN_RECEIVER_HPP_
#define CAN_DRIVER__CAN_RECEIVER_HPP_
#include <iostream>
#include <string>
#include <thread>
#include <bitset>
#include <atomic>
#include <cstring> // 用于memcpy
#include <queue>
#include <random>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdexcept>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include <poll.h> 
#include <queue>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <atomic>
#include <iomanip>   // for std::put_time
#include "can_driver/agv_config.h"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "agv_interfaces/action/tray_control.hpp"
#include "agv_interfaces/srv/battery_control.hpp"
#include "agv_interfaces/msg/battery_state.hpp"
#include "agv_interfaces/msg/tray_state.hpp"
#include "agv_interfaces/msg/differential_wheel.hpp"
#include "can_driver/can_send.hpp"

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "tier4_external_api_msgs/srv/engage.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <autoware_control_msgs/msg/safety_state.hpp>
#include "usbcan/controlcan.h"
# include "vda5050_interfaces/msg/agv_state.hpp"
#include "vda5050_interfaces/msg/error.hpp"
#include "usbcan/usbcan_utils.hpp"

#define WHEELBASE 1.1
#define PI 3.14159265358979323846
extern std::shared_ptr<can_driver::CanSend> send_queue_;
enum class TURN_STATE: int
{
  NO_COMMAND = 0,
  DISABLE = 1,
  ENABLE_LEFT = 2,
  ENABLE_RIGHT = 3,
};


using agv_interfaces::msg::BatteryState;
using agv_interfaces::msg::DifferentialWheel;


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
// 双轮差速
namespace can_driver
{
   
    class CanReceiver 
    {

    public:
        CanReceiver(std::shared_ptr<rclcpp::Node> node);
        ~CanReceiver();

        rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr
        getSteeringPub() const
        {
            return steering_publisher_;
        }
        rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr
        getVelocityPub() const
        {
            return velocity_publisher_;
        }

        rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
        getControlModePub() const
        {
            return control_mode_publisher_;
        }
      
        /**
         * @brief 初始化 CAN 接收器，创建并绑定 socket。
         *
         * @param interface_name 要监听的 CAN 接口名称 (如 "can0", "can1")
         * @param[out] socket_handle 初始化成功的 socket 文件描述符。如果失败，其值未定义。
         * @return true 初始化成功
         * @return false 初始化失败
         */
        // bool initialize(const std::string &interface_name, int &socket_handle);
        // void stop();
        bool isRunning() const;
        void receiveTask(int handle, const std::string &interface_name);
        bool setCanFilter(int socket, const std::vector<uint32_t> &accept_ids, bool is_extended = false);

        std::string decimalToHexString(int32_t value);
        void decimalToHexBytes(int32_t value, uint8_t bytes[4]);
        int toDecimal(int high, int low);
        // 速度和距离系数默认值都是1.0
        double speed_Scale_para = 1.0008;
        double distance_Scale_para = 1.0;

        int speed_upper_bound_; // 舵轮转动线速度上限
        double wheel_distance_;
        double engage_srv_wait_timeout_; 
        double engage_srv_call_timeout_;

    private:
            // 记录 0x181 报文相关
std::string record_file_path_ = "/home/nvidia/autoware/0x181_records.csv"; // 可配置
std::ofstream record_file_;
std::queue<std::string> record_queue_;
std::mutex record_queue_mutex_;
std::condition_variable record_cv_;
std::thread record_write_thread_;
std::atomic<bool> record_running_{false};

void initRecording();          // 启动时检查裁剪，启动写线程
void writeRecordToFile();      // 写线程函数
void pushRecord(const can_frame &frame, double angle, double speed);
        // --- 速度日志记录相关 ---
        std::string log_file_path_ = "velocity_command_log.csv";
        std::ofstream log_file_;
        std::vector<std::string> log_buffer_;
        int log_line_count_ = 0;
        static constexpr size_t BUFFER_FLUSH_SIZE = 1000;
        static constexpr int MAX_LOG_LINES = 100000;

        void flushLogBuffer();
        void truncateLogFile();
        void openLogFile();   // 仅在构造时调用一次
        rclcpp::Time last_control_time_;
        std::atomic<bool> running_{false};

        rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_publisher_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_publisher_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_publisher_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_report_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_NavSatFix_publisher_;
        rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr rtk_GnssInsOrientationStamped_publisher_;
        rclcpp::Publisher<agv_interfaces::msg::BatteryState>::SharedPtr battery_publisher_;
        rclcpp::Publisher<vda5050_interfaces::msg::Error>::SharedPtr error_publisher_;


        rclcpp::Subscription<vda5050_interfaces::msg::AGVState>::SharedPtr agv_state_subscript_;
        rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_subscript_;
        rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_cmd_subscript_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr keyboard_subscript_;;
        rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr gear_subscript_;
        rclcpp::Subscription<autoware_control_msgs::msg::SafetyState>::SharedPtr car_instance_subscript_;
        rclcpp::Subscription<autoware_control_msgs::msg::SafetyState>::SharedPtr person_instance_subscript_;
        // engage 客户端
        rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr engage_client_;
        void agv_state_callback(const vda5050_interfaces::msg::AGVState::ConstSharedPtr msg);

        void control_cmd_callback(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
        void gear_cmd_callback(const autoware_vehicle_msgs::msg::GearCommand::ConstSharedPtr msg);
        void publishGearStatus(uint8_t report);
        void turn_cmd_callback(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
        void keyboard_cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
        void person_instance_callback(const autoware_control_msgs::msg::SafetyState::ConstSharedPtr msg);
        void car_instance_callback(const autoware_control_msgs::msg::SafetyState::ConstSharedPtr msg);
        void sendSafetyFrameCallback();
        void CreateSafetyFrame();

        // engage_调用服务
        void call_engage_service(bool engage);
        void call_engage_service_async(bool engage);

        void handle_engage_response(rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture future, bool expected_engage);
        
        

        AGVInfo agv_info_;

        BatteryInfo battery_info_;
        agv_interfaces::msg::BatteryState battery_msg;
        agv_interfaces::msg::DifferentialWheel real_differentialwheel_vel_msg;



        double current_angle_ = 0;
        std::shared_ptr<rclcpp::Node> node_;
        // 每1分钟只向cu发送一次前进和自动驾驶的语音播报，防止其他语音被淹没
        struct can_frame voice_frame{}; 
        struct can_frame safe_frame{};
        rclcpp::TimerBase::SharedPtr safety_timer_;
        int gear = 1;
        uint8_t current_gear_report_ =
          autoware_vehicle_msgs::msg::GearReport::DRIVE;
    };

} // namespace can_driver
extern int hook;

#endif // CAN_DRIVER__CAN_RECEIVER_HPP_