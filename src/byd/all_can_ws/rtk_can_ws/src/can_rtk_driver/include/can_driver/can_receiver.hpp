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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include "can_driver/can_frame.hpp"
#include <sensor_msgs/msg/imu.hpp>
#define WHEELBASE 1.1
#define PI 3.14159265358979323846

enum class TURN_STATE: int
{
  NO_COMMAND = 0,
  DISABLE = 1,
  ENABLE_LEFT = 2,
  ENABLE_RIGHT = 3,
};


using agv_interfaces::msg::BatteryState;
using agv_interfaces::msg::DifferentialWheel;

constexpr double INT_POS = 1e8;
constexpr double INT_ALT = 1e8;
constexpr double INT_ATTI = 1e8;
constexpr int INT_SEC = 1000;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
// 双轮差速
namespace can_driver
{
    class CanReceiver 
    {

    public:
        CanReceiver(std::shared_ptr<rclcpp::Node> node,std::shared_ptr<CanSend> send_queue);
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
        // CAN 发布线程（按 CAN ID 分类）
        void publishNavSatFixTask();
        void publishGnssInsTask();
        // 设置 Publisher
        void setNavSatFixPublisher(const rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr &pub);

        std::string decimalToHexString(int32_t value);
        void decimalToHexBytes(int32_t value, uint8_t bytes[4]);
        int toDecimal(int high, int low);
        // 速度和距离系数默认值都是1.0
        double speed_Scale_para = 1.0008;
        double distance_Scale_para = 1.0;

        // int speed_upper_bound_; // 舵轮转动线速度上限
        double wheel_distance_;
        double engage_srv_wait_timeout_; 
        double engage_srv_call_timeout_;
        void publishTask();
    private:
        std::atomic<bool> running_{false};



        std::deque<CanFrame> frame_queue_;
        std::mutex queue_mutex_;
        std::condition_variable cv_;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_NavSatFix_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

        rclcpp::Publisher<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr rtk_GnssInsOrientationStamped_publisher_;




        
        rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_publisher_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_publisher_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_publisher_;

        

        AGVInfo agv_info_;

        BatteryInfo battery_info_;
        agv_interfaces::msg::BatteryState battery_msg;
        agv_interfaces::msg::DifferentialWheel real_differentialwheel_vel_msg;



        double current_angle_ = 0;

        std::shared_ptr<can_driver::CanSend> send_queue_;
        std::shared_ptr<rclcpp::Node> node_;
        struct can_frame voice_frame{}; 
        struct can_frame safe_frame{};
        int gear = 1;
    };

} // namespace can_driver

#endif // CAN_DRIVER__CAN_RECEIVER_HPP_