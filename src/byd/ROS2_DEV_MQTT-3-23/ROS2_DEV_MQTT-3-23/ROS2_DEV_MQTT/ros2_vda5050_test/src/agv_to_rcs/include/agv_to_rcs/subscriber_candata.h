/************************************** File Info ****************************************
* @file:       subscriber_candata.h                                                                     
* @author:     Auto                                                              
* @date:       2025-01-XX                                         
* @version:    V0.0                                                                              
* @brief:      CAN数据订阅方（IO数据、错误数据、硬件数据）
* @note:       类内更新数据，通过get方法获取，避免读写冲突
******************************************************************************************/
# ifndef SUBSCRIBER_CANDATA
# define SUBSCRIBER_CANDATA

# include "rclcpp/rclcpp.hpp"
# include "ref_slam_interface/msg/io_data.hpp"
# include "ref_slam_interface/msg/err_data.hpp"
# include "ref_slam_interface/msg/hardware_data.hpp"
# include "agv_interfaces/msg/automatic_switch.hpp"
# include "agv_config.h"
# include <mutex>
# include <chrono>
#include <vda5050_interfaces/msg/error.hpp>

using namespace std::chrono_literals;

// CAN数据监听器类
class CanDataListener
{
    
public:

    CanDataListener(std::shared_ptr<rclcpp::Node> node);

    // 返回IO数据
    ref_slam_interface::msg::IoData get_io_data();

    // 返回错误数据
    ref_slam_interface::msg::ErrData get_err_data();

    // 返回硬件数据
    ref_slam_interface::msg::HardwareData get_hardware_data();

    // 获取操作模式
    bool get_operation_mode();

    // IO数据接收标识
    bool get_io_data_flag;

    // 错误数据接收标识
    bool get_err_data_flag;

    // 硬件数据接收标识
    bool get_hardware_data_flag;

    // IO数据是否异常
    bool io_data_is_error;

    // 错误数据是否异常
    bool err_data_is_error;

    // 硬件数据是否异常
    bool hardware_data_is_error;

    // 是否需要更新载货状态  0：无效  1：强制有货  2：强制无货  3：不变
    int update_load_status;

    // 二维码自动切换接收标识
    bool get_qr_automatic_switch_flag;

private:

    std::shared_ptr<rclcpp::Node> node_;

    // IO数据回调函数
    void io_data_callback(const ref_slam_interface::msg::IoData &msg);

    // 错误数据回调函数
    void err_data_callback(const ref_slam_interface::msg::ErrData &msg);

    // 硬件数据回调函数
    void hardware_data_callback(const ref_slam_interface::msg::HardwareData &msg);
    
    // vcu错误回调函数
    void error_callback(const vda5050_interfaces::msg::Error &msg);

    // 二维码自动切换回调函数
    void qr_automatic_switch_callback(const agv_interfaces::msg::AutomaticSwitch &msg);

    // 定时器回调函数
    void timer_callback();

    // 订阅者
    rclcpp::Subscription<ref_slam_interface::msg::IoData>::SharedPtr io_data_subscription_;
    rclcpp::Subscription<ref_slam_interface::msg::ErrData>::SharedPtr err_data_subscription_;
    rclcpp::Subscription<ref_slam_interface::msg::HardwareData>::SharedPtr hardware_data_subscription_;
    rclcpp::Subscription<agv_interfaces::msg::AutomaticSwitch>::SharedPtr qr_automatic_switch_subscription_;
    rclcpp::Subscription<vda5050_interfaces::msg::Error>::SharedPtr vcu_subscription_;
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 最后接收消息的时间
    std::chrono::time_point<std::chrono::steady_clock> last_io_data_time_;
    std::chrono::time_point<std::chrono::steady_clock> last_err_data_time_;
    std::chrono::time_point<std::chrono::steady_clock> last_hardware_data_time_;
    std::chrono::time_point<std::chrono::steady_clock> last_qr_automatic_switch_time_;

    // 数据存储（内部存储）
    ref_slam_interface::msg::IoData io_data_;
    ref_slam_interface::msg::ErrData err_data_;
    ref_slam_interface::msg::HardwareData hardware_data_;

    // 二维码数据
    agv_interfaces::msg::AutomaticSwitch qr_automatic_switch_;

    unsigned int vcu_watch_dog_out_count;

    // 用于保护所有数据的互斥锁
    std::mutex data_mutex_;
    
};

# endif

