/************************************** File Info ****************************************
* @file:       subscriber_battery.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-21                                         
* @version:    V0.0                                                                              
* @brief:      电池相关数据订阅方
* @note:       类内更新数据，通过get方法获取，避免读写冲突
******************************************************************************************/
# ifndef SUBSCRIBER_BATTERY
# define SUBSCRIBER_BATTERY

# include "rclcpp/rclcpp.hpp"
# include "agv_config.h"
# include <mutex>
# include <chrono>

using namespace std::chrono_literals;

// 3.define node class
class BatteryListener
{
    
public:

    BatteryListener(std::shared_ptr<rclcpp::Node> node);

    // 返回电池数据
    BatteryMessages get_battery_messages();

    // 电池消息接收标识
    bool get_battery;

private:

    std::shared_ptr<rclcpp::Node> node_;

    void do_cb(const BatteryState &msg);

    void timer_callback();

    rclcpp::Subscription<BatteryState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::time_point<std::chrono::steady_clock> last_msg_time_ ;

    // 电池状态结构体（内部存储）
    BatteryMessages battery_messages_;

    // 用于保护所有数据的互斥锁
    std::mutex data_mutex_;
    
};

# endif