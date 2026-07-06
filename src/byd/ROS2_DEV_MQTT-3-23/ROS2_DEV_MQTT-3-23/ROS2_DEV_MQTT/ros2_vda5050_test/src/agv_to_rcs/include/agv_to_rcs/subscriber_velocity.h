/************************************** File Info ****************************************
* @file:       subscriber_velocity.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2025-03-05                                         
* @version:    V0.0                                                                              
* @brief:      速度数据订阅方
* @note:       类内更新数据，通过get方法获取，避免读写冲突
******************************************************************************************/
# ifndef SUBSCRIBER_VELOCITY
# define SUBSCRIBER_VELOCITY

# include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
# include "agv_config.h"
# include <mutex>
# include <chrono>

using nav_msgs::msg::Odometry;
using namespace std::chrono_literals;

// 3.define node class
class VelocityListener
{
    
public:

    VelocityListener(std::shared_ptr<rclcpp::Node> node);

    // 返回速度数据
    VelocityMessages get_velocity_messages();

    // 速度消息接收标识
    bool get_velocity;

private:

    std::shared_ptr<rclcpp::Node> node_;

    void do_cb(const Odometry &msg);

    void timer_callback();

    rclcpp::Subscription<Odometry>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::time_point<std::chrono::steady_clock> last_msg_time_ ;

    // 速度数据结构体（内部存储）
    VelocityMessages velocity_messages_;

    // 用于保护所有数据的互斥锁
    std::mutex data_mutex_;

};

# endif