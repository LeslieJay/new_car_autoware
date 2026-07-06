/************************************** File Info ****************************************
* @file:       subscriber_current_pose.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-09                                         
* @version:    V0.0                                                                              
* @brief:      订阅当前agv的位姿，将四元数转化为弧度
* @note:       需要处理开始能够读取到数据，但是中途中断的情况，以及消息未发布的异常情况
* @note:       AGV位姿信息不能简单地在监听者的callback里面对引用变量current_pose进行直接修改，
* @note:       因为该变量被使用频率很高，很容易形成读写冲突。因此采取在需要获取时在更新的方法
******************************************************************************************/
# ifndef SUBSCRIBER_CURRENT_POSE_H
# define SUBSCRIBER_CURRENT_POSE_H
// ROS2依赖库
# include "rclcpp/rclcpp.hpp"
# include "rclcpp/timer.hpp"
# include <cmath>
# include "agv_config.h"


class ListenerPose
{
public:
    // 类的构造函数
    ListenerPose(std::shared_ptr<rclcpp::Node> node);

    // 返回当前位姿
    CurrentPose get_current_pose();

    // 设置当前位姿（用于外部更新，如feedback回调）
    void set_current_pose(const CurrentPose& pose);

    CurrentPose current_pose;

    // 位姿消息接收标识
    bool get_pose;

private:

    // 订阅方绑定函数
    void agv_pose_callback(const PoseType &agv_state);

    // 定时器绑定函数
    void timer_callback();

    // 订阅指针
    rclcpp::Subscription<PoseType>::SharedPtr current_pose_subscription_; 

    // 定时器
    rclcpp::TimerBase::SharedPtr current_pose_timer_;
    
    // 四元数
    double quat_w;
    double quat_z;

    // 位姿
    double current_x;
    double current_y;
    double current_theta;

    // 用于保护所有数据的互斥锁
    std::mutex data_mutex_; 

    // 记录接收到最后一条数据的时间
    std::chrono::time_point<std::chrono::steady_clock> last_msg_time_ ;

    std::shared_ptr<rclcpp::Node> node_;

};


# endif