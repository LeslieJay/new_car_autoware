/************************************** File Info ****************************************
* @file:       client_fork_action.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-12-19                                         
* @version:    V0.0                                                                              
* @brief:      动作通信货叉控制客户端
******************************************************************************************/
# ifndef CLIENT_FORK_ACTION_H
# define CLIENT_FORK_ACTION_H

// 标准库引用
# include "rclcpp/rclcpp.hpp"
# include "rclcpp_action/rclcpp_action.hpp"
# include "agv_config.h"
# include <chrono>

// 自定义接口引用
# include "ref_slam_interface/action/ctrl_fork.hpp"

# include "agv_interfaces/action/tray_control.hpp"

// 使用命名空间
using ref_slam_interface::action::CtrlFork;

using agv_interfaces::action::TrayControl;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class LsaerForkActionClient
{

public:
    // 类构造函数
    LsaerForkActionClient(std::shared_ptr<rclcpp::Node> node);

    // 向服务端发送货叉目标
    void send_fork_goal(int to_fork_signal,int goal_fork_height);

    // 向服务端发送取消当前任务
    // void send_fork_cancel();

    // 检查任务是否完成（可选）
    bool is_task_completed() const;

    void cancel_action();

    bool flag_finish;
    bool flag_aborted;
    bool flag_canceled;
    bool flag_driving;

private:

    std::shared_ptr<rclcpp::Node> node_;

    // 动作通信的三个反馈函数（任务是否被接受、过程中反馈、结果反馈）
    void fork_goal_response_callback(rclcpp_action::ClientGoalHandle<CtrlFork>::SharedPtr goal_handle);
    void fork_feedback_callback(rclcpp_action::ClientGoalHandle<CtrlFork>::SharedPtr goal_handle,const std::shared_ptr<const CtrlFork::Feedback> feedback);
    void fork_result_callback(const rclcpp_action::ClientGoalHandle<CtrlFork>::WrappedResult & result);

    // 用于存储当前正在处理的动作目标句柄（action goal handle）。
    // 在 ROS 2 中，动作客户端通过目标句柄来管理与动作服务器的交互，包括发送目标、接收反馈和结果，以及取消任务。
    std::shared_future<rclcpp_action::ClientGoalHandle<CtrlFork>::SharedPtr> current_goal_future_;
    rclcpp_action::ClientGoalHandle<CtrlFork>::SharedPtr current_goal_handle_;

    // 货叉控制客户端
    rclcpp_action::Client<CtrlFork>::SharedPtr fork_client_;

    int fork_signal;
    int fork_height;

    // 记录发送任务的时间
    std::chrono::steady_clock::time_point goal_send_time_;

};


class QRForkActionClient
{

public:
    // 类构造函数
    QRForkActionClient(std::shared_ptr<rclcpp::Node> node);

    // 向服务端发送货叉目标
    void send_fork_goal(int to_fork_signal,int goal_fork_height, int goal_fork_rotation);

    // 向服务端发送取消当前任务
    // void send_fork_cancel();

    // 检查任务是否完成（可选）
    bool is_task_completed() const;

    void cancel_action();

    bool flag_finish;
    bool flag_aborted;
    bool flag_canceled;
    bool flag_driving;

private:

    std::shared_ptr<rclcpp::Node> node_;

    // 动作通信的三个反馈函数（任务是否被接受、过程中反馈、结果反馈）
    void fork_goal_response_callback(rclcpp_action::ClientGoalHandle<TrayControl>::SharedPtr goal_handle);
    void fork_feedback_callback(rclcpp_action::ClientGoalHandle<TrayControl>::SharedPtr goal_handle,const std::shared_ptr<const TrayControl::Feedback> feedback);
    void fork_result_callback(const rclcpp_action::ClientGoalHandle<TrayControl>::WrappedResult & result);

    // 用于存储当前正在处理的动作目标句柄（action goal handle）。
    // 在 ROS 2 中，动作客户端通过目标句柄来管理与动作服务器的交互，包括发送目标、接收反馈和结果，以及取消任务。
    std::shared_future<rclcpp_action::ClientGoalHandle<TrayControl>::SharedPtr> current_goal_future_;
    rclcpp_action::ClientGoalHandle<TrayControl>::SharedPtr current_goal_handle_;

    // 货叉控制客户端
    rclcpp_action::Client<TrayControl>::SharedPtr fork_client_;

    int fork_signal;
    int fork_height;
    int fork_rotation_;

    // 记录发送任务的时间
    std::chrono::steady_clock::time_point goal_send_time_;

    // 反馈回调计数器，用于稀释输出
    int feedback_counter_;

};

# endif