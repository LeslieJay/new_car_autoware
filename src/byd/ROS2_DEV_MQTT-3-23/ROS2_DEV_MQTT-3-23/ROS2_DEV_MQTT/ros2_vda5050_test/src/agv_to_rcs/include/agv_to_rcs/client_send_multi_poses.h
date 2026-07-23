/************************************** File Info ****************************************
* @file:       client_send_multi_poses.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-02                                         
* @version:    V0.0                                                                              
* @brief:      发送多个目标点到AGV，重置终点，单点发送会默认一个终点导致行走卡顿
******************************************************************************************/

# ifndef CLIENT_SEND_MULTI_POSES_H
# define CLIENT_SEND_MULTI_POSES_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <thread>
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <atomic>
#include <condition_variable>
#include <vector>
#include "agv_bone.h"
#include "agv_config.h"
#include "ref_slam_interface/srv/use_trajectory.hpp"
#include "subscriber_current_pose.h"
#include "ref_slam_interface/action/autoware_auto.hpp"
#include "agv_interfaces/msg/poses.hpp"
#include "agv_interfaces/action/agv_send.hpp" 

using AutowareAuto = ref_slam_interface::action::AutowareAuto;
using ref_slam_interface::srv::UseTrajectory;
using namespace std::chrono_literals;
using ACTION = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleACTION = rclcpp_action::ClientGoalHandle<AutowareAuto>;

class LaserSendMultiPose
{

public:
    // 类的构造函数，创建动作客户端
    LaserSendMultiPose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ListenerPose> current_pose_listener);

    // 连接服务端，如果连接成功返回true否则返回false
    bool connect_server();

    // 发送数据
    rclcpp::Client<UseTrajectory>::FutureAndRequestId send_request(std::vector<Point> goal_points);

    void send_goal(std::vector<Point> goal_points, bool forward);

    void cancel_action();

    // 导航是否完成，这个标志位初始化成false，防止初始化成true，每次tick的时候重新初始一遍出问题，有点混乱，他的功能可以完全被flag_driving取代
    bool flag_finish;
    bool flag_aborted;
    bool flag_canceled;
    // 是否正在移动
    bool flag_driving;

private:

    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<ListenerPose> current_pose_listener_;
    CurrentPose current_pose_;

    // 导航动作客户端的智能指针，动作客户端。ACTION是 Nav2 的标准动作，无需自己写服务，Nav2会处理
    // rclcpp_action::Client<ACTION>::SharedPtr action_client_;
    // 给autoware下发目的地令其上线
    rclcpp_action::Client<AutowareAuto>::SharedPtr action_client_;
    std::shared_ptr<rclcpp_action::ClientGoalHandle<AutowareAuto>> current_goal_handle_;
    // 使用轨迹客户端声明，服务客户端
    rclcpp::Client<UseTrajectory>::SharedPtr trajectory_client_;

    // 用于保存异步发送目标的返回值
    std::shared_future<GoalHandleACTION::SharedPtr> action_goal_future_;

    std::vector<double> goals_x_;
    std::vector<double> goals_y_;
    std::vector<double> goals_theta_;

    // 四元数
    double quat_w;
    double quat_z;

    // 位姿
    double m_current_x;
    double m_current_y;
    double m_current_theta;

    // 计数器
    int counter;

    void goal_response_callback(GoalHandleACTION::SharedPtr goal_handle);

    void feedback_callback(GoalHandleACTION::SharedPtr goal_handle,const std::shared_ptr<const AutowareAuto::Feedback> feedback);

    void result_callback(const GoalHandleACTION::WrappedResult &result);

};

// --------------------------------------------------------------------------------------------------------------------------------------

using agv_interfaces::action::AGVSend;
using GoalHandleAGVSend = rclcpp_action::ClientGoalHandle<AGVSend>;

class QRSendMultiPose
{

public:
    // 类的构造函数，创建动作客户端
    QRSendMultiPose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ListenerPose> current_pose_listener);

    // 连接服务端，如果连接成功返回true否则返回false
    bool connect_server();

    void send_goal(std::vector<agv_interfaces::msg::Poses> goal_poses);

    void cancel_action();

    std::atomic<int64_t> m_label;
    std::atomic<double> m_angle;
    std::vector<agv_interfaces::msg::Poses> path_poses;

    bool flag_finish;
    bool flag_aborted;
    bool flag_canceled;
    bool flag_driving;

private:

    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<ListenerPose> current_pose_listener_;
    CurrentPose current_pose_;

    // 导航动作客户端的智能指针
    rclcpp_action::Client<AGVSend>::SharedPtr action_client_;

    // 使用轨迹客户端声明 
    rclcpp::Client<UseTrajectory>::SharedPtr trajectory_client_;

    // 用于保存异步发送目标的返回值
    std::shared_future<GoalHandleAGVSend::SharedPtr> action_goal_future_;

    std::vector<double> goals_x_;
    std::vector<double> goals_y_;
    std::vector<double> goals_theta_;

    // 四元数
    double quat_w;
    double quat_z;

    // 位姿
    double m_current_x;
    double m_current_y;
    double m_current_theta;

    // 计数器
    int counter;

    void goal_response_callback(GoalHandleAGVSend::SharedPtr goal_handle);

    void feedback_callback(GoalHandleAGVSend::SharedPtr goal_handle,const std::shared_ptr<const AGVSend::Feedback> feedback);

    void result_callback(const GoalHandleAGVSend::WrappedResult &result);

};

# endif