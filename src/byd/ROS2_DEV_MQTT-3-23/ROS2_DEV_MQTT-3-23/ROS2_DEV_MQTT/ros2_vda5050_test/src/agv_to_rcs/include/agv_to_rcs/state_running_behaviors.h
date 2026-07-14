/************************************** File Info ****************************************
* @file:       state_running_behaviors.h
* @author:     刘鸿彬
* @date:       2025-01-02
* @version:    V0.0
* @brief:      运行时所有可能会执行的行为的头文件
******************************************************************************************/
# ifndef STATE_RUNNING_BEHAVIORS_H
# define STATE_RUNNING_BEHAVIORS_H

#include <iostream>
#include <cmath>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
# include "agv_bone.h"
# include "tool_math.h"
# include "control_agv_driver.h"
# include "control_execute_action.h"
# include "client_end_guide.h"
# include "subscriber_candata.h"
# include "vda5050_interfaces/msg/action_states.hpp"
# include <thread>
# include <atomic>
# include <mutex>

using namespace BT;
using vda5050_interfaces::msg::ActionStates;

class LaserRunningStateBehaviors : public SyncActionNode
{
public:
    LaserRunningStateBehaviors(const std::string& name, const NodeConfig& config,
                   AGVBone agv_bone);

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }


    /*****************************************************************************************
    * @brief:      处理运行状态下接收到任务的情况
    * @param:      无
    * @return:     返回当前状态处理之后的事件
    * @author:     刘鸿彬
    * @date:       2024-11-11
    * @version:    V0.0
    * @note:       运行状态有两种情况，1、由order触发的，2、由instantAction触发的
    * @note:       运行状态，1、先上报AGV状态，2、判断由哪种任务触发，3、完成对应任务、关闭线程，完成状态跳转
    ------------------------------------------------------------------------------------------
    * @modified:   是否需要进行一个控制，在进行动作的时候将驱动控制关闭，在驱动的时候禁止进行动作？
    * @date:       2024-11-18
    * @version:    V0.0
    * @description:监听控制的形式是通过监听速度，速度为0的时候才能进行动作控制
    * @note:       
    ******************************************************************************************/
    void OnReceiveOrder();

    // You must override the virtual function tick()
    NodeStatus tick() override;

    const double PI = agv_config.PI;
    const double DEGREES_TO_RADIANS = PI / 180.0;
    const double RADIANS_TO_DEGREES = 180.0 / PI;

    // 任务节点索引
    size_t point_index;
    // 当发送完一次任务后，far_point_index一般指向首个特殊点 或者 任务的终点
    size_t far_point_index;

    // 当前目标点是否已经到达
    bool if_reach_point;
    // 调度系统下发任务是否更新
    bool if_order_updated;
    // 表示小车是否刚取货完毕，打算离开 
    bool if_to_leave;
    // 是否正在暂停中
    bool flag_pausing;
    // 标识当前任务是前进还是后退，true表示前进，false表示后退
    bool forward;

    // 即将要发送的多点导航的点集
    std::vector<Point> goal_points_to_driver; 

    Math_Tool math_tool;

private:
    // 数据更新
    bool data_updates();

    // 点类型更新
    void update_points_type();

    // 已改为私有成员 instant_action_messages_
    // 已改为私有成员 order_messages_
    // 已改为私有成员 current_pose_
    int obstacle_wait_time;
    

    std::shared_ptr<InstantActionsListener> instant_action_listener;
    std::shared_ptr<ListenerPose> current_pose_listener;
    std::shared_ptr<OrderListener> order_listener;
    std::shared_ptr<BatteryListener> battery_listener;
    std::shared_ptr<VelocityListener> velocity_listener;
    std::shared_ptr<ObstacleServer> obstacle_server;
    std::shared_ptr<CanDataListener> can_data_listener_;

    // 从 listener/server 获取的数据（私有成员，避免临时创建）
    CurrentPose current_pose_;
    OrderMessages order_messages_;
    InstantActionMessages instant_action_messages_;
    VelocityMessages velocity_messages_;
    BatteryMessages battery_messages_;
    int obstacle_messages_;
    int current_obstacle_avoidance_channels_;
    InterruptOrderMessage interrupt_order_message_;


    // 声明类对象
    // std::shared_ptr<LaserSendMultiPose> send_multi_pose_;
    std::shared_ptr<LaserDriverControl> agv_driver_control;
    std::shared_ptr<LaserExecuteAction> execute_action_;
    std::shared_ptr<AGVDataPublish> agv_data_publish_;
    std::shared_ptr<EndGuideClient> agv_end_guide;
    std::string last_instant_action_order_id;

    // 针对不同点，有不同的精度要求
    const double high_distance_precision = agv_config.high_distance_precision;
    const double high_angle_precision = agv_config.high_angle_precision;

    const double low_distance_precision = agv_config.low_distance_precision;
    const double low_angle_precision = agv_config.low_angle_precision;

    const int fork_action_height = agv_config.fork_action_height;

    PointType target_point_type;

    /*****************************************************************************************
    * @brief:      处理暂停业务
    * @param:      navigation: true表示可能需要暂停导航任务
    * @param:      fork: true表示可能需要暂停货叉动作
    * @param:      battery: true表示可能需要暂停电池动作
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    * @note:       当检测到startPause时，根据参数取消相应的任务；等待stopPause时恢复相应任务
    * @note:       如果navigation为true且恢复了导航，会将结果存储在restored_drive_state_中
    ******************************************************************************************/
    void handle_pause(bool navigation, bool fork);
    
    // 用于存储恢复导航后的状态（由handle_pause方法设置）
    bool restored_drive_state_;

};


// --------------------------------------------------------------------------------------------------------



class QRRunningStateBehaviors : public SyncActionNode
{
public:
    QRRunningStateBehaviors(const std::string& name, const NodeConfig& config,
                   AGVBone agv_bone);

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }


    /*****************************************************************************************
    * @brief:      处理运行状态下接收到任务的情况
    * @param:      无
    * @return:     返回当前状态处理之后的事件
    * @author:     刘鸿彬
    * @date:       2024-11-11
    * @version:    V0.0
    * @note:       运行状态有两种情况，1、由order触发的，2、由instantAction触发的
    * @note:       运行状态，1、先上报AGV状态，2、判断由哪种任务触发，3、完成对应任务、关闭线程，完成状态跳转
    ------------------------------------------------------------------------------------------
    * @modified:   是否需要进行一个控制，在进行动作的时候将驱动控制关闭，在驱动的时候禁止进行动作？
    * @date:       2024-11-18
    * @version:    V0.0
    * @description:监听控制的形式是通过监听速度，速度为0的时候才能进行动作控制
    * @note:       
    ******************************************************************************************/
    void OnReceiveOrder();

    // You must override the virtual function tick()
    NodeStatus tick() override;

    const double PI = agv_config.PI;
    const double DEGREES_TO_RADIANS = PI / 180.0;
    const double RADIANS_TO_DEGREES = 180.0 / PI;

    // 任务节点索引
    size_t point_index;
    double current_qr; //二维码特性
    size_t far_point_index;

    // 当前目标点是否已经到达
    bool if_reach_point;
    // 调度系统下发任务是否更新
    bool if_order_updated;
    // 表示小车是否刚取货完毕，打算离开 
    bool if_to_leave;


    // 即将要发送的多点导航的点集
    std::vector<Point> goal_points_to_driver; 

    Math_Tool math_tool;

private:
    // 数据更新
    bool data_updates();

    // 点类型更新
    void update_points_type();

    // 已改为私有成员 instant_action_messages_
    // 已改为私有成员 order_messages_
    // 已改为私有成员 current_pose_
    int obstacle_wait_time;
    

    std::shared_ptr<InstantActionsListener> instant_action_listener;
    std::shared_ptr<ListenerPose> current_pose_listener;
    std::shared_ptr<OrderListener> order_listener;
    std::shared_ptr<BatteryListener> battery_listener;
    std::shared_ptr<VelocityListener> velocity_listener;
    std::shared_ptr<ObstacleServer> obstacle_server;
    std::shared_ptr<CanDataListener> can_data_listener_;

    // 从 listener/server 获取的数据（私有成员，避免临时创建）
    CurrentPose current_pose_;
    OrderMessages order_messages_;
    InstantActionMessages instant_action_messages_;
    VelocityMessages velocity_messages_;
    BatteryMessages battery_messages_;
    int obstacle_messages_;
    int current_obstacle_avoidance_channels_;
    InterruptOrderMessage interrupt_order_message_;
    

    // 声明类对象
    // std::shared_ptr<LaserSendMultiPose> send_multi_pose_;
    std::shared_ptr<QRDriverControl> agv_driver_control;
    std::shared_ptr<QRExecuteAction> execute_action_;
    std::shared_ptr<AGVDataPublish> agv_data_publish_;
    std::string last_instant_action_order_id;

    // 针对不同点，有不同的精度要求
    const double high_distance_precision = agv_config.high_distance_precision;
    const double high_angle_precision = agv_config.high_angle_precision;

    const double low_distance_precision = agv_config.low_distance_precision;
    const double low_angle_precision = agv_config.low_angle_precision;

    const int fork_action_height = agv_config.fork_action_height;

    PointType target_point_type;

};

# endif