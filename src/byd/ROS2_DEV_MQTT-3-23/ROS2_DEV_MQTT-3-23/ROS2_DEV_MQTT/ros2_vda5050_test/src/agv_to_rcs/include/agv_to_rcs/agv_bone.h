/************************************** File Info ****************************************
* @file:       agv_bone.h                                                                     
* @author:     刘鸿彬                                                             
* @date:       2024-12-25                                       
* @version:    V0.0                                                                              
* @brief:      该文件用于统一AGV的骨架信息，包括核心数据结构和贯穿AGV运行周期的重要变量
******************************************************************************************/
// agv_event.h
# ifndef AGV_BONE_H
# define AGV_BONE_H

# include "vda5050_interfaces/msg/agv_state.hpp"

# include "agv_config.h"
# include "subscriber_instant_action.h"
# include "subscriber_order.h"
# include "subscriber_current_pose.h"
# include "subscriber_battery.h"
# include "subscriber_velocity.h"
# include "subscriber_candata.h"
# include "server_obstacle.h"
# include "publishers_agv_state.h"

# include <map>
# include <queue>
# include <memory>

// AGV基本信息结构体，用于去除全局变量、以及初始化其他重要类

class AGVBone{
public:
    // 构造函数，完成会成员变量的赋值
    AGVBone(std::shared_ptr<rclcpp::Node> All_nodes,
            std::shared_ptr<InstantActionsListener> Instant_action_listener,
            std::shared_ptr<ListenerPose> Current_pose_listener,
            std::shared_ptr<OrderListener> Order_listener,
            std::shared_ptr<AGVDataPublish> Agv_data_publish,
            std::shared_ptr<BatteryListener> Battery_listener,
            std::shared_ptr<VelocityListener> Velocity_listener,
            std::shared_ptr<CanDataListener> Can_data_listener,
            std::shared_ptr<ObstacleServer> Obstacle_server,
            std::string &Agv_current_state
            ) : 
            agv_all_nodes(All_nodes),
            agv_instant_action_listener(Instant_action_listener),
            agv_current_pose_listener(Current_pose_listener),
            agv_order_listener(Order_listener),
            agv_data_publish(Agv_data_publish),
            agv_battery_listener(Battery_listener),
            agv_velocity_listener(Velocity_listener),
            agv_can_data_listener(Can_data_listener),
            agv_obstacle_server(Obstacle_server),
            agv_current_state(Agv_current_state)
            {}

    // 全局唯一节点，包含所有的ros平台功能节点
    std::shared_ptr<rclcpp::Node> agv_all_nodes;
    // 一些关键的ros2功能节点
    std::shared_ptr<InstantActionsListener> agv_instant_action_listener;
    std::shared_ptr<ListenerPose> agv_current_pose_listener;
    std::shared_ptr<OrderListener> agv_order_listener;
    std::shared_ptr<AGVDataPublish> agv_data_publish;
    std::shared_ptr<BatteryListener> agv_battery_listener;
    std::shared_ptr<VelocityListener> agv_velocity_listener;
    std::shared_ptr<CanDataListener> agv_can_data_listener;
    std::shared_ptr<ObstacleServer> agv_obstacle_server;

    // agv当前状态
    std::string &agv_current_state;

};


# endif