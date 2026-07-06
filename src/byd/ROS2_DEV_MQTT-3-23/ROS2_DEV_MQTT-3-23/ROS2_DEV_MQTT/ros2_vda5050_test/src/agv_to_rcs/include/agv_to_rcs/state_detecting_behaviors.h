/************************************** File Info ****************************************
* @file:       state_detecting_behaviors.h                                                                     
* @author:     刘鸿彬                                                             
* @date:       2024-09-27                                         
* @version:    V0.0                                                                              
* @brief:      货位检测状态的头文件
******************************************************************************************/
#ifndef STATE_DETECTING_BEHAVIORS_H
#define STATE_DETECTING_BEHAVIORS_H

#include <iostream>
# include "agv_bone.h"
#include "client_rack_detection.h"

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <mutex>

using namespace BT;

/**
 * 货位检测状态类
 */
class DetectingStateBehaviors : public SyncActionNode {
public:
    DetectingStateBehaviors(const std::string& name, const NodeConfig& config,
                   AGVBone agv_bone);

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    void OnDetecting();

    // You must override the virtual function tick()
    NodeStatus tick() override;

    /*****************************************************************************************
    * @brief:      获取 rack_detection_messages
    * @param:      无
    * @return:     rack_detection_messages 的副本
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    RackDetectionMessages get_rack_detection_messages() const;

private:
    // 货位检测相关数据（私有成员，避免临时创建）
    RackDetectionMessages rack_detection_messages_;
    // 用于保护 rack_detection_messages_ 的互斥锁
    mutable std::mutex rack_detection_mutex_;

    // 从 listener 获取的数据（私有成员，避免临时创建）
    CurrentPose current_pose_;
    OrderMessages order_messages_;
    InstantActionMessages instant_action_messages_;

    // 核心数据的引用、智能指针
    std::shared_ptr<InstantActionsListener> instant_action_listener;
    std::shared_ptr<ListenerPose> current_pose_listener;
    std::shared_ptr<OrderListener> order_listener;

    // 声明一个智能指针变量，管理指向RackDetectionClient类实例的内存
    std::shared_ptr<RackDetectionClient> rack_detection_client;

    // 尝试发送请求的次数
    int connect_times;

};

#endif // STATE_RACK_DETECTION_H