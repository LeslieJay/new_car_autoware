/************************************** File Info ****************************************
* @file:       state_idle_behaviors.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-19                                         
* @version:    V0.0                                                                              
* @brief:      空闲状态的头文件
******************************************************************************************/
// state_idle_behaviors.h
# ifndef STATE_IDLE_BEHAVIORS_H
# define STATE_IDLE_BEHAVIORS_H

# include <iostream>
# include "agv_bone.h"
# include "publishers_agv_state.h"
# include "subscriber_candata.h"
# include <behaviortree_cpp/behavior_tree.h>
# include <behaviortree_cpp/bt_factory.h>

using namespace BT;


class IdleStateBehaviors : public SyncActionNode {
public:
    IdleStateBehaviors(const std::string& name, const NodeConfig& config,
                   AGVBone agv_bone);

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    void OnInitSuccess();

    void OnTaskCompleted();

    // You must override the virtual function tick()
    NodeStatus tick() override;

    bool get_order;


private:
    // 从 listener 获取的数据（私有成员，避免临时创建）
    InstantActionMessages instant_action_messages_;
    OrderMessages order_messages_;
    CurrentPose current_pose_;

    std::shared_ptr<InstantActionsListener> instant_action_listener;
    std::shared_ptr<ListenerPose> current_pose_listener;
    std::shared_ptr<OrderListener> order_listener;
    std::shared_ptr<AGVDataPublish> data_publish;
    std::shared_ptr<CanDataListener> can_data_listener_;

};

# endif