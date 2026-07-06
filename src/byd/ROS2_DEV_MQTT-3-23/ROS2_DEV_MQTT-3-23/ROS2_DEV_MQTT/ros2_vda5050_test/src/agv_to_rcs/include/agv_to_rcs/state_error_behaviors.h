/************************************** File Info ****************************************
* @file:       state_error_behaviors.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-11                                        
* @version:    V0.0                                                                              
* @brief:      异常状态的头文件
******************************************************************************************/
# ifndef STATE_ERROR_BEHAVIORS_H
# define STATE_ERROR_BEHAVIORS_H

#include <iostream>
# include <behaviortree_cpp/behavior_tree.h>
# include <behaviortree_cpp/bt_factory.h>
# include "agv_bone.h"

using namespace BT;


/**
 * 这个函数用于状态转换成功时进行处理
 * @return 无
 */
class ErrorStateBehaviors : public SyncActionNode {
public:

    ErrorStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone);

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    void OnManualRecovery();

    // You must override the virtual function tick()
    NodeStatus tick() override;

private:

    bool manual_recovery();

    std::shared_ptr<ListenerPose> current_pose_listener;

    bool recovery_flag;

    std::string operation_mode;
};

# endif