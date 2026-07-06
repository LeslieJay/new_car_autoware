/************************************** File Info ****************************************
* @file:       state_lock_behaviors.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-19                                         
* @version:    V0.0                                                                              
* @brief:      锁死状态的头文件
******************************************************************************************/
# ifndef STATE_LOCK_BEHAVIORS_H
# define STATE_LOCK_BEHAVIORS_H

# include <iostream>
# include <behaviortree_cpp/behavior_tree.h>
# include <behaviortree_cpp/bt_factory.h>
# include "agv_bone.h"
# include "subscriber_candata.h"

using namespace BT;


/**
 * 锁死状态类
 */
class LockStateBehaviors : public SyncActionNode {
public:
    LockStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone);

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    void OnManualRecovery();

    // You must override the virtual function tick()
    NodeStatus tick() override;
    
    // AGV_Event HandleEvent(const AGV_Event& event) override;
private:

    bool manual_recovery();

    void deleteOldBackupFolders();

    std::shared_ptr<ListenerPose> current_pose_listener;

    std::shared_ptr<InstantActionsListener> instant_action_listener;

    std::shared_ptr<AGVDataPublish> data_publish;

    std::shared_ptr<CanDataListener> can_data_listener_;

    bool recovery_flag;

    std::string operation_mode;
};
# endif