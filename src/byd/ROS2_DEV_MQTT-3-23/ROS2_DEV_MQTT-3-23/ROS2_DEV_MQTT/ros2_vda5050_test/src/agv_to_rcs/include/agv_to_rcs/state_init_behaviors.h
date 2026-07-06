/************************************** File Info ****************************************
* @file:       state_init_behaviors.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-19                                         
* @version:    V0.0                                                                              
* @brief:      初始化状态的头文件
******************************************************************************************/
// state_init_behaviors.h
# ifndef STATE_INIT_BEHAVIORS_H
# define STATE_INIT_BEHAVIORS_H

#include <iostream>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

using namespace BT;

/**
 * 初始化状态类
 */
class InitStateBehaviors : public SyncActionNode {
public:

    InitStateBehaviors(const std::string& name, const NodeConfig& config);

    // this example doesn't require any port
    static PortsList providedPorts() { return {}; }

    /**
     * 这个函数用于处理该状态下的事件
     * @param param1 AGV_Event类型的常量引用event
     * @param param2 对StateMachine对象的引用stateMachine
     * @return 返回当前状态处理之后的事件
     */
    void OnInitTry();

    // You must override the virtual function tick()
    NodeStatus tick() override;

    // AGV_Event HandleEvent(const AGV_Event& event) override;
private:
    /**
     * 这个函数用于检查相机、雷达、plc是否能够正常连接
     * @return 连接正常返回true，否则false
     */
    bool check_module();
    /**
     * 这个函数用于检查上次任务是否完成
     * @return 完成返回true，否则false
     */
    bool check_last_task();
    /**
     * 私有成员变量，记录状态下任务的处理结果
     */
    bool check_last_task_;
    /**
     * 私有成员变量，记录状态下模块的连接结果
     */
    bool check_module_;
};
# endif