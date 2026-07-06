/******************************** File Info **********************************************
* @file:       control_execute_action.h                                                                   
* @author:     刘鸿彬                                                              
* @date:       2024-11-11                                        
* @version:    V0.0                                                                              
* @brief:      到达目标位姿之后，进行动作判断，判断该点是否有动作，如果有则使用对应的驱动执行，执行完毕或无动作返回true 
******************************************************************************************/

#ifndef CONTROL_EXECUTE_ACTION_H
#define CONTROL_EXECUTE_ACTION_H

# include <string>
# include <map> 
# include <iostream>
# include <mutex>

// vda5050协议接口
# include "vda5050_interfaces/msg/agv_state.hpp"

// 货叉控制
# include "control_agv_fork.h"

// 自定义结构体
# include "agv_bone.h"

// 充电控制
# include "control_agv_battery.h"

class LaserExecuteAction{

public:

    LaserExecuteAction(AGVBone agv_bone);
    /*****************************************************************************************
    * @brief:      到达目标点之后判断该点是否存在动作
    * @param:      1、当前任务动作数量，2、节点id，3、关联容器，包含相同键node_id对应的多个action_id
    * @return:     完成动作或者该点没有动作返回false
    * @author:     刘鸿彬
    * @date:       2024-11-09
    * @version:    V0.0
    ******************************************************************************************/
    int have_action(OrderMessages &order_messages,int point_index);
    /*****************************************************************************************
    * @brief:      执行货叉的动作，粗略完成，后续需要加上动作判断，目前只是发送一个驱动使能和高度，其他写死
    * @param:      货叉目标高度
    * @return:     返回将要执行的动作的id
    * @author:     刘鸿彬
    * @date:       2024-11-09
    * @version:    V0.0
    ******************************************************************************************/

    std::shared_ptr<LaserForkControl> agv_fork_control;

    std::shared_ptr<AGVBatteryControl> agv_battery_control;

    /*****************************************************************************************
    * @brief:      获取 loading 状态
    * @param:      无
    * @return:     loading 的值
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_loading() const;

    /*****************************************************************************************
    * @brief:      设置 loading 状态
    * @param:      loading_value loading 的值（true表示有货，false表示无货）
    * @return:     无
    * @author:     Assistant
    * @date:       2025-01-XX
    * @version:    V1.0
    ******************************************************************************************/
    void set_loading(bool loading_value);

    /*****************************************************************************************
    * @brief:      获取是否有货叉动作的标志
    * @param:      无
    * @return:     如果有货叉动作返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_have_fork_action() const;

    /*****************************************************************************************
    * @brief:      获取是否有电池动作的标志
    * @param:      无
    * @return:     如果有电池动作返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_have_battery_action() const;

    /*****************************************************************************************
    * @brief:      初始化标志（将flag_have_fork_action_和flag_have_battery_action_重置为false）
    * @param:      无
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    void init_flag();

    
private:

    bool flag_have_fork_action_; // 为true时，表示该点有货叉动作
    mutable std::mutex flag_fork_action_mutex_;  // 保护flag_have_fork_action_的互斥锁

    bool flag_have_battery_action_; // 为true时，表示该点有电池动作
    mutable std::mutex flag_battery_action_mutex_;  // 保护flag_have_battery_action_的互斥锁

    bool loading_; // 为true时，表示小车当前正在装货

    mutable std::mutex loading_mutex_;  // 保护loading_的互斥锁
    
};


// -------------------------------------------------------------------------------------------------



class QRExecuteAction{

public:

    QRExecuteAction(AGVBone agv_bone);
    /*****************************************************************************************
    * @brief:      到达目标点之后判断该点是否存在动作
    * @param:      1、当前任务动作数量，2、节点id，3、关联容器，包含相同键node_id对应的多个action_id
    * @return:     完成动作或者该点没有动作返回false
    * @author:     刘鸿彬
    * @date:       2024-11-09
    * @version:    V0.0
    ******************************************************************************************/
    int have_action(OrderMessages &order_messages,int point_index);
    /*****************************************************************************************
    * @brief:      执行货叉的动作，粗略完成，后续需要加上动作判断，目前只是发送一个驱动使能和高度，其他写死
    * @param:      货叉目标高度
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-09
    * @version:    V0.0
    ******************************************************************************************/

    std::shared_ptr<QRForkControl> agv_fork_control;

    std::shared_ptr<AGVBatteryControl> agv_battery_control;

    /*****************************************************************************************
    * @brief:      获取 loading 状态
    * @param:      无
    * @return:     loading 的值
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_loading() const;

    /*****************************************************************************************
    * @brief:      设置 loading 状态
    * @param:      loading_value loading 的值（true表示有货，false表示无货）
    * @return:     无
    * @author:     Assistant
    * @date:       2025-01-XX
    * @version:    V1.0
    ******************************************************************************************/
    void set_loading(bool loading_value);

    /*****************************************************************************************
    * @brief:      获取是否有货叉动作的标志
    * @param:      无
    * @return:     如果有货叉动作返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_have_fork_action() const;

    /*****************************************************************************************
    * @brief:      获取是否有电池动作的标志
    * @param:      无
    * @return:     如果有电池动作返回true，否则返回false
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_have_battery_action() const;

    /*****************************************************************************************
    * @brief:      初始化标志（将flag_have_fork_action_和flag_have_battery_action_重置为false）
    * @param:      无
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    void init_flag();

    
private:

    bool flag_have_fork_action_; // 为true时，表示该点有货叉动作
    mutable std::mutex flag_fork_action_mutex_;  // 保护flag_have_fork_action_的互斥锁

    bool flag_have_battery_action_; // 为true时，表示该点有电池动作
    mutable std::mutex flag_battery_action_mutex_;  // 保护flag_have_battery_action_的互斥锁

    bool loading_; // 为true时，表示小车当前正在装货
    mutable std::mutex loading_mutex_;  // 保护loading_的互斥锁
    
};


# endif