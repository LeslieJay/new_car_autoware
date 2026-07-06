/************************************** File Info ****************************************
* @file:       control_agv_battery.h                                                                  
* @author:     刘鸿彬                                                              
* @date:       2024-11-12                                         
* @version:    V0.0                                                                              
* @brief:      AGV电池控制类
******************************************************************************************/
# ifndef __CONTROL_AGV_BATTERY_H__
# define __CONTROL_AGV_BATTERY_H__

# include "agv_bone.h"
# include "client_request_charging.h"

class AGVBatteryControl{
public:

    AGVBatteryControl(AGVBone agv_bone);

    bool control();

    /*****************************************************************************************
    * @brief:      获取充电任务完成标志
    * @param:      无
    * @return:     充电任务是否完成
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_flag_finish();

    /*****************************************************************************************
    * @brief:      设置充电任务完成标志
    * @param:      flag 标志值
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    void set_flag_finish(bool flag);
    
private:

    std::shared_ptr<BatteryListener> battery_listener_;
    std::shared_ptr<InstantActionsListener> instant_action_listener;

    // 声明一个智能指针变量，管理指向RequestChargingClient类实例的内存
    std::shared_ptr<RequestChargingClient> request_charging_client_;

    // 
    std::shared_ptr<AGVDataPublish> agv_data_publish_;
    
    // 接收RCS传输的即时任务 
    InstantActionMessages instant_action_messages;

    // 从 listener 获取的数据（私有成员，避免临时创建）
    BatteryMessages battery_messages_;

    // 声明发送的数据
    int start_charge;
    int stop_charge;

    // 尝试发送请求的次数
    int connect_times;

    std::shared_ptr<rclcpp::Node> node_;

    // 充电任务完成标志
    bool flag_finish_;
};
# endif