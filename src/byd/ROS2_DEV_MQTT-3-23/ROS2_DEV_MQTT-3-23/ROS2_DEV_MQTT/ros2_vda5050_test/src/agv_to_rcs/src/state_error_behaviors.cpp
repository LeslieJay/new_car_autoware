/************************************** File Info ****************************************
* @file:       state_error_behaviors.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-11                                        
* @version:    V0.0                                                                              
* @brief:      异常状态的头文件
******************************************************************************************/

# include "state_error_behaviors.h"
# include "agv_config.h"

ErrorStateBehaviors::ErrorStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone) :
    SyncActionNode(name, config),
    current_pose_listener(agv_bone.agv_current_pose_listener){}

NodeStatus ErrorStateBehaviors::tick()
{
    // 主逻辑*****************
    std::cout << this->name() << "发生严重错误，需要人工干预！！！" << std::endl;
    OnManualRecovery();
    
    // 收尾工作*****************
    this->config().blackboard->set("fault_code", "NONE");
    return NodeStatus::SUCCESS;
}

void ErrorStateBehaviors::OnManualRecovery(){
    
    //首先等待小车进入手动模式，这个阶段小车先进行无限等待。这里先将条件置为false
    operation_mode = "manual";
    while(operation_mode != "manual") // TODO：后续操作模式需要从can报文里面获取
    {
        std::cout << "小车未进入手动模式，请人工干预" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "小车已进入手动模式，开始人工处理" << std::endl;


    recovery_flag = manual_recovery();
    if(recovery_flag){
        std::cout << "人工处理完毕！准备重新初始化" << std::endl;
        this->config().blackboard->set("last_state", "error");
        this->config().blackboard->set("current_state", "init");
        this->config().blackboard->set("AGV_Event", "init_try");
        this->config().blackboard->set("fault_code", "NONE");
    }
    else{
        std::cout << "人工处理失败，即将进入错误状态！" << std::endl;
        this->config().blackboard->set("last_state", "error");
        this->config().blackboard->set("current_state", "error");
        this->config().blackboard->set("AGV_Event", "manual_recovery_failed");
    }
}

bool ErrorStateBehaviors::manual_recovery(){
    // 人工处理错误状态，执行完毕返回true，否则false
    while(current_pose_listener->get_pose == false || operation_mode != "auto") // TODO：后续操作模式需要从can报文里面获取
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "系统尚未完全恢复，请求人工恢复！");
        // 休眠10s 
        std::this_thread::sleep_for(std::chrono::milliseconds(1000*5));
        operation_mode = "auto";
    }

    return true;
}