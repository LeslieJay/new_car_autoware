/************************************** File Info ****************************************
* @file:       state_idle_behaviors.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-11                                        
* @version:    V0.0                                                                              
* @brief:      agv空闲状态，待完善
******************************************************************************************/
// state_idle_behaviors.cpp
# include "state_idle_behaviors.h"
# include <rclcpp/rclcpp.hpp>

IdleStateBehaviors::IdleStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone) :
    SyncActionNode(name, config),
    instant_action_listener(agv_bone.agv_instant_action_listener),
    current_pose_listener(agv_bone.agv_current_pose_listener),
    order_listener(agv_bone.agv_order_listener),
    data_publish(agv_bone.agv_data_publish),
    can_data_listener_(agv_bone.agv_can_data_listener)
    {}


NodeStatus IdleStateBehaviors::tick()
{
    // 预备工作*****************
    // 更新接收任务标识，表示现在是空闲态，还没接到任务
    // 行为树中的order，order和instantAction都算，并不是单order
    get_order = false;

    // 主逻辑*****************
    std::string AGV_Event;
    if(!this->config().blackboard->get("AGV_Event", AGV_Event))
    {
        RCLCPP_ERROR(rclcpp::get_logger("agv_to_rcs_main"), "Idle: AGV_Event missing on blackboard");
        return NodeStatus::FAILURE;
    }

    if(AGV_Event == "init_success")
    {
        RCLCPP_DEBUG(rclcpp::get_logger("agv_to_rcs_main"), "%s: OnInitSuccess", this->name().c_str());
        OnInitSuccess();
    }
    else if(AGV_Event == "task_completed")
    {
        RCLCPP_INFO(rclcpp::get_logger("agv_to_rcs_main"), "%s: OnTaskCompleted", this->name().c_str());
        OnTaskCompleted();
    }
    
    // 收尾工作*****************
    return NodeStatus::SUCCESS;
}


/*****************************************************************************************
* @brief:      初始化成功后，进行请求上线
* @param:      无
* @return:     返回当前状态处理之后的事件
* @author:     刘鸿彬
* @date:       2024-11-09
* @version:    V0.0
* @note:       连接流程：1、订阅到当前位姿，2、发布请求订阅话题，3、订阅成功，销毁请求连接定时器并等待接受任务
* @note:       空闲有三种情况:1、连接成功前的状态，2、连接成功后，但是未接收到任务的状态，3、任务完成之后
* @note:       该部分在初始化之后，因此为订阅到当前位姿数据，证明订阅位姿部分有问题，跳转至error（目前是直接退出）
******************************************************************************************/
// void IdleStateBehaviors::OnInitSuccess(){

//     std::cout << "AGV空闲状态，等待接受任务！" << std::endl;

//     // 检查can数据是否异常
//     if(can_data_listener_->io_data_is_error || can_data_listener_->err_data_is_error || can_data_listener_->hardware_data_is_error){
//         std::cout << "can数据异常，跳转至lock状态！" << std::endl;
//         this->config().blackboard->set("last_state", "idle");
//         this->config().blackboard->set("current_state", "lock");
//         this->config().blackboard->set("AGV_Event", "state_error");
//         this->config().blackboard->set("fault_code", "CAN_DATA_ERROR");
//         return;
//     }

//     // 更新instantAction消息的数据
//     instant_action_messages_ = instant_action_listener->get_instant_action_messages();

//     // 1、判断是否订阅到当前位姿,订阅到当前位姿数据才进行下一步
//     if(current_pose_listener->get_pose){

//         // 2、请求连接
//         // 两种情况下请求连接:1、RCS未发布intantAction消息，2、rcs发布消息的时间间隔超过10s，则重新请求连接
//         // no_message_received是InstantAction话题超时，发了上线请求后，rcs就应该发送InstantAction
//         while(instant_action_messages_.action_type.empty() || instant_action_messages_.action_type == "no_message_received" || instant_action_messages_.action_type == "agv_offline"){
//             // 定时等待后，更新instant_action_messages_，重新查看是否赋值
//             instant_action_messages_ = instant_action_listener->get_instant_action_messages();
//             // 更新order消息，未收到数据的情况下为填充默认值
//             order_messages_ = order_listener->get_order_messages();
//             // 更新当前位姿数据
//             current_pose_ = current_pose_listener->get_current_pose();
//             // 超10s未连接的情况，如果没有请求连接定时器，则重新创建一个请求连接定时器
//             if(!data_publish->connection_timer_create){
//                 data_publish->publish_connection_request("ONLINE");
//                 data_publish->state_timer_callback();
//             }
//             // 休眠100ms
//             std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         }

//         // 3、连接成功
//         // 关闭请求连接的定时器
//         data_publish->cancel_connection_timer();
//         // 进行任务监听，有两种情况，1、instant_action下发的即时命令，2、order下发的命令
//         // 接收instant_action运行的情况
//         if(instant_action_messages_.action_type == "connect_success_auto"){

//             std::cout << "接收到instant_action命令！" << std::endl;
//             this->config().blackboard->set("last_state", "idle");
//             this->config().blackboard->set("current_state", "running");
//             this->config().blackboard->set("AGV_Event", "receive_order");
//         }
//         // 接收到货位检测任务
//         else if(instant_action_messages_.action_type == "start_rack_detection"){
//             std::cout << "接收到instant_action命令！" << std::endl;
//             this->config().blackboard->set("last_state", "idle");
//             this->config().blackboard->set("current_state", "detecting");
//             this->config().blackboard->set("AGV_Event", "detected");
//         }
//         // 接收order运行的情况
//         else if(order_listener->get_order){
//             std::cout << "接收到order命令！" << std::endl;
//             this->config().blackboard->set("last_state", "idle");
//             this->config().blackboard->set("current_state", "running");
//             this->config().blackboard->set("AGV_Event", "receive_order");
//         }
//         // 连接成功未接收到任务
//         else{
//             sleep(1);
//             // 否则再次进入空闲状态
//             this->config().blackboard->set("last_state", "idle");
//             this->config().blackboard->set("current_state", "idle");
//             this->config().blackboard->set("AGV_Event", "init_success");
//         }
//     }else{
//         std::cout << "订阅当前位姿失败！" << std::endl;
//         // 转换成错误状态
//         this->config().blackboard->set("last_state", "idle");
//         this->config().blackboard->set("current_state", "lock");
//         this->config().blackboard->set("AGV_Event", "state_error");
//         this->config().blackboard->set("fault_code", "NAVIGATION_LOST");
//     }

// }


void IdleStateBehaviors::OnInitSuccess(){

    // 检查can数据是否异常
    if(can_data_listener_->io_data_is_error || can_data_listener_->err_data_is_error || can_data_listener_->hardware_data_is_error){
        RCLCPP_ERROR(rclcpp::get_logger("agv_to_rcs_main"), "Idle: CAN data error, transition to lock");
        this->config().blackboard->set("last_state", "idle");
        this->config().blackboard->set("current_state", "lock");
        this->config().blackboard->set("AGV_Event", "state_error");
        this->config().blackboard->set("fault_code", "CAN_DATA_ERROR");
        return;
    }

    // 更新instantAction消息的数据
    instant_action_messages_ = instant_action_listener->get_instant_action_messages();

    RCLCPP_DEBUG(
        rclcpp::get_logger("agv_to_rcs_main"),
        "Idle: instant_action type=%s order_id=%s",
        instant_action_messages_.action_type.c_str(),
        instant_action_messages_.last_instant_action_order_id.c_str());

    // 1、判断是否订阅到当前位姿,订阅到当前位姿数据才进行下一步
    if(current_pose_listener->get_pose){
        RCLCPP_INFO_ONCE(
            rclcpp::get_logger("agv_to_rcs_main"),
            "Idle: online, waiting for order");

        // 3、连接成功
        // 关闭请求连接的定时器
        data_publish->cancel_connection_timer();
        // 进行任务监听，有两种情况，1、instant_action下发的即时命令，2、order下发的命令
        RCLCPP_DEBUG(
            rclcpp::get_logger("agv_to_rcs_main"),
            "Idle: get_order=%d", get_order);

        if(instant_action_messages_.action_type == "connect_success_auto"){
            RCLCPP_INFO(
                rclcpp::get_logger("agv_to_rcs_main"),
                "Idle: received instant_action connect_success_auto");
            this->config().blackboard->set("last_state", "idle");
            this->config().blackboard->set("current_state", "running");
            this->config().blackboard->set("AGV_Event", "receive_order");
        }
        // 接收order运行的情况
        else if(order_listener->get_order){
            RCLCPP_INFO(rclcpp::get_logger("agv_to_rcs_main"), "Idle: received order");
            this->config().blackboard->set("last_state", "idle");
            this->config().blackboard->set("current_state", "running");
            this->config().blackboard->set("AGV_Event", "receive_order");
        }
        // 连接成功未接收到任务
        else{
            sleep(1);
            // 否则再次进入空闲状态
            this->config().blackboard->set("last_state", "idle");
            this->config().blackboard->set("current_state", "idle");
            this->config().blackboard->set("AGV_Event", "init_success");
        }
    }else{
        RCLCPP_ERROR(rclcpp::get_logger("agv_to_rcs_main"), "Idle: current pose not available");
        // 转换成错误状态
        this->config().blackboard->set("last_state", "idle");
        this->config().blackboard->set("current_state", "lock");
        this->config().blackboard->set("AGV_Event", "state_error");
        this->config().blackboard->set("fault_code", "NAVIGATION_LOST");
    }
}

/*****************************************************************************************
* @brief:      处理任务完成之后的操作
* @param:      无
* @return:     返回当前状态处理之后的事件
* @author:     刘鸿彬
* @date:       2024-11-11
* @version:    V0.0
* @note:       任务完成之后有4种情况：1、接收到即时命令，2、接收到order任务，3、等待，4、下线
******************************************************************************************/
void IdleStateBehaviors::OnTaskCompleted(){

    // 检查can数据是否异常
    if(can_data_listener_->io_data_is_error || can_data_listener_->err_data_is_error || can_data_listener_->hardware_data_is_error){
        RCLCPP_ERROR(rclcpp::get_logger("agv_to_rcs_main"), "Idle(OnTaskCompleted): CAN data error, transition to lock");
        this->config().blackboard->set("last_state", "idle");
        this->config().blackboard->set("current_state", "lock");
        this->config().blackboard->set("AGV_Event", "state_error");
        this->config().blackboard->set("fault_code", "CAN_DATA_ERROR");
        return;
    }
    
    while(!get_order){

        // 休眠1s
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // 更新instantAction消息的数据
        instant_action_messages_ = instant_action_listener->get_instant_action_messages();
        // 更新order消息
        order_messages_ = order_listener->get_order_messages();
        // 更新当前位姿数据
        current_pose_ = current_pose_listener->get_current_pose();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "任务完成，wait for task！");
        // 是否接受到命令
        // if(instant_action_messages_.action_type == "connect_success_auto" || order_messages_.node_size != 0){
        if(instant_action_messages_.action_type == "connect_success_auto" || order_messages_.node_size != 0){
            std::cout << "接收到instant_action命令！" << std::endl;

            get_order = true;

            // 更新状态，跳出循环
            this->config().blackboard->set("last_state", "idle");
            this->config().blackboard->set("current_state", "running");
            this->config().blackboard->set("AGV_Event", "receive_order");
        }
        // 出现错误RCS才会要求下线
        else if(instant_action_messages_.action_type == "agv_offline"){

            std::cout << "接收RCS命令，AGV设备准备关闭！" << std::endl;

            get_order = true;

            // 更新事件，该事件将触发程序结束
            this->config().blackboard->set("last_state", "idle");
            this->config().blackboard->set("current_state", "lock");
            this->config().blackboard->set("AGV_Event", "state_error");
            this->config().blackboard->set("fault_code", "AGV_OFFLINE");
        }
        // 接收order运行的情况
        else if(order_listener->get_order){
            std::cout << "接收到order命令！" << std::endl;
            get_order = true;
            this->config().blackboard->set("last_state", "idle");
            this->config().blackboard->set("current_state", "running");
            this->config().blackboard->set("AGV_Event", "receive_order");
        }
        // 其他情况
        else{
            // 休眠1s
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    };   
}

