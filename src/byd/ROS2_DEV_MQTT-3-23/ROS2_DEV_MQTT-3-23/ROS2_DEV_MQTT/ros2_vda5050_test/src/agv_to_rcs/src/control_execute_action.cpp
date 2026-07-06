/******************************** File Info **********************************************
* @file:       control_execute_action.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-09                                         
* @version:    V0.0                                                                              
* @brief:      到达目标位姿之后，进行动作判断，判断该点是否有动作，如果有则使用对应的驱动执行，执行完毕或无动作返回true 
******************************************************************************************/

# include "control_execute_action.h"

/*****************************************************************************************
* @brief:      类的构造函数   
* @author:     刘鸿彬
* @date:       2024-11-9
******************************************************************************************/
LaserExecuteAction::LaserExecuteAction(AGVBone agv_bone):
    loading_(false),
    flag_have_fork_action_(false),
    flag_have_battery_action_(false)
{
    agv_fork_control = std::make_shared<LaserForkControl>(agv_bone);
    // 实例化电池控制客户端
    agv_battery_control = std::make_shared<AGVBatteryControl>(agv_bone);

}

bool LaserExecuteAction::get_loading() const {
    std::lock_guard<std::mutex> lock(loading_mutex_);
    return loading_;
}

void LaserExecuteAction::set_loading(bool loading_value) {
    std::lock_guard<std::mutex> lock(loading_mutex_);
    loading_ = loading_value;
}

bool LaserExecuteAction::get_flag_have_fork_action() const {
    std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
    return flag_have_fork_action_;
}

bool LaserExecuteAction::get_flag_have_battery_action() const {
    std::lock_guard<std::mutex> lock(flag_battery_action_mutex_);
    return flag_have_battery_action_;
}

void LaserExecuteAction::init_flag() {
    {
        std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
        flag_have_fork_action_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(flag_battery_action_mutex_);
        flag_have_battery_action_ = false;
    }
}

/*****************************************************************************************
* @brief:      到达目标点之后判断该点是否存在动作
* @param:      解析后的order_messages结构体
* @return:     完成动作或者该点没有动作返回false
* @author:     刘鸿彬
* @date:       2024-11-09
* @note:       1、迭代器查找该点位的所有动作（一个节点有多个动作的情况）
* @note:       2、将查找到的动作和容器中的动作进行对应,如果动作id能够对的上并且该动作处于未完成状态则执行动作
* @note:       3、执行完动作之后改变动作状态，由于参数为引用，因此能够直接改变原值
* @note：      动作的情况分为两种：1、开始的节点就包含动作，2、order命令中的第二个节点包含动作
* @note：      开始节点进行特殊处理，第二个节点为一般处理
* @version:    V0.0
******************************************************************************************/

int LaserExecuteAction::have_action(OrderMessages &order_messages,int point_index){

    std::cout << " 进行动作检查！" << std::endl;

    // 初始化标志
    init_flag();

    int actionID = -1;

    // 用于在关联容器中查找特定键值的所有实例的范围
    // 其中包含两个迭代器：first指向第一个相关动作，second指向最后一个该点动作之后的位置
    auto range = order_messages.action_vec.equal_range(order_messages.msg_state.node_states[point_index].node_id);

    // 遍历range
    for (auto it = range.first; it != range.second; ++it){
        std::cout << "节点 " << order_messages.msg_state.node_states[point_index].node_id << "有动作 " << it->second << std::endl;
            
        // std::cout << "动作容器大小为： " << order_messages.action_size << std::endl;

        int i = 0;
        // 遍历动作容器
        for(i=0;i<order_messages.action_size;i++){
            if(order_messages.msg_state.action_states[i].action_id == it->second)
                break;
        }

        std::cout << "action_type ： " << order_messages.msg_state.action_states[i].action_type.c_str() << std::endl;

        // 找到对应的动作，并且该动作未完成，则执行动作
        if(order_messages.msg_state.action_states[i].action_type == "LOAD" && order_messages.msg_state.action_states[i].action_status != "FINISHED"){
            
            actionID = i;

            // 设置货叉动作标志
            {
                std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
                flag_have_fork_action_ = true;
            }
            
            int base_height = std::stoi(order_messages.msg_state.action_states[i].action_description);
            int fork_height = base_height + agv_config.fork_action_height;
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "LOAD动作计算货叉高度: action_description=%d, fork_action_height=%d, 最终高度=%d", 
                       base_height, agv_config.fork_action_height, fork_height);
            
            agv_fork_control->setForkParameters(1, fork_height);
            bool fork_state = agv_fork_control->control();

            if(fork_state){
                // 结束货叉动作
                std::cout << "正在执行节点 " <<order_messages.msg_state.node_states[point_index].node_id << "的动作 " << order_messages.msg_state.action_states[i].action_id << std::endl;
                // 更新全局msg_state
                order_messages.msg_state.action_states[i].action_status = "RUNNING";
                {
                    std::lock_guard<std::mutex> lock(loading_mutex_);
                    loading_ = true;
                }
            }
            else{
                std::cout << "AGV货叉启动失败！" << std::endl;
            }

        }
        else if(order_messages.msg_state.action_states[i].action_type == "UNLOAD" && order_messages.msg_state.action_states[i].action_status != "FINISHED"){
            
            actionID = i;

            // 设置货叉动作标志
            {
                std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
                flag_have_fork_action_ = true;
            }
            
            int fork_height = std::stoi(order_messages.msg_state.action_states[i].action_description);
            
            agv_fork_control->setForkParameters(1, fork_height);
            bool fork_state = agv_fork_control->control();

            if(fork_state){
                // 结束货叉动作
                std::cout << "正在执行节点 " <<order_messages.msg_state.node_states[point_index].node_id << "的动作 " << order_messages.msg_state.action_states[i].action_id << std::endl;
                // 更新全局msg_state
                order_messages.msg_state.action_states[i].action_status = "RUNNING";
                {
                    std::lock_guard<std::mutex> lock(loading_mutex_);
                    loading_ = false;
                }
            }
            else{
                std::cout << "AGV货叉启动失败！" << std::endl;
            }
            
        }
        else if(order_messages.msg_state.action_states[i].action_type == "CHARGE" && order_messages.msg_state.action_states[i].action_status != "FINISHED"){
            
            actionID = i;

            // 设置电池动作标志
            {
                std::lock_guard<std::mutex> lock(flag_battery_action_mutex_);
                flag_have_battery_action_ = true;
            }
            
            std::cout << "调度系统发布充电任务！" << std::endl;

            bool charge_state = agv_battery_control->control();

            if(charge_state){
                // 结束充电
                std::cout << "执行完节点 " <<order_messages.msg_state.node_states[point_index].node_id << "的动作 " << order_messages.msg_state.action_states[i].action_id << std::endl;
                // 更新全局msg_state
                order_messages.msg_state.action_states[i].action_status = "FINISHED";
            }
            else{

                std::cout << "AGV充电启动失败！" << std::endl;
            }

        }
        else if(order_messages.msg_state.action_states[i].action_type == "WAIT" && order_messages.msg_state.action_states[i].action_status != "FINISHED") {
            double wait_time = std::stod(order_messages.msg_state.action_states[i].action_description);
            std::cout << "等待时长: " << wait_time << " 秒" << std::endl;   
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000*wait_time)));
            order_messages.msg_state.action_states[i].action_status = "FINISHED";
            std::cout << "✅ WAIT动作已完成，action_id: " << order_messages.msg_state.action_states[i].action_id << std::endl;
        }
        else{
            std::cout << "非法动作类型！" << std::endl;
            std::cout << "🔍 调试信息: action_type=" << order_messages.msg_state.action_states[i].action_type 
                      << ", action_status=" << order_messages.msg_state.action_states[i].action_status 
                      << ", action_id=" << order_messages.msg_state.action_states[i].action_id << std::endl;
        }

        
    }
    if(flag_have_fork_action_ || flag_have_battery_action_)
        return actionID;
    else
        return -1;
}


// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


QRExecuteAction::QRExecuteAction(AGVBone agv_bone):loading_(false),
    flag_have_fork_action_(false),
    flag_have_battery_action_(false)
{

    agv_fork_control = std::make_shared<QRForkControl>(agv_bone);
    // 实例化电池控制客户端
    agv_battery_control = std::make_shared<AGVBatteryControl>(agv_bone);
    
}

bool QRExecuteAction::get_loading() const {
    std::lock_guard<std::mutex> lock(loading_mutex_);
    return loading_;
}

void QRExecuteAction::set_loading(bool loading_value) {
    std::lock_guard<std::mutex> lock(loading_mutex_);
    loading_ = loading_value;
}

bool QRExecuteAction::get_flag_have_fork_action() const {
    std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
    return flag_have_fork_action_;
}

bool QRExecuteAction::get_flag_have_battery_action() const {
    std::lock_guard<std::mutex> lock(flag_battery_action_mutex_);
    return flag_have_battery_action_;
}

void QRExecuteAction::init_flag() {
    {
        std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
        flag_have_fork_action_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(flag_battery_action_mutex_);
        flag_have_battery_action_ = false;
    }
}

/*****************************************************************************************
* @brief:      到达目标点之后判断该点是否存在动作
* @param:      解析后的order_messages结构体
* @return:     
* @author:     刘鸿彬
* @date:       2024-11-09
* @note:       1、迭代器查找该点位的所有动作（一个节点有多个动作的情况）
* @note:       2、将查找到的动作和容器中的动作进行对应,如果动作id能够对的上并且该动作处于未完成状态则执行动作
* @note:       3、执行完动作之后改变动作状态，由于参数为引用，因此能够直接改变原值
* @note：      动作的情况分为两种：1、开始的节点就包含动作，2、order命令中的第二个节点包含动作
* @note：      开始节点进行特殊处理，第二个节点为一般处理
* @version:    V0.0
******************************************************************************************/

int QRExecuteAction::have_action(OrderMessages &order_messages,int point_index){

    std::cout << " 进行动作检查！" << std::endl;

    // 初始化标志
    init_flag();

    int actionID = -1;

    // 用于在关联容器中查找特定键值的所有实例的范围
    // 其中包含两个迭代器：first指向第一个相关动作，second指向最后一个该点动作之后的位置
    auto range = order_messages.action_vec.equal_range(order_messages.msg_state.node_states[point_index].node_id);

    // 遍历range
    for (auto it = range.first; it != range.second; ++it){
        std::cout << "节点 " << order_messages.msg_state.node_states[point_index].node_id << "有动作 " << it->second << std::endl;
            
        // std::cout << "动作容器大小为： " << order_messages.action_size << std::endl;

        int i = 0;
        // 遍历动作容器
        for(i=0;i<order_messages.action_size;i++){
            if(order_messages.msg_state.action_states[i].action_id == it->second)
                break;
        }

        std::cout << "action_type ： " << order_messages.msg_state.action_states[i].action_type.c_str() << std::endl;

        // 找到对应的动作，并且该动作未完成，则执行动作
        if(order_messages.msg_state.action_states[i].action_type == "pick" && order_messages.msg_state.action_states[i].action_status != "FINISHED"){
            
            actionID = i;

            // 设置货叉动作标志
            {
                std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
                flag_have_fork_action_ = true;
            }
            
            int fork_height = std::stoi(order_messages.msg_state.action_states[i].action_description) + agv_config.fork_action_height;
            
            agv_fork_control->setForkParameters(1, fork_height, 0);
            bool fork_state = agv_fork_control->control();

            if(fork_state){
                // 结束货叉动作
                std::cout << "执行完节点 " <<order_messages.msg_state.node_states[point_index].node_id << "的动作 " << order_messages.msg_state.action_states[i].action_id << std::endl;
                // 更新全局msg_state
                order_messages.msg_state.action_states[i].action_status = "FINISHED";
                {
                    std::lock_guard<std::mutex> lock(loading_mutex_);
                    loading_ = true;
                }
                
            }
            else{
                std::cout << "AGV货叉启动失败！" << std::endl;
            }

        }
        else if(order_messages.msg_state.action_states[i].action_type == "drop" && order_messages.msg_state.action_states[i].action_status != "FINISHED"){
            
            actionID = i;

            // 设置货叉动作标志
            {
                std::lock_guard<std::mutex> lock(flag_fork_action_mutex_);
                flag_have_fork_action_ = true;
            }
            
            int fork_height = std::stoi(order_messages.msg_state.action_states[i].action_description);
            
            agv_fork_control->setForkParameters(1, fork_height, 0);
            bool fork_state = agv_fork_control->control();

            if(fork_state){
                // 结束货叉动作
                std::cout << "执行完节点 " <<order_messages.msg_state.node_states[point_index].node_id << "的动作 " << order_messages.msg_state.action_states[i].action_id << std::endl;
                // 更新全局msg_state
                order_messages.msg_state.action_states[i].action_status = "FINISHED";
                {
                    std::lock_guard<std::mutex> lock(loading_mutex_);
                    loading_ = false;
                }
                
            }
            else{
                std::cout << "AGV货叉启动失败！" << std::endl;
            }
            
        }
        else if(order_messages.msg_state.action_states[i].action_type == "CHARGE" && order_messages.msg_state.action_states[i].action_status != "FINISHED"){
            
            actionID = i;

            // 设置电池动作标志
            {
                std::lock_guard<std::mutex> lock(flag_battery_action_mutex_);
                flag_have_battery_action_ = true;
            }
            
            std::cout << "调度系统发布充电任务！" << std::endl;

            // 等待小车挺稳后在切换充电模式
            std::this_thread::sleep_for(std::chrono::milliseconds(1000*15));

            bool charge_state = agv_battery_control->control();

            if(charge_state){
                // 结束充电
                std::cout << "执行完节点 " <<order_messages.msg_state.node_states[point_index].node_id << "的动作 " << order_messages.msg_state.action_states[i].action_id << std::endl;
                // 更新全局msg_state
                order_messages.msg_state.action_states[i].action_status = "FINISHED";
                
            }
            else{

                std::cout << "AGV充电启动失败！" << std::endl;
            }

        }
        else if(order_messages.msg_state.action_states[i].action_type == "WAIT" && order_messages.msg_state.action_states[i].action_status != "FINISHED") {
            double wait_time = std::stod(order_messages.msg_state.action_states[i].action_description);
            std::cout << "等待时长: " << wait_time << " 秒" << std::endl;   
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000*wait_time)));
            order_messages.msg_state.action_states[i].action_status = "FINISHED";
            std::cout << "✅ WAIT动作已完成，action_id: " << order_messages.msg_state.action_states[i].action_id << std::endl;
            
        }
        else{
            std::cout << "非法动作类型！" << std::endl;
            std::cout << "🔍 调试信息: action_type=" << order_messages.msg_state.action_states[i].action_type 
                      << ", action_status=" << order_messages.msg_state.action_states[i].action_status 
                      << ", action_id=" << order_messages.msg_state.action_states[i].action_id << std::endl;
        }

        
    }

    if(flag_have_fork_action_ || flag_have_battery_action_)
        return actionID;
    else
        return -1;
}
