/************************************** File Info ****************************************
* @file:       subscriber_instant_action.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-09                                         
* @version:    V0.0     
* @class:      订阅者                                                                         
* @brief:      AGV设备接受RCS即时操作话题的订阅端，订阅话题，解析消息
------------------------------------------------------------------------------------------
* @modified:   增加一个接收消息超时判断
* @date:       2024-11-09
* @version:    V0.1
* @description:当出现中断的情况时，接收到的数据不会更新，如果不处理，会影响后续的判断
* @note:        
******************************************************************************************/
// subscriber_instant_action.cpp
#include "subscriber_instant_action.h"

/*****************************************************************************************
* @brief:      类的构造函数
* @author:     刘鸿彬
* @date:       2024-11-09
******************************************************************************************/
InstantActionsListener::InstantActionsListener(std::shared_ptr<rclcpp::Node> node)
    :node_(node),
    receive_new_actions_(false)
{

    RCLCPP_INFO(node_->get_logger(),"created a order subscriber");
    // 创建订阅方
    std::string SerialNumber = agv_config.serial_number;
    subscription_ = node_->create_subscription<AGVInstantActions>("uagv/v1/BYD/" + SerialNumber + "/instantActions",10,std::bind(&InstantActionsListener::instant_action_callback,this,std::placeholders::_1));
    // 创建定时器
    instant_action_timer_ = node_->create_wall_timer(std::chrono::seconds(10),std::bind(&InstantActionsListener::timer_callback, this));

    // 记录上一条消息的时间
    last_msg_time_ = std::chrono::steady_clock::now();

    // 初始化接收到消息为false
    receive_new_messages = false;
}


/*****************************************************************************************
* @brief:      返回解析instantAction得到的所需数据
* @param:      无
* @return:     自定义结构体，包含目标位姿以及动作类型
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
******************************************************************************************/
InstantActionMessages InstantActionsListener::get_instant_action_messages(){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 在回调函数中，先赋值给m_goal_node_x，然后在实际使用instant_action_messages的时候在赋值给他，能够减小每次回调都赋值给instant_action_messages，导致调用时要等的时间开销，但也很小，更多是代码设计习惯，用于区分内部状态和读取快照
    instant_action_messages.goal_x = m_goal_node_x;
    instant_action_messages.goal_y = m_goal_node_y;
    instant_action_messages.goal_theta = m_goal_node_theta;

    instant_action_messages.action_type = action_type;

    instant_action_messages.last_instant_action_order_id = last_action_id;

    receive_new_messages = false;
    

    return instant_action_messages;
    
}

/*****************************************************************************************
* @brief:      获取条件变量 messagesNotEmpty 的引用
* @param:      无
* @return:     条件变量的引用
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
std::condition_variable& InstantActionsListener::get_messages_not_empty(){
    return messagesNotEmpty_;
}

/*****************************************************************************************
* @brief:      获取 receive_new_actions 标志
* @param:      无
* @return:     receive_new_actions 的值
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
bool InstantActionsListener::get_receive_new_actions() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return receive_new_actions_;
}

/*****************************************************************************************
* @brief:      设置 receive_new_actions 标志
* @param:      value - 要设置的值
* @return:     无
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
void InstantActionsListener::set_receive_new_actions(bool value) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    receive_new_actions_ = value;
}

/*****************************************************************************************
* @brief:      获取 interrupt_order_message
* @param:      无
* @return:     interrupt_order_message 的副本
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
InterruptOrderMessage InstantActionsListener::get_interrupt_order_message() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return interrupt_order_message_;
}

/*****************************************************************************************
* @brief:      更新 interrupt_order_message 的 action_status
* @param:      action_status - 新的状态值
* @return:     无
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
void InstantActionsListener::update_interrupt_order_message_status(const std::string& action_status) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    interrupt_order_message_.action_status = action_status;
}

/*****************************************************************************************
* @brief:      从MQTT消息同步数据（用于纯MQTT模式）
* @param:      mqtt_instant_action_messages - MQTT解析后的消息
* @return:     无
* @author:     Assistant
* @date:       2024-11-20
* @version:    V0.0
* @note:       在纯MQTT模式下，ROS2订阅者不会收到消息，需要通过此方法手动同步
******************************************************************************************/
// void InstantActionsListener::update_from_mqtt(const AGVInstantActions &mqtt_instant_action_messages){

//     RCLCPP_INFO(node_->get_logger(), "正在同步MQTT消息到InstantActionsListener...");

//     try {
//         instant_action_callback(mqtt_instant_action_messages);
//         std::cout << "DEBUG: instant_action_callback() 调用成功" << std::endl;
//     } catch (const std::exception& e) {
//         std::cout << "DEBUG: instant_action_callback() 抛出异常: " << e.what() << std::endl;
//         RCLCPP_ERROR(node_->get_logger(), "instant_action_callback异常: %s", e.what());
//         return;
//     }

//     RCLCPP_INFO(node_->get_logger(), "✓ MQTT消息已同步到InstantActionsListener成功");
// }
void InstantActionsListener::update_from_mqtt(const AGVInstantActions &mqtt_instant_action_messages){
    RCLCPP_INFO(node_->get_logger(), "正在同步MQTT消息到InstantActionsListener...");

    std::cout << "\n===== AGVInstantActions Message =====" << std::endl;
    std::cout << "header_id: " << mqtt_instant_action_messages.header_id << std::endl;
    std::cout << "timestamp: " << mqtt_instant_action_messages.timestamp << std::endl;
    std::cout << "version: " << mqtt_instant_action_messages.version << std::endl;
    std::cout << "manufacturer: " << mqtt_instant_action_messages.manufacturer << std::endl;
    std::cout << "serial_number: " << mqtt_instant_action_messages.serial_number << std::endl;
    
    std::cout << "actions count: " << mqtt_instant_action_messages.actions.size() << std::endl;
    for (size_t i = 0; i < mqtt_instant_action_messages.actions.size(); ++i) {
        const auto &action = mqtt_instant_action_messages.actions[i];
        std::cout << "  Action[" << i << "]:" << std::endl;
        std::cout << "    action_type: " << action.action_type << std::endl;
        std::cout << "    action_id: " << action.action_id << std::endl;
        std::cout << "    action_description: " << action.action_description << std::endl;
        std::cout << "    blocking_type: " << action.blocking_type << std::endl;
        
        std::cout << "    action_parameters count: " << action.action_parameters.size() << std::endl;
        for (size_t j = 0; j < action.action_parameters.size(); ++j) {
            const auto &param = action.action_parameters[j];
            // 临时输出，避免编译错误（后续替换为 switch-case 解析 value）
            std::cout << "      Parameter[" << j << "]: key=" << param.key 
                      << ", value=<Value at " << &param.value << ">" << std::endl;
        }
    }
    std::cout << "=====================================\n" << std::endl;

    try {
        instant_action_callback(mqtt_instant_action_messages);
        std::cout << "DEBUG: instant_action_callback() 调用成功" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "DEBUG: instant_action_callback() 抛出异常: " << e.what() << std::endl;
        RCLCPP_ERROR(node_->get_logger(), "instant_action_callback异常: %s", e.what());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "✓ MQTT消息已同步到InstantActionsListener成功");
}
// 订阅方回调函数
/*****************************************************************************************
* @brief:      订阅方回调函数,获取RCS发布的即时命令
* @param:      RCS发布的消息
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
******************************************************************************************/
void InstantActionsListener::instant_action_callback(const AGVInstantActions &msg){
    
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 接收到消息之后进行判断，剔除掉重复的数据
    if(receive_new_messages){
        // 旧消息的处理-重复的全局任务消息
        if (last_action_id  == msg.actions[0].action_id) {
            // 如果条件不满足，就不处理消息
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"订阅到的消息为旧消息！");
            return;
        }
    }

    // 此处判断一下发来的是不是心跳，如果是心跳，则过滤
    if(msg.actions[0].action_type == "heartbeat_ack"){
        last_msg_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(), "持续连接中！");
        return;
    }
    
    // 记录最近消息的action数据
    last_action_id  = msg.actions[0].action_id;

    // 数据解析
    action_type = msg.actions[0].action_type;
    
    ActionType action_type_convert = convertActionTypeOrThrow(action_type);

    // ========== 新增：循环内每次更新后输出 ==========
    std::cout << "Updated instant_action_messages_ info in loop: "
                << "action_type=" << msg.actions[0].action_type
                << std::endl;
    switch (action_type_convert) {

    case ActionType::CONNECT_SUCCESS_AUTO:

        RCLCPP_INFO(node_->get_logger(), "正在向最近节点位置移动！");

        m_goal_node_x = msg.actions[0].action_parameters[0].value.number_value;
        m_goal_node_y = msg.actions[0].action_parameters[1].value.number_value;
        m_goal_node_theta = msg.actions[0].action_parameters[2].value.number_value;
        RCLCPP_INFO(node_->get_logger(), "前往最近目标点 x: %f,y: %f,theta: %f,", m_goal_node_x, m_goal_node_y, m_goal_node_theta);
        break;

    case ActionType::CONNECT_FAIL_AUTO:

        RCLCPP_INFO(node_->get_logger(), "远离目标点！");
        break;

    case ActionType::AGV_ONLINE:

        RCLCPP_INFO(node_->get_logger(), "AGV上线成功！");
        
        // 从action_parameters中提取map_id
        for (const auto& param : msg.actions[0].action_parameters) {
            if (param.key == "map_id") {
                agv_config.agv_position.map_id = param.value.string_value;
                RCLCPP_INFO(node_->get_logger(), "从AGV_ONLINE instant action中获取map_id: %s", agv_config.agv_position.map_id.c_str());
                break;
            }
        }
        break;

    case ActionType::AGV_OFFLINE:

        RCLCPP_INFO(node_->get_logger(), "断开连接!");
        break;

    case ActionType::START_CHARGE:
        
        RCLCPP_INFO(node_->get_logger(), "开始充电!");
        break;

    case ActionType::STOP_CHARGE:
        RCLCPP_INFO(node_->get_logger(), "结束充电!");
        break;

    case ActionType::ORDER_FINISHED:

        RCLCPP_INFO(node_->get_logger(), "任务完成!");
        break;

    case ActionType::CANCEL_ORDER:

        RCLCPP_INFO(node_->get_logger(), "任务取消!");
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            interrupt_order_message_.action_id = msg.actions[0].action_id;
            interrupt_order_message_.action_type = msg.actions[0].action_type;
            interrupt_order_message_.action_description = msg.actions[0].action_description;
            interrupt_order_message_.action_status = "WAITING";
        }
        break;
    
    case ActionType::START_PAUSE:

        RCLCPP_INFO(node_->get_logger(), "车辆暂停!");
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            interrupt_order_message_.action_id = msg.actions[0].action_id;
            interrupt_order_message_.action_type = msg.actions[0].action_type;
            interrupt_order_message_.action_description = msg.actions[0].action_description;
            interrupt_order_message_.action_status = "WAITING";
        }
        break;
    
    case ActionType::STOP_PAUSE:

        RCLCPP_INFO(node_->get_logger(), "取消暂停!");
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            interrupt_order_message_.action_id = msg.actions[0].action_id;
            interrupt_order_message_.action_type = msg.actions[0].action_type;
            interrupt_order_message_.action_description = msg.actions[0].action_description;
            interrupt_order_message_.action_status = "WAITING";
        }
        break;
    
    case ActionType::FACTSHEET_REQUEST:

        RCLCPP_INFO(node_->get_logger(), "收到factsheet请求!");
        break;
    
    default:

        RCLCPP_INFO(node_->get_logger(), "其他未判断情况!");
    }

    // 更新接收上条数据的时间
    last_msg_time_ = std::chrono::steady_clock::now();
    // 读取完数据之后，改变标志位
    receive_new_messages = true;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        receive_new_actions_ = true;
    }
    
    // 通知所有等待的消费者线程
    messagesNotEmpty_.notify_all();
    RCLCPP_INFO(node_->get_logger(), "激活数据更新线程!");

}
/*****************************************************************************************
* @brief:      每隔10s触发一次定时器，如果收到消息则重置，未收到消息，进行处理
* @param:      无
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-09
* @version:    V0.0
******************************************************************************************/
void InstantActionsListener::timer_callback(){

    auto now = std::chrono::steady_clock::now();
    // 计算时间差
    auto diff = now - last_msg_time_;

    // 检查自上次接收消息以来的时间是否超过了超时阈值
    if (std::chrono::duration_cast<std::chrono::seconds>(diff).count() > 10) {
        RCLCPP_ERROR(node_->get_logger(), "超过10s未接收到instant action消息");
        std::lock_guard<std::mutex> lock(data_mutex_);
        action_type = "no_message_received";
    }
}
/*****************************************************************************************
* @brief:      定义了一个枚举来表示action类型,提高代码的可读性和可维护性
* @param:      action_type_str ：动作类型字符串
* @return:     匹配则返回对应的枚举值；如果不匹配，则抛出异常
* @author:     刘鸿彬
* @date:       2024-11-12
* @version:    V0.0
******************************************************************************************/
ActionType InstantActionsListener::convertActionTypeOrThrow(const std::string& action_type_str){

    if (action_type_str == "connect_success_auto") return ActionType::CONNECT_SUCCESS_AUTO;
    else if (action_type_str == "connect_fail_auto") return ActionType::CONNECT_FAIL_AUTO;
    else if (action_type_str == "agv_online") return ActionType::AGV_ONLINE;
    else if (action_type_str == "heartbeat_ack") return ActionType::HEARTBEAT_ACK;
    else if (action_type_str == "agv_offline") return ActionType::AGV_OFFLINE;
    else if (action_type_str == "start_charge") return ActionType::START_CHARGE;
    else if (action_type_str == "stop_charge") return ActionType::STOP_CHARGE;
    else if (action_type_str == "order_finished") return ActionType::ORDER_FINISHED;

    else if (action_type_str == "cancelOrder") return ActionType::CANCEL_ORDER;
    else if (action_type_str == "startPause") return ActionType::START_PAUSE;
    else if (action_type_str == "stopPause") return ActionType::STOP_PAUSE;
    else if (action_type_str == "factsheetRequest") return ActionType::FACTSHEET_REQUEST;
    else throw std::invalid_argument("Unknown action type: " + action_type_str);

}
// int main(int argc, char const *argv[]){
//     // 初始化，启动节点，释放资源
//     rclcpp::init(argc,argv);

//     auto  listener_instant_action = std::make_shared<InstantActionsListener>();

//     // auto connection = listener_instant_action->connection_;
//     // auto agv_state  = AGVStateToRCS::getInstance();
//     // agv_state->action_size=0;
//     // agv_state->node_size =0;
//     // agv_state->edge_size = 0;
//     // auto connection = std::make_shared<AGVConnectionToRCS>();
//     // auto agv_state = std::make_shared<AGVStateToRCS>();
//     bool topic_published;
//     // RCS未发布话题时，不断发送连接请求
//     // 退出条件
//     while(!topic_published){
//             // 判断RCS是否发布该话题
//         topic_published = listener_instant_action->count_publishers("uagv/v1/BYD/agv0001/instantActions") > 0;
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "没有检测到instantAction话题！");
//         // rclcpp::spin_some(connection);
//         // rclcpp::spin_some(agv_state);
//         sleep(1);

//     }
//     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "检测到instantAction话题！");

//     // rclcpp::spin_some(connection);
//     // rclcpp::spin_some(agv_state);
//     // 启动节点
//     rclcpp::spin(listener_instant_action);

//     rclcpp::shutdown();

//     return 0;
// }
