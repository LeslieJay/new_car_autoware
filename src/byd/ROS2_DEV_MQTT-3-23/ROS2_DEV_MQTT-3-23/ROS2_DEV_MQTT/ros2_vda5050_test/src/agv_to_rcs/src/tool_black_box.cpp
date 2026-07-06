/************************************** File Info ****************************************
* @file:       tool_black_box.cpp     
* @author:     刘鸿彬
* @date:       2024-09-18
* @version:    V0.0
* @class:      工具类
* @brief:      AGV黑匣子功能
******************************************************************************************/

// tool_black_box.cpp
#include "tool_black_box.h"

/*****************************************************************************************
* @brief:      类的构造函数，采用参数列表的形式获取外部参数
* @param:      &static_filepath：保存AGV静态数据信息的文件位置
* @param:      &instant_filepath：保存AGV即时数据信息的文件位置
* @param:      &currentpose：agv接收到的当前位姿消息结构体引用
* @param:      &batterymessage：agv电池相关数据结构体引用
* @param:      &ordermessage：agv接收到的order消息结构体引用
* @param:      &instant_action_messages：agv接收的即时任务结构体引用
* @param:      &messagesNotEmpty：条件变量，用于请求响应
* @param:      &agvMessagesMutex：互斥锁，用于多线程读写
* @param:      &receive_new_actions：用于判断是否收到最新的即时动作指令
* @author:     刘鸿彬
* @date:       2024-09-24
(               std::string &static_filepath,
                std::string &instant_filepath,
                CurrentPose &currentpose,
                BatteryMessages &batterymessage,
                OrderMessages &ordermessage,
                InstantActionMessages &instant_action_messages,
                std::condition_variable &messagesNotEmpty,
                std::mutex &agvMessagesMutex,
                bool &receive_new_actions,
                std::shared_ptr<OrderListener> order_listener
                )
******************************************************************************************/
AGVBlackBox::AGVBlackBox(AGVBone agv_bone):
                        current_state_message_(agv_bone.agv_current_state),
                        messagesNotEmpty_(agv_bone.agv_instant_action_listener->get_messages_not_empty()),
                        instant_action_listener(agv_bone.agv_instant_action_listener),
                        order_listener(agv_bone.agv_order_listener),
                        current_pose_listener(agv_bone.agv_current_pose_listener),
                        battery_listener_(agv_bone.agv_battery_listener),
                        velocity_listener_(agv_bone.agv_velocity_listener),
                        can_data_listener_(agv_bone.agv_can_data_listener),
                        obstacle_server_(agv_bone.agv_obstacle_server)
{
    // 构造类实例的时候，默认先关闭记录数据的线程
    isRecording_ = false;
    static_file_path_ = agv_config.static_file_path;
    instant_file_path_ = agv_config.instant_file_path;
    // CAN数据文件路径
    can_data_file_path_ = agv_config.candata_file_path;
    // 每次打开黑瞎子功能时，数据包的序列号归0
    header_id_ = 0;
    // 默认黑匣子记录两分钟（120s）内的数据
    record_duration_ = agv_config.record_duration;
    
}

/*****************************************************************************************
* @brief:      类的析构函数，调用黑匣子关闭程序
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
AGVBlackBox::~AGVBlackBox() {
    if (isRecording_) {
        stopRecording();
    }
}

/*****************************************************************************************
* @brief:      黑匣子启动程序；创建黑匣子记录静态和即时数据信息的两个线程
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
void AGVBlackBox::startRecording() {
    if (!isRecording_) {
        isRecording_ = true;
        static_recordingThread_  = std::thread(&AGVBlackBox::recordStaticData,  this);
        instant_recordingThread_ = std::thread(&AGVBlackBox::recordInstantData, this);
        can_data_recordingThread_ = std::thread(&AGVBlackBox::recordCanData, this);
    }
}

/*****************************************************************************************
* @brief:      黑匣子关闭程序；停止黑匣子记录静态和即时数据信息的两个线程
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
void AGVBlackBox::stopRecording() {
    if (isRecording_) {
        isRecording_ = false;
        if (static_recordingThread_.joinable()) {
            static_recordingThread_.join();
        }
        if (instant_recordingThread_.joinable()) {
            instant_recordingThread_.join();
        }
        if (can_data_recordingThread_.joinable()) {
            can_data_recordingThread_.join();
        }
    }
}

/*****************************************************************************************
* @brief:      OrderMessages类型（除去msg_state成员之后的拷贝函数）
* @author:     刘鸿彬
* @date:       2024-11-1
******************************************************************************************/
void AGVBlackBox::copyOrderMessagesExceptMsgState(OrderMessages& from, OrderMessages& to) {
    // 手动复制除了msg_state之外的所有成员
    to.goal_x = from.goal_x;
    to.goal_y = from.goal_y;
    to.goal_theta = from.goal_theta;
    to.node_size = from.node_size;
    to.edge_size = from.edge_size;
    to.action_size = from.action_size;
    to.current_goal_x = from.current_goal_x;
    to.current_goal_y = from.current_goal_y;
    to.current_goal_theta = from.current_goal_theta;
    to.action_vec = from.action_vec;
}

/*****************************************************************************************
* @brief:      信息获取：当前位姿
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
const CurrentPose& AGVBlackBox::get_current_pose_message() const {
    return current_pose_message_;
}

/*****************************************************************************************
* @brief:      信息获取：电池信息
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
BatteryMessages AGVBlackBox::get_battery_message() const {
    if (battery_listener_) {
        return battery_listener_->get_battery_messages();
    }
    BatteryMessages empty;
    return empty;
}

/*****************************************************************************************
* @brief:      信息获取：任务数据
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
OrderMessages AGVBlackBox::get_order_message() const {
    return order_message_;
}

/*****************************************************************************************
* @brief:      信息获取：即时命令
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
InstantActionMessages AGVBlackBox::get_instant_action_message() const {
    return instant_action_message_;
}

/*****************************************************************************************
* @brief:      信息获取：连接状态
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
const std::vector<std::string>& AGVBlackBox::get_system_connections() const {
    return systemConnections_;
}

/*****************************************************************************************
* @brief:      记录静态数据；记录record_duration_s内的std::vector<AGVState>类型数据
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
void AGVBlackBox::recordStaticData() {
    // 用于计数
    int counter = 0;

    // 要存储的数据类型 注意：数据包的尺寸=最大记录时长
    auto message = std::vector<AGVState>();
    auto order_messages = std::vector<OrderMessages>();
    auto fork_height_messages = std::vector<int>();
    auto current_state_messages = std::vector<std::string>();
    auto velocity_messages = std::vector<VelocityMessages>();
    auto obstacle_messages = std::vector<int>();

    message.resize(record_duration_);
    order_messages.resize(record_duration_);
    fork_height_messages.resize(record_duration_);
    current_state_messages.resize(record_duration_);
    velocity_messages.resize(record_duration_);
    obstacle_messages.resize(record_duration_);

    // 先初始化一下order_message_、current_pose_message_、instant_action_message_
    order_message_ = order_listener->get_order_messages();
    current_pose_message_ = current_pose_listener->get_current_pose();

    instant_action_message_ = instant_action_listener->get_instant_action_messages();


    // while (isRecording_&&order_listener->get_order) {
    while (isRecording_) {
        // 更新数据（每1s1次）
        order_message_ = order_listener->get_order_messages();
        current_pose_message_ = current_pose_listener->get_current_pose();
        
        instant_action_message_ = instant_action_listener->get_instant_action_messages();
        
        // 先保存一下order_message_（每1s1次）
        copyOrderMessagesExceptMsgState(order_message_, order_messages[counter%record_duration_]);
        // 记录数据到文件
        // 获取当前时间点
        auto now = std::chrono::system_clock::now();  

        // 转换为自epoch以来的时间点
        auto time_t_now = std::chrono::system_clock::to_time_t(now);

        // 转换为tm结构体，以便于格式化
        std::tm tm_now = *std::localtime(&time_t_now);

        // 格式化tm结构体为字符串
        char buffer[80];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_now);

        std::string time_string(buffer);
        
        header_id_++;

        // 数据填充

        // 基础信息
        message[counter%record_duration_].header_id = header_id_;
        message[counter%record_duration_].timestamp = time_string;
        message[counter%record_duration_].version = agv_config.version;
        message[counter%record_duration_].manufacturer = agv_config.manufacturer;
        message[counter%record_duration_].serial_number = agv_config.serial_number;

        message[counter%record_duration_].order_id = order_message_.msg_state.order_id;
        message[counter%record_duration_].order_update_id = order_message_.msg_state.order_update_id;
        message[counter%record_duration_].last_node_id = order_message_.msg_state.last_node_id;
        message[counter%record_duration_].last_node_sequence_id = order_message_.msg_state.last_node_sequence_id;

        // 设置数组大小，未设置将报错

        message[counter%record_duration_].node_states.resize(order_message_.node_size);
        for(int i=0;i<order_message_.node_size;i++){
            message[counter%record_duration_].node_states[i].node_id = order_message_.msg_state.node_states[i].node_id;
            message[counter%record_duration_].node_states[i].released = false;
            message[counter%record_duration_].node_states[i].sequence_id = order_message_.msg_state.node_states[i].sequence_id;
        }

        // 设置数组大小，未设置将报错

        message[counter%record_duration_].edge_states.resize(order_message_.edge_size);
        for(int i=0;i<order_message_.edge_size;i++){
            message[counter%record_duration_].edge_states[i].edge_id = order_message_.msg_state.edge_states[i].edge_id;
            message[counter%record_duration_].edge_states[i].sequence_id = order_message_.msg_state.edge_states[i].sequence_id;
            message[counter%record_duration_].edge_states[i].released = true;
        }

        message[counter%record_duration_].driving = true;
        message[counter%record_duration_].paused = false;
        // 设置数组大小，未设置将报错

        message[counter%record_duration_].action_states.resize(order_message_.action_size);
        for(int i=0;i<order_message_.action_size;i++){
            //RCLCPP_INFO(this->get_logger(),"action数据完成填充，大小 %d",order_messages.action_size);
            message[counter%record_duration_].action_states[i].action_id = order_message_.msg_state.action_states[i].action_id;
            message[counter%record_duration_].action_states[i].action_type = order_message_.msg_state.action_states[i].action_type;
            message[counter%record_duration_].action_states[i].action_description = order_message_.msg_state.action_states[i].action_description;
            message[counter%record_duration_].action_states[i].action_status = order_message_.msg_state.action_states[i].action_status;
        }
        // 当前位姿
        message[counter%record_duration_].agv_position.x = current_pose_message_.current_x;
        message[counter%record_duration_].agv_position.y = current_pose_message_.current_y;
        message[counter%record_duration_].agv_position.theta = current_pose_message_.current_theta;
        // 地图信息
        message[counter%record_duration_].agv_position.map_id = agv_config.agv_position.map_id;
        message[counter%record_duration_].agv_position.position_initialized = agv_config.agv_position.position_initialized;

        // 电量
        // 获取最新数据
        if (battery_listener_) {
            battery_message_ = battery_listener_->get_battery_messages();
        }
        if (velocity_listener_) {
            velocity_message_ = velocity_listener_->get_velocity_messages();
        }
        if (obstacle_server_) {
            obstacle_message_ = obstacle_server_->get_obstacle_messages();
        }
        
        message[counter%record_duration_].battery_state.battery_charge = battery_message_.battery_level;
        message[counter%record_duration_].battery_state.charging = battery_message_.battery_status;
        // 运行模式
        message[counter%record_duration_].operating_mode = agv_config.operating_mode;

        // 开始记录货叉高度信息
        fork_height_messages[counter%record_duration_] = 100; // battery_message_.fork_height;

        // 开始记录AGV状态信息
        current_state_messages[counter%record_duration_] = current_state_message_;

        // 开始记录AGV速度信息
        velocity_messages[counter%record_duration_] = velocity_message_;
        
        // 开始记录AGV障碍物信息
        obstacle_messages[counter%record_duration_] = obstacle_message_;
        
        // 每秒记录一次
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ++counter;
        // 存储小车record_duration_s内的信息
        recordStaticData_to_file(static_file_path_, message, order_messages, counter, fork_height_messages, current_state_messages, velocity_messages, obstacle_messages);
        // std::cout << "时刻"<< time_string <<  "完成一次黑匣子静态数据记录" << std::endl;

    }

}

/*****************************************************************************************
* @brief:      将进程中的静态数据（message数据包）保存至指定文件
* @param:      filepath：静态数据保存的文件路径
* @param:      messages：静态数据的结构体数据包
* @param:      counter： 本次开机到此为止的经过多少秒
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
void AGVBlackBox::recordStaticData_to_file(std::string filepath, std::vector<AGVState> &messages,std::vector<OrderMessages> &order_messages,int counter, 
                                            std::vector<int> &fork_height_messages, std::vector<std::string> &current_state_messages, std::vector<VelocityMessages> &velocity_messages, std::vector<int> &obstacle_messages){

    json j;

    // 定义开始和结束索引
    int start = (counter <= record_duration_) ? 0 : counter - record_duration_;
    int end = counter;

    // 序列化AGVState
    json agv_states = json::array();
    for (int i = start; i<end; i++) {
        auto state = messages[i%record_duration_];
        json agv_state = {
            {"header_id", state.header_id},
            {"timestamp", state.timestamp},
            {"version", state.version},
            {"manufacturer", state.manufacturer},
            {"serial_number", state.serial_number},
            {"order_id", state.order_id},
            {"order_update_id", state.order_update_id},
            {"last_node_id", state.last_node_id},
            {"last_node_sequence_id", state.last_node_sequence_id},
            {"driving", state.driving},
            {"paused", state.paused},
            {"operating_mode", state.operating_mode}
        };

        // 序列化node_states
        json node_states = json::array();
        for (const auto& node_state : state.node_states) {
            json node_state_json = {
                {"node_id", node_state.node_id},
                {"released", node_state.released},
                {"sequence_id", node_state.sequence_id}
            };
            node_states.push_back(node_state_json);
        }
        agv_state["node_states"] = node_states;

        // 序列化edge_states
        json edge_states = json::array();
        for (const auto& edge_state : state.edge_states) {
            json edge_state_json = {
                {"edge_id", edge_state.edge_id},
                {"released", edge_state.released},
                {"sequence_id", edge_state.sequence_id}
            };
            edge_states.push_back(edge_state_json);
        }
        agv_state["edge_states"] = edge_states;

        // 序列化action_states
        json action_states = json::array();
        for (const auto& action_state : state.action_states) {
            json action_state_json = {
                {"action_id", action_state.action_id},
                {"action_type", action_state.action_type},
                {"action_status", action_state.action_status},
                {"action_description", action_state.action_description}
            };
            action_states.push_back(action_state_json);
        }
        agv_state["action_states"] = action_states;

        // 序列化agv_position
        json agv_position = {
            {"x", state.agv_position.x},
            {"y", state.agv_position.y},
            {"theta", state.agv_position.theta},
            {"map_id", state.agv_position.map_id},
            {"position_initialized", state.agv_position.position_initialized},
            {"localization_score", state.agv_position.localization_score}
        };
        agv_state["agv_position"] = agv_position;

        // 序列化battery_state
        json battery_state = {
            {"battery_charge", state.battery_state.battery_charge},
            {"charging", state.battery_state.charging},
            {"voltage", state.battery_state.voltage},
            {"current", state.battery_state.current}
        };
        agv_state["battery_state"] = battery_state;

        // 写入AGV当前货叉高度
        agv_state["fork_height"] = fork_height_messages[i%record_duration_];

        // 写入AGV当前状态
        agv_state["current_state"] = current_state_messages[i%record_duration_];

        // 写入AGV当前速度信息
        json velocity_msg ={
            {"velocity_x", velocity_messages[i%record_duration_].velocity_x},
            {"velocity_y", velocity_messages[i%record_duration_].velocity_y},
            {"omega", velocity_messages[i%record_duration_].omega}
        };
        agv_state["velocity"] = velocity_msg;

        // 写入AGV当前障碍物信息
        agv_state["obstacle_status"] = obstacle_messages[i%record_duration_];

        agv_states.push_back(agv_state);
    }
    j["agv_states"] = agv_states;

    // 序列化OrderMessages
    json order_msgs = json::array();

    for (int i = start; i<end; i++) {
        auto order_msg = order_messages[i%record_duration_];
    // for (auto& order_msg : order_messages) {
        json order_msg_json = {
            {"goal_x", order_msg.goal_x},
            {"goal_y", order_msg.goal_y},
            {"goal_theta", order_msg.goal_theta},
            {"node_size", order_msg.node_size},
            {"edge_size", order_msg.edge_size},
            {"action_size", order_msg.action_size},
            {"current_goal_x", order_msg.current_goal_x},
            {"current_goal_y", order_msg.current_goal_y},
            {"current_goal_theta", order_msg.current_goal_theta},
            {"action_vec", json::object()}
        };

        // 序列化action_vec
        for (const auto& [key, value] : order_msg.action_vec) {
            order_msg_json["action_vec"][key] = value;
        }

        order_msgs.push_back(order_msg_json);
    }
    j["order_messages"] = order_msgs;

    // 写入counter
    j["counter"] = counter;

    // 将JSON数据写入文件
    std::ofstream file(filepath);
    file << std::setw(4) << j << std::endl;
    file.close();

}


/*****************************************************************************************
* @brief:      将进程中的即时数据（instant_action_message_数据包）（一般就是指即时动作的目标位姿）保存至指定文件
* @brief:      当主线程接收到新的即时任务指令时，会通知该线程等待，当完成即时数据的更新后——
* @brief:      ——该线程不再等待，开始工作，将最新的即时数据保存至指定文件。
* @author:     刘鸿彬
* @date:       2024-09-24
******************************************************************************************/
void AGVBlackBox::recordInstantData() {
    // 打开我们要保存的文件，没有则创建
    std::ofstream file(instant_file_path_, std::ios::out);

    if (file.is_open()) {
        while(isRecording_){
            std::unique_lock<std::mutex> lock(agvMessagesMutex_);
            std::cout << "线程阻塞，等待激活！"<< std::endl;
            // 阻塞程序，等待激活
            messagesNotEmpty_.wait(lock,[this]{ 
                return instant_action_listener->get_receive_new_actions(); 
            });
            std::cout << "线程激活OK！"<< std::endl;
            // 将即时动作指令保存至指定文件
            // 获取当前时间点
            auto now = std::chrono::system_clock::now();  
            // 转换为自epoch以来的时间点
            auto time_t_now = std::chrono::system_clock::to_time_t(now);
            // 转换为tm结构体，以便于格式化
            std::tm tm_now = *std::localtime(&time_t_now);
            // 格式化tm结构体为字符串
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_now);
            std::string time_string(buffer);

            // 创建JSON对象
            json j;
            j["timestamp"] = time_string;
            j["goal_x"] = instant_action_listener->m_goal_node_x;
            j["goal_y"] = instant_action_listener->m_goal_node_y;
            j["goal_theta"] = instant_action_listener->m_goal_node_theta;
            j["action_type"] = instant_action_listener->action_type;

            // 将JSON数据写入文件
            file << j.dump(4) << std::endl;

            instant_action_listener->set_receive_new_actions(false); // 让该线程再次进入等待状态，等待新的即时任务传来
            lock.unlock();
        }
        file << "------------------------------\n";
        // 关闭文件
        file.close();

    } else {
        std::cerr << "Unable to open or create file: " << instant_file_path_ << std::endl;
    }
    
}

/*****************************************************************************************
* @brief:      记录CAN数据；记录record_duration_秒内的IO数据、错误数据和硬件数据
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
void AGVBlackBox::recordCanData() {
    // 用于计数
    int counter = 0;

    // 要存储的数据类型，注意：数据包的尺寸=最大记录时长（秒）* 10（因为每0.1秒记录一次）
    auto io_data_messages = std::vector<ref_slam_interface::msg::IoData>();
    auto err_data_messages = std::vector<ref_slam_interface::msg::ErrData>();
    auto hardware_data_messages = std::vector<ref_slam_interface::msg::HardwareData>();
    auto timestamps = std::vector<std::string>();

    // 每0.1秒记录一次，所以缓冲区大小需要是 record_duration_ * 10
    int buffer_size = record_duration_ * 10;
    io_data_messages.resize(buffer_size);
    err_data_messages.resize(buffer_size);
    hardware_data_messages.resize(buffer_size);
    timestamps.resize(buffer_size);

    // 初始化数据
    if (can_data_listener_) {
        io_data_message_ = can_data_listener_->get_io_data();
        err_data_message_ = can_data_listener_->get_err_data();
        hardware_data_message_ = can_data_listener_->get_hardware_data();
    }

    while (isRecording_) {
        // 更新数据（每0.1s1次）
        if (can_data_listener_) {
            io_data_message_ = can_data_listener_->get_io_data();
            err_data_message_ = can_data_listener_->get_err_data();
            hardware_data_message_ = can_data_listener_->get_hardware_data();
        }

        // 获取当前时间点（精确到毫秒）
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm tm_now = *std::localtime(&time_t_now);
        
        // 获取毫秒部分
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        char buffer[100];
        std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_now);
        std::string time_string(buffer);
        // 确保毫秒部分有3位数字
        if (ms.count() < 10) {
            time_string += ".00" + std::to_string(ms.count());
        } else if (ms.count() < 100) {
            time_string += ".0" + std::to_string(ms.count());
        } else {
            time_string += "." + std::to_string(ms.count());
        }

        // 保存数据到循环缓冲区
        io_data_messages[counter % buffer_size] = io_data_message_;
        err_data_messages[counter % buffer_size] = err_data_message_;
        hardware_data_messages[counter % buffer_size] = hardware_data_message_;
        timestamps[counter % buffer_size] = time_string;

        // 每0.1秒记录一次
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++counter;

        // 存储record_duration_秒内的信息到文件
        recordCanData_to_file(can_data_file_path_, io_data_messages, err_data_messages, 
                              hardware_data_messages, timestamps, counter, buffer_size);
    }
}

/*****************************************************************************************
* @brief:      将进程中的CAN数据保存至指定文件
* @param:      filepath：CAN数据保存的文件路径
* @param:      io_data_messages：IO数据包
* @param:      err_data_messages：错误数据包
* @param:      hardware_data_messages：硬件数据包
* @param:      timestamps：时间戳数组
* @param:      counter：本次开机到此为止的经过多少次（每0.1秒一次）
* @param:      buffer_size：缓冲区大小（record_duration_ * 10）
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
void AGVBlackBox::recordCanData_to_file(std::string filepath, 
                                         std::vector<ref_slam_interface::msg::IoData> &io_data_messages,
                                         std::vector<ref_slam_interface::msg::ErrData> &err_data_messages,
                                         std::vector<ref_slam_interface::msg::HardwareData> &hardware_data_messages,
                                         std::vector<std::string> &timestamps,
                                         int counter,
                                         int buffer_size) {
    json j;

    // 定义开始和结束索引（基于buffer_size，而不是record_duration_）
    int start = (counter <= buffer_size) ? 0 : counter - buffer_size;
    int end = counter;

    // 序列化CAN数据
    json can_data_array = json::array();
    for (int i = start; i < end; i++) {
        int idx = i % buffer_size;
        json can_data_entry = {
            {"timestamp", timestamps[idx]},
            {"io_data", json::object()},
            {"err_data", json::object()},
            {"hardware_data", json::object()}
        };

        // 序列化IO数据
        auto& io_data = io_data_messages[idx];
        json io_data_json = json::object();
        io_data_json["epec_all_ok"] = io_data.epec_all_ok;
        io_data_json["vcu_watch_dog_out"] = io_data.vcu_watch_dog_out;
        io_data_json["steer_enabled"] = io_data.steer_enabled;
        io_data_json["drive_enabled"] = io_data.drive_enabled;
        io_data_json["pump_enabled"] = io_data.pump_enabled;
        io_data_json["fork_loaded"] = io_data.fork_loaded;
        io_data_json["epec_ems"] = io_data.epec_ems;
        io_data_json["low_power_alarm"] = io_data.low_power_alarm;
        io_data_json["vcu_pause"] = io_data.vcu_pause;
        io_data_json["vehicle_have_fault"] = io_data.vehicle_have_fault;
        io_data_json["safety_contact_switch"] = io_data.safety_contact_switch;
        io_data_json["l_fork_sensor_stop"] = io_data.l_fork_sensor_stop;
        io_data_json["r_fork_sensor_stop"] = io_data.r_fork_sensor_stop;
        io_data_json["charge_fail"] = io_data.charge_fail;
        io_data_json["low_power_shutup"] = io_data.low_power_shutup;
        io_data_json["allow_move"] = io_data.allow_move;
        io_data_json["auto_manual_switch"] = io_data.auto_manual_switch;
        io_data_json["do_power_control_relay"] = io_data.do_power_control_relay;
        io_data_json["do_driver_controller_power_relay"] = io_data.do_driver_controller_power_relay;
        io_data_json["do_vehicle_power_control"] = io_data.do_vehicle_power_control;
        io_data_json["do_steer_controller_power_relay"] = io_data.do_steer_controller_power_relay;
        io_data_json["do_down_valve"] = io_data.do_down_valve;
        io_data_json["brake_signal"] = io_data.brake_signal;
        io_data_json["charge_contactor_power"] = io_data.charge_contactor_power;
        io_data_json["bms_relay_power"] = io_data.bms_relay_power;
        io_data_json["charge_wifi_power"] = io_data.charge_wifi_power;
        io_data_json["charger_push_back"] = io_data.charger_push_back;
        io_data_json["sevenday_charge"] = io_data.sevenday_charge;
        io_data_json["ipc_ems_callback"] = io_data.ipc_ems_callback;
        io_data_json["screen_req_blackbox"] = io_data.screen_req_blackbox;
        
        // 序列化错误数据
        auto& err_data = err_data_messages[idx];
        json err_data_json = json::object();
        // 根据实际ErrData.msg字段填充
        err_data_json["cvc_can_err"] = err_data.cvc_can_err;
        err_data_json["driver_controller_can_err"] = err_data.driver_controller_can_err;
        err_data_json["steer_controller_can_err"] = err_data.steer_controller_can_err;
        err_data_json["pump_controller_can_err"] = err_data.pump_controller_can_err;
        err_data_json["bms_can_err"] = err_data.bms_can_err;
        err_data_json["height_encode_can_err"] = err_data.height_encode_can_err;
        err_data_json["left_laser_can_err"] = err_data.left_laser_can_err;
        err_data_json["right_laser_can_err"] = err_data.right_laser_can_err;
        err_data_json["head_laser_can_err"] = err_data.head_laser_can_err;
        err_data_json["back_laser_can_err"] = err_data.back_laser_can_err;
        err_data_json["turning_encode_can_err"] = err_data.turning_encode_can_err;
        err_data_json["ele_cylinder_can_err"] = err_data.ele_cylinder_can_err;
        err_data_json["charger_can_err"] = err_data.charger_can_err;
        err_data_json["driver_controller_err"] = err_data.driver_controller_err;
        err_data_json["steer_controller_err"] = err_data.steer_controller_err;
        err_data_json["back_laser_sensor_err"] = err_data.back_laser_sensor_err;
        err_data_json["head_laser_sensor_err"] = err_data.head_laser_sensor_err;
        err_data_json["left_laser_err"] = err_data.left_laser_err;
        err_data_json["right_laser_err"] = err_data.right_laser_err;
        err_data_json["height_detect_sensor_err"] = err_data.height_detect_sensor_err;
        err_data_json["charger_err"] = err_data.charger_err;
        err_data_json["cvc_controller_err"] = err_data.cvc_controller_err;
        err_data_json["remote_err"] = err_data.remote_err;
        err_data_json["battery_bms_err"] = err_data.battery_bms_err;
        err_data_json["speed_over_limit1"] = err_data.speed_over_limit1;
        err_data_json["forklift_over_time"] = err_data.forklift_over_time;
        err_data_json["fork_drop_over_time"] = err_data.fork_drop_over_time;
        err_data_json["ele_cylinder_err"] = err_data.ele_cylinder_err;
        err_data_json["left_laser_fault_code"] = err_data.left_laser_fault_code;
        err_data_json["right_laser_fault_code"] = err_data.right_laser_fault_code;
        err_data_json["head_laser_fault_code"] = err_data.head_laser_fault_code;
        err_data_json["back_laser_fault_code"] = err_data.back_laser_fault_code;
        err_data_json["driver_controller_err_code1"] = err_data.driver_controller_err_code1;
        err_data_json["driver_controller_err_code2"] = err_data.driver_controller_err_code2;
        err_data_json["steer_controller_err_code1"] = err_data.steer_controller_err_code1;
        err_data_json["steer_controller_err_code2"] = err_data.steer_controller_err_code2;
        err_data_json["ele_cylinder_controller_err_code1"] = err_data.ele_cylinder_controller_err_code1;
        err_data_json["ele_cylinder_controller_err_code2"] = err_data.ele_cylinder_controller_err_code2;

        // 序列化硬件数据
        auto& hardware_data = hardware_data_messages[idx];
        json hardware_data_json = json::object();
        hardware_data_json["battery_temp"] = hardware_data.battery_temp;
        hardware_data_json["ele_cylinder_i_q"] = hardware_data.ele_cylinder_i_q;
        hardware_data_json["driver_controller_i_q"] = hardware_data.driver_controller_i_q;
        hardware_data_json["steer_controller_i_q"] = hardware_data.steer_controller_i_q;
        hardware_data_json["power_on_time_h"] = hardware_data.power_on_time_h;
        hardware_data_json["power_on_time_min"] = hardware_data.power_on_time_min;
        hardware_data_json["work_time_h"] = hardware_data.work_time_h;
        hardware_data_json["work_time_min"] = hardware_data.work_time_min;
        hardware_data_json["total_distance"] = hardware_data.total_distance;
        hardware_data_json["current_distance"] = hardware_data.current_distance;

        can_data_entry["io_data"] = io_data_json;
        can_data_entry["err_data"] = err_data_json;
        can_data_entry["hardware_data"] = hardware_data_json;

        can_data_array.push_back(can_data_entry);
    }
    j["can_data"] = can_data_array;

    // 写入counter
    j["counter"] = counter;

    // 将JSON数据写入文件
    std::ofstream file(filepath);
    file << std::setw(4) << j << std::endl;
    file.close();
}


// int main(){

//     /*
//     AGVBlackBox(std::string &static_filepath,
//                 std::string &instant_filepath,
//                 CurrentPose &currentpose,
//                 BatteryMessages &batterymessage,
//                 OrderMessages &ordermessage,
//                 InstantActionMessages &instant_action_messages,
//                 std::condition_variable &messagesNotEmpty,
//                 std::mutex &agvMessagesMutex,
//                 bool &receive_new_actions
//                 );
//     */

//     std::string static_info_filepath  = "/home/byd/ros2_lhb_project/black_box_1/A_new/ros2_VDA5050_test/black_box_data_package/static.json";
//     std::string instant_info_filepath = "/home/byd/ros2_lhb_project/black_box_1/A_new/ros2_VDA5050_test/black_box_data_package/instant.json";
//     CurrentPose agv_current_pose;
//     agv_current_pose.current_theta = 0.789;
//     agv_current_pose.current_x = 3;
//     agv_current_pose.current_y = 4;
//     BatteryMessages battery_messages;
//     battery_messages.battery_status = 1;
//     battery_messages.battery_level = 0.7;
//     battery_messages.total_current = 2.5;
//     battery_messages.total_voltage = 220.9;

//     // 创建 OrderMessages 实例
//     OrderMessages orderMsg;

//     // 初始化目标点位姿
//     orderMsg.goal_x = {1.0, 2.0, 3.0};
//     orderMsg.goal_y = {4.0, 5.0, 6.0};
//     orderMsg.goal_theta = {0.0, 0.5, 1.0};

//     // 设置节点、边、动作数量
//     orderMsg.node_size = 10;
//     orderMsg.edge_size = 20;
//     orderMsg.action_size = 5;

//     // 设置当前时刻的目标位姿
//     orderMsg.current_goal_x = 1.0;
//     orderMsg.current_goal_y = 4.0;
//     orderMsg.current_goal_theta = 0.0;

//     // 添加一些动作关联
//     orderMsg.action_vec.insert({"node1", "action1"});
//     orderMsg.action_vec.insert({"node2", "action2"});

//     // 准备贝塞尔曲线的数据
//     CurveDate curveData;
//     curveData.start_node_id = "startNode";
//     curveData.end_node_id = "endNode";
//     curveData.degree = 3;
//     curveData.control_points.push_back({0.0, 0.0, 0.0});
//     curveData.control_points.push_back({1.0, 1.0, 0.25});
//     curveData.control_points.push_back({2.0, -1.0, 0.5});
//     curveData.control_points.push_back({3.0, 0.0, 0.75});
//     orderMsg.curve_date_queue.push(curveData);

//     // 初始化 AGVState 消息
//     orderMsg.msg_state.header_id = 1;
//     orderMsg.msg_state.timestamp = "2024-10-12T11:20:00Z";
//     orderMsg.msg_state.version = "1.0";
//     orderMsg.msg_state.manufacturer = "Example Manufacturer";
//     orderMsg.msg_state.serial_number = "AGV123456";
//     orderMsg.msg_state.order_id = "ORDER001";
//     orderMsg.msg_state.order_update_id = 1;
//     orderMsg.msg_state.last_node_id = "node09";
//     orderMsg.msg_state.last_node_sequence_id = 9;

//     // 设置数组大小，未设置将报错
//     orderMsg.msg_state.node_states.resize(orderMsg.node_size);
//     for(int i=0;i<orderMsg.node_size;i++){
//         orderMsg.msg_state.node_states[i].node_id = std::to_string(i*75);
//         orderMsg.msg_state.node_states[i].released = false;
//         orderMsg.msg_state.node_states[i].sequence_id = i*900;
//     }

//     // 设置数组大小，未设置将报错
//     orderMsg.msg_state.edge_states.resize(orderMsg.edge_size);
//     for(int i=0;i<orderMsg.edge_size;i++){
//         orderMsg.msg_state.edge_states[i].edge_id = std::to_string(i*80);
//         orderMsg.msg_state.edge_states[i].sequence_id = i*1080;
//         orderMsg.msg_state.edge_states[i].released = true;
//     }

//     // 设置数组大小，未设置将报错
//     orderMsg.msg_state.action_states.resize(orderMsg.action_size);
//     for(int i=0;i<orderMsg.action_size;i++){
//         //RCLCPP_INFO(this->get_logger(),"action数据完成填充，大小 %d",order_messages.action_size);
//         orderMsg.msg_state.action_states[i].action_id = std::to_string(i*120);
//         orderMsg.msg_state.action_states[i].action_type = std::to_string(i*77);
//         orderMsg.msg_state.action_states[i].action_description = std::to_string(i*64);
//         orderMsg.msg_state.action_states[i].action_status = std::to_string(i*86);
//     }
    

//     orderMsg.msg_state.driving = true;
//     orderMsg.msg_state.paused = false;
//     orderMsg.msg_state.operating_mode = "normal";

//     // 这里可以继续添加其他必要的数据

    

//     InstantActionMessages instant_action_messages;
//     instant_action_messages.goal_x     = 3.3;
//     instant_action_messages.goal_y     = 4.5;
//     instant_action_messages.goal_theta = 1.42;
//     instant_action_messages.action_type= "test";

//     std::condition_variable messagesNotEmpty;

//     std::mutex agvMessagesMutex;
    

//     bool receive_new_actions = false;

//     AGVBlackBox blackBox(static_info_filepath,
//                         instant_info_filepath,
//                         agv_current_pose,
//                         battery_messages,
//                         orderMsg,
//                         instant_action_messages,
//                         messagesNotEmpty,
//                         agvMessagesMutex,
//                         receive_new_actions);

//     blackBox.startRecording();
//     int timer = 0;
//     while(timer<10){
//         // std::lock_guard<std::mutex> lock(agvMessagesMutex);
//         // std::cout << "持续检查中！"<< std::endl;
//         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//         receive_new_actions = timer%2;
//         timer++;
//         if(receive_new_actions)
//             std::cout<<receive_new_actions<< std::endl;
//         // 通知一个等待的消费者线程
//         messagesNotEmpty.notify_all();
//     }
//     // 遇到障碍需要黑匣子记录数据
//     blackBox.stopRecording();


//     return 0;
// }