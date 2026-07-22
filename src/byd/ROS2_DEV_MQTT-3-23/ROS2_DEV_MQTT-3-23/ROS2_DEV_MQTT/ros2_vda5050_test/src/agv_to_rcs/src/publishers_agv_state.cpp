/************************************** File Info ****************************************
* @file:       publishers_agv_state.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-10                                         
* @version:    V0.0                                                                              
* @brief:      解析接收到的数据，将其进行数据填充发布方
******************************************************************************************/
# include "publishers_agv_state.h"
# include <iomanip>
# include <sstream>

/*****************************************************************************************
* @brief:      类的构造函数，进行节点名重写
* @param:      
* @return:     
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
******************************************************************************************/
AGVDataPublish::AGVDataPublish(std::shared_ptr<rclcpp::Node> node, std::string &agv_current_state, std::shared_ptr<ListenerPose> current_pose_listener, std::shared_ptr<OrderListener> order_listener, std::shared_ptr<BatteryListener> battery_listener, std::shared_ptr<VelocityListener> velocity_listener, std::shared_ptr<ObstacleServer> obstacle_server, std::shared_ptr<CanDataListener> can_data_listener)
               :node_(node),
                agv_current_state_(agv_current_state),
                current_pose_listener_(current_pose_listener),
                order_listener_(order_listener),
                battery_listener_(battery_listener),
                velocity_listener_(velocity_listener),
                obstacle_server_(obstacle_server),
                can_data_listener_(can_data_listener),
                obstacle_messages_(0)
{

    RCLCPP_INFO(node_->get_logger(),"created a publishers_agv_state");

    std::string SerialNumber = agv_config.serial_number;
    
    // 创建connection发布方
    connection_publisher_ = node_->create_publisher<AGVConnection>("uagv/v1/BYD/" + SerialNumber + "/connection",10);
    
    // 创建state发布方
    state_publisher_ = node_->create_publisher<AGVState>("uagv/v1/BYD/" + SerialNumber + "/state",1);

    // 创建visualization发布方
    visualization_publisher_ = node_->create_publisher<AGVVisualization>("uagv/v1/BYD/" + SerialNumber + "/visualization",1);
    
    // 创建factsheet发布方
    factsheet_publisher_ = node_->create_publisher<AGVFactsheet>("uagv/v1/BYD/" + SerialNumber + "/factsheet",10);

    // 创建小车状态发布方
    status_publisher_ = node_->create_publisher<VehicleStatus>("uagv/v1/BYD/" + SerialNumber + "/vehicle_status",10);

    // 初始化id
    header_id_connection = 0;
    header_id_factsheet = 0;
    header_id_visualization = 0;
    header_id_state = 0;
    header_id_status = 0;

    // 初始化计数器
    count_time = 28;
    // count_time = 0;

    // 初始化定时器创建标识
    state_timer_create = false;
    
    // 初始化MQTT相关变量
    mqtt_enabled_ = false;

    // 初始化驱动控制函数对象为空
    get_driving_fn_ = nullptr;
    
    // 初始化上一次的driving状态为false
    last_driving_state_ = false;

    // 初始化上一次的操作模式为true（AUTOMATIC）
    last_operation_mode_ = true;

}

/*****************************************************************************************
* @brief:      初始化MQTT客户端
* @param:      mqtt_config MQTT配置参数
* @return:     是否初始化成功
* @author:     Assistant
* @date:       2024-11-20
* @version:    V1.0
******************************************************************************************/
bool AGVDataPublish::initMQTT(const MQTTConfig& mqtt_config) {
    try {
        mqtt_client_ = std::make_unique<MQTTClient>(mqtt_config, node_->get_logger());
        
        if (mqtt_client_->connect()) {
            mqtt_enabled_ = true;
            RCLCPP_INFO(node_->get_logger(), "MQTT客户端初始化成功");
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "MQTT客户端连接失败");
            mqtt_enabled_ = false;
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "MQTT客户端初始化异常: %s", e.what());
        mqtt_enabled_ = false;
        return false;
    }
}

/*****************************************************************************************
* @brief:      创建定时器，接收解析后的数据，进行定时发布
* @param:      1、当强位姿，2、order命令数据
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
******************************************************************************************/
void AGVDataPublish::publish_agv_state(std::string state){

    // 创建state话题的定时器
    state_timer_ = node_->create_wall_timer(1s, [this]() {this->State_timer_callback();});
    
    // 创建visualization话题的定时器
    visualization_timer_ = node_->create_wall_timer(0.1s, [this]() {this->visualization_timer_callback();});
    
    // 创建visualization话题的定时器
    factsheet_timer_ = node_->create_wall_timer(10s, [this]() {this->factsheet_timer_callback();});

    // 创建connection话题的定时器
    connection_timer_ = node_->create_wall_timer(1s, [this,state]() {this->connection_timer_callback(state);});

    // 创建小车状态发布定时器(0.1秒间隔)
    status_timer_ = node_->create_wall_timer(0.1s, [this]() {this->status_timer_callback();});
    
    // 用于取消所有定时器，现在没用
    state_timer_create = true;
    // 上线后停止发送connection话题
    connection_timer_create = true;
}

/*****************************************************************************************
* @brief:      销毁定时器
* @param:      无
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-09
* @version:    V0.0
******************************************************************************************/
void AGVDataPublish::cancel_all_timer(){

    // 销毁定时器
    state_timer_.reset();
    visualization_timer_.reset();
    factsheet_timer_.reset();
    connection_timer_.reset();
    status_timer_.reset();

    // 更新定时器创建标识
    state_timer_create = false;
}

/*****************************************************************************************
* @brief:      创建请求连接的定时器
* @param:      请求的状态，ONLINE-请求上线，OFFLINE-请求下线
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-18
* @version:    V0.0
******************************************************************************************/
void AGVDataPublish::publish_connection_request(std::string state){
    // 创建connection话题的定时器
    connection_timer_ = node_->create_wall_timer(1s, [this,state]() {this->connection_timer_callback(state);});
    connection_timer_create = true;
}

/*****************************************************************************************
* @brief:      销毁请求连接的定时器
* @param:      无
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-17
* @version:    V0.0
******************************************************************************************/
void AGVDataPublish::cancel_connection_timer(){

    // 销毁定时器
    connection_timer_.reset();

    // 更新定时器创建标识
    connection_timer_create = false;
}

/*****************************************************************************************
* @brief:      connection发布方的定时器，用于定时发布connection消息
* @param:      请求状态，online为请求上线，offline为请求下线
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
******************************************************************************************/
void AGVDataPublish::connection_timer_callback(const std::string state){
    
    // RCLCPP_INFO(node_->get_logger(), "connection_timer_callback回调启动！");
    auto message = AGVConnection();

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

    header_id_connection++;

    // 数据填充
    message.header_id = header_id_connection;
    message.timestamp = time_string;
    message.version = agv_config.version;
    message.manufacturer = agv_config.manufacturer;
    message.serial_number = agv_config.serial_number;
    message.connection_state = state;
    // 数据发布,订阅带instantAction话题时，停止发布
  
    connection_publisher_->publish(message);
    // MQTT发布 (QoS 1: 连接状态信息重要，需要保证送达)
    if (mqtt_enabled_ && mqtt_client_->isConnected()) {
        std::string topic = "uagv/v1/BYD/" + agv_config.serial_number + "/connection";
        std::string json_payload = JsonConverter::toJson(message);
        mqtt_client_->publish(topic, json_payload, 1);
    }

    state_timer_callback();

}

/*****************************************************************************************
* @brief:      state话题定时器的回调函数，用于定时发布state数据
* @param:      1、当强位姿，2、order命令数据
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
* @note:       注意动作的判断，完成动作之后，需要更新last_node_id和last_node_sequence_id
******************************************************************************************/
void AGVDataPublish::State_timer_callback(){
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 更新最新数据（如果提供了listener/server指针）
    if (current_pose_listener_) {
        current_pose_ = current_pose_listener_->get_current_pose();
    }
    if (order_listener_) {
        order_messages_ = order_listener_->get_order_messages();
    }

    // 检测 driving 状态发生变化，立即上报 state
    if (get_driving_fn_) {
        bool current_driving = get_driving_fn_();
        // 如果 driving 状态发生变化，立即发布 state
        if (last_driving_state_ != current_driving) {
            RCLCPP_INFO(node_->get_logger(), "检测到 driving 状态发生变化，立即上报 state");
            // 立即发布数据
            this->state_timer_callback();
        }
        // 更新上一次的 driving 状态
        last_driving_state_ = current_driving;
    }

    // 检测操作模式发生变化，立即上报 state
    if (can_data_listener_) {
        bool current_operation_mode = can_data_listener_->get_operation_mode();
        if (last_operation_mode_ != current_operation_mode) {
            RCLCPP_INFO(node_->get_logger(), "检测到操作模式发生变化（%s -> %s），立即上报 state",
                last_operation_mode_ ? "AUTOMATIC" : "MANUAL",
                current_operation_mode ? "AUTOMATIC" : "MANUAL");
            this->state_timer_callback();
        }
        last_operation_mode_ = current_operation_mode;
    }

    // 达到目标位姿附近时立即发送当前状态
    if (abs(current_pose_.current_x - order_messages_.current_goal_x)<= 0.2 && abs(current_pose_.current_y - order_messages_.current_goal_y)<=0.2 && ( fmod(abs(current_pose_.current_theta - order_messages_.current_goal_theta),6.28)<=0.3 ||  fmod(abs(current_pose_.current_theta - order_messages_.current_goal_theta),6.28)>=(6.28-0.3) )){
        // 计数器清零
        count_time = 0;
        // 发布数据
        this->state_timer_callback();
    }
    // 每30s发布一次状态
    if(count_time >=30){

        // 计数器清零
        count_time = 0;
        // 发布数据
        this->state_timer_callback();
    }
    
    count_time++;
}

/*****************************************************************************************
* @brief:      发布方state定时器回调函数，AGV通过该消息发布设备接收到的任务，包括节点、边、动作等，其中动作状态由此发布出去
* @param:      fault_code 错误码，当不为NONE时填充errors字段
* @return:     无
* @author:     刘鸿彬
* @date:       2024-3-14
* @version:    V0.0
******************************************************************************************/

void AGVDataPublish::state_timer_callback(const std::string& fault_code){

    // RCLCPP_INFO(node_->get_logger(), "state_timer_callback回调启动！");
    
    // 更新最新数据（如果提供了listener/server指针）
    if (current_pose_listener_) {
        current_pose_ = current_pose_listener_->get_current_pose();
    }
    if (order_listener_) {
        order_messages_ = order_listener_->get_order_messages();
    }
    if (battery_listener_) {
        battery_messages_ = battery_listener_->get_battery_messages();
    }
    if (velocity_listener_) {
        velocity_messages_ = velocity_listener_->get_velocity_messages();
    }
    if (obstacle_server_) {
        obstacle_messages_ = obstacle_server_->get_obstacle_messages();
    }
    
    // 计时变量，该函数1s调用一次，加1为1s

    auto message = AGVState();
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
    
    header_id_state++;

    // 数据填充

    // 基础信息
    message.header_id = header_id_state;
    message.timestamp = time_string;
    message.version = agv_config.version;
    message.manufacturer = agv_config.manufacturer;
    message.serial_number = agv_config.serial_number;

    message.order_id = order_messages_.msg_state.order_id;
    message.order_update_id = order_messages_.msg_state.order_update_id;
    message.last_node_id = order_messages_.msg_state.last_node_id;
    message.last_node_sequence_id = order_messages_.msg_state.last_node_sequence_id;

    // 设置数组大小，未设置将报错
    message.node_states.resize(order_messages_.node_size);
    for(int i=0;i<order_messages_.node_size;i++){
        message.node_states[i].node_id = order_messages_.msg_state.node_states[i].node_id;
        message.node_states[i].released = false;
        message.node_states[i].sequence_id = order_messages_.msg_state.node_states[i].sequence_id;
    }

    // 设置数组大小，未设置将报错
    message.edge_states.resize(order_messages_.edge_size);
    for(int i=0;i<order_messages_.edge_size;i++){
        message.edge_states[i].edge_id = order_messages_.msg_state.edge_states[i].edge_id;
        message.edge_states[i].sequence_id = order_messages_.msg_state.edge_states[i].sequence_id;
        message.edge_states[i].released = true;
    }

    // 根据agv_driver_control是否为空来分类取值driving状态
    if (get_driving_fn_) {
        // 如果设置了驱动控制函数，则从驱动控制获取driving状态
        message.driving = get_driving_fn_();
    } else {
        // 如果未设置驱动控制函数，则使用order_messages_.msg_state.driving的值
        message.driving = false;
    }
    
    message.paused = order_messages_.msg_state.paused;

    // 同步action_size为实际action_states的长度
    order_messages_.action_size = order_messages_.msg_state.action_states.size();

    // 设置数组大小，未设置将报错
    message.action_states.resize(order_messages_.msg_state.action_states.size());
    for(int i=0;i<order_messages_.msg_state.action_states.size();i++){
        RCLCPP_INFO(node_->get_logger(),"action数据完成填充，大小 %zu",order_messages_.msg_state.action_states.size());
        message.action_states[i].action_id = order_messages_.msg_state.action_states[i].action_id;
        message.action_states[i].action_type = order_messages_.msg_state.action_states[i].action_type;
        message.action_states[i].action_description = order_messages_.msg_state.action_states[i].action_description;
        message.action_states[i].action_status = order_messages_.msg_state.action_states[i].action_status;
    }
    // 当前位姿
    message.agv_position.x = current_pose_.current_x;
    message.agv_position.y = current_pose_.current_y;
    message.agv_position.theta = current_pose_.current_theta;

    // 电量
    message.battery_state.battery_charge = battery_messages_.battery_level;
    message.battery_state.charging = battery_messages_.battery_status;
    message.battery_state.voltage = battery_messages_.total_voltage + 2; // 应rcs要求，这边加上2再发给rcs
    message.battery_state.current = battery_messages_.total_current;

    // 地图信息
    message.agv_position.map_id = agv_config.agv_position.map_id;
    message.agv_position.position_initialized = agv_config.agv_position.position_initialized;

    // 运行模式
    if (can_data_listener_ && can_data_listener_->get_operation_mode() == 0) {
        message.operating_mode = "MANUAL";
    } else {
        message.operating_mode = "AUTOMATIC";
    }

    // 错误信息填充
    if (fault_code != "NONE") {
        vda5050_interfaces::msg::Error error;
        error.error_type = fault_code;
        error.error_description = "AGV故障: " + fault_code;
        error.error_hint = "需要人工干预";
        error.error_level = "FATAL";
        // error_references 可以为空数组
        error.error_references.resize(0);
        
        // 将错误添加到errors数组
        message.errors.resize(1);
        message.errors[0] = error;
    } else {
        // 如果没有错误，errors数组为空
        message.errors.resize(0);
    }
    // 数据发布
    state_publisher_->publish(message);
    
    // MQTT发布 (QoS 0: AGV状态信息，频率较高，可以丢失，避免缓冲区积压)
    if (mqtt_enabled_ && mqtt_client_->isConnected()) {
        std::string topic = "uagv/v1/BYD/" + agv_config.serial_number + "/state";
        std::string json_payload = JsonConverter::toJson(message);
        mqtt_client_->publish(topic, json_payload, 0);
    }
    RCLCPP_INFO(node_->get_logger(),"state_timer_callback回调结束");
}

/*****************************************************************************************
* @brief:      visualization话题定时器的回调函数，用于定时发布visualization数据
* @param:      当前位姿
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
* @note:       后期需要补充线速度和角度
******************************************************************************************/
void AGVDataPublish::visualization_timer_callback(){
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 更新最新数据（如果提供了listener/server指针）
    if (current_pose_listener_) {
        current_pose_ = current_pose_listener_->get_current_pose();
    }
    if (velocity_listener_) {
        velocity_messages_ = velocity_listener_->get_velocity_messages();
    }
    
    // 实例化对象
    auto message = AGVVisualization();
    
    // 获取当前时间点
    auto date = std::chrono::system_clock::now();  

    // 转换为自epoch以来的时间点
    auto time_t_now = std::chrono::system_clock::to_time_t(date);

    // 转换为tm结构体，以便于格式化
    std::tm tm_now = *std::localtime(&time_t_now);

    // 格式化tm结构体为字符串
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_now);

    std::string time_string(buffer);

    header_id_visualization++;
    // 数据填充
    message.header_id = header_id_visualization;
    message.timestamp = time_string;
    message.version = agv_config.version;
    message.manufacturer = agv_config.manufacturer;
    message.serial_number = agv_config.serial_number;

    // 位姿数据
    message.agv_position.x = current_pose_.current_x;
    message.agv_position.y = current_pose_.current_y;
    message.agv_position.theta = current_pose_.current_theta;
    message.agv_position.map_id = agv_config.agv_position.map_id;
    message.agv_position.position_initialized = true;
    message.agv_position.localization_score = 0.8;
    // 速度数据（实时更新）
    message.velocity.vx = velocity_messages_.velocity_x;
    message.velocity.vy = velocity_messages_.velocity_y;
    message.velocity.omega = velocity_messages_.omega;

    visualization_publisher_->publish(message);    
    
    // MQTT发布 (QoS 0: 实时位姿数据，频率高，可以丢失，避免缓冲区积压)
    if (mqtt_enabled_ && mqtt_client_->isConnected()) {
        std::string topic = "uagv/v1/BYD/" + agv_config.serial_number + "/visualization";
        std::string json_payload = JsonConverter::toJson(message);
        mqtt_client_->publish(topic, json_payload, 0);
    }

}

/*****************************************************************************************
* @brief:      factsheet话题定时器的回调函数，用于定时发布factsheet数据
* @param:      当前位姿
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-10
* @version:    V0.0
******************************************************************************************/
void AGVDataPublish::factsheet_timer_callback(){
    // 实例化对象
    auto message = AGVFactsheet();

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
    header_id_factsheet++;
    // 数据填充
    message.header_id = header_id_factsheet;
    message.timestamp = time_string;
    message.version = agv_config.version;
    message.manufacturer = agv_config.manufacturer;
    message.serial_number = agv_config.serial_number;

    message.type_specification.series_name       = agv_config.type_specification.series_name;
    message.type_specification.agv_kinematic     = agv_config.type_specification.agv_kinematic;
    message.type_specification.agv_class         = agv_config.type_specification.agv_class;
    message.type_specification.max_load_mass     = agv_config.type_specification.max_load_mass;
    message.type_specification.navigation_types  = agv_config.type_specification.navigation_types;
    
    message.physical_parameters.speed_min        = agv_config.physical_parameters.speed_min;
    message.physical_parameters.speed_max        = agv_config.physical_parameters.speed_max;
    message.physical_parameters.acceleration_max = agv_config.physical_parameters.acceleration_max;
    message.physical_parameters.deceleration_max = agv_config.physical_parameters.deceleration_max;
    message.physical_parameters.height_max       = agv_config.physical_parameters.height_max;
    message.physical_parameters.width            = agv_config.physical_parameters.width;
    message.physical_parameters.length           = agv_config.physical_parameters.length;
    
    // 数据发布
    factsheet_publisher_->publish(message);
    
    // MQTT发布 (QoS 0: 设备规格信息，更新频率低，丢失影响不大)
    if (mqtt_enabled_ && mqtt_client_->isConnected()) {
        std::string topic = "uagv/v1/BYD/" + agv_config.serial_number + "/factsheet";
        std::string json_payload = JsonConverter::toJson(message);
        mqtt_client_->publish(topic, json_payload, 0);
    }

}

/*****************************************************************************************
* @brief:      小车状态发布定时器回调函数，用于定时发布VehicleStatus消息
* @param:      无
* @return:     无
* @author:     Assistant
* @date:       2024-11-20
* @version:    V1.0
******************************************************************************************/
void AGVDataPublish::status_timer_callback(){
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 更新最新数据（如果提供了listener/server指针）
    if (current_pose_listener_) {
        current_pose_ = current_pose_listener_->get_current_pose();
    }
    if (order_listener_) {
        order_messages_ = order_listener_->get_order_messages();
    }
    
    // 创建VehicleStatus消息
    auto message = VehicleStatus();
    
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

    header_id_status++;

    // 基础消息头部信息
    message.header_id = header_id_status;
    message.timestamp = time_string;
    message.version = agv_config.version;
    message.manufacturer = agv_config.manufacturer;
    message.serial_number = agv_config.serial_number;

    // 小车运行状态判断
    // 根据agv_current_state来判断status
    if (agv_current_state_ == "error" || agv_current_state_ == "lock") {
        message.status = 2;  // ERROR状态 (红灯)
        message.error_code = (agv_current_state_ == "error") ? 1 : 2;
        message.error_message = (agv_current_state_ == "error") ? "AGV处于错误状态" : "AGV处于锁定状态";
        message.warning_message = "";
    } else {
        message.status = 1;  // ONLINE状态 (绿灯) - 所有其他状态
        message.error_code = 0;
        message.error_message = "";
        
        // 更新最新数据（如果提供了listener/server指针）
        if (battery_listener_) {
            battery_messages_ = battery_listener_->get_battery_messages();
        }
        if (obstacle_server_) {
            obstacle_messages_ = obstacle_server_->get_obstacle_messages();
        }
        
        // 根据其他条件设置警告信息
        if (battery_messages_.battery_level < 20.0) {
            message.warning_message = "电池电量过低";
        } else if (!(mqtt_enabled_ && mqtt_client_ && mqtt_client_->isConnected())) {
            message.warning_message = "RCS连接断开";
        } else {
            message.warning_message = "";
        }
        
        // 系统状态详情
        message.is_connected = (mqtt_enabled_ && mqtt_client_ && mqtt_client_->isConnected());
        message.is_charging = battery_messages_.battery_status;  // 充电状态
        message.is_moving = (abs(current_pose_.current_x - order_messages_.current_goal_x) > 0.1 || 
                            abs(current_pose_.current_y - order_messages_.current_goal_y) > 0.1);  // 是否在移动
        message.is_emergency_stop = (obstacle_messages_ == 2 || obstacle_messages_ == 3);  // 急停状态

        // 附加状态信息
        message.battery_level = battery_messages_.battery_level;
    }
    
    message.current_task = order_messages_.msg_state.order_id;
    
    // 位置信息格式化
    std::ostringstream location_stream;
    location_stream << std::fixed << std::setprecision(2) 
                   << "(" << current_pose_.current_x 
                   << "," << current_pose_.current_y 
                   << "," << current_pose_.current_theta << ")";
    message.location = location_stream.str();

    // 系统时间戳（毫秒）
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    message.system_timestamp = millis;

    // 发布消息
    status_publisher_->publish(message);
    
    // 可选：记录日志
    if (header_id_status % 50 == 0) {  // 每5秒记录一次日志(0.1s * 50 = 5s)
        RCLCPP_INFO(node_->get_logger(), 
                   "发布小车状态: agv_state=%s, status=%d, battery=%.1f%%, task=%s, location=%s", 
                   agv_current_state_.c_str(), message.status, message.battery_level, 
                   message.current_task.c_str(), message.location.c_str());
    }
}

/*****************************************************************************************
* @brief:      设置驱动控制对象（用于获取driving状态）
* @param:      get_driving_fn 获取driving状态的函数对象
* @return:     无
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
* @note:       使用函数对象可以兼容LaserDriverControl和QRDriverControl两种类型
******************************************************************************************/
void AGVDataPublish::set_driving_fn(std::function<bool()> get_driving_fn){
    get_driving_fn_ = get_driving_fn;
}
