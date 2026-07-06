/************************************** File Info ****************************************
* @file:       subscriber_battery.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-21                                          
* @version:    V0.0         
* @class:      订阅者                                                                     
* @brief:      电池状态相关数据的订阅方
* @note:       类内更新数据，通过get方法获取，避免读写冲突
******************************************************************************************/

# include "subscriber_battery.h"

/*****************************************************************************************
* @brief:      电池数据订阅类的构造函数，用于初始化订阅者、定时器等
* @param:      node：ROS2节点指针
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-21
* @version:    V0.0
* @modified:   改为类内存储数据，不再使用引用参数
******************************************************************************************/
BatteryListener::BatteryListener(std::shared_ptr<rclcpp::Node> node):node_(node){
    
    RCLCPP_INFO(node_->get_logger(),"created a battery subscriber!");

    // 根据vehicle_type设置话题名称
    std::string topic_name;
    if (agv_config.vehicle_type == "laser") {
        // 激光导航：固定话题名称
        topic_name =  "battery";
    } else {
        // 二维码导航：使用序列号作为前缀
        std::string SerialNumber = agv_config.serial_number;
        topic_name = SerialNumber + "/battery_state_";
    }

    subscription_ = node_->create_subscription<BatteryState>(topic_name,1,std::bind(&BatteryListener::do_cb,this,std::placeholders::_1));

    timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&BatteryListener::timer_callback,this));

    // 初始化为默认值
    battery_messages_.battery_level = 0.0;
    battery_messages_.battery_status = 0.0;
    battery_messages_.total_voltage = 0.0;
    battery_messages_.total_current = 0.0;

    // 是否接收到数据
    get_battery = false;

    // 记录上一条消息的时间
    last_msg_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      返回电池数据
* @param:      无
* @return:     电池数据结构体
* @author:     刘鸿彬
* @date:       2024-11-21
* @version:    V0.0
* @modified:   改为类内存储数据，通过get方法获取
******************************************************************************************/
BatteryMessages BatteryListener::get_battery_messages(){

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    return battery_messages_;
}

/*****************************************************************************************
* @brief:      订阅回调函数，用于订阅数据的解析
* @param:      &msg ： 订阅到的数据
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-21
* @version:    V0.0
* @modified:   使用互斥锁保护数据写入
******************************************************************************************/
void BatteryListener::do_cb(const BatteryState &msg){

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 更新是否获取电池数据的状态
    get_battery = true;
    // RCLCPP_INFO(node_->get_logger(), "订阅充电电流电压");

    // 数据解析
    battery_messages_.battery_level = msg.battery_level;
    battery_messages_.battery_status = msg.battery_status;
    battery_messages_.total_voltage = msg.total_voltage;
    battery_messages_.total_current = msg.total_current;
    
    // 更新最后一条数据的时间
    last_msg_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      定时器回调函数，用于检测是否接受到数据或中间时刻连接中断
* @param:      无
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-21
* @version:    V0.0
* @modified:   添加超时检测，更新get_battery标识
******************************************************************************************/
void BatteryListener::timer_callback(){
    // 加锁保护共享数据
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 如果从未收到过消息，不检查超时（避免启动时的误报）
    if (!get_battery) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    // 计算时间差
    auto diff = now - last_msg_time_;
    auto time_diff_sec = std::chrono::duration_cast<std::chrono::seconds>(diff).count();
    
    // 检查自上次接收消息以来的时间是否超过了超时阈值
    if (time_diff_sec > 10) {
        RCLCPP_ERROR(node_->get_logger(), "超过10s未接收到battery消息 (已过%ld秒)", time_diff_sec);
        // 更新接受电池数据标识
        get_battery = false;
    }
}



// int main(int argc,char const *argv[]){
//     // 2.initialize ROS2 client
//     rclcpp::init(argc,argv);
//     // 4.call the spin function and pass in the node object pointer
//     rclcpp::spin(std::make_shared<BatteryListener>());
//     // 5.release resources
//     rclcpp::shutdown();

//     return 0;
// }

