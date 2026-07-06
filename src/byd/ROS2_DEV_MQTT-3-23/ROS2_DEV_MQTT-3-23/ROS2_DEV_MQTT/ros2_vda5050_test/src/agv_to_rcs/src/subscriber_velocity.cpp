/************************************** File Info ****************************************
* @file:       subscriber_velocity.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2025-03-05                                        
* @version:    V0.0         
* @class:      订阅者                                                                     
* @brief:      速度相关数据的订阅方
* @note:       类内更新数据，通过get方法获取，避免读写冲突
******************************************************************************************/

# include "subscriber_velocity.h"

/*****************************************************************************************
* @brief:      速度数据订阅类的构造函数，用于初始化订阅者、定时器等
* @param:      node：ROS2节点指针
* @return:     无
* @author:     刘鸿彬
* @date:       2025-03-05
* @version:    V0.0
* @modified:   改为类内存储数据，不再使用引用参数
******************************************************************************************/
VelocityListener::VelocityListener(std::shared_ptr<rclcpp::Node> node):node_(node){
    
    RCLCPP_INFO(node_->get_logger(),"created a velocity subscriber!");

    subscription_ = node_->create_subscription<Odometry>("odom",1,std::bind(&VelocityListener::do_cb,this,std::placeholders::_1));

    timer_ = node_->create_wall_timer(std::chrono::seconds(1), std::bind(&VelocityListener::timer_callback,this));

    // 初始化为默认值
    velocity_messages_.velocity_x = 0.0;
    velocity_messages_.velocity_y = 0.0;
    velocity_messages_.omega = 0.0;
    velocity_messages_.acceleration = 0.0;

    // 是否接收到数据
    get_velocity = false;

    last_msg_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      返回速度数据
* @param:      无
* @return:     速度数据结构体
* @author:     刘鸿彬
* @date:       2025-03-05
* @version:    V0.0
* @modified:   改为类内存储数据，通过get方法获取
******************************************************************************************/
VelocityMessages VelocityListener::get_velocity_messages(){

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    return velocity_messages_;
}

/*****************************************************************************************
* @brief:      订阅回调函数，用于订阅数据的解析
* @param:      &msg ： 订阅到的数据
* @return:     无
* @author:     刘鸿彬
* @date:       2025-03-05
* @version:    V0.0
* @modified:   使用互斥锁保护数据写入
******************************************************************************************/
void VelocityListener::do_cb(const Odometry &msg){

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 更新是否获取速度数据的状态
    get_velocity = true;

    double old_velocity = std::sqrt(velocity_messages_.velocity_x * velocity_messages_.velocity_x + velocity_messages_.velocity_y * velocity_messages_.velocity_y);
    
    // 数据解析
    velocity_messages_.velocity_x = msg.twist.twist.linear.x;
    velocity_messages_.velocity_y = msg.twist.twist.linear.y;
    velocity_messages_.omega      = msg.twist.twist.angular.z;
    
    double new_velocity = std::sqrt(velocity_messages_.velocity_x * velocity_messages_.velocity_x + velocity_messages_.velocity_y * velocity_messages_.velocity_y);
    
    // 更新最后一条数据的时间
    auto start = last_msg_time_;
    last_msg_time_ = std::chrono::steady_clock::now();
    // 计算时间差，单位为秒（浮点数）
    double elapsed_seconds = std::chrono::duration<double>(last_msg_time_ - start).count();

    velocity_messages_.acceleration = (new_velocity - old_velocity)/elapsed_seconds;
}

/*****************************************************************************************
* @brief:      定时器回调函数，用于检测是否接受到数据或中间时刻连接中断
* @param:      无
* @return:     无
* @author:     刘鸿彬
* @date:       2025-03-05
* @version:    V0.0
* @modified:   添加超时检测，更新get_velocity标识
******************************************************************************************/
void VelocityListener::timer_callback(){
    // 加锁保护共享数据
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 如果从未收到过消息，不检查超时（避免启动时的误报）
    if (!get_velocity) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    // 计算时间差
    auto diff = now - last_msg_time_;
    auto time_diff_sec = std::chrono::duration_cast<std::chrono::seconds>(diff).count();
    
    // 检查自上次接收消息以来的时间是否超过了超时阈值
    if (time_diff_sec > 10) {
        RCLCPP_ERROR(node_->get_logger(), "超过10s未接收到velocity消息 (已过%ld秒)", time_diff_sec);
        // 更新接受速度数据标识
        get_velocity = false;
    }
}

