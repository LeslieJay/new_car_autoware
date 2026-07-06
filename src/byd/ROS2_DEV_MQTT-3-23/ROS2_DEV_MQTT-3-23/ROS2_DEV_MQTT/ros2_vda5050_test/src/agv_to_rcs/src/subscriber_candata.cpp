/************************************** File Info ****************************************
* @file:       subscriber_candata.cpp                                                                     
* @author:     Auto                                                              
* @date:       2025-01-XX                                          
* @version:    V0.0         
* @class:      订阅者                                                                     
* @brief:      CAN数据订阅方（IO数据、错误数据、硬件数据）
* @note:       类内更新数据，通过get方法获取，避免读写冲突
******************************************************************************************/

# include "subscriber_candata.h"

/*****************************************************************************************
* @brief:      CAN数据订阅类的构造函数，用于初始化订阅者、定时器等
* @param:      node：ROS2节点指针
* @return:     无
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
CanDataListener::CanDataListener(std::shared_ptr<rclcpp::Node> node):node_(node){
    
    RCLCPP_INFO(node_->get_logger(),"created a CAN data subscriber!");

    // 创建IO数据订阅者
    io_data_subscription_ = node_->create_subscription<ref_slam_interface::msg::IoData>(
        "io_data", 1, 
        std::bind(&CanDataListener::io_data_callback, this, std::placeholders::_1));

    // 创建错误数据订阅者
    err_data_subscription_ = node_->create_subscription<ref_slam_interface::msg::ErrData>(
        "err_data", 1, 
        std::bind(&CanDataListener::err_data_callback, this, std::placeholders::_1));

    // 创建硬件数据订阅者
    hardware_data_subscription_ = node_->create_subscription<ref_slam_interface::msg::HardwareData>(
        "hardware_data", 1, 
        std::bind(&CanDataListener::hardware_data_callback, this, std::placeholders::_1));

    // 创建vcu错误订阅者
    vcu_subscription_ = node_->create_subscription<vda5050_interfaces::msg::Error>(
        "error", 1, 
        std::bind(&CanDataListener::error_callback, this, std::placeholders::_1));
        
    // 创建二维码自动切换订阅者
    qr_automatic_switch_subscription_ = node_->create_subscription<agv_interfaces::msg::AutomaticSwitch>(
        agv_config.serial_number + "/automatic_switch_", 1, 
        std::bind(&CanDataListener::qr_automatic_switch_callback, this, std::placeholders::_1));

    // 创建定时器，用于检测数据接收超时
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), 
        std::bind(&CanDataListener::timer_callback, this));

    // 初始化接收标识
    get_io_data_flag = false;
    get_err_data_flag = false;
    get_hardware_data_flag = false;
    get_qr_automatic_switch_flag = false;

    io_data_is_error = false;
    err_data_is_error = false;
    hardware_data_is_error = false;

    vcu_watch_dog_out_count = 0;

    update_load_status = 0;

    // 记录上一条消息的时间
    last_io_data_time_ = std::chrono::steady_clock::now();
    last_err_data_time_ = std::chrono::steady_clock::now();
    last_hardware_data_time_ = std::chrono::steady_clock::now();
    last_qr_automatic_switch_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      返回IO数据
* @param:      无
* @return:     IO数据结构体
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
ref_slam_interface::msg::IoData CanDataListener::get_io_data(){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    return io_data_;
}

/*****************************************************************************************
* @brief:      返回错误数据
* @param:      无
* @return:     错误数据结构体
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
ref_slam_interface::msg::ErrData CanDataListener::get_err_data(){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    return err_data_;
}

/*****************************************************************************************
* @brief:      返回硬件数据
* @param:      无
* @return:     硬件数据结构体
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
ref_slam_interface::msg::HardwareData CanDataListener::get_hardware_data(){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    return hardware_data_;
}

/*****************************************************************************************
* @brief:      获取操作模式
* @param:      无
* @return:     0：手动模式  1：自动模式
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
bool CanDataListener::get_operation_mode(){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (agv_config.vehicle_type == "laser")
    {
        // RCLCPP_INFO(node_->get_logger(),"io_data_.auto_manual_switch: %d",io_data_.auto_manual_switch);
        // 强制设置为自动模式
        return 1;
        // return io_data_.auto_manual_switch;
    }
    else if (agv_config.vehicle_type == "qr")
    {
        // RCLCPP_INFO(node_->get_logger(),"qr_automatic_switch_.automatic_status: %d",qr_automatic_switch_.automatic_status);
        // return qr_automatic_switch_.automatic_status;
        return 1;
    }
    else
    {
        return false;  // 不支持的操作模式
    }
}

/*****************************************************************************************
* @brief:      IO数据订阅回调函数
* @param:      msg：订阅到的IO数据
* @return:     无
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
void CanDataListener::io_data_callback(const ref_slam_interface::msg::IoData &msg){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    io_data_is_error = false;

    // 更新是否获取IO数据的状态
    get_io_data_flag = true;

    // 检查IO数据是否异常
    if(msg.epec_all_ok == false){
        io_data_is_error = true;
        RCLCPP_INFO(node_->get_logger(),"epec_all_ok发生异常!");
    }

    // 如果screen_req_blackbox由false转变为true，则视为有问题
    if(io_data_.screen_req_blackbox == false && msg.screen_req_blackbox == true){
        io_data_is_error = true;
        RCLCPP_INFO(node_->get_logger(),"用户触发黑盒!");
    }

    // 如果vcu_watch_dog_out连续10次为相同值，则视为有问题
    if(msg.vcu_watch_dog_out == io_data_.vcu_watch_dog_out){
        vcu_watch_dog_out_count++;
        if(vcu_watch_dog_out_count >= 10){
            io_data_is_error = true;
            RCLCPP_INFO(node_->get_logger(),"vcu_watch_dog_out发生异常!");
        }
    } else {
        // 值发生变化，重置计数
        vcu_watch_dog_out_count = 1;
    }

    // 如果screen_set_load_status由0变为非0值，则设置小车当前载货状态为对应状态  1:有货  2：无货  3：不变
    if(io_data_.screen_set_load_status == 0 && msg.screen_set_load_status != 0){
        update_load_status = msg.screen_set_load_status;
    }

    // 数据存储
    io_data_ = msg;
    
    // 更新最后一条数据的时间
    last_io_data_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      错误数据订阅回调函数
* @param:      msg：订阅到的错误数据
* @return:     无
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
void CanDataListener::err_data_callback(const ref_slam_interface::msg::ErrData &msg){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 更新是否获取错误数据的状态
    get_err_data_flag = true;

    // 检查err数据是否异常
    err_data_is_error   =  msg.cvc_can_err 
                        || msg.driver_controller_can_err 
                        || msg.steer_controller_can_err 
                        || msg.pump_controller_can_err 
                        || msg.bms_can_err 
                        || msg.height_encode_can_err 
                        || msg.left_laser_can_err 
                        || msg.right_laser_can_err 
                        || msg.head_laser_can_err 
                        || msg.back_laser_can_err 
                        || msg.turning_encode_can_err 
                        || msg.ele_cylinder_can_err 
                        || msg.charger_can_err 
                        || msg.driver_controller_err 
                        || msg.steer_controller_err 
                        || msg.back_laser_sensor_err 
                        || msg.head_laser_sensor_err 
                        || msg.left_laser_err 
                        || msg.right_laser_err 
                        || msg.height_detect_sensor_err 
                        || msg.charger_err 
                        || msg.cvc_controller_err 
                        || msg.remote_err 
                        || msg.battery_bms_err 
                        || msg.speed_over_limit1 
                        || msg.forklift_over_time 
                        || msg.fork_drop_over_time 
                        || msg.ele_cylinder_err;
    if(err_data_is_error)
    {
        RCLCPP_INFO(node_->get_logger(),"错误码发生异常!");
    }

    // 数据存储
    err_data_ = msg;
    
    // 更新最后一条数据的时间
    last_err_data_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      硬件数据订阅回调函数
* @param:      msg：订阅到的硬件数据
* @return:     无
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
void CanDataListener::hardware_data_callback(const ref_slam_interface::msg::HardwareData &msg){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 更新是否获取硬件数据的状态
    get_hardware_data_flag = true;

    // 数据存储
    hardware_data_ = msg;
    
    // 更新最后一条数据的时间
    last_hardware_data_time_ = std::chrono::steady_clock::now();
}

void CanDataListener::error_callback(const vda5050_interfaces::msg::Error &msg){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    if(msg.error_type == "hardware_failure"){
        hardware_data_is_error = true;
    } else {
        hardware_data_is_error = false;
    }
}

void CanDataListener::qr_automatic_switch_callback(const agv_interfaces::msg::AutomaticSwitch &msg){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    get_qr_automatic_switch_flag = true;

    // 数据存储
    qr_automatic_switch_ = msg;

    // 更新最后一条数据的时间
    last_qr_automatic_switch_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      定时器回调函数，用于检测是否接受到数据或中间时刻连接中断
* @param:      无
* @return:     无
* @author:     Auto
* @date:       2025-01-XX
* @version:    V0.0
******************************************************************************************/
void CanDataListener::timer_callback(){
    // 加锁保护共享数据
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    
    // 检查IO数据超时
    if (get_io_data_flag) {
        auto diff = now - last_io_data_time_;
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
        if (time_diff_ms > 1000*10) {
            RCLCPP_WARN(node_->get_logger(), "超过1000*10ms未接收到io_data消息 (已过%ld毫秒)", time_diff_ms);
            get_io_data_flag = false;
        }
    }
    
    // 检查错误数据超时
    if (get_err_data_flag) {
        auto diff = now - last_err_data_time_;
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
        if (time_diff_ms > 1000*10) {
            RCLCPP_WARN(node_->get_logger(), "超过1000*10ms未接收到err_data消息 (已过%ld毫秒)", time_diff_ms);
            get_err_data_flag = false;
        }
    }
    
    // 检查硬件数据超时
    if (get_hardware_data_flag) {
        auto diff = now - last_hardware_data_time_;
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
        if (time_diff_ms > 1000*10) {
            RCLCPP_WARN(node_->get_logger(), "超过1000*10ms未接收到hardware_data消息 (已过%ld毫秒)", time_diff_ms);
            get_hardware_data_flag = false;
        }
    }

    // 检查二维码自动切换超时
    if (get_qr_automatic_switch_flag) {
        auto diff = now - last_qr_automatic_switch_time_;
        auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
        if (time_diff_ms > 1000*10) {
            RCLCPP_WARN(node_->get_logger(), "超过1000*10ms未接收到qr_automatic_switch消息 (已过%ld毫秒)", time_diff_ms);
            get_qr_automatic_switch_flag = false;
        }
    }
}

