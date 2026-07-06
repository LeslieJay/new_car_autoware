/************************************** File Info ****************************************
* @file:       subscriber_current_pose.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-01                                        
* @version:    V0.0  
* @class:      订阅者                                                                            
* @brief:      订阅当前agv的位姿，将四元数转化为弧度（激光导航）或直接获取角度（二维码导航）
* @note:       需要处理开始能够读取到数据，但是中途中断的情况，以及消息未发布的异常情况
------------------------------------------------------------------------------------------
* @modified:   增加一个接收消息超时判断
* @date:       2024-11-09
* @version:    V0.0
* @description:当出现中断的情况时，接收到的数据不会更新，如果不处理，会影响后续的判断
* @note:       
------------------------------------------------------------------------------------------
* @modified:   统一激光导航和二维码导航两个版本
* @date:       2025-01-XX
* @version:    V0.1
* @description:根据agv_config.vehicle_type区分处理差异，差异配置在各自的agv_config文件中
* @note:       
******************************************************************************************/
# include "subscriber_current_pose.h"

/*****************************************************************************************
* @brief:      类的构造函数
* @author:     刘鸿彬
* @date:       2024-11-11
* @modified:   根据vehicle_type设置不同的话题名称
******************************************************************************************/
ListenerPose::ListenerPose(std::shared_ptr<rclcpp::Node> node):node_(node){
    
    // 根据vehicle_type设置话题名称
    std::string topic_name;
    if (agv_config.vehicle_type == "laser") {
        // 激光导航：固定话题名称
        topic_name =  "map_to_base_pose";
    } else {
        // 二维码导航：使用序列号作为前缀
        std::string SerialNumber = agv_config.serial_number;
        topic_name = SerialNumber + "/current_pose";
    }
    
    // agv设备当前位姿订阅方创建
    current_pose_subscription_ = node_->create_subscription<PoseType>(
        topic_name, 1, std::bind(&ListenerPose::agv_pose_callback, this, std::placeholders::_1));

    current_pose_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&ListenerPose::timer_callback, this));
    
    // 初始化为负值
    current_x = -1;
    current_y = -1;
    current_theta = -1;
    
    // 是否接收到数据
    get_pose = false;

    // 记录上一条消息的时间
    last_msg_time_ = std::chrono::steady_clock::now();
}

/*****************************************************************************************
* @brief:      返回当前位姿
* @param:      无
* @return:     当前位姿的结构体
* @author:     刘鸿彬
* @date:       2024-11-11
* @version:    V0.0
******************************************************************************************/
CurrentPose ListenerPose::get_current_pose(){

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 数据填充
    current_pose.current_x = current_x;
    current_pose.current_y = current_y;
    current_pose.current_theta = current_theta;

    return current_pose;
}

/*****************************************************************************************
* @brief:      设置当前位姿（用于外部更新，如feedback回调）
* @param:      pose - 要设置的位姿数据
* @return:     无
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
void ListenerPose::set_current_pose(const CurrentPose& pose){
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_x = pose.current_x;
    current_y = pose.current_y;
    current_theta = pose.current_theta;
    // 同时更新 current_pose 结构体
    current_pose = pose;
    // 更新最后接收消息时间
    last_msg_time_ = std::chrono::steady_clock::now();
    get_pose = true;
}

/*****************************************************************************************
* @brief:      定时器回调函数，用于检测是否接受到数据或中间时刻连接中断
* @param:      无
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-11
* @version:    V0.0
******************************************************************************************/
void ListenerPose::timer_callback(){
    // 加锁保护共享数据
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 如果从未收到过消息，不检查超时（避免启动时的误报）
    if (!get_pose) {
        return;
    }
    
    auto now = std::chrono::steady_clock::now();
    // 计算时间差
    auto diff = now - last_msg_time_;
    auto time_diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
    
    // 检查自上次接收消息以来的时间是否超过了超时阈值（800ms）
    if (time_diff_ms > 800) {
        RCLCPP_ERROR(node_->get_logger(), "超过800ms未接收到current pose消息 (已过%ld毫秒)", time_diff_ms);
        // 更新接受当前位姿标识
        get_pose = false;
    }
}

/*****************************************************************************************
* @brief:      订阅方绑定函数,订阅位姿消息，根据vehicle_type进行不同的数据解析
* @param:      位姿数据（PoseType，根据配置可能是PoseWithCovarianceStamped或Point）
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-11
* @version:    V0.0
* @modified:   根据vehicle_type区分处理激光导航（四元数转弧度）和二维码导航（直接角度）
* @note:       由于PoseType在编译时已确定，根据vehicle_type执行对应分支（编译时类型已匹配）
******************************************************************************************/
void ListenerPose::agv_pose_callback(const PoseType &agv_state){

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 更新是否获取当前位姿的状态
    get_pose = true;
    
    // 根据编译时版本进行不同的数据解析
    // 注意：由于编译时PoseType已经确定（激光版本为PoseWithCovarianceStamped，二维码版本为Point），
    // 使用条件编译来确保代码的正确性
#ifdef AGV_CONFIG_QR_VERSION
    // 二维码导航：从Point中提取数据
    // 在编译二维码版本时，PoseType就是Point，可以直接访问
    // 获取AGV的位姿信息（坐标需要除以1000，角度从度转为弧度）
    current_x = agv_state.x / 1000.0;
    current_y = agv_state.y / 1000.0;
    current_theta = agv_state.angle * (M_PI / 180.0);
#else
    // 激光导航：从PoseWithCovarianceStamped中提取数据
    // 在编译激光版本时，PoseType就是PoseWithCovarianceStamped，可以直接访问
    current_x = agv_state.pose.pose.position.x;
    current_y = agv_state.pose.pose.position.y;
    quat_w = agv_state.pose.pose.orientation.w;
    quat_z = agv_state.pose.pose.orientation.z;

    // 四元数转弧度
    current_theta = 2 * acos(quat_w);

    // 正反转判别
    if (quat_z < 0) {
        current_theta = -current_theta;
    }
    
    // 降低日志输出频率，使用DEBUG级别
    RCLCPP_DEBUG(node_->get_logger(), "AGV当前位姿（%.2f,%.2f,%.2f）",current_x,current_y,current_theta);
#endif
    
    // 更新接收上条消息的时间
    last_msg_time_ = std::chrono::steady_clock::now();
}
