/************************************** File Info ****************************************
* @file:       control_agv_driver.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-05                                         
* @version:    V0.0                                                                              
* @brief:      AGV驱动接口实现
******************************************************************************************/
# include "control_agv_driver.h"

// 静态成员变量定义
std::shared_ptr<LaserDriverControl> LaserDriverControl::instance_ = nullptr;
std::mutex LaserDriverControl::mutex_;

std::shared_ptr<QRDriverControl> QRDriverControl::instance_ = nullptr;
std::mutex QRDriverControl::mutex_;

LaserDriverControl::LaserDriverControl(AGVBone agv_bone){
    // 实例化驱动对象
    send_multi_pose_ =  std::make_shared<LaserSendMultiPose>(agv_bone.agv_all_nodes, agv_bone.agv_current_pose_listener);
}


/*****************************************************************************************
* @brief:      AGV驱动的实现函数，注意该任务有抢占需求，所以不能简单的使用spin族函数
* @param:      无
* @return:     驱动成功返回true否则返回false
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
bool LaserDriverControl::control(){
    // // 原版send_request用srv发送轨迹，用于校验轨迹可行性，现在不管了，直接发送目标点
    // // 告知EXC准备进行轨迹导航
    // auto future_and_requestid = send_multi_pose_->send_request(goal_points);
    
    // // 如果goal_points为空，则输出提示并直接返回true
    // if(goal_points.empty())
    // {
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal points is empty!");
    //     return true;
    // }

    // auto result = future_and_requestid.future.get();

    // int connect_times = 0;
    // while(result->response != true)
    // {
    //     connect_times++;
    //     if(connect_times>10)
    //     {
    //         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "10 times fail to start send_multi_pose!");
    //         return false;
    //     }
    //     auto future_and_requestid = send_multi_pose_->send_request(goal_points);
    //     auto result = future_and_requestid.future.get();
    // }
    // // 发送目标位姿
    std::vector<Point> multi_poses = {goal_points.back()}; //发给多点导航的点集只需要头尾即可
    send_multi_pose_->send_goal(multi_poses);

    // 驱动成功返回true否则返回false;目前默认成功
    return true;
}

/*****************************************************************************************
* @brief:      取消任务
* @param:      无
* @return:     取消任务成功返回true否则返回false
* @author:     刘鸿彬
* @date:       2025-04-29
******************************************************************************************/
bool LaserDriverControl::cancel(){

    send_multi_pose_->cancel_action();
    // 成功返回true否则返回false;目前默认成功
    return true;
}

/*****************************************************************************************
* @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
* @param:      goal_x：x轴坐标点容器
* @param:      goal_y：y轴坐标点容器
* @param:      goal_theta：旋转角度容器
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
void LaserDriverControl::setPostion(std::vector<Point> points){
    
    this->goal_points = points;
}

bool LaserDriverControl::get_flag_finish(){

    return send_multi_pose_->flag_finish;
}

bool LaserDriverControl::get_flag_aborted(){

    return send_multi_pose_->flag_aborted;
}

bool LaserDriverControl::get_flag_driving(){

    return send_multi_pose_->flag_driving;
}

void LaserDriverControl::set_flag_finish(bool flag){
    send_multi_pose_->flag_finish = flag;

}

/*****************************************************************************************
* @brief:      获取单例实例（懒汉模式，线程安全）
* @param:      无
* @return:     单例实例的共享指针
* @author:     刘鸿彬
* @date:       2025-01-XX
* @version:    V1.0
******************************************************************************************/
std::shared_ptr<LaserDriverControl> LaserDriverControl::getInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
        // 如果未初始化，返回 nullptr
        return nullptr;
    }
    return instance_;
}

/*****************************************************************************************
* @brief:      初始化单例（需要在第一次使用前调用）
* @param:      agv_bone - AGV骨架信息
* @return:     无
* @author:     刘鸿彬
* @date:       2025-01-XX
* @version:    V1.0
******************************************************************************************/
void LaserDriverControl::initialize(AGVBone agv_bone) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
        instance_ = std::shared_ptr<LaserDriverControl>(new LaserDriverControl(agv_bone));
    }
}

// ---------------------------------------------------------------------------------------------------



QRDriverControl::QRDriverControl(AGVBone agv_bone){
    // 实例化驱动对象
    send_multi_pose_ =  std::make_shared<QRSendMultiPose>(agv_bone.agv_all_nodes, agv_bone.agv_current_pose_listener);
}


/*****************************************************************************************
* @brief:      AGV驱动的实现函数，注意该任务有抢占需求，所以不能简单的使用spin族函数
* @param:      无
* @return:     驱动成功返回true否则返回false
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
bool QRDriverControl::control(){

    // 发送目标位姿
    send_multi_pose_->send_goal(goal_poses_);

    // 驱动成功返回true否则返回false;目前默认成功
    return true;
}

/*****************************************************************************************
* @brief:      取消任务
* @param:      无
* @return:     取消任务成功返回true否则返回false
* @author:     刘鸿彬
* @date:       2025-04-29
******************************************************************************************/
bool QRDriverControl::cancel(){

    send_multi_pose_->cancel_action();
    // 成功返回true否则返回false;目前默认成功
    return true;
}

/*****************************************************************************************
* @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
* @param:      goal_x：x轴坐标点容器
* @param:      goal_y：y轴坐标点容器
* @param:      goal_theta：旋转角度容器
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
void QRDriverControl::setPostion(std::vector<agv_interfaces::msg::Poses> goal_poses){
    
    this->goal_poses_ = goal_poses;
}

bool QRDriverControl::get_flag_finish(){

    return send_multi_pose_->flag_finish;
}

bool QRDriverControl::get_flag_aborted(){

    return send_multi_pose_->flag_aborted;
}

bool QRDriverControl::get_flag_driving(){

    return send_multi_pose_->flag_driving;
}

void QRDriverControl::set_flag_finish(bool flag){
    send_multi_pose_->flag_finish = flag;

}

/*****************************************************************************************
* @brief:      获取单例实例（懒汉模式，线程安全）
* @param:      无
* @return:     单例实例的共享指针
* @author:     刘鸿彬
* @date:       2025-01-XX
* @version:    V1.0
******************************************************************************************/
std::shared_ptr<QRDriverControl> QRDriverControl::getInstance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
        // 如果未初始化，返回 nullptr
        return nullptr;
    }
    return instance_;
}

/*****************************************************************************************
* @brief:      初始化单例（需要在第一次使用前调用）
* @param:      agv_bone - AGV骨架信息
* @return:     无
* @author:     刘鸿彬
* @date:       2025-01-XX
* @version:    V1.0
******************************************************************************************/
void QRDriverControl::initialize(AGVBone agv_bone) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (instance_ == nullptr) {
        instance_ = std::shared_ptr<QRDriverControl>(new QRDriverControl(agv_bone));
    }
}

