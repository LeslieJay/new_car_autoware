/************************************** File Info ****************************************
* @file:       control_agv_fork.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-05                                         
* @version:    V0.0                                                                              
* @brief:      AGV货叉控制接口实现
******************************************************************************************/
# include "control_agv_fork.h"

LaserForkControl::LaserForkControl(AGVBone agv_bone):
    current_fork_height_(0)
{
    // 实例化货叉控制客户端
    forkActionClient = std::make_shared<LsaerForkActionClient>(agv_bone.agv_all_nodes);
}

/*****************************************************************************************
* @brief:      AGV货叉控制的实现函数
* @param:      无
* @return:     驱动成功返回true否则返回false
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
bool LaserForkControl::control(){

    set_flag_finish(false); // 复位，表示货叉动作未完成

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "货叉目标高度：%d",forkHeight);
    // 动作发送
    forkActionClient->send_fork_goal(forkEnable,forkHeight);

    current_fork_height_ = forkHeight;

    return true;
}

int LaserForkControl::get_fork_height() const {
    return current_fork_height_;
}

/*****************************************************************************************
* @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
* @param:      forkEnable：货叉控制使能位（true：开启/false：关闭）
* @param:      forkHeight：货叉目标高度
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
void LaserForkControl::setForkParameters(bool forkEnable,int forkHeight){
    this->forkEnable = forkEnable;
    this->forkHeight = forkHeight;
}

bool LaserForkControl::get_flag_finish(){

    return forkActionClient->flag_finish;
}

void LaserForkControl::set_flag_finish(bool flag){
    forkActionClient->flag_finish = flag;

}

bool LaserForkControl::get_flag_aborted(){
    return forkActionClient->flag_aborted;
}

bool LaserForkControl::get_flag_driving(){
    return forkActionClient->flag_driving;
}

/*****************************************************************************************
* @brief:      取消正在执行的货叉动作
* @param:      无
* @return:     取消成功返回true，否则返回false
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
bool LaserForkControl::cancel(){
    if(forkActionClient) {
        forkActionClient->cancel_action();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "已取消货叉动作");
        return true;
    }
    return false;
}


// -----------------------------------------------------------------------------------------


QRForkControl::QRForkControl(AGVBone agv_bone):
    current_fork_height_(0)
{
    // 实例化货叉控制客户端
    forkActionClient = std::make_shared<QRForkActionClient>(agv_bone.agv_all_nodes);
}

/*****************************************************************************************
* @brief:      AGV货叉控制的实现函数
* @param:      无
* @return:     驱动成功返回true否则返回false
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
bool QRForkControl::control(){

    set_flag_finish(false);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "货叉目标高度：%d",forkHeight);
    // 动作发送
    forkActionClient->send_fork_goal(forkEnable,forkHeight, cargoRotation);

    current_fork_height_ = forkHeight;

    return true;
}

int QRForkControl::get_fork_height() const {
    return current_fork_height_;
}

/*****************************************************************************************
* @brief:      获取外界的数据，将外界数据传输给类内变量，为驱动提供参数
* @param:      forkEnable：货叉控制使能位（true：开启/false：关闭）
* @param:      forkHeight：货叉目标高度
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-06
******************************************************************************************/
void QRForkControl::setForkParameters(bool forkEnable,int forkHeight, int cargoRotation){
    this->forkEnable = forkEnable;
    this->forkHeight = forkHeight;
    this->cargoRotation = cargoRotation;
}

bool QRForkControl::get_flag_finish(){

    return forkActionClient->flag_finish;
}

void QRForkControl::set_flag_finish(bool flag){
    forkActionClient->flag_finish = flag;

}

bool QRForkControl::get_flag_aborted(){
    return forkActionClient->flag_aborted;
}

bool QRForkControl::get_flag_driving(){
    return forkActionClient->flag_driving;
}

/*****************************************************************************************
* @brief:      取消正在执行的货叉动作
* @param:      无
* @return:     取消成功返回true，否则返回false
* @author:     Assistant
* @date:       2024-12-XX
* @version:    V1.0
******************************************************************************************/
bool QRForkControl::cancel(){
    if(forkActionClient) {
        forkActionClient->cancel_action();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "已取消货叉动作");
        return true;
    }
    return false;
}
