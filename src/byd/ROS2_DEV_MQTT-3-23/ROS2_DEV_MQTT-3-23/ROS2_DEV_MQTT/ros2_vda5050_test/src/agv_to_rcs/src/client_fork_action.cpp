#include "client_fork_action.h"

// 类的构造函数
LsaerForkActionClient::LsaerForkActionClient(std::shared_ptr<rclcpp::Node> node): 
    node_(node), 
    current_goal_handle_(nullptr)
{
    RCLCPP_INFO(node_->get_logger(),"create fork action client!");

    // 创建客户端
    fork_client_ = rclcpp_action::create_client<CtrlFork>(node_, "fork_server");

    // 初始化
    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;

}


/*****************************************************************************************
* @brief:      发送货叉控制的请求
* @param:      1、货叉控制使能，2、货叉高度
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-12
* @note：      1、请求连接货叉控制服务端，2、连接成功，进行数据填充并发布，3、接受反馈高度，到达目标后释放
* @version:    V0.0
******************************************************************************************/
void LsaerForkActionClient::send_fork_goal(int to_fork_signal,int goal_fork_height){

    RCLCPP_INFO(node_->get_logger(),"进行货叉驱动控制!");
    // 连接服务端
    if (!(fork_client_->wait_for_action_server(10s))){
        RCLCPP_ERROR(node_->get_logger(),"连接服务段超时!");
        return;
    }

    // 发送具体请求
    auto goal_msg = CtrlFork::Goal();
    // 发送数据填充
    goal_msg.fork_goal_height = goal_fork_height;
    goal_msg.to_fork_signal = to_fork_signal;

    // 配置动作客户端的行为
    rclcpp_action::Client<CtrlFork>::SendGoalOptions send_goal_options;
    // 设置动作服务器确认接收到目标后被调用的回调函数
    send_goal_options.goal_response_callback = std::bind(&LsaerForkActionClient::fork_goal_response_callback,this,_1);
    // 设置动作服务器发送反馈信息时被调用的回调函数
    send_goal_options.feedback_callback = std::bind(&LsaerForkActionClient::fork_feedback_callback,this,_1,_2);
    // 设置动作完成或被取消时被调用的回调函数
    send_goal_options.result_callback = std::bind(&LsaerForkActionClient::fork_result_callback,this,_1);
    
    // 重置标志
    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;
    
    // 异步地向动作服务器发送一个目标
    current_goal_future_ = fork_client_->async_send_goal(goal_msg,send_goal_options);
    
    // 记录发送任务的时间
    goal_send_time_ = std::chrono::steady_clock::now();

}

void LsaerForkActionClient::cancel_action(){

    if (current_goal_future_.valid() && current_goal_future_.wait_for(0s) == std::future_status::ready) {
        auto goal_handle = current_goal_future_.get();
        RCLCPP_INFO(node_->get_logger(), "Cancelling goal...");
        auto future_cancel = fork_client_->async_cancel_goal(goal_handle);
    } 
    else {
        RCLCPP_INFO(node_->get_logger(), "No active goal to cancel.");
    }
}

// 处理服务端反馈值
void LsaerForkActionClient::fork_goal_response_callback(rclcpp_action::ClientGoalHandle<CtrlFork>::SharedPtr goal_handle){
    
    if (!goal_handle){
        RCLCPP_INFO(node_->get_logger(),"服务端拒绝请求!");

    }else{
        RCLCPP_INFO(node_->get_logger(),"请求通过!");
    }
}

// 处理连续反馈值
void LsaerForkActionClient::fork_feedback_callback(rclcpp_action::ClientGoalHandle<CtrlFork>::SharedPtr goal_handle,const std::shared_ptr<const CtrlFork::Feedback> feedback){
    
    // 避免编译器警告
    (void)goal_handle;
    int current_fork_height = feedback->fork_height;

    RCLCPP_INFO(node_->get_logger(),"当前货叉高度为: %d mm",current_fork_height);
    
    // 设置驱动标志
    flag_driving = true;
    
    // 检查是否超过60秒
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - goal_send_time_).count();
    
    if(elapsed_time > 60) {
        // 检查goal_handle是否存在
        if(goal_handle) {
            RCLCPP_WARN(node_->get_logger(), "货叉任务执行时间超过60秒，正在取消任务...");
            auto future_cancel = fork_client_->async_cancel_goal(goal_handle);
        } else {
            RCLCPP_WARN(node_->get_logger(), "货叉任务执行时间超过60秒，但goal_handle不存在，无法取消任务");
        }
    }
}

// 处理最终反馈值
void LsaerForkActionClient::fork_result_callback(const rclcpp_action::ClientGoalHandle<CtrlFork>::WrappedResult & result){
    // result.code
    // determine the final result status through the status code
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(),"任务完成状态 %d",result.result->finish);
            flag_finish = true;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(),"任务中断 !");
            flag_finish = false;
            flag_aborted = true;
            flag_canceled = false;
            flag_driving = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(),"任务取消 !");
            flag_finish = false;
            flag_aborted = false;
            flag_canceled = true;
            flag_driving = false;
            break;
        default:
            RCLCPP_INFO(node_->get_logger(),"未知异常 !");
            flag_finish = false;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            break;
    }

}


// ---------------------------------------------------------------------------------------------------




// 类的构造函数
QRForkActionClient::QRForkActionClient(std::shared_ptr<rclcpp::Node> node): 
    node_(node), 
    current_goal_handle_(nullptr)
{
    RCLCPP_INFO(node_->get_logger(),"create fork action client!");

    // 创建客户端
    std::string SerialNumber = agv_config.serial_number;
    fork_client_ = rclcpp_action::create_client<TrayControl>(node_, SerialNumber + "/tray_control");

    // 初始化
    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;
    feedback_counter_ = 0;

}


/*****************************************************************************************
* @brief:      发送货叉控制的请求
* @param:      1、货叉控制使能，2、货叉高度
* @return:     无
* @author:     刘鸿彬
* @date:       2024-11-12
* @note：      1、请求连接货叉控制服务端，2、连接成功，进行数据填充并发布，3、接受反馈高度，到达目标后释放
* @version:    V0.0
******************************************************************************************/
void QRForkActionClient::send_fork_goal(int to_fork_signal,int goal_fork_height, int goal_fork_rotation){

    RCLCPP_INFO(node_->get_logger(),"进行货叉驱动控制!");
    // 连接服务端
    if (!(fork_client_->wait_for_action_server(10s))){
        RCLCPP_ERROR(node_->get_logger(),"连接服务段超时!");
        return;
    }

    // 发送具体请求
    auto goal_msg = TrayControl::Goal();
    // 发送数据填充
    goal_msg.tray_goal_rotation = goal_fork_rotation;
    goal_msg.tray_goal_height = goal_fork_height;
    goal_msg.to_tray_signal = to_fork_signal;

    // 配置动作客户端的行为
    rclcpp_action::Client<TrayControl>::SendGoalOptions send_goal_options;
    // 设置动作服务器确认接收到目标后被调用的回调函数
    send_goal_options.goal_response_callback = std::bind(&QRForkActionClient::fork_goal_response_callback,this,_1);
    // 设置动作服务器发送反馈信息时被调用的回调函数
    send_goal_options.feedback_callback = std::bind(&QRForkActionClient::fork_feedback_callback,this,_1,_2);
    // 设置动作完成或被取消时被调用的回调函数
    send_goal_options.result_callback = std::bind(&QRForkActionClient::fork_result_callback,this,_1);
    
    // 重置标志
    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;
    feedback_counter_ = 0;  // 重置反馈计数器
    
    // 异步地向动作服务器发送一个目标
    current_goal_future_ = fork_client_->async_send_goal(goal_msg,send_goal_options);
    
    // 记录发送任务的时间
    goal_send_time_ = std::chrono::steady_clock::now();

}

void QRForkActionClient::cancel_action(){

    if (current_goal_future_.valid() && current_goal_future_.wait_for(0s) == std::future_status::ready) {
        auto goal_handle = current_goal_future_.get();
        RCLCPP_INFO(node_->get_logger(), "Cancelling goal...");
        auto future_cancel = fork_client_->async_cancel_goal(goal_handle);
    } 
    else {
        RCLCPP_INFO(node_->get_logger(), "No active goal to cancel.");
    }
}

// 处理服务端反馈值
void QRForkActionClient::fork_goal_response_callback(rclcpp_action::ClientGoalHandle<TrayControl>::SharedPtr goal_handle){
    
    if (!goal_handle){
        RCLCPP_INFO(node_->get_logger(),"服务端拒绝请求!");

    }else{
        RCLCPP_INFO(node_->get_logger(),"请求通过!");
    }
}

// 处理连续反馈值
void QRForkActionClient::fork_feedback_callback(rclcpp_action::ClientGoalHandle<TrayControl>::SharedPtr goal_handle,const std::shared_ptr<const TrayControl::Feedback> feedback){
    
    // 避免编译器警告
    (void)goal_handle;

    // 增加计数器
    feedback_counter_++;
    
    // 设置驱动标志
    flag_driving = true;
    
    // 每1次输出一次信息
    if(feedback_counter_ % 1 == 0) {
        double current_cargo_height = feedback->tray_height;
        double current_cargo_rotation = feedback->tray_rotation;
        RCLCPP_INFO(node_->get_logger(),"当前托盘高度为: %.2f mm,托盘角度为：%.2f mm (反馈次数: %d)",current_cargo_height,current_cargo_rotation, feedback_counter_);
    }
    
    // 检查是否超过60秒
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - goal_send_time_).count();
    
    if(elapsed_time > 60) {
        // 检查goal_handle是否存在
        if(goal_handle) {
            RCLCPP_WARN(node_->get_logger(), "货叉任务执行时间超过60秒，正在取消任务...");
            auto future_cancel = fork_client_->async_cancel_goal(goal_handle);
        } else {
            RCLCPP_WARN(node_->get_logger(), "货叉任务执行时间超过60秒，但goal_handle不存在，无法取消任务");
        }
    }
}

// 处理最终反馈值
void QRForkActionClient::fork_result_callback(const rclcpp_action::ClientGoalHandle<TrayControl>::WrappedResult & result){
    // result.code
    // determine the final result status through the status code
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(),"任务完成状态 %d %d", result.result->height_finish, result.result->rotation_finish);
            flag_finish = true;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(node_->get_logger(),"任务中断 !");
            flag_finish = false;
            flag_aborted = true;
            flag_canceled = false;
            flag_driving = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(),"任务取消 !");
            flag_finish = false;
            flag_aborted = false;
            flag_canceled = true;
            flag_driving = false;
            break;
        default:
            RCLCPP_INFO(node_->get_logger(),"未知异常 !");
            flag_finish = false;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            break;
    }

}

