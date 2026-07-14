/************************************** File Info ****************************************
* @file:       client_send_multi_poses.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-02                                         
* @version:    V0.0                                                                              
* @brief:      发送多个目标点到AGV，重置终点，单点发送会默认一个终点导致行走卡顿
******************************************************************************************/

# include "client_send_multi_poses.h"

std::string arr_to_str(const std::array<uint8_t, UUID_SIZE>& arr){
    std::string res;
    for(auto e : arr){
        res += std::to_string(e);
    }
    return res;
}

// 类的构造函数，创建动作客户端
LaserSendMultiPose::LaserSendMultiPose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ListenerPose> current_pose_listener): 
    node_(node),
    current_pose_listener_(current_pose_listener){

    trajectory_client_ = node_->create_client<UseTrajectory>("use_trajectory");
    action_client_ = rclcpp_action::create_client<AutowareAuto>(node_, "autoware_auto");

    // 初始化
    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;
    counter = 0;

}


// 发送数据
rclcpp::Client<UseTrajectory>::FutureAndRequestId LaserSendMultiPose::send_request(std::vector<Point> goal_points)
{
    // 数据声明
    auto request = std::make_shared<UseTrajectory::Request>();

    // // 创建一条路径
    // nav_msgs::msg::Path path_msg;
    // path_msg.header.stamp = node_->now();
    // path_msg.header.frame_id = "map";

    int num_poses = goal_points.size();
    int i = num_poses - 1;
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = "map";
    pose.pose.position.x = goal_points[i].x;
    pose.pose.position.y = goal_points[i].y;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_points[i].theta);
    pose.pose.orientation = tf2::toMsg(q);  

    request->goal_pose = pose;

    // 数据发送
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"数据成功发送！");
    return trajectory_client_->async_send_request(request);
    
}
void LaserSendMultiPose::send_goal(std::vector<Point> goal_points, bool forward)
{
    
    if(!action_client_->wait_for_action_server(10s)){

        RCLCPP_INFO(node_->get_logger(),"connecting out of time");
        return;
    }


    int num_poses = goal_points.size();
    int i = num_poses - 1;
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = "map";
    pose.pose.position.x = goal_points[i].x;
    pose.pose.position.y = goal_points[i].y;
    pose.pose.position.z = 0.0;
    // RCLCPP_INFO(node_->get_logger(), 
    // "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABefore conversion: theta = %f rad (%f deg)", 
    // goal_points[i].theta, goal_points[i].theta * M_PI / 180.0);
    // goal_points[i].theta = goal_points[i].theta * M_PI / 180.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_points[i].theta);
    pose.pose.orientation = tf2::toMsg(q);  
    // --- 输出转换后的四元数 ---
    RCLCPP_INFO(node_->get_logger(), 
    "After conversion: quaternion x=%.6f, y=%.6f, z=%.6f, w=%.6f",
    pose.pose.orientation.x, pose.pose.orientation.y, 
    pose.pose.orientation.z, pose.pose.orientation.w);
    auto goal_msg = AutowareAuto::Goal();
    goal_msg.goal_pose = pose;
    goal_msg.forward = forward;
    //goal_msg.behavior_tree = "/home/lf/ros_ws/nav_ws/src/agv_description/bt_xml/navigate_through_poses_w_replanning_and_recovery.xml";

    RCLCPP_INFO(node_->get_logger(), "Done! Sending goal...");
    auto send_goal_options = rclcpp_action::Client<AutowareAuto>::SendGoalOptions();
    
    send_goal_options.goal_response_callback = std::bind(&LaserSendMultiPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&LaserSendMultiPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&LaserSendMultiPose::result_callback, this, std::placeholders::_1);

    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;

    action_goal_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    counter = 0;
    
}


void LaserSendMultiPose::cancel_action(){

    if (action_goal_future_.valid() && action_goal_future_.wait_for(0s) == std::future_status::ready) {
        auto goal_handle = action_goal_future_.get();
        RCLCPP_INFO(node_->get_logger(), "Cancelling goal...");
        auto future_cancel = action_client_->async_cancel_goal(goal_handle);
    } 
    else {
        RCLCPP_INFO(node_->get_logger(), "No active goal to cancel.");
    }
}


void LaserSendMultiPose::goal_response_callback(GoalHandleACTION::SharedPtr goal_handle){

    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server.");
    } 
    else {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Goal accepted by server, waiting for result... ID:" << arr_to_str(goal_handle->get_goal_id()));
        
    }
}

void LaserSendMultiPose::feedback_callback(GoalHandleACTION::SharedPtr goal_handle,const std::shared_ptr<const AutowareAuto::Feedback> feedback){

    // 避免编译器警告
    (void)goal_handle;

    counter = counter +1;

    // 获取AGV的位姿信息
    m_current_x = feedback->current_pose.pose.position.x;
    m_current_y = feedback->current_pose.pose.position.y;

    quat_w = feedback->current_pose.pose.orientation.w;
    quat_z = feedback->current_pose.pose.orientation.z;

    // 四元数转弧度
    m_current_theta = 2 * acos(quat_w);
    // double imaginaryPart = -quat_z*quat_z;

    // 正反转判别
    if ( quat_z<0) {
        // RCLCPP_INFO(node_->get_logger(), "触发逆时针");
        m_current_theta = -m_current_theta;
    }

    current_pose_.current_x = m_current_x;
    current_pose_.current_y = m_current_y;
    current_pose_.current_theta = m_current_theta;
    // 更新 ListenerPose 中的位姿
    if (current_pose_listener_) {
        current_pose_listener_->set_current_pose(current_pose_);
    }

    if(counter%20 == 0)
    {
        RCLCPP_INFO(node_->get_logger(), "Feedback: cur X: %f    cur Y: %f    cur Theta: %f", m_current_x, m_current_y, m_current_theta);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Remaining Poses: " << feedback->number_of_poses_remaining << std::endl);
    }

    flag_driving = true;

}

void LaserSendMultiPose::result_callback(const GoalHandleACTION::WrappedResult &result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            flag_finish = true;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Goal succeeded! ID:" << arr_to_str(result.goal_id));
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN_STREAM(node_->get_logger(), "Goal was aborted! ID:" << arr_to_str(result.goal_id));
            flag_finish = false;
            flag_aborted = true;
            flag_canceled = false;
            flag_driving = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled by the user.");
            flag_finish = false;
            flag_aborted = false;
            flag_canceled = true;
            flag_driving = false;
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code!");
            flag_finish = true;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            break;
    }
}


// ---------------------------------------------------------------------------------------------------------------------


// 类的构造函数，创建动作客户端
QRSendMultiPose::QRSendMultiPose(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ListenerPose> current_pose_listener): 
    node_(node),
    current_pose_listener_(current_pose_listener){

    std::string SerialNumber = agv_config.serial_number;
    action_client_ = rclcpp_action::create_client<AGVSend>(node_, SerialNumber + "/agv_action");

    // 初始化
    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;
    counter = 0;

}

// 发目标点给nav2，导航到点位，才能上线
void QRSendMultiPose::send_goal(std::vector<agv_interfaces::msg::Poses> goal_poses)
{
    // 清除 path_poses 容器
    path_poses.clear();
    
    if(!action_client_->wait_for_action_server(10s)){

        RCLCPP_INFO(node_->get_logger(),"connecting out of time");
        return;
    }

    int num_poses = goal_poses.size();

    RCLCPP_INFO(node_->get_logger(),"the total number of poses: %d", num_poses);

    // std::cout<<"the total number of poses: " << num_poses<< std::endl;

    for (int i = 0; i < num_poses; i++)
    {
        auto pose = agv_interfaces::msg::Poses();

        pose.x = goal_poses[i].x;
        pose.y = goal_poses[i].y;
        pose.label = goal_poses[i].label;
        pose.angle = goal_poses[i].angle;
        pose.allowed_deviation_angle = goal_poses[i].allowed_deviation_angle; // 0~3.14
        pose.obstacle_channel_select = goal_poses[i].obstacle_channel_select;
        //将 pose 添加到 path_poses 容器的末尾
        path_poses.emplace_back(pose);

        RCLCPP_INFO(node_->get_logger(), "目标点[%d]: 坐标(%.2f, %.2f), 标签=%ld, 角度=%.2f, 允许角度偏差=%.2f, 障碍物通道选择=%d", 
                    i, pose.x, pose.y, pose.label, pose.angle, pose.allowed_deviation_angle, pose.obstacle_channel_select);

    }

    auto goal_msg = AGVSend::Goal();
    goal_msg.pose = path_poses;
    RCLCPP_INFO(node_->get_logger(), "Done! Sending goal...");

    auto send_goal_options = rclcpp_action::Client<AGVSend>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&QRSendMultiPose::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&QRSendMultiPose::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&QRSendMultiPose::result_callback, this, std::placeholders::_1);

    flag_finish = false;
    flag_aborted = false;
    flag_canceled = false;
    flag_driving = false;

    action_goal_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);

    counter = 0;
    
}


void QRSendMultiPose::cancel_action(){

    if (action_goal_future_.valid() && action_goal_future_.wait_for(0s) == std::future_status::ready) {
        auto goal_handle = action_goal_future_.get();
        RCLCPP_INFO(node_->get_logger(), "Cancelling goal...");
        auto future_cancel = action_client_->async_cancel_goal(goal_handle);
    } 
    else {
        RCLCPP_INFO(node_->get_logger(), "No active goal to cancel.");
    }
}


void QRSendMultiPose::goal_response_callback(GoalHandleAGVSend::SharedPtr goal_handle){

    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server.");
    } 
    else {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Goal accepted by server, waiting for result... ID:" << arr_to_str(goal_handle->get_goal_id()));
        
    }
}

void QRSendMultiPose::feedback_callback(GoalHandleAGVSend::SharedPtr goal_handle,const std::shared_ptr<const AGVSend::Feedback> feedback){

    // 避免编译器警告
    (void)goal_handle;
    
    counter = counter + 1;
    
    flag_driving = true;

    // 获取AGV的位姿信息
    m_label = feedback->current_label;
    m_angle = feedback->current_angle;
    
    if(counter % 20 == 0)
    {
    RCLCPP_INFO(node_->get_logger(), "Received feedback: label : %ld \t angle : %f", feedback->current_label, feedback->current_angle);
    }
    
}

void QRSendMultiPose::result_callback(const GoalHandleAGVSend::WrappedResult &result){
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            flag_finish = true;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            RCLCPP_INFO_STREAM(node_->get_logger(), "Goal succeeded! ID:" << arr_to_str(result.goal_id));
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN_STREAM(node_->get_logger(), "Goal was aborted! ID:" << arr_to_str(result.goal_id));
            flag_finish = false;
            flag_aborted = true;
            flag_canceled = false;
            flag_driving = false;
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "Goal was canceled by the user.");
            flag_finish = false;
            flag_aborted = false;
            flag_canceled = true;
            flag_driving = false;
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code!");
            flag_finish = false;
            flag_aborted = false;
            flag_canceled = false;
            flag_driving = false;
            break;
    }
}
