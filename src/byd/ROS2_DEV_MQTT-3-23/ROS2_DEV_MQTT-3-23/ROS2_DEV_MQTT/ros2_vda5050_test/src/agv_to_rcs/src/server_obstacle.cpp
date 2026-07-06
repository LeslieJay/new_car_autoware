/************************************** File Info ****************************************
 * @file:       server_obstacle.cpp
 * @author:     刘鸿彬
 * @date:       2025-03-07
 * @version:    V0.0
 * @class:      订阅者
 * @brief:      障碍物相关数据的服务端，但其实只是被导航模块通知一下障碍物消息
 * @note:       类内更新数据，通过get方法获取，避免读写冲突
 ******************************************************************************************/

#include "server_obstacle.h"

/*****************************************************************************************
 * @brief:      障碍物数据服务端的ros节点类的构造函数，用于初始化服务端等
 * @param:      node：ROS2节点指针
 * @param:      current_pose_listener：位姿监听者指针，用于获取当前位姿
 * @return:     无
 * @author:     刘鸿彬
 * @date:       2025-03-07
 * @version:    V0.0
 * @modified:   改为类内存储数据，通过位姿监听者获取当前位姿
 ******************************************************************************************/
ObstacleServer::ObstacleServer(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ListenerPose> current_pose_listener) : 
	node_(node),
	current_pose_listener_(current_pose_listener)
{
	RCLCPP_INFO(node_->get_logger(), "created an obstacle server!");
	
	// 初始化为默认值
	obstacle_messages_ = 0;
	get_obstacle = false;

	// 创建障碍物检测服务端
	obstacle_server_ = node_->create_service<ObstacleDetection>("obstacle_detection", std::bind(&ObstacleServer::obstacle_callback, this, _1, _2));
	
	// 创建障碍物通道发布者
	obstacle_channels_pub_ = node_->create_publisher<ObstacleChannels>("obstacle_channels", 1);
	RCLCPP_INFO(node_->get_logger(), "created obstacle channels publisher!");
}

/*****************************************************************************************
 * @brief:      返回障碍物数据
 * @param:      无
 * @return:     障碍物消息值（int类型）
 * @author:     刘鸿彬
 * @date:       2025-03-07
 * @version:    V0.0
 * @modified:   改为类内存储数据，通过get方法获取
 ******************************************************************************************/
int ObstacleServer::get_obstacle_messages(){
	// 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
	std::lock_guard<std::mutex> lock(data_mutex_);
	
	return obstacle_messages_;
}

/*****************************************************************************************
 * @brief:      服务端回调函数，用于反馈是否接收到来导航模块的障碍物信息
 * @param:      请求与回复
 * @return:     无
 * @author:     刘鸿彬
 * @date:       2025-03-07
 * @version:    V0.0
 * @modified:   使用互斥锁保护数据写入，通过位姿监听者获取当前位姿
 ******************************************************************************************/
void ObstacleServer::obstacle_callback(const ObstacleDetection::Request::SharedPtr request, ObstacleDetection::Response::SharedPtr response_message)
{
	// 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
	std::lock_guard<std::mutex> lock(data_mutex_);
	
	RCLCPP_INFO_STREAM(node_->get_logger(), "Obstacle detection message received : " << request->obstacle_status);

	// 更新是否获取障碍物数据的状态
	get_obstacle = true;
	
	obstacle_messages_ = request->obstacle_status;

	int ignore_flag = 0;

	// // 通过位姿监听者获取当前位姿
	// CurrentPose current_pose = current_pose_listener_->get_current_pose();
	// double theta = current_pose.current_theta;
	// double x     = current_pose.current_x;
	// double y     = current_pose.current_y;

    // // 计算小车四个角的位置
    // pointA.x = (local_x1 * cos(theta) - local_y1 * sin(theta)) + x;
    // pointA.y = (local_x1 * sin(theta) + local_y1 * cos(theta)) + y;

    // pointB.x = (local_x2 * cos(theta) - local_y2 * sin(theta)) + x;
    // pointB.y = (local_x2 * sin(theta) + local_y2 * cos(theta)) + y;

    // pointC.x = (local_x3 * cos(theta) - local_y3 * sin(theta)) + x;
    // pointC.y = (local_x3 * sin(theta) + local_y3 * cos(theta)) + y;

    // pointD.x = (local_x4 * cos(theta) - local_y4 * sin(theta)) + x;
    // pointD.y = (local_x4 * sin(theta) + local_y4 * cos(theta)) + y;

	// auto sites = agv_config.sites;

	// double distance;

	// for(const auto& site : sites)
	// {
	// 	x = site[0];
	// 	y = site[1];
	// 	distance = std::min( {sqrt(pow(x-pointA.x,2)+pow(y-pointA.y,2)), sqrt(pow(x-pointB.x,2)+pow(y-pointB.y,2)), sqrt(pow(x-pointC.x,2)+pow(y-pointC.y,2)), sqrt(pow(x-pointD.x,2)+pow(y-pointD.y,2))} );
	// 	RCLCPP_INFO_STREAM(node_->get_logger(), "当前小车与当前站点的距离 : " << distance);
	// 	if(distance < 1) // 当小车进入与站点距离1m之内的区域，则视为小车即将进行动作，此时屏蔽障碍物检测信号
	// 	{
	// 		ignore_flag = 1;
	// 		break;
	// 	}
	// }

	// 0:靠近站点，需要屏蔽障碍物检测
	// 1:小车行进过程中，需要障碍物检测
	response_message->ignore = ignore_flag;
	RCLCPP_INFO_STREAM(node_->get_logger(), "向客户端发送的标签 : " << ignore_flag);

}

/*****************************************************************************************
 * @brief:      发布障碍物通道信息
 * @param:      back_channel: 后激光通道
 * @param:      left_channel: 左激光通道
 * @param:      head_channel: 前激光通道
 * @param:      right_channel: 右激光通道
 * @return:     无
 * @author:     Assistant
 * @date:       2025-01-XX
 * @version:    V1.0
 * @note:       激光区域通道顺序: 后、左、前、右
 ******************************************************************************************/
void ObstacleServer::publish_obstacle_channels(int16_t back_channel, int16_t left_channel, int16_t head_channel, int16_t right_channel)
{
	ObstacleChannels msg;
	msg.back_laser_channel = back_channel;
	msg.left_laser_channel = left_channel;
	msg.head_laser_channel = head_channel;
	msg.right_laser_channel = right_channel;
	
	obstacle_channels_pub_->publish(msg);
	RCLCPP_INFO(node_->get_logger(), "Published obstacle channels: back=%d, left=%d, head=%d, right=%d", 
		back_channel, left_channel, head_channel, right_channel);
}
