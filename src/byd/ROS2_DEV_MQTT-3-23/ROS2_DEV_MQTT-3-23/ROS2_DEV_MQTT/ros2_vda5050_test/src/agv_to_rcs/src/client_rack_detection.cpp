/************************************** File Info ****************************************
* @file:       client_rack_detection.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-09-27                                         
* @version:    V0.0                                                                              
* @brief:      AGV货位检测客户端，发送检测请求，服务端处理后，返回检测状态
******************************************************************************************/

#include "client_rack_detection.h"

/*****************************************************************************************
* @brief:      服务通信客户端，用于发送控制指令到服务端，启动货位检测
* @author:     刘鸿彬
* @date:       2024-09-27
* @version:    V0.0
******************************************************************************************/

// 构造函数，创建客户端
RackDetectionClient::RackDetectionClient() : Node("rack_detection_client_node_cpp") {
    
    RCLCPP_INFO(this->get_logger(), "create a rack detection client!");
    // 创建客户端
    client_ = this->create_client<RackDetection>("rack_detection");
}

// 连接服务端，如果连接成功返回true否则返回false
bool RackDetectionClient::connect_server() {

    // 等待5s
    while (!client_->wait_for_service(std::chrono::seconds(5))) {

        // ctrl+c的特殊处理
        if (!rclcpp::ok()) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "force client termination!");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "connecting with server!");
    }
    return true;
}

/*****************************************************************************************
* @brief:      客户端发送数据到服务端
* @param:      detecting ：发送的数据（0：停止检测，1：开始检测）
* @author:     刘鸿彬
* @date:       2024-09-27
* @version:    V0.0
******************************************************************************************/
// 发送数据
rclcpp::Client<RackDetection>::FutureAndRequestId RackDetectionClient::send_request(int detecting) {
    
    // 数据声明
    auto request = std::make_shared<RackDetection::Request>();
    // 数据填充
    request->detecting = detecting;
    // 数据发送
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "货位检测数据成功发送！");
    return client_->async_send_request(request);
}