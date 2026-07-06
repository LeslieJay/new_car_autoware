/************************************** File Info ****************************************
* @file:       client_end_guide.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-09-27                                         
* @version:    V0.0                                                                              
* @brief:      AGV末端引导客户端，发送引导请求，服务端处理后，返回引导状态
******************************************************************************************/

#include "client_end_guide.h"

/*****************************************************************************************
* @brief:      服务通信客户端，用于发送控制指令到服务端，启动末端引导
* @author:     刘鸿彬
* @date:       2024-09-27
* @version:    V0.0
******************************************************************************************/

// 构造函数，创建客户端
EndGuideClient::EndGuideClient(std::shared_ptr<rclcpp::Node> node) : 
    node_(node)
{
    
    RCLCPP_INFO(node_->get_logger(), "create a end guide client!");
    // 创建客户端
    client_ = node_->create_client<EndGuide>("end_guide");
}

// 连接服务端，如果连接成功返回true否则返回false
bool EndGuideClient::connect_server() {

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
* @param:      is_target：表示前进
* @param:      is_terminus：表示返回
* @author:     刘鸿彬
* @date:       2024-09-27
* @version:    V0.0
******************************************************************************************/
// 发送数据
rclcpp::Client<EndGuide>::FutureAndRequestId EndGuideClient::send_request(bool is_target, bool is_terminus) {
    
    // 数据声明
    auto request = std::make_shared<EndGuide::Request>();
    // 数据填充
    request->is_target = is_target;
    request->is_terminus = is_terminus;
    // 数据发送
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "末端引导数据成功发送！");
    return client_->async_send_request(request);
}