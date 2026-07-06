/************************************** File Info ****************************************
* @file:       client_request_charging.h                                                                    
* @author:     刘鸿彬                                                              
* @date:       2024-11-18                                        
* @version:    V0.0                                                                              
* @brief:      AGV电池充电客户端，发送充电请求，服务端处理后，返回电池状态(充电/放电)
******************************************************************************************/
# ifndef CLIENT_REQUEST_CHARGING_H
# define CLIENT_REQUEST_CHARGING_H

#include <iostream>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "agv_config.h"


using namespace std::chrono_literals;

class RequestChargingClient
{

public:

    // 构造函数，创建客户端
    RequestChargingClient(std::shared_ptr<rclcpp::Node> node);

    // 连接服务端，如果连接成功返回true否则返回false
    bool connect_server();

    // 发送数据
    rclcpp::Client<BatteryControl>::FutureAndRequestId send_request(int charging);

    // 客户端声明 
    rclcpp::Client<BatteryControl>::SharedPtr client_;
    
private:

    std::shared_ptr<rclcpp::Node> node_;
};

# endif