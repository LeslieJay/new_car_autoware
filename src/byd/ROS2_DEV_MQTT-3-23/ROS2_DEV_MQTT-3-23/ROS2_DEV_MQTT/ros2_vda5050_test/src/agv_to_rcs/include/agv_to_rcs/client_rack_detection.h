/************************************** File Info ****************************************
* @file:       client_rack_detection.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-12-19                                         
* @version:    V0.0                                                                              
* @brief:      货位检测控制客户端
******************************************************************************************/
#ifndef CLIENT_RACK_DETECTION_H
#define CLIENT_RACK_DETECTION_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "agv_config.h"
#include "ref_slam_interface/srv/rack_detection.hpp"  // 包名和服务名

using ref_slam_interface::srv::RackDetection;

class RackDetectionClient : public rclcpp::Node
{
public:
    RackDetectionClient();
    bool connect_server();
    rclcpp::Client<RackDetection>::FutureAndRequestId send_request(int detecting);

private:
    rclcpp::Client<RackDetection>::SharedPtr client_;
};

#endif // CLIENT_RACK_DETECTION_H