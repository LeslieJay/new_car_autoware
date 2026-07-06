/************************************** File Info ****************************************
* @file:       client_end_guide.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-12-19                                         
* @version:    V0.0                                                                              
* @brief:      末端引导控制客户端
******************************************************************************************/
#ifndef CLIENT_END_GUIDE_H
#define CLIENT_END_GUIDE_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "agv_config.h"
#include "ref_slam_interface/srv/end_guide.hpp"  // 包名和服务名

using ref_slam_interface::srv::EndGuide;

class EndGuideClient
{
public:

    EndGuideClient(std::shared_ptr<rclcpp::Node> node);
    bool connect_server();
    rclcpp::Client<EndGuide>::FutureAndRequestId send_request(bool is_target, bool is_terminus);
    rclcpp::Client<EndGuide>::SharedPtr client_;

private:
    std::shared_ptr<rclcpp::Node> node_;
};

#endif // CLIENT_END_GUIDE_H