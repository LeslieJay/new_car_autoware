/************************************** File Info ****************************************
* @file:       tool_localization.h                                                                   
* @author:     刘鸿彬                                                              
* @date:       2024-12-19 
* @version:    V0.0                                                                              
* @brief:      在导航定位不在的时候，该模块模拟发送位姿信息给ros平台
******************************************************************************************/
#ifndef TOOL_LOCALIZATION_H

#define TOOL_LOCALIZATION_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <kdl/frames.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <queue>
#include <random>


// TEST
// #include "vda5050_interfaces/msg/poses.hpp"

#include <iostream>
#include <fstream>

using geometry_msgs::msg::PoseWithCovarianceStamped;

class Localization_Tool  : public rclcpp::Node
{
public:
    explicit Localization_Tool(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~Localization_Tool();
private:


    // test
    void publish_localization();
    PoseWithCovarianceStamped localization_msg;
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr localization_pub;
    rclcpp::TimerBase::SharedPtr timer;


};

#endif // TOOL_LOCALIZATION_H
