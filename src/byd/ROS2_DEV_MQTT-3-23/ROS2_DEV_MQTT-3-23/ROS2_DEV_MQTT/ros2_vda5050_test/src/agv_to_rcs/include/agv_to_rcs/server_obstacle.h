/************************************** File Info ****************************************
 * @file:       server_obstacle.h
 * @author:     刘鸿彬
 * @date:       2025-03-05
 * @version:    V0.0
 * @brief:      障碍物消息数据订阅方 注意：这里ros平台的角色定位是服务端，只有当导航模块发现障碍物时，
 * @brief:      它立即采取行动并告知ros平台，然后ros平台再转交给调度系统，此时导航模块具有最大优先级
 * @note:       类内更新数据，通过get方法获取，避免读写冲突
 ******************************************************************************************/
#ifndef SERVER_OBSTACLE
#define SERVER_OBSTACLE

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "ref_slam_interface/srv/obstacle_detection.hpp"
#include "ref_slam_interface/msg/obstacle_channels.hpp"
#include "agv_config.h"
#include "subscriber_current_pose.h"
#include <mutex>

using ref_slam_interface::srv::ObstacleDetection;
using ref_slam_interface::msg::ObstacleChannels;
using namespace std::chrono_literals;
using namespace std::placeholders;

// 3.define node class
class ObstacleServer
{

public:
    ObstacleServer(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<ListenerPose> current_pose_listener);

    // 返回障碍物数据
    int get_obstacle_messages();

    // 障碍物消息接收标识
    bool get_obstacle;

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
     ******************************************************************************************/
    void publish_obstacle_channels(int16_t back_channel, int16_t left_channel, int16_t head_channel, int16_t right_channel);

private:
    void obstacle_callback(const ObstacleDetection::Request::SharedPtr request, ObstacleDetection::Response::SharedPtr response_message);

    rclcpp::Service<ref_slam_interface::srv::ObstacleDetection>::SharedPtr obstacle_server_;

    // 障碍物通道发布者
    rclcpp::Publisher<ObstacleChannels>::SharedPtr obstacle_channels_pub_;

    std::shared_ptr<rclcpp::Node> node_;

    // 位姿监听者（用于获取当前位姿）
    std::shared_ptr<ListenerPose> current_pose_listener_;

    // 障碍物检测消息（内部存储），一个int类型值即可，0、1、2、3分别表示四种情况
    int obstacle_messages_;

    const double local_x1 = agv_config.physical_parameters.width,                    local_y1 = double(agv_config.physical_parameters.width)/2;       // 角点 A
    const double local_x2 = agv_config.physical_parameters.width,                    local_y2 = double(agv_config.physical_parameters.width)/(-2);    // 角点 B
    const double local_x3 = double(agv_config.physical_parameters.length) * (-1),    local_y3 = double(agv_config.physical_parameters.width)/2;       // 角点 C
    const double local_x4 = double(agv_config.physical_parameters.length) * (-1),    local_y4 = double(agv_config.physical_parameters.width)/(-2);    // 角点 D

    Point pointA,pointB,pointC,pointD;

    // 用于保护所有数据的互斥锁
    std::mutex data_mutex_;

};

#endif