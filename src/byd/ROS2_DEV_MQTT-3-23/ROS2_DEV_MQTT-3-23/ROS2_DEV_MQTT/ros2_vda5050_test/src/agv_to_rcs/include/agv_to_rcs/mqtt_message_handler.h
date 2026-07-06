/************************************** File Info ****************************************
* @file:       mqtt_message_handler.h                                                                   
* @author:     Assistant                                                          
* @date:       2024-11-20                                         
* @version:    V1.0                                                                              
* @brief:      MQTT消息处理器，用于处理来自MQTT的order和instant action消息
******************************************************************************************/
#ifndef MQTT_MESSAGE_HANDLER_H
#define MQTT_MESSAGE_HANDLER_H

#include <string>
#include <functional>
#include <memory>
#include <mutex>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include "agv_config.h"
#include "tool_json.h"
#include "vda5050_interfaces/msg/agv_order.hpp"
#include "vda5050_interfaces/msg/agv_instant_actions.hpp"

class MQTTMessageHandler {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点
     * @param order_callback order消息回调函数
     * @param instant_action_callback instant action消息回调函数
     */
    MQTTMessageHandler(
        std::shared_ptr<rclcpp::Node> node,
        std::function<void(const std::string&)> order_callback = nullptr,
        std::function<void(const std::string&)> instant_action_callback = nullptr
    );

    /**
     * @brief 处理接收到的MQTT消息
     * @param topic 主题名称
     * @param payload 消息内容
     */
    void handleMessage(const std::string& topic, const std::string& payload);

    /**
     * @brief 设置order消息回调函数
     * @param callback 回调函数
     */
    void setOrderCallback(std::function<void(const std::string&)> callback);

    /**
     * @brief 设置instant action消息回调函数
     * @param callback 回调函数
     */
    void setInstantActionCallback(std::function<void(const std::string&)> callback);

    /**
     * @brief 解析order消息并填充ROS2 AGVOrder消息
     * @param json_payload JSON格式的order消息
     * @param mqtt_order_messages 输出的ROS2 AGVOrder消息
     * @return 解析是否成功
     */
    bool parseOrderMessage(const std::string& json_payload, vda5050_interfaces::msg::AGVOrder& mqtt_order_messages);

    /**
     * @brief 解析instant action消息并填充ROS2 AGVInstantActions消息
     * @param json_payload JSON格式的instant action消息
     * @param mqtt_instant_action_messages 输出的ROS2 AGVInstantActions消息
     * @return 解析是否成功
     */
    bool parseInstantActionMessage(const std::string& json_payload, vda5050_interfaces::msg::AGVInstantActions& mqtt_instant_action_messages);

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger logger_;
    std::function<void(const std::string&)> order_callback_;
    std::function<void(const std::string&)> instant_action_callback_;
    std::mutex data_mutex_;
};

#endif // MQTT_MESSAGE_HANDLER_H