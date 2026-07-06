/************************************** File Info ****************************************
* @file:       mqtt_message_handler.cpp                                                                   
* @author:     Assistant                                                          
* @date:       2024-11-20                                         
* @version:    V2.0                                                                              
* @brief:      MQTT消息处理器实现 - 直接填充ROS2消息类型
******************************************************************************************/
#include "mqtt_message_handler.h"
#include <iostream>
#include "vda5050_interfaces/msg/agv_order.hpp"
#include "vda5050_interfaces/msg/agv_instant_actions.hpp"
#include "vda5050_interfaces/msg/nodes.hpp"
#include "vda5050_interfaces/msg/edges.hpp"
#include "vda5050_interfaces/msg/actions.hpp"
#include "vda5050_interfaces/msg/node_position.hpp"
#include "vda5050_interfaces/msg/trajectory.hpp"
#include "vda5050_interfaces/msg/control_points.hpp"
#include "vda5050_interfaces/msg/action_parameters.hpp"

MQTTMessageHandler::MQTTMessageHandler(
    std::shared_ptr<rclcpp::Node> node,
    std::function<void(const std::string&)> order_callback,
    std::function<void(const std::string&)> instant_action_callback
) : node_(node), logger_(node->get_logger()), 
    order_callback_(order_callback), instant_action_callback_(instant_action_callback) {
    
    RCLCPP_INFO(logger_, "MQTT消息处理器已初始化");
}

void MQTTMessageHandler::handleMessage(const std::string& topic, const std::string& payload) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    RCLCPP_DEBUG(logger_, "处理MQTT消息 - 主题: %s, 负载大小: %zu 字节", topic.c_str(), payload.size());
    
    // 根据主题类型分发消息
    if (topic == agv_config.mqtt_order_topic) {
        // 处理order消息
        RCLCPP_INFO(logger_, "收到MQTT order消息");
        if (order_callback_) {
            order_callback_(payload);
        }
    } else if (topic == agv_config.mqtt_instant_action_topic) {
        // 处理instant action消息
        RCLCPP_INFO(logger_, "收到MQTT instant action消息");
        if (instant_action_callback_) {
            instant_action_callback_(payload);
        }
    } else {
        RCLCPP_WARN(logger_, "收到未知主题的MQTT消息: %s", topic.c_str());
    }
}

void MQTTMessageHandler::setOrderCallback(std::function<void(const std::string&)> callback) {
    order_callback_ = callback;
}

void MQTTMessageHandler::setInstantActionCallback(std::function<void(const std::string&)> callback) {
    instant_action_callback_ = callback;
}

bool MQTTMessageHandler::parseOrderMessage(const std::string& json_payload, vda5050_interfaces::msg::AGVOrder& mqtt_order_messages) {
    try {
        RCLCPP_INFO(logger_, "json_payload: %s", json_payload.c_str());
        
        // 解析JSON
        json order_json = json::parse(json_payload);
        
        RCLCPP_DEBUG(logger_, "开始解析order消息JSON");
        
        // 基本验证
        if (!order_json.contains("orderId") || !order_json.contains("nodes") || !order_json.contains("edges")) {
            RCLCPP_ERROR(logger_, "order消息缺少必要字段");
            return false;
        }
        
        // 清空现有数据
        mqtt_order_messages.nodes.clear();
        mqtt_order_messages.edges.clear();
        
        // 解析header信息
        mqtt_order_messages.header_id = order_json.contains("headerId") ? order_json["headerId"].get<int>() : 0;
        mqtt_order_messages.timestamp = order_json.contains("timestamp") ? order_json["timestamp"].get<std::string>() : "";
        mqtt_order_messages.version = order_json.contains("version") ? order_json["version"].get<std::string>() : "";
        mqtt_order_messages.manufacturer = order_json.contains("manufacturer") ? order_json["manufacturer"].get<std::string>() : "";
        mqtt_order_messages.serial_number = order_json.contains("serialNumber") ? order_json["serialNumber"].get<std::string>() : "";
        mqtt_order_messages.order_id = order_json["orderId"].get<std::string>();
        mqtt_order_messages.order_update_id = order_json.contains("orderUpdateId") ? order_json["orderUpdateId"].get<int>() : 0;
        mqtt_order_messages.zone_set_id = order_json.contains("zoneSetId") ? order_json["zoneSetId"].get<std::string>() : "";
        
        // 解析nodes
        if (order_json.contains("nodes") && order_json["nodes"].is_array()) {
            for (const auto& node_json : order_json["nodes"]) {
                vda5050_interfaces::msg::Nodes node_msg;
                
                // 必选字段
                if (!node_json.contains("nodeId") || !node_json.contains("sequenceId") || !node_json.contains("released")) {
                    RCLCPP_WARN(logger_, "node缺少必选字段，跳过该node");
                    continue;
                }
                
                node_msg.node_id = node_json["nodeId"].get<std::string>();
                node_msg.sequence_id = node_json["sequenceId"].get<int>();
                node_msg.released = node_json["released"].get<bool>();
                
                // 解析nodePosition
                if (node_json.contains("nodePosition")) {
                    auto pos = node_json["nodePosition"];
                    node_msg.node_position.x = pos.contains("x") ? pos["x"].get<double>() : 0.0;
                    node_msg.node_position.y = pos.contains("y") ? pos["y"].get<double>() : 0.0;
                    node_msg.node_position.theta = pos.contains("theta") ? pos["theta"].get<double>() : 0.0;
                    node_msg.node_position.map_id = pos.contains("mapId") ? pos["mapId"].get<std::string>() : "";
                    node_msg.node_position.map_description = pos.contains("mapDescription") ? pos["mapDescription"].get<std::string>() : "";
                    node_msg.node_position.allowed_deviation_xy = pos.contains("allowedDeviationXY") ? pos["allowedDeviationXY"].get<double>() : 0.3;
                    node_msg.node_position.allowed_deviation_theta = pos.contains("allowedDeviationTheta") ? pos["allowedDeviationTheta"].get<double>() : 0.0;
                }
                
                // 解析node中的actions
                if (node_json.contains("actions") && node_json["actions"].is_array()) {
                    for (const auto& action_json : node_json["actions"]) {
                        vda5050_interfaces::msg::Actions action_msg;
                        action_msg.action_type = action_json.contains("actionType") ? action_json["actionType"].get<std::string>() : "";
                        action_msg.action_id = action_json.contains("actionId") ? action_json["actionId"].get<std::string>() : "";
                        action_msg.action_description = action_json.contains("actionDescription") ? action_json["actionDescription"].get<std::string>() : "";
                        action_msg.blocking_type = action_json.contains("blockingType") ? action_json["blockingType"].get<std::string>() : "";
                        
                        // 解析actionParameters（如果有）
                        if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array()) {
                            for (const auto& param_json : action_json["actionParameters"]) {
                                vda5050_interfaces::msg::ActionParameters param_msg;
                                param_msg.key = param_json.contains("key") ? param_json["key"].get<std::string>() : "";
                                
                                // 解析value字段
                                if (param_json.contains("value") && param_json["value"].is_object()) {
                                    const auto& value_json = param_json["value"];
                                    if (value_json.contains("numberValue")) {
                                        param_msg.value.number_value = value_json["numberValue"].get<double>();
                                    }
                                    if (value_json.contains("stringValue")) {
                                        param_msg.value.string_value = value_json["stringValue"].get<std::string>();
                                    }
                                    if (value_json.contains("booleanValue")) {
                                        param_msg.value.boolean_value = value_json["booleanValue"].get<bool>();
                                    }
                                    if (value_json.contains("arrayValue") && value_json["arrayValue"].is_array()) {
                                        for (const auto& arr_item : value_json["arrayValue"]) {
                                            if (arr_item.is_string()) {
                                                param_msg.value.array_value.push_back(arr_item.get<std::string>());
                                            }
                                        }
                                    }
                                }
                                
                                action_msg.action_parameters.push_back(param_msg);
                            }
                        }
                        
                        node_msg.actions.push_back(action_msg);
                    }
                }
                
                mqtt_order_messages.nodes.push_back(node_msg);
            }
        }
        
        // 解析edges
        if (order_json.contains("edges") && order_json["edges"].is_array()) {
            for (const auto& edge_json : order_json["edges"]) {
                vda5050_interfaces::msg::Edges edge_msg;
                
                // 必选字段验证
                if (!edge_json.contains("edgeId") || !edge_json.contains("sequenceId") || 
                    !edge_json.contains("released") || !edge_json.contains("startNodeId") || 
                    !edge_json.contains("endNodeId") || !edge_json.contains("obstacleAvoidanceChannel")) {
                    RCLCPP_WARN(logger_, "edge缺少必选字段，跳过该edge");
                    continue;
                }
                
                edge_msg.edge_id = edge_json["edgeId"].get<std::string>();
                edge_msg.sequence_id = edge_json["sequenceId"].get<int>();
                edge_msg.released = edge_json["released"].get<bool>();
                edge_msg.start_node_id = edge_json["startNodeId"].get<std::string>();
                edge_msg.end_node_id = edge_json["endNodeId"].get<std::string>();
                edge_msg.obstacle_avoidance_channel = edge_json["obstacleAvoidanceChannel"].get<int>();
                
                // 可选字段（使用VDA5050默认值）
                edge_msg.edge_description = edge_json.contains("edgeDescription") ? edge_json["edgeDescription"].get<std::string>() : "";
                edge_msg.max_speed = edge_json.contains("maxSpeed") ? edge_json["maxSpeed"].get<double>() : 3.0;
                edge_msg.max_height = edge_json.contains("maxHeight") ? edge_json["maxHeight"].get<double>() : 10.0;
                edge_msg.orientation = edge_json.contains("orientation") ? edge_json["orientation"].get<double>() : 0.0;
                edge_msg.orientation_type = edge_json.contains("orientationType") ? edge_json["orientationType"].get<std::string>() : "TANGENTIALTANGENTIAL";
                edge_msg.direction = edge_json.contains("direction") ? edge_json["direction"].get<std::string>() : "";
                edge_msg.rotation_allowed = edge_json.contains("rotationAllowed") ? edge_json["rotationAllowed"].get<bool>() : true;
                edge_msg.max_rotation_speed = edge_json.contains("maxRotationSpeed") ? edge_json["maxRotationSpeed"].get<double>() : 3.14;
                edge_msg.length = edge_json.contains("length") ? edge_json["length"].get<double>() : 99999999.0;
                
                // 解析trajectory
                if (edge_json.contains("trajectory")) {
                    auto traj_json = edge_json["trajectory"];
                    edge_msg.trajectory.degree = traj_json.contains("degree") ? traj_json["degree"].get<int>() : 1;
                    
                    // knotVector
                    if (traj_json.contains("knotVector") && traj_json["knotVector"].is_array()) {
                        for (const auto& knot : traj_json["knotVector"]) {
                            if (knot.is_number()) {
                                edge_msg.trajectory.knot_vector.push_back(knot.get<double>());
                            }
                        }
                    }
                    
                    // controlPoints
                    if (traj_json.contains("controlPoints") && traj_json["controlPoints"].is_array()) {
                        for (const auto& cp : traj_json["controlPoints"]) {
                            if (cp.contains("x") && cp.contains("y")) {
                                vda5050_interfaces::msg::ControlPoints control_point;
                                control_point.x = cp["x"].get<double>();
                                control_point.y = cp["y"].get<double>();
                                control_point.weight = cp.contains("weight") ? cp["weight"].get<double>() : 1.0;
                                edge_msg.trajectory.control_points.push_back(control_point);
                            }
                        }
                    }
                } else {
                    // 没有trajectory字段，使用默认空trajectory
                    edge_msg.trajectory.degree = -1; // 表示没有轨迹
                }
                
                // 解析edge中的actions
                if (edge_json.contains("actions") && edge_json["actions"].is_array()) {
                    for (const auto& action_json : edge_json["actions"]) {
                        vda5050_interfaces::msg::Actions action_msg;
                        action_msg.action_type = action_json.contains("actionType") ? action_json["actionType"].get<std::string>() : "";
                        action_msg.action_id = action_json.contains("actionId") ? action_json["actionId"].get<std::string>() : "";
                        action_msg.action_description = action_json.contains("actionDescription") ? action_json["actionDescription"].get<std::string>() : "";
                        action_msg.blocking_type = action_json.contains("blockingType") ? action_json["blockingType"].get<std::string>() : "";
                        
                        // 解析actionParameters（如果有）
                        if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array()) {
                            for (const auto& param_json : action_json["actionParameters"]) {
                                vda5050_interfaces::msg::ActionParameters param_msg;
                                param_msg.key = param_json.contains("key") ? param_json["key"].get<std::string>() : "";
                                
                                // 解析value字段
                                if (param_json.contains("value") && param_json["value"].is_object()) {
                                    const auto& value_json = param_json["value"];
                                    if (value_json.contains("numberValue")) {
                                        param_msg.value.number_value = value_json["numberValue"].get<double>();
                                    }
                                    if (value_json.contains("stringValue")) {
                                        param_msg.value.string_value = value_json["stringValue"].get<std::string>();
                                    }
                                    if (value_json.contains("booleanValue")) {
                                        param_msg.value.boolean_value = value_json["booleanValue"].get<bool>();
                                    }
                                    if (value_json.contains("arrayValue") && value_json["arrayValue"].is_array()) {
                                        for (const auto& arr_item : value_json["arrayValue"]) {
                                            if (arr_item.is_string()) {
                                                param_msg.value.array_value.push_back(arr_item.get<std::string>());
                                            }
                                        }
                                    }
                                }
                                
                                action_msg.action_parameters.push_back(param_msg);
                            }
                        }
                        
                        edge_msg.actions.push_back(action_msg);
                    }
                }
                
                mqtt_order_messages.edges.push_back(edge_msg);
            }
        }
        
        RCLCPP_INFO(logger_, "成功解析order消息: %zu个节点, %zu个边", 
                   mqtt_order_messages.nodes.size(), mqtt_order_messages.edges.size());
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "解析order消息失败: %s", e.what());
        return false;
    }
}

bool MQTTMessageHandler::parseInstantActionMessage(const std::string& json_payload, vda5050_interfaces::msg::AGVInstantActions& mqtt_instant_action_messages) {
    try {
        // 解析JSON
        json instant_json = json::parse(json_payload);
        
        RCLCPP_DEBUG(logger_, "开始解析instant action消息JSON (VDA5050格式)");
        
        // VDA5050标准格式验证
        if (!instant_json.contains("actions") || !instant_json["actions"].is_array()) {
            RCLCPP_ERROR(logger_, "instant action消息缺少actions数组字段");
            return false;
        }
        
        if (instant_json["actions"].empty()) {
            RCLCPP_ERROR(logger_, "instant action消息的actions数组为空");
            return false;
        }
        
        // 清空现有数据
        mqtt_instant_action_messages.actions.clear();
        
        // 解析header信息
        mqtt_instant_action_messages.header_id = instant_json.contains("headerId") ? instant_json["headerId"].get<int>() : 0;
        mqtt_instant_action_messages.timestamp = instant_json.contains("timestamp") ? instant_json["timestamp"].get<std::string>() : "";
        mqtt_instant_action_messages.version = instant_json.contains("version") ? instant_json["version"].get<std::string>() : "";
        mqtt_instant_action_messages.manufacturer = instant_json.contains("manufacturer") ? instant_json["manufacturer"].get<std::string>() : "";
        mqtt_instant_action_messages.serial_number = instant_json.contains("serialNumber") ? instant_json["serialNumber"].get<std::string>() : "";
        
        // 解析所有actions
        for (const auto& action_json : instant_json["actions"]) {
            vda5050_interfaces::msg::Actions action_msg;
            
            // 解析必选字段
            if (!action_json.contains("actionType") || !action_json.contains("actionId")) {
                RCLCPP_WARN(logger_, "action缺少必选字段，跳过该action");
                continue;
            }
            
            action_msg.action_type = action_json["actionType"].get<std::string>();
            action_msg.action_id = action_json["actionId"].get<std::string>();
            action_msg.action_description = action_json.contains("actionDescription") ? action_json["actionDescription"].get<std::string>() : "";
            action_msg.blocking_type = action_json.contains("blockingType") ? action_json["blockingType"].get<std::string>() : "";
            
            // 解析actionParameters
            if (action_json.contains("actionParameters") && action_json["actionParameters"].is_array()) {
                for (const auto& param_json : action_json["actionParameters"]) {
                    vda5050_interfaces::msg::ActionParameters param_msg;
                    param_msg.key = param_json.contains("key") ? param_json["key"].get<std::string>() : "";
                    
                    // 解析value字段
                    if (param_json.contains("value") && param_json["value"].is_object()) {
                        const auto& value_json = param_json["value"];
                        if (value_json.contains("numberValue")) {
                            param_msg.value.number_value = value_json["numberValue"].get<double>();
                        }
                        if (value_json.contains("stringValue")) {
                            param_msg.value.string_value = value_json["stringValue"].get<std::string>();
                        }
                        if (value_json.contains("booleanValue")) {
                            param_msg.value.boolean_value = value_json["booleanValue"].get<bool>();
                        }
                        if (value_json.contains("arrayValue") && value_json["arrayValue"].is_array()) {
                            for (const auto& arr_item : value_json["arrayValue"]) {
                                if (arr_item.is_string()) {
                                    param_msg.value.array_value.push_back(arr_item.get<std::string>());
                                }
                            }
                        }
                    }
                    
                    action_msg.action_parameters.push_back(param_msg);
                }
            }
            
            mqtt_instant_action_messages.actions.push_back(action_msg);
        }
        
        RCLCPP_INFO(logger_, "✓ 解析VDA5050 instant action: %zu个动作", mqtt_instant_action_messages.actions.size());
        if (!mqtt_instant_action_messages.actions.empty()) {
            RCLCPP_INFO(logger_, "  首个动作: 类型=%s, ID=%s", 
                       mqtt_instant_action_messages.actions[0].action_type.c_str(),
                       mqtt_instant_action_messages.actions[0].action_id.c_str());
            
            if (mqtt_instant_action_messages.actions[0].action_type == "agv_online") {
                RCLCPP_INFO(logger_, "AGV上线成功！");
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "✗ 解析instant action消息失败: %s", e.what());
        RCLCPP_ERROR(logger_, "  JSON内容: %s", json_payload.c_str());
        return false;
    }
}
