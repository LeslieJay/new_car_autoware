/************************************** File Info ****************************************
* @file:       json_converter.cpp                                                                   
* @author:     Assistant                                                          
* @date:       2024-11-20                                         
* @version:    V1.0                                                                              
* @brief:      ROS消息到JSON转换工具实现
******************************************************************************************/
#include "json_converter.h"

std::string JsonConverter::toJson(const AGVConnection& msg) {
    std::ostringstream json;
    json << "{"
         << "\"headerId\":" << msg.header_id << ","
         << "\"timestamp\":\"" << escapeJson(msg.timestamp) << "\","
         << "\"version\":\"" << escapeJson(msg.version) << "\","
         << "\"manufacturer\":\"" << escapeJson(msg.manufacturer) << "\","
         << "\"serialNumber\":\"" << escapeJson(msg.serial_number) << "\","
         << "\"connectionState\":\"" << escapeJson(msg.connection_state) << "\""
         << "}";
    return json.str();
}

std::string JsonConverter::toJson(const AGVState& msg) {
    std::ostringstream json;
    json << "{"
         << "\"headerId\":" << msg.header_id << ","
         << "\"timestamp\":\"" << escapeJson(msg.timestamp) << "\","
         << "\"version\":\"" << escapeJson(msg.version) << "\","
         << "\"manufacturer\":\"" << escapeJson(msg.manufacturer) << "\","
         << "\"serialNumber\":\"" << escapeJson(msg.serial_number) << "\","
         << "\"orderId\":\"" << escapeJson(msg.order_id) << "\","
         << "\"orderUpdateId\":" << msg.order_update_id << ","
         << "\"lastNodeId\":\"" << escapeJson(msg.last_node_id) << "\","
         << "\"lastNodeSequenceId\":" << msg.last_node_sequence_id << ","
         << "\"driving\":" << (msg.driving ? "true" : "false") << ","
         << "\"paused\":" << (msg.paused ? "true" : "false") << ","
         << "\"operatingMode\":\"" << escapeJson(msg.operating_mode) << "\","
         
         // AGV位置信息
         << "\"agvPosition\":{"
         << "\"x\":" << formatDouble(msg.agv_position.x) << ","
         << "\"y\":" << formatDouble(msg.agv_position.y) << ","
         << "\"theta\":" << formatDouble(msg.agv_position.theta) << ","
         << "\"mapId\":\"" << escapeJson(msg.agv_position.map_id) << "\","
         << "\"positionInitialized\":" << (msg.agv_position.position_initialized ? "true" : "false")
         << "},"
         
         // 电池状态
         << "\"batteryState\":{"
         << "\"batteryCharge\":" << formatDouble(msg.battery_state.battery_charge) << ","
         << "\"charging\":" << (msg.battery_state.charging ? "true" : "false") << ","
         << "\"voltage\":" << formatDouble(msg.battery_state.voltage) << ","
         << "\"current\":" << formatDouble(msg.battery_state.current)
         << "},"
         
         // 节点状态数组
         << "\"nodeStates\":[";
    for (size_t i = 0; i < msg.node_states.size(); ++i) {
        if (i > 0) json << ",";
        json << "{"
             << "\"nodeId\":\"" << escapeJson(msg.node_states[i].node_id) << "\","
             << "\"sequenceId\":" << msg.node_states[i].sequence_id << ","
             << "\"released\":" << (msg.node_states[i].released ? "true" : "false")
             << "}";
    }
    json << "],"
         
         // 边状态数组
         << "\"edgeStates\":[";
    for (size_t i = 0; i < msg.edge_states.size(); ++i) {
        if (i > 0) json << ",";
        json << "{"
             << "\"edgeId\":\"" << escapeJson(msg.edge_states[i].edge_id) << "\","
             << "\"sequenceId\":" << msg.edge_states[i].sequence_id << ","
             << "\"released\":" << (msg.edge_states[i].released ? "true" : "false")
             << "}";
    }
    json << "],"
         
         // 动作状态数组
         << "\"actionStates\":[";
    for (size_t i = 0; i < msg.action_states.size(); ++i) {
        if (i > 0) json << ",";
        json << "{"
             << "\"actionId\":\"" << escapeJson(msg.action_states[i].action_id) << "\","
             << "\"actionType\":\"" << escapeJson(msg.action_states[i].action_type) << "\","
             << "\"actionDescription\":\"" << escapeJson(msg.action_states[i].action_description) << "\","
             << "\"actionStatus\":\"" << escapeJson(msg.action_states[i].action_status) << "\""
             << "}";
    }
    json << "],"
         
         // 错误数组
         << "\"errors\":[";
    for (size_t i = 0; i < msg.errors.size(); ++i) {
        if (i > 0) json << ",";
        json << "{"
             << "\"errorType\":\"" << escapeJson(msg.errors[i].error_type) << "\","
             << "\"errorDescription\":\"" << escapeJson(msg.errors[i].error_description) << "\","
             << "\"errorHint\":\"" << escapeJson(msg.errors[i].error_hint) << "\","
             << "\"errorLevel\":\"" << escapeJson(msg.errors[i].error_level) << "\","
             << "\"errorReferences\":[";
        for (size_t j = 0; j < msg.errors[i].error_references.size(); ++j) {
            if (j > 0) json << ",";
            json << "{"
                 << "\"referenceKey\":\"" << escapeJson(msg.errors[i].error_references[j].reference_key) << "\","
                 << "\"referenceValue\":\"" << escapeJson(msg.errors[i].error_references[j].reference_value) << "\""
                 << "}";
        }
        json << "]"
             << "}";
    }
    json << "],"
         
         // 安全状态
         << "\"safetyState\":{"
         << "\"eStop\":\"" << escapeJson(msg.safety_state.e_stop) << "\","
         << "\"fieldViolation\":" << (msg.safety_state.field_violation ? "true" : "false")
         << "}"
         << "}";
    
    return json.str();
}

std::string JsonConverter::toJson(const AGVFactsheet& msg) {
    std::ostringstream json;
    json << "{"
         << "\"headerId\":" << msg.header_id << ","
         << "\"timestamp\":\"" << escapeJson(msg.timestamp) << "\","
         << "\"version\":\"" << escapeJson(msg.version) << "\","
         << "\"manufacturer\":\"" << escapeJson(msg.manufacturer) << "\","
         << "\"serialNumber\":\"" << escapeJson(msg.serial_number) << "\","
         
         // 类型规格
         << "\"typeSpecification\":{"
         << "\"seriesName\":\"" << escapeJson(msg.type_specification.series_name) << "\","
         << "\"agvKinematic\":\"" << escapeJson(msg.type_specification.agv_kinematic) << "\","
         << "\"agvClass\":\"" << escapeJson(msg.type_specification.agv_class) << "\","
         << "\"maxLoadMass\":" << formatDouble(msg.type_specification.max_load_mass) << ","
         << "\"localizationTypes\":\"" << escapeJson(msg.type_specification.localization_types) << "\","
         << "\"navigationTypes\":\"" << escapeJson(msg.type_specification.navigation_types) << "\""
         << "},"
         
         // 物理参数
         << "\"physicalParameters\":{"
         << "\"speedMin\":" << formatDouble(msg.physical_parameters.speed_min) << ","
         << "\"speedMax\":" << formatDouble(msg.physical_parameters.speed_max) << ","
         << "\"accelerationMax\":" << formatDouble(msg.physical_parameters.acceleration_max) << ","
         << "\"decelerationMax\":" << formatDouble(msg.physical_parameters.deceleration_max) << ","
         << "\"heightMax\":" << formatDouble(msg.physical_parameters.height_max) << ","
         << "\"width\":" << formatDouble(msg.physical_parameters.width) << ","
         << "\"length\":" << formatDouble(msg.physical_parameters.length)
         << "}"
         << "}";
    
    return json.str();
}

std::string JsonConverter::toJson(const AGVVisualization& msg) {
    std::ostringstream json;
    json << "{"
         << "\"headerId\":" << msg.header_id << ","
         << "\"timestamp\":\"" << escapeJson(msg.timestamp) << "\","
         << "\"version\":\"" << escapeJson(msg.version) << "\","
         << "\"manufacturer\":\"" << escapeJson(msg.manufacturer) << "\","
         << "\"serialNumber\":\"" << escapeJson(msg.serial_number) << "\","
         
         // AGV位置信息
         << "\"agvPosition\":{"
         << "\"x\":" << formatDouble(msg.agv_position.x) << ","
         << "\"y\":" << formatDouble(msg.agv_position.y) << ","
         << "\"theta\":" << formatDouble(msg.agv_position.theta) << ","
         << "\"mapId\":\"" << escapeJson(msg.agv_position.map_id) << "\","
         << "\"positionInitialized\":" << (msg.agv_position.position_initialized ? "true" : "false") << ","
         << "\"localizationScore\":" << formatDouble(msg.agv_position.localization_score)
         << "},"
         
         // 速度信息
         << "\"velocity\":{"
         << "\"vx\":" << formatDouble(msg.velocity.vx) << ","
         << "\"vy\":" << formatDouble(msg.velocity.vy) << ","
         << "\"omega\":" << formatDouble(msg.velocity.omega)
         << "}"
         << "}";
    
    return json.str();
}

std::string JsonConverter::escapeJson(const std::string& input) {
    std::ostringstream escaped;
    for (char c : input) {
        switch (c) {
            case '"':  escaped << "\\\""; break;
            case '\\': escaped << "\\\\"; break;
            case '\b': escaped << "\\b"; break;
            case '\f': escaped << "\\f"; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default:
                if (c >= 0 && c < 0x20) {
                    escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
                } else {
                    escaped << c;
                }
                break;
        }
    }
    return escaped.str();
}

std::string JsonConverter::formatDouble(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}