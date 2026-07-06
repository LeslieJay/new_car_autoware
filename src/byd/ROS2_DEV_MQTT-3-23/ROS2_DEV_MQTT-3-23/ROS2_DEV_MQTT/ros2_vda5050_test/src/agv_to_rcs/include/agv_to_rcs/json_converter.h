/************************************** File Info ****************************************
* @file:       json_converter.h                                                                   
* @author:     Assistant                                                          
* @date:       2024-11-20                                         
* @version:    V1.0                                                                              
* @brief:      ROS消息到JSON转换工具
******************************************************************************************/
#ifndef JSON_CONVERTER_H
#define JSON_CONVERTER_H

#include <string>
#include <sstream>
#include <iomanip>

// VDA5050协议接口
#include "vda5050_interfaces/msg/agv_connection.hpp"
#include "vda5050_interfaces/msg/agv_factsheet.hpp"
#include "vda5050_interfaces/msg/agv_state.hpp"
#include "vda5050_interfaces/msg/agv_visualization.hpp"

using vda5050_interfaces::msg::AGVConnection;
using vda5050_interfaces::msg::AGVState;
using vda5050_interfaces::msg::AGVFactsheet;
using vda5050_interfaces::msg::AGVVisualization;

class JsonConverter {
public:
    /**
     * @brief 将AGVConnection消息转换为JSON字符串
     * @param msg AGVConnection消息
     * @return JSON字符串
     */
    static std::string toJson(const AGVConnection& msg);
    
    /**
     * @brief 将AGVState消息转换为JSON字符串
     * @param msg AGVState消息
     * @return JSON字符串
     */
    static std::string toJson(const AGVState& msg);
    
    /**
     * @brief 将AGVFactsheet消息转换为JSON字符串
     * @param msg AGVFactsheet消息
     * @return JSON字符串
     */
    static std::string toJson(const AGVFactsheet& msg);
    
    /**
     * @brief 将AGVVisualization消息转换为JSON字符串
     * @param msg AGVVisualization消息
     * @return JSON字符串
     */
    static std::string toJson(const AGVVisualization& msg);

private:
    /**
     * @brief 转义JSON字符串中的特殊字符
     * @param input 输入字符串
     * @return 转义后的字符串
     */
    static std::string escapeJson(const std::string& input);
    
    /**
     * @brief 格式化浮点数为字符串
     * @param value 浮点数值
     * @param precision 精度
     * @return 格式化后的字符串
     */
    static std::string formatDouble(double value, int precision = 6);
};

#endif // JSON_CONVERTER_H