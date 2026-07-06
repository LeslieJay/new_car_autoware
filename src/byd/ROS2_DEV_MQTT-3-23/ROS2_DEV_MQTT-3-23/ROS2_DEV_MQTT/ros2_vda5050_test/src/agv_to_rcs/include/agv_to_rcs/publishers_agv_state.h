/************************************** File Info ****************************************
* @file:       publishers_agv_state.h                                                                   
* @author:     刘鸿彬                                                              
* @date:       2024-11-10                                         
* @version:    V0.0                                                                              
* @brief:      解析接收到的数据，将其进行数据填充发布方
******************************************************************************************/
# ifndef PUBLISHERS_AGV_STATE_H
# define PUBLISHERS_AGV_STATE_H

# include "rclcpp/rclcpp.hpp"
# include "agv_config.h"
# include "subscriber_current_pose.h"
# include "subscriber_battery.h"
# include "subscriber_velocity.h"
# include "server_obstacle.h"
# include "subscriber_order.h"
# include "subscriber_candata.h"
# include <functional>

// VDA5050协议接口
# include "vda5050_interfaces/msg/agv_connection.hpp"
# include "vda5050_interfaces/msg/agv_factsheet.hpp"
# include "vda5050_interfaces/msg/agv_state.hpp"
# include "vda5050_interfaces/msg/agv_visualization.hpp"

// MQTT相关头文件
# include "mqtt_client.h"
# include "json_converter.h"

// 命名空间中的字面量（literals），在代码中可以直接使用时间字面量而不需要显式地指定命名空间。
using namespace std::chrono_literals;

// vda5050协议展开
using vda5050_interfaces::msg::AGVVisualization;
using vda5050_interfaces::msg::AGVState;
using vda5050_interfaces::msg::AGVFactsheet;
using vda5050_interfaces::msg::AGVConnection;

// 占位符
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class AGVDataPublish
{
public:
    
    AGVDataPublish(std::shared_ptr<rclcpp::Node> node, std::string &agv_current_state, std::shared_ptr<ListenerPose> current_pose_listener = nullptr, std::shared_ptr<OrderListener> order_listener = nullptr, std::shared_ptr<BatteryListener> battery_listener = nullptr, std::shared_ptr<VelocityListener> velocity_listener = nullptr, std::shared_ptr<ObstacleServer> obstacle_server = nullptr, std::shared_ptr<CanDataListener> can_data_listener = nullptr);
    
    /*****************************************************************************************
    * @brief:      初始化MQTT客户端
    * @param:      mqtt_config MQTT配置参数
    * @return:     是否初始化成功
    * @author:     Assistant
    * @date:       2024-11-20
    * @version:    V1.0
    ******************************************************************************************/
    bool initMQTT(const MQTTConfig& mqtt_config);

    /*****************************************************************************************
    * @brief:      创建定时器，接收解析后的数据，进行定时发布
    * @param:      1、当强位姿，2、order命令数据
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-10
    * @version:    V0.0
    ******************************************************************************************/
    void publish_agv_state(std::string state);

    /*****************************************************************************************
    * @brief:      创建请求连接的定时器，进行定时发布
    * @param:      无
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-17
    * @version:    V0.0
    ******************************************************************************************/
    void publish_connection_request(std::string state);

    /*****************************************************************************************
    * @brief:      
    * @param:      1、当强位姿，2、order命令数据
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-10
    * @version:    V0.0
    ******************************************************************************************/
    void connection_request();

    /*****************************************************************************************
    * @brief:      销毁定时器
    * @param:      无
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-09
    * @version:    V0.0
    ******************************************************************************************/
    void cancel_all_timer();

    /*****************************************************************************************
    * @brief:      销毁请求连接的定时器
    * @param:      无
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-17
    * @version:    V0.0
    ******************************************************************************************/
    void cancel_connection_timer();

    /*****************************************************************************************
    * @brief:      state话题定时器的回调函数，用于定时发布state数据
    * @param:      fault_code 错误码，当不为NONE时填充errors字段
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-10
    * @version:    V0.0
    * @note:       注意动作的判断，完成动作之后，需要更新last_node_id和last_node_sequence_id
    ******************************************************************************************/
    void state_timer_callback(const std::string& fault_code = "NONE");

    /*****************************************************************************************
    * @brief:      设置驱动控制对象（用于获取driving状态）
    * @param:      get_driving_fn 获取driving状态的函数对象
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    * @note:       使用函数对象可以兼容LaserDriverControl和QRDriverControl两种类型
    ******************************************************************************************/
    void set_driving_fn(std::function<bool()> get_driving_fn);

    // 定时器是否创建的标识
    bool state_timer_create;

    // 定时器是否创建的标识
    bool connection_timer_create;


private:

    void State_timer_callback();

    /*****************************************************************************************
    * @brief:      visualization话题定时器的回调函数，用于定时发布visualization数据
    * @param:      当前位姿
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-10
    * @version:    V0.0
    * @note:       后期需要补充线速度和角度
    ******************************************************************************************/
    void visualization_timer_callback();
    /*****************************************************************************************
    * @brief:      factsheet话题定时器的回调函数，用于定时发布factsheet数据
    * @param:      当前位姿
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-10
    * @version:    V0.0
    ******************************************************************************************/
    void factsheet_timer_callback();
    /*****************************************************************************************
    * @brief:      connection发布方的定时器，用于定时发布connection消息
    * @param:      请求状态，online为请求上线，offline为请求下线
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-10
    * @version:    V0.0
    ******************************************************************************************/
    void connection_timer_callback(const std::string state);
    
    /*****************************************************************************************
    * @brief:      小车状态发布定时器回调函数，用于定时发布VehicleStatus消息
    * @param:      无
    * @return:     无
    * @author:     Assistant
    * @date:       2024-11-20
    * @version:    V1.0
    ******************************************************************************************/
    void status_timer_callback();
    
    // 发布方声明
    rclcpp::Publisher<AGVConnection>::SharedPtr connection_publisher_;
    rclcpp::Publisher<AGVFactsheet>::SharedPtr factsheet_publisher_;
    rclcpp::Publisher<AGVVisualization>::SharedPtr visualization_publisher_;
    rclcpp::Publisher<AGVState>::SharedPtr state_publisher_;
    rclcpp::Publisher<VehicleStatus>::SharedPtr status_publisher_; 

    // 声明定时器
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    rclcpp::TimerBase::SharedPtr factsheet_timer_;
    rclcpp::TimerBase::SharedPtr connection_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;  // 小车状态发布定时器

    // 自增
    int header_id_connection;
    int header_id_factsheet;
    int header_id_visualization;
    int header_id_state;
    int header_id_status;  // 小车状态消息ID计数器

    std::shared_ptr<rclcpp::Node> node_;

    // 声明数据结构体引用
    std::string &agv_current_state_;  // AGV当前状态引用

    // 订阅者和服务器指针（可选，用于实时更新数据）
    // 作为数据发布类，需要创建订阅类，来实时更新消息
    std::shared_ptr<ListenerPose> current_pose_listener_;
    std::shared_ptr<OrderListener> order_listener_;
    std::shared_ptr<BatteryListener> battery_listener_;
    std::shared_ptr<VelocityListener> velocity_listener_;
    std::shared_ptr<ObstacleServer> obstacle_server_;
    std::shared_ptr<CanDataListener> can_data_listener_;

    // 数据成员变量（私有，在定时器回调中更新）
    // 回调获得的消息使用如下变量保存，can信息不用上报rcs，这里没有
    CurrentPose current_pose_;
    OrderMessages order_messages_;
    BatteryMessages battery_messages_;
    VelocityMessages velocity_messages_;
    int obstacle_messages_;

    // 定时器的触发计数
    int count_time;
    
    // MQTT相关成员变量
    std::unique_ptr<MQTTClient> mqtt_client_;
    bool mqtt_enabled_;

    // 驱动控制相关的函数对象（用于获取driving状态）
    // 使用函数对象可以兼容LaserDriverControl和QRDriverControl两种类型
    std::function<bool()> get_driving_fn_;
    
    // 上一次的driving状态（用于检测状态变化）
    bool last_driving_state_;

    // 上一次的操作模式（用于检测状态变化）
    bool last_operation_mode_;

};

# endif