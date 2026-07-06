/************************************** File Info ****************************************
* @file:       agv_config.h                                                                     
* @author:     刘鸿彬                                                             
* @date:       2025-01-07                                      
* @version:    V0.0                                                                              
* @brief:      参数的规整和统一布局
******************************************************************************************/
// agv_config.h
# ifndef AGV_CONFIG_H
# define AGV_CONFIG_H

# include <map>
# include <queue>
# include <memory>
# include <tool_json.h>
# include <iostream>
# include <string>
# include <vector>
# include <cmath>

# include "agv_interfaces/msg/point.hpp"

# include "vda5050_interfaces/msg/agv_state.hpp"
# include "vda5050_interfaces/msg/action_states.hpp"
# include "vda5050_interfaces/msg/agv_instant_actions.hpp"
# include "vda5050_interfaces/msg/agv_order.hpp"

# include "agv_interfaces/srv/battery_control.hpp"
# include "agv_interfaces/msg/battery_state.hpp"
# include "agv_interfaces/msg/vehicle_status.hpp"

using BatteryControl = agv_interfaces::srv::BatteryControl;
using BatteryState = agv_interfaces::msg::BatteryState;
using VehicleStatus = agv_interfaces::msg::VehicleStatus;

using PoseType = agv_interfaces::msg::Point;
#define AGV_CONFIG_QR_VERSION  // 标识这是二维码版本

using json = nlohmann::json;
using vda5050_interfaces::msg::AGVState;
using vda5050_interfaces::msg::ActionStates;
using vda5050_interfaces::msg::AGVInstantActions;
using vda5050_interfaces::msg::AGVOrder;

// 定义一个二维点的结构体
struct Point {
    double x, y,theta;
};

// 点类型
enum class PointType {       
    ORDINARY,       // 普通点
    SHARP,          // 尖点
    READYUNLOAD,    // 准备卸货点
    READYLOAD,      // 准备装货点
    HARDACTION,     // 硬动作点（至少有一个HARD类型的动作）
    SOFTACTION      // 软动作点（所有动作都是SOFT类型）
};

struct ControlPoint {
    double x;
    double y;
    double weight;
};

struct Trajectory {
    int degree; // B样条或贝塞尔曲线的次数
    std::vector<double> knots; // 节点向量（仅用于NURBS）
    std::vector<ControlPoint> control_points; // 控制点
};

// 接收order命令后初步解析出来的所需数据结构体
struct OrderMessages{
    // 接收rcs消息解析出来的目标点位姿（站点:一个id对应一个三维坐标）和目标边正走还是倒走信息
    std::vector<std::string> goal_node_id;
    std::vector<double> goal_x,goal_y,goal_theta;
    std::vector<double> goal_allowed_deviation_xy;     // 允许的偏差XY
    std::vector<double> goal_allowed_deviation_theta;  // 允许的偏差Theta

    std::vector<double> goal_edge_orientation;
    std::vector<int> edge_obstacle_avoidance_channels; // 边的避障通道
    // 接收rcs消息解析出来的目标轨迹（曲线上的取样-点集）
    std::vector<Trajectory> goal_trajectory;
    // 每个点的类型及目标货叉高度
    std::vector<PointType> point_types;
    std::vector<int> goal_heights;
    
    // Edges可选字段（VDA5050协议）
    std::vector<std::string> edge_descriptions;       // 边的描述信息（默认""）
    std::vector<double> edge_max_speeds;              // 边的最大速度（默认3.0）
    std::vector<double> edge_max_heights;             // 边的最大高度（默认10.0）
    std::vector<std::string> edge_orientation_types;  // 边的方向类型（默认"TANGENTIALTANGENTIAL"）
    std::vector<std::string> edge_directions;         // 边的行驶方向（默认""）
    std::vector<bool> edge_rotation_allowed;          // 是否允许旋转（默认true）
    std::vector<double> edge_max_rotation_speeds;     // 最大旋转速度（默认3.14）
    std::vector<double> edge_lengths;                 // 边的长度（默认99999999）
    
    // 接收rcs消息解析解析出节点、边、动作数量，state数据填充时需要该值声明容器大小
    int node_size, edge_size, action_size;
    // 当前时刻的目标位姿
    double current_goal_x,current_goal_y,current_goal_theta;
    // 关联容器，内部存储node_id和action_id之间的对用关系
    std::multimap<std::string,std::string> action_vec;
    // 数据填充
    vda5050_interfaces::msg::AGVState msg_state;  
};

// 接收instantAction命令后初步解析出来的所需数据结构体
struct InstantActionMessages{
    // 接收rcs消息解析出来的目标点位姿
    double goal_x,goal_y,goal_theta;
    // 接收rcs消息解析出的动作类型
    std::string action_type;
    // 接收rcs消息解析出的order_id
    std::string last_instant_action_order_id;
};

// 打断任务的相关消息
struct InterruptOrderMessage{

    std::string action_id;

    std::string action_type;

    std::string action_description;
    
    std::string action_status; // 表示该取消任务指令是否已完成

};

// 当前位姿数据结构体
struct CurrentPose{
    // 订阅到的AGV当前位姿
    double current_x,current_y,current_theta;
};

// agv速度数据结构体
struct VelocityMessages{
    // x方向上的线速度
    double velocity_x;
    // y方向上的线速度
    double velocity_y;
    // 角速度（弧度制）
    double omega;
    // 加速度
    double acceleration;
};

// 电池数据结构体
struct BatteryMessages{
    // 电池电量
    double battery_level;
    // 电池状态（充电\放电）
    double battery_status;
    // 电池电压
    double total_voltage;
    // 电池电流
    double total_current;
    // 货叉高度
    // int    fork_height;
};

struct CANFrame{
    uint32_t id;
    uint32_t time_stamp;
    uint8_t time_flag;
    uint8_t send_type;
    uint8_t remote_flag;  // 是否是远程帧
    uint8_t extern_flag;  // 是否是扩展帧
    uint8_t data_len;
    uint8_t data[8];
    uint8_t reserved[3];
};

// can数据结构体
struct CANMessages{
    int32_t port; // 端口
    int32_t cnt; // 帧数
    std::vector<CANFrame> can_frames;
};

// 单个货位信息
struct Racks {
    double x,y,z;
    bool state;
};

// 货位检测数据结构体
struct RackDetectionMessages{
    // int header_id;
    // bool if_finished;
    // bool current_state;
    std::vector<Racks> racks; 
};


// 以下是从配置文件中加载配置信息保存至全局变量——agv_config当中。
// 配置结构体
struct TypeSpecification {
    std::string series_name;
    std::string agv_kinematic;
    std::string agv_class;
    int max_load_mass;
    std::string navigation_types;
};

struct PhysicalParameters {
    double speed_min;
    double speed_max;
    double acceleration_max;
    double deceleration_max;
    double height_max;
    double width;
    double length;
};

struct AGVPosition {
    std::string map_id;
    bool position_initialized;
};

struct AGVConfig {
    std::string vehicle_type;  // "laser" or "qr"
    double max_linear_velocity;
    double max_angular_velocity;
    int min_battery_level;
    double max_offset;
    double high_distance_precision;
    double high_angle_precision;
    double low_distance_precision;
    double low_angle_precision;
    double PI;
    int can_channels;
    int can0_frames;
    int can1_frames;
    std::vector<int> gap;
    std::string static_file_path;
    std::string instant_file_path;
    std::string candata_file_path;
    std::string file_path_prefix;
    int record_batch_size;
    std::string version;
    std::string manufacturer;
    std::string serial_number;
    TypeSpecification type_specification;
    PhysicalParameters physical_parameters;
    AGVPosition agv_position;
    std::string operating_mode;
    int fork_running_height;
    int fork_action_height;
    int record_duration;
    std::vector<std::vector<double>> sites;
    int sites_num;
    double micro_distance;
    int max_pause_time;
    
    // 新增：通信方式配置
    std::string communication_mode;  // "ros2_only", "mqtt_only", "dual_channel"
    bool enable_mqtt_subscription;   // 是否启用MQTT订阅
    std::string mqtt_broker_host;    // MQTT代理地址
    int mqtt_broker_port;            // MQTT代理端口
    std::string mqtt_username;       // MQTT用户名
    std::string mqtt_password;       // MQTT密码
    std::string mqtt_order_topic;    // MQTT order主题
    std::string mqtt_instant_action_topic;  // MQTT instant action主题
};

// 声明全局配置变量
extern AGVConfig agv_config;

// 声明加载函数
bool load_agv_config(const std::string& path);


# endif