/************************************** File Info ****************************************
* @file:       subscriber_order.h                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-03                                         
* @version:    V0.0                                                                              
* @brief:      AGV设备接受RCS任务话题的订阅端，订阅话题，解析消息
******************************************************************************************/
# ifndef SUBSCRIBER_ORDER_H
# define SUBSCRIBER_ORDER_H

# include "rclcpp/rclcpp.hpp"
# include "vda5050_interfaces/msg/agv_order.hpp"
# include "vda5050_interfaces/msg/agv_state.hpp"
# include "vda5050_interfaces/msg/node_states.hpp"
# include "vda5050_interfaces/msg/edge_states.hpp"
# include "vda5050_interfaces/msg/action_states.hpp"
# include "agv_config.h"
# include <queue>

using vda5050_interfaces::msg::AGVOrder;
using vda5050_interfaces::msg::AGVState;
using vda5050_interfaces::msg::NodeStates;
using vda5050_interfaces::msg::EdgeStates;
using vda5050_interfaces::msg::ActionStates;

class AGVDataPublish;

// 类的构造函数
// 继承节点
class OrderListener
{
public:
    // 节点构造函数
    OrderListener(std::shared_ptr<rclcpp::Node> node);

    // 返回解析order得到的所需数据
    OrderMessages get_order_messages();

    // 设置msg_state（用于同步外部修改）
    void set_msg_state(const AGVState& new_msg_state);

    // 设置状态发布方指针
    void set_data_publish(std::shared_ptr<AGVDataPublish> data_publish);

    // 从MQTT消息同步数据（用于纯MQTT模式）
    void update_from_mqtt(const AGVOrder &mqtt_order_messages);

    // 任务已完成，进行数据清零
    void OrderFinished();

    // 设置即时动作状态（用于connect_success_auto等，线程安全）
    void set_instant_action_state(const ActionStates& action_state);

    // 批量更新 action_status（用于状态机流转，线程安全）
    bool update_action_status(const std::string& from_status, const std::string& to_status);

    // 检查是否有挂钩动作（线程安全）
    bool has_hook_action();

    // 检查是否有需到达后执行的设备动作（如LOAD/UNLOAD/CHARGE/WAIT等，线程安全）
    bool has_device_action();

    // 将 last_node_id 和 last_node_sequence_id 更新为订单中最后一个节点（线程安全）
    void update_last_node_to_final();

    // 检查所有动作是否已完成（线程安全）
    bool is_action_finished();

    // 是否接收到数据，只要一直接收到数据，处理过程就不会停
    bool get_order;
    // 是否接受到新消息的标识，新消息才更新
    bool messages_change;
    // 是否是新的任务，第一次三个点都是新的，之后就只处理后两个点，因为第一个点就是上一次三个点中的最后一个点
    bool first_order;

    // 接收rcs消息解析出来的目标点位姿
    std::vector<std::string> released_goal_node_id;
    // 存储该order所有的目标点?
    std::vector<double> released_goal_x;
    std::vector<double> released_goal_y;
    std::vector<double> released_goal_theta;
    std::vector<double> released_goal_allowed_deviation_xy;
    std::vector<double> released_goal_allowed_deviation_theta;
    std::vector<double> released_edge_orientation;
    std::vector<int> released_edge_obstacle_avoidance_channels;

    std::vector<Trajectory> released_goal_trajectory;

    // 接收rcs消息解析解析出节点、边、动作数量，state数据填充时需要该值声明容器大小
    int released_node_size, released_edge_size, released_action_size;

    vda5050_interfaces::msg::AGVState msg_state;   
    // order_messages结构体一次性获取所有的变量的值，保存为数据快照，防止数据不一致的问题
    OrderMessages order_messages;

    // 上一个子任务的order_update_id
    int last_order_update_id;

private:

    // 订阅方的回调函数，解析获取到的数据
    void order_callback(const AGVOrder &msg);

    // 订阅方声明
    rclcpp::Subscription<AGVOrder>::SharedPtr subscription_;

    // 动作容器索引，在nodes和edgs都有可能有动作，该值用于辅助填充容器
    int m_vector_size;

    // 声明一个容器，
    std::multimap<std::string,std::string> m_action_vec;
    
    // 添加递归互斥锁，避免内部回调时死锁
    std::recursive_mutex data_mutex_;  

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<AGVDataPublish> data_publish_;

};
# endif