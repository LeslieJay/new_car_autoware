/************************************** File Info ****************************************
* @file:       subscriber_instant_action.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-09                                         
* @version:    V0.0                                                                              
* @brief:      AGV设备接受RCS即时操作话题的订阅端，订阅话题，解析消息
------------------------------------------------------------------------------------------
* @modified:   增加一个接收消息超时判断
* @date:       2024-11-09
* @version:    V0.1
* @description:当出现中断的情况时，接收到的数据不会更新，如果不处理，会影响后续的判断
* @note:        
******************************************************************************************/

# ifndef SUBSCRIBER_INSTANT_ACTION_H
# define SUBSCRIBER_INSTANT_ACTION_H

// agv_receive_instant_action.h
# include "rclcpp/rclcpp.hpp"

// vda5050协议接口 
# include "vda5050_interfaces/msg/agv_instant_actions.hpp"

// agv参数结构体
# include "agv_config.h"

// 互斥锁
# include <mutex>
# include <condition_variable>

using vda5050_interfaces::msg::AGVInstantActions;

/*****************************************************************************************
* @brief:      定义了一个枚举来表示action类型
* @author:     刘鸿彬
* @date:       2024-11-12
* @version:    V0.0
* @note:       提高代码的可读性和可维护性
******************************************************************************************/
enum class ActionType {

  CONNECT_SUCCESS_AUTO,
  CONNECT_FAIL_AUTO,
  AGV_ONLINE,
  HEARTBEAT_ACK,
  AGV_OFFLINE,
  START_CHARGE,
  STOP_CHARGE,
  ORDER_FINISHED,
  CANCEL_ORDER,
  START_PAUSE,
  STOP_PAUSE,
  FACTSHEET_REQUEST,
  OTHER

};

class InstantActionsListener
{
public:

    /*****************************************************************************************
    * @brief:      类的构造函数
    * @param:      无
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-09
    * @version:    V0.0
    ******************************************************************************************/
    InstantActionsListener(std::shared_ptr<rclcpp::Node> node);
    // int messages_flag = 0;
    
    /*****************************************************************************************
    * @brief:      获取最新的action_type
    * @param:      无
    * @return:     无
    * @author:     刘鸿彬
    * @date:       2024-11-09
    * @version:    V0.0
    ******************************************************************************************/
    InstantActionMessages get_instant_action_messages();

    /*****************************************************************************************
    * @brief:      获取条件变量 messagesNotEmpty 的引用
    * @param:      无
    * @return:     条件变量的引用
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    std::condition_variable& get_messages_not_empty();

    /*****************************************************************************************
    * @brief:      获取 receive_new_actions 标志
    * @param:      无
    * @return:     receive_new_actions 的值
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    bool get_receive_new_actions() const;

    /*****************************************************************************************
    * @brief:      设置 receive_new_actions 标志
    * @param:      value - 要设置的值
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    void set_receive_new_actions(bool value);

    /*****************************************************************************************
    * @brief:      获取 interrupt_order_message
    * @param:      无
    * @return:     interrupt_order_message 的副本
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    InterruptOrderMessage get_interrupt_order_message() const;

    /*****************************************************************************************
    * @brief:      更新 interrupt_order_message 的 action_status
    * @param:      action_status - 新的状态值
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-XX
    * @version:    V1.0
    ******************************************************************************************/
    void update_interrupt_order_message_status(const std::string& action_status);

    /*****************************************************************************************
    * @brief:      从MQTT消息同步数据（用于纯MQTT模式）
    * @param:      mqtt_instant_action_messages - MQTT解析后的ROS2 AGVInstantActions消息
    * @return:     无
    * @author:     Assistant
    * @date:       2024-12-02
    * @version:    V1.0
    * @note:       在纯MQTT模式下，ROS2订阅者不会收到消息，需要通过此方法手动同步
    ******************************************************************************************/
    void update_from_mqtt(const AGVInstantActions& mqtt_instant_action_messages);

    InstantActionMessages instant_action_messages;

    // 接受到新数据
    std::atomic<bool> receive_new_messages;

    // 声明最近的目标节点位姿变量
    // 原子操作，确保任意时刻只有一个线程对这个资源进行访问，避免了锁的使用，提高了效率
    double m_goal_node_x;
    double m_goal_node_y;
    double m_goal_node_theta;

    std::string action_type;

    
private:

    std::shared_ptr<rclcpp::Node> node_; 
    
    // 订阅方回调函数
    void instant_action_callback(const AGVInstantActions &msg);

    void timer_callback();

    // 将字符串转化为枚举类型，未知值则反馈抛出异常
    ActionType convertActionTypeOrThrow(const std::string& action_type_str);

    // 声明私有变量
    rclcpp::Subscription<AGVInstantActions>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr instant_action_timer_;

    std::mutex data_mutex_;
    // 用于保护 receive_new_actions_ 和 interrupt_order_message_ 的互斥锁
    mutable std::mutex state_mutex_;

    // 记录接收到最后一条数据的时间
    std::chrono::time_point<std::chrono::steady_clock> last_msg_time_ ;

    // 判断是否接收到消息（内部管理，线程安全）
    bool receive_new_actions_;
    // 记录上个消息的action_id

    // 条件变量，用于通知等待的线程
    std::condition_variable messagesNotEmpty_;
    
    std::string last_action_id;

    // 中断订单消息（内部管理，线程安全）
    InterruptOrderMessage interrupt_order_message_;
    
};
# endif