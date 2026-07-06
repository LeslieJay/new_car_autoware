/************************************** File Info ****************************************
* @file:       tool_black_box.h     
* @author:     刘鸿彬
* @date:       2024-11-18
* @version:    V0.0
* @class:      工具类
* @brief:      AGV黑匣子功能
******************************************************************************************/
// tool_black_box.h
#ifndef TOOL_BLACK_BOX_H
#define TOOL_BLACK_BOX_H


// 全局变量
// # include "agv_bone.h"
# include "agv_bone.h"
# include "vda5050_interfaces/msg/agv_state.hpp"
# include "subscriber_order.h"
# include "ref_slam_interface/msg/io_data.hpp"
# include "ref_slam_interface/msg/err_data.hpp"
# include "ref_slam_interface/msg/hardware_data.hpp"
# include <iostream>
# include <string>
# include <thread>
# include <chrono>
# include <vector>
# include "tool_json.h"
# include <fstream>
# include <mutex> // 互斥锁
# include <condition_variable> // 条件变量
using vda5050_interfaces::msg::AGVState;
using json = nlohmann::json;

class AGVBlackBox {
public:
    // 构造函数，私有成员变量的初始化；与主线程的相关变量用引用绑定
    /*****************************************************************************************
    * @brief:      类的构造函数，采用参数列表的形式获取外部参数
    * @param:      &static_filepath：保存AGV静态数据信息的文件位置
    * @param:      &instant_filepath：保存AGV即时数据信息的文件位置
    * @param:      &currentpose：agv接收到的当前位姿消息结构体引用
    * @param:      &batterymessage：agv电池相关数据结构体引用
    * @param:      &ordermessage：agv接收到的order消息结构体引用
    * @param:      &instant_action_messages：agv接收的即时任务结构体引用
    * @param:      &messagesNotEmpty：条件变量，用于请求响应
    * @param:      &agvMessagesMutex：互斥锁，用于多线程读写
    * @param:      &receive_new_actions：用于判断是否收到最新的即时动作指令
    * @author:     刘鸿彬
    * @date:       2024-09-24
    ******************************************************************************************/
    AGVBlackBox(AGVBone agv_bone);
    ~AGVBlackBox();

    // 用于打开和关闭数据记录的线程
    void startRecording();
    void stopRecording();

    // OrderMessages类型（除去msg_state成员之后的拷贝函数）
    void copyOrderMessagesExceptMsgState(OrderMessages& from, OrderMessages& to);

    // 获取数据的方法（用于测试和调试）
    const CurrentPose& get_current_pose_message() const;
    BatteryMessages get_battery_message() const;
    OrderMessages get_order_message() const;
    InstantActionMessages get_instant_action_message() const;
    const std::vector<std::string>& get_system_connections() const;

private:
    // agv当前状态（通过引用）
    std::string &current_state_message_;
    // 条件变量，用于请求响应（从 instant_action_listener 获取）
    std::condition_variable &messagesNotEmpty_;
    // 互斥锁，用于多线程读写（内部创建）
    std::mutex agvMessagesMutex_;

    // 订阅者和服务器指针（用于获取数据）
    std::shared_ptr<InstantActionsListener> instant_action_listener;
    std::shared_ptr<OrderListener> order_listener;
    std::shared_ptr<ListenerPose> current_pose_listener;
    std::shared_ptr<BatteryListener> battery_listener_;
    std::shared_ptr<VelocityListener> velocity_listener_;
    std::shared_ptr<CanDataListener> can_data_listener_;
    std::shared_ptr<ObstacleServer> obstacle_server_;

    // 从 listener/server 获取的数据（私有成员，避免临时创建）
    CurrentPose current_pose_message_;
    OrderMessages order_message_;
    InstantActionMessages instant_action_message_;
    BatteryMessages battery_message_;
    VelocityMessages velocity_message_;
    ref_slam_interface::msg::IoData io_data_message_;
    ref_slam_interface::msg::ErrData err_data_message_;
    ref_slam_interface::msg::HardwareData hardware_data_message_;
    int obstacle_message_;

    // 是否记录数据的标签；构造类实例时默认关闭它
    bool isRecording_;
    // 小车静态数据（如自身状态）保存的文件地址
    std::string static_file_path_;
    // 小车动态数据（如即时指令）保存的文件地址
    std::string instant_file_path_;
    // CAN数据保存的文件地址
    std::string can_data_file_path_;
    // 记录静态数据的独立线程
    std::thread static_recordingThread_;
    // 记录即时数据的独立线程
    std::thread instant_recordingThread_;
    // 记录CAN数据的独立线程
    std::thread can_data_recordingThread_;

    // 系统间的连接情况
    std::vector<std::string> systemConnections_;

    // 报头，用来标签信息的序列号，从而剔除重复的数据
    int header_id_;
    // 记录时长，默认设定为120s
    int record_duration_;
    // 黑匣子具体解析数据包并记录静态数据的独立线程
    void recordStaticData();
    // 将解析好的数据包保存至指定黑匣子文件中（没有则创建；只保存120s内的数据）
    void recordStaticData_to_file(std::string filepath,std::vector<AGVState> &messages,std::vector<OrderMessages> &order_messages,int counter, 
                                  std::vector<int> &fork_height_messages, std::vector<std::string> &current_state_messages, std::vector<VelocityMessages> &velocity_messages, std::vector<int> &obstacle_messages);
    // 黑匣子具体解析数据包并记录即时信息的独立线程
    void recordInstantData();
    // 黑匣子具体解析数据包并记录CAN数据的独立线程
    void recordCanData();
    // 将解析好的CAN数据包保存至指定黑匣子文件中
    void recordCanData_to_file(std::string filepath, 
                               std::vector<ref_slam_interface::msg::IoData> &io_data_messages,
                               std::vector<ref_slam_interface::msg::ErrData> &err_data_messages,
                               std::vector<ref_slam_interface::msg::HardwareData> &hardware_data_messages,
                               std::vector<std::string> &timestamps,
                               int counter,
                               int buffer_size);

};
#endif // TOOL_BLACK_BOX_H