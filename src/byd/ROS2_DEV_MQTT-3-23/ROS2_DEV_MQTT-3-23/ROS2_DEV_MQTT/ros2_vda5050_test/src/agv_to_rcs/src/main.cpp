/************************************** File Info ****************************************
* @file:       main.cpp
* @author:     刘鸿彬
* @date:       2024-11-20
* @version:    V0.0
* @brief:      主函数，功能为初始化ros2应用程序和事件，实例化节点和状态机，注册状态到状态机，初始化状态并循环处理状态事件
* @note:       计划使用状态机来管理设备的状态，同时使用行为树来处理短期的决策和行动
------------------------------------------------------------------------------------------
* @modified:   增加while休眠时间，降低cpu开销
* @date:       2024-11-28
* @version:    V0.0
* @description:未增加将使得cpu占用率提高，影响程序运行
------------------------------------------------------------------------------------------
* @modified:   使用多线程的形式，将部分需要在整个程序生命周期运行的功能独立出去
* @date:       2024-11-10
* @version:    V0.0
* @description:该部分功能：1、需要一直运行，2、其更新的数据其他部分需要用到。非线程处理，代码各个模块将耦合较高
* @note:       通过全局变量+多线程+互斥锁的结构解决上述问题，需要该数据的部分，在互斥锁的约束下进行读取
------------------------------------------------------------------------------------------
* @modified:   在整个系统生命周期内，保证一个类只能产生一个实例，确保该类的唯一性,节省资源,避免了多个对象引起的复杂操作
* @date:       2024-11-15
* @version:    V0.0
* @description:
* @note:
******************************************************************************************/

#include "agv_bone.h"

#include "server_obstacle.h"
#include "subscriber_battery.h"
#include "subscriber_velocity.h"
#include "subscriber_candata.h"
#include "mqtt_client.h"
#include "mqtt_message_handler.h"
#include <iostream>
#include <string>
#include <memory>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <state_detecting_behaviors.h>
#include <state_error_behaviors.h>
#include <state_idle_behaviors.h>
#include <state_init_behaviors.h>
#include <state_lock_behaviors.h>
#include <state_running_behaviors.h>
#include <tool_black_box.h>
#include "vda5050_interfaces/msg/agv_order.hpp"
#include "vda5050_interfaces/msg/agv_instant_actions.hpp"
#include "vda5050_interfaces/msg/agv_connection.hpp"
#include "json_converter.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <climits>
#include <thread>
#include <sys/syscall.h>
#include <unistd.h>
#include <sstream>
#include <thread>

std::string thread_id_str(const std::thread::id& id) {
    std::ostringstream oss;
    oss << id;
    return oss.str();
}
using namespace BT;
using vda5050_interfaces::msg::AGVConnection;

// 定义全局函数用于日志输出和状态转换处理
NodeStatus LogStateTransition(const TreeNode &node)
{
    std::cout << "1111111111111111111111111!" << std::endl;

    // 从节点配置中获取黑板指针
    auto blackboard = node.config().blackboard;

    std::string last_state, current_state;
    bool success = blackboard->get("last_state", last_state);
    success = success && blackboard->get("current_state", current_state);
    std::cout << "退出 " << last_state << ", AGV进入 " << current_state << "\n";

    if (success)
        return NodeStatus::SUCCESS;
    else
        return NodeStatus::FAILURE;
}

// 定义条件节点的回调函数
NodeStatus IsCurrentState(const TreeNode &node, const std::string &state)
{
    std::cout << "进入IsCurrentStateInit!" << std::endl; std::cout.flush();
    std::string current_state;
    bool success = node.config().blackboard->get("current_state", current_state);
    if (current_state == state && success)
        return NodeStatus::SUCCESS;
    else
        return NodeStatus::FAILURE;
}

NodeStatus IsEvent(const TreeNode &node, const std::string &event)
{
    std::string AGV_Event;
    bool success = node.config().blackboard->get("AGV_Event", AGV_Event);
    if (AGV_Event == event && success)
        return NodeStatus::SUCCESS;
    else
        return NodeStatus::FAILURE;
}

std::atomic<bool> exit_flag = false; // 全局退出标志

/*****************************************************************************************
 * @brief:      处理程序异常信号的执行函数，监测到异常则执行该函数
 * @param:      捕获到的信号的编号
 * @return:     无
 * @author:     刘鸿彬
 * @date:       2024-11-29
 * @version:    V0.0
 ******************************************************************************************/
void handle_sigint(int signum)
{
    // 处理信号的代码
    std::cout << "捕获到异常信号： " << signum << " ！" << std::endl;
    // 释放资源，结束其他线程
    rclcpp::shutdown();
    exit_flag = true;
}
/*****************************************************************************************
 * @brief:      处理程序异常信号的检测函数，用于检测异常信号
 * @author:     刘鸿彬
 * @date:       2024-11-29
 * @version:    V0.0
 * @note:       signal(int sig, void (*func)(int))),sig：要处理的信号类型，例如SIGINT表示中断信号（Ctrl+C）
 * @note:       func：指向信号处理函数的指针。如果将该参数设置为SIG_IGN，则忽略该信号；如果设置为SIG_DFL，则使用默认的信号处理方式
 ******************************************************************************************/
void Anomaly_detection()
{
    // 按下ctrl + c
    signal(SIGINT, handle_sigint);
    // 进程异常终止信号，通常由abort()函数产生
    signal(SIGABRT, handle_sigint);
    // 退出信号，通常由用户按下Ctrl+\产生
    signal(SIGQUIT, handle_sigint);
    // 终止进程信号，通常用于程序正常退出
    // signal(SIGTERM,handle_sigint);
}
pid_t get_tid() {
    return syscall(SYS_gettid);
}
// 主函数
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    // 打印系统 TID（可被 ps 查询）
    auto all_nodes = std::make_shared<rclcpp::Node>("agv_to_rcs_main");
    RCLCPP_INFO(all_nodes->get_logger(), "Main thread TID (LWP): %d", get_tid());

    // 初始化参数配置
    if (!load_agv_config("/home/nvidia/autoware/src/byd/ROS2_DEV_MQTT-3-23/ROS2_DEV_MQTT-3-23/ROS2_DEV_MQTT/ros2_vda5050_test/src/agv_to_rcs/src/agv_config.yaml")) {
        // 处理错误
        std::cout << "加载配置文件失败！" << std::endl;
        return 0;
    }

    RCLCPP_INFO(all_nodes->get_logger(), "创建 agv_to_rcs_main 节点");

    Anomaly_detection();


    std::string agv_current_state = "unknown";

    // 以下两个变量用于存储从mqtt传来的数据（ROS2消息类型）
    vda5050_interfaces::msg::AGVOrder mqtt_order_messages;
    vda5050_interfaces::msg::AGVInstantActions mqtt_instant_action_messages;

    // 根据配置决定使用ROS2订阅者还是MQTT订阅者
    std::shared_ptr<MQTTClient> mqtt_subscriber = nullptr;
    std::shared_ptr<MQTTMessageHandler> mqtt_handler = nullptr;
    
    // 总是创建ROS2订阅者（为了兼容性，避免空指针问题）
    // 在mqtt_only模式下，这些订阅者会被创建但不会被积极使用
    auto instant_action_listener = std::make_shared<InstantActionsListener>(all_nodes);
    auto order_listener = std::make_shared<OrderListener>(all_nodes);
    
    // 根据通信模式决定主要使用哪种通信方式
    if (agv_config.communication_mode == "ros2_only" || agv_config.communication_mode == "dual_channel") {
        RCLCPP_INFO(all_nodes->get_logger(), "✓ 已启用ROS2订阅者用于order和instant action消息");
    }
    
    if (agv_config.communication_mode == "mqtt_only" || agv_config.communication_mode == "dual_channel") {
        if (agv_config.enable_mqtt_subscription) {
            // 创建MQTT订阅者
            MQTTConfig mqtt_sub_config;
            mqtt_sub_config.client_id = "agv_subscriber_" + agv_config.serial_number;
            mqtt_sub_config.host = agv_config.mqtt_broker_host;
            mqtt_sub_config.port = agv_config.mqtt_broker_port;
            mqtt_sub_config.username = agv_config.mqtt_username;
            mqtt_sub_config.password = agv_config.mqtt_password;
            mqtt_sub_config.keepalive = 5;  // 减少keepalive时间，加快异常断开检测（5秒）
            mqtt_sub_config.clean_session = true;
            mqtt_sub_config.max_inflight_messages = 1000;  // 增加缓冲区，避免消息积压
            
            // 订阅者通常不需要遗嘱信息（可选）
            // mqtt_sub_config.will.enabled = false;
            
            mqtt_subscriber = std::make_shared<MQTTClient>(mqtt_sub_config, all_nodes->get_logger());
            mqtt_handler = std::make_shared<MQTTMessageHandler>(all_nodes);
            
            RCLCPP_INFO(all_nodes->get_logger(), "✓ 已启用MQTT订阅者用于order和instant action消息");
        }
    }
    
    // 创建统一的位姿订阅者（根据vehicle_type自动选择激光或二维码导航）
    std::shared_ptr<ListenerPose> current_pose_listener = std::make_shared<ListenerPose>(all_nodes);
    if (agv_config.vehicle_type == "laser") {
        RCLCPP_INFO(all_nodes->get_logger(), "✓ 创建激光SLAM位姿订阅者");
    } else {
        RCLCPP_INFO(all_nodes->get_logger(), "✓ 创建二维码定位位姿订阅者");
    }
    
    auto battery_subscriber = std::make_shared<BatteryListener>(all_nodes);
    auto velocity_subscriber = std::make_shared<VelocityListener>(all_nodes);
    auto can_data_listener = std::make_shared<CanDataListener>(all_nodes);
    RCLCPP_INFO(all_nodes->get_logger(), "✓ 创建CAN数据订阅者（IO数据、错误数据、硬件数据）");
    auto obstacle_server = std::make_shared<ObstacleServer>(all_nodes, current_pose_listener);
    
    auto agv_data_publish = std::make_shared<AGVDataPublish>(all_nodes, agv_current_state, current_pose_listener, order_listener, battery_subscriber, velocity_subscriber, obstacle_server, can_data_listener);

    // AGV运行过程中的核心重要变量
    AGVBone agv_bone(all_nodes,
                     instant_action_listener,
                     current_pose_listener,
                     order_listener,
                     agv_data_publish,
                     battery_subscriber,
                     velocity_subscriber,
                     can_data_listener,
                     obstacle_server,
                     agv_current_state);

    // 这个是单线程执行器方案
    // std::thread all_nodes_thread([all_nodes]() { rclcpp::spin(all_nodes); });

    // 这个是多线程执行器方案
    // 创建一个子线程运行 MultiThreadedExecutor
    std::thread executor_thread([all_nodes]() {
        auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor->add_node(all_nodes);
        executor->spin();  // 在子线程中运行多线程执行器
    });


    // 启动线程订阅需要一段等待时间，否则程序运行过快，后面判断未订阅到位姿消息出错
    // 经50次快速重启测试，3s等待时间只有有一次未成功订阅到数据，兼顾时间的情况下，3s较为合适
    std::this_thread::sleep_for(std::chrono::seconds(3));



    // 确保状态发布的多线程在获取位姿之后启动
    std::string pose_type_str = (agv_config.vehicle_type == "laser") ? "激光SLAM" : "二维码定位";
    while (!current_pose_listener->get_pose) {
        std::cout << "等待系统获取AGV当前位姿信息（" << pose_type_str << "）！" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // 获取到当前位姿之后再启动状态上报
    // 位姿数据已由 ListenerPose 内部管理，无需手动更新

    // 配置MQTT发布功能（可选功能，失败不影响ROS运行）
    MQTTConfig mqtt_config;
    mqtt_config.client_id = "agv_publisher_" + agv_config.serial_number;
    mqtt_config.host = agv_config.mqtt_broker_host;
    mqtt_config.port = agv_config.mqtt_broker_port;
    mqtt_config.username = agv_config.mqtt_username;
    mqtt_config.password = agv_config.mqtt_password;
    mqtt_config.keepalive = 5;  // 减少keepalive时间，加快异常断开检测（5秒）
    mqtt_config.clean_session = true;
    mqtt_config.max_inflight_messages = 1000;  // 增加缓冲区，避免消息积压
    
    // 配置MQTT遗嘱信息（当AGV异常断开时，自动发布离线消息）
    // 参考 connection_timer_callback 的方式，创建完整的 AGVConnection 消息
    mqtt_config.will.enabled = true;
    mqtt_config.will.topic = "uagv/v1/BYD/" + agv_config.serial_number + "/connection";
    
    // 创建完整的 AGVConnection 消息对象
    AGVConnection will_message;
    
    // 获取当前时间点并格式化为字符串（参考 connection_timer_callback）
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now = *std::localtime(&time_t_now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tm_now);
    std::string time_string(buffer);
    
    // 填充消息字段（参考 connection_timer_callback）
    will_message.header_id = 0;  // 遗嘱消息使用 0 作为 header_id
    will_message.timestamp = time_string;
    will_message.version = agv_config.version;
    will_message.manufacturer = agv_config.manufacturer;
    will_message.serial_number = agv_config.serial_number;
    will_message.connection_state = "OFFLINE";
    
    // 使用 JsonConverter 转换为 JSON（与 connection_timer_callback 保持一致）
    mqtt_config.will.payload = JsonConverter::toJson(will_message);
    mqtt_config.will.qos = 1;  // 使用QoS 1确保消息至少送达一次
    mqtt_config.will.retained = false;  // 不保留消息
    
    // 尝试初始化MQTT发布功能
    if (agv_data_publish->initMQTT(mqtt_config)) {
        RCLCPP_INFO(all_nodes->get_logger(), "✓ MQTT发布功能已启用，数据将同时通过ROS和MQTT发布");
        RCLCPP_INFO(all_nodes->get_logger(), "  MQTT发布Topics:");
        RCLCPP_INFO(all_nodes->get_logger(), "  - uagv/v1/BYD/%s/connection", agv_config.serial_number.c_str());
        RCLCPP_INFO(all_nodes->get_logger(), "  - uagv/v1/BYD/%s/state", agv_config.serial_number.c_str());
        RCLCPP_INFO(all_nodes->get_logger(), "  - uagv/v1/BYD/%s/visualization", agv_config.serial_number.c_str());
        RCLCPP_INFO(all_nodes->get_logger(), "  - uagv/v1/BYD/%s/factsheet", agv_config.serial_number.c_str());
    } else {
        RCLCPP_WARN(all_nodes->get_logger(), "⚠ MQTT发布功能未启用，仅使用ROS发布");
        RCLCPP_WARN(all_nodes->get_logger(), "  可能原因：MQTT代理未运行或网络连接问题");
    }

    // 启动MQTT订阅功能（如果已配置）
    if (mqtt_subscriber && mqtt_handler) {
        // 设置消息处理回调
        mqtt_handler->setOrderCallback([&mqtt_order_messages, &mqtt_handler, order_listener](const std::string& payload) {
            if (mqtt_handler->parseOrderMessage(payload, mqtt_order_messages)) {
                // 同步MQTT消息到OrderListener（用于纯MQTT模式）
                order_listener->update_from_mqtt(mqtt_order_messages);
            }
        });
        
        mqtt_handler->setInstantActionCallback([&mqtt_instant_action_messages, &mqtt_handler, instant_action_listener](const std::string& payload) {
            if (mqtt_handler->parseInstantActionMessage(payload, mqtt_instant_action_messages)) {
                // 同步MQTT消息到InstantActionsListener（用于纯MQTT模式）
                instant_action_listener->update_from_mqtt(mqtt_instant_action_messages);
            }
        });
        
        mqtt_subscriber->setMessageCallback([mqtt_handler](const std::string& topic, const std::string& payload) {
            mqtt_handler->handleMessage(topic, payload);
        });
        
        // 连接MQTT代理
        if (mqtt_subscriber->connect()) {
            // 订阅相关主题
            bool subscription_success = true;
            if (!mqtt_subscriber->subscribe(agv_config.mqtt_order_topic, 1)) {
                RCLCPP_ERROR(all_nodes->get_logger(), "订阅order主题失败: %s", agv_config.mqtt_order_topic.c_str());
                subscription_success = false;
            }
            if (!mqtt_subscriber->subscribe(agv_config.mqtt_instant_action_topic, 1)) {
                RCLCPP_ERROR(all_nodes->get_logger(), "订阅instant action主题失败: %s", agv_config.mqtt_instant_action_topic.c_str());
                subscription_success = false;
            }
            
            if (subscription_success) {
                // 启动消息接收循环
                mqtt_subscriber->startMessageLoop();
                RCLCPP_INFO(all_nodes->get_logger(), "✓ MQTT订阅功能已启用");
                RCLCPP_INFO(all_nodes->get_logger(), "  MQTT订阅Topics:");
                RCLCPP_INFO(all_nodes->get_logger(), "  - %s", agv_config.mqtt_order_topic.c_str());
                RCLCPP_INFO(all_nodes->get_logger(), "  - %s", agv_config.mqtt_instant_action_topic.c_str());
            } else {
                RCLCPP_ERROR(all_nodes->get_logger(), "MQTT主题订阅失败，断开连接");
                mqtt_subscriber->disconnect();
            }
        } else {
            RCLCPP_ERROR(all_nodes->get_logger(), "MQTT订阅连接失败，将仅使用ROS2订阅");
        }
    }

    // 创建发布方
    agv_data_publish->publish_agv_state("ONLINE");
    
    // 以下是黑匣子功能类实例的创建和数据记录的线程运行。
    AGVBlackBox blackBox(agv_bone);

    blackBox.startRecording();

    // 开始构建行为树*****************************************************************************

    // 创建工厂对象用于注册自定义节点
    BehaviorTreeFactory factory;

    // 为行为树的节点们准备一个全局访问的黑板（用于存放一些变量）
    auto blackboard = Blackboard::create();
    std::cout << "调用 publish_agv_state 完成" << std::endl;

    // 注册自定义的条件节点和动作
    // registerSimpleCondition注册一个简单条件节点，判断某个状态是否成立
    factory.registerSimpleCondition("IsCurrentStateInit", std::bind(&IsCurrentState, std::placeholders::_1, "init"));
    factory.registerSimpleCondition("IsCurrentStateIdle", std::bind(&IsCurrentState, std::placeholders::_1, "idle"));
    factory.registerSimpleCondition("IsCurrentStateRunning", std::bind(&IsCurrentState, std::placeholders::_1, "running"));
    factory.registerSimpleCondition("IsCurrentStateDetecting", std::bind(&IsCurrentState, std::placeholders::_1, "detecting"));
    factory.registerSimpleCondition("IsCurrentStateError", std::bind(&IsCurrentState, std::placeholders::_1, "error"));
    factory.registerSimpleCondition("IsCurrentStateLock", std::bind(&IsCurrentState, std::placeholders::_1, "lock"));

    // 状态转换日志registerSimpleAction注册一个简单动作节点，不用继承BT 节点基类
    factory.registerSimpleAction("LogStateTransition", &LogStateTransition);
    // factory.registerSimpleAction("LogStateTransition", std::bind(&LogStateTransition, std::placeholders::_1, agv_current_state));

    // registerNodeType注册一个完整的自定义节点类型，在任何类成员函数中可以通过this直接给Blackboard变量赋值
    // 虽然在这里定义了节点，但是只有在createTreeFromText的时候，才会开始执行类的构造函数
    factory.registerNodeType<InitStateBehaviors>("InitStateBehaviors");
    factory.registerNodeType<ErrorStateBehaviors>("ErrorStateBehaviors", agv_bone);
    factory.registerNodeType<LockStateBehaviors>("LockStateBehaviors", agv_bone);
    factory.registerNodeType<IdleStateBehaviors>("IdleStateBehaviors", agv_bone);

    if (agv_config.vehicle_type == "laser") 
    {
        factory.registerNodeType<LaserRunningStateBehaviors>("RunningStateBehaviors", agv_bone);
    } 
    else 
    {
        factory.registerNodeType<QRRunningStateBehaviors>("RunningStateBehaviors", agv_bone);
    }

    factory.registerNodeType<DetectingStateBehaviors>("DetectingStateBehaviors", agv_bone);

    // 创建行为树XML字符串
    const char *xml_text =
        R"(
        <root BTCPP_format="4" >
            <BehaviorTree ID="MainTree">
                <Sequence>
                    <LogStateTransition />
                    <Fallback>
                        <!-- init -->
                        <Sequence>
                            <IsCurrentStateInit />
                            <InitStateBehaviors />
                        </Sequence>
                        <!-- idle -->
                        <Sequence>
                            <IsCurrentStateIdle />
                            <IdleStateBehaviors />
                        </Sequence>
                        <!-- running -->
                        <Sequence>
                            <IsCurrentStateRunning />
                            <RunningStateBehaviors />
                        </Sequence>
                        <!-- detecting -->
                        <Sequence>
                            <IsCurrentStateDetecting />
                            <DetectingStateBehaviors />
                        </Sequence>
                        <!-- error -->
                        <Sequence>
                            <IsCurrentStateError />
                            <ErrorStateBehaviors />
                        </Sequence>
                        <!-- lock -->
                        <Sequence>
                            <IsCurrentStateLock />
                            <LockStateBehaviors />
                        </Sequence>
                    </Fallback>
                </Sequence>
            </BehaviorTree>
        </root>
        )";

    // 设置初始状态到黑板上
    blackboard->set("last_state", "unknown");
    blackboard->set("current_state", "init");
    blackboard->set("AGV_Event", "init_try");
    blackboard->set("fault_code", "NONE");
    // blackboard->set("current_state", "running");
    // blackboard->set("AGV_Event", "receive_order");

    std::cout << "准备调用 publish_agv_state(ONLINE)111" << std::endl;

    // 加载行为树
    auto tree = factory.createTreeFromText(xml_text, blackboard);
    std::cout << "准备调用 publish_agv_state(ONLINE)" << std::endl;

    while (!exit_flag)
    {

        NodeStatus status = tree.tickOnce();
        bool success = blackboard->get("current_state", agv_current_state);
        if (status != NodeStatus::SUCCESS || !success)
        {
            std::cout << "行为树发生错误！" << std::endl;
            break;
        }
        // 休眠10ms
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 遇到障碍需要黑匣子记录数据
    blackBox.stopRecording();

    // 停止MQTT订阅（如果已启动）
    if (mqtt_subscriber) {
        mqtt_subscriber->stopMessageLoop();
        mqtt_subscriber->disconnect();
        RCLCPP_INFO(all_nodes->get_logger(), "MQTT订阅已停止");
    }

    // 等待节点线程结束
    executor_thread.join();

    // 关闭所有已创建的ROS2节点，结束上述三个线程中的节点，线程退出
    rclcpp::shutdown();

    return 0;
}
