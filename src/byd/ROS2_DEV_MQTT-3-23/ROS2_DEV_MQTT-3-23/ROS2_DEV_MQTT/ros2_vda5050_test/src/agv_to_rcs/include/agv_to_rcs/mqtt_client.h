/************************************** File Info ****************************************
* @file:       mqtt_client.h                                                                   
* @author:     Assistant                                                          
* @date:       2024-11-20                                         
* @version:    V1.0                                                                              
* @brief:      MQTT客户端包装类，用于发布AGV状态数据到MQTT代理
******************************************************************************************/
#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"
#include "rclcpp/rclcpp.hpp"

// MQTT遗嘱信息结构体
struct MQTTWill {
    std::string topic;        // 遗嘱主题
    std::string payload;       // 遗嘱消息内容
    int qos = 1;              // 遗嘱消息的QoS等级 (0, 1, 2)
    bool retained = false;    // 遗嘱消息是否保留
    bool enabled = false;     // 是否启用遗嘱
};

// MQTT配置结构体
struct MQTTConfig {
    std::string client_id;
    std::string host = "localhost";
    int port = 1883;
    int keepalive = 60;
    bool clean_session = true;
    std::string username = "";
    std::string password = "";
    int reconnect_delay = 1;
    int max_reconnect_delay = 60;
    int max_inflight_messages = 1000;  // 增加缓冲区：默认20，增加到1000
    MQTTWill will;                    // 遗嘱信息配置
};

// 连接状态枚举
enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    DISCONNECTING
};

class MQTTClient {
public:
    /**
     * @brief 构造函数
     * @param config MQTT配置
     * @param logger ROS2日志器
     */
    MQTTClient(const MQTTConfig& config, rclcpp::Logger logger);
    
    /**
     * @brief 析构函数
     */
    ~MQTTClient();

    /**
     * @brief 连接到MQTT代理
     * @return 连接是否成功
     */
    bool connect();

    /**
     * @brief 断开MQTT连接
     */
    void disconnect();

    /**
     * @brief 发布消息到指定主题
     * @param topic 主题名称
     * @param payload 消息内容
     * @param qos 服务质量等级 (0, 1, 2)
     * @return 发布是否成功
     */
    bool publish(const std::string& topic, const std::string& payload, int qos = 0);

    /**
     * @brief 获取连接状态
     * @return 当前连接状态
     */
    ConnectionStatus getStatus() const { return status_; }

    /**
     * @brief 检查是否已连接
     * @return 是否已连接
     */
    bool isConnected() const { return status_ == ConnectionStatus::CONNECTED; }

    /**
     * @brief 订阅主题
     * @param topic 要订阅的主题
     * @param qos 服务质量等级 (0, 1, 2)
     * @return 订阅是否成功
     */
    bool subscribe(const std::string& topic, int qos = 0);

    /**
     * @brief 取消订阅主题
     * @param topic 要取消订阅的主题
     * @return 取消订阅是否成功
     */
    bool unsubscribe(const std::string& topic);

    /**
     * @brief 设置消息接收回调函数
     * @param callback 回调函数，参数为(topic, payload)
     */
    void setMessageCallback(std::function<void(const std::string&, const std::string&)> callback);

    /**
     * @brief 启动消息接收循环（在独立线程中运行）
     */
    void startMessageLoop();

    /**
     * @brief 停止消息接收循环
     */
    void stopMessageLoop();

private:
    MQTTConfig config_;
    rclcpp::Logger logger_;
    std::unique_ptr<mqtt::async_client> client_;
    std::atomic<ConnectionStatus> status_;
    std::function<void(const std::string&, const std::string&)> message_callback_;
    std::atomic<bool> message_loop_running_;
    std::thread message_loop_thread_;
    
    // 记录已订阅的主题和QoS，用于重连后自动重新订阅
    struct SubscriptionInfo {
        std::string topic;
        int qos;
    };
    std::vector<SubscriptionInfo> subscribed_topics_;
    std::mutex subscribed_topics_mutex_;  // 保护订阅列表的互斥锁
    
    // 添加MQTT回调类
    class MQTTCallback : public virtual mqtt::callback {
    public:
        MQTTCallback(MQTTClient* client) : client_(client) {}
        
        void message_arrived(mqtt::const_message_ptr msg) override {
            if (client_ && client_->message_callback_) {
                std::string topic = msg->get_topic();
                std::string payload = msg->to_string();
                client_->message_callback_(topic, payload);
            }
        }
        
        void connection_lost(const mqtt::string& cause) override {
            if (client_) {
                std::string cause_str = cause.empty() ? "未知原因（可能为网络中断或Keep-Alive超时）" : std::string(cause);
                RCLCPP_WARN(client_->logger_, "MQTT连接丢失: %s", cause_str.c_str());
                // 更新连接状态
                client_->status_ = ConnectionStatus::DISCONNECTED;
            }
        }
        
        void connected(const mqtt::string& cause) override {
            if (client_) {
                std::string cause_str = cause.empty() ? "初始连接" : std::string(cause);
                RCLCPP_INFO(client_->logger_, "MQTT连接成功: %s", cause_str.c_str());
                // 更新连接状态
                client_->status_ = ConnectionStatus::CONNECTED;
                // 延迟重新订阅，等待连接稳定和缓冲区清空
                // 使用异步方式，避免阻塞回调
                std::thread([client = client_]() {
                    // 等待1秒，让连接稳定，同时等待缓冲区清空
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    if (client && client->status_ == ConnectionStatus::CONNECTED) {
                        client->resubscribe_topics();
                    }
                }).detach();
            }
        }
        
    private:
        MQTTClient* client_;
    };
    
    std::shared_ptr<MQTTCallback> mqtt_callback_;
    
    // 重新订阅所有已订阅的主题（内部方法）
    void resubscribe_topics();
};

#endif // MQTT_CLIENT_H