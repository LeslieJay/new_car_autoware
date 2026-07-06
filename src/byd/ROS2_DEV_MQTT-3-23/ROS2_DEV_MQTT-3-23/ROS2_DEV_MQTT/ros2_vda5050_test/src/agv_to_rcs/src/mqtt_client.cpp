/************************************** File Info ****************************************
* @file:       mqtt_client.cpp                                                                   
* @author:     Assistant                                                          
* @date:       2024-11-20                                         
* @version:    V1.0                                                                              
* @brief:      MQTT客户端包装类实现
******************************************************************************************/
#include "mqtt_client.h"
#include <chrono>
#include <mutex>
#include <algorithm>

MQTTClient::MQTTClient(const MQTTConfig& config, rclcpp::Logger logger)
    : config_(config), logger_(logger), status_(ConnectionStatus::DISCONNECTED), 
      message_loop_running_(false) {
    
    // 构建服务器地址
    std::string server_address = "tcp://" + config_.host + ":" + std::to_string(config_.port);
    
    // 创建MQTT客户端
    client_ = std::make_unique<mqtt::async_client>(server_address, config_.client_id);
    
    // 创建并设置回调
    mqtt_callback_ = std::make_shared<MQTTCallback>(this);
    client_->set_callback(*mqtt_callback_);
    
    RCLCPP_INFO(logger_, "MQTT客户端初始化完成，服务器地址: %s", server_address.c_str());
}

MQTTClient::~MQTTClient() {
    stopMessageLoop();
    disconnect();
}

bool MQTTClient::connect() {
    if (status_ != ConnectionStatus::DISCONNECTED) {
        RCLCPP_WARN(logger_, "MQTT客户端已经在连接或已连接状态");
        return false;
    }
    
    status_ = ConnectionStatus::CONNECTING;
    
    try {
        // 配置连接选项
        mqtt::connect_options conn_opts;
        conn_opts.set_keep_alive_interval(config_.keepalive);
        conn_opts.set_clean_session(config_.clean_session);
        conn_opts.set_automatic_reconnect(config_.reconnect_delay, config_.max_reconnect_delay);
        
        // 设置最大飞行中消息数量（增加缓冲区：默认20，增加到1000）
        conn_opts.set_max_inflight(config_.max_inflight_messages);
        
        // 设置用户名密码（如果提供）
        if (!config_.username.empty() && !config_.password.empty()) {
            conn_opts.set_user_name(config_.username);
            conn_opts.set_password(config_.password);
        }
        
        // 设置遗嘱信息（如果启用）
        if (config_.will.enabled && !config_.will.topic.empty()) {
            mqtt::will_options will_opts(
                config_.will.topic,
                config_.will.payload,
                config_.will.qos,
                config_.will.retained
            );
            conn_opts.set_will(will_opts);
            RCLCPP_INFO(logger_, "已设置MQTT遗嘱信息 - 主题: %s, QoS: %d, 保留: %s", 
                       config_.will.topic.c_str(), 
                       config_.will.qos,
                       config_.will.retained ? "是" : "否");
        }
        
        RCLCPP_INFO(logger_, "正在连接到MQTT代理 %s:%d", config_.host.c_str(), config_.port);
        
        // 同步连接（避免回调问题）
        auto token = client_->connect(conn_opts);
        
        // 等待连接完成（最多等待5秒）
        if (token->wait_for(std::chrono::seconds(5))) {
            status_ = ConnectionStatus::CONNECTED;
            RCLCPP_INFO(logger_, "MQTT连接成功，最大缓冲区消息数: %d", config_.max_inflight_messages);
            return true;
        } else {
            status_ = ConnectionStatus::DISCONNECTED;
            RCLCPP_ERROR(logger_, "MQTT连接超时");
            return false;
        }
        
    } catch (const mqtt::exception& exc) {
        status_ = ConnectionStatus::DISCONNECTED;
        RCLCPP_ERROR(logger_, "MQTT连接异常: %s", exc.what());
        return false;
    }
}

void MQTTClient::disconnect() {
    if (status_ == ConnectionStatus::DISCONNECTED) {
        return;
    }
    
    status_ = ConnectionStatus::DISCONNECTING;
    
    try {
        if (client_->is_connected()) {
            auto token = client_->disconnect();
            token->wait_for(std::chrono::seconds(3));
        }
        status_ = ConnectionStatus::DISCONNECTED;
        RCLCPP_INFO(logger_, "MQTT连接已断开");
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(logger_, "MQTT断开连接异常: %s", exc.what());
        status_ = ConnectionStatus::DISCONNECTED;
    }
}

bool MQTTClient::publish(const std::string& topic, const std::string& payload, int qos) {
    if (status_ != ConnectionStatus::CONNECTED) {
        RCLCPP_WARN(logger_, "MQTT未连接，无法发布消息到主题: %s", topic.c_str());
        return false;
    }
    
    try {
        // 创建消息
        auto msg = mqtt::make_message(topic, payload);
        msg->set_qos(qos);
        
        // 异步发布（不使用回调避免纯虚函数问题）
        auto token = client_->publish(msg);
        
        RCLCPP_DEBUG(logger_, "发布消息到主题 %s，大小: %zu 字节", topic.c_str(), payload.size());
        return true;
        
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(logger_, "发布消息到主题 %s 失败: %s, %zu字节", topic.c_str(), exc.what(), payload.size());
        return false;
    }
}

bool MQTTClient::subscribe(const std::string& topic, int qos) {
    if (status_ != ConnectionStatus::CONNECTED) {
        RCLCPP_WARN(logger_, "MQTT未连接，无法订阅主题: %s", topic.c_str());
        return false;
    }
    
    try {
        auto token = client_->subscribe(topic, qos);
        
        // 等待订阅完成（最多等待3秒）
        if (token->wait_for(std::chrono::seconds(3))) {
            RCLCPP_INFO(logger_, "成功订阅主题: %s (QoS: %d)", topic.c_str(), qos);
            
            // 记录订阅的主题（用于重连后自动重新订阅）
            std::lock_guard<std::mutex> lock(subscribed_topics_mutex_);
            // 检查是否已经记录过该主题
            bool already_subscribed = false;
            for (auto& sub : subscribed_topics_) {
                if (sub.topic == topic) {
                    sub.qos = qos;  // 更新QoS
                    already_subscribed = true;
                    break;
                }
            }
            if (!already_subscribed) {
                subscribed_topics_.push_back({topic, qos});
            }
            
            return true;
        } else {
            RCLCPP_ERROR(logger_, "订阅主题超时: %s", topic.c_str());
            return false;
        }
        
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(logger_, "订阅主题 %s 失败: %s", topic.c_str(), exc.what());
        return false;
    }
}

bool MQTTClient::unsubscribe(const std::string& topic) {
    if (status_ != ConnectionStatus::CONNECTED) {
        RCLCPP_WARN(logger_, "MQTT未连接，无法取消订阅主题: %s", topic.c_str());
        return false;
    }
    
    try {
        auto token = client_->unsubscribe(topic);
        
        // 等待取消订阅完成（最多等待3秒）
        if (token->wait_for(std::chrono::seconds(3))) {
            RCLCPP_INFO(logger_, "成功取消订阅主题: %s", topic.c_str());
            
            // 从记录中移除该主题
            std::lock_guard<std::mutex> lock(subscribed_topics_mutex_);
            subscribed_topics_.erase(
                std::remove_if(subscribed_topics_.begin(), subscribed_topics_.end(),
                    [&topic](const SubscriptionInfo& sub) { return sub.topic == topic; }),
                subscribed_topics_.end()
            );
            
            return true;
        } else {
            RCLCPP_ERROR(logger_, "取消订阅主题超时: %s", topic.c_str());
            return false;
        }
        
    } catch (const mqtt::exception& exc) {
        RCLCPP_ERROR(logger_, "取消订阅主题 %s 失败: %s", topic.c_str(), exc.what());
        return false;
    }
}

void MQTTClient::setMessageCallback(std::function<void(const std::string&, const std::string&)> callback) {
    message_callback_ = callback;
}

void MQTTClient::startMessageLoop() {
    if (message_loop_running_.load()) {
        RCLCPP_WARN(logger_, "消息接收循环已经在运行");
        return;
    }
    
    message_loop_running_ = true;
    RCLCPP_INFO(logger_, "✓ MQTT消息接收功能已启用（使用回调模式）");
    // 现在使用MQTT库的内置回调机制，不需要额外的线程
    // 消息将通过MQTTCallback::message_arrived自动处理
}

void MQTTClient::stopMessageLoop() {
    if (message_loop_running_.load()) {
        message_loop_running_ = false;
        if (message_loop_thread_.joinable()) {
            message_loop_thread_.join();
        }
        RCLCPP_INFO(logger_, "已停止MQTT消息接收循环");
    }
}

void MQTTClient::resubscribe_topics() {
    // 检查连接状态
    if (status_ != ConnectionStatus::CONNECTED) {
        RCLCPP_WARN(logger_, "MQTT未连接，无法重新订阅主题");
        return;
    }
    
    std::lock_guard<std::mutex> lock(subscribed_topics_mutex_);
    
    if (subscribed_topics_.empty()) {
        RCLCPP_DEBUG(logger_, "没有需要重新订阅的主题");
        return;
    }
    
    RCLCPP_INFO(logger_, "开始重新订阅 %zu 个主题...", subscribed_topics_.size());
    
    int success_count = 0;
    int fail_count = 0;
    
    for (const auto& sub : subscribed_topics_) {
        // 重试机制：最多重试3次
        bool subscribe_success = false;
        for (int retry = 0; retry < 3 && !subscribe_success; retry++) {
            // 每次重试前检查连接状态
            if (status_ != ConnectionStatus::CONNECTED) {
                RCLCPP_WARN(logger_, "连接已断开，停止重新订阅");
                fail_count++;
                break;
            }
            
        try {
            auto token = client_->subscribe(sub.topic, sub.qos);
            
                // 等待订阅完成（最多等待5秒）
                if (token->wait_for(std::chrono::seconds(5))) {
                    RCLCPP_INFO(logger_, "重新订阅成功: %s (QoS: %d)%s", 
                               sub.topic.c_str(), sub.qos,
                               retry > 0 ? (" (重试第" + std::to_string(retry) + "次)").c_str() : "");
                success_count++;
                    subscribe_success = true;
                } else {
                    if (retry < 2) {
                        RCLCPP_WARN(logger_, "重新订阅超时: %s，%d秒后重试...", 
                                   sub.topic.c_str(), retry + 1);
                        std::this_thread::sleep_for(std::chrono::seconds(retry + 1));
            } else {
                        RCLCPP_ERROR(logger_, "重新订阅超时: %s (已重试3次)", sub.topic.c_str());
                fail_count++;
                    }
            }
        } catch (const mqtt::exception& exc) {
                std::string error_msg = exc.what();
                // 检查是否是缓冲区满的错误
                if (error_msg.find("buffered") != std::string::npos || 
                    error_msg.find("-12") != std::string::npos) {
                    if (retry < 2) {
                        RCLCPP_WARN(logger_, "重新订阅失败（缓冲区满）: %s，%d秒后重试...", 
                                   sub.topic.c_str(), retry + 1);
                        std::this_thread::sleep_for(std::chrono::seconds(retry + 1));
                    } else {
                        RCLCPP_ERROR(logger_, "重新订阅失败（缓冲区满）: %s (已重试3次)", sub.topic.c_str());
                        fail_count++;
                    }
                } else {
            RCLCPP_ERROR(logger_, "重新订阅主题 %s 失败: %s", sub.topic.c_str(), exc.what());
            fail_count++;
                    break;  // 非缓冲区错误，不重试
                }
            }
        }
    }
    
    RCLCPP_INFO(logger_, "重新订阅完成: 成功 %d 个, 失败 %d 个", success_count, fail_count);
}

// 注意：已移除回调类实现，使用同步方式避免纯虚函数问题