/*
 * @Author: du.xiaoying1
 * @Date: 2025-08-12 17:24:07
 * @LastEditors: dxy
 * @LastEditTime: 2025-08-14 16:40:31
 * @FilePath: /qr_agv_0627_r/src/can_driver/include/can_driver/can_send.hpp
 * @Description:
 *
 * Copyright (c) 2025 by du.xiaoying1 , All Rights Reserved.
 */
// src/can_send.hpp
#pragma once
#include <atomic>
#include <chrono>
#include <mutex>
#include <queue>
#include <vector>

#include <linux/can.h>

namespace can_driver
{
// 充电控制与监测相关的全局原子变量
extern std::atomic<bool> charging_requested_;    // 是否处于充电请求/充电中状态
extern std::atomic<bool> is_charging_mode_;       // 底盘0x2A1反馈是否已进入充电模式 (Byte5 == 3)
extern std::atomic<bool> is_battery_full_;        // 电池是否已充满 (SOC >= 100%)
extern std::atomic<bool> is_charge_fault_;        // 是否存在充电/电池故障
extern std::atomic<bool> battery_fault_0x2a1_;    // 0x2A1 电池状态反馈故障 (Byte5 == 1)
extern std::atomic<bool> charger_fault_0x1a1_;    // 0x1A1 充电机故障
extern std::atomic<bool> bms_fault_0x481_;        // 0x481 BMS故障码/超时

// 语音控制与监测相关的全局原子变量
extern std::atomic<bool> is_voice_active_;        // 当前是否有语音正在播报中（禁止打断）
extern std::atomic<uint8_t> active_voice_id_;     // 当前正在播报的语音编号

struct SendOptions
    {
        int timeout_per_frame_ms = 100; // 每帧超时
        bool stop_on_error = false;     // 出错是否立即返回
        int inter_frame_delay_ms = 1;   // 帧间延迟
    };
    
/**
 * @brief 线程安全的 CAN 发送队列管理类
 * 所有业务模块通过 push() 提交帧，由统一发送线程 pop_all() 发送
 */
class CanSend
{
public:
    /**
     * @brief 批量提交 CAN 帧到发送队列
     * 支持单帧（vector 大小为1）或多帧提交
     * @param frames 待发送的 CAN 帧列表
     */
    void push(const std::vector<struct ::can_frame> &frames);

    /// Replace the stateful 0x201 control frame sent periodically by sendTask().
    void setLatestControlFrame(const struct ::can_frame & frame);

    /// Configure the periodic control-frame interval (20 ms for the VCU protocol).
    void setControlFramePeriodMs(int period_ms);

    /**
     * @brief 取出并清空队列中所有待发送帧
     * @return 包含所有待发送帧的 vector
     */
    std::vector<struct ::can_frame> pop_all();

    /**
     * @brief 检查队列是否为空
     * @return true 表示队列为空
     */
    bool empty() const;

    static bool sendFrames(
        int socket_handle,
        const std::vector<struct ::can_frame> &frames,
        const SendOptions &options = SendOptions{});
    void sendTask(int socket,  std::atomic<bool> &running);

private:
    std::queue<struct ::can_frame> queue_;
    struct ::can_frame latest_control_frame_{};
    bool has_latest_control_frame_{false};
    std::atomic<int> control_frame_period_ms_{20};
    
    mutable std::mutex mtx_; // 保护队列的互斥锁
};
}
