/*
 * @Author: du.xiaoying1
 * @Date: 2025-08-12 17:24:36
 * @LastEditors: dxy
 * @LastEditTime: 2025-08-19 11:29:48
 * @FilePath: /qr_agv_0627_r/src/can_driver/src/can_send.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by du.xiaoying1 , All Rights Reserved. 
 */
// src/can_send.cpp
#include "can_driver/can_send.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <sys/select.h>
#include <thread>
#include <unistd.h>

namespace can_driver
{

std::atomic<bool> charging_requested_{false};
std::atomic<bool> is_charging_mode_{false};
std::atomic<bool> is_battery_full_{false};
std::atomic<bool> is_charge_fault_{false};
std::atomic<bool> battery_fault_0x2a1_{false};
std::atomic<bool> charger_fault_0x1a1_{false};
std::atomic<bool> bms_fault_0x481_{false};

std::atomic<bool> is_voice_active_{false};
std::atomic<uint8_t> active_voice_id_{0};

static void apply0x401FrameRules(struct ::can_frame & frame)
{
    if ((frame.can_id & CAN_SFF_MASK) != 0x401U) {
        return;
    }
    // 语音守护原则：
    // 当存在有效播报中的语音时，严格守护 0x401 发送报文：
    // 强制保持语音模块开关为开 (Byte0 Bit4 = 1)，保持语音编号为当前播报编号 (Byte2 = active_voice_id)
    // 绝不允许任何发送路径下发关闭语音或覆盖编号的帧打断当前播放！
    if (is_voice_active_.load(std::memory_order_relaxed)) {
        frame.data[0] |= (1 << 4);
        frame.data[2] = active_voice_id_.load(std::memory_order_relaxed);
    }
}

static void applyControlFrameRules(struct ::can_frame & frame)
{
    if ((frame.can_id & CAN_SFF_MASK) != 0x201U) {
        return;
    }

    const bool is_full = is_battery_full_.load(std::memory_order_relaxed);
    const bool is_fault = is_charge_fault_.load(std::memory_order_relaxed);
    const bool charging_req = charging_requested_.load(std::memory_order_relaxed);
    const bool in_charge_mode = is_charging_mode_.load(std::memory_order_relaxed);

    // 核心安全原则：
    // 只要收到充电请求 (charging_req) 或车辆整车处于充电模式 (in_charge_mode)，
    // 必须确保全部使能断掉，并且速度和转向都是0，绝不能有一帧带速度转向和使能的下发！
    if (charging_req || in_charge_mode) {
        frame.data[0] = 0; // 速度指令低8位 = 0
        frame.data[1] = 0; // 速度指令高8位 = 0
        frame.data[2] = 0; // 转向指令低8位 = 0
        frame.data[3] = 0; // 转向指令高8位 = 0
        frame.data[5] = 0; // 加速度 = 0
        frame.data[6] = 0; // 减速度 = 0
        frame.data[7] = 0;

        // Byte4 使能位严格受控：
        // Bit0: 导航定位控制器控制器_AllOK = 1
        // Bit1: EnableSteer = 0 (严禁使能转向)
        // Bit2: 0
        // Bit3: EnableDrive = 0 (严禁使能驱动)
        // Bit4: brakeoff = 0 (严禁松开抱闸，必须处于机械抱闸状态)
        // Bit5: Charing_Enable: 0：停止充电，1：开始充电
        //       在充电请求有效期间 (charging_req=true)，且未充满、无故障时，持续保持 Bit5=1 以维系底盘VCU充电；
        //       只有在未请求充电 (charging_req=false，如收到stop_charge)、充满或故障时，Bit5 才置为 0。
        if (charging_req && !is_full && !is_fault) {
            frame.data[4] = 0b00100001; // Bit0=1, Bit5=1 (持续维持充电使能)
        } else {
            frame.data[4] = 0b00000001; // Bit0=1, Bit5=0 (停止充电，保持抱闸与使能断开)
        }
    } else {
        // 非充电模式且无充电请求下，无论何处生成的控制帧，强制清除 Bit5 充电使能，确保任何地方都不能再下发充电帧
        frame.data[4] &= ~(1 << 5);
    }
}

void CanSend::push(const std::vector<struct ::can_frame> & frames) {
    
    std::lock_guard<std::mutex> lock(mtx_);
   
    for (auto frame : frames) {
        if ((frame.can_id & CAN_SFF_MASK) == 0x201U) {
            applyControlFrameRules(frame);
            latest_control_frame_ = frame;
            has_latest_control_frame_ = true;
        } else {
            queue_.push(frame);
        }
    }
}

void CanSend::setLatestControlFrame(const struct ::can_frame & frame)
{
    struct ::can_frame f = frame;
    applyControlFrameRules(f);
    std::lock_guard<std::mutex> lock(mtx_);
    latest_control_frame_ = f;
    has_latest_control_frame_ = true;
}

void CanSend::setControlFramePeriodMs(const int period_ms)
{
    control_frame_period_ms_.store(std::max(period_ms, 1), std::memory_order_relaxed);
}

std::vector<::can_frame> CanSend::pop_all()
{
    std::queue<::can_frame> tmp;
    {
        std::lock_guard<std::mutex> lk(mtx_);
        tmp.swap(queue_);   // 整个队列搬走，O(1)
    }

    std::vector<::can_frame> res;
    res.reserve(tmp.size());
    while (!tmp.empty()) {
        res.emplace_back(tmp.front());
        tmp.pop();
    }
    return res;
}

bool CanSend::empty() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return queue_.empty();
}
void CanSend::sendTask(int socket,  std::atomic<bool> &running)
{
    SendOptions opt;
    opt.timeout_per_frame_ms = 50;
    opt.inter_frame_delay_ms = 0;

    auto next_send_time = std::chrono::steady_clock::now();
    while (running)
    {
        std::vector<::can_frame> frames;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            // Stateful control is coalesced: only the newest 0x201 is sent each cycle.
            if (has_latest_control_frame_) {
                applyControlFrameRules(latest_control_frame_);
                frames.push_back(latest_control_frame_);
            }
            // Event frames retain FIFO ordering and are sent after the control heartbeat.
            while (!queue_.empty()) {
                auto qf = queue_.front();
                queue_.pop();
                applyControlFrameRules(qf);
                apply0x401FrameRules(qf);
                frames.push_back(qf);
            }
        }

        if (!frames.empty())
        {
            sendFrames(socket, frames, opt);
        }

        const auto period = std::chrono::milliseconds(
          control_frame_period_ms_.load(std::memory_order_relaxed));
        next_send_time += period;
        const auto now = std::chrono::steady_clock::now();
        if (next_send_time <= now) {
            next_send_time = now + period;
        }
        std::this_thread::sleep_until(next_send_time);
    }
}

bool CanSend::sendFrames(
    int socket_handle,
    const std::vector<struct ::can_frame> & frames,
    const SendOptions & options)
{
    if (socket_handle < 0) {
        std::cerr << "Invalid socket handle in sendFrames." << std::endl;
        return false;
    }

    if (frames.empty()) {
        return true;
    }

    bool all_sent = true;
    int frame_index = 0;

    for (auto frame : frames) {
        applyControlFrameRules(frame);
        fd_set write_fds;
        struct timeval tv;

        FD_ZERO(&write_fds);
        FD_SET(socket_handle, &write_fds);

        tv.tv_sec = options.timeout_per_frame_ms / 1000;
        tv.tv_usec = (options.timeout_per_frame_ms % 1000) * 1000;

        const int ret = select(socket_handle + 1, nullptr, &write_fds, nullptr, &tv);
        if (ret <= 0) {
            all_sent = false;
            if (options.stop_on_error) {
                return false;
            }
            ++frame_index;
            continue;
        }

        const ssize_t bytes_sent = write(socket_handle, &frame, sizeof(frame));
        if (bytes_sent != sizeof(frame)) {
            std::cerr << "Failed to send frame[" << frame_index
                      << "]: ID=0x" << std::hex << frame.can_id
                      << ", Sent=" << bytes_sent << ", Expected=" << sizeof(frame)
                      << ", Error: " << std::strerror(errno) << std::dec << std::endl;
            all_sent = false;
            if (options.stop_on_error) {
                return false;
            }
        }

        if (options.inter_frame_delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.inter_frame_delay_ms));
        }

        ++frame_index;
    }

    return all_sent;
}

}  // namespace can_driver
