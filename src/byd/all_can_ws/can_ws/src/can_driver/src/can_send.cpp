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

void CanSend::push(const std::vector<struct ::can_frame> & frames) {
    
    std::lock_guard<std::mutex> lock(mtx_);
   
    for (const auto& frame : frames) {
        if ((frame.can_id & CAN_SFF_MASK) == 0x201U) {
            latest_control_frame_ = frame;
            has_latest_control_frame_ = true;
        } else {
            queue_.push(frame);
        }
    }
}

void CanSend::setLatestControlFrame(const struct ::can_frame & frame)
{
    std::lock_guard<std::mutex> lock(mtx_);
    latest_control_frame_ = frame;
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
                frames.push_back(latest_control_frame_);
            }
            // Event frames retain FIFO ordering and are sent after the control heartbeat.
            while (!queue_.empty()) {
                frames.push_back(queue_.front());
                queue_.pop();
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

    for (const auto & frame : frames) {
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
