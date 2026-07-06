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
#include <iostream>
#include <string>
#include <thread>
#include <bitset>
#include <atomic>
#include "rclcpp/rclcpp.hpp"
#include <cstring> // 用于memcpy
#include <queue>
#include <random>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdexcept>
#include <fstream>

namespace can_driver
{
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
    void push(const std::vector<struct can_frame> &frames);

    /**
     * @brief 取出并清空队列中所有待发送帧
     * @return 包含所有待发送帧的 vector
     */
    std::vector<struct can_frame> pop_all();

    /**
     * @brief 检查队列是否为空
     * @return true 表示队列为空
     */
    bool empty() const;

    static bool sendFrames(
        int socket_handle,
        const std::vector<struct can_frame> &frames,
        const SendOptions &options = SendOptions{});
    void sendTask(int socket,  std::atomic<bool> &running);

private:
    std::queue<struct can_frame> queue_;
    
    mutable std::mutex mtx_; // 保护队列的互斥锁
};
}