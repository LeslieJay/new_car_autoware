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

using namespace can_driver;
using namespace std;

void CanSend::push(const std::vector<struct ::can_frame>& frames) {
    
    std::lock_guard<std::mutex> lock(mtx_);
   
    for (const auto& frame : frames) {
        queue_.push(frame);
    }
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

    while (running)
    {
        // 将CanSend类变量的所有帧取出来
        auto frames = pop_all();
        
        // std::cout << "sendTask" <<frames.size()<< std::endl;
        //测试使用
        // #ifndef DEBUG 
        // if (frames.empty())  std::cout<<"指令队列为空！！"<<std::endl;
        //#endif
        if (!frames.empty())
        {
            // std::cout<<"指令队列大小为： "<<frames.size()<<std::endl;
            sendFrames(socket, frames, opt);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }
}

bool CanSend::sendFrames(
        int socket_handle,
        const std::vector<struct ::can_frame> &frames,
        const SendOptions &options ) // 默认参数
    {
        if (socket_handle < 0)
        {
            std::cerr << "Invalid socket handle in sendFrames." << std::endl;
            return false;
        }

        if (frames.empty())
        {
            return true; // 空列表，视为成功
        }

        bool all_sent = true;
        int frame_index = 0;

        for (const auto &frame : frames)
        {
            // Step 1: 使用 select 等待 socket 可写（带超时）
            fd_set write_fds;
            struct timeval tv;

            FD_ZERO(&write_fds);
            FD_SET(socket_handle, &write_fds);

            tv.tv_sec = options.timeout_per_frame_ms / 1000;
            tv.tv_usec = (options.timeout_per_frame_ms % 1000) * 1000;

            int ret = select(socket_handle + 1, nullptr, &write_fds, nullptr, &tv);
            if (ret <= 0)
            {
                // std::cerr << " Socket not ready for writing (timeout or error) for frame["
                //           << frame_index << "], ID: 0x" << std::hex << frame.can_id << std::dec << std::endl;
                all_sent = false;
                if (options.stop_on_error)
                {
                    return false;
                }
                frame_index++;
                continue;
            }

            // Step 2: 调用 write 发送
            ssize_t bytes_sent = write(socket_handle, &frame, sizeof(frame));
            // std::cout<<"sizeof(frame)："<<sizeof(frame)<<std::endl;
            // std::cout<<"bytes_sent："<<bytes_sent<<std::endl;
            
            if (bytes_sent != sizeof(frame))
            {
                std::cerr << " Failed to send frame[" << frame_index
                          << "]: ID=0x" << std::hex << frame.can_id
                          << ", Sent=" << bytes_sent << ", Expected=" << sizeof(frame)
                          << ", Error: " << strerror(errno) << std::dec << std::endl;
                all_sent = false;
                if (options.stop_on_error)
                {
                    return false;
                }
            }
            else
            {
                // std::cout << " Sent frame[" << frame_index
                //           << "]: ID=0x" << std::hex << frame.can_id
                //           << ", DLC=" << std::dec << (int)frame.can_dlc << std::endl;
            }

            // Step 3: 帧间延迟
            if (options.inter_frame_delay_ms > 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(options.inter_frame_delay_ms));
            }

            frame_index++;
        }

        return all_sent;
    }
