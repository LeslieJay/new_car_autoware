/*
 * @Author: du.xiaoying1
 * @Date: 2025-08-13 09:42:10
 * @LastEditors: dxy
 * @LastEditTime: 2025-08-19 13:17:04
 * @FilePath: /qr_agv_0627_r/src/can_driver/include/can_driver/can_node.hpp
 * @Description:
 *
 * Copyright (c) 2025 by du.xiaoying1 , All Rights Reserved.
 */
// include/can_driver/can_node.hpp
#ifndef CAN_DRIVER__CAN_NODE_HPP_
#define CAN_DRIVER__CAN_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace can_driver
{
    class CanNode : public rclcpp::Node
    {
    public:
        CanNode();
        ~CanNode();
        // 获取 can0 的信息
        std::string getCan0InterfaceName() const { return interface_; }
        bool getCan0UseStatus() const { return can0_use_; }

        // 获取 can1 的信息
        std::string getCan1InterfaceName() const { return interface1_; }
        bool getCan1UseStatus() const { return can1_use_; }

        /**
         * 静态函数：初始化 CAN 接口，返回 socket 句柄
         * @param interface_name CAN 接口名，如 "can0"
         * @param socket_handle 输出参数，成功时返回有效的 socket fd
         * @return 成功返回 true，失败返回 false
         */
        static bool initialize(const std::string &interface_name, int &socket_handle);

    private:
        void configureTxQueueLength(const std::string & interface_name) const;

        // 成员变量
        std::string interface_;
        int bitrate_;
        bool can0_use_;

        std::string interface1_;
        int bitrate1_;
        bool can1_use_;

        int heartbeat_base_id_;
        int heartbeat_period_ms_;
        int txqueuelen_;
    };

} // namespace can_driver

#endif // CAN_DRIVER__CAN_NODE_HPP_
