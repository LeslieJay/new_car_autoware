// src/can_node.cpp
#include "can_driver/can_node.hpp"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std::chrono_literals;

// 在命名空间 can_driver 内定义类
namespace can_driver
{

    CanNode::CanNode() : Node("can_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing CAN Node...");

        // =============== 声明所有参数 ===============
        this->declare_parameter("interface", "can0");
        this->declare_parameter("bitrate", 250000);
        this->declare_parameter("can0_use", true);

        this->declare_parameter("interface1", "can1");
        this->declare_parameter("bitrate1", 250000);
        this->declare_parameter("can1_use", false);
        this->declare_parameter("txqueuelen", 1000);

        this->declare_parameter("heartbeat_base_id", 1);
        this->declare_parameter("heartbeat_period", 25);

        // =============== 获取参数值 ===============
        interface_ = this->get_parameter("interface").as_string();
        bitrate_ = this->get_parameter("bitrate").as_int();
        can0_use_ = this->get_parameter("can0_use").as_bool();

        interface1_ = this->get_parameter("interface1").as_string();
        bitrate1_ = this->get_parameter("bitrate1").as_int();
        can1_use_ = this->get_parameter("can1_use").as_bool();
        txqueuelen_ = this->get_parameter("txqueuelen").as_int();

        heartbeat_base_id_ = this->get_parameter("heartbeat_base_id").as_int();
        heartbeat_period_ms_ = this->get_parameter("heartbeat_period").as_int();

        RCLCPP_INFO(this->get_logger(), " CAN Node started with parameters:");
        RCLCPP_INFO(this->get_logger(), "   CAN0: %s @ %d bps, enabled: %s", interface_.c_str(), bitrate_, can0_use_ ? "yes" : "no");
        RCLCPP_INFO(this->get_logger(), "   CAN1: %s @ %d bps, enabled: %s", interface1_.c_str(), bitrate1_, can1_use_ ? "yes" : "no");
        RCLCPP_INFO(this->get_logger(), "   Heartbeat: base ID=0x%X, period=%d ms", heartbeat_base_id_, heartbeat_period_ms_);

        if (can0_use_) {
            configureTxQueueLength(interface_);
        }
        if (can1_use_) {
            configureTxQueueLength(interface1_);
        }
    }

    CanNode::~CanNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down CAN node...");
    }

    void CanNode::configureTxQueueLength(const std::string & interface_name) const
    {
        std::ostringstream command;
        command << "ip link set dev " << interface_name << " txqueuelen " << txqueuelen_;
        const int ret = std::system(command.str().c_str());
        if (ret != 0) {
            RCLCPP_WARN(
              this->get_logger(),
              "Failed to set txqueuelen for %s (cmd: '%s'). Please ensure runtime has NET_ADMIN privilege.",
              interface_name.c_str(), command.str().c_str());
            return;
        }

        RCLCPP_INFO(
          this->get_logger(), "Configured %s txqueuelen=%d", interface_name.c_str(), txqueuelen_);
    }

    bool CanNode::initialize(const std::string &interface_name, int &socket_handle)
    {
        // 1. 创建 socket
        socket_handle = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_handle < 0)
        {
            perror("socket");
            return false;
        }

        // 2. 获取接口索引
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0'; // 确保字符串以 null 结尾

        if (::ioctl(socket_handle, SIOCGIFINDEX, &ifr) < 0)
        {
            perror("SIOCGIFINDEX");
            ::close(socket_handle); // 创建失败，立即关闭 socket
            socket_handle = -1;     // 确保返回无效值
            return false;
        }

        // 3. 绑定 socket 到接口
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (::bind(socket_handle, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("bind");
            ::close(socket_handle);
            socket_handle = -1;
            return false;
        }

        std::cout << "CanReceiver initialized successfully for interface: " << interface_name << std::endl;
        return true; // 成功，socket_handle 已被设置
    }

} // namespace can_driver