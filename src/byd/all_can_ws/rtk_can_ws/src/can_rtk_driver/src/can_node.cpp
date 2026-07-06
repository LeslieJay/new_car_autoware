// src/can_node.cpp
#include "can_driver/can_node.hpp"

#include <sys/stat.h>
#include <fstream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>   // fcntl, F_GETFL, F_SETFL

using namespace std::chrono_literals;

namespace can_driver
{
CanNode::CanNode() : Node("can_rtk_node")
{
    RCLCPP_INFO(this->get_logger(), "Initializing CAN Node...");

    this->declare_parameter("interface", "can0");
    this->declare_parameter("bitrate", 500000);
    this->declare_parameter("can0_use", true);

    this->declare_parameter("interface1", "can1");
    this->declare_parameter("bitrate1", 500000);
    this->declare_parameter("can1_use", true);
    this->declare_parameter("txqueuelen", 1000);

    this->declare_parameter("heartbeat_base_id", 1);
    this->declare_parameter("heartbeat_period", 25);

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
    RCLCPP_INFO(this->get_logger(), "   CAN0: %s @ %d bps, enabled: %s",
                interface_.c_str(), bitrate_, can0_use_ ? "yes" : "no");
}

CanNode::~CanNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down CAN node...");
}

bool CanNode::checkCanInterfaceExists(const std::string &interface_name)
{
    std::string path = "/sys/class/net/" + interface_name;
    struct stat info;

    if (::stat(path.c_str(), &info) != 0)
        return false;

    if (!(info.st_mode & S_IFDIR))
        return false;

    std::string type_path = path + "/type";
    std::ifstream type_file(type_path);
    int type = 0;
    type_file >> type;

    if (type != 280)
        return false;

    std::string device_path = path + "/device";
    if (::stat(device_path.c_str(), &info) != 0)
        return false;

    return true;
}

void CanNode::setupCanInterface(const std::string &interface, int bitrate, int txqueuelen)
{
    std::string cmd;
    int ret;

    // 关闭接口
    cmd = "sudo ip link set " + interface + " down";
    ret = std::system(cmd.c_str());
    if (ret != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to bring down %s", interface.c_str());
        return;
    }

    // ⭐ CANFD 配置
    cmd = "sudo ip link set " + interface +
          " type can bitrate " + std::to_string(bitrate) +
          " dbitrate 2000000 fd on restart-ms 100";

    ret = std::system(cmd.c_str());
    if (ret != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure CANFD for %s", interface.c_str());
        return;
    }

    // 发送队列
    cmd = "sudo ip link set " + interface +
          " txqueuelen " + std::to_string(txqueuelen);

    std::system(cmd.c_str());

    // 启动接口
    cmd = "sudo ip link set " + interface + " up";
    ret = std::system(cmd.c_str());

    if (ret != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to bring up %s", interface.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Successfully configured %s bitrate=%d dbitrate=2000000 FD",
                interface.c_str(), bitrate);
}

bool CanNode::initialize(const std::string &interface_name, int &socket_handle)
{
    socket_handle = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_handle < 0) { perror("socket"); return false; }

    // 允许 CAN FD
    int enable_canfd = 1;
    if (setsockopt(socket_handle, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        perror("setsockopt CAN FD");
        return false;
    }

    // 接收所有 CAN ID
    struct can_filter filter;
    filter.can_id = 0;
    filter.can_mask = 0;
    if (setsockopt(socket_handle, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0)
    {
        perror("setsockopt CAN filter");
        return false;
    }

    struct ifreq ifr{};
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ-1);
    ifr.ifr_name[IFNAMSIZ-1] = '\0';

    if (ioctl(socket_handle, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("SIOCGIFINDEX");
        close(socket_handle);
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_handle, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        close(socket_handle);
        return false;
    }

    // int flags = fcntl(socket_handle, F_GETFL, 0);
    // fcntl(socket_handle, F_SETFL, flags | O_NONBLOCK);

    RCLCPP_INFO(rclcpp::get_logger("can_rtk_node"), "CANFD socket initialized: %s", interface_name.c_str());
    return true;
}

// ⭐ CANFD 读取
bool CanNode::readFrame(int socket_handle, CanFrame &frame)
{
    struct canfd_frame rx_frame;

    int nbytes = read(socket_handle, &rx_frame, sizeof(rx_frame));

    if (nbytes <= 0)
        return false;

    frame.frameId = rx_frame.can_id;
    frame.dataLen = rx_frame.len;

    memcpy(frame.data, rx_frame.data, rx_frame.len);

    return true;
}

} // namespace can_driver