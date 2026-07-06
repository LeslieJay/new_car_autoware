// src/can_node.cpp
#include "can_driver/can_node.hpp"

#include <sys/stat.h>
#include <fstream>

using namespace std::chrono_literals;

// 在命名空间 can_driver 内定义类
namespace can_driver
{

    struct CanFrame
    {
        uint32_t frameId;   // 帧id
        uint8_t dataLen{0}; // 帧数据包长度
        uint8_t data[8]{0}; // 存放can帧的数据包缓冲区
    };

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

        // =============== 检查接口是否存在 ===============
        // if (!checkCanInterfaceExists(interface_))
        // {
        //     RCLCPP_ERROR(this->get_logger(), " CAN interface '%s' does not exist!", interface_.c_str());
        // }
        // else
        // {
        //     setupCanInterface(interface_, bitrate_,txqueuelen_);
        // }

        // if (!checkCanInterfaceExists(interface1_))
        // {
        //     RCLCPP_ERROR(this->get_logger(), " CAN interface '%s' does not exist!", interface1_.c_str());
        // }
        // else
        // {
        //     setupCanInterface(interface1_, bitrate1_,txqueuelen_);
        // }
    }

    CanNode::~CanNode()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down CAN node...");
    }
    /**
     * @brief 检车设备中是否存在物理can接口
     *
     * @param interface_name can接口名
     * @return true
     * @return false
     */
    bool CanNode::checkCanInterfaceExists(const std::string &interface_name)
    {
        // 1. 检查接口是否存在
        std::string path = "/sys/class/net/" + interface_name;
        struct stat info;
        if (::stat(path.c_str(), &info) != 0)
        {
            return false; // 接口不存在
        }
        if (!(info.st_mode & S_IFDIR))
        {
            return false; // 不是目录
        }

        // 2. 检查是否为 CAN 类型 (ARPHRD_CAN = 280)
        std::string type_path = path + "/type";
        std::ifstream type_file(type_path);
        int type = 0;
        type_file >> type;
        if (type != 280)
        {
            return false; // 不是 CAN 接口
        }

        // 3. ✅ 关键：检查是否有底层硬件设备（排除 vcan）
        std::string device_path = path + "/device";
        if (::stat(device_path.c_str(), &info) != 0)
        {
            return false; // 没有 device 目录 → 很可能是虚拟接口（如 vcan）
        }

        return true; // 是真实物理 CAN 接口
    }
    /**
     * @brief 设置can接口的波特率
     *
     * @param interface can接口名
     * @param bitrate 设置的波特率值
     * @param handle socket返回值
     */
    void CanNode::setupCanInterface(const std::string &interface, int bitrate,int txqueuelen)
    {
        // 初始化 handle

        std::string cmd;
        int ret;

        // Step 1: 确保接口先关闭（关键！）
        cmd = "sudo ip link set " + interface + " down";
        ret = std::system(cmd.c_str());
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), " Failed to bring down %s", interface.c_str());
            return;
        }

        // Step 2: 配置比特率和自动重启
        cmd = "sudo ip link set " + interface + " type can bitrate " + std::to_string(bitrate) + " restart-ms 100";
        ret = std::system(cmd.c_str());
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), " Failed to configure bitrate for %s", interface.c_str());
            return;
        }
         
        // 设置发送队列大小
        cmd = "sudo ip link set " + interface + " txqueuelen " + std::to_string(txqueuelen) ;
        ret = std::system(cmd.c_str());
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), " Failed to configure txqueuelen for %s", interface.c_str());
            return;
        }
        // Step 3: 启动接口
        cmd = "sudo ip link set " + interface + " up";
        ret = std::system(cmd.c_str());
        if (ret != 0)
        {
            RCLCPP_ERROR(this->get_logger(), " Failed to bring up %s", interface.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully configured and opened %s (bitrate: %d bps)",
                    interface.c_str(), bitrate);
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

/* // =============== 主函数 ===============
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<can_driver::CanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} */