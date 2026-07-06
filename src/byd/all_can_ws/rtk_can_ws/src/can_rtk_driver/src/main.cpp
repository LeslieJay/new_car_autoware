// src/main.cpp
#include <iostream>
#include <csignal>
#include <thread>
#include <chrono>
#include <vector>
#include <memory>
#include <iomanip>
#include <sstream>
#include <pwd.h>
#include <unistd.h>
#include <ctime>
#include <cstdlib>
#include <sys/stat.h>
#include <sys/types.h>
#include <filesystem>

#include "can_driver/can_receiver.hpp"
#include "can_driver/can_node.hpp"
#include "can_driver/can_send.hpp"
#include "can_driver/can_battery.hpp"

namespace fs = std::filesystem;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
using namespace std::chrono;
using namespace can_driver;

/* ---------- 全局变量 ---------- */
std::vector<std::shared_ptr<CanSend>>       g_senders;
std::vector<std::shared_ptr<CanReceiver>>   g_receivers;
std::vector<int>                            g_socket_handles;
std::atomic<bool>                           g_running{true};

/* ---------- 信号处理 ---------- */
void signalHandler(int signum) {
    std::cout << "\nReceived signal " << signum << ". Shutting down..." << std::endl;
    g_running = false;          // 只做通知
    /* 这里不再 close socket，让主线程统一关 */
}

/* ---------- 日志目录相关 ---------- */
std::string generateLogDirectory(const std::string &baseDir) {
    std::time_t t = std::time(nullptr);
    char timeBuffer[20];
    std::strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d_%H-%M-%S", std::localtime(&t));
    return baseDir + "/log_" + std::string(timeBuffer);
}

bool createLogDirectory(const std::string &logDir) {
    try {
        fs::create_directories(logDir);
    } catch (const fs::filesystem_error &e) {
        std::cerr << "Failed to create log directory: " << logDir << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

bool setLogDirectory(const std::string &logDir) {
    if (setenv("ROS_LOG_DIR", logDir.c_str(), 1) != 0) {
        std::cerr << "Failed to set ROS_LOG_DIR environment variable." << std::endl;
        return false;
    }
    return true;
}

void cleanupOldLogs(const std::string &baseDir) {
    std::vector<fs::path> logFiles;
    for (const auto &entry : fs::directory_iterator(baseDir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".log") {
            logFiles.push_back(entry.path());
        }
    }

    std::sort(logFiles.begin(), logFiles.end(), [](const fs::path &a, const fs::path &b) {
        return fs::last_write_time(a) > fs::last_write_time(b);
    });

    for (size_t i = 15; i < logFiles.size(); ++i) {
        fs::remove(logFiles[i]);
    }
}

/* ---------- main ---------- */
int main(int argc, char *argv[])
{
    /* 1. 信号处理 */
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    /* 4. ROS 初始化 */
    rclcpp::init(argc, argv);
    auto can_rtk_node = std::make_shared<can_driver::CanNode>();

    /* 5. 接口配置 */
    std::string interface0 = can_rtk_node->getCan0InterfaceName();
    std::string interface1 = can_rtk_node->getCan1InterfaceName();
    bool can0_use = can_rtk_node->getCan0UseStatus();
    bool can1_use = can_rtk_node->getCan1UseStatus();

    int socket0 = -1, socket1 = -1;
    if (can0_use) can0_use = CanNode::initialize(interface0, socket0);
    if (can1_use) can1_use = CanNode::initialize(interface1, socket1);

    /* 6. 线程容器 */
    std::vector<std::thread> receive_threads, send_threads;

    /* 6-1 提前准备两个发送器（每个 CAN 口一个） */
    std::shared_ptr<CanSend> sender0 = nullptr;
    std::shared_ptr<CanSend> sender1 = nullptr;

    /* 6-2 lambda：创建接收器/发送器/线程 */
    auto create_interface = [&](int socket,
                            const std::string& if_name,
                            const std::vector<uint32_t>& filter_ids,
                            std::shared_ptr<CanSend>& out_sender)
{
    if (socket < 0) return;

    if (!out_sender) out_sender = std::make_shared<CanSend>();
    auto receiver = std::make_shared<CanReceiver>(can_rtk_node, out_sender);

    
    g_senders.emplace_back(out_sender);
    g_receivers.emplace_back(receiver);
    g_socket_handles.push_back(socket);

    receive_threads.emplace_back(&CanReceiver::receiveTask,
                                 receiver.get(), socket, if_name);

    // 多线程发布
    send_threads.emplace_back(&CanReceiver::publishNavSatFixTask, receiver.get());
    send_threads.emplace_back(&CanReceiver::publishGnssInsTask, receiver.get());
};

    /* 6-3 实际启动接口 */
    create_interface(socket0, interface0, {}, sender0);
    // create_interface(socket1, interface1,
    //                  {0x488,0x489,0x184,0x188,0x288,0x181,0x201,0x401,0x221,
    //                   0x388,0x701,0x708,0x709,0x702,0x704,0x264,0x304,0x404},
    //                  sender1);


    /* 8. 主循环 */
    rclcpp::spin(can_rtk_node);
    g_running = false;

    /* 9. 等待线程退出 */
    for (auto& t : receive_threads) if (t.joinable()) t.join();
    for (auto& t : send_threads)    if (t.joinable()) t.join();

    /* 10. 关闭 socket */
    for (int &handle : g_socket_handles) {
        if (handle >= 0) {
            std::cout << "Closing socket handle: " << handle << std::endl;
            ::close(handle);
            handle = -1;
        }
    }

    rclcpp::shutdown();
    std::cout << "ROS 2 shutdown. Program exiting." << std::endl;
    return 0;
}
