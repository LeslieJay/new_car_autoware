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
#include "can_driver/usbcan_parser.hpp"
#include "can_driver/usbcan_battery.hpp"
#include "can_driver/can_receiver.hpp"
#include "can_driver/can_node.hpp"
#include "can_driver/can_send.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include "can_driver/hook_action_server.hpp" 
#include <rclcpp/executors.hpp>
namespace fs = std::filesystem;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;
using namespace std::chrono;
using namespace can_driver;
/* ---------- 全局变量 ---------- */

Battery::BatteryInfo battery_info_; // 电池的状态信息
std::mutex que_mtx_;
std::shared_ptr<can_driver::CanSend> send_queue_ = nullptr;
// 1正在起升，2起升到顶，0无动作-1正在下降，-2下降到位
int hook;
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
    rclcpp::init(argc, argv);
    send_queue_ = std::make_shared<can_driver::CanSend>();
    auto parser = std::make_shared<CANParser>();
    auto battery = std::make_shared<Battery>(parser);


    /* 1. 信号处理 */
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    // if(!usbcan_init()){
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("Usbcan"), "Initialize CAN failed!");
    //     return 0;
    // }
    // std::string logid = "default_logid";
    // struct passwd *pwd = getpwuid(getuid());
    // std::string baseDir = std::string(pwd->pw_dir) + logid + "/usbcan_log";
    // std::string logDir  = generateLogDirectory(baseDir);
    // if (!createLogDirectory(logDir) || !setLogDirectory(logDir)) {
    //     return 1;
    // }

    /* 4. ROS 初始化 */
    auto can_node = std::make_shared<can_driver::CanNode>();
    auto fork_server = std::make_shared<ForkActionServer>();
    /* 5. 接口配置 */
    // can接口名，getCan0InterfaceName是自己写的函数，返回一个类变量值，在构造函数中初始化，这里直接写interface0 = "can0"是一样的效果，can0_use同理，可以直接can0_use=true
    std::string interface0 = can_node->getCan0InterfaceName();
    std::string interface1 = can_node->getCan1InterfaceName();
    bool can0_use = can_node->getCan0UseStatus();
    bool can1_use = can_node->getCan1UseStatus();

    int socket0 = -1, socket1 = -1;
    // 创建并绑定套接字到can
    if (can0_use) can0_use = CanNode::initialize(interface0, socket0);
    if (can1_use) can1_use = CanNode::initialize(interface1, socket1);

    /* 6. 线程容器 */
    std::vector<std::thread> receive_threads, send_threads;

    /* 6-1 提前准备两个发送器（每个 CAN 口一个） */
    std::shared_ptr<CanSend> sender0 = nullptr;
    std::shared_ptr<CanSend> sender1 = nullptr;
    /* 6-2 lambda：统一创建接收器/发送器/线程 */
    // [&]捕获列表，通过引用的方式使用外部变量，create_interface类似函数名，通过它调用函数
    auto create_interface = [&](int socket,
                                const std::string& if_name,
                                const std::vector<uint32_t>& filter_ids,
                                std::shared_ptr<CanSend>& out_sender) -> void
    {
        if (socket < 0) return;
        // 自定义的类，用来向can总线发送消息
        if (!out_sender) out_sender = std::make_shared<CanSend>();
        // 自定义的类，用来接收can总线消息
        auto receiver = std::make_shared<CanReceiver>(can_node);
        // 根本用不上
        g_senders.emplace_back(out_sender);
        g_receivers.emplace_back(receiver);
        g_socket_handles.push_back(socket);

        // // 验证是否能设置过滤条件，如果能，在setCanFilter函数内就会设置，以后所有从该socket来的帧，都会被这个过滤规则过滤
        // if (!receiver->setCanFilter(socket, filter_ids, false)) {
        //     RCLCPP_WARN(can_node->get_logger(),
        //                 "[%s] setCanFilter failed", if_name.c_str());
        // }
        // 具体发送和接收报文的处理逻辑 
        // emplace_back与push_back的优势相比，就是可以直接在（）里面构造实例，这里面相当于std::thread t(&CanReceiver::receiveTask, receiver.get(), socket, if_name);
        // 线程thread的参数，第一个是要执行的函数，如果是类成员函数，第二个必须是类实例的指针，如此才能调用类成员函数。receiver是receiver 是一个智能指针对象，有自己的方法（get()、reset()等。receiver.get() 返回的是一个指向 CanReceiver 对象的指针
        // 这行代码执行时，线程立即开始创建，异步执行，不会阻塞主线程，在具体函数里跑循环，持续监听
        // sendTask只做将帧推到can总线的工作，整个代码中任何位置都可能执行将帧放进等待推到总线的vector中
        receive_threads.emplace_back(&CanReceiver::receiveTask,
                                     receiver.get(), socket, if_name);
        send_threads.emplace_back(&CanSend::sendTask,
                                  send_queue_.get(), socket, std::ref(g_running));
    };

    /* 6-3 实际启动接口 */
    create_interface(socket0, interface0, {},               sender0);
    create_interface(socket1, interface1,
                     {0x488,0x489,0x184,0x188,0x288, 0x181, 0x201,0x401, 0x221,
                      0x388,0x701,0x708,0x709,0x702,0x704,0x264,0x304,0x404},
                     sender1);



    /* 若需要绑定到 sender1，同理处理 */

    /* 8. 使用多线程执行器同时运行两个节点 */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(can_node);
    executor.add_node(fork_server);
    executor.spin();   
    // 阻塞，直到 SIGINT 使 rclcpp 关闭
    g_running = false;

    for (auto& t : receive_threads) if (t.joinable()) t.join();
    for (auto& t : send_threads)    if (t.joinable()) t.join();
    /* 现在才安全地关闭 socket */
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
