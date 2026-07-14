#include <atomic>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <unistd.h>

#include <rclcpp/executors.hpp>

#include "can_driver/can_node.hpp"
#include "can_driver/can_receiver.hpp"
#include "can_driver/can_send.hpp"
#include "can_driver/hook_action_server.hpp"

namespace can_driver
{
std::shared_ptr<CanSend> send_queue_ = nullptr;
}  // namespace can_driver

// 1: 正在起升, 2: 起升到顶, 0: 无动作, -1: 正在下降, -2: 下降到位
int hook = 0;
std::vector<std::shared_ptr<can_driver::CanReceiver>> g_receivers;
std::vector<int> g_socket_handles;
std::atomic<bool> g_running{true};

void signalHandler(int signum)
{
  std::cout << "\nReceived signal " << signum << ". Shutting down..." << std::endl;
  g_running = false;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  can_driver::send_queue_ = std::make_shared<can_driver::CanSend>();

  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  auto can_node = std::make_shared<can_driver::CanNode>();
  auto fork_server = std::make_shared<can_driver::ForkActionServer>();

  const std::string interface0 = can_node->getCan0InterfaceName();
  const std::string interface1 = can_node->getCan1InterfaceName();
  bool can0_use = can_node->getCan0UseStatus();
  bool can1_use = can_node->getCan1UseStatus();

  int socket0 = -1;
  int socket1 = -1;
  if (can0_use) {
    can0_use = can_driver::CanNode::initialize(interface0, socket0);
  }
  if (can1_use) {
    can1_use = can_driver::CanNode::initialize(interface1, socket1);
  }

  std::vector<std::thread> receive_threads;
  std::vector<std::thread> send_threads;
  auto create_interface = [&](const int socket, const std::string & if_name) {
    if (socket < 0) {
      return;
    }

    auto receiver = std::make_shared<can_driver::CanReceiver>(can_node);
    g_receivers.emplace_back(receiver);
    g_socket_handles.push_back(socket);

    receive_threads.emplace_back(&can_driver::CanReceiver::receiveTask, receiver.get(), socket, if_name);
    send_threads.emplace_back(
      &can_driver::CanSend::sendTask, can_driver::send_queue_.get(), socket, std::ref(g_running));
  };

  create_interface(socket0, interface0);
  create_interface(socket1, interface1);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(can_node);
  executor.add_node(fork_server);
  executor.spin();

  g_running = false;
  for (auto & t : receive_threads) {
    if (t.joinable()) {
      t.join();
    }
  }
  for (auto & t : send_threads) {
    if (t.joinable()) {
      t.join();
    }
  }
  for (int & handle : g_socket_handles) {
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
