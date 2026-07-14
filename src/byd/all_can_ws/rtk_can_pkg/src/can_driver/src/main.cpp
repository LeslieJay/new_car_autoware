#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "can_driver/can_node.hpp"
#include "can_driver/can_receiver.hpp"
#include "can_driver/sixents_sdk.h"
#include "can_driver/sixents_types.h"

using namespace can_driver;

namespace
{
constexpr int kCaMaxLen = 3000;
constexpr const char * kSixMountPoint = "RTCM32_GRECJ2";
constexpr const char * kSixAk = "4734542738";
constexpr const char * kSixAs = "fb0vwkac16a5gyqziya6d385rwdg1z75qmjdy66gbzey57k19debgnwnt3v0vkf0";
constexpr const char * kSixDevId = "1421225047560";
constexpr const char * kSixDevType = "SDK";
constexpr int kSixRtcmPort = 4405;
constexpr const char * kCanFdIf = "can1";
constexpr canid_t kCanFdId = 0x611;
}  // namespace

int g_canfd_socket = -1;
std::atomic<bool> g_six_sdk_running{false};

std::vector<std::shared_ptr<CanReceiver>> g_receivers;
std::vector<int> g_socket_handles;
std::string g_ca_cert_path;

static int init_canfd();
static void send_rtcm_via_canfd(const unsigned char * data, unsigned int len);

void sixents_diff_rtcm_process(const sixents_char * buff, sixents_uint32 len)
{
  if (!buff || len == 0) {
    return;
  }
  send_rtcm_via_canfd(reinterpret_cast<const unsigned char *>(buff), len);
}

void sixents_status_process(sixents_uint32 /*status*/) {}

int sixents_log_process_main(const sixents_char * /*buff*/, sixents_uint16 /*len*/)
{
  return 0;
}

static int init_canfd()
{
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    perror("CAN FD socket");
    return -1;
  }

  int enable = 1;
  if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
    perror("setsockopt CAN_RAW_FD_FRAMES");
    close(sock);
    return -1;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, kCanFdIf, IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl SIOCGIFINDEX");
    close(sock);
    return -1;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("bind CAN FD");
    close(sock);
    return -1;
  }
  return sock;
}

static void send_rtcm_via_canfd(const unsigned char * data, unsigned int len)
{
  if (!data || len == 0) {
    return;
  }
  if (g_canfd_socket < 0) {
    std::cerr << "[RTCM_DEBUG] 发送失败：g_canfd_socket 当前无效 (值为: " << g_canfd_socket
              << ")\n";
    return;
  }

  unsigned int offset = 0;
  int packet_idx = 0;
  while (offset < len) {
    struct canfd_frame frame{};
    frame.can_id = kCanFdId;
    frame.flags = CANFD_BRS;

    unsigned int chunk = (len - offset) > 64 ? 64 : (len - offset);
    std::memcpy(frame.data, data + offset, chunk);
    frame.len = chunk;

    ssize_t nbytes = write(g_canfd_socket, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0) {
      int err = errno;
      std::cerr << "[RTCM_DEBUG] 第 " << packet_idx << " 包发送异常！错误码: " << err
                << ", 原因: " << std::strerror(err) << "\n";
      break;
    }

    offset += chunk;
    packet_idx++;
  }
}

void sixents_thread_func()
{
  const char * path = g_ca_cert_path.c_str();
  char pcount[kCaMaxLen];
  std::memset(pcount, 0, kCaMaxLen);

  FILE * fp = fopen(path, "r");
  if (fp == nullptr) {
    std::cerr << "[错误] 无法打开证书文件: " << path << "\n";
    return;
  }

  size_t retVal = fread(pcount, sizeof(char), kCaMaxLen - 1, fp);
  pcount[retVal] = '\0';
  fclose(fp);

  std::cout << "==========================================\n";
  std::cout << "成功读取证书！读取长度: " << retVal << " 字节\n";

  g_canfd_socket = init_canfd();
  if (g_canfd_socket < 0) {
    fprintf(stderr, "Sixents thread: CAN FD init failed, RTCM will not be sent\n");
  }

  sixents_sdkConf param;
  std::memset(&param, 0, sizeof(param));
  param.paramSize = sizeof(param);
  param.keyType = SIXENTS_KEY_TYPE_AK;
  std::memcpy(param.key, kSixAk, strlen(kSixAk));
  std::memcpy(param.secret, kSixAs, strlen(kSixAs));
  std::memcpy(param.devID, kSixDevId, strlen(kSixDevId));
  std::memcpy(param.devType, kSixDevType, strlen(kSixDevType));
  std::memcpy(param.mountPoint, kSixMountPoint, strlen(kSixMountPoint));
  param.logPrintLevel = SIXENTS_LL_DEBUG;
  param.sockIOBlockFlag = SIXENTS_SOCK_IOFLAG_NOBLOCK;
  param.timeout = 10;
  param.pid = SIXENTS_PT_TLS_ONE;

  size_t ca_len = strlen(pcount);
  param.rootCA = static_cast<sixents_char *>(malloc(ca_len + 1));
  if (param.rootCA != nullptr) {
    strcpy(reinterpret_cast<char *>(const_cast<sixents_char *>(param.rootCA)), pcount);
  }
  param.serverPort = kSixRtcmPort;
  std::cout << "RootCA size: " << strlen(reinterpret_cast<const char *>(param.rootCA))
            << " bytes\n";

  param.cbGetDiffData = &sixents_diff_rtcm_process;
  param.cbGetStatus = &sixents_status_process;
  param.cbTrace = &sixents_log_process_main;

  if (sixents_sdkInit(&param) != SIXENTS_RET_OK) {
    fprintf(stderr, "Sixents SDK init failed\n");
    return;
  }
  if (sixents_sdkStart() != SIXENTS_RET_OK) {
    fprintf(stderr, "Sixents SDK start failed\n");
    sixents_sdkFinal();
    return;
  }

  g_six_sdk_running = true;
  std::cout << "Sixents SDK started. Entering Tick loop.\n";

  while (g_six_sdk_running && rclcpp::ok()) {
    sixents_int32 ret = sixents_sdkTick();
    if (ret != SIXENTS_RET_OK) {
      fprintf(stderr, "sixents_sdkTick error: %d\n", ret);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  g_six_sdk_running = false;
  sixents_sdkStop();
  sixents_sdkFinal();
  if (g_canfd_socket >= 0) {
    close(g_canfd_socket);
    g_canfd_socket = -1;
  }
}

void signalHandler(int signum)
{
  std::cout << "\nReceived signal " << signum << ". Shutting down..." << std::endl;
  g_six_sdk_running = false;
}

int main(int argc, char * argv[])
{
  // Sixents TLS/socket writes can raise SIGPIPE when the remote closes;
  // default disposition terminates the whole process (ROS exit code -13).
  std::signal(SIGPIPE, SIG_IGN);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  rclcpp::init(argc, argv);
  auto can_node = std::make_shared<can_driver::CanNode>();
  g_ca_cert_path = can_node->getCaCertPath();

  std::string interface1 = can_node->getCan1InterfaceName();
  bool can1_use = can_node->getCan1UseStatus();

  int socket1 = -1;
  if (can1_use) {
    can1_use = CanNode::initialize(interface1, socket1);
  }
  std::cout << "Socket1 fd: " << socket1 << std::endl;

  std::thread sixents_thread(sixents_thread_func);

  std::vector<std::thread> receive_threads;
  std::vector<std::thread> publish_threads;

  if (socket1 >= 0) {
    auto receiver = std::make_shared<CanReceiver>(can_node);
    g_receivers.emplace_back(receiver);
    g_socket_handles.push_back(socket1);

    receive_threads.emplace_back(&CanReceiver::receiveTask, receiver.get(), socket1, interface1);
    publish_threads.emplace_back(&CanReceiver::publishNavSatFixTask, receiver.get());
    publish_threads.emplace_back(&CanReceiver::publishGnssInsTask, receiver.get());
  } else {
    RCLCPP_ERROR(can_node->get_logger(), "Failed to initialize CAN interface %s", interface1.c_str());
  }

  rclcpp::spin(can_node);

  g_six_sdk_running = false;
  for (auto & receiver : g_receivers) {
    receiver->stop();
  }

  if (sixents_thread.joinable()) {
    sixents_thread.join();
  }
  for (auto & t : receive_threads) {
    if (t.joinable()) {
      t.join();
    }
  }
  for (auto & t : publish_threads) {
    if (t.joinable()) {
      t.join();
    }
  }
  g_receivers.clear();

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
