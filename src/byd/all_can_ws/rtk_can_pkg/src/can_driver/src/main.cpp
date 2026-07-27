#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
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
#include "std_msgs/msg/string.hpp"

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
constexpr const char * kSixentsLogRoot = "/home/nvidia/autoware/log/sixtens";
}  // namespace

int g_canfd_socket = -1;
std::atomic<bool> g_six_sdk_running{false};

std::vector<std::shared_ptr<CanReceiver>> g_receivers;
std::vector<int> g_socket_handles;
std::string g_ca_cert_path;

std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> g_sixents_status_log_pub;
std::atomic<sixents_uint32> g_latest_sixents_status{0};
std::mutex g_sixents_log_mutex;
std::string g_latest_sixents_log;

std::mutex g_sixents_file_mutex;
std::ofstream g_sixents_log_file;
std::string g_sixents_log_file_path;

static int init_canfd();
static void send_rtcm_via_canfd(const unsigned char * data, unsigned int len);
static bool init_sixents_log_file();
static std::string current_time_string(const char * format);
static void publish_sixents_status_log(const char * event_type);
static void close_sixents_log_file();

void sixents_diff_rtcm_process(const sixents_char * buff, sixents_uint32 len)
{
  if (!buff || len == 0) {
    return;
  }
  send_rtcm_via_canfd(reinterpret_cast<const unsigned char *>(buff), len);
}

static std::string current_time_string(const char * format)
{
  const std::time_t now = std::time(nullptr);
  std::tm local_tm{};
  localtime_r(&now, &local_tm);

  std::ostringstream stream;
  stream << std::put_time(&local_tm, format);
  return stream.str();
}

static bool init_sixents_log_file()
{
  try {
    // 加入时分秒，保证同一天多次启动节点时不会覆盖之前的日志。
    const std::filesystem::path log_dir =
      std::filesystem::path(kSixentsLogRoot) / current_time_string("%Y%m%d_%H%M%S");

    std::filesystem::create_directories(log_dir);
    g_sixents_log_file_path = (log_dir / "status.log").string();

    std::lock_guard<std::mutex> lock(g_sixents_file_mutex);
    g_sixents_log_file.open(g_sixents_log_file_path, std::ios::out | std::ios::trunc);
    if (!g_sixents_log_file.is_open()) {
      std::cerr << "[Sixents日志] 无法创建日志文件: "
                << g_sixents_log_file_path << "\n";
      return false;
    }

    g_sixents_log_file << "# Sixents status/log started at "
                       << current_time_string("%Y-%m-%d %H:%M:%S") << "\n";
    g_sixents_log_file << "# format: [time] event=... status=... buff=...\n";
    g_sixents_log_file.flush();

    std::cout << "[Sixents日志] 日志文件: " << g_sixents_log_file_path << "\n";
    return true;
  } catch (const std::filesystem::filesystem_error & error) {
    std::cerr << "[Sixents日志] 创建日志目录失败: " << error.what() << "\n";
    return false;
  }
}

static void close_sixents_log_file()
{
  std::lock_guard<std::mutex> lock(g_sixents_file_mutex);
  if (g_sixents_log_file.is_open()) {
    g_sixents_log_file << "# Sixents status/log stopped at "
                       << current_time_string("%Y-%m-%d %H:%M:%S") << "\n";
    g_sixents_log_file.flush();
    g_sixents_log_file.close();
  }
}

static void publish_sixents_status_log(const char * event_type)
{
  std::string latest_log;
  {
    std::lock_guard<std::mutex> lock(g_sixents_log_mutex);
    latest_log = g_latest_sixents_log;
  }

  const sixents_uint32 latest_status = g_latest_sixents_status.load();

  // 发布 ROS 2 话题。日志文件打开失败时不影响话题发布。
  if (g_sixents_status_log_pub) {
    std_msgs::msg::String msg;
    msg.data = "status=" + std::to_string(latest_status) +
      "\nbuff=" + latest_log;
    g_sixents_status_log_pub->publish(msg);
  }

  // 将和话题相同的数据保存到本次节点启动对应的 status.log。
  std::lock_guard<std::mutex> lock(g_sixents_file_mutex);
  if (g_sixents_log_file.is_open()) {
    g_sixents_log_file << "[" << current_time_string("%Y-%m-%d %H:%M:%S") << "] "
                       << "event=" << (event_type != nullptr ? event_type : "unknown")
                       << " status=" << latest_status
                       << " buff=" << latest_log << "\n";
    // SDK 状态/日志通常频率不高，立即 flush 可避免节点异常退出时丢失缓存数据。
    g_sixents_log_file.flush();
  }
}

void sixents_status_process(sixents_uint32 status)
{
  g_latest_sixents_status.store(status);
  publish_sixents_status_log("status");
}

int sixents_log_process_main(const sixents_char * buff, sixents_uint16 len)
{
  if (buff == nullptr || len == 0) {
    return 0;
  }

  {
    std::lock_guard<std::mutex> lock(g_sixents_log_mutex);
    g_latest_sixents_log.assign(
      reinterpret_cast<const char *>(buff),
      static_cast<std::size_t>(len));
  }

  publish_sixents_status_log("buff");
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

  g_sixents_status_log_pub =
    can_node->create_publisher<std_msgs::msg::String>("/sixents/status_log", rclcpp::QoS(10));

  // 在启动 Sixents SDK 线程前创建本次运行的日志文件。
  // 创建失败只打印错误，不阻止节点继续运行和发布 ROS 2 话题。
  init_sixents_log_file();

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

  close_sixents_log_file();
  g_sixents_status_log_pub.reset();
  rclcpp::shutdown();
  std::cout << "ROS 2 shutdown. Program exiting." << std::endl;
  return 0;
}