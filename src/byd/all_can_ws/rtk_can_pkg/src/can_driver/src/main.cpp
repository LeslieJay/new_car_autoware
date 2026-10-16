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
// 原常量全部改为全局变量，通过 six.yaml 进行赋值
int g_ca_max_len = 3000;
std::string g_six_mount_point = "RTCM32_GRECJ2";
std::string g_six_ak = "4734542738";
std::string g_six_as = "fb0vwkac16a5gyqziya6d385rwdg1z75qmjdy66gbzey57k19debgnwnt3v0vkf0";
std::string g_six_dev_id = "1421225047560";
std::string g_six_dev_type = "SDK";
int g_six_rtcm_port = 4405;
std::string g_can_fd_if = "can1";
canid_t g_can_fd_id = 0x611;
std::string g_sixents_log_root = "/home/nvidia/autoware/log/sixtens";
}  // namespace

int g_canfd_socket = -1;
std::atomic<bool> g_six_sdk_running{false};

// 差分数据（RTCM）与上行统计全局变量
std::atomic<uint64_t> g_rtcm_rx_count{0};
std::atomic<uint64_t> g_rtcm_rx_bytes{0};
std::atomic<int64_t> g_last_rtcm_rx_time_ms{0};

std::atomic<uint64_t> g_rtcm_canfd_tx_frames{0};
std::atomic<uint64_t> g_rtcm_canfd_tx_bytes{0};
std::atomic<uint64_t> g_rtcm_canfd_tx_err_count{0};

std::atomic<uint64_t> g_gga_tx_count{0};
std::atomic<int64_t> g_last_gga_tx_time_ms{0};

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

// ========== 简单的 YAML 配置文件加载（无外部依赖）==========
static void trim(std::string &s)
{
  s.erase(0, s.find_first_not_of(" \t\r\n"));
  s.erase(s.find_last_not_of(" \t\r\n") + 1);
}

static bool loadConfigFromYaml(const std::string &filepath)
{
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "[配置] 无法打开 " << filepath << "，使用默认值。\n";
    return false;
  }

  std::string line;
  int line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    trim(line);
    if (line.empty()) continue;

    size_t colon = line.find(':');
    if (colon == std::string::npos) {
      std::cerr << "[配置] 第 " << line_no << " 行缺少冒号，跳过。\n";
      continue;
    }

    std::string key = line.substr(0, colon);
    std::string value = line.substr(colon + 1);
    trim(key);
    trim(value);

    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }

    if (key == "kCaMaxLen") {
      g_ca_max_len = std::stoi(value);
    } else if (key == "kSixMountPoint") {
      g_six_mount_point = value;
    } else if (key == "kSixAk") {
      g_six_ak = value;
    } else if (key == "kSixAs") {
      g_six_as = value;
    } else if (key == "kSixDevId") {
      g_six_dev_id = value;
    } else if (key == "kSixDevType") {
      g_six_dev_type = value;
    } else if (key == "kSixRtcmPort") {
      g_six_rtcm_port = std::stoi(value);
    } else if (key == "kCanFdIf") {
      g_can_fd_if = value;
    } else if (key == "kCanFdId") {
      g_can_fd_id = std::stoul(value, nullptr, 0);
    } else if (key == "kSixentsLogRoot") {
      g_sixents_log_root = value;
    } else {
      std::cerr << "[配置] 未知键: " << key << "，已忽略。\n";
    }
  }
  std::cout << "[配置] 成功从 " << filepath << " 加载参数。\n";
  return true;
}
// =====================================================

void sixents_diff_rtcm_process(const sixents_char * buff, sixents_uint32 len)
{
  if (!buff || len == 0) {
    std::cerr << "[六分差分-下发] 警告: 接收到空的 RTCM 数据包 (buff=" << reinterpret_cast<const void *>(buff)
              << ", len=" << len << ")\n";
    return;
  }

  const unsigned char * udata = reinterpret_cast<const unsigned char *>(buff);
  auto now = std::chrono::system_clock::now();
  int64_t now_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  g_last_rtcm_rx_time_ms.store(now_ms);
  g_rtcm_rx_count++;
  g_rtcm_rx_bytes += len;

  // RTCM 3.x 报文特征解析（前导码 0xD3，高 6 位保留位全为 0，10 位长度，12 位 Message Type）
  int rtcm_msg_type = -1;
  bool valid_preamble = (udata[0] == 0xD3);
  if (valid_preamble && len >= 6) {
    // 字节 0: 0xD3 (前导码 8 bits)
    // 字节 1: 000000 + 长度高 2 位
    // 字节 2: 长度低 8 位
    // 字节 3: Message Number 高 8 位
    // 字节 4: Message Number 低 4 位 (高 4 位)
    rtcm_msg_type = (static_cast<int>(udata[3]) << 4) | (static_cast<int>(udata[4]) >> 4);
  }

  // 节流汇总日志，防止高频差分数据（1Hz~5Hz）导致终端刷屏
  static auto last_log_time = std::chrono::steady_clock::now();
  static uint32_t interval_rx_bytes = 0;
  static uint32_t interval_rx_packets = 0;
  interval_rx_bytes += len;
  interval_rx_packets++;

  auto steady_now = std::chrono::steady_clock::now();
  auto elapsed_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(steady_now - last_log_time).count();

  // 前3包或每隔1秒输出详细诊断
  if (elapsed_ms >= 1000 || g_rtcm_rx_count <= 3) {
    std::cout << "[六分差分-下发] 收到云端RTCM数据包: " << len << " 字节";
    if (valid_preamble && rtcm_msg_type > 0) {
      std::cout << " (RTCM3前导码正常 0xD3, MessageType=" << rtcm_msg_type << ")";
    } else if (!valid_preamble) {
      std::cout << " (警告: 前导码非0xD3, 首字节=0x" << std::hex << static_cast<int>(udata[0])
                << std::dec << ")";
    }
    std::cout << " | 近" << elapsed_ms << "ms下发: " << interval_rx_packets << "包 ("
              << interval_rx_bytes << "字节) | 累计: " << g_rtcm_rx_count.load() << "包 ("
              << g_rtcm_rx_bytes.load() << "字节)\n";
    last_log_time = steady_now;
    interval_rx_bytes = 0;
    interval_rx_packets = 0;
  }

  send_rtcm_via_canfd(udata, len);
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
    const std::filesystem::path log_dir =
      std::filesystem::path(g_sixents_log_root) / current_time_string("%Y%m%d_%H%M%S");

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

  if (g_sixents_status_log_pub) {
    std_msgs::msg::String msg;
    msg.data = "status=" + std::to_string(latest_status) +
      "\nbuff=" + latest_log;
    g_sixents_status_log_pub->publish(msg);
  }

  std::lock_guard<std::mutex> lock(g_sixents_file_mutex);
  if (g_sixents_log_file.is_open()) {
    g_sixents_log_file << "[" << current_time_string("%Y-%m-%d %H:%M:%S") << "] "
                       << "event=" << (event_type != nullptr ? event_type : "unknown")
                       << " status=" << latest_status
                       << " buff=" << latest_log << "\n";
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
  std::strncpy(ifr.ifr_name, g_can_fd_if.c_str(), IFNAMSIZ - 1);
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
    static auto last_sock_err_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_sock_err_time).count() >= 2) {
      std::cerr << "[RTCM->CANFD] 回灌失败: CAN-FD socket 当前无效 (fd=" << g_canfd_socket
                << ", 目标接口=" << g_can_fd_if << ")\n";
      last_sock_err_time = now;
    }
    return;
  }

  unsigned int offset = 0;
  int packet_idx = 0;
  int success_frames = 0;
  while (offset < len) {
    struct canfd_frame frame{};
    frame.can_id = g_can_fd_id;
    frame.flags = CANFD_BRS;

    unsigned int chunk = (len - offset) > 64 ? 64 : (len - offset);
    std::memcpy(frame.data, data + offset, chunk);
    frame.len = chunk;

    ssize_t nbytes = write(g_canfd_socket, &frame, sizeof(struct canfd_frame));
    if (nbytes < 0) {
      int err = errno;
      g_rtcm_canfd_tx_err_count++;
      std::cerr << "[RTCM->CANFD] 发送异常! 接口=" << g_can_fd_if << ", CAN-ID=0x"
                << std::hex << g_can_fd_id << std::dec << ", 第 " << packet_idx
                << " 包, 错误码: " << err << " (" << std::strerror(err) << ")\n";
      break;
    }

    success_frames++;
    g_rtcm_canfd_tx_frames++;
    g_rtcm_canfd_tx_bytes += chunk;
    offset += chunk;
    packet_idx++;
  }

  // 节流汇总日志，防止每帧都打印
  static auto last_tx_log_time = std::chrono::steady_clock::now();
  auto steady_now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(steady_now - last_tx_log_time).count() >=
        1000 ||
      g_rtcm_canfd_tx_frames <= 5)
  {
    std::cout << "[RTCM->CANFD] 回灌成功: 本次已将 " << len << " 字节分 " << success_frames
              << " 帧写入 " << g_can_fd_if << " (ID=0x" << std::hex << g_can_fd_id << std::dec
              << ") | 累计写入: " << g_rtcm_canfd_tx_frames.load() << " 帧 ("
              << g_rtcm_canfd_tx_bytes.load() << " 字节)\n";
    last_tx_log_time = steady_now;
  }
}

void sixents_thread_func()
{
  const char * path = g_ca_cert_path.c_str();
  std::vector<char> pcount(g_ca_max_len);
  std::memset(pcount.data(), 0, g_ca_max_len);

  FILE * fp = fopen(path, "r");
  if (fp == nullptr) {
    std::cerr << "[错误] 无法打开证书文件: " << path << "\n";
    return;
  }

  size_t retVal = fread(pcount.data(), sizeof(char), g_ca_max_len - 1, fp);
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

  std::memcpy(param.key, g_six_ak.c_str(), g_six_ak.length());
  std::memcpy(param.secret, g_six_as.c_str(), g_six_as.length());
  std::memcpy(param.devID, g_six_dev_id.c_str(), g_six_dev_id.length());
  std::memcpy(param.devType, g_six_dev_type.c_str(), g_six_dev_type.length());
  std::memcpy(param.mountPoint, g_six_mount_point.c_str(), g_six_mount_point.length());

  param.logPrintLevel = SIXENTS_LL_DEBUG;
  param.sockIOBlockFlag = SIXENTS_SOCK_IOFLAG_NOBLOCK;
  param.timeout = 10;
  param.pid = SIXENTS_PT_TLS_ONE;

  size_t ca_len = strlen(pcount.data());
  param.rootCA = static_cast<sixents_char *>(malloc(ca_len + 1));
  if (param.rootCA != nullptr) {
    strcpy(reinterpret_cast<char *>(const_cast<sixents_char *>(param.rootCA)), pcount.data());
  }
  param.serverPort = g_six_rtcm_port;
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
    fprintf(stderr, "\n==================== [六分SDK 启动失败] ====================\n");
    fprintf(stderr, "[错误] sixents_sdkStart() 返回非 OK 错误码!\n");
    fprintf(stderr, "[排查建议]:\n");
    fprintf(stderr, "  1. 请检查日志文件获取底层具体状态码: %s\n", g_sixents_log_file_path.c_str());
    fprintf(stderr, "  2. 常见原因对照:\n");
    fprintf(stderr, "     - status=1305: 另一台车正在使用相同的设备ID/账号 (单点登录互斥)\n");
    fprintf(stderr, "     - status=1312: 系统时间错误导致 TLS 证书失效 (运行 date 检查时间)\n");
    fprintf(
      stderr,
      "     - status=1302/1304: 差分端口 4405 或 443 超时/无法连接 (运行 nc -zv vrs.sixents.com "
      "4405)\n");
    fprintf(stderr, "     - status=1202/1206: AK/AS/DevID 鉴权失败或 yaml 配置路径不正确\n");
    fprintf(stderr, "============================================================\n\n");
    sixents_sdkFinal();
    return;
  }

  g_six_sdk_running = true;
  std::cout << "Sixents SDK started. Entering Tick loop.\n";

  while (g_six_sdk_running && rclcpp::ok()) {
    std::string gga_str;
    if (can_driver::waitForGGA(gga_str, std::chrono::milliseconds(200))) {
      sixents_retCode ret_gga = sixents_sdkSendGGAStr(gga_str.c_str(), gga_str.size());
      g_gga_tx_count++;
      auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
      g_last_gga_tx_time_ms.store(now_ms);

      static auto last_gga_log = std::chrono::steady_clock::now();
      auto steady_now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(steady_now - last_gga_log).count() >= 5 ||
          g_gga_tx_count <= 2)
      {
        if (ret_gga == SIXENTS_RET_OK) {
          std::cout << "[六分上行-GGA] 成功上报GGA位置数据 (长度: " << gga_str.size() << " 字节, 累计: "
                    << g_gga_tx_count.load() << " 次) -> 云端据此下发差分\n";
        } else {
          std::cerr << "[六分上行-GGA] 上报GGA失败! 错误码: " << ret_gga << "\n";
        }
        last_gga_log = steady_now;
      }
    }

    sixents_int32 ret = sixents_sdkTick();
    if (ret != SIXENTS_RET_OK) {
      fprintf(stderr, "[六分Tick] sixents_sdkTick 异常返回值: %d\n", ret);
    }
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
  can_driver::notifyGGA();
}

int main(int argc, char * argv[])
{
  loadConfigFromYaml("/home/nvidia/autoware/src/byd/all_can_ws/rtk_can_pkg/src/can_driver/src/six.yaml");

  std::signal(SIGPIPE, SIG_IGN);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  rclcpp::init(argc, argv);
  auto can_node = std::make_shared<can_driver::CanNode>();
  g_ca_cert_path = can_node->getCaCertPath();

  g_sixents_status_log_pub =
    can_node->create_publisher<std_msgs::msg::String>("/sixents/status_log", rclcpp::QoS(10));

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

  if (socket1 >= 0) {
    auto receiver = std::make_shared<CanReceiver>(can_node);
    g_receivers.emplace_back(receiver);
    g_socket_handles.push_back(socket1);

    // 只启动一个接收线程，内部直接处理所有数据
    receive_threads.emplace_back(&CanReceiver::receiveTask, receiver.get(), socket1, interface1);
  } else {
    RCLCPP_ERROR(can_node->get_logger(), "Failed to initialize CAN interface %s", interface1.c_str());
  }

  rclcpp::spin(can_node);

  // 停止所有接收器
  g_six_sdk_running = false;
  can_driver::notifyGGA();
  for (auto & receiver : g_receivers) {
    receiver->stop();
  }

  // 关闭 CAN socket 以唤醒被 read() 阻塞的线程
  for (int & handle : g_socket_handles) {
    if (handle >= 0) {
      std::cout << "Closing socket handle: " << handle << std::endl;
      ::close(handle);
      handle = -1;
    }
  }

  if (sixents_thread.joinable()) {
    sixents_thread.join();
  }
  for (auto & t : receive_threads) {
    if (t.joinable()) {
      t.join();
    }
  }
  g_receivers.clear();

  close_sixents_log_file();
  g_sixents_status_log_pub.reset();
  rclcpp::shutdown();
  std::cout << "ROS 2 shutdown. Program exiting." << std::endl;
  return 0;
}