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

namespace can_driver
{

std::string sixentsStatusToString(uint32_t status)
{
  switch (status) {
    case SIXENTS_STAT_INIT:
      return "初始状态 (0: SIXENTS_STAT_INIT)";
    case SIXENTS_STATE_NET_REQUEEST_OK:
      return "网络请求成功 (1001: SIXENTS_STATE_NET_REQUEEST_OK)";
    case SIXENTS_STATE_NET_REQUEEST_FAIL:
      return "网络请求失败 (1002: SIXENTS_STATE_NET_REQUEEST_FAIL)";
    case SIXENTS_STATE_NET_DISABLE:
      return "网络不可用 (1003: SIXENTS_STATE_NET_DISABLE)";
    case SIXENTS_STATE_AUTHENTICATE_OK:
      return "鉴权成功 (1201: SIXENTS_STATE_AUTHENTICATE_OK)";
    case SIXENTS_STATE_AUTHENTICATE_AKORAS_INVALID:
      return "AK或AS参数无效 (1202: SIXENTS_STATE_AUTHENTICATE_AKORAS_INVALID)";
    case SIXENTS_STATE_AUTHENTICATE_AS_INVALID:
      return "AS无效 (1203: SIXENTS_STATE_AUTHENTICATE_AS_INVALID)";
    case SIXENTS_STATE_AUTHENTICATE_ACCOUNT_PASS_LIMIT:
      return "账号已用完 (1204: SIXENTS_STATE_AUTHENTICATE_ACCOUNT_PASS_LIMIT)";
    case SIXENTS_STATE_AUTHENTICATE_ACCOUNT_INVALID:
      return "账号不存在 (1205: SIXENTS_STATE_AUTHENTICATE_ACCOUNT_INVALID)";
    case SIXENTS_STATE_AUTHENTICATE_FAIL:
      return "鉴权失败 (1206: SIXENTS_STATE_AUTHENTICATE_FAIL)";
    case SIXENTS_STATE_AUTHENTICATE_ACCOUNT_NOT_ACTIVE:
      return "账号未激活 (1207: SIXENTS_STATE_AUTHENTICATE_ACCOUNT_NOT_ACTIVE)";
    case SIXENTS_STATE_AUTHENTICATE_ACCOUNT_OVERDUE:
      return "账号已过期 (1208: SIXENTS_STATE_AUTHENTICATE_ACCOUNT_OVERDUE)";
    case SIXENTS_STATE_AUTHENTICATE_SERVER_EXCEPTION:
      return "服务器异常 (1209: SIXENTS_STATE_AUTHENTICATE_SERVER_EXCEPTION)";
    case SIXENTS_STATE_AUTHENTICATE_LOADING:
      return "AK/AS鉴权中 (1210: SIXENTS_STATE_AUTHENTICATE_LOADING)";
    case SIXENTS_STATE_AUTHENTICATE_ACCOUNT_EXCEPTION:
      return "账号异常，为空等 (1211: SIXENTS_STATE_AUTHENTICATE_ACCOUNT_EXCEPTION)";
    case SIXENTS_STATE_AUTHENTICATE_BINDING_FAILURE:
      return "绑定失败 (1212: SIXENTS_STATE_AUTHENTICATE_BINDING_FAILURE)";
    case SIXENTS_STATE_AUTHENTICATE_UNSUPPORTED_PROTOCOL:
      return "不支持的协议 (1213: SIXENTS_STATE_AUTHENTICATE_UNSUPPORTED_PROTOCOL)";
    case SIXENTS_STATE_AUTHENTICATE_DEVICE_NO_BINDING:
      return "设备未手动绑定 (1214: SIXENTS_STATE_AUTHENTICATE_DEVICE_NO_BINDING)";
    case SIXENTS_STATE_AUTHENTICATE_ACTIVED_FAILURE:
      return "激活失败 (1215: SIXENTS_STATE_AUTHENTICATE_ACTIVED_FAILURE)";
    case SIXENTS_STATE_HAVE_LOGIN_FAILURE:
      return "已登录 (1216: SIXENTS_STATE_HAVE_LOGIN_FAILURE)";
    case SIXENTS_STATE_INSTANCE_UNUSED_FAILURE:
      return "没有账号 (1217: SIXENTS_STATE_INSTANCE_UNUSED_FAILURE)";
    case SIXENTS_STATE_INSTANCE_IN_BLACKLIST:
      return "账号在黑名单 (1218: SIXENTS_STATE_INSTANCE_IN_BLACKLIST)";
    case SIXENTS_STATE_NW_CONNECT_SUCCESS:
      return "网络连接成功 (1301: SIXENTS_STATE_NW_CONNECT_SUCCESS)";
    case SIXENTS_STATE_NW_CONNECT_FAIL:
      return "网络连接失败 (1302: SIXENTS_STATE_NW_CONNECT_FAIL)";
    case SIXENTS_STATE_NW_CONNECT_LOADING:
      return "网络连接中 (1303: SIXENTS_STATE_NW_CONNECT_LOADING)";
    case SIXENTS_STATE_NW_CONNECT_TIMEOUT:
      return "网络连接超时 (1304: SIXENTS_STATE_NW_CONNECT_TIMEOUT)";
    case SIXENTS_STATE_NW_ACCOUNT_OUT:
      return "同一账号在另外设备登录 (1305: SIXENTS_STATE_NW_ACCOUNT_OUT)";
    case SIXENTS_STATE_SERVER_AUTHENTICATE_FAIL:
      return "数据服务认证失败 (1306: SIXENTS_STATE_SERVER_AUTHENTICATE_FAIL)";
    case SIXENTS_STATE_SERVER_AUTHENTICATE_SUCCESS:
      return "数据服务认证成功 (1307: SIXENTS_STATE_SERVER_AUTHENTICATE_SUCCESS)";
    case SIXENTS_STATE_SERVER_DOMAIN_ERROR:
      return "域名解析错误 (1308: SIXENTS_STATE_SERVER_DOMAIN_ERROR)";
    case SIXENTS_STATE_TLS_CONF_ERROR:
      return "TLS配置初始化失败 (1311: SIXENTS_STATE_TLS_CONF_ERROR)";
    case SIXENTS_STATE_TLS_CONN_ERROR:
      return "TLS连接失败 (1312: SIXENTS_STATE_TLS_CONN_ERROR)";
    case SIXENTS_STATE_RTCM_GET_SUCCESS:
      return "RTCM差分数据获取成功 (1401: SIXENTS_STATE_RTCM_GET_SUCCESS)";
    case SIXENTS_STATE_RTCM_GRID_OUT_OF_RANGE:
      return "地理网格码不在服务范围 (1402: SIXENTS_STATE_RTCM_GRID_OUT_OF_RANGE)";
    case SIXENTS_STATE_RTCM_GGA_OUT_OF_RANGE:
      return "GGA不在服务范围 (1403: SIXENTS_STATE_RTCM_GGA_OUT_OF_RANGE)";
    case SIXENTS_STATE_RTCM_GGA_GET_TIMEOUT:
      return "60秒未接收到GGA数据 (1404: SIXENTS_STATE_RTCM_GGA_GET_TIMEOUT)";
    case SIXENTS_STATE_RTCM_GGA_SEND_EXCEPTION:
      return "发送GGA数据时异常 (1405: SIXENTS_STATE_RTCM_GGA_SEND_EXCEPTION)";
    case SIXENTS_STATE_RTCM_GET_RTCM_TIMEOUT:
      return "60秒未获取到RTCM数据 (1406: SIXENTS_STATE_RTCM_GET_RTCM_TIMEOUT)";
    case SIXENTS_STATE_RTCM_SERVER_ERR:
      return "RTCM服务器错误 (1407: SIXENTS_STATE_RTCM_SERVER_ERR)";
    case SIXENTS_STATE_RTCM_UNKNOWN_ERR:
      return "未知错误 (1408: SIXENTS_STATE_RTCM_UNKNOWN_ERR)";
    case SIXENTS_STATE_GGA_SUCCESS:
      return "GGA数据有效 (1501: SIXENTS_STATE_GGA_SUCCESS)";
    case SIXENTS_STATE_GGA_INVALID:
      return "GGA数据无效 (1502: SIXENTS_STATE_GGA_INVALID)";
    default:
      return "状态码 (" + std::to_string(status) + ")";
  }
}

std::string sixentsRetCodeToString(int32_t ret)
{
  switch (ret) {
    case SIXENTS_RET_OK:
      return "成功 (0: SIXENTS_RET_OK)";
    case SIXENTS_RET_FAILED:
      return "失败 (-1: SIXENTS_RET_FAILED)";
    case SIXENTS_RET_NULL_PTR:
      return "空指针 (-2: SIXENTS_RET_NULL_PTR)";
    case SIXENTS_RET_INVALID_PARAM:
      return "无效参数 (-3: SIXENTS_RET_INVALID_PARAM)";
    case SIXENTS_RTT_COMPILER_SIZE:
      return "编译器宽度未对齐 (-4: SIXENTS_RTT_COMPILER_SIZE)";
    case SIXENTS_RET_INVALID_STATUS:
      return "SDK状态不合法 (-11: SIXENTS_RET_INVALID_STATUS)";
    case SIXENTS_RET_SOCKET_DISCONNECTED:
      return "网络被断开 (-101: SIXENTS_RET_SOCKET_DISCONNECTED)";
    case SIXENTS_RET_SOCKET_CREATE_ERROR:
      return "网络创建失败 (-102: SIXENTS_RET_SOCKET_CREATE_ERROR)";
    case SIXENTS_RET_SOCKET_BIND_ERROR:
      return "网络绑定本地IP失败 (-103: SIXENTS_RET_SOCKET_BIND_ERROR)";
    case SIXENTS_RET_SOCKET_CONNECT_ERROR:
      return "网络connect失败 (-104: SIXENTS_RET_SOCKET_CONNECT_ERROR)";
    case SIXENTS_RET_SOCKET_SEND_ERROR:
      return "网络send失败 (-105: SIXENTS_RET_SOCKET_SEND_ERROR)";
    case SIXENTS_RET_SOCKET_RECV_ERROR:
      return "网络recv失败 (-106: SIXENTS_RET_SOCKET_RECV_ERROR)";
    case SIXENTS_RET_SOCKET_TIMEOUT_ERROR:
      return "网络超时 (-107: SIXENTS_RET_SOCKET_TIMEOUT_ERROR)";
    case SIXENTS_RET_DNS_ERROR:
      return "域名解析错误 (-108: SIXENTS_RET_DNS_ERROR)";
    case SIXENTS_RET_NW_STATUS_ERROR:
      return "网络状态错误 (-109: SIXENTS_RET_NW_STATUS_ERROR)";
    case SIXENTS_RET_SOCKET_INVALID_ERROR:
      return "无效的socket (-110: SIXENTS_RET_SOCKET_INVALID_ERROR)";
    case SIXENTS_RET_SSL_INIT_ERROR:
      return "SSL初始化失败 (-111: SIXENTS_RET_SSL_INIT_ERROR)";
    case SIXENTS_RET_SSL_LOADCA_ERROR:
      return "SSL Load RootCA失败 (-112: SIXENTS_RET_SSL_LOADCA_ERROR)";
    case SIXENTS_RET_SSL_HANDSHAKE_ERROR:
      return "SSL handshake失败 (-113: SIXENTS_RET_SSL_HANDSHAKE_ERROR)";
    case SIXENTS_RET_SSL_VERIFYSVRCERT_ERROR:
      return "SSL校验服务证书失败 (-114: SIXENTS_RET_SSL_VERIFYSVRCERT_ERROR)";
    case SIXENTS_RET_SSL_LOAD_CLIENT_CERT_ERROR:
      return "SSL Load CLCA失败 (-115: SIXENTS_RET_SSL_LOAD_CLIENT_CERT_ERROR)";
    case SIXENTS_RET_SSL_LOAD_CLIENT_PKEY_ERROR:
      return "SSL Load CLPK失败 (-116: SIXENTS_RET_SSL_LOAD_CLIENT_PKEY_ERROR)";
    case SIXENTS_RET_GGA_OUT_OF_SERVICE_AREA:
      return "上报GGA不在服务区 (-201: SIXENTS_RET_GGA_OUT_OF_SERVICE_AREA)";
    case SIXENTS_RET_INVALID_GGA:
      return "上报GGA无效 (-202: SIXENTS_RET_INVALID_GGA)";
    case SIXENTS_RET_AUTH_FAILED:
      return "鉴权失败 (-301: SIXENTS_RET_AUTH_FAILED)";
    case SIXENTS_RET_START_FAILED:
      return "启动失败 (-401: SIXENTS_RET_START_FAILED)";
    case SIXENTS_RET_OUT_OF_MEMORY:
      return "内存溢出 (-501: SIXENTS_RET_OUT_OF_MEMORY)";
    case SIXENTS_RET_START_THREAD_ERROR:
      return "启动线程失败 (-502: SIXENTS_RET_START_THREAD_ERROR)";
    case SIXENTS_RET_OUT_OF_BOUNDARY:
      return "数组越界 (-507: SIXENTS_RET_OUT_OF_BOUNDARY)";
    default:
      return "错误码 (" + std::to_string(ret) + ")";
  }
}

std::string getLatestSixentsLog()
{
  std::lock_guard<std::mutex> lock(g_sixents_log_mutex);
  return g_latest_sixents_log;
}

uint32_t getLatestSixentsStatus()
{
  return g_latest_sixents_status.load();
}

bool isRtcmNormal()
{
  if (!g_six_sdk_running.load()) {
    return false;
  }
  int64_t last_rx = g_last_rtcm_rx_time_ms.load();
  if (last_rx == 0) {
    return false;
  }
  auto now_sys = std::chrono::system_clock::now();
  int64_t now_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now_sys.time_since_epoch()).count();
  if (now_ms - last_rx > 3000) {
    return false;
  }
  return true;
}

}  // namespace can_driver

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
    RCLCPP_WARN(
      rclcpp::get_logger("can_rtk_node"), "[配置] 无法打开 %s，使用默认值。", filepath.c_str());
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
      RCLCPP_WARN(
        rclcpp::get_logger("can_rtk_node"), "[配置] 第 %d 行缺少冒号，跳过。", line_no);
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
      RCLCPP_WARN(
        rclcpp::get_logger("can_rtk_node"), "[配置] 未知键: %s，已忽略。", key.c_str());
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("can_rtk_node"), "[配置] 成功从 %s 加载参数。", filepath.c_str());
  return true;
}
// =====================================================

void sixents_diff_rtcm_process(const sixents_char * buff, sixents_uint32 len)
{
  sixents_uint32 cur_status = g_latest_sixents_status.load();
  std::string cur_log = getLatestSixentsLog();

  if (!buff || len == 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("can_rtk_node"),
      "[六分差分未正常下发] 警告: 接收到空的 RTCM 数据包 (buff=%p, len=%u)! 服务器返回状态码: %u (%s) | 服务器响应/内容: %s",
      static_cast<const void *>(buff), len,
      cur_status, sixentsStatusToString(cur_status).c_str(),
      cur_log.empty() ? "(暂无)" : cur_log.c_str());
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
  } else if (!valid_preamble) {
    // 前导码非 0xD3，差分数据没有正常下发，很可能是服务器返回的错误文本或异常报文
    std::string raw_text(reinterpret_cast<const char *>(udata), std::min<size_t>(len, 256));
    RCLCPP_WARN(
      rclcpp::get_logger("can_rtk_node"),
      "[六分差分未正常下发] 警告: 前导码非0xD3 (首字节=0x%02x, 长度=%u 字节)! 服务器返回状态码: %u (%s) | 数据流原始内容: %s | 服务器响应: %s",
      static_cast<unsigned int>(udata[0]), len,
      cur_status, sixentsStatusToString(cur_status).c_str(),
      raw_text.c_str(),
      cur_log.empty() ? "(暂无)" : cur_log.c_str());
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
    std::string preamble_info;
    if (valid_preamble && rtcm_msg_type > 0) {
      preamble_info = " (RTCM3前导码正常 0xD3, MessageType=" + std::to_string(rtcm_msg_type) + ")";
    } else if (!valid_preamble) {
      char hexbuf[64];
      std::snprintf(hexbuf, sizeof(hexbuf), " (警告: 前导码非0xD3, 首字节=0x%02x)", static_cast<unsigned int>(udata[0]));
      preamble_info = hexbuf;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("can_rtk_node"),
      "[六分差分-下发] 收到云端RTCM数据包: %u 字节%s | 近%ldms下发: %u包 (%u字节) | 累计: %lu包 (%lu字节)",
      len, preamble_info.c_str(), static_cast<long>(elapsed_ms), interval_rx_packets,
      interval_rx_bytes, static_cast<unsigned long>(g_rtcm_rx_count.load()),
      static_cast<unsigned long>(g_rtcm_rx_bytes.load()));
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
      RCLCPP_ERROR(
        rclcpp::get_logger("can_rtk_node"),
        "[Sixents日志] 无法创建日志文件: %s", g_sixents_log_file_path.c_str());
      return false;
    }

    g_sixents_log_file << "# Sixents status/log started at "
                       << current_time_string("%Y-%m-%d %H:%M:%S") << "\n";
    g_sixents_log_file << "# format: [time] event=... status=... buff=...\n";
    g_sixents_log_file.flush();

    RCLCPP_INFO(
      rclcpp::get_logger("can_rtk_node"),
      "[Sixents日志] 日志文件: %s", g_sixents_log_file_path.c_str());
    return true;
  } catch (const std::filesystem::filesystem_error & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "[Sixents日志] 创建日志目录失败: %s", error.what());
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

  // 差分数据正常更新时，绝不输出任何服务器返回内容
  if (isRtcmNormal()) {
    return;
  }

  // 正常流程的状态码（1401 RTCM成功、1501 GGA有效）无需在控制台报警输出
  if (status == SIXENTS_STATE_RTCM_GET_SUCCESS || status == SIXENTS_STATE_GGA_SUCCESS) {
    return;
  }

  std::string latest_log = getLatestSixentsLog();
  // 只有在差分数据不能正常更新时，才输出状态码及服务器返回的所有内容
  if (status == SIXENTS_STATE_AUTHENTICATE_LOADING ||
      status == SIXENTS_STATE_NW_CONNECT_LOADING)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("can_rtk_node"),
      "[六分差分状态更新] 服务器返回状态码: %u (%s) | 服务器响应/内容: %s",
      status, sixentsStatusToString(status).c_str(),
      latest_log.empty() ? "(暂无)" : latest_log.c_str());
  } else if (status != SIXENTS_STATE_NET_REQUEEST_OK &&
             status != SIXENTS_STATE_AUTHENTICATE_OK &&
             status != SIXENTS_STATE_NW_CONNECT_SUCCESS &&
             status != SIXENTS_STATE_SERVER_AUTHENTICATE_SUCCESS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "[六分差分未正常更新] 差分服务异常! 服务器返回状态码: %u (%s) | 服务器返回所有内容: %s",
      status, sixentsStatusToString(status).c_str(),
      latest_log.empty() ? "(暂无)" : latest_log.c_str());
  }
}

int sixents_log_process_main(const sixents_char * buff, sixents_uint16 len)
{
  if (buff == nullptr || len == 0) {
    return 0;
  }

  std::string log_str(reinterpret_cast<const char *>(buff), static_cast<std::size_t>(len));
  while (!log_str.empty() && (log_str.back() == '\r' || log_str.back() == '\n')) {
    log_str.pop_back();
  }

  {
    std::lock_guard<std::mutex> lock(g_sixents_log_mutex);
    g_latest_sixents_log = log_str;
  }

  publish_sixents_status_log("buff");

  // 差分数据正常更新时，绝不输出任何服务器返回的内容
  if (isRtcmNormal()) {
    return 0;
  }

  sixents_uint32 status = g_latest_sixents_status.load();
  // 差分数据没有正常更新，且不是普通的本地GGA发送调试日志时，才输出服务器返回详情
  if (log_str.find("sixents_sendGgaStr") == std::string::npos) {
    RCLCPP_WARN(
      rclcpp::get_logger("can_rtk_node"),
      "[六分差分未正常更新-服务器返回详情] 状态码: %u (%s) | 服务器内容: %s",
      status, sixentsStatusToString(status).c_str(),
      log_str.c_str());
  }
  return 0;
}

static int init_canfd()
{
  int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"), "CAN FD socket 创建失败: %s", std::strerror(errno));
    return -1;
  }

  int enable = 1;
  if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"), "setsockopt CAN_RAW_FD_FRAMES 失败: %s", std::strerror(errno));
    close(sock);
    return -1;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, g_can_fd_if.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"), "ioctl SIOCGIFINDEX 获取接口 %s 失败: %s",
      g_can_fd_if.c_str(), std::strerror(errno));
    close(sock);
    return -1;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"), "bind CAN FD 接口 %s 失败: %s",
      g_can_fd_if.c_str(), std::strerror(errno));
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
      RCLCPP_ERROR(
        rclcpp::get_logger("can_rtk_node"),
        "[RTCM->CANFD] 回灌失败: CAN-FD socket 当前无效 (fd=%d, 目标接口=%s)",
        g_canfd_socket, g_can_fd_if.c_str());
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
      RCLCPP_ERROR(
        rclcpp::get_logger("can_rtk_node"),
        "[RTCM->CANFD] 发送异常! 接口=%s, CAN-ID=0x%x, 第 %d 包, 错误码: %d (%s)",
        g_can_fd_if.c_str(), g_can_fd_id, packet_idx, err, std::strerror(err));
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
    RCLCPP_DEBUG(
      rclcpp::get_logger("can_rtk_node"),
      "[RTCM->CANFD] 回灌成功: 本次已将 %u 字节分 %d 帧写入 %s (ID=0x%x) | 累计写入: %lu 帧 (%lu 字节)",
      len, success_frames, g_can_fd_if.c_str(), g_can_fd_id,
      static_cast<unsigned long>(g_rtcm_canfd_tx_frames.load()),
      static_cast<unsigned long>(g_rtcm_canfd_tx_bytes.load()));
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
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"), "[错误] 无法打开证书文件: %s", path);
    return;
  }

  size_t retVal = fread(pcount.data(), sizeof(char), g_ca_max_len - 1, fp);
  pcount[retVal] = '\0';
  fclose(fp);

  RCLCPP_INFO(
    rclcpp::get_logger("can_rtk_node"),
    "成功读取证书！证书路径: %s, 读取长度: %zu 字节", path, retVal);

  g_canfd_socket = init_canfd();
  if (g_canfd_socket < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "Sixents thread: CAN FD 初始化失败, 差分RTCM将无法发送至CAN硬件");
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
  RCLCPP_INFO(
    rclcpp::get_logger("can_rtk_node"),
    "RootCA 证书大小: %zu 字节, 差分端口: %d",
    strlen(reinterpret_cast<const char *>(param.rootCA)), g_six_rtcm_port);

  param.cbGetDiffData = &sixents_diff_rtcm_process;
  param.cbGetStatus = &sixents_status_process;
  param.cbTrace = &sixents_log_process_main;

  if (sixents_sdkInit(&param) != SIXENTS_RET_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "Sixents SDK 初始化失败 (sixents_sdkInit failed)");
    return;
  }
  sixents_retCode ret_start = sixents_sdkStart();
  if (ret_start != SIXENTS_RET_OK) {
    sixents_uint32 cur_status = g_latest_sixents_status.load();
    std::string cur_log = getLatestSixentsLog();
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "==================== [六分SDK 启动失败 - 差分数据无法下发] ====================");
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "sixents_sdkStart() 返回非 OK 错误码: %d (%s)!",
      ret_start, sixentsRetCodeToString(ret_start).c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "服务器返回状态码: %u (%s)", cur_status, sixentsStatusToString(cur_status).c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "服务器返回所有内容/日志: %s", cur_log.empty() ? "(暂无)" : cur_log.c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "详细日志文件: %s", g_sixents_log_file_path.c_str());
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "常见原因对照:");
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "  - status=1305: 另一台车正在使用相同的设备ID/账号 (单点登录互斥)");
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "  - status=1312: 系统时间错误导致 TLS 证书失效 (运行 date 检查时间)");
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "  - status=1302/1304: 差分端口 4405 或 443 超时/无法连接 (运行 nc -zv vrs.sixents.com 4405)");
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "  - status=1202/1206: AK/AS/DevID 鉴权失败或 yaml 配置路径不正确");
    RCLCPP_ERROR(
      rclcpp::get_logger("can_rtk_node"),
      "===============================================================================");
    sixents_sdkFinal();
    return;
  }

  g_six_sdk_running = true;
  RCLCPP_INFO(
    rclcpp::get_logger("can_rtk_node"),
    "Sixents SDK 启动成功, 进入 Tick 循环。");

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
          RCLCPP_DEBUG(
            rclcpp::get_logger("can_rtk_node"),
            "[六分上行-GGA] 成功上报GGA位置数据 (长度: %zu 字节, 累计: %lu 次) -> 云端据此下发差分",
            gga_str.size(), static_cast<unsigned long>(g_gga_tx_count.load()));
        } else {
          sixents_uint32 cur_status = g_latest_sixents_status.load();
          std::string cur_log = getLatestSixentsLog();
          RCLCPP_ERROR(
            rclcpp::get_logger("can_rtk_node"),
            "[六分上行-GGA] 上报GGA失败! 错误码: %d (%s) | 差分数据将无法正常下发! 服务器状态码: %u (%s) | 服务器响应: %s",
            ret_gga, sixentsRetCodeToString(ret_gga).c_str(),
            cur_status, sixentsStatusToString(cur_status).c_str(),
            cur_log.empty() ? "(暂无)" : cur_log.c_str());
        }
        last_gga_log = steady_now;
      }
    }

    sixents_int32 ret = sixents_sdkTick();
    if (ret != SIXENTS_RET_OK) {
      sixents_uint32 cur_status = g_latest_sixents_status.load();
      std::string cur_log = getLatestSixentsLog();
      RCLCPP_WARN(
        rclcpp::get_logger("can_rtk_node"),
        "[六分Tick] sixents_sdkTick 异常返回值: %d (%s) | 差分数据状态: 服务器状态码: %u (%s) | 服务器响应: %s",
        ret, sixentsRetCodeToString(ret).c_str(),
        cur_status, sixentsStatusToString(cur_status).c_str(),
        cur_log.empty() ? "(暂无)" : cur_log.c_str());
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
  RCLCPP_INFO(
    rclcpp::get_logger("can_rtk_node"),
    "接收到信号 %d, 正在退出...", signum);
  g_six_sdk_running = false;
  can_driver::notifyGGA();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  loadConfigFromYaml("/home/nvidia/autoware/src/byd/all_can_ws/rtk_can_pkg/src/can_driver/src/six.yaml");

  std::signal(SIGPIPE, SIG_IGN);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

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
  RCLCPP_INFO(
    rclcpp::get_logger("can_rtk_node"), "CAN1 Socket fd: %d", socket1);

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
      RCLCPP_INFO(
        rclcpp::get_logger("can_rtk_node"), "正在关闭 socket 句柄: %d", handle);
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
  RCLCPP_INFO(
    rclcpp::get_logger("can_rtk_node"), "ROS 2 shutdown. 程序退出。");
  rclcpp::shutdown();
  return 0;
}