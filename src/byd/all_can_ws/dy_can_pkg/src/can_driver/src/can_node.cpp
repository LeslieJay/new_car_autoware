#include "can_driver/can_node.hpp"

#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace can_driver
{

CanNode::CanNode() : Node("can_six_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing CAN Node...");

  this->declare_parameter("interface1", "can1");
  this->declare_parameter("bitrate1", 250000);
  this->declare_parameter("can1_use", true);
  this->declare_parameter("txqueuelen", 1000);
  this->declare_parameter("raw_log_basename", "/tmp/rtk_canfd/rtk_canfd");

  std::string default_ca;
  try {
    default_ca =
      ament_index_cpp::get_package_share_directory("can_six_driver") + "/certs/sixentsCA.crt";
  } catch (const std::exception &) {
    default_ca = "";
  }
  this->declare_parameter("ca_cert_path", default_ca);

  interface1_ = this->get_parameter("interface1").as_string();
  bitrate1_ = this->get_parameter("bitrate1").as_int();
  can1_use_ = this->get_parameter("can1_use").as_bool();
  txqueuelen_ = this->get_parameter("txqueuelen").as_int();
  raw_log_basename_ = this->get_parameter("raw_log_basename").as_string();
  ca_cert_path_ = this->get_parameter("ca_cert_path").as_string();

  RCLCPP_INFO(
    this->get_logger(), "CAN1: %s @ %d bps, enabled: %s", interface1_.c_str(), bitrate1_,
    can1_use_ ? "yes" : "no");
  RCLCPP_INFO(this->get_logger(), "CA cert path: %s", ca_cert_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Raw log basename: %s", raw_log_basename_.c_str());
}

CanNode::~CanNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down CAN node...");
}

bool CanNode::initialize(const std::string & interface_name, int & socket_handle)
{
  socket_handle = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_handle < 0) {
    perror("socket");
    return false;
  }

  int enable_canfd = 1;
  if (setsockopt(
        socket_handle, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0)
  {
    perror("setsockopt CAN FD");
    close(socket_handle);
    socket_handle = -1;
    return false;
  }

  struct can_filter filter;
  filter.can_id = 0;
  filter.can_mask = 0;
  if (setsockopt(socket_handle, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    perror("setsockopt CAN filter");
    close(socket_handle);
    socket_handle = -1;
    return false;
  }

  struct ifreq ifr{};
  strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_handle, SIOCGIFINDEX, &ifr) < 0) {
    perror("SIOCGIFINDEX");
    close(socket_handle);
    socket_handle = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_handle, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("bind");
    close(socket_handle);
    socket_handle = -1;
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("can_node"), "CANFD socket initialized: %s", interface_name.c_str());
  return true;
}

}  // namespace can_driver
