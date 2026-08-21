#ifndef CAN_DRIVER__CAN_NODE_HPP_
#define CAN_DRIVER__CAN_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

namespace can_driver
{

class CanNode : public rclcpp::Node
{
public:
  CanNode();
  ~CanNode() override;

  std::string getCan1InterfaceName() const { return interface1_; }
  bool getCan1UseStatus() const { return can1_use_; }
  std::string getRawLogBasename() const { return raw_log_basename_; }
  std::string getCaCertPath() const { return ca_cert_path_; }

  /**
   * Initialize a CAN FD socket bound to interface_name.
   * @return true on success, false on failure
   */
  static bool initialize(const std::string & interface_name, int & socket_handle);

private:
  std::string interface1_;
  int bitrate1_;
  bool can1_use_;
  int txqueuelen_;
  std::string raw_log_basename_;
  std::string ca_cert_path_;
};

}  // namespace can_driver

#endif  // CAN_DRIVER__CAN_NODE_HPP_
