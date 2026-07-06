/*** 
 * @Author: lwr
 * @Date: 2025-03-04 16:14:53
 * @LastEditTime: 2025-03-28 10:31:58
 * @LastEditors: lwr
 * @Description: 电池控制实现类
 * @FilePath: /qr_agv/src/usbcan/include/usbcan/usbcan_battery.hpp
 */

#ifndef __USBCAN_BATTERY_HPP__
#define __USBCAN_BATTERY_HPP__

#include "can_driver/can_send.hpp"
#include "agv_interfaces/msg/battery_state.hpp"  
#include "agv_interfaces/srv/battery_control.hpp" 
#include "can_driver/agv_config.h" 

using agv_interfaces::srv::BatteryControl;
using agv_interfaces::msg::BatteryState;
using namespace std::placeholders;
using namespace std::chrono_literals;



namespace can_driver
{
class Battery{
public:
  explicit Battery(std::shared_ptr<rclcpp::Node> node,std::shared_ptr<CanSend> send_queue);
  ~Battery(){};

  enum class 
  RCSRequest {  
    Off = 0,   // 关闭充电模式  
    On = 1, // 打开充电模式 
    Error = -1,  // 错误状态
    CheckCharging = 2  // 检查是否正在充电
    
  };

private:

  void battery_service_responce(
    const BatteryControl::Request::SharedPtr request,
    BatteryControl::Response::SharedPtr response);
  
  rclcpp::Publisher<agv_interfaces::msg::BatteryState>::SharedPtr battery_state_pub_;
  
  rclcpp::Service<agv_interfaces::srv::BatteryControl>::SharedPtr battery_ctrl_server_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  

  int max_checks_ = 50;//充电失败定时用的
  
  BatteryInfo battery_info_;
  
  std::vector<VehicleConfig> vehicles_; 
  std::shared_ptr<can_driver::CanSend> send_queue_;
  std::shared_ptr<rclcpp::Node> node_;
  

};
}

#endif  //!__USBCAN_BATTERY_HPP__