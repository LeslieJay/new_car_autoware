/*
 * @Author: LiFang6606397
 * @Date: 2024-07-15 08:49:32
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2025-09-12 11:53:44
 * @FilePath: /work_ws/src/usbcan/include/usbcan/usbcan_battery.hpp
 * @Description: 添加电池状态监控和电池充电控制功能. 基于涂兆诚工代码重构而来.
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */

#ifndef __USBCAN_BATTERY_HPP__
#define __USBCAN_BATTERY_HPP__


#include "ref_slam_interface/srv/battery_control.hpp"
#include "ref_slam_interface/msg/battery_state.hpp"
#include "usbcan_parser.hpp"
#include "usbcan/usbcan_utils.hpp"

extern std::shared_ptr<can_driver::CanSend> send_queue_;

using ref_slam_interface::srv::BatteryControl;
using namespace std::placeholders;
using namespace std::chrono_literals;

class Battery{
public:
  explicit Battery(std::shared_ptr<CANParser> node);
  ~Battery(){};

  struct BatteryInfo{
    int       charge_status;      // 充电状态. 0：未充电; 1：正在充电; 2：充电完成; 3：充电中止
    int       charge_allowed;     // 充电是否允许. 0 不允许; 1 允许
    int    discharge_allowed;     // 放电是否允许. 0 不允许; 1 允许
    

    double battery_level;   // 电池组当前容量指数 0-100%
    double total_voltage;   // 电池组当前总电压 0-1000V
    double total_current;   // 电池组当前总电流 0-500A
  };

  enum class RCSRequest {  
    Off = 0,   // 关闭充电模式  
    On = 1, // 打开充电模式 
    CheckCharging = 2,  // 检查是否正在充电
    Error = -1  // 错误状态
  };

private:

  void battery_ctrl_callback(
    const BatteryControl::Request::SharedPtr request,
    BatteryControl::Response::SharedPtr response);

  void battry_timer_callback();


  rclcpp::Publisher<agv_interfaces::msg::BatteryState>::SharedPtr battery_state_pub_;
  
  rclcpp::Service<ref_slam_interface::srv::BatteryControl>::SharedPtr battery_ctrl_server_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  std::shared_ptr<CANParser> node_;

  int max_checks_ = 50;
};

extern Battery::BatteryInfo battery_info_;

#endif  //!__USBCAN_BATTERY_HPP__




