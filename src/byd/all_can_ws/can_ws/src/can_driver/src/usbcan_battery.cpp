/*
 * @Author: LiFang6606397
 * @Date: 2024-07-15 10:24:49
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2025-09-26 14:43:12
 * @FilePath: /work_ws/src/usbcan/src/usbcan_battery.cpp
 * @Description: Battery类的代码实现
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */
#include "can_driver/usbcan_battery.hpp"
#include <linux/can.h>
#include <linux/can/raw.h> 
extern std::shared_ptr<can_driver::CanSend> send_queue_;

Battery::Battery(std::shared_ptr<CANParser> node) : node_(node)
{
	RCLCPP_INFO(node_->get_logger(), "Battery node started");

	battery_ctrl_server_ = node_->create_service<BatteryControl>("battery_control", std::bind(&Battery::battery_ctrl_callback, this, _1, _2));

	battery_state_pub_ = node_->create_publisher<agv_interfaces::msg::BatteryState>("battery", 10);

	timer_ = node_->create_wall_timer(1s, std::bind(&Battery::battry_timer_callback, this));
}


void
Battery::battery_ctrl_callback(
  const BatteryControl::Request::SharedPtr  request,
        BatteryControl::Response::SharedPtr response){

  RCLCPP_INFO_STREAM(node_->get_logger(), 
    "Battery control request received : " << request->charging);
  std::vector<struct can_frame> send_frames;
  // 在指定时间内检查 充电是否允许 是否变更到 指定状态 的函数. 
  // 变更成功,随时退出; 超时则进行错误处理
  auto read_and_check_status = 
    [&](int target_status, const std::string& err_msg){
      // 指定时间内 等待 状态变更
      int check_count = 0;
      while(check_count < max_checks_){
        if(battery_info_.charge_allowed == target_status){break;}
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++check_count;
      }
      // 指定时间后仍未更改为 指定状态, 进行错误处理
      if(battery_info_.charge_allowed != target_status){
        RCLCPP_ERROR(
          rclcpp::get_logger("Battery Control Server"),
          "%s", err_msg.c_str());
      }
    };

  RCSRequest rcs_request;

  switch(request->charging){
    case 0:
      rcs_request = RCSRequest::Off;
      break;
    case 1:
      rcs_request = RCSRequest::On;
      break;
    case 2:
      rcs_request = RCSRequest::CheckCharging;
      break;
    default:
      rcs_request = RCSRequest::Error;
      break;
  }

  switch(rcs_request){
    case RCSRequest::On:

      // 不允许充电时, 首先转为允许充电状态
      if (battery_info_.charge_allowed == 0) {
          RCLCPP_INFO(node_->get_logger(), "AGV从放电模式转换成充电模式");
          {
            std::lock_guard<std::mutex> lock(que_mtx_);
            send_frames.push_back(gen_0x404(0x04, 0));
          }
          read_and_check_status(1, "改变电池状态为充电模式后, 读取AGV电池状态超时");
      }
      
      // 等待 状态处于充电状态
      read_and_check_status(1, "读取AGV电池状态超时");

      // 允许充电状态, 直接进行返回值填充并发送
      response->battery_status = battery_info_.charge_allowed;
      response->charging_status = battery_info_.charge_status;
      RCLCPP_INFO(node_->get_logger(), "成功转换为充电模式");
      
      break;

    case RCSRequest::Off:
      // TODO: 为什么不先检查是否在充电? 若外部在放电时直接发送关闭充电请求会怎样?
      {
        std::lock_guard<std::mutex> lock(que_mtx_);
        send_frames.push_back(gen_0x404(0x00, 0));
      }
      read_and_check_status(0, "改变电池为放电状态后, 读取AGV电池状态超时");
      response->battery_status = battery_info_.charge_allowed;
      response->charging_status = battery_info_.charge_status;
      RCLCPP_INFO(node_->get_logger(), "成功转换为放电模式");
      
      break;

    case RCSRequest::CheckCharging:
      read_and_check_status(0, "改变电池为放电状态后, 读取AGV电池状态超时");
      response->battery_status = battery_info_.charge_allowed;
      response->charging_status = battery_info_.charge_status;
      RCLCPP_INFO(node_->get_logger(), "成功读取到充电状态");  

      break;

    default:
      response->battery_status = -2;
      response->charging_status = -2;
      RCLCPP_INFO(node_->get_logger(), "RCS的请求非法");
      
      break;
    
  }
  send_queue_->push(send_frames);
  // ULONG result = VCI_Transmit(
  //   gDevType, 
  //   gDevIdx, 
  //   0,                      // 0=CAN0
  //   &send_frames[0],     // 【关键】直接传入 vector 的数据指针，等价于 
  //   static_cast<UINT>(send_frames.size()) // 发送帧的数量
  // );
  RCLCPP_INFO_STREAM(node_->get_logger(), "AGV电池是否允许充电: " << battery_info_.charge_allowed);
}

void 
Battery::battry_timer_callback(){
  BatteryState msg;
  msg.battery_status = battery_info_.charge_allowed;
  msg.battery_level  = battery_info_.battery_level;
  msg.total_current  = battery_info_.total_current;
  msg.total_voltage  = battery_info_.total_voltage;
  battery_state_pub_->publish(msg);
}
