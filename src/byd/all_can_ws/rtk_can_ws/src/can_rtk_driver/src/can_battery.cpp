/*
 * @Author: du.xiaoying1
 * @Date: 2025-08-13 16:47:28
 * @LastEditors: dxy
 * @LastEditTime: 2025-08-19 14:24:23
 * @FilePath: /qr_agv_0627_r/src/can_driver/src/can_battery.cpp
 * @Description: 
 * 
 * Copyright (c) 2025 by du.xiaoying1 , All Rights Reserved. 
 */
/***
 * @Author: lwr
 * @Date: 2025-03-05 08:53:16
 * @LastEditTime: 2025-03-07 09:24:11
 * @LastEditors: lwr
 * @Description: 电池控制实现类
 * @FilePath: /qr_agv/src/usbcan/src/usbcan_battery.cpp
 */
#include "can_driver/can_battery.hpp"
using namespace can_driver;
Battery::Battery(std::shared_ptr<rclcpp::Node> node,std::shared_ptr<CanSend> send_queue) : node_(node),vehicles_(AGVConfigManager::getInstance().getVehicles())
,send_queue_(std::move(send_queue))
{
  RCLCPP_INFO(rclcpp::get_logger("Battery node"), "Battery node started");

  battery_ctrl_server_ =
      node_->create_service<BatteryControl>(
          vehicles_[0].name + "/" + vehicles_[0].type + "/" + vehicles_[0].id + "/battery_control",
          std::bind(&Battery::battery_service_responce, this, _1, _2));
}

void Battery::battery_service_responce(
    const BatteryControl::Request::SharedPtr request,
    BatteryControl::Response::SharedPtr response)
{

  RCLCPP_INFO_STREAM(rclcpp::get_logger("Battery node"),
                     "充电请求接收 : " << request->charging);

  RCSRequest rcs_request;

  switch (request->charging)
  {
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

  switch (rcs_request)
  {
  case RCSRequest::On:

    RCLCPP_INFO(rclcpp::get_logger("Battery node"), "AGV从不允许充电模式转换为允许充电模式");
    {

      // 需不需要锁??
      std::vector<struct can_frame> send_frames;
      struct can_frame frame_404 = {
          .can_id = 0x404,
          .can_dlc = 8,
          .data = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
      send_frames.push_back(frame_404);
      // 添加到队列
      send_queue_->push(send_frames);
    }

    // 允许充电状态, 直接进行返回值填充并发送
    response->battery_status = 1;
    response->charging_status = 1;
    RCLCPP_INFO(rclcpp::get_logger("Battery node"), "成功转换为充电模式");
    break;

  case RCSRequest::Off:
    //
    {
      std::vector<struct can_frame> send_frames;
      struct can_frame frame_404 = {
          .can_id = 0x404,
          .can_dlc = 8,
          .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
      send_frames.push_back(frame_404);
      // 添加到队列
      send_queue_->push(send_frames);
    }
    //   read_and_check_status(1, "改变电池为不允许充电");
    response->battery_status = 0;
    response->charging_status = 0;
    RCLCPP_INFO(rclcpp::get_logger("Battery node"), "成功转换为不允许充电模式");

    break;

  default:
    response->battery_status = -2;
    response->charging_status = -2;
    RCLCPP_INFO(rclcpp::get_logger("Battery node"), "RCS的请求非法");

    break;
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("Battery node"), "AGV电池是否允许充电: " << battery_info_.charge_allowed);
}