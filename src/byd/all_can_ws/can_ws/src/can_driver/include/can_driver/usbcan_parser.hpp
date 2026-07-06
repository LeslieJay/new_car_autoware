/*
 * @Author: LiFang6606397
 * @Date: 2024-02-26 16:52:45
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2025-10-11 10:07:18
 * @FilePath: /work_ws/src/usbcan/include/usbcan/usbcan_parser.hpp
 * @Description: 
 *    input : CAN 
 *    output : 里程计, tf变换, 轮子的转速, 电池信息
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */

#ifndef __USBCAN_PARSER__H__
#define __USBCAN_PARSER__H__


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include <queue>
#include <mutex>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ref_slam_interface/action/ctrl_fork.hpp"
#include <linux/can.h>
#include <linux/can/raw.h> 
#include "can_driver/can_receiver.hpp"
#include "usbcan/usbcan_utils.hpp"

#define MAX_CHANNELS  2
#define CHECK_POINT  10
#define RX_WAIT_TIME  10
#define RX_BUFF_SIZE  100

#define ROT_ANG_SPEED_LIMIT 0.3

#define PI 3.14159265358979323846

/** pose of base_link */
extern double distance_x;
extern double distance_y;
extern double theta_k;

/** 货叉当前高度 */
extern double fork_height_;

/** 舵轮当前姿态(角度) */
extern double steer_ang_;

extern std::mutex que_mtx_;
extern bool ctl_queue_;

class CANParser : public rclcpp::Node{
public:
  explicit CANParser(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CANParser();

  using ActCtrlFork = ref_slam_interface::action::CtrlFork;
  using GH_CtrlFork = rclcpp_action::ServerGoalHandle<ActCtrlFork>;

  enum LiftMode{
    RUN_AND_LIFT,
    STOP_AND_LIFT
  };
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr 
  getPub() const {
    return publisher_;
  }
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> 
  getTF() const {
    return tf_broadcaster_;
  }

  std::shared_ptr<std::queue<GeneralFrame>>
  getQ() const {
    return cmd_queue_;
  }

private:
  // 处理接收到的目标值
  rclcpp_action::GoalResponse 
  handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActCtrlFork::Goal> goal);

  // 处理取消任务
  rclcpp_action::CancelResponse 
  handle_cancel(std::shared_ptr<GH_CtrlFork> goal_handle);

  // 产生连续反馈和最终结果
  void execute(std::shared_ptr<GH_CtrlFork> goal_handle);

  void handle_accepted(std::shared_ptr<GH_CtrlFork> goal_handle);

  void topic_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

  void shield_timer_cb();

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscript_;
  rclcpp_action::Server<ref_slam_interface::action::CtrlFork>::SharedPtr server_;

  LiftMode liftmode_ = LiftMode::RUN_AND_LIFT;
  std::atomic<bool> is_fork_action_running_ = false;
  std::shared_ptr<std::queue<GeneralFrame>> cmd_queue_;

  int speed_upper_bound_;  // 舵轮转动线速度上限

  double eps_ = 1e-4;   // 速度绝对值小于此值时视为0
  double fork_height_error_ = 10.0;   // 货叉高度误差允许范围/mm
  double rot_in_place_error_ = 1e-2;  // 原地旋转时, 当前舵轮角度与目标角度的误差 允许范围/度

	int16_t angle_command_;   // epec需要的指令格式。单位：0.01度
	int16_t speed_command_;   // epec需要的指令格式。单位：mm/s
  double goal_fork_height_;   // 记录最近一次的货叉目标高度

  std::string shiled_command_;   // 屏蔽安全信号指令, 长度必须为8. 每60ms发送一次, 从左到右为0x344报文的0-7位.
  GeneralFrame shield_cmd_; // 0x344报文
  rclcpp::TimerBase::SharedPtr shiled_timer_;   // 用于定时发送屏蔽报文的定时器

  int steer_enc_offset_ = 65280;  // 修正舵轮物理角度的offset
};

#endif  //!__USBCAN_PARSER__H__