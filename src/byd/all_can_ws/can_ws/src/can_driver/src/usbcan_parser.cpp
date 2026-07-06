/*
 * @Author: LiFang6606397
 * @Date: 2024-02-26 16:52:45
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2025-10-11 11:02:30
 * @FilePath: /work_ws/src/usbcan/src/usbcan_parser.cpp
 * @Description: CANParser类的代码实现
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */

#include "can_driver/usbcan_parser.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

CANParser::CANParser(const rclcpp::NodeOptions & options)
: rclcpp::Node("usbcan_parser", options){
  // TransformBroadcaster 会自动使用节点的 remapping
  // 通过 launch 文件中的 remapping ('/tf', 'tf')，它会发布到命名空间下的 tf 话题
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  // subscript_ = this->create_subscription<geometry_msgs::msg::Twist>(
  //   "cmd_vel", 1, std::bind(&CANParser::topic_callback, this, _1));
  // 原版货叉控制的server，现在换成hook，放在hook_action_server.cpp中
  // server_ = rclcpp_action::create_server<ActCtrlFork>(
  //   this, "fork_server",
  //   std::bind(&CANParser::handle_goal,this,_1,_2),
  //   std::bind(&CANParser::handle_cancel,this,_1),
  //   std::bind(&CANParser::handle_accepted,this,_1));
  cmd_queue_ = std::make_shared<std::queue<GeneralFrame>>();

  angle_command_ = 0;
  speed_command_ = 0;
  goal_fork_height_ = 0;

  this->declare_parameter("speed_upper_bound", rclcpp::ParameterValue(2500));
  this->get_parameter("speed_upper_bound", speed_upper_bound_);

  this->declare_parameter("fork_height_error", rclcpp::ParameterValue(10.0));
  this->get_parameter("fork_height_error", fork_height_error_);

  this->declare_parameter("rot_in_place_error", rclcpp::ParameterValue(1e-2));
  this->get_parameter("rot_in_place_error", rot_in_place_error_);

  this->declare_parameter("shiled_command", rclcpp::ParameterValue("10000000"));
  this->get_parameter("shiled_command", shiled_command_);

  this->declare_parameter("steer_enc_offset", rclcpp::ParameterValue(65100));
  this->get_parameter("steer_enc_offset", steer_enc_offset_);

  shield_cmd_ = gen_0x344(shiled_command_);
  shiled_timer_ = this->create_wall_timer(60ms, std::bind(&CANParser::shield_timer_cb, this));
}

CANParser::~CANParser(){}

// void CANParser::topic_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){

//   RCLCPP_INFO_STREAM(this->get_logger(),
//     "From cmd_vel: linear.x:\t" << msg->linear.x << "\t" << "angular.z:\t" << msg->angular.z);

//   double rot_vel, rot_ang;

//   // 套用双轮差速模型，然后反计算舵轮的速度和转角
//   if(std::abs(msg->linear.x) < eps_ && std::abs(msg->angular.z) < eps_){  // 特殊情况0：静止不动
//     rot_vel = 0;
//     rot_ang = 0;
//   }else if(std::abs(msg->angular.z) < eps_){   // 特殊情况1：直行
//     rot_vel = msg->linear.x;
//     rot_ang = 0;
//   }else if(std::abs(msg->linear.x) < eps_){   // 特殊情况2：原地旋转
//     double target_angle = msg->angular.z > 0 ? 90 : -90;
//     double angle_diff = std::abs(std::abs(target_angle) - std::abs(steer_ang_));
//     // 检查当前舵轮姿态是否到达目标姿态
//     if(angle_diff > rot_in_place_error_){
//       rot_vel = 0;    // 若未到达, 则速度置零, 仅转动舵轮
//     }else{
//       rot_vel = std::min(std::abs(msg->angular.z * WHEELBASE), ROT_ANG_SPEED_LIMIT);    // 若已到达, 则开始赋予线速度
//     }
//     rot_ang = target_angle * PI / 180;
//   }else{   // 一般情况
//     // 首先计算差速模型回转半径，回转半径是矢量，符号表示回转中心在左侧还是右侧(y轴正半轴或负半轴)
//     double rot_d = msg->linear.x / msg->angular.z;
//     double rot_r = std::sqrt(std::pow(rot_d, 2) + std::pow(WHEELBASE, 2));  // 计算舵轮的回转半径
//     if(rot_d < 0){rot_r = -rot_r;}  // 判断回转中心方向。回转中心在右侧时, 舵轮回转半径为负
//     rot_vel = msg->angular.z * rot_r; // 计算舵轮的转动速度 m/s
//     rot_ang = atan(WHEELBASE / rot_d);  // 计算舵轮的朝向角 rad
//   }

//   // 转化为角度并乘100，转化为epec需要的指令格式
//   angle_command_ = 100 * rot_ang * 180 / PI;

//   // 转化为epec需要的指令格式。单位：mm/s
//   speed_command_ = rot_vel * 1000;
  
//   // 基本控制驱动方实行的速度限制
//   speed_command_ = std::min(speed_command_, (int16_t)speed_upper_bound_);
//   speed_command_ = std::max(speed_command_, (int16_t)-speed_upper_bound_);

//   // 舵轮转角超过阈值时, 主动降低转动速度
//   // TODO: 分阶段降低; 参数化
//   if(std::fabs(angle_command_) >= 1000){
//     int16_t min_speed = std::min(std::abs(speed_command_), 300);
//     speed_command_ = speed_command_ < 0 ? -min_speed : min_speed;
//   }

//   {
//     std::lock_guard<std::mutex> lock(que_mtx_);

//     // 货叉动作运行时判断运行模式，若必须停止才能控制货叉，则更改速度和角度指令
//     if(is_fork_action_running_.load()){
//       if(liftmode_==LiftMode::STOP_AND_LIFT){
//         // 保持上一刻的姿态，防止舵轮姿态总是归零
//         cmd_queue_->push(gen_0x404(0x08, 0));
//         cmd_queue_->push(gen_0x304(0, steer_ang_*100, steer_enc_offset_, goal_fork_height_));
//       }else{
//         // RUN_AND_LIFT 模式下，舵轮运动不受影响
//         cmd_queue_->push(gen_0x404(0x0B, 0));
//         cmd_queue_->push(gen_0x304(speed_command_, angle_command_, steer_enc_offset_, goal_fork_height_));
//       }
//     }else{
//       // 货叉动作不运行时一律不允许控制货叉；
//       cmd_queue_->push(gen_0x404(0x03, 0x00));
//       cmd_queue_->push(gen_0x304(speed_command_, angle_command_, steer_enc_offset_, fork_height_));
//     }
//   }
// }

// // 处理接收到的目标值
// rclcpp_action::GoalResponse
// CANParser::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActCtrlFork::Goal> goal){
//   (void)uuid;
//   if(goal->to_fork_signal != 1){
//     RCLCPP_INFO(this->get_logger(), "To use fork, the to_fork_signal must be 1.");
//     return rclcpp_action::GoalResponse::REJECT;
//   }
//   RCLCPP_INFO(this->get_logger(), "Goal accepted.");
//   return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

// // 处理取消任务
// rclcpp_action::CancelResponse 
// CANParser::handle_cancel(std::shared_ptr<GH_CtrlFork> goal_handle){
//   (void) goal_handle;
//   RCLCPP_INFO(this->get_logger(),"Goal canceled.");
//   return rclcpp_action::CancelResponse::ACCEPT;
// }

// void 
// CANParser::execute(std::shared_ptr<GH_CtrlFork> goal_handle){

//   is_fork_action_running_.store(true);
//   // 发送控制货叉指令
//   {
//     std::lock_guard<std::mutex> lock(que_mtx_);

//     goal_fork_height_ = goal_handle->get_goal()->fork_goal_height;

//     cmd_queue_->push(gen_0x204(0x17, 0, 0x01)); // TODO: 这里的车辆ID是否不应该写死
//     if(liftmode_==LiftMode::STOP_AND_LIFT){
//       cmd_queue_->push(gen_0x404(0x08, 0));
//       cmd_queue_->push(gen_0x304(0, steer_ang_*100, steer_enc_offset_, goal_fork_height_));  // 保持上一刻的姿态，保持稳定
//     }
//     cmd_queue_->push(gen_0x404(0x0B, 0));
//     cmd_queue_->push(gen_0x304(0x03, 0x01, steer_enc_offset_, goal_fork_height_));
//   }
//   auto feedback = std::make_shared<ActCtrlFork::Feedback>();
//   auto result = std::make_shared<ActCtrlFork::Result>();

//   /** TODO: 这里可能导致 距离差在10mm内时，is_fork_action_running_ 标志位变为false，导航舵轮提前发生运动 */
//   while(std::abs(goal_handle->get_goal()->fork_goal_height - fork_height_) >= fork_height_error_ && rclcpp::ok()){
//     feedback->fork_height = fork_height_;
//     goal_handle->publish_feedback(feedback);

//     if(goal_handle->is_canceling()){
//       {
//         // 发送关闭货叉使能 ----> 队列中放入停止命令
//         std::lock_guard<std::mutex> lock(que_mtx_);
//         cmd_queue_->push(gen_0x404(0x03, 0));
//         RCLCPP_INFO_STREAM(this->get_logger(), "Aborting fork control");
//       }
      
//       result->finish = false;
//       goal_handle->canceled(result);

//       is_fork_action_running_.store(false);

//       RCLCPP_INFO_STREAM(this->get_logger(), "Goal Aborted.");
//       return;
//     }
//     std::this_thread::sleep_for(50ms);
//   }

//   if(rclcpp::ok()){
//     result->finish = true;
//     goal_handle->succeed(result);
//     RCLCPP_INFO_STREAM(this->get_logger(), "Goal succeed. Current heght: " << fork_height_);
//   }
//   is_fork_action_running_.store(false);
//   return;
// }

// void 
// CANParser::handle_accepted(std::shared_ptr<GH_CtrlFork> goal_handle){ 
//   std::thread{std::bind(&CANParser::execute, this, goal_handle)}.detach();
// }


void
CANParser::shield_timer_cb(){
  std::lock_guard<std::mutex> lock(que_mtx_);
  cmd_queue_->push(shield_cmd_);
}
