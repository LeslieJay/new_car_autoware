#include "can_driver/can_action_server.hpp"
#include <linux/can.h>
#include <linux/can/raw.h> 
#include "can_driver/can_send.hpp"
#include <thread>
#include <chrono>

namespace can_driver
{
extern std::shared_ptr<CanSend> send_queue_;

using CtrlFork = ref_slam_interface::action::CtrlFork;
using GoalHandleFork = rclcpp_action::ServerGoalHandle<CtrlFork>;
using BatteryControl = ref_slam_interface::srv::BatteryControl;

CanActionServer::CanActionServer(const rclcpp::NodeOptions & options)
: Node("can_action_server", options)
{
    // 1. 创建挂钩动作服务器（保持原有功能不变）
    fork_action_server_ = rclcpp_action::create_server<CtrlFork>(
        this,
        "fork_server",
        std::bind(&CanActionServer::handle_fork_goal, this, 
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&CanActionServer::handle_fork_cancel, this, 
                  std::placeholders::_1),
        std::bind(&CanActionServer::handle_fork_accepted, this, 
                  std::placeholders::_1));

    // 2. 创建充电服务服务器（服务名需与 RequestChargingClient 一致）
    charge_service_ = this->create_service<BatteryControl>(
        "battery_control",   // 或 "charging_server"，取决于客户端实际调用的服务名
        std::bind(&CanActionServer::handle_charge_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    current_height_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "CAN动作服务器已启动（挂钩 + 充电）");
}

// ================== 挂钩动作服务器回调（与原ForkActionServer完全一致）==================
rclcpp_action::GoalResponse CanActionServer::handle_fork_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CtrlFork::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "收到挂钩目标: signal=%d, target_height=%d",
                goal->to_fork_signal, goal->fork_goal_height);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CanActionServer::handle_fork_cancel(
    const std::shared_ptr<GoalHandleFork> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "处理ROS平台取消挂钩动作要求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CanActionServer::handle_fork_accepted(
    const std::shared_ptr<GoalHandleFork> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "接受挂钩目标，开始在新线程中执行");
    std::thread{std::bind(&CanActionServer::execute_fork, this, goal_handle)}.detach();
}

void CanActionServer::execute_fork(
    const std::shared_ptr<GoalHandleFork> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CtrlFork::Feedback>();
    auto result = std::make_shared<CtrlFork::Result>();

    double target = goal->fork_goal_height;
    int signal = goal->to_fork_signal;
    
    RCLCPP_INFO(this->get_logger(), 
                "开始执行挂钩: 目标高度=%.2f, 信号=%d", target, signal);

    if (goal_handle->is_canceling()) {
        result->finish = false;
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "挂钩目标在开始执行前被取消");
        return;
    }

    if (goal->fork_goal_height != 0) {
        std::vector<struct can_frame> send_frames;
        struct can_frame v_frame{};
        v_frame.can_id = 0x401;
        v_frame.can_dlc = 8;
        for(int i = 0; i < 8; i++) {
            v_frame.data[i] = 0;
        }

        auto fork_goal_height = goal->fork_goal_height;
        if(fork_goal_height == 1) {
            v_frame.data[4] = 1;
            RCLCPP_INFO(this->get_logger(), 
                       "下达挂钩升降到位置1命令 (CAN ID:0x401, data[4]=1)");
        }
        else if (fork_goal_height == 2) {
            v_frame.data[4] = 2;
            RCLCPP_INFO(this->get_logger(), 
                       "下达挂钩升降到位置2命令 (CAN ID:0x401, data[4]=2)");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), 
                        "RCS挂钩指令异常: 不支持的目标高度 %d", 
                        fork_goal_height);
            result->finish = false;
            goal_handle->abort(result);
            return;
        }
        
        send_frames.push_back(v_frame);
        if (send_queue_) {
            send_queue_->push(send_frames);
            RCLCPP_INFO(this->get_logger(), "挂钩CAN帧已成功发送到发送队列");
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "send_queue_ 未初始化，无法发送挂钩CAN帧");
            result->finish = false;
            goal_handle->abort(result);
            return;
        }
        
        rclcpp::sleep_for(std::chrono::seconds(3));
        result->finish = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "挂钩动作执行成功完成");
    }
    else {
        RCLCPP_WARN(this->get_logger(), "挂钩目标高度为0，不执行任何操作");
        result->finish = false;
        goal_handle->abort(result);
    }
}

// ================== 充电服务回调 ==================
void CanActionServer::handle_charge_request(
    const std::shared_ptr<BatteryControl::Request> request,
    std::shared_ptr<BatteryControl::Response> response)
{
    int command = request->charging;   // 1 = 开始充电, 0 = 停止充电
    RCLCPP_INFO(this->get_logger(), "收到充电服务请求: charging=%d", command);

    std::vector<struct can_frame> send_frames;
    struct can_frame charge_frame{};
    charge_frame.can_id = 0x201;
    charge_frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        charge_frame.data[i] = 0;
    }

    if (command == 1) {
        // 开始充电：bit5=1, bit0=1 => 0b00100001 = 0x21
        charge_frame.data[4] = 0b00100001;
        RCLCPP_INFO(this->get_logger(), "下达开始充电命令 (0x201 data[4]=0x%02x)", charge_frame.data[4]);
    } else if (command == 0) {
        // 停止充电：bit5=0, bit0=1 => 0b00000001 = 0x01
        charge_frame.data[4] = 0b00000001;
        RCLCPP_INFO(this->get_logger(), "下达停止充电命令 (0x201 data[4]=0x%02x)", charge_frame.data[4]);
    } else {
        RCLCPP_ERROR(this->get_logger(), "无效的充电命令: %d", command);
        // 填充响应字段（根据实际情况赋值，这里简单置0表示失败）
        response->battery_status = 0;
        response->charging_status = 0;
        return;
    }

    send_frames.push_back(charge_frame);
    if (send_queue_) {
        send_queue_->push(send_frames);
        // 成功：假设发送即成功，设置充电状态为1，电池状态保持不变（此处仅为示例）
        response->battery_status = 1;    // 1=充电中（需与实际协议一致）
        response->charging_status = 1;   // 1=成功
        RCLCPP_INFO(this->get_logger(), "充电CAN帧已成功发送到发送队列");
    } else {
        RCLCPP_ERROR(this->get_logger(), "send_queue_ 未初始化，无法发送充电CAN帧");
        response->battery_status = 0;
        response->charging_status = 0;
    }
}

}  // namespace can_driver