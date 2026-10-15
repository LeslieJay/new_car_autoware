#include "can_driver/can_action_server.hpp"
#include <linux/can.h>
#include <linux/can/raw.h> 
#include "can_driver/can_send.hpp"
#include <thread>
#include <chrono>

extern std::atomic<int> hook;

namespace can_driver
{
extern std::shared_ptr<CanSend> send_queue_;
extern std::atomic<uint8_t> hook_target_cmd_;

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

    auto fork_goal_height = goal->fork_goal_height;
    if(fork_goal_height > 100) fork_goal_height = fork_goal_height - 100;

    int signal = goal->to_fork_signal;
    
    RCLCPP_INFO(this->get_logger(), 
                "开始执行挂钩: 目标高度=%d, 信号=%d", fork_goal_height, signal);

    if (goal_handle->is_canceling()) {
        result->finish = false;
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "挂钩目标在开始执行前被取消");
        return;
    }

    if (fork_goal_height != 1 && fork_goal_height != 2) {
        RCLCPP_ERROR(this->get_logger(), 
                    "RCS挂钩指令异常: 不支持的目标高度 %d", 
                    fork_goal_height);
        result->finish = false;
        goal_handle->abort(result);
        return;
    }

    if (!send_queue_) {
        RCLCPP_ERROR(this->get_logger(), 
                    "send_queue_ 未初始化，无法发送挂钩CAN帧");
        result->finish = false;
        goal_handle->abort(result);
        return;
    }

    const uint8_t target_cmd = static_cast<uint8_t>(fork_goal_height);

    // 检查是否已经在目标位置 (hook: 2为起升到位, -2为下降到位)
    const int initial_hook = hook.load(std::memory_order_relaxed);
    if ((target_cmd == 1 && initial_hook == 2) || (target_cmd == 2 && initial_hook == -2)) {
        RCLCPP_INFO(this->get_logger(), "底盘挂钩已处于目标位置 (hook=%d)，无需重复动作", initial_hook);
        feedback->fork_height = (target_cmd == 1) ? 1 : 2;
        goal_handle->publish_feedback(feedback);
        result->finish = true;
        goal_handle->succeed(result);
        return;
    }

    RCLCPP_INFO(this->get_logger(), 
                "下达挂钩%s命令 (CAN ID:0x401, data[4]=%d)", 
                (target_cmd == 1 ? "上升" : "下降"), target_cmd);

    // 1. 设置全局挂钩目标命令，确保 can_receiver 组装 0x401 报文时 data[4] 保持该值
    hook_target_cmd_.store(target_cmd, std::memory_order_relaxed);

    // 2. 持续以周期（20Hz / 50ms）检测到位并推送挂钩帧（防 control_cmd 未发布导致无 0x401）
    const auto start_time = std::chrono::steady_clock::now();
    const auto total_duration = std::chrono::seconds(5); // 最长5秒超时
    rclcpp::Rate rate(20);  // 20Hz
    bool reached_target = false;

    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_time < total_duration)) {
        if (goal_handle->is_canceling()) {
            hook_target_cmd_.store(0, std::memory_order_relaxed);
            // 立即下发一帧复位停止帧
            std::vector<struct can_frame> cancel_frames;
            struct can_frame cancel_frame{};
            cancel_frame.can_id = 0x401;
            cancel_frame.can_dlc = 8;
            cancel_frames.push_back(cancel_frame);
            send_queue_->push(cancel_frames);

            result->finish = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "挂钩目标在执行中被取消");
            return;
        }

        // 检查 0x181 反馈是否到位 (hook: 2为起升到位, -2为下降到位)
        const int current_hook = hook.load(std::memory_order_relaxed);
        if ((target_cmd == 1 && current_hook == 2) || (target_cmd == 2 && current_hook == -2)) {
            RCLCPP_INFO(this->get_logger(), 
                        "检测到底盘0x181反馈挂钩已到位 (hook=%d)", current_hook);
            reached_target = true;
            break;
        }

        // 持续推送挂钩帧，保证即使在停车无 control_cmd 时也有挂钩帧下发
        std::vector<struct can_frame> send_frames;
        struct can_frame v_frame{};
        v_frame.can_id = 0x401;
        v_frame.can_dlc = 8;
        v_frame.data[4] = target_cmd;
        send_frames.push_back(v_frame);
        send_queue_->push(send_frames);

        feedback->fork_height = (current_hook == 2 || current_hook == 1) ? 1 : 2;
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }

    // 3. 动作结束（到位或超时），复位全局挂钩命令为 0
    hook_target_cmd_.store(0, std::memory_order_relaxed);

    // 下发复位帧 (data[4]=0)
    std::vector<struct can_frame> reset_frames;
    struct can_frame reset_frame{};
    reset_frame.can_id = 0x401;
    reset_frame.can_dlc = 8;
    reset_frames.push_back(reset_frame);
    send_queue_->push(reset_frames);

    result->finish = reached_target;
    if (reached_target) {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "挂钩动作执行成功完成 (到位退出)");
    } else {
        RCLCPP_WARN(this->get_logger(), "挂钩动作在5秒内未检测到到位信号 (超时退出)");
        goal_handle->succeed(result);
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