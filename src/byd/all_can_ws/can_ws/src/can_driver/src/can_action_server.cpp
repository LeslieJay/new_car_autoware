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
    std::lock_guard<std::mutex> lock(charge_mutex_);
    int command = request->charging;   // 1 = 开始充电, 0 = 停止充电
    RCLCPP_INFO(this->get_logger(), "收到充电服务请求: charging=%d", command);

    if (command == 1) {
        // 1. 检查是否充满或有充电/电池故障
        if (is_battery_full_.load(std::memory_order_relaxed)) {
            RCLCPP_WARN(this->get_logger(), "电池已充满 (SOC>=100%%)，拒绝下发充电帧");
            response->battery_status = 0;
            response->charging_status = 0;
            return;
        }
        if (is_charge_fault_.load(std::memory_order_relaxed)) {
            RCLCPP_ERROR(this->get_logger(), "检测到底盘存在充电/电池故障，拒绝下发充电帧");
            response->battery_status = 0;
            response->charging_status = 0;
            return;
        }

        // 2. 检查是否已处于充电模式
        if (is_charging_mode_.load(std::memory_order_relaxed)) {
            RCLCPP_INFO(this->get_logger(), "底盘当前已处于充电模式，保持充电使能状态");
            charging_requested_.store(true, std::memory_order_relaxed);
            struct can_frame hold_frame{};
            hold_frame.can_id = 0x201;
            hold_frame.can_dlc = 8;
            hold_frame.data[4] = 0b00100001; // Bit5=1, Bit0=1
            send_queue_->setLatestControlFrame(hold_frame);
            response->battery_status = 1;
            response->charging_status = 1;
            return;
        }

        if (!send_queue_) {
            RCLCPP_ERROR(this->get_logger(), "send_queue_ 未初始化，无法发送充电CAN帧");
            response->battery_status = 0;
            response->charging_status = 0;
            return;
        }

        // 标记已收到充电请求
        charging_requested_.store(true, std::memory_order_relaxed);
        // 立即覆盖当前控制帧，确保没有任何带速度、转向或使能的旧报文被周期任务发送
        struct can_frame init_charge_frame{};
        init_charge_frame.can_id = 0x201;
        init_charge_frame.can_dlc = 8;
        init_charge_frame.data[4] = 0b00100001; // Bit5=1, Bit0=1
        send_queue_->setLatestControlFrame(init_charge_frame);
        bool entered_charging = false;
        const int max_retries = 10;

        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            // 每次下发前检查是否充满或出现故障
            if (is_battery_full_.load(std::memory_order_relaxed)) {
                RCLCPP_WARN(this->get_logger(), "检测到电池已充满，终止下发充电帧");
                break;
            }
            if (is_charge_fault_.load(std::memory_order_relaxed)) {
                RCLCPP_ERROR(this->get_logger(), "检测到充电故障，终止下发充电帧");
                break;
            }

            // 下发充电帧 (0x201 data[4] = 0b00100001: Bit5=1, Bit0=1)
            struct can_frame charge_frame{};
            charge_frame.can_id = 0x201;
            charge_frame.can_dlc = 8;
            charge_frame.data[4] = 0b00100001;
            charging_requested_.store(true, std::memory_order_relaxed);
            send_queue_->setLatestControlFrame(charge_frame);
            send_queue_->push({charge_frame});

            RCLCPP_INFO(this->get_logger(), 
                        "下发第 %d/%d 次充电帧 (0x201 data[4]=0x%02x)，等待3秒检测是否进入充电模式...", 
                        attempt, max_retries, charge_frame.data[4]);

            // 间隔三秒监测是否进入充电模式
            const auto start_wait = std::chrono::steady_clock::now();
            const auto wait_duration = std::chrono::seconds(3);

            while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_wait < wait_duration)) {
                if (is_charging_mode_.load(std::memory_order_relaxed)) {
                    entered_charging = true;
                    break;
                }
                if (is_charge_fault_.load(std::memory_order_relaxed) || 
                    is_battery_full_.load(std::memory_order_relaxed)) {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (entered_charging) {
                RCLCPP_INFO(this->get_logger(), "底盘已成功进入充电模式 (第 %d 次尝试成功)!", attempt);
                break;
            }

            if (is_charge_fault_.load(std::memory_order_relaxed) || 
                is_battery_full_.load(std::memory_order_relaxed)) {
                break;
            }

            if (attempt < max_retries) {
                RCLCPP_WARN(this->get_logger(), "第 %d 次下发充电帧后未进入充电模式，间隔3秒后重发...", attempt);
            }
        }

        if (entered_charging) {
            // 底盘已成功进入充电模式，持续保持 charging_requested_ 为 true，维持下发充电帧 (0x201 data[4]=0x21)
            charging_requested_.store(true, std::memory_order_relaxed);
            struct can_frame hold_frame{};
            hold_frame.can_id = 0x201;
            hold_frame.can_dlc = 8;
            hold_frame.data[4] = 0b00100001; // Bit5=1, Bit0=1 (持续维系充电使能)
            send_queue_->setLatestControlFrame(hold_frame);
            send_queue_->push({hold_frame});
            RCLCPP_INFO(this->get_logger(), "底盘已进入充电模式，持续保持充电使能帧 (0x201 data[4]=0x21)");

            response->battery_status = 1;
            response->charging_status = 1;
        } else {
            charging_requested_.store(false, std::memory_order_relaxed);
            // 失败或遇故障/充满，下发停止充电帧
            struct can_frame stop_frame{};
            stop_frame.can_id = 0x201;
            stop_frame.can_dlc = 8;
            stop_frame.data[4] = 0b00000001;
            send_queue_->setLatestControlFrame(stop_frame);
            send_queue_->push({stop_frame});

            response->battery_status = 0;
            response->charging_status = 0;
            RCLCPP_ERROR(this->get_logger(), "未能进入充电模式 (已重试10次或检测到充满/故障)，返回不能进入充电模式");
        }
    } else if (command == 0) {
        RCLCPP_INFO(this->get_logger(), "收到停止充电服务请求 (command=0)");
        // 1. 立即清除充电请求标志，确保 can_ws 任何地方都不能再下发充电帧
        charging_requested_.store(false, std::memory_order_relaxed);

        if (!send_queue_) {
            RCLCPP_ERROR(this->get_logger(), "send_queue_ 未初始化，无法发送停止充电CAN帧");
            response->battery_status = 0;
            response->charging_status = 0;
            return;
        }

        // 2. 构造停止充电帧 (0x201 data[4] = 0b00000001: Bit5=0, Bit0=1, 速度转向全0, 保持机械抱闸, 断开驱动使能)
        struct can_frame stop_frame{};
        stop_frame.can_id = 0x201;
        stop_frame.can_dlc = 8;
        stop_frame.data[0] = 0;
        stop_frame.data[1] = 0;
        stop_frame.data[2] = 0;
        stop_frame.data[3] = 0;
        stop_frame.data[4] = 0b00000001; // Bit5=0停止充电使能，Bit0=1控制器OK保持抱闸
        stop_frame.data[5] = 0;
        stop_frame.data[6] = 0;
        stop_frame.data[7] = 0;

        send_queue_->setLatestControlFrame(stop_frame);
        send_queue_->push({stop_frame});

        // 3. 确保已经退出了充电模式，如果没有则隔三秒重发，最多十次，等待退出充电模式
        bool exited_charging = false;
        const int max_retries = 10;

        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            // 检查底盘是否已退出充电模式 (0x2A1反馈 is_charging_mode_ == false)
            if (!is_charging_mode_.load(std::memory_order_relaxed)) {
                exited_charging = true;
                RCLCPP_INFO(this->get_logger(), "底盘已确认退出充电模式 (第 %d 次检测成功)!", attempt);
                break;
            }

            // 再次下发停止充电帧
            send_queue_->setLatestControlFrame(stop_frame);
            send_queue_->push({stop_frame});

            RCLCPP_INFO(this->get_logger(),
                        "下发第 %d/%d 次停止充电帧 (0x201 data[4]=0x%02x)，等待3秒检测是否退出充电模式...",
                        attempt, max_retries, stop_frame.data[4]);

            // 间隔三秒监测是否退出充电模式
            const auto start_wait = std::chrono::steady_clock::now();
            const auto wait_duration = std::chrono::seconds(3);

            while (rclcpp::ok() && (std::chrono::steady_clock::now() - start_wait < wait_duration)) {
                if (!is_charging_mode_.load(std::memory_order_relaxed)) {
                    exited_charging = true;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            if (exited_charging) {
                RCLCPP_INFO(this->get_logger(), "底盘已成功退出充电模式 (第 %d 次尝试成功)!", attempt);
                break;
            }

            if (attempt < max_retries) {
                RCLCPP_WARN(this->get_logger(), "第 %d 次下发停止充电帧后底盘仍未退出充电模式，间隔3秒后重发...", attempt);
            }
        }

        // 4. 等待退出充电模式后，返回给 ROS 平台
        if (exited_charging || !is_charging_mode_.load(std::memory_order_relaxed)) {
            response->battery_status = 0;
            response->charging_status = 0;
            RCLCPP_INFO(this->get_logger(), "已成功退出充电模式，返回给ROS平台: battery_status=0, charging_status=0");
        } else {
            response->battery_status = 1;
            response->charging_status = 1;
            RCLCPP_ERROR(this->get_logger(), "未能退出充电模式 (已重试10次)，返回给ROS平台失败状态");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "无效的充电命令: %d", command);
        response->battery_status = 0;
        response->charging_status = 0;
    }
}

}  // namespace can_driver