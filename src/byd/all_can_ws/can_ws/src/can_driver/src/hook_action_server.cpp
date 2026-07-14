#include "can_driver/hook_action_server.hpp"
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

// 构造函数
ForkActionServer::ForkActionServer(const rclcpp::NodeOptions & options)
: Node("fork_action_server", options)
{
    action_server_ = rclcpp_action::create_server<CtrlFork>(
        this,
        "fork_server",
        std::bind(&ForkActionServer::handle_goal, this, 
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&ForkActionServer::handle_cancel, this, 
                  std::placeholders::_1),
        std::bind(&ForkActionServer::handle_accepted, this, 
                  std::placeholders::_1));

    current_height_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "挂钩动作服务器已启动");
}

// 目标接收回调
rclcpp_action::GoalResponse ForkActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CtrlFork::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "收到目标: signal=%d, target_height=%d",
                goal->to_fork_signal, goal->fork_goal_height);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消回调
rclcpp_action::CancelResponse ForkActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleFork> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "处理ros平台取消挂钩动作要求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// 接受目标后的处理
void ForkActionServer::handle_accepted(
    const std::shared_ptr<GoalHandleFork> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "接受目标，开始在新线程中执行");
    std::thread{std::bind(&ForkActionServer::execute, this, goal_handle)}.detach();
}

// 实际执行逻辑
void ForkActionServer::execute(
    const std::shared_ptr<GoalHandleFork> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CtrlFork::Feedback>();
    auto result = std::make_shared<CtrlFork::Result>();

    double target = goal->fork_goal_height;
    int signal = goal->to_fork_signal;
    
    RCLCPP_INFO(this->get_logger(), 
                "开始执行: 目标高度=%.2f, 信号=%d", target, signal);

    // 检查是否在执行前就被取消了
    if (goal_handle->is_canceling()) {
        result->finish = false;
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "目标在开始执行前被取消");
        return;
    }

    if (goal->fork_goal_height != 0) {
        // 准备CAN帧
        std::vector<struct can_frame> send_frames;
        struct can_frame v_frame{};
        v_frame.can_id = 0x401;
        v_frame.can_dlc = 8;
        
        // 初始化所有数据字节为0
        for(int i = 0; i < 8; i++) {
            v_frame.data[i] = 0;
        }
        
        // 根据目标高度设置指令
        if(goal->fork_goal_height == 1) {
            v_frame.data[4] = 1;
            RCLCPP_INFO(this->get_logger(), 
                       "下达挂钩升降到位置1命令 (CAN ID:0x401, data[4]=1)");
        }
        else if (goal->fork_goal_height == 2) {
            v_frame.data[4] = 2;
            RCLCPP_INFO(this->get_logger(), 
                       "下达挂钩升降到位置2命令 (CAN ID:0x401, data[4]=2)");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), 
                        "rcs挂钩指令异常: 不支持的目标高度 %d", 
                        goal->fork_goal_height);
            result->finish = false;
            goal_handle->abort(result);
            return;
        }
        
        // 发送CAN帧到队列
        send_frames.push_back(v_frame);
        
        if (send_queue_) {
            send_queue_->push(send_frames);
            RCLCPP_INFO(this->get_logger(), "CAN帧已成功发送到发送队列");
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                        "send_queue_ 未初始化，无法发送CAN帧");
            result->finish = false;
            goal_handle->abort(result);
            return;
        }
        
        // 模拟等待硬件执行
        RCLCPP_INFO(this->get_logger(), "等待硬件执行...");
        rclcpp::sleep_for(std::chrono::seconds(2));
        
        // 执行成功
        result->finish = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "挂钩动作执行成功完成");
    }
    else {
        RCLCPP_WARN(this->get_logger(), "目标高度为0，不执行任何操作");
        result->finish = false;
        goal_handle->abort(result);
    }
}

}  // namespace can_driver