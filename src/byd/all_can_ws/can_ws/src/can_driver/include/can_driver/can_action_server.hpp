#ifndef CAN_DRIVER__CAN_ACTION_SERVER_HPP_
#define CAN_DRIVER__CAN_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ref_slam_interface/action/ctrl_fork.hpp"
#include "ref_slam_interface/srv/battery_control.hpp"   // 充电服务接口
#include <memory>

namespace can_driver
{

class CanActionServer : public rclcpp::Node
{
public:
    explicit CanActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // 挂钩动作服务器
    rclcpp_action::Server<ref_slam_interface::action::CtrlFork>::SharedPtr fork_action_server_;
    double current_height_;

    // 充电服务服务器
    rclcpp::Service<ref_slam_interface::srv::BatteryControl>::SharedPtr charge_service_;

    // 挂钩动作回调
    rclcpp_action::GoalResponse handle_fork_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ref_slam_interface::action::CtrlFork::Goal> goal);
    rclcpp_action::CancelResponse handle_fork_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);
    void handle_fork_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);
    void execute_fork(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);

    // 充电服务回调
    void handle_charge_request(
        const std::shared_ptr<ref_slam_interface::srv::BatteryControl::Request> request,
        std::shared_ptr<ref_slam_interface::srv::BatteryControl::Response> response);
};

}  // namespace can_driver

#endif  // CAN_DRIVER__CAN_ACTION_SERVER_HPP_