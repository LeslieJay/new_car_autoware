#ifndef CAN_DRIVER__HOOK_ACTION_SERVER_HPP_
#define CAN_DRIVER__HOOK_ACTION_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ref_slam_interface/action/ctrl_fork.hpp"
#include <memory>

namespace can_driver
{

class ForkActionServer : public rclcpp::Node
{
public:
    explicit ForkActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp_action::Server<ref_slam_interface::action::CtrlFork>::SharedPtr action_server_;
    double current_height_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ref_slam_interface::action::CtrlFork::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);

    void execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ref_slam_interface::action::CtrlFork>> goal_handle);
};

}  // namespace can_driver

#endif  // CAN_DRIVER__HOOK_ACTION_SERVER_HPP_