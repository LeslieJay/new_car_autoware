#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ref_slam_interface/action/autoware_auto.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
using namespace std::placeholders;

class AutowareAutoServer : public rclcpp::Node
{
public:
    using AutowareAuto = ref_slam_interface::action::AutowareAuto;
    using GoalHandle = rclcpp_action::ServerGoalHandle<AutowareAuto>;

    AutowareAutoServer() : Node("autoware_auto_server"), current_pose_valid_(false)
    {
        action_server_ = rclcpp_action::create_server<AutowareAuto>(
            this,
            "autoware_auto",
            std::bind(&AutowareAutoServer::handle_goal, this, _1, _2),
            std::bind(&AutowareAutoServer::handle_cancel, this, _1),
            std::bind(&AutowareAutoServer::handle_accepted, this, _1)
        );

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "map_to_base_pose",
            10,
            std::bind(&AutowareAutoServer::pose_callback, this, _1)
        );

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/planning/mission_planning/goal",
            10
        );

        RCLCPP_INFO(this->get_logger(), "Autoware Auto Action Server Ready. Waiting for /map_to_base_pose topic...");
        RCLCPP_INFO(this->get_logger(), "Publishing goals to /planning/mission_planning/goal");
    }

private:
    rclcpp_action::Server<AutowareAuto>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_msg_;
    bool current_pose_valid_;

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_msg_ = *msg;
        current_pose_valid_ = true;
    }

    bool get_current_pose(geometry_msgs::msg::PoseStamped &pose_out)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!current_pose_valid_)
            return false;
        pose_out.header = current_pose_msg_.header;
        pose_out.pose = current_pose_msg_.pose.pose;
        return true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const AutowareAuto::Goal> goal)
    {
        (void)uuid;
        double target_yaw = tf2::getYaw(goal->goal_pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(),
            "Received goal: pos=(%.2f, %.2f), "
            "quat=(x=%.4f, y=%.4f, z=%.4f, w=%.4f), "
            "yaw=%.2f rad (%.2f deg)",
            goal->goal_pose.pose.position.x,
            goal->goal_pose.pose.position.y,
            goal->goal_pose.pose.orientation.x,
            goal->goal_pose.pose.orientation.y,
            goal->goal_pose.pose.orientation.z,
            goal->goal_pose.pose.orientation.w,
            target_yaw,
            target_yaw * 180.0 / M_PI);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "响应ros平台取消导航");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&AutowareAutoServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AutowareAuto::Feedback>();
        auto result = std::make_shared<AutowareAuto::Result>();

        // ========== 发布目标点到话题 ==========
        geometry_msgs::msg::PoseStamped goal_pose_to_pub = goal->goal_pose;
        if (goal_pose_to_pub.header.stamp.sec == 0 && goal_pose_to_pub.header.stamp.nanosec == 0)
        {
            goal_pose_to_pub.header.stamp = this->now();
        }
        goal_pub_->publish(goal_pose_to_pub);
        RCLCPP_INFO(this->get_logger(), "Published goal to /planning/mission_planning/goal: (%.2f, %.2f)",
                    goal_pose_to_pub.pose.position.x, goal_pose_to_pub.pose.position.y);

        // ========== 新增：在执行时也打印详细的目标信息 ==========
        double target_yaw = tf2::getYaw(goal->goal_pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(),
            "Executing goal: pos=(%.2f, %.2f), "
            "quat=(x=%.4f, y=%.4f, z=%.4f, w=%.4f), "
            "yaw=%.2f rad (%.2f deg)",
            goal->goal_pose.pose.position.x,
            goal->goal_pose.pose.position.y,
            goal->goal_pose.pose.orientation.x,
            goal->goal_pose.pose.orientation.y,
            goal->goal_pose.pose.orientation.z,
            goal->goal_pose.pose.orientation.w,
            target_yaw,
            target_yaw * 180.0 / M_PI);

        // 原有监控逻辑
        double target_x = goal->goal_pose.pose.position.x;
        double target_y = goal->goal_pose.pose.position.y;
        // target_yaw 已获取

        const double DIST_TOLERANCE = 0.1;
        const double ANGLE_TOLERANCE = 0.1;
        const double TIMEOUT_SEC = 60000.0;
        auto start_time = this->now();

        rclcpp::Rate loop_rate(10.0);

        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                result->success = false;
                result->message = "Goal canceled by user";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                return;
            }

            if ((this->now() - start_time).seconds() > TIMEOUT_SEC)
            {
                result->success = false;
                result->message = "Goal timed out";
                goal_handle->abort(result);
                RCLCPP_WARN(this->get_logger(), "Goal aborted due to timeout.");
                return;
            }

            geometry_msgs::msg::PoseStamped current_pose;
            if (!get_current_pose(current_pose))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Waiting for /map_to_base_pose topic...");
                loop_rate.sleep();
                continue;
            }

            double dx = current_pose.pose.position.x - target_x;
            double dy = current_pose.pose.position.y - target_y;
            double distance = std::hypot(dx, dy);
            double current_yaw = tf2::getYaw(current_pose.pose.orientation);
            double yaw_diff = std::abs(current_yaw - target_yaw);
            yaw_diff = std::fmod(yaw_diff, 2 * M_PI);
            if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;

            feedback->current_pose = current_pose;
            feedback->number_of_poses_remaining = 0;
            goal_handle->publish_feedback(feedback);

            RCLCPP_INFO(this->get_logger(), "Feedback: distance=%.3f m, angle_diff=%.3f rad",
                        distance, yaw_diff);

            if (distance < DIST_TOLERANCE && yaw_diff < ANGLE_TOLERANCE)
            {
                result->success = true;
                result->message = "Reached goal successfully";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
                return;
            }

            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutowareAutoServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}