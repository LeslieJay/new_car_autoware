#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ref_slam_interface/action/autoware_auto.hpp"
#include "reverse_parking_planner/srv/set_goal_pose.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"   // 实际消息头文件
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

using namespace std::placeholders;

class AutowareAutoServer : public rclcpp::Node
{
public:
    using AutowareAuto = ref_slam_interface::action::AutowareAuto;
    using GoalHandle = rclcpp_action::ServerGoalHandle<AutowareAuto>;

    AutowareAutoServer() : Node("autoware_auto_server"), current_pose_valid_(false), current_autoware_state_(0)
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

        // ========== 订阅 /autoware/state（类型 autoware_system_msgs::msg::AutowareState） ==========
        state_sub_ = this->create_subscription<autoware_system_msgs::msg::AutowareState>(
            "/byd/autoware/state",
            10,
            std::bind(&AutowareAutoServer::state_callback, this, _1)
        );

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/planning/mission_planning/goal",
            10
        );

        reverse_parking_client_ = this->create_client<reverse_parking_planner::srv::SetGoalPose>(
            "/reverse_parking_planner/set_goal_pose");

        RCLCPP_INFO(this->get_logger(), "Autoware Auto Action Server Ready. Waiting for topics...");
        RCLCPP_INFO(this->get_logger(), "Monitoring /autoware/state for arrival (state == 6).");
        RCLCPP_INFO(this->get_logger(), "Preemption enabled: new goal cancels previous one.");
    }

private:
    rclcpp_action::Server<AutowareAuto>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Client<reverse_parking_planner::srv::SetGoalPose>::SharedPtr reverse_parking_client_;

    // ---------- /autoware/state 相关 ----------
    rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr state_sub_;
    std::mutex state_mutex_;
    int current_autoware_state_;   // 当前状态，目标到达时为 6
    // -----------------------------------------

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_msg_;
    bool current_pose_valid_;

    std::mutex goal_handle_mutex_;
    std::shared_ptr<GoalHandle> current_goal_handle_;
    std::thread execution_thread_;

    static constexpr int ARRIVAL_STATE = 6;         // 到达状态值
    static constexpr double DIST_TOLERANCE = 0.1;   // 保留（仅用于状态打印）
    static constexpr double ANGLE_TOLERANCE = 0.1;

    // /autoware/state 回调
    void state_callback(const autoware_system_msgs::msg::AutowareState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_autoware_state_ = msg->state;
    }

    int get_autoware_state()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return current_autoware_state_;
    }

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
        std::string mode_str = goal->forward ? "FORWARD" : "REVERSE";
        RCLCPP_INFO(this->get_logger(),
            "Received goal [%s]: pos=(%.2f, %.2f), yaw=%.2f deg",
            mode_str.c_str(),
            goal->goal_pose.pose.position.x,
            goal->goal_pose.pose.position.y,
            target_yaw * 180.0 / M_PI);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Cancel request received.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);
        if (current_goal_handle_ && current_goal_handle_->is_active()) {
            auto result = std::make_shared<AutowareAuto::Result>();
            result->success = false;
            result->message = "Preempted by a newer goal";
            RCLCPP_WARN(this->get_logger(), "Preempting previous goal.");
            current_goal_handle_->abort(result);
        }
        if (execution_thread_.joinable()) {
            execution_thread_.detach();
        }
        execution_thread_ = std::thread(
            &AutowareAutoServer::execute, this, goal_handle);
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            current_goal_handle_ = goal_handle;
        }

        auto cleanup = [this, goal_handle]() {
            std::lock_guard<std::mutex> lock(goal_handle_mutex_);
            if (current_goal_handle_ == goal_handle) {
                current_goal_handle_.reset();
            }
        };

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AutowareAuto::Feedback>();
        auto result = std::make_shared<AutowareAuto::Result>();
        bool forward = goal->forward;

        // ========== 方向策略（前进/倒车） ==========
        if (forward) {
            geometry_msgs::msg::PoseStamped goal_pose_to_pub = goal->goal_pose;
            if (goal_pose_to_pub.header.stamp.sec == 0 &&
                goal_pose_to_pub.header.stamp.nanosec == 0) {
                goal_pose_to_pub.header.stamp = this->now();
            }
            goal_pub_->publish(goal_pose_to_pub);
            RCLCPP_INFO(this->get_logger(), "Published forward goal.");
        } else {
            if (!reverse_parking_client_->wait_for_service(std::chrono::seconds(5))) {
                if (goal_handle->is_active()) {
                    result->success = false;
                    result->message = "Reverse parking service unavailable";
                    goal_handle->abort(result);
                }
                cleanup();
                return;
            }

            auto request = std::make_shared<reverse_parking_planner::srv::SetGoalPose::Request>();
            request->goal_pose = goal->goal_pose;

            auto future = reverse_parking_client_->async_send_request(request);
            while (rclcpp::ok() && goal_handle->is_active()) {
                auto status = future.wait_for(std::chrono::milliseconds(100));
                if (status == std::future_status::ready) break;
            }
            if (!goal_handle->is_active()) {
                RCLCPP_INFO(this->get_logger(), "Reverse goal preempted during service call.");
                cleanup();
                return;
            }
            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(this->get_logger(), "Reverse parking service failed: %s", response->message.c_str());
                result->success = false;
                result->message = "Reverse parking planning failed: " + response->message;
                goal_handle->abort(result);
                cleanup();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Reverse parking goal accepted. Path points: %u", response->path_points_num);
        }

        double target_yaw = tf2::getYaw(goal->goal_pose.pose.orientation);
        std::string mode_str = forward ? "FORWARD" : "REVERSE";
        RCLCPP_INFO(this->get_logger(),
            "Executing goal [%s]: pos=(%.2f, %.2f), yaw=%.2f deg. Awaiting state==6 for arrival.",
            mode_str.c_str(),
            goal->goal_pose.pose.position.x,
            goal->goal_pose.pose.position.y,
            target_yaw * 180.0 / M_PI);

        double target_x = goal->goal_pose.pose.position.x;
        double target_y = goal->goal_pose.pose.position.y;

        const double TIMEOUT_SEC = 60000.0;
        auto start_time = this->now();
        rclcpp::Rate loop_rate(10.0);
        auto last_status_time = this->now();

        while (rclcpp::ok() && goal_handle->is_active()) {
            // 超时
            if ((this->now() - start_time).seconds() > TIMEOUT_SEC) {
                result->success = false;
                result->message = "Goal timed out";
                goal_handle->abort(result);
                cleanup();
                return;
            }

            // 获取当前位姿（仅用于反馈）
            geometry_msgs::msg::PoseStamped current_pose;
            bool pose_ok = get_current_pose(current_pose);
            if (pose_ok) {
                feedback->current_pose = current_pose;
                feedback->number_of_poses_remaining = 0;
                goal_handle->publish_feedback(feedback);
            }

            // ========== 核心：通过 /autoware/state 判断到达 ==========
            int state = get_autoware_state();
            if (state == ARRIVAL_STATE) {
                result->success = true;
                result->message = "Arrived at goal (autoware state = 6)";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "[%s] Goal succeeded. Autoware state is 6.", mode_str.c_str());
                cleanup();
                return;
            }
            // ==========================================================

            // 状态打印（每秒一次）
            auto now = this->now();
            if ((now - last_status_time).seconds() >= 1.0) {
                double dist = 999.0, yaw_diff = 999.0;
                if (pose_ok) {
                    double dx = current_pose.pose.position.x - target_x;
                    double dy = current_pose.pose.position.y - target_y;
                    dist = std::hypot(dx, dy);
                    double current_yaw = tf2::getYaw(current_pose.pose.orientation);
                    yaw_diff = std::abs(current_yaw - target_yaw);
                    yaw_diff = std::fmod(yaw_diff, 2 * M_PI);
                    if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
                }
                RCLCPP_INFO(this->get_logger(),
                    "[%s] Autoware state=%d | dist=%.3f, angle_diff=%.3f | Waiting for state==6",
                    mode_str.c_str(), state, dist, yaw_diff);
                last_status_time = now;
            }

            loop_rate.sleep();
        }

        if (!goal_handle->is_active()) {
            RCLCPP_INFO(this->get_logger(), "Goal terminated during execution.");
        }
        cleanup();
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