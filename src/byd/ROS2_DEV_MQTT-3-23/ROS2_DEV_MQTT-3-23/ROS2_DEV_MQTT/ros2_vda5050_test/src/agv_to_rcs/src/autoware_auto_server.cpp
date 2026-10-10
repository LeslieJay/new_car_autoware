#include <memory>
#include <thread>
#include <chrono>
#include <mutex>
#include <cmath>
#include <condition_variable>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "ref_slam_interface/action/autoware_auto.hpp"
#include "reverse_parking_planner/srv/set_goal_pose.hpp"
#include "autoware_system_msgs/msg/autoware_state.hpp"
#include "autoware_adapi_v1_msgs/srv/change_operation_mode.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

using namespace std::placeholders;

class AutowareAutoServer : public rclcpp::Node
{
public:
    using AutowareAuto = ref_slam_interface::action::AutowareAuto;
    using GoalHandle = rclcpp_action::ServerGoalHandle<AutowareAuto>;

    AutowareAutoServer()
        : Node("autoware_auto_server"),
          current_autoware_state_(0),
          current_pose_valid_(false),
          running_(true)
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
            "/agv_high_precision_reverse_controller/set_goal_pose");

        autonomous_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
            "/api/operation_mode/change_to_autonomous");
        stop_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
            "/api/operation_mode/change_to_stop");

        // 串行执行线程：所有 goal 都在这里依次执行
        worker_thread_ = std::thread(&AutowareAutoServer::worker_loop, this);

        RCLCPP_INFO(this->get_logger(),
            "Autoware Auto Action Server Ready (serialized worker).");
        RCLCPP_INFO(this->get_logger(),
            "All new goals (forward/reverse) share the same preemption path.");
    }

    ~AutowareAutoServer()
    {
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            running_ = false;
            preempt_requested_ = true;
        }
        goal_cv_.notify_all();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    rclcpp_action::Server<AutowareAuto>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Client<reverse_parking_planner::srv::SetGoalPose>::SharedPtr reverse_parking_client_;

    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr autonomous_client_;
    rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr stop_client_;

    rclcpp::Subscription<autoware_system_msgs::msg::AutowareState>::SharedPtr state_sub_;
    std::mutex state_mutex_;
    int current_autoware_state_;

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_msg_;
    bool current_pose_valid_;

    // ---- 串行执行相关 ----
    std::mutex goal_mutex_;
    std::condition_variable goal_cv_;
    std::shared_ptr<GoalHandle> pending_goal_;   // 等待执行的最新 goal
    std::shared_ptr<GoalHandle> active_goal_;    // 正在执行的 goal
    std::thread worker_thread_;
    bool running_;
    std::atomic<bool> preempt_requested_{false};

    static constexpr int ARRIVAL_STATE = 6;
    static constexpr double DIST_TOLERANCE = 0.1;
    static constexpr double ANGLE_TOLERANCE = 0.1;

    // ---------- 模式切换辅助函数 ----------
    bool call_autonomous_mode()
    {
        if (!autonomous_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Autonomous mode service not available.");
            return false;
        }
        auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
        auto future = autonomous_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            auto response = future.get();
            if (response->status.success) {
                RCLCPP_INFO(this->get_logger(), "Switched to autonomous mode.");
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Autonomous mode switch failed: %s",
                            response->status.message.c_str());
                return false;
            }
        }
        RCLCPP_WARN(this->get_logger(), "Timeout while switching to autonomous mode.");
        return false;
    }

    bool call_stop_mode()
    {
        if (!stop_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Stop mode service not available.");
            return false;
        }
        auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
        auto future = stop_client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
            auto response = future.get();
            if (response->status.success) {
                RCLCPP_INFO(this->get_logger(), "Switched to stop mode.");
                return true;
            } else {
                RCLCPP_WARN(this->get_logger(), "Stop mode switch failed: %s",
                            response->status.message.c_str());
                return false;
            }
        }
        RCLCPP_WARN(this->get_logger(), "Timeout while switching to stop mode.");
        return false;
    }

    // ---------- 订阅回调 ----------
    void state_callback(const autoware_system_msgs::msg::AutowareState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (msg->state == ARRIVAL_STATE) {
            geometry_msgs::msg::PoseStamped cur;
            bool ok = false;
            double gx = 0.0, gy = 0.0;
            bool has_goal = false;
            {
                std::lock_guard<std::mutex> p_lock(pose_mutex_);
                if (current_pose_valid_) {
                    cur.pose = current_pose_msg_.pose.pose;
                    ok = true;
                }
            }
            {
                std::lock_guard<std::mutex> g_lock(goal_mutex_);
                if (active_goal_ && active_goal_->is_active()) {
                    gx = active_goal_->get_goal()->goal_pose.pose.position.x;
                    gy = active_goal_->get_goal()->goal_pose.pose.position.y;
                    has_goal = true;
                }
            }
            if (ok && has_goal) {
                double dist = std::hypot(cur.pose.position.x - gx, cur.pose.position.y - gy);
                if (dist > 1.0) {
                    current_autoware_state_ = 0;
                    return;
                }
            }
        }
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
        if (!current_pose_valid_) return false;
        pose_out.header = current_pose_msg_.header;
        pose_out.pose = current_pose_msg_.pose.pose;
        return true;
    }

    // ---------- Action 回调 ----------
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
        RCLCPP_INFO(this->get_logger(), "Cancel request received. Notify worker to stop.");
        preempt_requested_ = true;
        goal_cv_.notify_all();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::shared_ptr<GoalHandle> old_pending;
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            if (pending_goal_ && pending_goal_ != goal_handle) {
                old_pending = pending_goal_;
            }
            pending_goal_ = goal_handle;
            preempt_requested_ = true;   // 通知正在执行的旧 goal 尽快退出
        }
        if (old_pending) {
            auto result = std::make_shared<AutowareAuto::Result>();
            result->success = false;
            result->message = "Preempted by newer goal (pending)";
            old_pending->abort(result);
        }
        goal_cv_.notify_all();
    }

    // ---------- 串行执行线程 ----------
    void worker_loop()
    {
        while (rclcpp::ok()) {
            std::shared_ptr<GoalHandle> goal;
            {
                std::unique_lock<std::mutex> lock(goal_mutex_);
                goal_cv_.wait(lock, [this] {
                    return !running_ || pending_goal_ != nullptr;
                });
                if (!running_) break;
                goal = pending_goal_;
                pending_goal_.reset();
                active_goal_ = goal;
                preempt_requested_ = false;   // 新 goal 开始，清除旧的中断标志
            }

            // 串行执行：execute 返回后才会取下一个 goal
            execute(goal);

            {
                std::lock_guard<std::mutex> lock(goal_mutex_);
                if (active_goal_ == goal) {
                    active_goal_.reset();
                }
            }
        }
    }

    // ---------- 统一执行流程（前进 / 倒车共用同一条抢占路径） ----------
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<AutowareAuto::Result>();
        bool forward = goal->forward;
        double target_x = goal->goal_pose.pose.position.x;
        double target_y = goal->goal_pose.pose.position.y;
        double target_yaw = tf2::getYaw(goal->goal_pose.pose.orientation);
        std::string mode_str = forward ? "FORWARD" : "REVERSE";

        auto preempted = [this, goal_handle]() {
            return preempt_requested_.load()
                || !goal_handle->is_active()
                || goal_handle->is_canceling();
        };

        auto interruptible_sleep = [this, &preempted](int ms) -> bool {
            int elapsed = 0;
            while (rclcpp::ok() && elapsed < ms) {
                if (preempted()) return false;
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                elapsed += 50;
            }
            return !preempted();
        };

        auto handle_startup_preempt = [&]() {
            RCLCPP_WARN(this->get_logger(), "[%s] Preempted/canceled during startup, stopping.",
                        mode_str.c_str());
            call_stop_mode();
            if (goal_handle->is_active()) {
                result->success = false;
                if (goal_handle->is_canceling()) {
                    result->message = "Goal canceled";
                    goal_handle->canceled(result);
                } else {
                    result->message = "Preempted by newer goal";
                    goal_handle->abort(result);
                }
            }
        };

        // ---- 1. 如果此前处于暂停/停止模式，先唤醒并切换到自主模式，确保 Autoware 退出 STOP 状态 ----
        call_autonomous_mode();
        if (!interruptible_sleep(150)) {
            handle_startup_preempt();
            return;
        }

        // ---- 2. 清空内部旧状态 ----
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_autoware_state_ = 0;
        }

        // ---- 3. 下发目标并等待路径规划 ----
        if (forward) {
            geometry_msgs::msg::PoseStamped goal_pose_to_pub = goal->goal_pose;
            if (goal_pose_to_pub.header.stamp.sec == 0 &&
                goal_pose_to_pub.header.stamp.nanosec == 0) {
                goal_pose_to_pub.header.stamp = this->now();
            }
            goal_pub_->publish(goal_pose_to_pub);
            RCLCPP_INFO(this->get_logger(), "Forward goal published.");

            // 给 Autoware 全局路径规划充足时间 (通常需 200~500ms)
            if (!interruptible_sleep(400)) {
                handle_startup_preempt();
                return;
            }

            // 路径规划完成后，再次调用自主模式以启动车辆 (Engage)
            if (!call_autonomous_mode()) {
                RCLCPP_WARN(this->get_logger(),
                    "Could not switch to autonomous mode after planning, will retry in loop.");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Waiting for reverse parking service...");
            // 可中断等待服务
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
            while (rclcpp::ok() && !preempted()) {
                if (reverse_parking_client_->service_is_ready()) break;
                if (std::chrono::steady_clock::now() > deadline) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            if (preempted()) {
                handle_startup_preempt();
                return;
            }
            if (!reverse_parking_client_->service_is_ready()) {
                result->success = false;
                result->message = "Reverse parking service unavailable";
                RCLCPP_ERROR(this->get_logger(), "Reverse parking service unavailable.");
                if (goal_handle->is_active()) goal_handle->abort(result);
                return;
            }

            auto request = std::make_shared<reverse_parking_planner::srv::SetGoalPose::Request>();
            request->goal_pose = goal->goal_pose;
            auto future = reverse_parking_client_->async_send_request(request);

            // 可中断等待服务响应
            while (rclcpp::ok()) {
                if (preempted()) {
                    handle_startup_preempt();
                    return;
                }
                if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                    break;
                }
            }
            if (!goal_handle->is_active()) return;
            auto response = future.get();
            if (!response->success) {
                RCLCPP_ERROR(this->get_logger(),
                    "Reverse parking service failed: %s", response->message.c_str());
                result->success = false;
                result->message = "Reverse parking planning failed: " + response->message;
                if (goal_handle->is_active()) goal_handle->abort(result);
                return;
            }
            RCLCPP_INFO(this->get_logger(),
                "Reverse parking goal accepted. Path points: %u", response->path_points_num);

            if (!call_autonomous_mode()) {
                RCLCPP_WARN(this->get_logger(),
                    "Could not switch to autonomous mode for reverse, will retry in loop.");
            }
        }

        RCLCPP_INFO(this->get_logger(),
            "Executing goal [%s]: pos=(%.2f, %.2f), yaw=%.2f deg. Awaiting state==6.",
            mode_str.c_str(), target_x, target_y, target_yaw * 180.0 / M_PI);

        // ---- 4. 主循环 ----
        const double TIMEOUT_SEC = 60000.0;
        auto start_time = this->now();
        rclcpp::Rate loop_rate(10.0);
        auto last_status_time = this->now();
        auto last_engage_retry_time = this->now();
        auto feedback = std::make_shared<AutowareAuto::Feedback>();

        while (rclcpp::ok()) {
            // 统一抢占 / 取消路径 (支持暂停状态下的新旧抢占)
            if (preempted()) {
                RCLCPP_WARN(this->get_logger(), "[%s] Preempted/canceled, stopping.",
                            mode_str.c_str());
                call_stop_mode();
                if (goal_handle->is_active()) {
                    result->success = false;
                    if (goal_handle->is_canceling()) {
                        result->message = "Goal canceled";
                        goal_handle->canceled(result);
                    } else {
                        result->message = "Preempted by newer goal";
                        goal_handle->abort(result);
                    }
                }
                return;
            }

            if ((this->now() - start_time).seconds() > TIMEOUT_SEC) {
                result->success = false;
                result->message = "Goal timed out";
                if (goal_handle->is_active()) goal_handle->abort(result);
                return;
            }

            // 提前计算距离与航向差
            geometry_msgs::msg::PoseStamped current_pose;
            bool pose_ok = get_current_pose(current_pose);
            double dist = 999.0, yaw_diff = 999.0;

            if (pose_ok) {
                feedback->current_pose = current_pose;
                feedback->number_of_poses_remaining = 0;
                goal_handle->publish_feedback(feedback);

                double dx = current_pose.pose.position.x - target_x;
                double dy = current_pose.pose.position.y - target_y;
                dist = std::hypot(dx, dy);
                
                double current_yaw = tf2::getYaw(current_pose.pose.orientation);
                yaw_diff = std::abs(current_yaw - target_yaw);
                yaw_diff = std::fmod(yaw_diff, 2 * M_PI);
                if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;
            }

            int state = get_autoware_state();
            auto now = this->now();
            
            // 加入距离校验，判断是否是真的终点到达（规避残余状态 6 的 Bug）
            if (state == ARRIVAL_STATE) {
                if (dist <= 1.0) { // 只有物理距离在 1m 以内，才认可 state 6 为最终完成
                    result->success = true;
                    result->message = "Arrived at goal (autoware state = 6)";
                    if (pose_ok) {
                        if (dist > DIST_TOLERANCE || yaw_diff > ANGLE_TOLERANCE) {
                            RCLCPP_WARN(this->get_logger(),
                                "[%s] state=6 but pose slightly off target: dist=%.3f (tol %.2f), "
                                "yaw_diff=%.3f (tol %.2f)",
                                mode_str.c_str(), dist, DIST_TOLERANCE, yaw_diff, ANGLE_TOLERANCE);
                            result->message += " (pose off target)";
                        }
                    }
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "[%s] Goal succeeded (state=6).", mode_str.c_str());
                    return;
                } else {
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        current_autoware_state_ = 0;
                    }
                    if ((now - last_status_time).seconds() >= 1.0) {
                        RCLCPP_WARN(this->get_logger(),
                            "[%s] Autoware state=6, but dist=%.3fm (> 1.0m). Cleared residual state.",
                            mode_str.c_str(), dist);
                        last_status_time = now;
                    }
                }
            } else {
                if ((now - last_status_time).seconds() >= 1.0) {
                    RCLCPP_INFO(this->get_logger(),
                        "[%s] Autoware state=%d | dist=%.3f, angle_diff=%.3f | Waiting for state==6",
                        mode_str.c_str(), state, dist, yaw_diff);
                    last_status_time = now;
                }
            }

            // 车辆未启动或处于等待介入状态处理 (若 dist > 1.0m 且 state == 0，说明未进入自主行驶或规划未启动)
            if (state == 0 && dist > 1.0 && (now - last_engage_retry_time).seconds() >= 1.5) {
                if (forward) {
                    geometry_msgs::msg::PoseStamped retry_pose = goal->goal_pose;
                    retry_pose.header.stamp = this->now();
                    goal_pub_->publish(retry_pose);
                    RCLCPP_INFO(this->get_logger(),
                        "[%s] Autoware still in state=0 (dist=%.2fm). Re-publishing goal and re-engaging autonomous mode...",
                        mode_str.c_str(), dist);
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "[%s] Autoware still in state=0 (dist=%.2fm). Re-engaging autonomous mode...",
                        mode_str.c_str(), dist);
                }
                call_autonomous_mode();
                last_engage_retry_time = now;
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