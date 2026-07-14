#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>

class OdometryToPoseConverter : public rclcpp::Node
{
public:
    OdometryToPoseConverter() : Node("odometry_to_pose_converter")
    {
        // 声明参数：是否启用模拟数据发布，默认 false
        this->declare_parameter<bool>("simulation_enabled", false);

        // 创建订阅者，订阅原始 Odometry 话题
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state",
            10,
            std::bind(&OdometryToPoseConverter::odometryCallback, this, std::placeholders::_1));

        // 创建发布者，发布转换后的 PoseWithCovarianceStamped 话题
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "map_to_base_pose",
            10);

        // 创建发布模拟 Odometry 的发布者（与订阅话题相同）
        simulated_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state",
            10);

        // 设置参数回调，以便动态启用/禁用模拟发布
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&OdometryToPoseConverter::paramCallback, this, std::placeholders::_1));

        // 初始根据参数决定是否启动定时器
        bool sim_enabled = this->get_parameter("simulation_enabled").as_bool();
        if (sim_enabled) {
            startSimulationTimer();
        }

        RCLCPP_INFO(this->get_logger(), "Odometry to Pose converter node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /localization/kinematic_state");
        RCLCPP_INFO(this->get_logger(), "Publishing to: map_to_base_pose");
        RCLCPP_INFO(this->get_logger(), "Simulation mode is %s", sim_enabled ? "enabled" : "disabled");
    }

private:
    // 原始 Odometry 回调：转换为 PoseWithCovarianceStamped 并发布
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header = msg->header;
        pose_msg.pose.pose = msg->pose.pose;

        if (msg->pose.covariance.size() == pose_msg.pose.covariance.size()) {
            pose_msg.pose.covariance = msg->pose.covariance;
        } else {
            RCLCPP_WARN_ONCE(this->get_logger(), "Covariance matrix size mismatch, using identity");
        }

        pose_pub_->publish(pose_msg);
        
        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Current position - x: %.3f, y: %.3f, z: %.3f",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z);
    }

    // 定时器回调：生成并发布模拟的 Odometry 消息
    void publishSimulatedOdometry()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        rclcpp::Time now = this->now();

        // 填充头部
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "base_link";

        // // 设置一个固定的位置
        // odom_msg.pose.pose.position.x = -18.0;
        // odom_msg.pose.pose.position.y = -8.4;
        // odom_msg.pose.pose.position.z = 0.0;
        // odom_msg.pose.pose.orientation.x = 0.0;
        // odom_msg.pose.pose.orientation.y = 0.0;
        // odom_msg.pose.pose.orientation.z = -0.9689;
        // odom_msg.pose.pose.orientation.w = 0.248;
        // 设置一个固定的位置
        odom_msg.pose.pose.position.x = 0;
        odom_msg.pose.pose.position.y = 0;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.orientation.w = 0.248;
        // 设置速度
        odom_msg.twist.twist.linear.x = 0;
        odom_msg.twist.twist.angular.z = 0;

        simulated_odom_pub_->publish(odom_msg);

        RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Simulated position - x: %.3f, y: %.3f, z: %.3f",
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z);
    }

    // 其余代码保持不变...
    void startSimulationTimer()
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&OdometryToPoseConverter::publishSimulatedOdometry, this));
        RCLCPP_INFO(this->get_logger(), "Simulation timer started");
    }

    void stopSimulationTimer()
    {
        if (timer_) {
            timer_->cancel();
            timer_.reset();
            RCLCPP_INFO(this->get_logger(), "Simulation timer stopped");
        }
    }

    rcl_interfaces::msg::SetParametersResult paramCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto & param : parameters) {
            if (param.get_name() == "simulation_enabled") {
                bool new_value = param.as_bool();
                bool current_value = this->get_parameter("simulation_enabled").as_bool();

                if (new_value != current_value) {
                    if (new_value) {
                        startSimulationTimer();
                    } else {
                        stopSimulationTimer();
                    }
                    RCLCPP_INFO(this->get_logger(), "Simulation enabled set to %s", new_value ? "true" : "false");
                }
            }
        }
        return result;
    }

    // 成员变量
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr simulated_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryToPoseConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}