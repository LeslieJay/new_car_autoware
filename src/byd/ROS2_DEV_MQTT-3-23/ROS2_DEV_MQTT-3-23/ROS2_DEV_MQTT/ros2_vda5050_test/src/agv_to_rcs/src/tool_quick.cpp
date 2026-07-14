/************************************** File Info ****************************************
 * @file:       tool_quick.cpp
 * @author:     刘鸿彬
 * @date:       2025-03-17
 * @version:    V0.0
 * @brief:      快捷工具
 ******************************************************************************************/

#include "client_end_guide.h"
#include "client_request_charging.h"
#include "client_fork_action.h"
#include "client_send_multi_poses.h"
#include "server_obstacle.h"
#include <iostream>
#include <string>
#include <memory>
#include <iomanip>
#include "agv_bone.h"
#include "tool_math.h"

#include "ref_slam_interface/action/micro_motion_control.hpp"
#include <thread>
#include <future>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>

using MMC = ref_slam_interface::action::MicroMotionControl;

std::string arr_to_string(const std::array<uint8_t, UUID_SIZE> &arr)
{
    std::string res;
    for (auto e : arr)
    {
        res += std::to_string(e);
    }
    return res.substr(res.size() - 4);
}

/**
 * @brief 从四元数计算位姿角度（弧度）
 * @param quat_w 四元数w分量
 * @param quat_z 四元数z分量
 * @return 计算得到的角度（弧度）
 */
double calculate_pose_theta_from_quaternion(double quat_w, double quat_z)
{
    // 四元数转弧度
    double theta = 2 * acos(quat_w);
    
    // 正反转判别
    if (quat_z < 0) {
        theta = -theta;
    }
    
    return theta;
}


class MicroMotionCtrlClient
{
public:
    using GoalHandleMMC = rclcpp_action::ClientGoalHandle<MMC>;

    MicroMotionCtrlClient(std::shared_ptr<rclcpp::Node> node): 
    node_(node)
    {
        this->client_ = rclcpp_action::create_client<MMC>(node_, "micro_motion_control");
        RCLCPP_INFO(node_->get_logger(), "MicroMotionCtrlClient initialized.");
    }

    // 发送goal并等待结果
    void send_goal_async(const MMC::Goal &goal)
    {

        using namespace std::placeholders;

        if (!this->client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto send_goal_options = rclcpp_action::Client<MMC>::SendGoalOptions();

        // 设置响应、反馈和结果回调
        send_goal_options.goal_response_callback =
            std::bind(&MicroMotionCtrlClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MicroMotionCtrlClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MicroMotionCtrlClient::result_callback, this, _1);

        // 将目标goal发送到服务器
        auto future_handle = this->client_->async_send_goal(goal, send_goal_options);
        last_goal_handle_ = future_handle.get();
        // 有可能会被拒绝, 此时是空指针
        if (last_goal_handle_)
        {
            RCLCPP_INFO_STREAM(node_->get_logger(), "Sent goal : " << arr_to_string(last_goal_handle_->get_goal_id()));
        }
    }


private:
    std::shared_ptr<rclcpp::Node> node_;
    GoalHandleMMC::SharedPtr last_goal_handle_{nullptr};
    rclcpp_action::Client<MMC>::SharedPtr client_;

    void goal_response_callback(GoalHandleMMC::SharedPtr future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // 假设服务器接受了目标，它将开始处理。对客户端的任何反馈都将由该函数进行
    void feedback_callback(
        GoalHandleMMC::SharedPtr handle, const std::shared_ptr<const MMC::Feedback> feedback)
    {
        
        // RCLCPP_INFO_STREAM(node_->get_logger(),
        //                    "Feedback: " << arr_to_string(handle->get_goal_id()) << " ang = " << feedback->ang_current << " , dis = " << feedback->dis_traveled);
    }

    // 当服务器完成处理后，它将向客户端返回一个结果。
    void result_callback(const GoalHandleMMC::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO_STREAM(node_->get_logger(), "Goal succeeded. " << arr_to_string(result.goal_id));
            RCLCPP_INFO_STREAM(node_->get_logger(), "准备记录当前位置...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));            
            
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Goal was aborted. " << arr_to_string(result.goal_id));
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Goal was canceled. " << arr_to_string(result.goal_id));
            return;
        default:
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown result code.");
            return;
        }
        RCLCPP_INFO_STREAM(node_->get_logger(), "Result received: reached : " << result.result->is_reached);
        return;
    }

};

// 此处是打开充电的小工具。
int main(int argc, char *argv[])
{

    // 初始化参数配置

    if (!load_agv_config("/home/agv_config.yaml"))
    {
        // 处理错误
        std::cout << "加载配置文件失败！" << std::endl;
        return 0;
    }

    // 初始化
    rclcpp::init(argc, argv);

    auto all_nodes = std::make_shared<rclcpp::Node>("All_ROS2_nodes");
    RCLCPP_INFO(all_nodes->get_logger(), "创建all_nodes");

    // 初始化，客户端发送的控制数据，增加可读性
    CurrentPose current_pose_message;
    int start_charge = 1;
    int stop_charge = 0;
    int obstacle_messages = 0;

    Math_Tool math_tool = Math_Tool();

    /*-------------------------------------------------------------------------------------------------------------*/

    std::string userInput;
    int heightInput;
    int angleInput;
    int point_num;
    double x, y, theta, allowed_deviation_angle;
    int i;
    int qr, obstacle_channel_select;

    if(agv_config.vehicle_type == "laser") 
    {

        std::shared_ptr<ListenerPose> current_pose_listener = std::make_shared<ListenerPose>(all_nodes);

        std::shared_ptr<RequestChargingClient> request_charging_client_ = std::make_shared<RequestChargingClient>(all_nodes);

        std::shared_ptr<LsaerForkActionClient> fork_action_client = std::make_shared<LsaerForkActionClient>(all_nodes);

        std::shared_ptr<LaserSendMultiPose> send_multi_pose = std::make_shared<LaserSendMultiPose>(all_nodes, current_pose_listener);

        std::shared_ptr<EndGuideClient> agv_end_guide = std::make_shared<EndGuideClient>(all_nodes);

        std::shared_ptr<ObstacleServer> obstacle_server = std::make_shared<ObstacleServer>(all_nodes, current_pose_listener);

        std::shared_ptr<MicroMotionCtrlClient> mmc_client = std::make_shared<MicroMotionCtrlClient>(all_nodes);

        std::thread all_nodes_thread([all_nodes]()
                                    { rclcpp::spin(all_nodes); });

        while (true)
        {
            std::cout << "欢迎使用快捷工具，请选择功能：  1.轨迹导航  2.多点导航  3.进入充电模式  4.进入放电模式  5.控制货叉高度  6.末端引导 7.观察障碍物检测 8.计算误差 9.取消货叉500动作 10.计算位姿 11.打印实时位姿 12.切换避障通道 0.退出工具" << std::endl;
            std::cin >> userInput;
            if (userInput == "0")
            {
                break;
            }
            else if (userInput == "12") // 切换避障通道
            {
                std::cout << "请输入避障通道 (0-2):" << std::endl;
                int obstacle_channel;
                std::cin >> obstacle_channel;
                obstacle_server->publish_obstacle_channels(obstacle_channel, obstacle_channel, obstacle_channel, obstacle_channel);
            }
            else if (userInput == "9") // 先发送货叉抬升到1200任务，然后过一秒钟取消该任务。
            {
                fork_action_client->send_fork_goal(1, 500);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                fork_action_client->cancel_action();
            }
            else if (userInput == "10") // 计算位姿
            {
                double input_x, input_y, input_z, input_w;
                
                std::cout << "\n=== 计算位姿 ===" << std::endl;
                std::cout << "请输入四个值 (x y z w):" << std::endl;
                std::cout << "  x: 位置X坐标 (单位: m)" << std::endl;
                std::cout << "  y: 位置Y坐标 (单位: m)" << std::endl;
                std::cout << "  z: 四元数z分量" << std::endl;
                std::cout << "  w: 四元数w分量" << std::endl;
                std::cout << "请输入: ";
                
                std::cin >> input_x >> input_y >> input_z >> input_w;
                
                // 使用封装的函数计算角度
                double calculated_theta = calculate_pose_theta_from_quaternion(input_w, input_z);
                
                std::cout << "\n=== 计算结果 ===" << std::endl;
                std::cout << "输入值:" << std::endl;
                std::cout << "  X坐标: " << std::fixed << std::setprecision(5) << input_x << " m" << std::endl;
                std::cout << "  Y坐标: " << std::fixed << std::setprecision(5) << input_y << " m" << std::endl;
                std::cout << "  四元数z: " << std::fixed << std::setprecision(5) << input_z << std::endl;
                std::cout << "  四元数w: " << std::fixed << std::setprecision(5) << input_w << std::endl;
                std::cout << "\n计算得到的位姿:" << std::endl;
                std::cout << "  X坐标: " << std::fixed << std::setprecision(5) << input_x << " m" << std::endl;
                std::cout << "  Y坐标: " << std::fixed << std::setprecision(5) << input_y << " m" << std::endl;
                std::cout << "  角度(弧度): " << std::fixed << std::setprecision(5) << calculated_theta << " rad" << std::endl;
                std::cout << "  角度(度): " << std::fixed << std::setprecision(2) << (calculated_theta * 180.0 / M_PI) << " °" << std::endl;
                std::cout << "\n位姿格式: (" << input_x << ", " << input_y << ", " << calculated_theta << ")" << std::endl;
                std::cout << "==================\n" << std::endl;
            }
            else if (userInput == "11") // 打印实时位姿
            {
                // 确保状态发布的多线程在获取位姿之后启动
                while (!current_pose_listener->get_pose)
                {
                    std::cout << "等待系统获取AGV当前位姿信息！" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                std::cout << "开始打印实时位姿，按Ctrl+C退出..." << std::endl;
                
                while (true)
                {
                    current_pose_message = current_pose_listener->get_current_pose();
                    std::cout << "(" << std::fixed << std::setprecision(5) 
                              << current_pose_message.current_x << ", " 
                              << current_pose_message.current_y << ", " 
                              << current_pose_message.current_theta << ")" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            else if (userInput == "8")
            {
                double i = 0.0;
                CurrentPose current_pose;
                CurrentPose some_poses[50];
                MMC::Goal goal;
                // 初始化
                goal.en_rot = false;
                goal.en_mov = false;
                goal.target_ang = 0.0;
                goal.target_dis = 0.0;

                while (i <= 365) {
                    goal.en_rot = true;
                    goal.target_ang = 3.1415926*(i/360.0)*2.0;
                    if(goal.target_ang > 3.1415927)
                        goal.target_ang = goal.target_ang - (3.1415926*2.0);
                    
                    if(i>5)
                    {
                        mmc_client->send_goal_async(goal);
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000*10));
                    }
                    
                    std::cout << "小车" << i << "度位置如下：" << std::endl;

                    int j = 0;
                    for(j=0;j<50;j++)
                    {
                        current_pose_message = current_pose_listener->get_current_pose();
                        some_poses[j].current_x     = current_pose_message.current_x;
                        some_poses[j].current_y     = current_pose_message.current_y;
                        some_poses[j].current_theta = current_pose_message.current_theta;
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    }

                    current_pose.current_x     = 0;
                    current_pose.current_y     = 0;
                    current_pose.current_theta = 0;
                    for(j=0;j<50;j++)
                    {
                        current_pose.current_x     += some_poses[j].current_x;
                        current_pose.current_y     += some_poses[j].current_y;
                        current_pose.current_theta += some_poses[j].current_theta;
                    }
                    current_pose.current_x     = current_pose.current_x/50;
                    current_pose.current_y     = current_pose.current_y/50;
                    current_pose.current_theta = current_pose.current_theta/50;

                    std::cout << current_pose.current_x << " " << current_pose.current_y << " " << current_pose.current_theta << std::endl;

                    i = i + 10;
                }
            }
            else if (userInput == "7")
            {
                while (true)
                {
                    current_pose_message = current_pose_listener->get_current_pose();
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            else if (userInput == "5")
            {
                std::cout << "请指定货叉高度 (单位:mm) (100-1000)" << std::endl;
                std::cin >> heightInput;
                if (heightInput < 0 || heightInput > 1000)
                {
                    std::cout << "非法输入！" << std::endl;
                }
                else
                {
                    fork_action_client->send_fork_goal(1, heightInput);
                    // std::cout << "nothing happened" << std::endl;
                }
            }
            else if (userInput == "1")
            {
                // 确保状态发布的多线程在获取位姿之后启动
                while (!current_pose_listener->get_pose)
                {

                    std::cout << "等待系统获取AGV当前位姿信息！" << std::endl;
                    // 休眠500ms
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                std::cout << "请选择轨迹类型：  1.直线  2.贝塞尔曲线" << std::endl;
                
                int trajectory_choice;
                std::cin >> trajectory_choice;
                
                Point start_point;
                Point end_point;

                // 初始化轨迹数据结构
                Trajectory trajectory;
                
                if (trajectory_choice == 1) {
                    // 直线轨迹
                    std::cout << "\n=== 直线轨迹设置 ===" << std::endl;
                    
                    // 输入起点坐标
                    std::cout << "请输入起点坐标 (x y theta)：" << std::endl;
                    std::cin >> start_point.x >> start_point.y >> start_point.theta;
                    
                    // 输入终点坐标
                    std::cout << "请输入终点坐标 (x y theta)：" << std::endl;
                    std::cin >> end_point.x >> end_point.y >> end_point.theta;
                    
                    trajectory.degree = 0; // 直线轨迹
                    trajectory.knots.clear();
                    trajectory.control_points.clear();
                    
                    std::cout << "\n直线轨迹设置完成:" << std::endl;
                    std::cout << "  起点: (" << start_point.x << ", " << start_point.y << ", " << start_point.theta << ")" << std::endl;
                    std::cout << "  终点: (" << end_point.x << ", " << end_point.y << ", " << end_point.theta << ")" << std::endl;
                } 
                else if (trajectory_choice == 2) {
                    // 贝塞尔曲线轨迹
                    std::cout << "\n=== 贝塞尔曲线轨迹设置 ===" << std::endl;
                    
                    // 输入起点坐标
                    std::cout << "请输入起点坐标 (x y theta)：" << std::endl;
                    std::cin >> start_point.x >> start_point.y >> start_point.theta;
                    
                    // 输入终点坐标
                    std::cout << "请输入终点坐标 (x y theta)：" << std::endl;
                    std::cin >> end_point.x >> end_point.y >> end_point.theta;
                    
                    // 输入控制点1
                    ControlPoint cp1;
                    std::cout << "请输入控制点1坐标 (x y)：" << std::endl;
                    std::cin >> cp1.x >> cp1.y;
                    cp1.weight = 1.0;
                    
                    // 输入控制点2
                    ControlPoint cp2;
                    std::cout << "请输入控制点2坐标 (x y)：" << std::endl;
                    std::cin >> cp2.x >> cp2.y;
                    cp2.weight = 1.0;
                    
                    trajectory.degree = 1; // 使用贝塞尔曲线模式
                    trajectory.knots.clear();
                    trajectory.control_points.clear();
                    
                    // 对于贝塞尔曲线，需要包含起点、控制点和终点
                    // 起点
                    ControlPoint start_cp = {start_point.x, start_point.y, 1.0};
                    trajectory.control_points.push_back(start_cp);
                    
                    // 控制点
                    trajectory.control_points.push_back(cp1);
                    trajectory.control_points.push_back(cp2);
                    
                    // 终点
                    ControlPoint end_cp = {end_point.x, end_point.y, 1.0};
                    trajectory.control_points.push_back(end_cp);
                    
                    std::cout << "\n贝塞尔曲线轨迹设置完成:" << std::endl;
                    std::cout << "  起点: (" << start_point.x << ", " << start_point.y << ", " << start_point.theta << ")" << std::endl;
                    std::cout << "  控制点1: (" << cp1.x << ", " << cp1.y << ")" << std::endl;
                    std::cout << "  控制点2: (" << cp2.x << ", " << cp2.y << ")" << std::endl;
                    std::cout << "  终点: (" << end_point.x << ", " << end_point.y << ", " << end_point.theta << ")" << std::endl;
                }
                else {
                    std::cout << "无效选择, 退出程序" << std::endl;
                    return false;
                }

                // 选择正反向
                std::cout << "\n请选择行驶方向：  1.正向  2.反向" << std::endl;
                int direction_choice;
                std::cin >> direction_choice;
                double edge_orientation;
                if (direction_choice == 1) {
                    edge_orientation = 0.0;
                    std::cout << "行驶方向: 正向" << std::endl;
                } else if (direction_choice == 2) {
                    edge_orientation = agv_config.PI;
                    std::cout << "行驶方向: 反向" << std::endl;
                } else {
                    std::cout << "无效选择, 退出程序" << std::endl;
                    return false;
                }

                std::vector<Point> goal_points = math_tool.generateTrajectory(start_point, end_point, trajectory, edge_orientation);
                
                // 显示生成的轨迹信息
                std::cout << "\n=== 轨迹生成完成 ===" << std::endl;
                std::cout << "生成的轨迹点数量: " << goal_points.size() << std::endl;
                
                if (trajectory_choice == 1) {
                    std::cout << "轨迹类型: 直线" << std::endl;
                } else if (trajectory_choice == 2) {
                    std::cout << "轨迹类型: 贝塞尔曲线" << std::endl;
                } else {
                    std::cout << "轨迹类型: 直线 (默认)" << std::endl;
                }
                
                std::cout << "\n详细轨迹点坐标:" << std::endl;
                std::cout << "格式: (x, y, theta)" << std::endl;
                std::cout << "==================" << std::endl;


                // 在此处将goal_points打印出来
                for (size_t i = 0; i < goal_points.size(); ++i) {
                    const auto& point = goal_points[i];
                    std::cout << "("  << std::fixed << std::setprecision(5) << point.x << ", " << point.y << ", " << point.theta << ")" << std::endl;
                }
                
                std::cout << "==================" << std::endl;

                // 告知EXC准备进行轨迹导航
                auto future_and_requestid = send_multi_pose->send_request(goal_points);
                auto result = future_and_requestid.future.get();

                int connect_times = 0;
                while (result->response != true)
                {
                    connect_times++;
                    if (connect_times > 10)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "10 times fail to start guide!");
                        return false;
                    }
                    auto future_and_requestid = send_multi_pose->send_request(goal_points);
                    auto result = future_and_requestid.future.get();
                }

                // 发送目标位姿
                std::vector<Point> multi_poses = {goal_points.back()}; // 发给多点导航的点集只需要头尾即可
                send_multi_pose->send_goal(multi_poses, true);

                send_multi_pose->flag_finish = false;

                current_pose_message = current_pose_listener->get_current_pose();
                std::cout << "小车实际到达位置：" << std::endl;

                // 接下来打印小车到达终点前的经过点坐标
                while (!send_multi_pose->flag_finish && (abs(current_pose_message.current_x-(end_point.x)) > 0.1 || abs(current_pose_message.current_y-(end_point.y)) > 0.1))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    current_pose_message = current_pose_listener->get_current_pose();
                    std::cout << "(" << current_pose_message.current_x << ", " << current_pose_message.current_y << ", " << current_pose_message.current_theta << ")" << std::endl;
                }

                while(!send_multi_pose->flag_finish)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                current_pose_message = current_pose_listener->get_current_pose();
                std::cout << "小车完成导航任务，最终坐标：" << std::endl;
                std::cout << "(" << current_pose_message.current_x << ", " << current_pose_message.current_y << ", " << current_pose_message.current_theta << ")" << std::endl;
                

            }
            else if (userInput == "2")
            {
                std::cout << "请输入多点导航的点数量:" << std::endl;
                std::cin >> point_num;
                if (point_num == 1 || point_num == 2 || point_num == 3 || point_num == 4 || point_num == 5 ||
                    point_num == 6 || point_num == 7 || point_num == 8 || point_num == 9 || point_num == 10)
                {
                    std::cout << "请在下面" << point_num << "行输入这些点坐标（示例： 8 0 3.14）" << std::endl;
                    std::vector<double> goal_x_to_driver, goal_y_to_driver, goal_theta_to_driver;
                    std::vector<Point> goal_points;

                    for (i = 0; i < point_num; i++)
                    {
                        std::cin >> x >> y >> theta;
                        Point pointA = {x, y, theta};
                        goal_points.push_back(pointA);
                    }
                    std::cout << "小车将按照以下顺序进行导航:" << std::endl;
                    for (i = 0; i < point_num; i++)
                    {
                        std::cout << "(" << goal_points[i].x << "," << goal_points[i].y << "," << goal_points[i].theta << ")" << std::endl;
                    }

                    send_multi_pose->send_goal(goal_points, true);
                }
                else
                {
                    std::cout << "非法输入！点数量范围为[1~10]" << std::endl;
                }
            }
            else if (userInput == "3")
            {
                bool flag = request_charging_client_->connect_server();
                // 2、根据连接结果进行下一步
                if (!flag)
                {
                    RCLCPP_ERROR(all_nodes->get_logger(), "fail to connect server!");
                    return false;
                }

                int connect_times = 0; // 连接次数

                // 3、调用发送数据的函数(最多尝试10次)
                RCLCPP_ERROR(all_nodes->get_logger(), "发送充电请求!");
                auto future_and_requestid = request_charging_client_->send_request(start_charge);
                while (!(rclcpp::spin_until_future_complete(all_nodes, future_and_requestid.future) == rclcpp::FutureReturnCode::SUCCESS))
                {
                    connect_times++;
                    if (connect_times > 10)
                    {
                        RCLCPP_INFO(all_nodes->get_logger(), "10s!fail to response!");
                        return 0;
                    }
                    auto future_and_requestid = request_charging_client_->send_request(start_charge);
                }

                RCLCPP_INFO(all_nodes->get_logger(), "成功切换至充电模式！");
            }
            else if (userInput == "4")
            {
                bool flag = request_charging_client_->connect_server();
                // 2、根据连接结果进行下一步
                if (!flag)
                {
                    RCLCPP_ERROR(all_nodes->get_logger(), "fail to connect server!");
                    return false;
                }

                int connect_times = 0; // 连接次数

                // 3、调用发送数据的函数(最多尝试10次)
                RCLCPP_ERROR(all_nodes->get_logger(), "发送放电请求!");
                auto future_and_requestid = request_charging_client_->send_request(stop_charge);
                while (!(rclcpp::spin_until_future_complete(all_nodes, future_and_requestid.future) == rclcpp::FutureReturnCode::SUCCESS))
                {
                    connect_times++;
                    if (connect_times > 10)
                    {
                        RCLCPP_INFO(all_nodes->get_logger(), "10s!fail to response!");
                        return 0;
                    }
                    auto future_and_requestid = request_charging_client_->send_request(stop_charge);
                }

                RCLCPP_INFO(all_nodes->get_logger(), "成功切换至放电模式！");
            }
            else if (userInput == "6")
            {
                auto future_and_requestid = agv_end_guide->send_request(true, false); // 表示前进
                auto result = future_and_requestid.future.get();

                std::cout << "result->is_reached:" << result->is_reached << "  result->is_out_of_error:" << result->is_out_of_error << std::endl;

                if (result->is_reached && !result->is_out_of_error)
                {
                    RCLCPP_INFO(all_nodes->get_logger(), "末端引导成功！");
                    current_pose_message = current_pose_listener->get_current_pose();
                    std::cout << "最终到达：(" << current_pose_message.current_x << ", " << current_pose_message.current_y << ", " << current_pose_message.current_theta << ")" << std::endl;
                }
                else
                {
                    RCLCPP_INFO(all_nodes->get_logger(), "末端引导失败！");
                    current_pose_message = current_pose_listener->get_current_pose();
                    std::cout << "最终到达：(" << current_pose_message.current_x << ", " << current_pose_message.current_y << ", " << current_pose_message.current_theta << ")" << std::endl;
                }
            }
            else
            {
                std::cout << "未知命令，请重新输入。" << std::endl;
            }
        }

        all_nodes_thread.join();

    }
    else if(agv_config.vehicle_type == "qr") 
    {

        std::shared_ptr<ListenerPose> current_pose_listener = std::make_shared<ListenerPose>(all_nodes);

        std::shared_ptr<RequestChargingClient> request_charging_client_ = std::make_shared<RequestChargingClient>(all_nodes);

        std::shared_ptr<QRForkActionClient> fork_action_client = std::make_shared<QRForkActionClient>(all_nodes);

        std::shared_ptr<QRSendMultiPose> send_multi_pose = std::make_shared<QRSendMultiPose>(all_nodes, current_pose_listener);

        std::thread all_nodes_thread([all_nodes]()
                                    { rclcpp::spin(all_nodes); });

        while (true)
        {
            std::cout << "欢迎使用快捷工具，请选择功能：  1.退出工具  2.多点导航  3.进入充电模式  4.进入放电模式  5.控制托盘高度  6.控制托盘角度  7.模拟任务  8.获取当前位姿" << std::endl;
            std::cin >> userInput;
            if (userInput == "1")
            {
                break;
            }
            else if (userInput == "2")
            {
                std::cout << "请输入多点导航的点数量（支持2-50个点）:" << std::endl;
                std::cin >> point_num;
                if (point_num == 2 || point_num == 3 || point_num == 4 || point_num == 5 ||
                    point_num == 6 || point_num == 7 || point_num == 8 || point_num == 9 || point_num == 10 ||
                    point_num == 11 || point_num == 12 || point_num == 13 || point_num == 14 || point_num == 15 ||
                    point_num == 16 || point_num == 17 || point_num == 18 || point_num == 19 || point_num == 20 ||
                    point_num == 21 || point_num == 22 || point_num == 23 || point_num == 24 || point_num == 25 ||
                    point_num == 26 || point_num == 27 || point_num == 28 || point_num == 29 || point_num == 30 ||
                    point_num == 31 || point_num == 32 || point_num == 33 || point_num == 34 || point_num == 35 ||
                    point_num == 36 || point_num == 37 || point_num == 38 || point_num == 39 || point_num == 40 ||
                    point_num == 41 || point_num == 42 || point_num == 43 || point_num == 44 || point_num == 45 ||
                    point_num == 46 || point_num == 47 || point_num == 48 || point_num == 49 || point_num == 50)
                {
                    std::cout << "请在下面" << point_num << "行输入这些点坐标  按照以下格式: (qr, theta, allowed_deviation_angle, x, y, obstacle_channel_select)  示例： (72 0 3.14 0 1 0)" << std::endl;
                    std::vector<agv_interfaces::msg::Poses> goal_poses_to_driver;

                    for(i=0; i<point_num; i++)
                    {
                        std::cin >> qr >> theta >> allowed_deviation_angle >> x >> y >> obstacle_channel_select;
                        agv_interfaces::msg::Poses goal_pose;
                        goal_pose.x = x;
                        goal_pose.y = y;
                        goal_pose.label = qr;
                        goal_pose.angle = theta;
                        goal_pose.allowed_deviation_angle = allowed_deviation_angle;
                        goal_pose.obstacle_channel_select = obstacle_channel_select;
                        goal_poses_to_driver.push_back(goal_pose);
                    }
                    send_multi_pose->send_goal(goal_poses_to_driver);
                }
                else
                {
                    std::cout << "非法输入！点数量范围为[2~10]" << std::endl;
                }
            }
            else if (userInput == "3")
            {
                bool flag = request_charging_client_->connect_server();
                // 2、根据连接结果进行下一步
                if (!flag)
                {
                    RCLCPP_ERROR(all_nodes->get_logger(), "fail to connect server!");
                    return false;
                }

                int connect_times = 0; // 连接次数

                // 3、调用发送数据的函数(最多尝试10次)
                RCLCPP_ERROR(all_nodes->get_logger(), "发送充电请求!");
                auto future_and_requestid = request_charging_client_->send_request(start_charge);
                while (!(rclcpp::spin_until_future_complete(all_nodes, future_and_requestid.future) == rclcpp::FutureReturnCode::SUCCESS))
                {
                    connect_times++;
                    if (connect_times > 10)
                    {
                        RCLCPP_INFO(all_nodes->get_logger(), "10s!fail to response!");
                        return 0;
                    }
                    auto future_and_requestid = request_charging_client_->send_request(start_charge);
                }

                RCLCPP_INFO(all_nodes->get_logger(), "成功切换至充电模式！");
            }
            else if (userInput == "4")
            {
                bool flag = request_charging_client_->connect_server();
                // 2、根据连接结果进行下一步
                if (!flag)
                {
                    RCLCPP_ERROR(all_nodes->get_logger(), "fail to connect server!");
                    return false;
                }

                int connect_times = 0; // 连接次数

                // 3、调用发送数据的函数(最多尝试10次)
                RCLCPP_ERROR(all_nodes->get_logger(), "发送放电请求!");
                auto future_and_requestid = request_charging_client_->send_request(stop_charge);
                while (!(rclcpp::spin_until_future_complete(all_nodes, future_and_requestid.future) == rclcpp::FutureReturnCode::SUCCESS))
                {
                    connect_times++;
                    if (connect_times > 10)
                    {
                        RCLCPP_INFO(all_nodes->get_logger(), "10s!fail to response!");
                        return 0;
                    }
                    auto future_and_requestid = request_charging_client_->send_request(stop_charge);
                }

                RCLCPP_INFO(all_nodes->get_logger(), "成功切换至放电模式！");
            }
            else if (userInput == "5")
            {
                std::cout << "请指定托盘高度 (单位:mm) (0-60)" << std::endl;
                std::cin >> heightInput;
                if (heightInput < 0 || heightInput > 60)
                {
                    std::cout << "非法输入！" << std::endl;
                }
                else
                {
                    fork_action_client->send_fork_goal(1, heightInput, 0);
                    // std::cout << "nothing happened" << std::endl;
                }
            }
            else if (userInput == "6")
            {
                std::cout << "请指定托盘角度 (单位:°) (-180-180)" << std::endl;
                std::cin >> angleInput;
                if(angleInput < -180 || angleInput > 180)
                {
                    std::cout << "非法输入！" << std::endl;
                }
                else
                {
                    fork_action_client->send_fork_goal(1,0,angleInput);
                    // std::cout << "nothing happened" << std::endl;
                }
            }
            else if (userInput == "8") //打印当前时间戳和位姿
            {
                while (true)
                {
                    auto date = std::chrono::system_clock::now();
                    current_pose_message = current_pose_listener->get_current_pose();
                    std::cout << "当前时间戳：" << date.time_since_epoch().count() << " 当前位姿：(" << current_pose_message.current_x << ", " << current_pose_message.current_y << ", " << current_pose_message.current_theta << ")" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            else if (userInput == "7") //模拟任务
            {
                // 确保状态发布的多线程在获取位姿之后启动
                while(!current_pose_listener->get_pose){
                    std::cout << "等待系统获取AGV当前位姿信息！"<< std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                // 从文件读取路线信息
                std::string route_file_path = "/home/二维码地图路线.txt";
                std::ifstream route_file(route_file_path);
                
                if (!route_file.is_open())
                {
                    std::cout << "错误：无法打开路线文件: " << route_file_path << std::endl;
                    continue;
                }

                // 存储路线点信息 [label, angle, allowed_deviation_angle, x, y]
                struct RoutePoint {
                    int label;
                    double angle;
                    double allowed_deviation_angle;
                    double x;
                    double y;
                };
                std::vector<RoutePoint> route_points;

                // 读取文件内容
                std::string line;
                while (std::getline(route_file, line))
                {
                    // 跳过空行
                    if (line.empty() || line.find_first_not_of(" \t\n\r") == std::string::npos)
                    {
                        continue;
                    }

                    std::istringstream iss(line);
                    RoutePoint point;
                    if (iss >> point.label >> point.angle >> point.allowed_deviation_angle >> point.x >> point.y)
                    {
                        route_points.push_back(point);
                    }
                    else
                    {
                        std::cout << "警告：无法解析行: " << line << std::endl;
                    }
                }
                route_file.close();

                if (route_points.empty())
                {
                    std::cout << "错误：路线文件为空或无法解析！" << std::endl;
                    continue;
                }

                std::cout << "成功读取 " << route_points.size() << " 个路线点" << std::endl;
                std::cout << "开始执行纯导航模拟路线..." << std::endl;

                // 纯导航模拟路线 - 循环执行路线
                int route_size = route_points.size();
                for(i = 0; i < route_size * 100; i++)  // 循环执行100次完整路线
                {
                    int current_idx = i % route_size;
                    int next_idx = (i + 1) % route_size;
                    int next_next_idx = (i + 2) % route_size;

                    std::vector<agv_interfaces::msg::Poses> goal_poses_to_driver;
                    
                    // 当 next_idx 对应的 label 为 75 或 129 时，只发送2个点；否则发送3个点
                    bool send_two_points = (route_points[next_idx].label == 75 || route_points[next_idx].label == 129);
                    
                    agv_interfaces::msg::Poses pose1;
                    pose1.x = route_points[current_idx].x;  // 使用文件中的坐标
                    pose1.y = route_points[current_idx].y;
                    pose1.label = route_points[current_idx].label;
                    pose1.angle = route_points[current_idx].angle;
                    pose1.allowed_deviation_angle = route_points[current_idx].allowed_deviation_angle;
                    goal_poses_to_driver.push_back(pose1);
                    
                    agv_interfaces::msg::Poses pose2;
                    pose2.x = route_points[next_idx].x;
                    pose2.y = route_points[next_idx].y;
                    pose2.label = route_points[next_idx].label;
                    pose2.angle = route_points[next_idx].angle;
                    pose2.allowed_deviation_angle = route_points[next_idx].allowed_deviation_angle;
                    goal_poses_to_driver.push_back(pose2);
                    
                    if (!send_two_points)  // 不是75或129时，发送第3个点
                    {
                        agv_interfaces::msg::Poses pose3;
                        pose3.x = route_points[next_next_idx].x;
                        pose3.y = route_points[next_next_idx].y;
                        pose3.label = route_points[next_next_idx].label;
                        pose3.angle = route_points[next_next_idx].angle;
                        pose3.allowed_deviation_angle = route_points[next_next_idx].allowed_deviation_angle;
                        goal_poses_to_driver.push_back(pose3);
                    }
                    
                    send_multi_pose->send_goal(goal_poses_to_driver);

                    std::cout << "------------------------------" << std::endl;
                    if (send_two_points)
                    {
                        std::cout << "目标点序列: " 
                                << route_points[current_idx].label << " (angle: " 
                                << route_points[current_idx].angle << ", dev: " 
                                << route_points[current_idx].allowed_deviation_angle << ")"
                                << " -> " 
                                << route_points[next_idx].label << " (angle: " 
                                << route_points[next_idx].angle << ", dev: " 
                                << route_points[next_idx].allowed_deviation_angle << ")"
                                << std::endl;
                    }
                    else
                    {
                        std::cout << "目标点序列: " 
                                << route_points[current_idx].label << " (angle: " 
                                << route_points[current_idx].angle << ", dev: " 
                                << route_points[current_idx].allowed_deviation_angle << ")"
                                << " -> " 
                                << route_points[next_idx].label << " (angle: " 
                                << route_points[next_idx].angle << ", dev: " 
                                << route_points[next_idx].allowed_deviation_angle << ")"
                                << " -> " 
                                << route_points[next_next_idx].label << " (angle: " 
                                << route_points[next_next_idx].angle << ", dev: " 
                                << route_points[next_next_idx].allowed_deviation_angle << ")"
                                << std::endl;
                    }
                    std::cout << "------------------------------" << std::endl;

                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    send_multi_pose->flag_finish = false;

                    // 等待到达下一个目标点：检查当前坐标与目标点的差值是否在0.3之内
                    std::cout << "正在导航到目标二维码: " << route_points[next_idx].label 
                            << " 目标坐标: (" << route_points[next_idx].x << ", " 
                            << route_points[next_idx].y << ")" << std::endl;
                    
                    current_pose_message = current_pose_listener->get_current_pose();
                    double target_x = route_points[next_idx].x;
                    double target_y = route_points[next_idx].y;
                    
                    // 检查当前坐标与目标点的差值是否在0.3之内
                    while(abs(target_x - current_pose_message.current_x) > 0.3 || 
                          abs(target_y - current_pose_message.current_y) > 0.3)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        current_pose_message = current_pose_listener->get_current_pose();
                        std::cout << "导航中，当前坐标：(" << current_pose_message.current_x << ", " 
                                << current_pose_message.current_y << ")" << std::endl;
                    }
                    
                    std::cout << "完成到达点: " << route_points[next_idx].label << std::endl;
                    std::cout << "目标坐标：(" << target_x << ", " << target_y << ")" << std::endl;
                    std::cout << "实际坐标：(" << current_pose_message.current_x << ", " 
                            << current_pose_message.current_y << ")" << std::endl;
                    std::cout << "==============================" << std::endl;
                    
                    // 当小车到达75时，进行抬升货叉动作
                    if (route_points[next_idx].label == 75)
                    {

                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        send_multi_pose->flag_finish = false;

                        // 纯导航模式：直接等待导航完成（坐标不重要，设为0,0）
                        std::cout << "等待导航彻底到达75点..." << std::endl;
                        
                        // 等待导航彻底完成
                        while(!send_multi_pose->flag_finish)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }

                        std::cout << "彻底到达75点，准备抬升货叉..." << std::endl;
                        fork_action_client->flag_finish = false;
                        fork_action_client->send_fork_goal(1, 30, 0);  // 抬升到30mm高度
                        while(!fork_action_client->flag_finish)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        std::cout << "货叉抬升完成" << std::endl;
                        std::cout << "==============================" << std::endl;
                    }

                    // 当小车到达129时，进行降低货叉动作
                    if (route_points[next_idx].label == 129)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        send_multi_pose->flag_finish = false;

                        // 纯导航模式：直接等待导航完成（坐标不重要，设为0,0）
                        std::cout << "等待导航彻底到达129点..." << std::endl;
                        
                        // 等待导航彻底完成
                        while(!send_multi_pose->flag_finish)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }

                        std::cout << "彻底到达129点，准备降低货叉..." << std::endl;
                        fork_action_client->flag_finish = false;
                        fork_action_client->send_fork_goal(1, 0, 0);  // 降低到0mm高度
                        while(!fork_action_client->flag_finish)
                        {
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        std::cout << "货叉降低完成" << std::endl;
                        std::cout << "==============================" << std::endl;
                    }

                }
                
                std::cout << "路线执行完成！" << std::endl;
            }
            else
            {
                std::cout << "未知命令，请重新输入。" << std::endl;
            }
        }

        all_nodes_thread.join();

    }


    return 0;

    /*-------------------------------------------------------------------------------------------------------------*/
}