/************************************** File Info ****************************************
* @file:       state_running_behaviors.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-11                                           
* @version:    V0.0                                                                              
* @brief:      agv运行状态，待完善
------------------------------------------------------------------------------------------
* @modified:   通过可变长数组存储一段任务的所有目标点，到达目标点之后直接更新到下一个点位
* @date:       2024-11-30
* @version:    V0.0
* @description:卡顿的问题是因为到达目标点之后，停在附近，上传数据，等到order接受到新任务的时候再运行到下一个点位
* @note:       代码已优化，还没测试
------------------------------------------------------------------------------------------
* @modified:   通过动作通信客户端的形式进行控制数据的发送
* @date:       2024-11-10
* @version:    V0.0
* @description:话题形式发送缺失一个行为树的数据，可能是这个导致部分导航问题，riviz上的也是动作客户端的形式
* @note:       代码已优化还没测试
******************************************************************************************/
# include "state_running_behaviors.h"
#include <map>
#include <string>
#include <cctype>


bool if_angle_qualify(double current_angle, double goal_angle, double percision)
{
    return ( fmod(abs(current_angle - goal_angle),6.28)<=percision ||  fmod(abs(current_angle - goal_angle),6.28)>=(6.28-percision) );
}

/**
 * @brief 从node_id字符串中提取数字部分
 * @param node_id 节点ID字符串，例如 "qrcode_one_122"
 * @param default_value 如果提取失败，返回的默认值
 * @return 提取的数字，例如从 "qrcode_one_122" 提取出 122
 * 
 * 该函数会从字符串末尾向前查找连续的数字字符，提取出数字部分
 */
int extract_number_from_node_id(const std::string& node_id, int default_value = -1)
{
    if(node_id.empty()) {
        return default_value;
    }
    
    // 从字符串末尾向前查找连续的数字字符
    std::string number_str;
    for(int i = node_id.length() - 1; i >= 0; i--) {
        if(std::isdigit(node_id[i])) {
            number_str = node_id[i] + number_str;  // 在字符串前面插入，保持顺序
        } else {
            // 如果遇到非数字字符，且已经找到数字，则停止
            if(!number_str.empty()) {
                break;
            }
        }
    }
    
    // 如果找到了数字字符串，尝试转换
    if(!number_str.empty()) {
        try {
            return std::stoi(number_str);
        } catch(const std::exception&) {
            // 转换失败，返回默认值
            return default_value;
        }
    }
    
    // 如果没有找到数字，返回默认值
    return default_value;
}


LaserRunningStateBehaviors::LaserRunningStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone) : 
    SyncActionNode(name, config),
    instant_action_listener(agv_bone.agv_instant_action_listener),
    current_pose_listener(agv_bone.agv_current_pose_listener),
    order_listener(agv_bone.agv_order_listener),
    battery_listener(agv_bone.agv_battery_listener),
    velocity_listener(agv_bone.agv_velocity_listener),
    obstacle_server(agv_bone.agv_obstacle_server),
    agv_data_publish_(agv_bone.agv_data_publish),
    can_data_listener_(agv_bone.agv_can_data_listener)
    {

    // 初始化驱动控制单例
    LaserDriverControl::initialize(agv_bone);
    // 获取驱动控制单例
    agv_driver_control = LaserDriverControl::getInstance();
    // 设置驱动控制函数，用于同步driving状态
    agv_data_publish_->set_driving_fn([this]() { return agv_driver_control->get_flag_driving(); });

    // 实例化末端动作控制（包含货叉控制、充电控制）
    execute_action_ = std::make_shared<LaserExecuteAction>(agv_bone);

    // 实例化末端引导客户端
    agv_end_guide = std::make_shared<EndGuideClient>(agv_bone.agv_all_nodes);

    // 任务节点索引
    point_index = 0;
    far_point_index = 0;

    obstacle_wait_time = 0;
    current_obstacle_avoidance_channels_ = -1;
    // 初始化 是否到达目标点标签设置为true
    if_reach_point = true;
    if_order_updated = true;
    if_to_leave = false;
    flag_pausing = false;
    restored_drive_state_ = true;

    goal_points_to_driver = {};

    math_tool = Math_Tool();
}


NodeStatus LaserRunningStateBehaviors::tick()
{
    std::cout << this->name() << " 进入LaserRunningStateBehaviors::tick() " << std::endl;

    // 预备工作
    // 更新instantAction消息的数据
    instant_action_messages_ = instant_action_listener->get_instant_action_messages();
    // 更新order消息
    order_messages_ = order_listener->get_order_messages();
    // 更新当前位姿数据
    current_pose_ = current_pose_listener->get_current_pose();

    // 一次执行OnReceiveOrder()的过程中，会阻塞行为树，不用担心10s一次的tick会重置标志位，导致执行失效
    // 任务节点索引
    point_index = 0;
    // vda5050的执行思路中，不仅关注下一个目标点，还关注关键操作点（停靠，充电等）
    far_point_index = 0;

    obstacle_wait_time = 0;
    current_obstacle_avoidance_channels_ = -1;
    // 初始化 是否到达目标点标签设置为true
    if_reach_point = true;
    if_order_updated = true;
    flag_pausing = false; // 初始化暂停标志为false

    goal_points_to_driver = {};

    // 主逻辑
    // 根据can传来的是否有货的消息，更新是否载货
    if(can_data_listener_->update_load_status == 1 || can_data_listener_->update_load_status == 2){
        execute_action_->set_loading(can_data_listener_->update_load_status == 1 ? true : false);
        can_data_listener_->update_load_status = 0;
    }

    std::cout << this->name() << " 正在执行:OnReceiveOrder() " << std::endl;
    OnReceiveOrder();

    // 收尾工作
    // 导航完成，导航任务标志重置
    agv_driver_control->set_flag_finish(false);
    return NodeStatus::SUCCESS;
}


void LaserRunningStateBehaviors::OnReceiveOrder(){

    // 标志导航状态是否正常
    bool drive_state = true;

    far_point_index = 0;
    
    // 1、instant_action发布的任务
    if(instant_action_messages_.action_type == "connect_success_auto"){

        goal_points_to_driver = {};
        // instant_action瞬时任务，只有一个目标点
        Point one_point = {instant_action_messages_.goal_x, instant_action_messages_.goal_y, instant_action_messages_.goal_theta};
        goal_points_to_driver.push_back(one_point);
        
        // agv_driver_control有下发任务，取消任务等等大量功能，send_multi_pose_只负责发送目标点
        std::cout << this->name() << " 往autoware发送目标点send_goal " << std::endl;
        agv_driver_control->send_multi_pose_->send_goal(goal_points_to_driver, true);
        // 处理完当前消息，action_type置为没收到消息
        instant_action_listener->action_type = "no_message_received";
        // 判断是否完成本次导航，初始值为false
        while (!agv_driver_control->get_flag_finish() && !agv_driver_control->get_flag_aborted())
        {
            // 休眠100ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // 完成instantAction的任务，将任务标识置false
        // 导航是否完成，这个标志位初始化成false，防止初始化成true，每次tick的时候重新初始一遍出问题，有点混乱，他的功能可以完全被flag_driving取代
        agv_driver_control->set_flag_finish(false);
    }

    // 2、order发布的任务，需要完成的是一系列任务
    while(order_listener->get_order && drive_state){

        // 数据更新函数，更新的数据包括：instantAction消息数据、order消息数据、当前位姿消息数据
        if(!data_updates())
        {
            drive_state = false;
            break;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "目标坐标(%.2f,%.2f,%.2f),当前坐标(%.2f,%.2f,%.2f),",order_messages_.goal_x[point_index],order_messages_.goal_y[point_index],order_messages_.goal_theta[point_index],current_pose_.current_x,current_pose_.current_y,current_pose_.current_theta);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "允许偏差(%.2f,%.2f,%.2f)",order_messages_.goal_allowed_deviation_xy[point_index],order_messages_.goal_allowed_deviation_xy[point_index],order_messages_.goal_allowed_deviation_theta[point_index]);
        // 完成分段式导航任务

        // 可能要更改避障通道
        if(point_index > 0 && current_obstacle_avoidance_channels_ != order_messages_.edge_obstacle_avoidance_channels[point_index-1])
        {
            // 此处需要向底层发送激光区域通道选择。
            current_obstacle_avoidance_channels_ = order_messages_.edge_obstacle_avoidance_channels[point_index-1];
            obstacle_server->publish_obstacle_channels(current_obstacle_avoidance_channels_, current_obstacle_avoidance_channels_, current_obstacle_avoidance_channels_, current_obstacle_avoidance_channels_);
        }

        // 如果目标点和当前位姿存在误差，则启动驱动，否则不启动驱动
        // if(abs(order_messages_.goal_x[point_index] - current_pose_.current_x)> high_distance_precision || abs(order_messages_.goal_y[point_index] - current_pose_.current_y)>high_distance_precision || abs(order_messages_.goal_theta[point_index] - current_pose_.current_theta)>high_angle_precision){
        // if_reach_point初始化为true，进入下面else if的条件
        if((!if_reach_point) && (abs(order_messages_.goal_x[point_index] - current_pose_.current_x) > order_messages_.goal_allowed_deviation_xy[point_index] || abs(order_messages_.goal_y[point_index] - current_pose_.current_y) > order_messages_.goal_allowed_deviation_xy[point_index] || !if_angle_qualify(order_messages_.goal_theta[point_index], current_pose_.current_theta, order_messages_.goal_allowed_deviation_theta[point_index])))
        {
            // far_point_index是一小段路径的终点（特殊操作点）或者路径终点，如果point_index > far_point_index当前索引已经超过终点索引，就代表上一段路径走完了，也是一种意义上的订单更新
            if(point_index > far_point_index || if_order_updated == true) // 如果发现当前目标点是全新的（未处理过的）|| RCS发来的order更新了，则需要进行多点导航
            {
                goal_points_to_driver = {};

                // // 离开取货点，和进入取货点，两种情况用末端引导的方式导航，qqa暂时无需末端引导
                // if(if_to_leave == true) // 如果小车将要离开取货点
                // {
                //     auto future_and_requestid = agv_end_guide->send_request(false, true); // 表示后退
                //     auto result = future_and_requestid.future.get();
                //     if(result->is_reached)
                //     {
                //         if_reach_point = true;
                //         if_to_leave = false; // 小车已经成功离开取货点
                //         agv_driver_control->set_flag_finish(true);
                //         std::cout << "小车已通过末端引导离开取货点！" << std::endl;
                //         continue;
                //     }
                //     else
                //     {
                //         order_listener->get_order = false; // 强制结束 running state，进入lock态
                //         drive_state = false;
                //         break;
                //     }
                // }
                // // 末端引导需要小车已经处于一个确定的位置，通常是上一个点 point_index-1，当 degree == -1 时，表示这条边上没有有效的控制点，无法生成常规的贝塞尔/多项式轨迹，用于标记那些需要 AGV 进行特殊操作（如精准取货、充电对接）的线段
                // if(point_index > 1 && order_messages_.goal_trajectory[point_index-1].degree == -1) // 如果小车将要进行一段无轨迹线段的行驶，则开启末端引导
                // {
                //     // 可能还有存留的导航任务会干扰末端引导微运动，所以需要先等待导航任务做完。
                //     while(agv_driver_control->get_flag_driving()){
                //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "在做末端引导前，先等待导航完成...");
                //         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                //         // 以下是暂停业务，如果收到的action的type是暂停，就暂停动作************************************************************************************************************
                //         handle_pause(true, false);
                //         if(flag_pausing == false) {
                //             drive_state = restored_drive_state_;
                //         }
                //         // 以上是暂停业务************************************************************************************************************
                //     }
                //     auto future_and_requestid = agv_end_guide->send_request(true, false); // 表示前进
                //     auto result = future_and_requestid.future.get();
                //     if(result->is_reached)
                //     {
                //         if_reach_point = true;
                //         if_to_leave = true; // 接下来小车将要离开取货点
                //         // 由于进行末端引导前将导航任务取消了（这会导致任务没有成功执行完），但其实任务是被手动取消的，所以需要将导航任务的完成标志置为true
                //         agv_driver_control->set_flag_finish(true);
                //         std::cout << "小车已通过末端引导到达取货点！" << std::endl;
                //         continue;
                //     }
                //     else
                //     {
                //         order_listener->get_order = false; // 强制结束 running state，进入lock态
                //         drive_state = false;
                //         break;
                //     }
                // }
                // // 在生成当前边(point_index-1 -> point_index)之前，先把上一条边(point_index-2 -> point_index-1)的轨迹加进来，多段轨迹一次性下发，从而连续行驶，防止小车一走一停
                if (point_index >= 2 && order_messages_.point_types[point_index-1] == PointType::ORDINARY) {
                    Point prev_start = {order_messages_.goal_x[point_index-2], order_messages_.goal_y[point_index-2], order_messages_.goal_theta[point_index-2]};
                    Point prev_end   = {order_messages_.goal_x[point_index-1], order_messages_.goal_y[point_index-1], order_messages_.goal_theta[point_index-1]};
                    std::vector<Point> prev_edge_points = math_tool.generateTrajectory(
                        prev_start,
                        prev_end,
                        order_messages_.goal_trajectory[point_index-2],
                        order_messages_.goal_edge_orientation[point_index-2]
                    );
                    goal_points_to_driver.insert(goal_points_to_driver.end(), prev_edge_points.begin(), prev_edge_points.end());
                }

                // 将当前路径加入轨迹
                Point start_point   = {order_messages_.goal_x[point_index-1], order_messages_.goal_y[point_index-1], order_messages_.goal_theta[point_index-1]};
                Point end_point     = {order_messages_.goal_x[point_index]  , order_messages_.goal_y[point_index]  , order_messages_.goal_theta[point_index]};
                
                // std::cout << "=== DEBUG: 准备生成轨迹 ===" << std::endl;
                // std::cout << "Point index: " << point_index << std::endl;
                // std::cout << "Start point (from index " << (point_index-1) << "): (" 
                //           << start_point.x << ", " << start_point.y << ", " << start_point.theta << ")" << std::endl;
                // std::cout << "End point (to index " << point_index << "): (" 
                //           << end_point.x << ", " << end_point.y << ", " << end_point.theta << ")" << std::endl;
                // std::cout << "Trajectory index: " << (point_index-1) << std::endl;
                // std::cout << "Trajectory control points size: " << order_messages_.goal_trajectory[point_index-1].control_points.size() << std::endl;
                // std::cout << "Trajectory degree: " << order_messages_.goal_trajectory[point_index-1].degree << std::endl;
                // std::cout << "Edge orientation: " << order_messages_.goal_edge_orientation[point_index-1] << std::endl;
                
                // 比如说，我现在的目标点是第5个点，那么我会关注第4个点到第5个点之间的轨迹。而这条曲线是第4条边。
                std::vector<Point> goal_points = math_tool.generateTrajectory(start_point, end_point, order_messages_.goal_trajectory[point_index-1], order_messages_.goal_edge_orientation[point_index-1]);

                // std::cout << "Generated " << goal_points.size() << " goal points for trajectory" << std::endl;
                // std::cout << "Current goal_points_to_driver size before insert: " << goal_points_to_driver.size() << std::endl;

                goal_points_to_driver.insert(goal_points_to_driver.end(), goal_points.begin(), goal_points.end()); // 将goal_points加入到goal_points_to_driver的末尾

                // std::cout << "Updated goal_points_to_driver size after insert: " << goal_points_to_driver.size() << std::endl;
                // std::cout << "=== END DEBUG: 轨迹生成完成 ===" << std::endl;

                // point_index后面的点，如果也是普通点，把他们也加入轨迹，直到遇到第一个为非特殊点或终点结束
                // end_point的下标是idx+1，所以这段路的终点是第一个特殊点或者全路径终点
                size_t idx = point_index; // 不管怎么样，先单独行走一个任务的第一段路线
                while( idx > 1 && (idx < order_messages_.goal_x.size()-1) && (order_messages_.point_types[idx] == PointType::ORDINARY) )
                {
                    Point start_point   = {order_messages_.goal_x[idx],   order_messages_.goal_y[idx],   order_messages_.goal_theta[idx]  };
                    Point end_point     = {order_messages_.goal_x[idx+1], order_messages_.goal_y[idx+1], order_messages_.goal_theta[idx+1]};
                    // 比如说，我现在的目标点是第5个点，那么我会关注第4个点到第5个点之间的轨迹。而这条曲线是第4条边。
                    std::vector<Point> goal_points = math_tool.generateTrajectory(start_point, end_point, order_messages_.goal_trajectory[idx], order_messages_.goal_edge_orientation[idx]);

                    goal_points_to_driver.insert(goal_points_to_driver.end(), goal_points.begin(), goal_points.end()); // 将goal_points加入到goal_points_to_driver的末尾
                    idx++ ;
                }

                far_point_index = idx; // 当发送完一次任务后，far_point_index一般指向首个特殊点 或者 任务的终点

                // std::cout << "=== DEBUG: 发送轨迹到驱动器 ===" << std::endl;
                // std::cout << "Far point index: " << far_point_index << std::endl;
                // std::cout << "Total goal_points_to_driver size: " << goal_points_to_driver.size() << std::endl;
                
                // // 打印前几个和后几个轨迹点
                // int print_count = std::min(5, (int)goal_points_to_driver.size());
                // std::cout << "First " << print_count << " trajectory points:" << std::endl;
                // for(int i = 0; i < print_count; i++) {
                //     std::cout << "  Point[" << i << "]: (" 
                //               << goal_points_to_driver[i].x << ", " 
                //               << goal_points_to_driver[i].y << ", " 
                //               << goal_points_to_driver[i].theta << ")" << std::endl;
                // }
                
                // if(goal_points_to_driver.size() > print_count) {
                //     std::cout << "Last " << print_count << " trajectory points:" << std::endl;
                //     int start_idx = goal_points_to_driver.size() - print_count;
                //     for(int i = start_idx; i < (int)goal_points_to_driver.size(); i++) {
                //         std::cout << "  Point[" << i << "]: (" 
                //                   << goal_points_to_driver[i].x << ", " 
                //                   << goal_points_to_driver[i].y << ", " 
                //                   << goal_points_to_driver[i].theta << ")" << std::endl;
                //     }
                // }
                // 先将目标点赋值给类变量，然后在通过control()发送目标点到多点导航的服务端

                // 临时方案
                goal_points_to_driver = {};
                // forward = true;
                // for (const auto& value : order_messages_.goal_edge_orientation) {
                //     std::cout << value << " ";
                // }
                // std::cout << std::endl;
                std::cout << "AAAAAAAAAAAAAAASSSSSSSSSSSSSSSSSSSSSSSASAAAAAAAA车辆前进模式: " << order_messages_.goal_edge_orientation[0] << std::endl;

                if (order_messages_.goal_edge_orientation[0] == 0) {
                    // 如果是切向行驶，直接使用目标点
                    forward = true;
                } else{
                    forward = false;
                }
                // instant_action瞬时任务，只有一个目标点
                Point one_point = {order_messages_.goal_x[point_index], order_messages_.goal_y[point_index], order_messages_.goal_theta[point_index]};
                goal_points_to_driver.push_back(one_point);
                agv_driver_control->setPostion(goal_points_to_driver);
                agv_driver_control->setForward(forward);
                drive_state = agv_driver_control->control();
                
                // std::cout << "Drive state result: " << drive_state << std::endl;
                // std::cout << "=== END DEBUG: 驱动器轨迹发送完成 ===" << std::endl;

                if_order_updated = false;

            }

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "即将驱动到目标位置（%.2f,%.2f,%.2f）",order_messages_.goal_x[point_index],order_messages_.goal_y[point_index],order_messages_.goal_theta[point_index]);


            // 其实小车在运行过程中，此循环占据了主进程的大部分
            // 主线程阻塞直到 AGV 到达当前目标点（point_index），或者发生异常/超时
            int times = 0;
            // x,y,theta某个误差超过偏差
            while(abs(order_messages_.goal_x[point_index] - current_pose_.current_x) > order_messages_.goal_allowed_deviation_xy[point_index] || abs(order_messages_.goal_y[point_index] - current_pose_.current_y) > order_messages_.goal_allowed_deviation_xy[point_index] || !if_angle_qualify(order_messages_.goal_theta[point_index], current_pose_.current_theta, order_messages_.goal_allowed_deviation_theta[point_index]))
            {
                // 数据更新函数，更新的数据包括：instantAction消息数据、order消息数据、当前位姿消息数据
                // 36000ms，一小时算超时
                if(!data_updates() || times > 36000)
                {
                    // x,y,theta均已与far_point_index的误差达标
                    if(abs(order_messages_.goal_x[far_point_index] - current_pose_.current_x) <= order_messages_.goal_allowed_deviation_xy[far_point_index] && abs(order_messages_.goal_y[far_point_index] - current_pose_.current_y) <= order_messages_.goal_allowed_deviation_xy[far_point_index] && if_angle_qualify(order_messages_.goal_theta[far_point_index], current_pose_.current_theta, order_messages_.goal_allowed_deviation_theta[far_point_index]))
                    {
                        // 走到第一个特殊点，将当前索引更新到这
                        point_index = far_point_index;
                        break;
                    }
                    else
                    {
                        drive_state = false;
                        std::cout << "=== 未能在60秒内到达下一个目标点！ ===" << std::endl;
                        this->config().blackboard->set("fault_code", "DRIVE_OUT_TIME");
                        break;
                    }
                }

                // 以下是暂停业务************************************************************************************************************
                handle_pause(true, false);
                if(flag_pausing == false) {
                    drive_state = restored_drive_state_;
                }
                // 以上是暂停业务************************************************************************************************************

                std::this_thread::sleep_for(std::chrono::milliseconds(60));
                times++;
            }
            if_reach_point = true;
            // SHARP啥意思，虽然误差到了标准值内，但是导航仍未完成？那为什么不把误差调小
            if(order_messages_.point_types[point_index] == PointType::SHARP)
            {
                while(agv_driver_control->get_flag_driving()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "在离开尖点前，先等待导航完成...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                    // 以下是暂停业务************************************************************************************************************
                    handle_pause(true, false);
                    if(flag_pausing == false) {
                        drive_state = restored_drive_state_;
                    }
                    // 以上是暂停业务************************************************************************************************************
                }
            }
            // 说明当前位置已经超过了上一段的终点，上一段走完了，通过far_point_index = 0来标志
            if(point_index > far_point_index) // 当到达多点导航的最终目标时，重置far_point_index
            {
                far_point_index = 0;
            }
        
        
        }
        // 初始if_reach_point，目标位置和当前位置在误差内
        else if(
           (if_reach_point)
        || (abs(order_messages_.goal_x[point_index] - current_pose_.current_x) <= order_messages_.goal_allowed_deviation_xy[point_index] && abs(order_messages_.goal_y[point_index] - current_pose_.current_y) <= order_messages_.goal_allowed_deviation_xy[point_index] && if_angle_qualify(order_messages_.goal_theta[point_index], current_pose_.current_theta, order_messages_.goal_allowed_deviation_theta[point_index]))
        ){
            if_reach_point = false;
            
            std::cout<<"point_index:"<<point_index<<"  "<<order_messages_.msg_state.node_states.size()<<std::endl;
            // 到达目标点附近，更新last_node_id 、last_node_sequence_id，用于上报给rcs，sequence_id标识当前执行到第几个点了，通常和point_index相同，暂时没有想到区别
            order_messages_.msg_state.last_node_id = order_messages_.msg_state.node_states[point_index].node_id;
            order_messages_.msg_state.last_node_sequence_id = order_messages_.msg_state.node_states[point_index].sequence_id;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "last_node_id ： %s ", order_messages_.msg_state.node_states[point_index].node_id.c_str());
            
            // 同步msg_state到order_listener
            order_listener->set_msg_state(order_messages_.msg_state);
            
            if(!data_updates())
            {
                drive_state = false;
                break;
            }

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"order_messages_.msg_state.order_update_id: %d",order_messages_.msg_state.order_update_id);

            agv_data_publish_->state_timer_callback();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "运行状态更新state!  即将目标坐标(%.2f,%.2f,%.2f),当前坐标(%.2f,%.2f,%.2f)",order_messages_.goal_x[point_index],order_messages_.goal_y[point_index],order_messages_.goal_theta[point_index],current_pose_.current_x,current_pose_.current_y,current_pose_.current_theta);
            // 上报完状态之后，需要等待200ms，给rcs足够时间发来反馈或者新任务
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            // 有动作的时候，到了位置也不一定执行完，还要等货叉什么的不动了
            if(order_messages_.point_types[point_index] == PointType::HARDACTION){

                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "即将执行HARD动作，需要等待导航完成...");
                while(agv_driver_control->get_flag_driving()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "等待导航完成...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                    // 以下是暂停业务************************************************************************************************************
                    handle_pause(true, false);
                    if(flag_pausing == false) {
                        drive_state = restored_drive_state_;
                    }
                    // 以上是暂停业务************************************************************************************************************
                }
            }

            // 调度系统发来取消任务的即时指令
            interrupt_order_message_ = instant_action_listener->get_interrupt_order_message();
            if(interrupt_order_message_.action_type == "cancelOrder" && interrupt_order_message_.action_status == "WAITING") // 表明现在要取消任务
            {
                // 先反馈正在执行取消任务
                instant_action_listener->update_interrupt_order_message_status("RUNNING");
                interrupt_order_message_.action_status = "RUNNING";

                ActionStates one_action_state;
                one_action_state.action_id          = interrupt_order_message_.action_id;
                one_action_state.action_type        = interrupt_order_message_.action_type;
                one_action_state.action_status      = interrupt_order_message_.action_status;
                one_action_state.action_description = interrupt_order_message_.action_description;
                order_messages_.msg_state.action_states.push_back(one_action_state);

                // 同步msg_state到order_listener
                order_listener->set_msg_state(order_messages_.msg_state);

                // 通知调度系统，我已经完成取消任务指令了
                agv_data_publish_->state_timer_callback();


                // 接下来控制小车去下一个点，然后上报完成取消任务
                instant_action_listener->update_interrupt_order_message_status("FINISHED");
                interrupt_order_message_.action_status = "FINISHED";

                // 抢占一下导航任务，让小车运动到当前目标点就停下来
                Point start_point   = {order_messages_.goal_x[point_index-1], order_messages_.goal_y[point_index-1], order_messages_.goal_theta[point_index-1]};
                Point end_point     = {order_messages_.goal_x[point_index]  , order_messages_.goal_y[point_index]  , order_messages_.goal_theta[point_index]  };
                std::vector<Point> goal_points = math_tool.generateTrajectory(start_point, end_point, order_messages_.goal_trajectory[point_index-1], order_messages_.goal_edge_orientation[point_index-1]);

                agv_driver_control->setPostion(goal_points);
                drive_state = agv_driver_control->control();

                while(agv_driver_control->get_flag_driving()){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "等待小车停稳至下一个点位，然后才完成取消任务...");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                }

                // 更新一下order_messages_.msg_state.action_states，在末尾增加有关取消任务的"任务完成"信号

                order_messages_.msg_state.action_states.back().action_status = interrupt_order_message_.action_status;

                // 同步msg_state到order_listener
                order_listener->set_msg_state(order_messages_.msg_state);

                // 通知调度系统，我已经完成取消任务指令了
                agv_data_publish_->state_timer_callback();

                // 已完成取消任务指令，接下来进入空闲状态，等待调度系统发来新的指令
                last_instant_action_order_id = instant_action_messages_.last_instant_action_order_id;
                
                if(execute_action_->get_loading() == true)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "任务已取消，但小车处于载货状态，需要人工处理！");
                    drive_state = false;
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "任务已取消，进行数据清零！");
                    drive_state = true;
                }

                order_listener->OrderFinished();
                if_reach_point = true;
                break;

            }

            // 检查该段任务是否存在动作，如果有动作则判断在该node_id上是否有动作，有的话执行动作
            if(order_messages_.action_size != 0){
                // ****************************************完整的执行运动流程**************************************                
                int actionID = execute_action_->have_action(order_messages_,point_index); // 获取将要执行的动作的序号

                while(execute_action_->get_flag_have_fork_action() && !execute_action_->agv_fork_control->get_flag_finish()
                || execute_action_->get_flag_have_battery_action() && !execute_action_->agv_battery_control->get_flag_finish())
                
                {
                    // 休眠时间100ms
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    // 以下是暂停业务************************************************************************************************************
                    handle_pause(false, true);
                    // 以下是暂停业务************************************************************************************************************

                }

                if(actionID > -1) // 表示有要做的动作
                {
                    order_messages_.msg_state.action_states[actionID].action_status = "FINISHED";
                    agv_data_publish_->state_timer_callback();
                }
                // ****************************************完整的执行运动流程**************************************

            }


            // 以下逻辑是对小车的货叉进行控制
            if (point_index > 0 && // 小车至少要到第二个点
                order_messages_.point_types[point_index] != PointType::HARDACTION && 
                order_messages_.point_types[point_index] != PointType::SOFTACTION && 
                execute_action_->agv_fork_control->get_fork_height() != order_messages_.goal_heights[point_index] &&
                order_messages_.goal_trajectory[point_index].degree != -1) // 如果轨迹为空，则不进行货叉控制
            {
                // 休眠时间1s
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                execute_action_->agv_fork_control->setForkParameters(1, order_messages_.goal_heights[point_index]);
                bool fork_state = execute_action_->agv_fork_control->control();

                if(!fork_state){
                    std::cout << "AGV货叉启动失败！" << std::endl;
                }

                while(!execute_action_->agv_fork_control->get_flag_finish()){

                    // 休眠时间100ms
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    // 以下是暂停业务************************************************************************************************************
                    handle_pause(false, true);
                    // 以上是暂停业务************************************************************************************************************
                    
                }

                    // current_fork_height 现在由 execute_action_->agv_fork_control 管理
            }

            // 索引更新
            if (point_index < static_cast<size_t>(order_messages_.node_size-1)) // 如果当前点不是最后一个点
                point_index++;
            else
                if_reach_point = true;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "point_index : %zu ", point_index);

        }
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "if finish this task? ");
        
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"last_instant_action_order_id: %s instant_action_messages_.last_instant_action_order_id: %s",last_instant_action_order_id.c_str(),instant_action_messages_.last_instant_action_order_id.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"instant_action_messages_.action_type: %s",instant_action_messages_.action_type.c_str());
        //  收到RCS任务完成的情况
        if(last_instant_action_order_id != instant_action_messages_.last_instant_action_order_id && instant_action_messages_.action_type == "order_finished"){
            
            last_instant_action_order_id = instant_action_messages_.last_instant_action_order_id;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "任务完成，进行数据清零！");
            order_listener->OrderFinished();

            if_reach_point = true;
            // 跳出循环，由于是RCS下发的任务结束，为正常的任务结束
            break;
        }
        // // 如果小车断开连接，则进入空闲态
        // if(instant_action_messages_.action_type == "no_message_received" || instant_action_messages_.action_type == "agv_offline"){
        //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "小车已离线！");
        //     order_listener->OrderFinished();

        //     // 先发布离线请求
        //     agv_data_publish_->p` ublish_connection_request("OFFLINE");
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1000*5));
        //     agv_data_publish_->cancel_connection_timer(); // 及时关闭下线定时器
        //     instant_action_listener->action_type = "no_message_received"; // 设置action_type为no_message_received
            
        //     break;
        // }

        // 休眠100ms 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    // 如果小车断开连接，则进入空闲态然后去上线
    if(instant_action_messages_.action_type == "no_message_received" || instant_action_messages_.action_type == "agv_offline"){
        this->config().blackboard->set("last_state", "running");
        this->config().blackboard->set("current_state", "idle");
        this->config().blackboard->set("AGV_Event", "init_success");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "小车离线，重新进入空闲态然后去上线！");
    }
    else if(drive_state){
        this->config().blackboard->set("last_state", "running");
        this->config().blackboard->set("current_state", "idle");
        this->config().blackboard->set("AGV_Event", "task_completed");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "驱动启动成功！");
    }
    else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "进入锁死态之前，进行任务数据清零！");
        order_listener->OrderFinished();
        this->config().blackboard->set("last_state", "running");
        this->config().blackboard->set("current_state", "lock");
        this->config().blackboard->set("AGV_Event", "emergency");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "驱动启动失败！");
    }

}


/*****************************************************************************************
* @brief:      数据更新函数，更新的数据包括：instantAction消息数据、order消息数据、当前位姿消息数据
* @brief:      由于data_updates函数在小车运行过程中被高度调用，并且占据主进程的大部分，所以自然地，可以在这里加入紧急介入控制算法
* @param:      无
* @return:     当无异常情况地正常更新数据之后，返回true，否则返回false
* @author:     刘鸿彬
* @date:       2024-11-22
* @version:    V0.0
******************************************************************************************/
bool LaserRunningStateBehaviors::data_updates(){

    // 检查can数据是否异常
    if(can_data_listener_->io_data_is_error || can_data_listener_->err_data_is_error || can_data_listener_->hardware_data_is_error){
        std::cout << "can数据异常，跳转至lock状态！" << std::endl;
        this->config().blackboard->set("fault_code", "CAN_DATA_ERROR");
        return false;
    }

    // 更新instantAction消息的数据
    instant_action_messages_ = instant_action_listener->get_instant_action_messages();
    
    // 只有在order更新的时候才调用，防止多次读取旧order，覆盖现有数据，导致重复执行动作
    // msg_state只有在新旧订单order_id不同时才会调用OrderFinished();清空，所以要记录执行到哪了，但是
    if(order_listener->messages_change){

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "是否更新数据 ： %d",order_listener->messages_change);
        
        std::cout << "DEBUG: 准备保存已完成动作状态" << std::endl;
        std::cout << "DEBUG: order_messages_.action_size = " << order_messages_.action_size << std::endl;
        std::cout << "DEBUG: order_messages_.msg_state.action_states.size() = " << order_messages_.msg_state.action_states.size() << std::endl;
        
        // 🔧 修复：保存当前已完成的动作状态，防止被新任务数据覆盖
        std::map<std::string, std::string> finished_action_states;
        for(int i = 0; i < order_messages_.action_size; i++) {
            std::cout << "DEBUG: 循环 i=" << i << " (保存阶段)" << std::endl;
            if(order_messages_.msg_state.action_states[i].action_status == "FINISHED") {
                finished_action_states[order_messages_.msg_state.action_states[i].action_id] = "FINISHED";
                std::cout << "🔄 保存已完成动作状态: " << order_messages_.msg_state.action_states[i].action_id << std::endl;
            }
        }
        
        std::cout << "DEBUG: 保存完成，准备更新order消息" << std::endl;
        
        // 更新order消息
        order_messages_ = order_listener->get_order_messages();
        
        std::cout << "DEBUG: order消息已更新" << std::endl;
        std::cout << "DEBUG: 更新后 order_messages_.action_size = " << order_messages_.action_size << std::endl;
        std::cout << "DEBUG: 更新后 order_messages_.msg_state.action_states.size() = " << order_messages_.msg_state.action_states.size() << std::endl;
        
        // 🔧 修复：恢复已完成的动作状态
        for(int i = 0; i < order_messages_.action_size; i++) {
            std::cout << "DEBUG: 循环 i=" << i << " (恢复阶段)" << std::endl;
            if(finished_action_states.find(order_messages_.msg_state.action_states[i].action_id) != finished_action_states.end()) {
                order_messages_.msg_state.action_states[i].action_status = "FINISHED";
                std::cout << "✅ 恢复已完成动作状态: " << order_messages_.msg_state.action_states[i].action_id << std::endl;
            }
        }
        
        // 同步msg_state到order_listener
        order_listener->set_msg_state(order_messages_.msg_state);
        
        std::cout << "DEBUG: 动作状态恢复完成" << std::endl;

        // 任务更新，重置索引
        // 接到全新order后更新，索引重置，回调函数中，first_order置为true，messages_change置为true，更新一次point_index = 0后，messages_change为false，直接不进入判定
        // 只有在第二次order的回调才会将first_order置为false，如果接不到第二次order，就会一直为true，一直将point_index=0从头开始执行，而有了messages_change为false，直接不进入，既可以防止rcs掉线（网络波动），agv一直执行第一步，也能够节省处理重复订单的开销。也可以防止每次行为树tick，都要调用多次该函数。
        if(order_listener->first_order)
        {
            point_index = 0;
            far_point_index = 0;
        }

        update_points_type();
        // 因为order_listener->messages_change要立刻被重置为false，所以需要一个额外的标志位标识订单被更新了，双标志位思想
        if_order_updated = true;
        order_listener->messages_change = false;
    }

    // 更新当前位姿数据
    current_pose_ = current_pose_listener->get_current_pose();

    // 更新当前目标位姿，全局变量更新，在多线程发布状态部分需要根据这个发布state消息
    order_messages_.current_goal_x = order_messages_.goal_x[point_index];    
    order_messages_.current_goal_y = order_messages_.goal_y[point_index];  
    order_messages_.current_goal_theta = order_messages_.goal_theta[point_index];

    // 获取最新数据
    if (velocity_listener) {
        velocity_messages_ = velocity_listener->get_velocity_messages();
    }
    if (obstacle_server) {
        obstacle_messages_ = obstacle_server->get_obstacle_messages();
    }

    // 以下是紧急介入控制部分算法
    
    // 3. 电量检查
    if (battery_listener) {
        battery_messages_ = battery_listener->get_battery_messages();
    }
    
    double current_velocity = std::sqrt(velocity_messages_.velocity_x * velocity_messages_.velocity_x + velocity_messages_.velocity_y * velocity_messages_.velocity_y);
    double max_linear_velocity = agv_config.max_linear_velocity;
    double max_angular_velocity = agv_config.max_angular_velocity;
    double min_battery_level = agv_config.min_battery_level;
    double max_offset = agv_config.max_offset;
    double min_distance = math_tool.minDistance(current_pose_, goal_points_to_driver);
    
    bool has_error = false;
    std::string error_info = "";
    std::string fault_code = "NONE";
    
    // 以下是紧急介入设计
    // 1. 速度检查
    if(current_velocity > max_linear_velocity) {
        has_error = true;
        fault_code = "VELOCITY_OVER_LIMIT";
        error_info += "❌ 速度超限: 当前线速度 " + std::to_string(current_velocity) + " > 最大线速度 " + std::to_string(max_linear_velocity) + "\n";
    }
    
    // 2. 角速度检查
    if(velocity_messages_.omega > max_angular_velocity) {
        has_error = true;
        if(fault_code == "NONE") fault_code = "ANGULAR_VELOCITY_OVER_LIMIT";
        error_info += "❌ 角速度超限: 当前角速度 " + std::to_string(velocity_messages_.omega) + " > 最大角速度 " + std::to_string(max_angular_velocity) + "\n";
    }
    
    // // 3. 电量检查
    // if(battery_messages_.battery_level < min_battery_level) {
    //     has_error = true;
    //     if(fault_code == "NONE") fault_code = "BATTERY_LOW";
    //     error_info += "❌ 电量过低: 当前电量 " + std::to_string(battery_messages_.battery_level) + " < 最低电量 " + std::to_string(min_battery_level) + "\n";
    // }
    
    // 4. 路径偏移检查
    if(min_distance > max_offset) {
        has_error = true;
        if(fault_code == "NONE") fault_code = "PATH_OFFSET_TOO_LARGE";
        error_info += "❌ 路径偏移过大: 当前偏移 " + std::to_string(min_distance) + " > 最大偏移 " + std::to_string(max_offset) + "\n";
    }
    
    // 5. 装载冲突检查
    if(point_index < order_messages_.point_types.size() && execute_action_->get_loading() == true && order_messages_.point_types[point_index] == PointType::READYLOAD) {
        has_error = true;
        if(fault_code == "NONE") fault_code = "LOADING_CONFLICT";
        error_info += "❌ 装载冲突: 小车已装载货物但当前点为装货点\n";
    }
    
    // // 6. 障碍物检查
    // if(obstacle_messages_ == 3 && velocity_messages_.acceleration > 0.01) {
    //     // has_error = true;
    //     // if(fault_code == "NONE") fault_code = "OBSTACLE_DANGER";
    //     // error_info += "❌ 障碍物危险: 障碍物等级 " + std::to_string(obstacle_messages_) + " == 3 且加速度 " + std::to_string(velocity_messages_.acceleration) + " >= 0 (未减速)\n";
        
    //     has_error = false;
    //     error_info += "❌ 障碍物危险: 障碍物等级 " + std::to_string(obstacle_messages_) + " == 3 且加速度 " + std::to_string(velocity_messages_.acceleration) + " >= 0 (未减速)\n";
    //     std::cout << error_info << std::endl;

    // }

    // 7.丢导航
    if(current_pose_listener->get_pose == false)
    {
        has_error = true;
        if(fault_code == "NONE") fault_code = "NAVIGATION_LOST";
        error_info += "❌ 丢导航: 当前位姿获取失败\n";
    }
    
    if(has_error)
    {
        // 设置故障代码到黑板
        this->config().blackboard->set("fault_code", fault_code);
        
        std::cout << "=== 紧急介入控制检测到异常 ===" << std::endl;
        std::cout << "故障代码: " << fault_code << std::endl;
        std::cout << error_info << std::endl;
        std::cout << "🛑 驱动启动失败，系统将进入锁定状态" << std::endl;
        std::cout << "=================================" << std::endl;

        if(agv_driver_control->get_flag_driving())
            agv_driver_control->cancel();

        order_listener->get_order = false; // 强制结束 running state，进入lock态

        return false;
    }

    return true;

}



void LaserRunningStateBehaviors::update_points_type()
{
    // 标记点状态
    order_messages_.point_types.clear();
    order_messages_.goal_heights.clear();

    for(int i=0;i<order_messages_.node_size;i++)
    {
        PointType point_type = PointType::ORDINARY; // 初始化该点为普通点;
        int goal_height = agv_config.fork_running_height; // 初始化该点的目标货叉高度为行进货叉高度;

        // 如果有些点没有给状态描述，那后续就没法处理了，会数组越界
        // 正常情况每个节点都要有node_states，防备数据不规范报错
        if(i >= order_messages_.msg_state.node_states.size()) {
            break;
        }
        
        auto node_id = order_messages_.msg_state.node_states[i].node_id;
        // action_vec是std::multimap类型，一个键对应多个值，用equal_range返回的first是键为node_id的第一个值，second是第一个键大于node_id的第一个值，
        auto range = order_messages_.action_vec.equal_range(node_id);
        // 两者相等，说明容器中不存在以当前 node_id 为键的任何元素，代表该点没有动作
        if(range.first == range.second) 
        {
            std::cout << " "<< std::endl;
            // // 判断是否还有下一个点，如果有，则判断是否是动作，如果是动作，则升级当前点为准备点，原逻辑在准备点抬升货叉，在取货点仅仅识别位置取货，改成挂钩后必须在点位上抬升
            // if(i < order_messages_.node_size-1)
            // {
            //     if((i+1) >= order_messages_.msg_state.node_states.size()) {
            //         break;
            //     }
            //     // 遍历下一个点的所有action，看是否有动作
            //     auto next_node_id = order_messages_.msg_state.node_states[i+1].node_id;
                
            //     auto range1 = order_messages_.action_vec.equal_range(next_node_id);

            //     // 遍历range1
            //     for (auto it = range1.first; it != range1.second; ++it){
            //         int ii = 0;
            //         // range1.second是action_id，而msg_state在存储所有的action时按照顺序存储，不是个字典，不能通过键来找到数据，所以我们要遍历来找到这个action_id对应的下标，从而获取数据判断他是不是动作点
            //         // 遍历动作容器，找到对应动作的下标，注意：不是动作id
            //         for(ii=0;ii<order_messages_.action_size;ii++){
            //             // 越界检测
            //             if(ii >= order_messages_.msg_state.action_states.size()) {
            //                 break;
            //             }
                        
            //             auto action_id = order_messages_.msg_state.action_states[ii].action_id;
            //             // 找到了下标，跳出循环
            //             if(action_id == it->second) {
            //                 break;
            //             }
            //         }
            //         // 再次越界检查，为什么要做这么多次？？
            //         if(ii >= order_messages_.action_size || ii >= order_messages_.msg_state.action_states.size()) {
            //             continue;
            //         }
                    
            //         // 找到对应的动作，并且该动作未完成，则执行动作
            //         auto action_type        = order_messages_.msg_state.action_states[ii].action_type;
            //         auto action_finished    = order_messages_.msg_state.action_states[ii].action_status;

            //         std::cout << next_node_id << "点 有动作：" << action_type << " 状态：" << action_finished << std::endl;;
            //         // 下一个点有动作，将当前点置为准备点，在准备点
            //         if(action_type == "LOAD" && action_finished  != "FINISHED")
            //         {
            //             // 条件中的action_type是下标为ii的下一个节点的，这里的point_type和goal_height是当前节点的，是在循环外定义的
            //             point_type = PointType::READYLOAD;
            //             goal_height = std::stoi(order_messages_.msg_state.action_states[ii].action_description);
            //         }
            //         else if(action_type == "UNLOAD" && action_finished != "FINISHED")
            //         {
            //             point_type = PointType::READYUNLOAD;
            //             goal_height = std::stoi(order_messages_.msg_state.action_states[ii].action_description) + fork_action_height;
            //         }
            //     }
            // }
        }
        else // 该点有动作，即为动作点
        {
            // 遍历该点的所有动作，检查是否存在HARD类型的动作
            bool has_hard_action = false;
            
            // 遍历该点的所有动作
            for (auto it = range.first; it != range.second; ++it)
            {
                // 找到对应的动作
                for(int ii = 0; ii < order_messages_.action_size; ii++)
                {
                    if(ii >= order_messages_.msg_state.action_states.size()) {
                        break;
                    }
                    
                    auto action_id = order_messages_.msg_state.action_states[ii].action_id;
                    
                    if(action_id == it->second)
                    {
                        // 找到对应动作，检查其blocking_type
                        auto blocking_type = order_messages_.msg_state.action_states[ii].blocking_type;
                        
                        if(blocking_type == "HARD")
                        {
                            has_hard_action = true;
                            break;
                        }
                    }
                }
                
                if(has_hard_action)
                {
                    break;
                }
            }
            
            // 根据是否有HARD动作来设置点类型
            if(has_hard_action)
            {
                point_type = PointType::HARDACTION;
            }
            else
            {
                point_type = PointType::SOFTACTION;
            }
        }

        // 检测尖点：如果当前点还没有被标记为特殊类型（ORDINARY除外），且是中间点，则检测是否为尖点 如果这两条边一条边是正走、另外一条边是倒走：则视为尖点
        // std::cout<< i << std::endl;
        // std::cout<< order_messages_.node_size - 1 << std::endl;
        // std::cout<< order_messages_.goal_edge_orientation[i-1] << std::endl;
        // std::cout<< order_messages_.goal_edge_orientation[i] << std::endl;

        if(point_type == PointType::ORDINARY && i > 0 && i < order_messages_.node_size - 1 && abs(order_messages_.goal_edge_orientation[i-1] - order_messages_.goal_edge_orientation[i]) > 3)
        {
            point_type = PointType::SHARP;
        }

        order_messages_.point_types.push_back(point_type);
        order_messages_.goal_heights.push_back(goal_height);

        if(point_type == PointType::ORDINARY)
            std::cout<<"ORDINARY    ";
        else if(point_type == PointType::SHARP)
            std::cout<<"SHARP    ";
        else if(point_type == PointType::READYLOAD)
            std::cout<<"READYLOAD    ";
        else if(point_type == PointType::READYUNLOAD)
            std::cout<<"READYUNLOAD    ";
        else if(point_type == PointType::HARDACTION)
            std::cout<<"HARDACTION    ";
        else if(point_type == PointType::SOFTACTION)
            std::cout<<"SOFTACTION    ";
        else 
            std::cout<<"UNKNOWN    ";

    }
    std::cout<<std::endl;

}


void LaserRunningStateBehaviors::handle_pause(bool navigation, bool fork)
{
    interrupt_order_message_ = instant_action_listener->get_interrupt_order_message();
    // 只有action_type为打断类型，并且任务状态为刚刚开始打断WAITING才会执行
    if(interrupt_order_message_.action_type == "startPause" && interrupt_order_message_.action_status == "WAITING")
    {  
        flag_pausing = true;

        bool need_pause_fork = false;

        std::cout << "正在处理暂停业务" << std::endl;
        std::cout << "导航动作：" << navigation << std::endl;
        std::cout << "货叉动作：" << fork << std::endl;
        std::cout << "execute_action_->agv_fork_control->get_flag_driving()：" << execute_action_->agv_fork_control->get_flag_driving() << std::endl;
        
        // 设置paused
        data_updates();
        order_messages_.msg_state.paused = true;
        // 同步msg_state到order_listener
        order_listener->set_msg_state(order_messages_.msg_state);
        // 通知调度系统，当前正在暂停
        agv_data_publish_->state_timer_callback();


        // 根据参数取消相应的任务
        if(navigation)
        {
            agv_driver_control->cancel(); // 暂停了，小车导航任务先取消！
        }
        
        if(fork && execute_action_->agv_fork_control->get_flag_driving())
        {
            need_pause_fork = true;
            execute_action_->agv_fork_control->cancel(); // 暂停了，小车货叉动作先取消！
        }
        
        instant_action_listener->update_interrupt_order_message_status("FINISHED");
        interrupt_order_message_.action_status = "FINISHED";

        while(flag_pausing)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            interrupt_order_message_ = instant_action_listener->get_interrupt_order_message();
            if(interrupt_order_message_.action_type == "stopPause" && interrupt_order_message_.action_status == "WAITING")
            {
                flag_pausing = false;
                
                // 根据参数恢复相应的任务
                if(navigation)
                {
                    restored_drive_state_ = agv_driver_control->control();
                    std::cout << "取消暂停。已恢复导航任务！" << std::endl;
                }
                
                if(need_pause_fork)
                {
                    execute_action_->agv_fork_control->control();
                    std::cout << "取消暂停。已恢复货叉动作！" << std::endl;
                }
                
                instant_action_listener->update_interrupt_order_message_status("FINISHED");
                interrupt_order_message_.action_status = "FINISHED";

                // 设置paused
                data_updates();
                order_messages_.msg_state.paused = false;
                // 同步msg_state到order_listener
                order_listener->set_msg_state(order_messages_.msg_state);
                // 通知调度系统，当前不在暂停
                agv_data_publish_->state_timer_callback();

                break;
            }
        }
    }
}

// ------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------


QRRunningStateBehaviors::QRRunningStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone) : 
    SyncActionNode(name, config),
    instant_action_listener(agv_bone.agv_instant_action_listener),
    current_pose_listener(agv_bone.agv_current_pose_listener),
    order_listener(agv_bone.agv_order_listener),
    battery_listener(agv_bone.agv_battery_listener),
    velocity_listener(agv_bone.agv_velocity_listener),
    obstacle_server(agv_bone.agv_obstacle_server),
    agv_data_publish_(agv_bone.agv_data_publish),
    can_data_listener_(agv_bone.agv_can_data_listener)
    {
    // 初始化驱动控制单例
    QRDriverControl::initialize(agv_bone);
    // 获取驱动控制单例
    agv_driver_control = QRDriverControl::getInstance();
    // 设置驱动控制函数，用于同步driving状态
    agv_data_publish_->set_driving_fn([this]() { return agv_driver_control->get_flag_driving(); });

    // 实例化末端动作控制（包含货叉控制、充电控制）
    execute_action_ = std::make_shared<QRExecuteAction>(agv_bone);

    // 任务节点索引
    point_index = 0;
    current_qr = -1;
    far_point_index = 0;

    obstacle_wait_time = 0;
    current_obstacle_avoidance_channels_ = -1;
    // 初始化 是否到达目标点标签设置为true
    if_reach_point = true;
    if_order_updated = true;
    if_to_leave = false;

    goal_points_to_driver = {};

    math_tool = Math_Tool();
}


NodeStatus QRRunningStateBehaviors::tick()
{

    // 预备工作
    // 更新instantAction消息的数据
    instant_action_messages_ = instant_action_listener->get_instant_action_messages();
    // 更新order消息
    order_messages_ = order_listener->get_order_messages();
    // 更新当前位姿数据
    current_pose_ = current_pose_listener->get_current_pose();

    // data_updates();

    // 任务节点索引
    point_index = 0;
    current_qr = -1;
    far_point_index = 0;

    obstacle_wait_time = 0;
    current_obstacle_avoidance_channels_ = -1;
    // 初始化 是否到达目标点标签设置为true
    if_reach_point = true;
    if_order_updated = true;

    goal_points_to_driver = {};

    // 主逻辑
    std::cout << this->name() << " 正在执行:OnReceiveOrder() " << std::endl;
    agv_driver_control->set_flag_finish(true); // 先默认小车完成了上一次的导航任务
    OnReceiveOrder();

    // 收尾工作
    // 导航完成，导航任务标志重置
    agv_driver_control->set_flag_finish(false);
    return NodeStatus::SUCCESS;
}


void QRRunningStateBehaviors::OnReceiveOrder(){

    bool drive_state = true;

    far_point_index = 0;
    
    // // 1、instant_action发布的任务
    // if(instant_action_messages_.action_type == "connect_success_auto"){

    //     goal_points_to_driver = {};
    //     Point one_point = {instant_action_messages_.goal_x, instant_action_messages_.goal_y, instant_action_messages_.goal_theta};
    //     goal_points_to_driver.push_back(one_point);

    //     agv_driver_control->setPostion(goal_points_to_driver);
    //     drive_state = agv_driver_control->control();

    //     // 判断是否完成本次导航，初始值未false
    //     while (!agv_driver_control->get_flag_finish()){

    //         // 数据更新函数，更新的数据包括：instantAction消息数据、order消息数据、当前位姿消息数据
    //         data_updates();

    //         // 休眠100ms
    //         std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     }
    //     // 完成instantAction的任务，将任务标识置false
    //     agv_driver_control->set_flag_finish(false);
    //     // 任务完成，跳转至空闲状态
    //     this->config().blackboard->set("last_state", "running");
    //     this->config().blackboard->set("current_state", "idle");
    //     this->config().blackboard->set("AGV_Event", "task_completed");
    // }

    // 2、order发布的任务，需要完成的是一系列任务
    while(order_listener->get_order){

        // 数据更新函数，更新的数据包括：instantAction消息数据、order消息数据、当前位姿消息数据
        data_updates();
        if(!drive_state) // 如果发现异常，就需要紧急控制
            break;
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "目标坐标(%.2f,%.2f,%.2f),当前坐标(%.2f,%.2f,%.2f)",order_messages_.goal_x[point_index],order_messages_.goal_y[point_index],order_messages_.goal_theta[point_index],current_pose_.current_x,current_pose_.current_y,current_pose_.current_theta);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "允许偏差(%.2f,%.2f,%.2f)",order_messages_.goal_allowed_deviation_xy[point_index],order_messages_.goal_allowed_deviation_xy[point_index],order_messages_.goal_allowed_deviation_theta[point_index]);        
        // 完成分段式导航任务

        // 可能要更改避障通道
        if(point_index > 0 && current_obstacle_avoidance_channels_ != order_messages_.edge_obstacle_avoidance_channels[point_index-1])
        {
            // 此处需要向底层发送激光区域通道选择。TODO: 实现该功能
            current_obstacle_avoidance_channels_ = order_messages_.edge_obstacle_avoidance_channels[point_index-1];
            obstacle_server->publish_obstacle_channels(current_obstacle_avoidance_channels_, current_obstacle_avoidance_channels_, current_obstacle_avoidance_channels_, current_obstacle_avoidance_channels_);
        }

        // 如果目标点和当前位姿存在误差，则启动驱动，否则不启动驱动
        // if(abs(order_messages_.goal_x[point_index] - current_pose_.current_x)> high_distance_precision || abs(order_messages_.goal_y[point_index] - current_pose_.current_y)>high_distance_precision || abs(order_messages_.goal_theta[point_index] - current_pose_.current_theta)>high_angle_precision){
        if((!if_reach_point)
        || (abs(order_messages_.goal_x[point_index] - current_pose_.current_x) > order_messages_.goal_allowed_deviation_xy[point_index] || abs(order_messages_.goal_y[point_index] - current_pose_.current_y) > order_messages_.goal_allowed_deviation_xy[point_index] || !if_angle_qualify(order_messages_.goal_theta[point_index], current_pose_.current_theta, order_messages_.goal_allowed_deviation_theta[point_index]))
        ){
            if(point_index > far_point_index || if_order_updated == true) // 如果发现当前目标点是全新的（未处理过的）|| RCS发来的order更新了，则需要进行多点导航
            {
                std::vector<agv_interfaces::msg::Poses> goal_poses_to_driver;
                // 二维码项目特性：导航任务先加一个当前点
                agv_interfaces::msg::Poses current_pose;
                current_pose.x = current_pose_.current_x;
                current_pose.y = current_pose_.current_y;
                current_pose.label = current_qr;
                current_pose.angle = current_pose_.current_theta;
                current_pose.allowed_deviation_angle = order_messages_.goal_allowed_deviation_theta[point_index-1];
                current_pose.obstacle_channel_select = order_messages_.edge_obstacle_avoidance_channels[point_index-1];
                goal_poses_to_driver.push_back(current_pose);
                size_t i;
                // 直接读取所有目标点，组合成位姿点
                for (i = point_index; i < order_messages_.goal_x.size() && order_messages_.msg_state.node_states[i].released; ++i){

                    agv_interfaces::msg::Poses goal_pose;
                    goal_pose.x = order_messages_.goal_x[i];
                    goal_pose.y = order_messages_.goal_y[i];
                    goal_pose.label = extract_number_from_node_id(order_messages_.goal_node_id[i], -1);
                    goal_pose.angle = order_messages_.goal_theta[i];
                    goal_pose.allowed_deviation_angle = order_messages_.goal_allowed_deviation_theta[i];

                    if(i < order_messages_.edge_obstacle_avoidance_channels.size())
                    {
                        goal_pose.obstacle_channel_select = order_messages_.edge_obstacle_avoidance_channels[i];
                    }
                    else
                    {
                        goal_pose.obstacle_channel_select = order_messages_.edge_obstacle_avoidance_channels[order_messages_.edge_obstacle_avoidance_channels.size()-1];
                    }

                    goal_poses_to_driver.push_back(goal_pose);

                    if(order_messages_.point_types[i] != PointType::ORDINARY)
                    {
                        i = i + 1;
                        break;
                    }
                }
                // 根据驱动器的反馈数据，判断是否到达目标点，到达的话进行数据更新并跳出
                // 发送目标位姿至驱动模块

                far_point_index = i-1; // 当发送完一次任务后，far_point_index一般指向首个特殊点 或者 任务的终点

                agv_driver_control->setPostion(goal_poses_to_driver);
                drive_state = agv_driver_control->control();

                if_order_updated = false;

            }

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "即将驱动到目标位置（%.2f,%.2f,%.2f）",order_messages_.goal_x[point_index],order_messages_.goal_y[point_index],order_messages_.goal_theta[point_index]);

            // 其实小车在运行过程中，此循环占据了主进程的大部分，等待到达目标点
            while(abs(order_messages_.goal_x[point_index] - current_pose_.current_x) > order_messages_.goal_allowed_deviation_xy[point_index] || abs(order_messages_.goal_y[point_index] - current_pose_.current_y) > order_messages_.goal_allowed_deviation_xy[point_index] || !if_angle_qualify(order_messages_.goal_theta[point_index], current_pose_.current_theta, order_messages_.goal_allowed_deviation_theta[point_index]))
            {
                int times = 0;
                // 数据更新函数，更新的数据包括：instantAction消息数据、order消息数据、当前位姿消息数据
                if(!data_updates() || times > 16*60) // 如果在更新数据时发现异常，就需要紧急控制
                {
                    if(agv_driver_control->get_flag_driving())
                        agv_driver_control->cancel();

                    order_listener->get_order = false; // 强制结束 running state，进入lock态
                    drive_state = false;

                    std::cout << "=== 未能在60秒内到达下一个目标点！ ===" << std::endl;
                    break;
                }

                // 休眠60ms
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
                times++;
            }
            if_reach_point = true;
            if(point_index > far_point_index) // 当到达多点导航的最终目标的下一个点时，重置far_point_index
            {
                far_point_index = 0;
            }
        
        
        }
        // 到达了目标点，误差也在允许范围内
        // else if(abs(order_messages_.goal_x[point_index] - current_pose_.current_x)<= high_distance_precision && abs(order_messages_.goal_y[point_index] - current_pose_.current_y)<=high_distance_precision && abs(order_messages_.goal_theta[point_index] - current_pose_.current_theta)<=high_angle_precision){
        else if(
           (if_reach_point)
        || (abs(order_messages_.goal_x[point_index] - current_pose_.current_x) <= order_messages_.goal_allowed_deviation_xy[point_index] && abs(order_messages_.goal_y[point_index] - current_pose_.current_y) <= order_messages_.goal_allowed_deviation_xy[point_index] && if_angle_qualify(order_messages_.goal_theta[point_index], current_pose_.current_theta, order_messages_.goal_allowed_deviation_theta[point_index]))
        ){
            if_reach_point = false;

            current_qr = extract_number_from_node_id(order_messages_.goal_node_id[point_index], -1);
            
            std::cout<<"point_index:"<<point_index<<"  "<<order_messages_.msg_state.node_states.size()<<std::endl;
            // 到达目标点附近，更新last_node_id 、last_node_sequence_id 
            order_messages_.msg_state.last_node_id = order_messages_.msg_state.node_states[point_index].node_id;
            order_messages_.msg_state.last_node_sequence_id = order_messages_.msg_state.node_states[point_index].sequence_id;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "last_node_id ： %s ", order_messages_.msg_state.node_states[point_index].node_id.c_str());
            
            // 同步msg_state到order_listener
            order_listener->set_msg_state(order_messages_.msg_state);
            
            data_updates();

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"order_messages_.msg_state.order_update_id: %d",order_messages_.msg_state.order_update_id);

            agv_data_publish_->state_timer_callback();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "运行状态更新state!  目标坐标(%.2f,%.2f,%.2f),当前坐标(%.2f,%.2f,%.2f)",order_messages_.goal_x[point_index],order_messages_.goal_y[point_index],order_messages_.goal_theta[point_index],current_pose_.current_x,current_pose_.current_y,current_pose_.current_theta);
            // 上报完状态之后，需要等待200ms，给rcs足够时间发来反馈或者新任务
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 检查该段任务是否存在动作，如果有动作则判断在该node_id上是否有动作，有的话执行动作
            if(order_messages_.action_size != 0){
                // 判断在该node_id上是否有动作，有的话执行动作,定时器内部更新动作状态;但是需要看blocking_type，当blocking_type为HARD时，需要等待导航完成才能执行动作
                if(order_messages_.point_types[point_index] == PointType::HARDACTION){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "即将执行HARD动作，需要等待导航完成...");
                    while(agv_driver_control->get_flag_driving()){
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "等待导航完成...");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                }

                // ****************************************完整的执行运动流程**************************************
                int actionID = execute_action_->have_action(order_messages_,point_index);

                while(actionID > -1 && !execute_action_->agv_fork_control->get_flag_finish()){

                    // 休眠时间100ms
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                if(actionID > -1)
                {
                    agv_data_publish_->state_timer_callback();
                }
                // ****************************************完整的执行运动流程**************************************
            }

            // 调度系统发来取消任务的即时指令
            interrupt_order_message_ = instant_action_listener->get_interrupt_order_message();
            if(interrupt_order_message_.action_type == "cancelOrder" && (interrupt_order_message_.action_status == "WAITING" || interrupt_order_message_.action_status == "RUNNING")) // 表明现在要取消任务
            {

                if(interrupt_order_message_.action_status == "WAITING")
                {
                    ActionStates one_action_state;
                    one_action_state.action_id          = interrupt_order_message_.action_id;
                    one_action_state.action_type        = interrupt_order_message_.action_type;
                    one_action_state.action_status      = interrupt_order_message_.action_status;
                    one_action_state.action_description = interrupt_order_message_.action_description;
                    order_messages_.msg_state.action_states.push_back(one_action_state);
                }

                if(point_index < static_cast<size_t>(order_messages_.node_size-1)) // 如果当前点不是最后一个点
                {
                    instant_action_listener->update_interrupt_order_message_status("RUNNING");
                    interrupt_order_message_.action_status = "RUNNING";
                }
                else
                {
                    instant_action_listener->update_interrupt_order_message_status("FINISHED");
                    interrupt_order_message_.action_status = "FINISHED";
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "任务已取消，进行数据清零！");
                }

                order_messages_.msg_state.action_states.back().action_status = interrupt_order_message_.action_status;

                // 同步msg_state到order_listener
                order_listener->set_msg_state(order_messages_.msg_state);

                // 通知调度系统，我已经完成取消任务指令了
                agv_data_publish_->state_timer_callback();

                // 已完成取消任务指令，接下来进入空闲状态，等待调度系统发来新的指令
                last_instant_action_order_id = instant_action_messages_.last_instant_action_order_id;

                if(point_index == static_cast<size_t>(order_messages_.node_size-1)) // 如果走完当前任务，则退出
                {
                    order_listener->OrderFinished();
                    if_reach_point = true;
                    break;
                }
            }

            // 以下逻辑是对小车的货叉进行控制
            if (point_index > 0 && // 小车至少要到第二个点
                order_messages_.point_types[point_index] != PointType::HARDACTION && 
                order_messages_.point_types[point_index] != PointType::SOFTACTION && 
                execute_action_->agv_fork_control->get_fork_height() != order_messages_.goal_heights[point_index])
            {
                if(order_messages_.point_types[point_index] == PointType::READYUNLOAD || order_messages_.point_types[point_index] == PointType::READYLOAD)
                {
                    while(!agv_driver_control->get_flag_finish())
                    {
                        std::cout << "等待小车到达准备点，之后再进行托盘调整。" << std::endl;
                        // 休眠时间100ms
                        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    }
                }

                execute_action_->agv_fork_control->setForkParameters(1, order_messages_.goal_heights[point_index], 0);
                bool fork_state = execute_action_->agv_fork_control->control();

                if(!fork_state){
                    std::cout << "AGV货叉启动失败！" << std::endl;
                }

                while(!execute_action_->agv_fork_control->get_flag_finish()){

                    // 休眠时间100ms
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                // current_fork_height 现在由 execute_action_->agv_fork_control 管理
            }

            // 索引更新
            if (point_index < static_cast<size_t>(order_messages_.node_size-1)) // 如果当前点不是最后一个点
                point_index++;
            else
                if_reach_point = true;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "point_index : %zu ", point_index);

        }
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "if finish this task? ");
        
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"last_instant_action_order_id: %s instant_action_messages_.last_instant_action_order_id: %s",last_instant_action_order_id.c_str(),instant_action_messages_.last_instant_action_order_id.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"instant_action_messages_.action_type: %s",instant_action_messages_.action_type.c_str());
        //  收到RCS任务完成的情况
        if(last_instant_action_order_id != instant_action_messages_.last_instant_action_order_id && instant_action_messages_.action_type == "order_finished"){
            
            last_instant_action_order_id = instant_action_messages_.last_instant_action_order_id;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "任务完成，进行数据清零！");
            order_listener->OrderFinished();

            if_reach_point = true;
            // 跳出循环，由于是RCS下发的任务结束，为正常的任务结束
            break;
        }
        // 如果小车断开连接，则进入空闲态
        if(instant_action_messages_.action_type == "no_message_received" || instant_action_messages_.action_type == "agv_offline"){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "小车已离线！");
            order_listener->OrderFinished();

            // 先发布离线请求
            agv_data_publish_->publish_connection_request("OFFLINE");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000*5));
            agv_data_publish_->cancel_connection_timer(); // 及时关闭下线定时器
            instant_action_listener->action_type = "no_message_received"; // 设置action_type为no_message_received

            break;
        }

        // 休眠100ms 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 如果小车断开连接，则进入空闲态然后去上线
    if(instant_action_messages_.action_type == "no_message_received" || instant_action_messages_.action_type == "agv_offline"){
        this->config().blackboard->set("last_state", "running");
        this->config().blackboard->set("current_state", "idle");
        this->config().blackboard->set("AGV_Event", "init_success");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "小车离线，重新进入空闲态然后去上线！");
    }
    else if(drive_state){
        this->config().blackboard->set("last_state", "running");
        this->config().blackboard->set("current_state", "idle");
        this->config().blackboard->set("AGV_Event", "task_completed");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "驱动启动成功！");
    }
    else{
        this->config().blackboard->set("last_state", "running");
        this->config().blackboard->set("current_state", "lock");
        this->config().blackboard->set("AGV_Event", "emergency");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "驱动启动失败！");
    }

}


/*****************************************************************************************
* @brief:      数据更新函数，更新的数据包括：instantAction消息数据、order消息数据、当前位姿消息数据
* @brief:      由于data_updates函数在小车运行过程中被高度调用，并且占据主进程的大部分，所以自然地，可以在这里加入紧急介入控制算法
* @param:      无
* @return:     当无异常情况地正常更新数据之后，返回true，否则返回false
* @author:     刘鸿彬
* @date:       2024-11-22
* @version:    V0.0
******************************************************************************************/
bool QRRunningStateBehaviors::data_updates(){

    // 更新instantAction消息的数据
    instant_action_messages_ = instant_action_listener->get_instant_action_messages();

    // 更新order消息,只有在数据更新的时候才读取，防止多次读取旧数据，覆盖现有数据，导致重复执行动作
    // 主要是每次行为树tick，都要调用多次该函数，order_listener->first_order只有在第二次order的回调才会将first_order置为false，如果接不到第二次order，就会一直为true，一直将point_index=0从头开始执行，而有了messages_change，第一次接到order就会置为false，直接不进入，既可以防止rcs掉线，agv一直执行第一步，也能够节省处理重复订单的开销。
    if(order_listener->messages_change){

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "是否更新数据 ： %d",order_listener->messages_change);
        
        std::cout << "DEBUG: 准备保存已完成动作状态" << std::endl;
        std::cout << "DEBUG: order_messages_.action_size = " << order_messages_.action_size << std::endl;
        std::cout << "DEBUG: order_messages_.msg_state.action_states.size() = " << order_messages_.msg_state.action_states.size() << std::endl;
        
        // 🔧 修复：保存当前已完成的动作状态，防止被新任务数据覆盖
        std::map<std::string, std::string> finished_action_states;
        for(int i = 0; i < order_messages_.action_size; i++) {
            std::cout << "DEBUG: 循环 i=" << i << " (保存阶段)" << std::endl;
            if(order_messages_.msg_state.action_states[i].action_status == "FINISHED") {
                finished_action_states[order_messages_.msg_state.action_states[i].action_id] = "FINISHED";
                std::cout << "🔄 保存已完成动作状态: " << order_messages_.msg_state.action_states[i].action_id << std::endl;
            }
        }
        
        std::cout << "DEBUG: 保存完成，准备更新order消息" << std::endl;
        
        // 更新order消息
        order_messages_ = order_listener->get_order_messages();
        
        std::cout << "DEBUG: order消息已更新" << std::endl;
        std::cout << "DEBUG: 更新后 order_messages_.action_size = " << order_messages_.action_size << std::endl;
        std::cout << "DEBUG: 更新后 order_messages_.msg_state.action_states.size() = " << order_messages_.msg_state.action_states.size() << std::endl;
        
        // 🔧 修复：恢复已完成的动作状态
        for(int i = 0; i < order_messages_.action_size; i++) {
            std::cout << "DEBUG: 循环 i=" << i << " (恢复阶段)" << std::endl;
            if(finished_action_states.find(order_messages_.msg_state.action_states[i].action_id) != finished_action_states.end()) {
                order_messages_.msg_state.action_states[i].action_status = "FINISHED";
                std::cout << "✅ 恢复已完成动作状态: " << order_messages_.msg_state.action_states[i].action_id << std::endl;
            }
        }
        
        // 同步msg_state到order_listener
        order_listener->set_msg_state(order_messages_.msg_state);
        
        std::cout << "DEBUG: 动作状态恢复完成" << std::endl;

        // 任务更新，重置索引
        if(order_listener->first_order)
        {
            point_index = 0;
            far_point_index = 0;
        }

        update_points_type();
        if_order_updated = true;
        order_listener->messages_change = false;
    }

    // 更新当前位姿数据
    current_pose_ = current_pose_listener->get_current_pose();

    // 更新当前目标位姿，全局变量更新，在多线程发布状态部分需要根据这个发布state消息
    order_messages_.current_goal_x = order_messages_.goal_x[point_index];    
    order_messages_.current_goal_y = order_messages_.goal_y[point_index];  
    order_messages_.current_goal_theta = order_messages_.goal_theta[point_index];

    // 以下是紧急介入控制部分算法
    
    // double current_velocity = std::sqrt(velocity_messages_.velocity_x * velocity_messages_.velocity_x + velocity_messages_.velocity_y * velocity_messages_.velocity_y);
    // double max_linear_velocity = agv_config.max_linear_velocity;
    // double max_angular_velocity = agv_config.max_angular_velocity;
    // double min_battery_level = agv_config.min_battery_level;
    // double max_offset = agv_config.max_offset;
    // double min_distance = math_tool.minDistance(current_pose_, goal_points_to_driver);
    
    
    bool has_error = false;
    std::string error_info = "";
    std::string fault_code = "NONE";
    
    // 以下是紧急介入设计

    // 7.丢导航
    if(current_pose_listener->get_pose == false)
    {
        has_error = true;
        if(fault_code == "NONE") fault_code = "NAVIGATION_LOST";
        error_info += "❌ 丢导航: 当前位姿获取失败\n";
    }
    
    if(has_error)
    {
        // 设置故障代码到黑板
        this->config().blackboard->set("fault_code", fault_code);
        
        std::cout << "=== 紧急介入控制检测到异常 ===" << std::endl;
        std::cout << "故障代码: " << fault_code << std::endl;
        std::cout << error_info << std::endl;
        std::cout << "🛑 驱动启动失败，系统将进入锁定状态" << std::endl;
        std::cout << "=================================" << std::endl;

        if(agv_driver_control->get_flag_driving())
            agv_driver_control->cancel();

        order_listener->get_order = false; // 强制结束 running state，进入lock态

        return false;
    }

    return true;

}



void QRRunningStateBehaviors::update_points_type()
{
    // 标记点状态
    order_messages_.point_types.clear();
    order_messages_.goal_heights.clear();

    for(int i=0;i<order_messages_.node_size;i++)
    {
        PointType point_type = PointType::ORDINARY; // 初始化该点为普通点;
        int goal_height = agv_config.fork_running_height; // 初始化该点的目标货叉高度为行进货叉高度;

        if(i >= order_messages_.msg_state.node_states.size()) {
            break;
        }
        
        auto node_id = order_messages_.msg_state.node_states[i].node_id;

        auto range = order_messages_.action_vec.equal_range(node_id);

        if(range.first == range.second) // 如果该点没有动作 + 该点的下一个点是取卸货动作点；那么该点是准备点
        {
            if(i < order_messages_.node_size-1)
            {
                if((i+1) >= order_messages_.msg_state.node_states.size()) {
                    break;
                }
                
                auto next_node_id = order_messages_.msg_state.node_states[i+1].node_id;
                
                auto range1 = order_messages_.action_vec.equal_range(next_node_id);

                // 遍历range1
                for (auto it = range1.first; it != range1.second; ++it){
                    int ii = 0;
                    // 遍历动作容器，找到对应动作的下标，注意：不是动作id
                    for(ii=0;ii<order_messages_.action_size;ii++){
                        if(ii >= order_messages_.msg_state.action_states.size()) {
                            break;
                        }
                        
                        auto action_id = order_messages_.msg_state.action_states[ii].action_id;
                        
                        if(action_id == it->second) {
                            break;
                        }
                    }
                    
                    if(ii >= order_messages_.action_size || ii >= order_messages_.msg_state.action_states.size()) {
                        continue;
                    }
                    
                    // 找到对应的动作，并且该动作未完成，则执行动作
                    auto action_type        = order_messages_.msg_state.action_states[ii].action_type;
                    auto action_finished    = order_messages_.msg_state.action_states[ii].action_status;

                    std::cout << next_node_id << "点 有动作：" << action_type << " 状态：" << action_finished << std::endl;;
                    
                    if(action_type == "pick" && action_finished  != "FINISHED")
                    {
                        point_type = PointType::READYLOAD;
                        goal_height = std::stoi(order_messages_.msg_state.action_states[ii].action_description);
                    }
                    else if(action_type == "drop" && action_finished != "FINISHED")
                    {
                        point_type = PointType::READYUNLOAD;
                        goal_height = std::stoi(order_messages_.msg_state.action_states[ii].action_description) + fork_action_height;
                    }
                }
            }
        }
        else // 该点有动作，即为动作点
        {
            // 遍历该点的所有动作，检查是否存在HARD类型的动作
            bool has_hard_action = false;
            
            // 遍历该点的所有动作
            for (auto it = range.first; it != range.second; ++it)
            {
                // 找到对应的动作
                for(int ii = 0; ii < order_messages_.action_size; ii++)
                {
                    if(ii >= order_messages_.msg_state.action_states.size()) {
                        break;
                    }
                    
                    auto action_id = order_messages_.msg_state.action_states[ii].action_id;
                    
                    if(action_id == it->second)
                    {
                        // 找到对应动作，检查其blocking_type
                        auto blocking_type = order_messages_.msg_state.action_states[ii].blocking_type;
                        
                        if(blocking_type == "HARD")
                        {
                            has_hard_action = true;
                            break;
                        }
                    }
                }
                
                if(has_hard_action)
                {
                    break;
                }
            }
            
            // 根据是否有HARD动作来设置点类型
            if(has_hard_action)
            {
                point_type = PointType::HARDACTION;
            }
            else
            {
                point_type = PointType::SOFTACTION;
            }
        }

        order_messages_.point_types.push_back(point_type);
        order_messages_.goal_heights.push_back(goal_height);

        if(point_type == PointType::ORDINARY)
            std::cout<<"ORDINARY    ";
        else if(point_type == PointType::READYLOAD)
            std::cout<<"READYLOAD    ";
        else if(point_type == PointType::READYUNLOAD)
            std::cout<<"READYUNLOAD    ";
        else if(point_type == PointType::HARDACTION)
            std::cout<<"HARDACTION    ";
        else if(point_type == PointType::SOFTACTION)
            std::cout<<"SOFTACTION    ";
        else 
            std::cout<<"UNKNOWN    ";
    }
    std::cout<<std::endl;

}
