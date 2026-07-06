/************************************** File Info ****************************************
* @file:       subscriber_order.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-12                                         
* @version:    V0.0
* @class:      订阅者                                                                          
* @brief:      订阅RCS任务消息，并将消息转输出出去
******************************************************************************************/
# include "subscriber_order.h"

// 节点构造函数
OrderListener::OrderListener(std::shared_ptr<rclcpp::Node> node):node_(node){

    RCLCPP_INFO(node_->get_logger(),"created a order subscriber");

    // 创建订阅方
    std::string SerialNumber = agv_config.serial_number;
    subscription_ = node_->create_subscription<AGVOrder>("uagv/v1/BYD/" + SerialNumber + "/order",10,std::bind(&OrderListener::order_callback,this,std::placeholders::_1));

    // 初始化，防止未收到命令时，声明容器大小部分报错
    get_order = false;
    messages_change = false;
    first_order = true;
    released_node_size = 0;
    released_edge_size = 0;
    released_action_size = 0;
    m_vector_size = 0;

    last_order_update_id = -1;

}

// 返回解析order得到的所需数据
OrderMessages OrderListener::get_order_messages(){

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 数据填充，order_messages结构体一次性获取所有的变量的值，保存为数据快照，防止数据不一致的问题
    order_messages.goal_node_id = released_goal_node_id;
    order_messages.goal_x       = released_goal_x;
    order_messages.goal_y       = released_goal_y;
    order_messages.goal_theta   = released_goal_theta;
    order_messages.goal_allowed_deviation_xy = released_goal_allowed_deviation_xy;
    order_messages.goal_allowed_deviation_theta = released_goal_allowed_deviation_theta;

    order_messages.goal_edge_orientation = released_edge_orientation;
    order_messages.edge_obstacle_avoidance_channels = released_edge_obstacle_avoidance_channels;

    // 容器大小
    order_messages.action_size = released_action_size;
    order_messages.edge_size = released_edge_size;
    order_messages.node_size = released_node_size;

    order_messages.goal_trajectory = released_goal_trajectory;

    // 关联容器存储node_id和action_id之间的对用关系
    order_messages.action_vec = m_action_vec;

    // 需要解析的其他数据
    order_messages.msg_state = msg_state;

    return order_messages;
}

// 设置msg_state（用于同步外部修改）
void OrderListener::set_msg_state(const AGVState& new_msg_state){
    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 更新内部的msg_state
    msg_state = new_msg_state;
}

// 从MQTT消息同步数据（用于纯MQTT模式）
void OrderListener::update_from_mqtt(const AGVOrder &mqtt_order_messages){
    
    RCLCPP_INFO(node_->get_logger(), "正在同步MQTT消息到OrderListener...");

    try {
        order_callback(mqtt_order_messages);
        std::cout << "DEBUG: order_callback() 调用成功" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "DEBUG: order_callback() 抛出异常: " << e.what() << std::endl;
        RCLCPP_ERROR(node_->get_logger(), "order_callback异常: %s", e.what());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "✓ MQTT消息已同步到OrderListener成功");
}


void OrderListener::OrderFinished(){

    // 更新任务状态
    get_order = false;

    // 任务数据清零，防止状态报告用到过时数据
    released_node_size = 0;
    msg_state.node_states = {};

    released_edge_size = 0;
    msg_state.edge_states = {};

    released_action_size = 0;
    msg_state.action_states = {};


    released_goal_node_id   = {};
    released_goal_x         = {};
    released_goal_y         = {};
    released_goal_theta     = {};
    released_goal_allowed_deviation_xy = {};
    released_goal_allowed_deviation_theta = {};
    released_edge_orientation={};
    released_edge_obstacle_avoidance_channels={};
    released_goal_trajectory= {};

    msg_state.order_id      = "";
    msg_state.last_node_id  = "";
    msg_state.last_node_sequence_id = -1; 

    // 定时发布的数据是根据该全局变量的
    // order_messages.msg_state.order_update_id = 0;

    last_order_update_id = 0;
    first_order = true;
    
    // 🔧 修复：清理动作关联容器，避免历史动作数据污染新任务
    m_action_vec.clear();
    m_vector_size = 0;
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "任务数据清理完成，包括动作关联容器");
}

// 订阅方的回调函数，解析获取到的数据
void OrderListener::OrderListener::order_callback(const AGVOrder &msg){

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"执行order话题回调函数！");

    // 如果是收到的不是第一条数据，则进行两个判断
    // 1、该数据是否为旧数据，是的话则不进行数据解析
    // 2、该数据是否为重复数据，是的话则不进行数据解析
    if(get_order){
        // 需要一个if(get_order)是因为此时记录的order_id和rcs发过来的order_id（通常不会是默认值0）不相同，会一直错误地触发变量初始化，导致msg_state.order_id一直被初始化为0，但是这个变量的标志和first_order有什么不同的用处
        if(msg_state.order_id != msg.order_id){
            // order_id不同说明已经是全新订单，清空所有变量
            OrderFinished();
        }
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"msg_state.order_id: %s msg.order_id: %s",msg_state.order_id.c_str(),msg.order_id.c_str());
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"msg_state.order_update_id: %d msg.order_update_id: %d",msg_state.order_update_id,msg.order_update_id);
        // 旧消息不处理
        if (msg_state.order_id == msg.order_id && msg_state.order_update_id >= msg.order_update_id) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"订阅到的消息为旧消息！");
            return;
        }
        // 旧消息处理-重复的子任务消息
        if (msg_state.order_id == msg.order_id && last_order_update_id == msg.order_update_id){

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"进行重复的任务消息过滤！");
            return;
        }

    }

    int new_node_action_size = 0;
    int new_edge_action_size = 0;
    
    // 否则更新总任务

    // 尝试获取互斥锁,离开作用域时，会自动调用unlock()释放互斥锁
    std::lock_guard<std::mutex> lock(data_mutex_);


/*-------------------------------------------------------------------------------------------------------------------------------*/
    // 先考虑点的事务
    int node_start, node_end, i; // 计算得到改组任务的released为true并且不重复的的点的起始点和末位点
    // 一次3个任务点，第一次123，第二次345，这样最少一次性有两个要去的目标点45，不会使车辆快到4就开始减速
    if(released_goal_x.empty())
    {
        first_order = true;
        node_start = 0;
    }
    else
    {
        first_order = false;
        // 因为第0个点，已经在上一轮执行过了
        node_start = 1;
    }
    // node_start表示该order里的第一个非重复的released为true的点

    for(i = 0; i < static_cast<int>(msg.nodes.size()); i++)
    {
        if(msg.nodes[i].released == false)
            break;
    }
    node_end = i-1; // node_end表示改order里的最后一个released为true的点的序号

    std::cout << "点情况：" << "released_node_size:" << released_node_size << "node_end:" << node_end << "node_start:" << node_start << std::endl;

    released_node_size = released_node_size + node_end - node_start + 1;

    // 填充节点数据
    for(i=node_start ; i <= node_end; i++){

        std::cout << "=== DEBUG: 处理节点 " << i << " ===" << std::endl;
        std::cout << "Node ID: " << msg.nodes[i].node_id << std::endl;
        std::cout << "Position: (" << msg.nodes[i].node_position.x 
                  << ", " << msg.nodes[i].node_position.y 
                  << ", " << msg.nodes[i].node_position.theta << ")" << std::endl;
        std::cout << "Released: " << msg.nodes[i].released << std::endl;
        std::cout << "Sequence ID: " << msg.nodes[i].sequence_id << std::endl;
        std::cout << "Actions count: " << msg.nodes[i].actions.size() << std::endl;

        NodeStates one_node_state;
        one_node_state.node_id = msg.nodes[i].node_id;
        one_node_state.released = msg.nodes[i].released;
        one_node_state.sequence_id = msg.nodes[i].sequence_id;
        msg_state.node_states.push_back(one_node_state);

        // 拿到全部的目标点信息
        released_goal_node_id.push_back(msg.nodes[i].node_id);
        released_goal_x.push_back(msg.nodes[i].node_position.x);
        released_goal_y.push_back(msg.nodes[i].node_position.y);
        released_goal_theta.push_back(msg.nodes[i].node_position.theta);
        released_goal_allowed_deviation_xy.push_back(msg.nodes[i].node_position.allowed_deviation_xy);
        released_goal_allowed_deviation_theta.push_back(msg.nodes[i].node_position.allowed_deviation_theta);
        new_node_action_size = new_node_action_size + msg.nodes[i].actions.size();
        
        std::cout << "=== END DEBUG: 节点 " << i << " ===" << std::endl;
    }

    // 获取released为true的节点总数量，如果是子任务，则需要从第二个点开始加入原先的released为true的点集合当中
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"更新之后released为true的节点数量：%d",released_node_size);
/*-------------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------------*/
    // 接下来考虑边的事务
    int edge_start, edge_end;
    edge_start = 0; // 边都是新的边，非重复的边

    std::cout << static_cast<int>(msg.edges.size()) << "条边！" << std::endl;

    for(i = 0; i < static_cast<int>(msg.edges.size()); i++)
    {
        if(msg.edges[i].released == false)
            break;
    }
    edge_end = i-1; // edge_end表示改order里的最后一个released为true的点的序号

    std::cout << "边情况：" << "released_edge_size:" << released_edge_size << "edge_end:" << edge_end << "edge_start:" << edge_start << std::endl;

    released_edge_size = released_edge_size + edge_end - edge_start + 1;

    // 填充边数据
    for(i=edge_start ; i <= edge_end; i++){

        std::cout<<i<<std::endl;
        
        std::cout << "=== DEBUG: 处理边 " << i << " ===" << std::endl;
        std::cout << "Edge ID: " << msg.edges[i].edge_id << std::endl;
        std::cout << "Start Node: " << msg.edges[i].start_node_id << std::endl;
        std::cout << "End Node: " << msg.edges[i].end_node_id << std::endl;
        std::cout << "Released: " << msg.edges[i].released << std::endl;
        std::cout << "Sequence ID: " << msg.edges[i].sequence_id << std::endl;
        std::cout << "Max Speed: " << msg.edges[i].max_speed << std::endl;

        EdgeStates one_edge_state;
        one_edge_state.edge_id = msg.edges[i].edge_id;
        one_edge_state.released = msg.edges[i].released;
        one_edge_state.sequence_id = msg.edges[i].sequence_id;
        msg_state.edge_states.push_back(one_edge_state);

        // 如果这条边有方向，则将对应方向push进released_edge_orientation，否则将0.0 push进
        if (msg.edges[i].orientation_type == "TANGENTIAL" && msg.edges[i].rotation_allowed == true)
            released_edge_orientation.push_back(msg.edges[i].orientation);
        else
            released_edge_orientation.push_back(0.0);

        // 填充边的避障通道
        released_edge_obstacle_avoidance_channels.push_back(msg.edges[i].obstacle_avoidance_channel);

        Trajectory trajectory;

        trajectory.degree = msg.edges[i].trajectory.degree;
        std::cout << "Trajectory degree: " << trajectory.degree << std::endl;
        std::cout << "Knot vector size: " << msg.edges[i].trajectory.knot_vector.size() << std::endl;
        std::cout << "Control points size: " << msg.edges[i].trajectory.control_points.size() << std::endl;
        
        for(int j = 0; j < static_cast<int>(msg.edges[i].trajectory.knot_vector.size()); j++)
        {
            trajectory.knots.push_back(msg.edges[i].trajectory.knot_vector[j]);
            std::cout << "Knot[" << j << "]: " << msg.edges[i].trajectory.knot_vector[j] << std::endl;
        }
        for(int k = 0; k < static_cast<int>(msg.edges[i].trajectory.control_points.size()); k++)
        {
            ControlPoint one_control_point;
            one_control_point.x     = msg.edges[i].trajectory.control_points[k].x;
            one_control_point.y     = msg.edges[i].trajectory.control_points[k].y;
            one_control_point.weight = msg.edges[i].trajectory.control_points[k].weight;
            trajectory.control_points.push_back(one_control_point);
            std::cout << "Control Point[" << k << "]: (" 
                      << one_control_point.x << ", " 
                      << one_control_point.y << ", " 
                      << one_control_point.weight << ")" << std::endl;
        }

        released_goal_trajectory.push_back(trajectory);
        new_edge_action_size = new_edge_action_size + msg.edges[i].actions.size();
        
        std::cout << "=== END DEBUG: 边 " << i << " ===" << std::endl;
    }


    // 获取released为true的边总数量
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"更新之后released为true的边数量：%d",released_edge_size);
/*-------------------------------------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------------------*/
    // 接下来考虑动作的事务

    released_action_size = released_action_size + new_node_action_size + new_edge_action_size;

    for(i=node_start ; i <= node_end; i++){
        for(size_t j=0;j<msg.nodes[i].actions.size();j++)
        {   
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"第 %d 个节点有 %ld 个动作",i,msg.nodes[i].actions.size());

            ActionStates one_action_state;
            one_action_state.action_id = msg.nodes[i].actions[j].action_id;
            one_action_state.action_type = msg.nodes[i].actions[j].action_type;
            one_action_state.action_status = "INITIALIZING";
            // action_parameters不一定有内容，需要判断一下
            if(msg.nodes[i].actions[j].action_parameters.size() > 0){
                one_action_state.action_description = std::to_string(msg.nodes[i].actions[j].action_parameters[0].value.number_value);
            }
            else{
                one_action_state.action_description = "";
            }

            one_action_state.blocking_type = msg.nodes[i].actions[j].blocking_type;
            msg_state.action_states.push_back(one_action_state);


            m_action_vec.insert({msg.nodes[i].node_id, msg.nodes[i].actions[j].action_id});

            // m_action_vec.insert[msg.nodes[i].node_id] = msg.nodes[i].actions[j].action_id;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"在node节点%s中填充了动作，其类型为%s，索引号为 %d",msg.nodes[i].node_id.c_str(), msg.nodes[i].actions[j].action_type.c_str(), m_vector_size);
            
            m_vector_size++;
        }
    }

    for(i=edge_start ; i <= edge_end; i++){
        for(size_t j=0;j<msg.edges[i].actions.size();j++)
        {                                                       
            ActionStates one_action_state;
            one_action_state.action_id = msg.edges[i].actions[j].action_id;
            one_action_state.action_type = msg.edges[i].actions[j].action_type;
            one_action_state.action_status = "INITIALIZING";
            one_action_state.action_description = std::to_string(msg.edges[i].actions[j].action_parameters[0].value.number_value);
            one_action_state.blocking_type = msg.edges[i].actions[j].blocking_type;
            msg_state.action_states.push_back(one_action_state);

            m_action_vec.insert({msg.edges[i].edge_id, msg.edges[i].actions[j].action_id});
            // m_action_vec[msg.edges[i].edge_id] = msg.edges[i].actions[j].action_id;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"在edge节点中填充了action_id，索引号为 %d",m_vector_size);
            m_vector_size++;
        }
    }

    // 获取released为true的边总数量
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"更新之后released为true的动作数量：%d",released_action_size);
/*-------------------------------------------------------------------------------------------------------------------------------*/

    // 容器大小临时值清零
    m_vector_size = 0;

    // 更新一下state的order_id数据
    msg_state.order_id = msg.order_id;

    last_order_update_id = msg.order_update_id;

    // 更新last_node和last_node_sequence
    // 最后一个点位判断
    // msg_state.last_node_id = msg.nodes[0].node_id;
    // msg_state.last_node_sequence_id = msg.nodes[0].sequence_id;
    msg_state.order_update_id = msg.order_update_id;
    msg_state.order_id = msg.order_id;

    // 更新任务接收
    get_order = true;
    messages_change = true;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"是否接到order命令标志位: %d", get_order);

    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"msg_state.order_update_id: %d msg.order_update_id: %d",msg_state.order_update_id,msg.order_update_id);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"接收到任务，任务id: %s",msg.order_id.c_str());

}

