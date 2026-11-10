/************************************** File Info ****************************************
* @file:       control_agv_battery.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-12                                         
* @version:    V0.0                                                                              
* @brief:      AGV电池控制类
******************************************************************************************/
# include "control_agv_battery.h"


/*****************************************************************************************
* @brief:      类的构造函数，用于实例化电池控制客户端以及初始化部分参数
* @param:      &battery_messages : 电池数据结构体地址
* @author:     刘鸿彬
* @date:       2024-11-12
* @version:    V0.0
******************************************************************************************/
AGVBatteryControl::AGVBatteryControl(AGVBone agv_bone):
    battery_listener_(agv_bone.agv_battery_listener),
    instant_action_listener(agv_bone.agv_instant_action_listener),
    agv_data_publish_(agv_bone.agv_data_publish),
    node_(agv_bone.agv_all_nodes),
    flag_finish_(false)
    {
    // 实例化充电相关请求（包含充电和放电）的客户端
    request_charging_client_ = std::make_shared<RequestChargingClient>(agv_bone.agv_all_nodes);

    bool flag = request_charging_client_->connect_server(1, 1s);

    if(flag == false)
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Initial connection to charging_client not ready (will connect during control)");

    // 初始化，客户端发送的控制数据，增加可读性
    start_charge = 1;
    stop_charge = 0;
}

bool AGVBatteryControl::get_flag_finish(){
    return flag_finish_;
}

void AGVBatteryControl::set_flag_finish(bool flag){
    flag_finish_ = flag;
}
/*****************************************************************************************
* @brief:      该函数用于进行充电任务,启动充电任务之后等待RCS发布取消充电的命令，或者充满之后自动跳转
* @param:      无
* @return:     充满或RCS下发任务断开返回true，其他情况断开返回false
* @author:     刘鸿彬
* @date:       2024-11-12
* @version:    V0.0
******************************************************************************************/
bool AGVBatteryControl::control(){
    flag_finish_ = false;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "已经进入充电模块，等待10秒以确保底层导航驱动使能完全释放...");
    // 先等10秒，这是因为导航模块尽管已经完成导航，但仍会在几秒内发送0x404去设置驱动使能，这会导致充电使能被关闭，所以需要等它操作完再轮到我们
    std::this_thread::sleep_for(std::chrono::milliseconds(1000*10));
    agv_data_publish_->state_timer_callback();
    // 上报状态,等待调度系统发来instant_action = "start_charge"，随后才开始进行充电逻辑，在此之前，先老老实实等着
    int wait_time = 0;
    while(true){
        // 更新instantAction消息的数据
        instant_action_messages = instant_action_listener->get_instant_action_messages();
        
        // 检查是否收到 start_charge (包括锁存的单次消息以及当前消息类型)
        if(instant_action_listener->has_start_charge_latched() || instant_action_messages.action_type == "start_charge"){
            instant_action_listener->clear_start_charge_latched();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "成功捕获并消费 start_charge 动作指令!");
            break;
        }

        if(instant_action_messages.action_type == "cancelOrder"){
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "等待充电期间收到 cancelOrder 指令，退出充电模块!");
            flag_finish_ = true;
            return false;
        }

        if(wait_time > 0 && wait_time % 5000 == 0)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "正在等待调度系统下发 start_charge 即时指令 (已等待 %d 秒)...", wait_time / 1000);
        }
        // 休眠100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_time = wait_time + 100;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Instant action (start_charge) received in %d ms", wait_time);

    // 1、首先判断是否连接上服务端 (每次等待2秒，最多重试10次，10次后还不行才认为服务端有问题)
    bool flag = request_charging_client_->connect_server(10, 2s);

    // 2、根据连接结果进行下一步
    if(!flag){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "尝试10次(共20秒)连接充电服务端失败，判定服务端不可用!");
        flag_finish_ = true;  // 连接失败也标记为完成（失败）
        return false;
    }

    // 3、发送充电请求并等待底盘切换至充电状态 (每次等待2秒，最多尝试10次，10次后还不行才认为服务端有问题)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "首次发送充电请求(start_charge=1)...");
    auto future_and_requestid = request_charging_client_->send_request(start_charge);
    if (future_and_requestid.future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
        auto result = future_and_requestid.future.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "收到服务端响应: battery_status = %d, charging_status = %d", 
                    result->battery_status, result->charging_status);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "首次充电请求等待服务端响应超时(2s)");
    }

    // 等待底盘响应并切换到充电状态: 每次等待2秒，10次后还不行才认为服务端有问题
    connect_times = 0;
    const int max_charge_retries = 10;
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        battery_messages_ = battery_listener_->get_battery_messages();
        if(battery_messages_.battery_status == 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "底盘已成功进入充电状态 (battery_status=1)!");
            break;
        }

        connect_times++;
        if(connect_times >= max_charge_retries)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                         "尝试10次(每次等待2秒，共20秒)后底盘仍未进入充电状态，判定服务端/底盘硬件存在问题!");
            flag_finish_ = true;
            return false;
        }

        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
                    "底盘尚未进入充电模式 (当前 battery_status=%d)，第 %d/10 次重试发送充电请求...", 
                    static_cast<int>(battery_messages_.battery_status), connect_times);
        auto retry_future = request_charging_client_->send_request(start_charge);
        if (retry_future.future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            auto res = retry_future.future.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "重试响应: battery_status = %d, charging_status = %d",
                        res->battery_status, res->charging_status);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "重试请求等待服务端响应超时(2s)");
        }
    }


    // 如果状态转化为充电状态，那么接下来有3种情况：1、充电完成自动断开 2、RCS命令断开 3、外部中断（待完善）
    if(battery_messages_.battery_status == 1){
        
        // 已切换至充电模式，通知调度系统
        agv_data_publish_->state_timer_callback();
        
        // 跳出条件，后续需要补充
        while(true){
            // 更新instantAction消息的数据
            instant_action_messages = instant_action_listener->get_instant_action_messages();
            
            // 1、充电完成自动断开，检查state中的充电状态是否改变
            // 通过读取state中的充放电状态判断是否自动断开充电，两个条件(1、充电模式改变 2、电池电量到达阈值)
            if(battery_listener_) {
                battery_messages_ = battery_listener_->get_battery_messages();
            }
            std::cout << "充电状态中获取的电池电量 ："<< battery_messages_.battery_level << "       电池状态：" << battery_messages_.battery_status<<std::endl;

            // 检查是否有 cancelOrder
            if(instant_action_messages.action_type == "cancelOrder"){
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "充电期间收到 cancelOrder 指令，停止充电并退出...");
                auto cancle_future_and_requestid = request_charging_client_->send_request(stop_charge);
                cancle_future_and_requestid.future.wait_for(std::chrono::seconds(2));
                flag_finish_ = true;
                return false;
            }

            if(battery_messages_.battery_level == 100 && battery_messages_.battery_status == 0){

                // 充电完成，自动断开充电
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Battery fully charged, automatically disconnected!");
                flag_finish_ = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000*30)); // 等待30秒，等待epec就绪
                return true;
            }

            // 2、RCS命令断开
            // 读取instantAction消息或检查锁存，判断是否需要断开充电
            if(instant_action_listener->has_stop_charge_latched() || instant_action_messages.action_type == "stop_charge"){
                instant_action_listener->clear_stop_charge_latched();
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "收到停止充电指令，向服务端发送 stop_charge 请求...");
                // 调用发送数据的函数，充电模式转换成放电模式
                auto cancle_future_and_requestid = request_charging_client_->send_request(stop_charge);
                if (cancle_future_and_requestid.future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                    cancle_future_and_requestid.future.get();
                }
                battery_messages_ = battery_listener_->get_battery_messages();
                int stop_times = 0;
                while(battery_messages_.battery_status != 0)
                {
                    stop_times++;
                    if(stop_times > 10)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "尝试10次未能停止充电!");
                        flag_finish_ = true;  // 多次尝试失败也标记为完成（失败）
                        return false;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    auto cancle_retry = request_charging_client_->send_request(stop_charge);
                    cancle_retry.future.wait_for(std::chrono::seconds(2));
                    battery_messages_ = battery_listener_->get_battery_messages();
                }


                // 服务端回复的数据
                if(battery_messages_.battery_status == 0){
                    
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"RCS request to stop charging , automatically disconnected!");
                    flag_finish_ = true;
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000*30)); // 等待30秒，等待epec就绪
                    return true;
                }
                // 返回的是其他数据，那么都是异常情况
                else{
                    RCLCPP_ERROR(node_->get_logger(),"fail to stop charging!");
                    flag_finish_ = true;  // 异常情况也标记为完成（失败）
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000*30)); // 等待30秒，等待epec就绪
                    return false;
                }

            }
            // 休眠500ms
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

    }
    // 其他未跳转至充电模式的情况，发送跳转命令未跳转，那么就是异常情况
    else{
        std::cout << "服务端回复的数据异常" << std::endl;

        RCLCPP_ERROR(node_->get_logger(),"the data replied by the server is abnormal!");
        flag_finish_ = true;  // 异常情况也标记为完成（失败）
        return false;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"完成充电任务！");


    return true;

}
    

