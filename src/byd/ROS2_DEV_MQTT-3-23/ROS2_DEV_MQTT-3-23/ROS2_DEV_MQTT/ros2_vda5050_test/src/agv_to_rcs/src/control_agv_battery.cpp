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

    bool flag = request_charging_client_->connect_server();

    if(flag == false)
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Connection to charging_client failed!");

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
    // 先等10秒，这是因为导航模块尽管已经完成导航，但仍会在几秒内发送0x404去设置驱动使能，这会导致充电使能被关闭，所以需要等它操作完再轮到我们
    std::this_thread::sleep_for(std::chrono::milliseconds(1000*10));
    agv_data_publish_->state_timer_callback();
    // 上报状态,等待调度系统发来instant_action = "start_charge"，随后才开始进行充电逻辑，在此之前，先老老实实等着
    int wait_time = 0;
    while(true){
        // 更新instantAction消息的数据
        instant_action_messages = instant_action_listener->get_instant_action_messages();
        std::cout<<instant_action_messages.action_type<<std::endl;
        if(instant_action_messages.action_type == "start_charge"){
            break;
        }
        else if(wait_time > 100*60)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "waiting for instant action (start_charge) for too long!");
        }
        // 休眠1000ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_time = wait_time + 100;
    }

    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Instant action (start_charge) received in %d ms", wait_time);

    // 1、首先判断是否连接上服务端
    bool flag = request_charging_client_->connect_server();

    // 2、根据连接结果进行下一步
    if(!flag){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "fail to connect server!");
        flag_finish_ = true;  // 连接失败也标记为完成（失败）
        return false;
    }

    connect_times = 0; // 连接次数

    // 3、调用发送数据的函数(最多尝试10次)
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "发送充电请求!");
    auto future_and_requestid = request_charging_client_->send_request(start_charge);
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "发送充电请求完成!");
    auto result = future_and_requestid.future.get();

    battery_messages_ = battery_listener_->get_battery_messages();
    while(battery_messages_.battery_status != 1)
    {
        connect_times++;
        if(connect_times>10)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "10 seconds fail to start charge!");
            flag_finish_ = true;  // 多次尝试失败也标记为完成（失败）
            return false;
        }
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "再次发送充电请求完成!");
        auto future_and_requestid = request_charging_client_->send_request(start_charge);
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "再次发送充电请求完成!");
        auto result = future_and_requestid.future.get();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        battery_messages_ = battery_listener_->get_battery_messages();
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
            if (battery_listener_) {
                battery_messages_ = battery_listener_->get_battery_messages();
            }
            std::cout << "充电状态中获取的电池电量 ："<< battery_messages_.battery_level << "       电池状态：" << battery_messages_.battery_status<<std::endl;

            if(battery_messages_.battery_level == 100 && battery_messages_.battery_status == 0){

                // 充电完成，自动断开充电
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Battery fully charged, automatically disconnected!");
                flag_finish_ = true;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000*30)); // 等待30秒，等待epec就绪
                return true;
            }

            // 2、RCS命令断开
            //  读取instantAction消息，判断是否需要断开充电
            if(instant_action_messages.action_type == "stop_charge"){
                // 调用发送数据的函数，充电模式转换成放电模式
                auto cancle_future_and_requestid = request_charging_client_->send_request(stop_charge);
                auto result = cancle_future_and_requestid.future.get();
                battery_messages_ = battery_listener_->get_battery_messages();
                while(battery_messages_.battery_status != 0)
                {
                    connect_times++;
                    if(connect_times>30)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "30 seconds fail to stop charge!");
                        flag_finish_ = true;  // 多次尝试失败也标记为完成（失败）
                        return false;
                    }
                    auto cancle_future_and_requestid = request_charging_client_->send_request(stop_charge);
                    auto result = cancle_future_and_requestid.future.get();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
    

