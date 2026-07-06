/************************************** File Info ****************************************
* @file:       client_request_charging.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-11-18                                         
* @version:    V0.0                                                                              
* @brief:      AGV电池充电客户端，发送充电请求，服务端处理后，返回电池状态(充电/放电)
******************************************************************************************/
# include "client_request_charging.h"

/*****************************************************************************************
* @brief:      服务通信客户端，用于发送控制指令到服务端，控制AGV电池的充放电
* @author:     刘鸿彬
* @date:       2024-11-19
* @version:    V0.0
******************************************************************************************/

// 构造函数，创建客户端
RequestChargingClient::RequestChargingClient(std::shared_ptr<rclcpp::Node> node) : 
    node_(node)
{
    
    RCLCPP_INFO(node_->get_logger(),"create a battery control client!");

    // 根据vehicle_type设置话题名称
    std::string client_name;
    if (agv_config.vehicle_type == "laser") {
        // 激光导航：固定话题名称
        client_name =  "battery_control";
    } else {
        // 二维码导航：使用序列号作为前缀
        std::string SerialNumber = agv_config.serial_number;
        client_name = SerialNumber + "/battery_control";
    }

    // 创建客户端
    client_ = node_->create_client<BatteryControl>(client_name);
    
}

// // 连接服务端，如果连接成功返回true否则返回false
// bool RequestChargingClient::connect_server(){

//     // 等待5s
//     while (!client_->wait_for_service(5s)){

//         // ctrl+c的特殊处理
//         if (!rclcpp::ok()){
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"force client termination!");
//             return false;
//         }
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"connecting with server!");
//     }
//     return true;
// }
bool RequestChargingClient::connect_server(){
    return false;
}
/*****************************************************************************************
* @brief:      客户端发送数据到服务端
* @param:      charging ：发送的数据（0：关闭充电，1：打开充电，2：查看充电状态）
* @return:     battery_status ：电池状态（0：放电 1：充电 -1：初始值/未读取到数据 -2：输入的参数非法）
* @return:     charging_status : 充电状态（0：未充电 1：正在充电 2：充电 3：充电终止 -1：初始值/未读取到数据 -2：输入的参数非法）
* @author:     刘鸿彬
* @date:       2024-11-19
* @version:    V0.0
******************************************************************************************/
// 发送数据
rclcpp::Client<BatteryControl>::FutureAndRequestId RequestChargingClient::send_request(int charging){
    
    // 数据声明
    auto request = std::make_shared<BatteryControl::Request>();
    // 数据填充
    request->charging = charging;
    // 数据发送
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"充电请求数据成功发送！%d", charging);
    return client_->async_send_request(request);
    
}

// int main(int argc,char const *argv[]){
//     //argc have three parameter,first is the address of files,others are submiting by user
//     if (argc !=2){

//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"please submit to two integer number!");
//         return 1;
//     }
//     // 初始化
//     rclcpp::init(argc,argv);
//     // 创建客户端对象
//     auto client = std::make_shared<RequestChargingClient>();
//     // 调用连接服务端的函数
//     bool flag = client->connect_server();

//     // 根据连接结果进行下一步
//     if (!flag){

//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"fail to connect server, program exit");
//         return 0;
//     }

//     // 调用发送数据的函数
//     auto future = client->send_request(atoi(argv[1]));

//     // 处理服务器的回复
//     if (rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS){

//         auto result = future.get();
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"接收到数据！");
//         RCLCPP_INFO(client->get_logger(),"success to response! battery_status = %d  ",result->battery_status);
//         RCLCPP_INFO(client->get_logger(),"success to response! charging_status = %d ",result->charging_status);
//     }
    
//     else{

//         RCLCPP_INFO(client->get_logger(),"fail to response!");
//     }
    
//     // 5.release resources
//     rclcpp::shutdown();
//     return 0;
// }

