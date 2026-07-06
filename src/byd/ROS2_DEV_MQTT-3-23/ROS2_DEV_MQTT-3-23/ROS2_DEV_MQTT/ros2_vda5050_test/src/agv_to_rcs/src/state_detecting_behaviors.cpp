#include "state_detecting_behaviors.h"

/*****************************************************************************************
* @brief:      货位检测状态类的构造函数
* @param:      stateMachine ： 状态机类
* @author:     刘鸿彬
* @date:       2024-09-27
* @version:    V0.0
******************************************************************************************/
DetectingStateBehaviors::DetectingStateBehaviors(const std::string& name, const NodeConfig& config, AGVBone agv_bone) : 
  SyncActionNode(name, config),
  instant_action_listener(agv_bone.agv_instant_action_listener),
  current_pose_listener(agv_bone.agv_current_pose_listener),
  order_listener(agv_bone.agv_order_listener)
  {

  rack_detection_client = std::make_shared<RackDetectionClient>();

}

RackDetectionMessages DetectingStateBehaviors::get_rack_detection_messages() const {
    std::lock_guard<std::mutex> lock(rack_detection_mutex_);
    return rack_detection_messages_;
}


NodeStatus DetectingStateBehaviors::tick()
{
    // 预备工作
    // 更新instantAction消息的数据
    instant_action_messages_ = instant_action_listener->get_instant_action_messages();
    // 更新order消息
    order_messages_ = order_listener->get_order_messages();
    // 更新当前位姿数据
    current_pose_ = current_pose_listener->get_current_pose();

    // 主逻辑
    std::cout << this->name() << " 正在执行:OnDetecting() " << std::endl;
    OnDetecting();

    // 收尾工作
    return NodeStatus::SUCCESS;
}


/*****************************************************************************************
* @brief:      用于处理货位检测任务
* @param:      无
* @return:     返回当前状态处理之后的事件
* @author:     刘鸿彬
* @date:       2024-09-27
* @version:    V0.0
* @note:       1、从其他状态到货位检测状态 2、从货位检测状态到其他状态
* @note:       上个状态为空闲状态，因此进入该函数的条件是从其他状态到货位检测状态
******************************************************************************************/
void DetectingStateBehaviors::OnDetecting(){

  // 1、首先判断是否连接上服务端
  bool flag = rack_detection_client->connect_server();

  // 2、根据连接结果进行下一步
  if(!flag){
    RCLCPP_ERROR(rack_detection_client->get_logger(), "fail to connect server!");

    // 跳转至异常状态
    this->config().blackboard->set("last_state", "detecting");
    this->config().blackboard->set("current_state", "lock");
    this->config().blackboard->set("AGV_Event", "state_error");
  }

  connect_times = 0;// 当连接次数
  // 3、调用发送数据的函数
  auto future1 = rack_detection_client->send_request(1);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  auto future = rack_detection_client->send_request(1);
  while(!(rclcpp::spin_until_future_complete(rack_detection_client, future) == rclcpp::FutureReturnCode::SUCCESS))
  {
    connect_times++;
    if(connect_times>10)
    {
      RCLCPP_INFO(rack_detection_client->get_logger(), "10s!fail to response!");
      this->config().blackboard->set("last_state", "detecting");
      this->config().blackboard->set("current_state", "lock");
      this->config().blackboard->set("AGV_Event", "state_error");
    }
    auto future = rack_detection_client->send_request(1);
  }

  // 4、处理服务器的回复
  if (rclcpp::spin_until_future_complete(rack_detection_client, future) == rclcpp::FutureReturnCode::SUCCESS) {
    
    auto result = future.get();
    
    {
        std::lock_guard<std::mutex> lock(rack_detection_mutex_);
        rack_detection_messages_.racks.resize(result->racks_size);
        for(int i=0;i<result->racks_size;i++){
            std::cout << result->racks[i].x << std::endl;
            rack_detection_messages_.racks[i].x = result->racks[i].x;
            rack_detection_messages_.racks[i].y = result->racks[i].y;
            rack_detection_messages_.racks[i].z = result->racks[i].z;
            rack_detection_messages_.racks[i].state = result->racks[i].state;
        }
    }// 至此，ros平台完成了对下位（导航定位）传来的数据（货位检测信息）的收集，以rack_detection_messages_私有成员格式呈现。

    // if (result->current_status == 1) {  // 假设1表示货位检测成功
      // 跳出条件，后续需要补充
    while (true) {
      // 更新instantAction消息的数据
      instant_action_messages_ = instant_action_listener->get_instant_action_messages();

      // 1、货位检测完成自动断开：（货位检测任务已完成）
      if (result->current_status == 1) {
        // 跳转至空闲状态
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "货位检测任务已完成!");
        this->config().blackboard->set("last_state", "detecting");
        this->config().blackboard->set("current_state", "idle");
        this->config().blackboard->set("AGV_Event", "init_success");
      }

      // 2、RCS命令断开
      if (instant_action_messages_.action_type == "stop_rack_detection") {
        auto cancel_future = rack_detection_client->send_request(0);
        if (rclcpp::spin_until_future_complete(rack_detection_client, cancel_future) == rclcpp::FutureReturnCode::SUCCESS) {
          auto cancel_result = cancel_future.get();
          if (cancel_result->current_status == 0) {
            // RCS请求跳转至空闲状态成功
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "上位机已停止货位检测任务!");
            this->config().blackboard->set("last_state", "detecting");
            this->config().blackboard->set("current_state", "idle");
            this->config().blackboard->set("AGV_Event", "init_success");
          } else {
            // 异常情况
            this->config().blackboard->set("last_state", "detecting");
            this->config().blackboard->set("current_state", "lock");
            this->config().blackboard->set("AGV_Event", "state_error");
          }
        }
      }
      // 休眠500ms
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "接收到数据！");
    RCLCPP_INFO(rack_detection_client->get_logger(), "success to response! detecting_status = %d", result->current_status);
  } else {
    RCLCPP_INFO(rack_detection_client->get_logger(), "fail to response!");
  }

}