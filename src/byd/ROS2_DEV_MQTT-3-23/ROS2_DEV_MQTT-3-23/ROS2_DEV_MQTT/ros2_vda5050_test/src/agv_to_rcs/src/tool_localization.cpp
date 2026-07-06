/************************************** File Info ****************************************
* @file:       tool_localization.cpp                                                                     
* @author:     刘鸿彬                                                              
* @date:       2024-12-19                                         
* @version:    V0.0
* @class:      工具类                                                                              
* @brief:      在导航定位不在的时候，该模块模拟发送位姿信息给ros平台
******************************************************************************************/

#include "tool_localization.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

// 在DM上的agv定位坐标
double agv_with_qr_x ;
double agv_with_qr_y ;
double agv_with_qr_angle ;

// 在DM之间的AGv定位坐标
double agv_without_qr_x ;
double agv_without_qr_y ;
double agv_without_qr_angle ;

// 读取的二维码在地图中坐标
double map_x = 0.0;
double map_y = 0.0;
double map_angle = 0.0;

Localization_Tool::Localization_Tool (const rclcpp::NodeOptions & options) 
: rclcpp::Node("pose_publisher",options) 
{   
    RCLCPP_INFO(this->get_logger(),"localization node! ");
    
    localization_pub = this->create_publisher<PoseWithCovarianceStamped>("map_to_base_pose",1);
    timer = create_wall_timer(std::chrono::milliseconds(1000), [this]() { publish_localization();});

}



// test 测试初始化位置使用
void Localization_Tool::publish_localization()
{

  // 设置 header
  // localization_msg.header.stamp = this->now();  // 当前时间
  localization_msg.header.frame_id = "map";  // 坐标系名称

  // 设置 position
  localization_msg.pose.pose.position.x = 8.0;
  localization_msg.pose.pose.position.y = 0;
  localization_msg.pose.pose.position.z = 3.0;

  // 设置 orientation (四元数)
  localization_msg.pose.pose.orientation.x = 0.0;
  localization_msg.pose.pose.orientation.y = 0.0;
  localization_msg.pose.pose.orientation.z = 0.0;
  localization_msg.pose.pose.orientation.w = 1.0;  // 表示没有旋转

  // 设置 covariance (这里使用全零矩阵作为示例)
  for(auto & elem : localization_msg.pose.covariance) {
    elem = 0.0;
  }
    localization_pub->publish(localization_msg);
}

Localization_Tool::~Localization_Tool(){}

// 此处是模拟定位的小工具。
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto pose_node = std::make_shared<Localization_Tool>(); // 创建节点对象
  
  rclcpp::spin(pose_node);
  rclcpp::shutdown();
  return 0;

}
