/*
 * @Author: LiFang6606397
 * @Date: 2024-05-08 16:04:38
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2025-10-10 14:19:24
 * @FilePath: /work_ws/src/usbcan/src/usbcan_utils.cpp
 * @Description: usbcan功能包 通用函数的实现
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */
#include <linux/can.h>
#include <linux/can/raw.h> 
#include <vector>
#include <cstdint>
#include "usbcan/usbcan_utils.hpp"

unsigned gDevType = 4;
unsigned gDevIdx = 0;
unsigned gChMask = 1;
unsigned gTxType = 2;
unsigned gTxSleep = 59;
unsigned gTxFrames = 2;
unsigned gTxCount = 1000;
int gReserved = 0;
std::vector<unsigned> gBaud{0X1C01, 0X1C01}; // CAN1使用250k波特率


bool usbcan_init();
/// @brief 将单个字节转换为8位有符号整数
/// @param byte 字节值（0-255）
/// @return 返回转换后的8位有符号整数（-128 到 127）
int8_t ToInt8(int byte) {
  int8_t result = static_cast<int8_t>(byte & 0xFF);
  return result;
}

/// @brief 将单个字节转换为8位无符号整数
/// @param byte 字节值（0-255）
/// @return 返回转换后的8位无符号整数（0 到 255）
uint8_t ToUint8(int byte) {
  return static_cast<uint8_t>(byte & 0xFF);
}

/// @brief 将两个字节转换为一个16位有符号整数
/// @param high 高8位的字节（0-255）
/// @param low 低8位的字节（0-255）
/// @return 返回转换后的16位有符号整数（-32768 到 32767）
int ToInt16(int high, int low) {
    // 将两个字节组合成16位整数，使用位运算
    // int16_t 会自动处理有符号数的补码表示
    int16_t result = static_cast<int16_t>((high << 8) | (low & 0xFF));
    return static_cast<int>(result);
}

/// @brief 将两个字节转换为一个16位无符号整数
/// @param high 高8位的字节（0-255）
/// @param low 低8位的字节（0-255）
/// @return 返回转换后的16位无符号整数（0 到 65535）
unsigned int ToUint16(int high, int low) {
    uint16_t result = static_cast<uint16_t>((high << 8) | (low & 0xFF));
    return static_cast<unsigned int>(result);
}

/// @brief 将四个字节转换为32位有符号整数
/// @param byte3 最高8位的字节（0-255）
/// @param byte2 次高8位的字节（0-255）
/// @param byte1 次低8位的字节（0-255）
/// @param byte0 最低8位的字节（0-255）
/// @return 返回转换后的32位有符号整数（-2147483648 到 2147483647）
int32_t ToInt32(int byte3, int byte2, int byte1, int byte0) {
  int32_t result = static_cast<int32_t>((byte3 << 24) | ((byte2 & 0xFF) << 16) | 
                                         ((byte1 & 0xFF) << 8) | (byte0 & 0xFF));
  return result;
}

/// @brief 将四个字节转换为32位无符号整数
/// @param byte3 最高8位的字节（0-255）
/// @param byte2 次高8位的字节（0-255）
/// @param byte1 次低8位的字节（0-255）
/// @param byte0 最低8位的字节（0-255）
/// @return 返回转换后的32位无符号整数（0 到 4294967295）
uint32_t ToUint32(int byte3, int byte2, int byte1, int byte0) {
    uint32_t result = static_cast<uint32_t>((byte3 << 24) | ((byte2 & 0xFF) << 16) | 
                                             ((byte1 & 0xFF) << 8) | (byte0 & 0xFF));
    return result;
}


/// @brief 根据传入的通用数据帧结构体填充对应的CAN数据
/// @param can can对象指针
/// @param frame 通用数据帧结构体
void FillFrame(VCI_CAN_OBJ *can, GeneralFrame frame){   
  // 将指定的内存区域设置为一个特定的值,将can内存全设置为0
  memset(can, 0, sizeof(VCI_CAN_OBJ));
  
  // 填充CAN数据
  can->ID = frame.ID;
  can->DataLen = frame.DataLen;
  can->SendType = frame.SendType;
  can->DataLen = frame.DataLen;
  can->ExternFlag = frame.ExternFlag;

  for(int i = 0; i < 8; i++){
    can->Data[i] = frame.can_data[i];
  }
}

// /// @brief 定期向EPEC发送数据
// /// @param frames 需要发送的通用数据帧结构体
// void SendMessages(std::vector<GeneralFrame>& frames){
//   // 动态分配内存
//   VCI_CAN_OBJ *buff = (VCI_CAN_OBJ *)malloc(sizeof(VCI_CAN_OBJ) * frames.size());
  
//   // 错误标识
//   bool err = false;

//   // 上一次数据发送成功才能继续发送数据
//   if(!err){
//     // 判断一次需要发送几帧数据，将要发送的数据装载到CAN结构体上
//     for(int j = 0; j < frames.size(); j++){
//       FillFrame(&buff[j], frames[j]);
//     }

//     auto start = std::chrono::steady_clock::now(); // 开始计时
//     // TODO: 这里看看是否可以整合
//     if(frames.size() != VCI_Transmit(gDevType,gDevIdx, 1, &buff[0], frames.size())){
//       RCLCPP_INFO_STREAM(
//         rclcpp::get_logger("rclcpp"),
//         "----CAN" << 0 << "TX failed----" << "ID = 0x" << buff->ID);
//       err = true;
//     }
//     auto end = std::chrono::steady_clock::now(); // 结束计时
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); // 计算时间差
//     auto remainingTime = 60 - elapsedTime;
//     std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime)); // 以每60ms一次的频率发送

//     end = std::chrono::steady_clock::now(); // 计算带休眠运行时间
//     auto diff = end - start;
//     //  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
//     //    "Code executed in " << diff.count() * 1e-6 << " ms\n"); 
//   } 
//   free(buff);
// }

/// @brief 由EPEC发布的数据帧，用于无实车模拟测试
/// @param speed 舵轮速度
/// @param angle 舵轮转角
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x184(int speed, int angle, int fork_actual_height, int VCUstatus){
  GeneralFrame send_0x184;

  send_0x184.ID = 0x184;
  send_0x184.SendType = gTxType;
  send_0x184.DataLen = 8;
  send_0x184.ExternFlag = 0;

  send_0x184.can_data[0] = speed & 0xff;
  send_0x184.can_data[1] = speed >> 8;
  send_0x184.can_data[2] = angle & 0xff;
  send_0x184.can_data[3] = angle >> 8;
  send_0x184.can_data[4] = fork_actual_height & 0xff;
  send_0x184.can_data[5] = fork_actual_height >> 8;
  send_0x184.can_data[6] = VCUstatus & 0xff;
  send_0x184.can_data[7] = 0;

  return send_0x184;
}

/// @brief 包含AGV控制模式的数据帧
/// @param AGV_ID AGV车辆ID
/// @param OperationMode 操作模式. 0:自动  1:手动 2:半自动 3:半手动
/// @param CurrentNavMethod 当前导航模式. 1:反光板 2:色带 4:磁条 5: 反光墙 6:自然 7:二维码
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x204(unsigned int AGV_ID,unsigned int OperationMode,unsigned int CurrentNavMethod){
  
  GeneralFrame send_0x204;

  send_0x204.ID = 0x204;
  send_0x204.SendType = gTxType;
  send_0x204.DataLen = 4;
  send_0x204.ExternFlag = 0;

  send_0x204.can_data[0] = AGV_ID & 0xff;
  send_0x204.can_data[1] = AGV_ID >> 8;
  send_0x204.can_data[2] = OperationMode;    // 0
  send_0x204.can_data[3] = CurrentNavMethod; // 0x01

  return send_0x204;
}

/// @brief 设置舵轮速度和转角，货叉高度，舵轮转角补偿值
/// @param SetSpeed 舵轮速度指令 正值前进负值后退，单位：mm/s
/// @param SetAngle 舵轮转角指令 -9000 - 9000：-90 - 90度, 0.01deg
/// @param SteerEncOffset 用于修正舵角机械误差的偏移量
/// @param Fork_SetHeight 门架给定起升高度
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x304(int SetSpeed, int SetAngle, int SteerEncOffset, int Fork_SetHeight){

  GeneralFrame send_0x304;

  send_0x304.ID = 0x304;
  send_0x304.SendType = gTxType;
  send_0x304.DataLen = 8;
  send_0x304.ExternFlag = 0;

  send_0x304.can_data[0] = SetSpeed & 0xff;
  send_0x304.can_data[1] = SetSpeed >> 8;
  send_0x304.can_data[2] = SetAngle & 0xff;
  send_0x304.can_data[3] = SetAngle >> 8;
  send_0x304.can_data[4] = SteerEncOffset & 0xff;
  send_0x304.can_data[5] = SteerEncOffset >> 8;
  send_0x304.can_data[6] = Fork_SetHeight & 0xff;
  send_0x304.can_data[7] = Fork_SetHeight >> 8;

  return send_0x304;
}

/// @brief 设置小车各种使能信号
/// @param IPC_Status 工控机状态.
///  0 |—— `EnableSteer` 转向使能;
///  1 |—— `EnableDrive` 行驶使能;
///  2 |—— `Charing_Enable` 充电使能;
///  3 |—— `Fork_EnableHeight` 货叉起升下降使能;
///  4 |—— `LostNav` 导航丢失;
///  5 |—— `Camera_EnableHeight` 相机伸出使能;
///  6 |—— `SetLoadStateCallBack` 设置货位状态反馈;
///  7 |—— 保留;
/// @param PositionAngle 当前导航角度
/// @return 通用数据帧结构体 GeneralFrame
can_frame gen_0x404(int8_t IPC_Status, int PositionAngle){

  can_frame send_0x404;

  send_0x404.can_id = 0x404;
  send_0x404.can_dlc = 8;
  
  send_0x404.data[0] = IPC_Status;
  send_0x404.data[1] = PositionAngle & 0xff;
  send_0x404.data[2] = PositionAngle >> 8;
  send_0x404.data[3] = 0x00; 
  send_0x404.data[4] = 0x00; 
  send_0x404.data[5] = 0x00; 
  send_0x404.data[6] = 0x00; 
  send_0x404.data[7] = 0x00; 

  // 打印发送 0x404 报文信息
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "[0x404] 发送报文 - IPC_Status: " << static_cast<int>(IPC_Status) 
    << " (0x" << std::hex << static_cast<int>(IPC_Status) << std::dec
    << "), PositionAngle: " << PositionAngle
    << ", Data: [0x" << std::hex 
    << static_cast<int>(send_0x404.data[0]) << " "
    << static_cast<int>(send_0x404.data[1]) << " "
    << static_cast<int>(send_0x404.data[2]) << " "
    << static_cast<int>(send_0x404.data[3]) << std::dec << "]");
  
  return send_0x404;
}

/// @brief 设置小车导航坐标
/// @param PositionX 导航坐标X
/// @param PositionY 导航坐标Y
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x504(uint32_t PositionX, uint32_t PositionY){
  GeneralFrame send_0x504;

  send_0x504.ID = 0x504;
  send_0x504.SendType = gTxType;
  send_0x504.DataLen = 8;
  send_0x504.ExternFlag = 0;

  send_0x504.can_data[0] = PositionX & 0xff;
  send_0x504.can_data[1] = (PositionX >> 8) & 0xff;
  send_0x504.can_data[2] = (PositionX >> 16) & 0xff;
  send_0x504.can_data[3] = (PositionX >> 24) & 0xff;
  send_0x504.can_data[4] = PositionY & 0xff;
  send_0x504.can_data[5] = (PositionY >> 8) & 0xff;
  send_0x504.can_data[6] = (PositionY >> 16) & 0xff;
  send_0x504.can_data[7] = (PositionY >> 24) & 0xff;

  return send_0x504;
}

/// @brief 设置工控机程序版本类型和显示版本
/// @param IPC_ProgramType 工控机程序版本类型  1:初版，2:测试版本，3：稳定版本
/// @param IPC_ProgramDisplayVer 工控机显示版本  举例：010101
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x224(uint8_t IPC_ProgramType, uint32_t IPC_ProgramDisplayVer){
  GeneralFrame send_0x224;

  send_0x224.ID = 0x224;
  send_0x224.SendType = gTxType;
  send_0x224.DataLen = 5;
  send_0x224.ExternFlag = 0;

  send_0x224.can_data[0] = IPC_ProgramType;
  send_0x224.can_data[1] = IPC_ProgramDisplayVer & 0xff;
  send_0x224.can_data[2] = (IPC_ProgramDisplayVer >> 8) & 0xff;
  send_0x224.can_data[3] = (IPC_ProgramDisplayVer >> 16) & 0xff;
  send_0x224.can_data[4] = (IPC_ProgramDisplayVer >> 24) & 0xff;

  return send_0x224;
}

/// @brief 设置工控机程序时间版本和地图版本
/// @param IPC_ProgramTimeVer 工控机程序时间版本  举例：250417
/// @param LayoutVer 地图版本号
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x324(uint32_t IPC_ProgramTimeVer, uint32_t LayoutVer){
  GeneralFrame send_0x324;
  send_0x324.ID = 0x324;
  send_0x324.SendType = gTxType;
  send_0x324.DataLen = 8;
  send_0x324.ExternFlag = 0;

  send_0x324.can_data[0] = IPC_ProgramTimeVer & 0xff;
  send_0x324.can_data[1] = (IPC_ProgramTimeVer >> 8) & 0xff;
  send_0x324.can_data[2] = (IPC_ProgramTimeVer >> 16) & 0xff;
  send_0x324.can_data[3] = (IPC_ProgramTimeVer >> 24) & 0xff;
  send_0x324.can_data[4] = LayoutVer & 0xff;
  send_0x324.can_data[5] = (LayoutVer >> 8) & 0xff;
  send_0x324.can_data[6] = (LayoutVer >> 16) & 0xff;
  send_0x324.can_data[7] = (LayoutVer >> 24) & 0xff;

  return send_0x324;
}

/// @brief 设置工控机EMS信号
/// @param IPC_EMS_Signal 工控机EMS信号
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x424(uint8_t IPC_EMS_Signal){
  GeneralFrame send_0x424;

  send_0x424.ID = 0x424;
  send_0x424.SendType = gTxType;
  send_0x424.DataLen = 1;
  send_0x424.ExternFlag = 0;

  send_0x424.can_data[0] = IPC_EMS_Signal;

  return send_0x424;
}

/// @brief 生成工控机心跳CAN帧 (0x524)
/// @param IPC_HeartBeat 通信超时  工控机自己监控其他模块的超时， 若该信号为真时停车，闪烁信号
/// @param BackLaserFieldSel 后激光区域
/// @param LeftLaserFieldSel 左激光区域
/// @param HeadLaserFieldSel 顶部激光区域
/// @param RightLaserFieldSel 右激光区域
/// @return 通用数据帧结构体 GeneralFrame
GeneralFrame gen_0x524(uint8_t IPC_HeartBeat, uint8_t BackLaserFieldSel, uint8_t LeftLaserFieldSel, uint8_t HeadLaserFieldSel, uint8_t RightLaserFieldSel){
  GeneralFrame send_0x524;

  send_0x524.ID = 0x524;
  send_0x524.SendType = gTxType;
  send_0x524.DataLen = 5;
  send_0x524.ExternFlag = 0;

  send_0x524.can_data[0] = IPC_HeartBeat;
  send_0x524.can_data[1] = BackLaserFieldSel;
  send_0x524.can_data[2] = LeftLaserFieldSel;
  send_0x524.can_data[3] = HeadLaserFieldSel;
  send_0x524.can_data[4] = RightLaserFieldSel;

  return send_0x524;
}

GeneralFrame gen_0x344(std::string str){
  GeneralFrame send_0x344;

  send_0x344.ID = 0x344;
  send_0x344.SendType = gTxType;
  send_0x344.DataLen = 8;
  send_0x344.ExternFlag = 0;

  for(int i = 0; i < 8; i++){
    send_0x344.can_data[i] = str[i];
  }
  return send_0x344;
}
