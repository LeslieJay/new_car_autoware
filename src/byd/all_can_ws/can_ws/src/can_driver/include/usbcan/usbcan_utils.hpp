/*
 * @Author: LiFang6606397
 * @Date: 2024-03-26 16:13:48
 * @LastEditors: LiFang6606397
 * @LastEditTime: 2025-10-10 14:13:44
 * @FilePath: /work_ws/src/usbcan/include/usbcan/usbcan_utils.hpp
 * @Description: can通信通用接口抽象
 * 
 * Copyright (c) 2024 by LiFang6606397, All Rights Reserved. 
 */
#ifndef __USBCAN_UTILS__H__
#define __USBCAN_UTILS__H__

// #include  <time.h> 
// #include  <thread>

#include <chrono>
#include <iostream>
#include <string.h>
#include <string>
#include <vector>
#include <thread>
#include <bitset>
#include <cmath>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"

#include "usbcan/controlcan.h"
#define MAX_CHANNELS  2

// 设备类型号 usbcan-ii:4
extern unsigned gDevType;
// 设备索引号，当只有一个设备时为0，增加同一个类型设备时为1
extern unsigned gDevIdx;
// bit0-CAN1, bit1-CAN2, bit2-CAN3, bit3-CAN4, 3=CAN1+CAN2, 7=CAN1+CAN2+CAN3
extern unsigned gChMask;
// 发送数据类型 0-normal, 1-single, 2-self_test, 3-single_self_test, 4-single_no_wait....
extern unsigned gTxType;
// 发送的暂停时间，暂时未使用
extern unsigned gTxSleep;
// 一次发送帧数
extern unsigned gTxFrames;
// 发送次数
extern unsigned gTxCount;
// 保留参数，通常为 0
extern int gReserved;
// 波特率.下标表示通道. 0x1c01 = 250k, 0x1c03 = 125k
extern std::vector<unsigned> gBaud;

struct RX_CTX{
    unsigned channel; // channel index, 0~3
    unsigned stop; // stop RX-thread
    unsigned total; // total received
    unsigned error; // error(s) detected
};

struct GeneralFrame{
  UINT ID;
	UINT TimeStamp; // 设备接收到某一帧的时间标识。时间标示从 CAN 卡上电开始计时，计时单位为 0.1ms。
	BYTE TimeFlag = 0;  // 是否使用时间标识。为 1 时 TimeStamp 有效，TimeFlag 和 TimeStamp 只在此帧为接收帧时有意义。
	/** 发送帧类型:
   * =0 时为正常发送（发送失败会自动重发，重发最长时间为 1.5-3 秒）
   * =1 时为单次发送（只发送一次，不自动重发）；
   * =2 时为自发自收（自测试模式，用于测试 CAN 卡是否损坏）；
   * =3 时为单次自发自收（单次自测试模式，只发送一次）。
   * 只在此帧为发送帧时有意义。
   */
  BYTE SendType = 1;  
	BYTE RemoteFlag = 0;// 是否是远程帧。=0 时为为数据帧，=1 时为远程帧（数据段空）
	BYTE ExternFlag = 0;// 是否是扩展帧。=0 时为标准帧（11 位 ID），=1 时为扩展帧（29 位 ID）
  BYTE DataLen; // 数据长度 DLC (<=8)，即 CAN 帧 Data 有几个字节。约束了后面 Data[8]中的有效字节。

  BYTE can_data[8];
};

int8_t ToInt8(int byte);
uint8_t ToUint8(int byte);

int ToInt16(int high, int low);
unsigned int ToUint16(int high, int low);

int32_t ToInt32(int byte3, int byte2, int byte1, int byte0);
uint32_t ToUint32(int byte3, int byte2, int byte1, int byte0);

void FillFrame(VCI_CAN_OBJ *can, GeneralFrame frame);

void SendMessages(std::vector<GeneralFrame>& frames);

GeneralFrame gen_0x184(int speed, int angle, int fork_actual_height, int VCUstatus);

GeneralFrame gen_0x204(
	unsigned int AGV_ID,
	unsigned int OperationMode,
	unsigned int CurrentNavMethod);

GeneralFrame gen_0x304(
	int SetSpeed,
	int SetAngle,
	int SteerEncOffset,
	int Fork_SetHeight);

can_frame gen_0x404(
	int8_t IPC_Status,
	int PositionAngle = 0);

GeneralFrame gen_0x504(
	uint32_t PositionX, 
	uint32_t PositionY);

GeneralFrame gen_0x224(
	uint8_t IPC_ProgramType, 
	uint32_t IPC_ProgramDisplayVer);

GeneralFrame gen_0x324(
	uint32_t IPC_ProgramTimeVer, 
	uint32_t LayoutVer);

GeneralFrame gen_0x424(
	uint8_t IPC_EMS_Signal);

GeneralFrame gen_0x524(
	uint8_t IPC_HeartBeat, 
	uint8_t BackLaserFieldSel, 
	uint8_t LeftLaserFieldSel, 
	uint8_t HeadLaserFieldSel, 
	uint8_t RightLaserFieldSel);

GeneralFrame gen_0x344(std::string str);
extern int hook;
#endif  //!__USBCAN_UTILS__H__