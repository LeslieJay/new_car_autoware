// can_driver/include/can_driver/can_frame.hpp
#pragma once
#include <cstdint>

namespace can_driver {

struct CanFrame
{
    uint32_t frameId;
    uint8_t  dataLen;
    uint8_t  data[64];
};
constexpr double INT_POS = 1e8;
constexpr double INT_ALT = 1e8;
constexpr double INT_IMU = 1e6;
constexpr int INT_SEC = 10000;

struct ins_pos_can_t {
    uint8_t  valid;
    uint8_t  state;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint64_t seconds;
    int64_t  pos_ins[3];
    int64_t  vel[3];
} __attribute__((packed));

struct ins_atti_can_t {
    uint8_t  valid;
    uint8_t  state;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    int64_t  seconds;
    int64_t  atti[3];
    int32_t  gyro_xyz[3];
    int32_t  acc_xyz[3];
} __attribute__((packed));
} // namespace can_driver