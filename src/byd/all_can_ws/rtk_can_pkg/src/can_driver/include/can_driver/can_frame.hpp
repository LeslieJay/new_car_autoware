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


struct ins_pos_can_t {
    uint8_t  valid;
    uint8_t  state;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint32_t seconds;
    int32_t  pos_ins[3];
    int16_t  vel[3];
    int16_t  atti[2];
    uint16_t atti3;
    int16_t  gyro_xyz[3];
    int16_t  acc_xyz[3];
} __attribute__((packed));

struct ins_atti_can_t {
    int16_t  gyro_xyz[3];
    int16_t  acc_xyz[3];
    int16_t temp;
} __attribute__((packed));
} // namespace can_driver