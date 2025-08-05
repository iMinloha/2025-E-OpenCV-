#ifndef EC_GIMBAL_EMM_CONTROL_H
#define EC_GIMBAL_EMM_CONTROL_H

#include <stdint-gcc.h>

#define RPM_TO_RAD 0.10471975511965977461542144610932f // 角速度转换系数
#define RAD_TO_RPM 9.549296585513721f // 角速度转换系数

#define DEG_TO_RAD 0.017453292519943295769236907684886f // 角度转弧度系数
#define RAD_TO_DEG 57.295779513082320876798154814105f // 弧度转角度系数
#define Angle_To_CLK(angle) ((uint32_t) (angle * 3200.0f / 360.0f)) // 电角度转换为时钟周期数

void Emm_V5_En_Control(uint8_t addr,  uint8_t *tx_cmd);

void EmmV5_PositionControl(uint8_t addr, float angle, float speed, uint8_t *tx_cmd);

#endif //EC_GIMBAL_EMM_CONTROL_H
