#include <cmath>
#include "Emm_Control.h"

void Emm_V5_En_Control(uint8_t addr,  uint8_t *tx_cmd) {
    tx_cmd[0] =  addr;                       // 地址
    tx_cmd[1] =  0xF3;                       // 功能码
    tx_cmd[2] =  0xAB;                       // 辅助码
    tx_cmd[3] =  (uint8_t) 0x00;             // 使能状态
    tx_cmd[4] =  0x00;                        // 多机同步运动标志
    tx_cmd[5] =  0x6B;                       // 校验字节
}

void EmmV5_PositionControl(uint8_t addr, float angle, float speed, uint8_t *tx_cmd) {
    float abs_angle = fabs(angle);
    uint32_t clk = Angle_To_CLK(abs_angle);
    uint16_t vel = (uint16_t)(speed * RAD_TO_RPM); // 转换为RPM
    tx_cmd[0]  =  addr;                      // 地址
    tx_cmd[1]  =  0xFD;                      // 功能码
    tx_cmd[2]  =  angle < 0 ? 1 : 0;         // 方向
    tx_cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
    tx_cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节
    tx_cmd[5]  =  250;                        // 加速度，注意：0是直接启动
    tx_cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
    tx_cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
    tx_cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
    tx_cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
    tx_cmd[10] =  1;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
    tx_cmd[11] =  0;                       // 多机同步运动标志，false为不启用，true为启用
    tx_cmd[12] =  0x6B;                      // 校验字节
}



void EmmV3_PositionControl(uint8_t addr, float angle, uint8_t speed, uint8_t *tx_cmd) {
    auto vel = speed;
    uint16_t clk = Angle_To_CLK(angle);
    tx_cmd[0] = addr;
    tx_cmd[1] = 0xfd;
    tx_cmd[2] = (angle < 0 ? 0x80 : 0x00) | vel;
    tx_cmd[3] = (clk >> 8) & 0xff;
    tx_cmd[4] = clk & 0xff;
}
