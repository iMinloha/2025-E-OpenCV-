#ifndef EC_GIMBAL_PID_HPP
#define EC_GIMBAL_PID_HPP

#include "opencv2/opencv.hpp"

class PID {
public:
    float kp, ki, kd;       // PID系数
    float max_iout;         // 积分限幅
    float max_out;          // 输出限幅
    int mode;               // 1:位置式PID  2:增量式PID

    PID(float kp, float ki, float kd, float max_iout, float max_out, int mode) : kp(kp),
                    ki(ki), kd(kd), mode(mode), max_iout(max_iout), max_out(max_out) {};

    // 计算PID输出
    float calculate(float ref, float set) {
        updateErrors(set, ref);

        if (mode == 1) {
            return positionalPID();
        } else if (mode == 2) {
            return incrementalPID();
        }
        return 0.0f;
    }

    void clear() {
        for (int i = 0; i < 3; i ++) {
            error[i] = 0; Dbuf[i] = 0;
        }
    }

private:
    // 状态变量
    float error[3] = {0};   // 当前、前一次、前两次误差
    float Dbuf[3] = {0};    // 微分项缓冲区
    float pid_out[3] = {0}; // P/I/D输出分量
    float out = 0;          // 总输出
    float fdb = 0;          // 反馈值
    float set = 0;          // 设定值

    // 更新误差序列
    void updateErrors(float new_set, float new_fdb) {
        error[2] = error[1];
        error[1] = error[0];
        set = new_set;
        fdb = new_fdb;
        error[0] = set - fdb;
    }

    // 位置式PID
    float positionalPID() {
        // P项
        if (error[0] > 100) pid_out[0] = 2 * kp * error[0];
        else pid_out[0] = kp * error[0];

        // I项（带积分限幅）
        pid_out[1] += ki * error[0];
        pid_out[1] = clamp(pid_out[1], -max_iout, max_iout);

        // D项（微分先行）
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = error[0] - error[1];
        pid_out[2] = kd * Dbuf[0];

        // 总和输出
        out = pid_out[0] + pid_out[1] + pid_out[2];
        return clamp(out, -max_out, max_out);
    }

    // 增量式PID
    float incrementalPID() {
        // P项
        pid_out[0] = kp * error[0];

        // I项
        pid_out[1] = ki * error[0];
        pid_out[1] = clamp(pid_out[1], -max_iout, max_iout);

        // D项（二阶差分）
        Dbuf[2] = Dbuf[1];
        Dbuf[1] = Dbuf[0];
        Dbuf[0] = error[0] - 2 * error[1] + error[2];
        pid_out[2] = kd * Dbuf[0];

        // 增量输出
        out += pid_out[0] + pid_out[1] + pid_out[2];
        return clamp(out, -max_out, max_out);
    }

    // 数值限幅函数
    float clamp(float value, float min, float max) {
        return (value < min) ? min : (value > max) ? max : value;
    }
};


#endif //EC_GIMBAL_PID_HPP
