#include "DrivceTask.h"
#include "Camera.h"
#include "USB/USBCamera.h"
#include "UTD.h"
#include "UART.h"
#include <iostream>
#include "PID.hpp"
#include "Emm_Control.h"
#include "Detector.h"
//#include <wiringPi.h>

const int use_emm_v5 = 1;
using namespace std;
Point2f cali_center(363.85, 250.20);

// PID参数设定
PID yaw_pid(-0.01, 0.0, -0.01, 1, 10, 1);

PID pitch_pid(0.01, 0.0, 0.01, 1, 10, 1);

// PID yaw_pid(-0.03, 0.0, 0.0, 20, 20, 1);
// PID pitch_pid(0.0, 0.0, 0.0, 20, 20, 1);


// PID yaw_pid(-0.005, 0, 0, 20, 20, 1);
// PID pitch_pid(0, 0, 0, 20, 20, 1);

uint8_t tx_cmd[13] = {0}; // 发送命令缓存

[[noreturn]] void CalibrationTask() {
    Camera *pUsbCamera = new USBCamera(0, 320, 240, 0.8, 0);
    cv::Mat frame;
    Detector detector;

    while (true) {
        frame = pUsbCamera->capture();
        Point2f purple = detector.purple_point(frame);

        cv::circle(frame, purple, 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("frame", frame);
        waitKey(1);
        if (purple.x > 0 && purple.y > 0) {
            std::cout << purple << std::endl;
            cali_center = purple;
        }
    }
}


class Kalman {
public:
    Kalman(float dt = 0.033f) : dt(dt) { // 存储dt用于动态调整
        // 状态向量: [位置, 速度, 加速度]
        state = (cv::Mat_<float>(3, 1) << 0, 2.5f, 10);

        // 状态转移矩阵 (三阶匀加速模型)
        F = (cv::Mat_<float>(3, 3) <<
                                   1, dt, 0.5*dt*dt,
                0,  1,       dt,
                0,  0,        1);

        // 观测矩阵 (只能观测位置)
        H = (cv::Mat_<float>(1, 3) << 1, 0, 0);

        // 初始协方差 (更大的加速度不确定性)
        P = (cv::Mat_<float>(3, 3) <<
                                   100,   0,    0,
                0,  10,    0,
                0,   0,   50);

        // 过程噪声 (调整加速度噪声权重)
        q_acc_base = 1.0f; // 基准加速度噪声
        Q = (cv::Mat_<float>(3, 3) <<
                                   0.1f,     0,     0,
                0,  0.5f,    0,
                0,     0, q_acc_base);

        // 观测噪声
        R = (cv::Mat_<float>(1, 1) << 10);
    }

    // 主更新接口 (保持原有签名)
    float update(float measurement) {
        predictInternal();       // 分离预测逻辑
        correct(measurement);    // 分离更新逻辑
        return state.at<float>(0);
    }

    // 新增：获取未来N步预测 (不更新内部状态)
    std::vector<float> predict(int steps) const {
        std::vector<float> predictions;
        cv::Mat temp_state = state.clone();
        cv::Mat temp_F = F.clone();

        for (int i = 0; i < steps; ++i) {
            temp_state = temp_F * temp_state;
            predictions.push_back(temp_state.at<float>(0));
        }
        return predictions;
    }

    // 新增：动态调整时间间隔 (应对帧率变化)
    void setDt(float new_dt) {
        dt = new_dt;
        F = (cv::Mat_<float>(3, 3) <<
                                   1, dt, 0.5*dt*dt,
                0,  1,       dt,
                0,  0,        1);
    }

    // 获取当前状态
    float getPosition() const { return state.at<float>(0); }
    float getVelocity() const { return state.at<float>(1); }
    float getAcceleration() const { return state.at<float>(2); }

private:
    float dt;          // 时间间隔
    float q_acc_base;  // 基准加速度噪声
    cv::Mat state, F, H, P, Q, R;

    // 拆分的预测和更新逻辑
    void predictInternal() {
        state = F * state;
        P = F * P * F.t() + Q;

        // 动态噪声调整：加速度变化剧烈时增大Q(2,2)
        float accel_change = abs(state.at<float>(2));
        Q.at<float>(2,2) = q_acc_base * (1.0f + 0.5f * accel_change);
    }

    void correct(float measurement) {
        cv::Mat K = P * H.t() * (H * P * H.t() + R).inv();
        cv::Mat z = (cv::Mat_<float>(1, 1) << measurement);
        state = state + K * (z - H * state);
        P = (cv::Mat::eye(3,3,CV_32F) - K * H) * P;
    }
};



// 这里是相机捕获线程, 自行修改
[[noreturn]] void CameraCaptureTask() {
    Camera * pUsbCamera = new USBCamera(0, 640, 480, 0.8, 0);
    cv::Mat frame;
    cv::Point2f gimbal_angle;
    Detector detector;
    Kalman kf_x;
    UART uart("/dev/ttyS6", 115200);
    if (uart.isOpen()) std::cout << "Serial device Opened successful!" << endl;
    else std::cerr << "Error: Serial device Open failed, \"UARTTask\" destroy!" << endl;
//    wiringPiSetup();
//    pinMode(1, INPUT);
    float last_valid_x = 320; // 默认初始位置（图像中心）

    while (true) {
        frame = pUsbCamera->capture();
        Point2f center = detector.run(frame);
//        int pin_value = digitalRead(1);

//        if (pin_value == 0 && center.x < 0 && center.y < 0) {
//            center.x = 350;
//            center.y = 240;
//        }

        if (center.x < 0 || center.y < 0) {
            yaw_pid.clear();
            pitch_pid.clear();
            continue;
        }

        // 对center.x进行卡尔曼滤波，速度为0.25m/s
        last_valid_x = kf_x.update(center.x);
        center.x = kf_x.predict(5)[4];


	    cv::circle(frame, center, 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(frame, cali_center, 5, cv::Scalar(255, 0, 0), -1);

        imshow("frame", frame);
        waitKey(1);

        std::cout << center << " | " << cali_center << std::endl;
        
        gimbal_angle.x += yaw_pid.calculate(cali_center.x, center.x);
        gimbal_angle.y += pitch_pid.calculate(cali_center.y, center.y);

        gimbal_angle.y = std::clamp(gimbal_angle.y, -10.0f, 10.0f);


        // 发布坐标控制电机
        EmmV5_PositionControl(0x02, gimbal_angle.x, 30, tx_cmd);
        uart.send(tx_cmd, 13);
        UTD::msDelay(10);

        EmmV5_PositionControl(0x01, gimbal_angle.y, 30, tx_cmd);
        uart.send(tx_cmd, 13);
        UTD::msDelay(10);

	    std::cout << gimbal_angle << std::endl;
    }
}
