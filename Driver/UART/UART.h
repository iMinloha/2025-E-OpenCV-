#ifndef _UART_H
#define _UART_H

#include <iostream>

// 判断操作系统
#ifdef _WIN32
#undef NOMINMAX
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

// 避免 ACCESS_MASK 冲突
#undef ACCESS_MASK
#define ACCESS_MASK DWORD


class UART {
public:
    // 构造函数，仅配置串口名称
    explicit UART(const std::string& port, int baudRate = 115200);
    // 析构函数，自动关闭串口
    ~UART();
    // 关闭串口
    void close();
    // 发送数据
    bool send(const void* buf, int len);
    // 接收数据
    std::string receive();
    // 是否打开串口
    bool isOpen();

private:
    std::string port_;       // 串口名称（如 "COM3"）
    HANDLE handle_;          // 串口句柄
    bool isOpen_ = false;            // 串口是否打开
};


#else
#include <string>
#include <cstdint>

class UART {
public:
    // 构造函数
    UART(const std::string& device_name, int baud_rate);

    // 析构函数
    ~UART(); // 使用默认析构函数

    // 发送数据
    void send(const uint8_t* buf, int len);

    // 接收数据
    void receive(uint8_t* buf, int len);

    // 是否打开串口
    bool isOpen();

private:
    std::string portname; // 串口设备名称
    int baudrate;         // 波特率
    int fd;               // 文件描述符
    bool isOpen_ = false;            // 串口是否打开
};
#endif

#endif