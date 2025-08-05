#include "UART.h"
#include <iostream>


#ifdef _WIN32

#include "UART.h"
#include <iostream>
#include <stdexcept>

UART::UART(const std::string& port, int baudRate)
        : port_("\\\\.\\" + port), handle_(INVALID_HANDLE_VALUE), isOpen_(false) {
    handle_ = CreateFileA(port_.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
        std::cerr << "Failed to open serial port: " << port_ << std::endl;
        return;
    }

    // 配置串口参数
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState(handle_, &dcb)) {
        std::cerr << "Failed to get serial port state." << std::endl;
        close();
        return;
    }

    dcb.BaudRate = baudRate;     // 波特率
    dcb.ByteSize = 8;            // 数据位
    dcb.StopBits = ONESTOPBIT;   // 停止位
    dcb.Parity = NOPARITY;       // 无校验位

    if (!SetCommState(handle_, &dcb)) {
        std::cerr << "Failed to set serial port state." << std::endl;
        close();
        return;
    }

    isOpen_ = true;
    std::cout << "Serial port opened: " << port_ << std::endl;
}

UART::~UART() {
    close();
}

void UART::close() {
    if (isOpen_) {
        CloseHandle(handle_);
        handle_ = INVALID_HANDLE_VALUE;
        isOpen_ = false;
        std::cout << "Serial port closed: " << port_ << std::endl;
    }
}

bool UART::send(const void* buf, int len) {
    // 检查串口是否打开
    if (!isOpen_) {
        std::cerr << "Error: Serial port is not open." << std::endl;
        return false;
    }

    // 检查缓冲区指针和长度是否有效
    if (!buf || len <= 0) {
        std::cerr << "Error: Invalid buffer or length." << std::endl;
        return false;
    }

    // 发送数据
    DWORD bytesWritten;
    if (!WriteFile(handle_, buf, static_cast<DWORD>(len), &bytesWritten, nullptr)) {
        std::cerr << "Error: Failed to write to serial port. Error code: " << GetLastError() << std::endl;
        return false;
    }

    // 检查是否成功写入所有数据
    if (bytesWritten != static_cast<DWORD>(len)) {
        std::cerr << "Error: Failed to write all bytes to serial port. Bytes written: " << bytesWritten << std::endl;
        return false;
    }

    return true;
}



std::string UART::receive() {
    if (!isOpen_) {
        std::cerr << "Serial port is not open." << std::endl;
        return "";
    }

    char buffer[256];
    DWORD bytesRead;
    if (!ReadFile(handle_, buffer, sizeof(buffer) - 1, &bytesRead, nullptr)) {
        std::cerr << "Failed to read from serial port." << std::endl;
        return "";
    }

    buffer[bytesRead] = '\0'; // 添加字符串结束符
    return std::string(buffer);
}


bool UART::isOpen(){
    return this->isOpen_;
}


#else
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <system_error>
#include <cstring>
#include <stdexcept>

// 构造函数
UART::UART(const std::string& device_name, int baud_rate)
    : portname(device_name), baudrate(baud_rate), fd(-1), isOpen_(false) {

    // 打开串口设备（非阻塞模式打开）
    fd = open(portname.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        throw std::system_error(errno, std::system_category(),
                              "Failed to open UART device");
    }

    // 检查是否为终端设备
    if (!isatty(fd)) {
        close(fd);
        throw std::runtime_error(portname + " is not a terminal device");
    }

    // 配置串口参数
    struct termios options;
    memset(&options, 0, sizeof(options));

    // 获取当前属性
    if (tcgetattr(fd, &options) != 0) {
        close(fd);
        throw std::system_error(errno, std::system_category(),
                              "tcgetattr failed");
    }

    // 设置波特率
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            close(fd);
            throw std::invalid_argument("Unsupported baud rate");
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 基本配置
    options.c_cflag &= ~PARENB;   // 无奇偶校验
    options.c_cflag &= ~CSTOPB;   // 1停止位
    options.c_cflag &= ~CSIZE;    // 清除数据位掩码
    options.c_cflag |= CS8;       // 8数据位
    options.c_cflag |= (CLOCAL | CREAD); // 本地连接，启用接收

    // 原始模式配置
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    // 设置超时：15秒（以100ms为单位）
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN]  = 0;

    // 刷新并应用设置
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        close(fd);
        throw std::system_error(errno, std::system_category(),
                              "tcsetattr failed");
    }

    isOpen_ = true;
}

UART::~UART() {
    if (isOpen_) {
        close(fd);
        isOpen_ = false;
    }
}

void UART::send(const uint8_t* buf, int len) {
    if (!isOpen_) {
        throw std::runtime_error("UART port is not open");
    }

    int written = write(fd, buf, len);
    if (written < 0) {
        throw std::system_error(errno, std::system_category(),
                              "UART write failed");
    }
    if (written != len) {
        throw std::runtime_error("UART write incomplete");
    }
}

void UART::receive(uint8_t* buf, int len) {
    if (!isOpen_) {
        throw std::runtime_error("UART port is not open");
    }

    int received = read(fd, buf, len);
    if (received < 0) {
        throw std::system_error(errno, std::system_category(),
                              "UART read failed");
    }
    if (received == 0) {
        throw std::runtime_error("UART read timeout");
    }
}

bool UART::isOpen() {
    return isOpen_;
}
#endif