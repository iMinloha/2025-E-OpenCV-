#ifndef _UTD_H
#define _UTD_H

#include <thread>
#include <iostream>
#include <atomic>
#include <queue>
#include <unordered_map>
#include <mutex>
#include <condition_variable>
#include <utility>
#include <stdexcept>
#include <future>
#include <optional>
#include "opencv2/opencv.hpp"

typedef std::function<void()> FuncHandle;

// 一个简单的线程类
class UTD {
public:
    explicit UTD(FuncHandle func) : func_(std::move(func)), isRunning_(false), isPaused_(false) {}
    ~UTD();
    void Run();             // 同步运行
    void AsyncRun();        // 异步运行
    void Stop();            // 停止
    void Suspend();         // 暂停
    void Resume();          // 恢复
    static void osDelay(int seconds);
    static void msDelay(int ms);

private:
    FuncHandle func_;
    void ThreadFunc();
    std::thread thread_;            // 线程对象
    std::thread funcThread_;
    std::atomic<bool> isRunning_{};   // 线程运行状态
    std::atomic<bool> isPaused_{};    // 线程暂停状态
    std::promise<void> promise_;
    std::future<void> future_;
};

// 消息队列
template <typename T>
class MessageQueue {
public:
    // 构造函数
    MessageQueue() = default;

    // 析构函数
    ~MessageQueue() = default;

    // 添加消息
    void push(const std::string& name, const T& message);

    // 非阻塞获取消息（立即返回默认值）
    T pop(const std::string& name, const T& defaultValue);

    // 非阻塞获取消息（返回 std::optional）
    T pop(const std::string& name);

    // 非阻塞获取消息（带超时）
    std::optional<T> pop(const std::string& name, const std::chrono::milliseconds& timeout);

    // 检查队列是否为空
    bool empty(const std::string& name) const;

    // 获取队列大小
    size_t size(const std::string& name) const;

private:
    // 消息队列数据结构
    std::unordered_map<std::string, std::queue<T>> queues_;

    // 互斥锁和条件变量
    mutable std::mutex mutex_;
    std::condition_variable cv_;
};


// 常用模板声明
template class MessageQueue<int>;
//template class MessageQueue<std::string>;
template class MessageQueue<cv::Mat>;
template class MessageQueue<cv::Point2f>;

// 整数消息
extern MessageQueue<int> MSQ_INT;
// 图像消息
extern MessageQueue<cv::Mat> MSQ_MAT;
// 目标坐标
extern MessageQueue<cv::Point2f> MSQ_AIM;


#endif
