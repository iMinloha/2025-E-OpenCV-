#include "UTD.h"

UTD::~UTD(){
    Stop();
}

void UTD::Run() {
    if (func_) func_();
}

void UTD::AsyncRun() {
    if (isRunning_) {
        std::cerr << "Thread is already running." << std::endl;
        return;
    }
    isRunning_ = true;
    isPaused_ = false;
    promise_ = std::promise<void>();
    future_ = promise_.get_future();
    thread_ = std::thread(&UTD::ThreadFunc, this);
}

void UTD::Stop() {
    if (isRunning_) {
        isRunning_ = false;
        isPaused_ = false;
        promise_.set_value();
        if (thread_.joinable()) thread_.join();
    }
}

void UTD::Suspend() {
    if (isRunning_ && !isPaused_) isPaused_ = true;
}

// 恢复线程
void UTD::Resume() {
    if (isRunning_ && isPaused_) isPaused_ = false;
}

void UTD::ThreadFunc() {
    while (isRunning_) {
        if (!isPaused_) {
            // 启动 func_ 线程
            funcThread_ = std::thread(func_);
            // 定期检查终止信号
            while (isRunning_ && !isPaused_) {
                if (future_.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
                    isRunning_ = false; // 收到终止信号
                    funcThread_.detach(); // 分离线程，避免阻塞
                    break;
                }
            }
        } else std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 暂停时休眠
    }
}

void UTD::osDelay(int seconds) {
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}


void UTD::msDelay(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}



// -----------------------------------------

template <typename T>
void MessageQueue<T>::push(const std::string& name, const T& message) {
    std::unique_lock<std::mutex> lock(mutex_);
    queues_[name].push(message);
    cv_.notify_one(); // 通知等待的线程
}

// 阻塞式
template <typename T>
T MessageQueue<T>::pop(const std::string& name) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this, &name] {
        auto it = queues_.find(name);
        return it != queues_.end() && !it->second.empty();
    });
    T message = queues_[name].front();
    queues_[name].pop();
    return message;
}

// 获取消息
template <typename T>
T MessageQueue<T>::pop(const std::string& name, const T& defaultValue) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto it = queues_.find(name);
    if (it == queues_.end() || it->second.empty()) return defaultValue;
    T message = it->second.front();
    it->second.pop();
    return message;
}


// 延迟阻断
template <typename T>
std::optional<T> MessageQueue<T>::pop(const std::string& name, const std::chrono::milliseconds& timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    auto it = queues_.find(name);
    if (it == queues_.end()) return std::nullopt; // 队列不存在，返回空值
    // 等待队列不为空或超时
    if (cv_.wait_for(lock, timeout, [this, &name] {
        auto it = queues_.find(name);
        return it != queues_.end() && !it->second.empty();
    })) {
        T message = it->second.front();
        it->second.pop();
        return message;
    }
    return std::nullopt; // 超时，返回空值
}

// 检查队列是否为空
template <typename T>
bool MessageQueue<T>::empty(const std::string& name) const {
    std::unique_lock<std::mutex> lock(mutex_);
    auto it = queues_.find(name);
    if (it == queues_.end()) return true;
    return it->second.empty();
}


template <typename T>
size_t MessageQueue<T>::size(const std::string& name) const {
    std::unique_lock<std::mutex> lock(mutex_);
    auto it = queues_.find(name);
    if (it == queues_.end()) return 0;
    return it->second.size();
}


// 整数消息(测试用)
MessageQueue<int> MSQ_INT;
// 图像消息
MessageQueue<cv::Mat> MSQ_MAT;
// 坐标信息
MessageQueue<cv::Point2f> MSQ_AIM;