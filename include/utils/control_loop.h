#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

/**
 * 固定频率控制线程（header-only）。
 *
 * 使用方式：
 * @code
 *   ControlLoop loop(1000.0, [&]() -> bool {
 *       arm.set_all_positions(targets);
 *       return true;  // 返回 false 则自动停止
 *   });
 *   // ... 规划/输入更新 targets ...
 *   loop.stop();  // 或析构时自动停止
 * @endcode
 *
 * 时序策略：
 *   - 每次迭代计算下一个唤醒时间点 = 上次时间点 + period，
 *     若 callback 执行时间超过 period 则跳过 sleep（最大吞吐模式）。
 *   - pause() / resume() 通过原子标志让线程跳过 callback，
 *     用于 enable/disable 等需要独占 CAN 总线的操作前后。
 *
 * 实际频率：受 callback 执行时间（CAN 往返延迟）限制，
 * freq_hz 仅作为上限目标。
 */
class ControlLoop {
public:
    /**
     * 构造并立即启动控制线程。
     * @param freq_hz   目标频率 [Hz]，实际频率受 callback 耗时限制
     * @param callback  每次迭代调用的函数；返回 false 则线程自动终止
     */
    ControlLoop(double freq_hz, std::function<bool()> callback)
        : running_(true), paused_(false)
    {
        using namespace std::chrono;
        auto period = duration_cast<nanoseconds>(duration<double>(1.0 / freq_hz));

        thread_ = std::thread([this, period, cb = std::move(callback)]() {
            auto next = std::chrono::steady_clock::now();
            while (running_.load(std::memory_order_relaxed)) {
                if (!paused_.load(std::memory_order_relaxed)) {
                    if (!cb()) {
                        running_.store(false, std::memory_order_relaxed);
                        return;
                    }
                }
                next += period;
                std::this_thread::sleep_until(next);
            }
        });
    }

    ~ControlLoop() { stop(); }

    ControlLoop(const ControlLoop&)            = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    /**
     * 原子暂停：线程继续运行但跳过 callback，CAN 总线空闲。
     * 在 enable() / disable() / set_zero_position() 前调用。
     */
    void pause()  { paused_.store(true,  std::memory_order_relaxed); }

    /** 恢复 callback 执行。 */
    void resume() { paused_.store(false, std::memory_order_relaxed); }

    /** 停止线程并等待其退出。析构时自动调用，可手动提前调用。 */
    void stop() {
        running_.store(false, std::memory_order_relaxed);
        if (thread_.joinable()) thread_.join();
    }

    /** 当前是否处于暂停状态。 */
    bool is_paused() const { return paused_.load(std::memory_order_relaxed); }

    /** 当前是否仍在运行。 */
    bool is_running() const { return running_.load(std::memory_order_relaxed); }

private:
    std::atomic<bool> running_;
    std::atomic<bool> paused_;
    std::thread       thread_;
};
