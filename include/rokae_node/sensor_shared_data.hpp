#pragma once
#include <atomic>
#include <array>
#include <chrono>

struct SensorSharedData {
    std::atomic_flag lock = ATOMIC_FLAG_INIT;
    std::array<double, 6> force_torque = {0.0};
    std::chrono::steady_clock::time_point timestamp;

    // ROS Thread writes data
    void update(const std::array<double, 6>& data) {
        while (lock.test_and_set(std::memory_order_acquire)) {
            // 自旋等待
        }
        force_torque = data;
        timestamp = std::chrono::steady_clock::now();
        lock.clear(std::memory_order_release);
    }

    // SDK Thread reads data (Try-Get / Non-blocking)
    bool try_get(std::array<double, 6>& out_data, std::chrono::steady_clock::time_point& out_time) {
        if (!lock.test_and_set(std::memory_order_acquire)) {
            out_data = force_torque;
            out_time = timestamp;
            lock.clear(std::memory_order_release);
            return true;
        }
        return false; // 锁忙，直接返回
    }
};
