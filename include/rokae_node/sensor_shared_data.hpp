#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <thread>

/**
 * @brief 传感器共享数据。用于 ROS 订阅回调与 SDK 实时回调之间的数据交换。
 */
struct SensorSharedData {
    std::array<double, 6> force_torque{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::chrono::steady_clock::time_point timestamp{std::chrono::steady_clock::now()};
    std::atomic_flag lock = ATOMIC_FLAG_INIT;

    void update(const std::array<double, 6>& new_force_torque,
                const std::chrono::steady_clock::time_point new_timestamp = std::chrono::steady_clock::now()) {
        while (lock.test_and_set(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        force_torque = new_force_torque;
        timestamp = new_timestamp;
        lock.clear(std::memory_order_release);
    }

    bool try_get(std::array<double, 6>& out_force_torque,
                 std::chrono::steady_clock::time_point& out_timestamp) {
        if (lock.test_and_set(std::memory_order_acquire)) {
            return false;
        }
        out_force_torque = force_torque;
        out_timestamp = timestamp;
        lock.clear(std::memory_order_release);
        return true;
    }
};

