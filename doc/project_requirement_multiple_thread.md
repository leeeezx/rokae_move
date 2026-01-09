# 项目实施计划书：Rokae 机械臂 ROS 2 节点多线程实时控制重构

## 必须严格遵守的要求
### SDK函数调用约束
**重构时必须严格遵守以下 SDK 嵌套规则，禁止变更位置：**

| 约束项 | 位置要求 | 说明 |
|--------|----------|------|
| `startReceiveRobotState()` | `startLoop` 之前 | 参数需预先配置 |
| `getStateData()` | callback **内部** | 仅在控制回调中调用 |
| `is_control_running_` (原子标志位) | **类成员变量** | **严禁使用局部变量**。用于跨线程同步状态。 |
| `output.setFinished()` | callback 内部，轨迹结束时 | 标记轨迹结束，随后需手动设置 `is_control_running_ = false` |
| 退出清理序列 (`stopLoop` -> `stopMove`) | **主线程**定时器回调中 | **严禁在 SDK 回调线程中直接调用**。必须通过状态监测异步执行。 |

## 进度检查与更新
每次修改代码后，在此处以任务列表形式记录进度：
- [x] 3.1 创建 SensorSharedData
- [x] 3.4 修改 RobotController 头文件
- [x] 3.2 修改 Rokae_Move 头文件
- [x] 3.5 重构 RobotController 源文件 (核心逻辑)
- [x] 3.3 修改 Rokae_Move 源文件 (连接逻辑)

## 1. 项目背景与目标
**现状**：当前控制逻辑中，`usr_rt_cartesian_v_control` 函数内包含 `while(stopManually.load())` 死循环，导致主线程阻塞，无法响应 ROS 话题（传感器、键盘）。
**问题**：
1.  **阻塞问题**：主线程无法执行 `rclcpp::spin()`，无法与传感器ROS2包的外部传感器数据topic进行通讯。
2.  **生命周期漏洞**：若直接去除 `while`，局部变量 `stopManually` 会被销毁，导致 SDK 回调访问非法内存（Segfault）。
3.  **清理时机**：非阻塞模式下，函数立即返回，无法在函数末尾直接执行清理。

**目标**：重构为 **"非阻塞启动 + 状态机监控"** 模式。确保控制线程独立运行，主线程负责 ROS 通讯和生命周期管理。

## 2. 系统架构设计
采用 **"ROS 主线程 (通讯+监控) + SDK 后台线程 (控制) + 共享内存"** 架构。

1.  **主线程 (Main Thread)**：
    *   **通讯**：运行 `rclcpp::spin()`，实时接收 Sensor (400Hz) 和 Keyboard Topic。
    *   **写入**：收到传感器数据后，写入无锁共享内存。
    *   **监控**：通过低频定时器 (e.g., 20Hz) 监控 SDK 线程状态。若发现运动结束，执行清理序列。
2.  **SDK 线程 (Background RT Thread)**：
    *   以 1000Hz 频率执行控制回调。
    *   使用 **"Try-Get"** 策略读取共享内存中的传感器数据。
    *   运动结束时，仅标记状态位，**不执行**清理函数。
3.  **数据共享**：
    *   使用 `std::atomic_flag` 实现自旋锁，保护力控数据。

## 3. 详细实施步骤 (按文件划分)

### 3.1 新增：`include/rokae_node/sensor_shared_data.hpp`
**任务**：定义线程安全的共享数据结构。
*   **内容**：
    *   结构体 `SensorSharedData`。
    *   成员：`std::array<double, 6> force_torque`，`std::chrono::steady_clock::time_point timestamp`，`std::atomic_flag lock`。
    *   方法 `update(...)`: 自旋获取锁 -> 写入数据 -> 释放锁。
    *   方法 `try_get(...)`: `test_and_set` 尝试获取锁 -> 成功则拷贝并返回 true -> 失败返回 false (实现 ZOH 零阶保持)。

### 3.2 修改：`include/rokae_node/rokae_move_node.hpp`
**任务**：增加监控定时器和共享数据对象。
*   **新增成员**：
    *   `SensorSharedData sensor_data_;` (实例化共享数据)
    *   `rclcpp::TimerBase::SharedPtr status_monitor_timer_;` (状态监控定时器)
*   **修改函数**：
    *   将 `z_force_callback` 替换为 `void sensor_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);`
    *   新增 `void monitor_loop_callback();` (用于检测机器人是否需要清理)

### 3.3 修改：`src/rokae_move_node.cpp`
**任务**：实现 ROS 数据转发与状态监控。
*   **构造函数**：
    *   初始化 `RobotController` 时，将 `&sensor_data_` 指针传递给它。
    *   创建 `status_monitor_timer_` (建议 20Hz/50ms)，绑定到 `monitor_loop_callback`。
    *   订阅 `/force_sensor_x` 等话题，绑定到 `sensor_callback`。
*   **`sensor_callback`**：
    *   解析 ROS 消息。
    *   调用 `sensor_data_.update(...)` 写入最新数据。
*   **`monitor_loop_callback`**：
    *   检查 `robot_controller_->is_running()` 是否为 `false`。
    *   检查 `robot_controller_->needs_cleanup()` 是否为 `true`（防止重复清理）。
    *   若满足条件：调用 `robot_controller_->stop_control()` 并重置标志位。

### 3.4 修改：`include/rokae_node/rokae_robot_controller.hpp`
**任务**：提升状态标志位为成员变量，提供清理接口。
*   **新增成员**：
    *   `SensorSharedData* shared_data_;` (持有共享数据指针)
    *   `std::atomic<bool> is_control_running_{false};` (替代原局部变量 `stopManually`)
    *   `std::atomic<bool> cleanup_needed_{false};` (通知主线程进行清理)
*   **新增/修改接口**：
    *   构造函数接收 `SensorSharedData*`。
    *   `bool is_running() const`。
    *   `bool needs_cleanup() const`。
    *   `void stop_control()` (封装 stopLoop, stopMove, stopReceiveRobotState)。

### 3.5 修改：`src/rokae_robot_controller.cpp`
**任务**：重构控制函数为非阻塞模式。
*   **`usr_rt_cartesian_v_control` 重构**：
    1.  **移除** `while(stopManually.load())` 及其后的清理代码。
    2.  设置 `is_control_running_ = true` 和 `cleanup_needed_ = true`。
    3.  调用 `rtCon_->startLoop(false)`。
    4.  **立即 return** (非阻塞)。
*   **SDK 回调 (Lambda) 修改**：
    1.  **数据读取**：使用 `shared_data_->try_get(...)` 替换原有逻辑。
    2.  **退出条件**：当轨迹结束 (`index >= size`) 时：
        *   执行 `output.setFinished()`。
        *   设置 `is_control_running_ = false`。
        *   **绝对不要**在这里调用 `stopLoop`。
*   **新增 `stop_control` 实现**：
    1.  执行 `rtCon_->stopLoop()`。
    2.  执行 `rtCon_->stopMove()`。
    3.  执行 `robot_->stopReceiveRobotState()`。
    4.  重置相关的 Timer (如果有)。
    5.  打印 "Control Loop Stopped & Cleaned up"。

## 4. 关键配置要求
*   **QoS**: 全链路 (Sensor -> Robot) 使用 **BEST_EFFORT** + **VOLATILE**。
*   **编译**: 确保 CMake 链接 `pthread`。

## 5. 验收测试标准
1.  **非阻塞验证**：启动控制后，主线程仍能响应键盘 `keystroke` 话题，且终端不卡死。
2.  **清理验证**：轨迹运行结束后，程序应自动打印 "Control Loop Stopped & Cleaned up"，且机械臂停止在目标位置，无报错。
3.  **实时响应**：在运动过程中，人为触发传感器力阈值，机械臂应能立即响应（切换轨迹或急停）。

