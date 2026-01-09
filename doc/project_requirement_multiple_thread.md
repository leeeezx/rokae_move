# 项目实施计划书：Rokae 机械臂 ROS 2 节点多线程实时控制重构

## 1. 项目背景与目标
**现状**：机械臂控制程序运行在 PREEMPT_RT 内核上，作为独立 ROS 2 节点。SDK 使用 `setControlLoop` 接口以 1000Hz (1ms) 频率发送指令。
**问题**：当前单线程模型下，SDK 高频回调阻塞主线程，导致 ROS 执行器无法调度，造成外部传感器 (Sensor) 和键盘 (Keyboard) 话题通讯阻塞。
**目标**：重构控制节点架构，实现 1000Hz 控制回路与 ROS 通讯回路分离。确保在 `startLoop(false)` 非阻塞模式下，控制线程与 ROS 接收线程并行运行，解决通讯阻塞。

## 2. 系统架构设计
采用 **"ROS 主线程 + SDK 后台线程 + 自旋锁共享内存"** 架构。

1.  **主线程 (Main Thread / ROS Thread)**：
    *   运行 `rclcpp::spin(node)`。
    *   负责实时接收 Sensor (400Hz) 和 Keyboard Topic 数据。
    *   收到数据后，获取自旋锁并写入共享内存。
2.  **SDK 线程 (Background RT Thread)**：
    *   由 `robot_ptr->startLoop(false)` 启动。
    *   以 1000Hz 频率执行控制回调。
    *   执行 **"Try-Lock + ZOH (零阶保持)"** 策略读取传感器数据并计算控制律。
3.  **共享内存 (Shared Memory)**：
    *   使用 `std::atomic_flag` 实现自旋锁，保护力控数据与时间戳。

## 3. 详细实施步骤

### 3.1 数据结构定义
实现基于 `std::atomic_flag` 的轻量级共享存储类，禁止使用 `std::mutex`。

*   **数据成员**：
    *   `std::array<double, 6> force_torque`: 力/力矩数据。
    *   `std::chrono::steady_clock::time_point timestamp`: 数据接收时间戳（用于看门狗）。
*   **方法**：
    *   `void update(...)`: 自旋等待锁，写入数据与当前时间。
    *   `bool try_get(...)`: 尝试获取锁 (`test_and_set`)。成功则拷贝数据并清除锁；失败则返回 false（不等待）。

### 3.2 机械臂节点类改造 (RobotNode)
在 `include/rokae_node/rokae_move_node.hpp` 和对应的 cpp 文件中修改。

1.  **新增成员**：实例化上述共享存储类对象 `sensor_data_buffer_`。
2.  **ROS 回调 (`sensor_callback`)**：
    *   接收 `geometry_msgs::msg::WrenchStamped`。
    *   调用 `sensor_data_buffer_.update()` 写入最新数据。
3.  **SDK 回调 (`control_loop`)**：
    *   **读取策略**：调用 `try_get()`。
        *   **成功**：更新本地缓存 `current_force`，检查时间戳是否超时（阈值 50ms）。若超时，触发急停保护。
        *   **失败 (锁忙) 或 无新数据**：维持 `current_force` 不变 (ZOH)。
    *   **控制计算**：基于 `current_force` 计算阻抗控制律。
    *   **约束**：禁止 `sleep`，计算耗时 < 1ms。

### 3.3 主函数 (`main`) 重构
修改 `src/rokae_move_node.cpp`：

1.  初始化 SDK 与 ROS。
2.  调用 `robot->setControlLoop(...)` 绑定回调。
3.  调用 `robot->startLoop(false)` 启动后台控制线程。
4.  调用 `rclcpp::spin(node)` 在主线程处理 ROS 消息。
5.  程序退出前调用 `stopLoop()` 和 `stopMove()`。

## 4. 关键配置要求

### 4.1 QoS 配置变更 (强制)
全链路修改为 **BEST_EFFORT** 以降低延迟。
*   **Action**: 修改 Robot (Subscriber), Sensor (Publisher), Keyboard (Publisher) 的 QoS 配置。
    *   Reliability: `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`
    *   Durability: `RMW_QOS_POLICY_DURABILITY_VOLATILE`
    *   History: `KEEP_LAST`, Depth: 1

### 4.2 编译与链接
*   确保 `CMakeLists.txt` 链接 `pthread`。

## 5. 验收测试标准
1.  **功能验证**：在机械臂运动期间，`ros2 topic hz /force_sensor_x` 显示稳定频率，且机械臂能随传感器数值变化做出实时阻抗响应。
2.  **实时性**：控制回调周期抖动 < 0.1ms。
3.  **安全性**：断开传感器连接，机械臂应在 50ms 内检测到超时并停止运动。

## 6. 附录：Sensor 节点接口说明 (补充信息)
本项目的 Force Sensor 节点（`sensor` 包）提供以下接口，Robot 控制节点需据此进行适配：

*   **发布话题 (Topics)**:
    *   `/force_sensor_x`
    *   `/force_sensor_y`
    *   `/force_sensor_z`
*   **消息类型 (Message Type)**: `geometry_msgs/msg/WrenchStamped`
    *   **数据单位**：牛顿 (N)
    *   **数据填充**：各话题仅填充对应轴的力数据。例如 `/force_sensor_x` 仅填充 `msg.wrench.force.x`，其他轴为 0。
*   **QoS 配置现状**:
    *   Sensor 节点目前使用默认 QoS 配置：`History=KEEP_LAST`, `Depth=10`, `Reliability=RELIABLE`, `Durability=VOLATILE`。
    *   **注意**：Robot 节点的订阅端应配置为 `RELIABLE` 以匹配当前 Sensor 设置，或者修改 Sensor 源码为 `BEST_EFFORT` 以优化实时性（正如第 4.1 节建议）。
*   **硬件端口映射**:
    *   X轴: `/dev/ttyUSB0`
    *   Y轴: `/dev/ttyUSB1`
    *   Z轴: `/dev/ttyUSB2`

## 7. 附录：Keyboard 节点接口说明 (补充信息)
本项目的键盘控制节点（`keyboard` 包）提供以下接口，Robot 控制节点需据此进行适配以接收实时指令：

*   **节点名称 (Node Name)**: `input_keyboard`
*   **发布话题 (Topics)**:
    *   `keystroke`
*   **消息类型 (Message Type)**: `std_msgs/msg/String`
    *   **数据含义**: 包含用户在终端按下的单个字符（如 `w`, `a`, `s`, `d` 等），用于控制机械臂运动或切换模式。
*   **架构特点**:
    *   采用 **“主线程读取 + 子线程通讯”** 的架构。主线程通过 `tty` 原始模式阻塞读取终端输入，确保按键响应的低延迟；子线程运行 `rclpy.spin` 维持 ROS 上下文。
    *   **注意**: 这是一个 Python 节点，但在 ROS 2 通讯层面与 C++ 节点兼容。
*   **QoS 配置现状**:
    *   代码中使用 `create_publisher(String, "keystroke", 10)`，即默认 QoS：`Reliability=RELIABLE`, `Durability=VOLATILE`, `Depth=10`。
    *   **适配建议**: Robot 节点的订阅端建议配置为 `RELIABLE`，以防止指令丢失。

## 8. 进度检查与更新
每次修改代码后，在此处以任务列表形式记录进度：
