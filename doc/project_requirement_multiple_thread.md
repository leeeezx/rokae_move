# 项目实施计划书：Rokae 机械臂 ROS 2 节点多线程实时控制重构

## 1. 项目背景与目标
**当前现状**：
目前机械臂控制程序运行在实时内核（PREEMPT_RT）上，作为一个独立的 ROS 2 节点。该节点使用官方 SDK 的 `setControlLoop` 接口以 1000Hz（1ms周期）的高频回调callback向机械臂发送指令。@src\rokae_robot_controller.cpp @src\rokae_robot_node.cpp
**遇到问题**：
由于 ROS 2 节点默认是单线程模型，当 SDK 的高频控制回调占用主线程时，导致节点内的 ROS 执行器（Executor）无法获得 CPU 时间片，从而无法处理外部 Sensor 节点和键盘节点发布的话题消息（Topic），造成通讯阻塞。例如将传感器数据传到callback中。
**重构目标**：
在**不修改 Sensor 包和键盘包源码**的前提下，对机械臂控制节点进行**多线程架构改造**。实现控制回路（1000Hz）与通讯回路（ROS Spin）的分离，确保在进行高频实时控制的同时，能够实时接收外部传感器数据。

## 2. 系统架构设计
我们将采用 **“双线程 + 共享内存”** 的架构模式：

1.  **线程 A（通讯线程 - ROS Thread）**：
    *   **职责**：负责运行 `rclcpp::spin()`。
    *   **功能**：实时接收 Sensor（力传感器）和 Keyboard（指令）的 Topic 数据。
    *   **行为**：收到数据后，立即将其更新到“共享数据区”。
2.  **线程 B（控制线程 - SDK Thread）**：
    *   **职责**：由 SDK 的 `setLoop` 驱动，运行在实时优先级。
    *   **功能**：执行 1000Hz 控制律计算。
    *   **行为**：每个周期从“共享数据区”读取最新的传感器数据，计算控制量，调用 SDK 发送指令。
3.  **共享数据区（Shared Data）**：
    *   作为两个线程的桥梁，需具备线程安全性（Thread-safe）。

## 3. 详细实施步骤与代码规范

### 3.1 数据结构定义
定义一个用于存储传感器数据的结构体，并使用线程安全机制保护。

*   **要求**：鉴于 1000Hz 的实时性要求，优先考虑 `std::atomic`（如果是简单数据）或 `std::mutex`（需注意锁的开销，尽量使用 `try_lock` 或轻量级锁）。

```cpp
// 示例结构
struct SharedSensorData {
    std::vector<double> force_torque; // 传感器数值
    bool is_updated;                  // 数据更新标志
};
```

### 3.2 机械臂节点类改造 (RobotNode)
在类中区分 ROS 回调与 SDK 回调。

*   **成员变量**：包含 ROS 订阅者、共享数据结构、互斥锁。
*   **ROS 回调函数 (`sensor_callback`)**：
    *   仅负责将 `msg` 数据写入共享结构体。
    *   **禁止**在此进行耗时的控制计算。
*   **SDK 回调函数 (`control_loop_callback`)**：
    *   **第一步**：加锁/原子操作，快速读取共享结构体中的数据到本地变量。
    *   **第二步**：基于本地变量执行控制算法（阻抗/力控等）。
    *   **第三步**：调用 SDK 发送指令。
    *   **严格约束**：**绝对禁止使用 `sleep`**，执行总耗时必须严格小于 1ms。

### 3.3 主函数 (`main`) 重构
这是本次修改的核心，需要显式分离线程。

```cpp
// 伪代码逻辑
int main(int argc, char ** argv) {
    // 1. 初始化 ROS 和 SDK
    rclcpp::init(argc, argv);
    RokaeSDK::init();
    auto node = std::make_shared<RobotNode>();

    // 2. 启动通讯线程 (Thread A)
    // 使用 std::thread 运行 ROS 执行器
    std::thread communication_thread([node]() {
        rclcpp::spin(node); 
    });

    // 3. 配置 SDK 回调并启动控制 (Thread B)
    // 绑定 node 中的控制函数
    RokaeSDK::setLoop(std::bind(&RobotNode::control_loop_callback, node, ...));
    RokaeSDK::start(); // 这通常会阻塞或开启内部循环

    // 4. 资源清理与等待
    if (communication_thread.joinable()) {
        communication_thread.join();
    }
    rclcpp::shutdown();
    return 0;
}
```

## 4. 关键配置要求 (Checklist)

### 4.1 QoS 配置 (必须检查)
为了保证实时性，防止 TCP 重传导致的延迟，必须在代码中显式配置 QoS。

*   **Robot 节点订阅端 (Subscriber)**：
    *   Reliability: `RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT`
    *   Durability: `RMW_QOS_POLICY_DURABILITY_VOLATILE`
*   **Sensor 节点发布端 (Publisher)**：
    *   **注意**：虽然不改代码，但需确认 Sensor 包启动文件或参数中是否允许配置 QoS。如果 Sensor 包默认是 Reliable，Robot 端最好也兼容匹配，或者建议修改 Sensor 的 QoS 配置以降低延迟。

### 4.2 线程优先级 (建议)
在实时内核下，为了防止 ROS 通讯抢占控制线程：

*   **控制线程 (Thread B/SDK)**：建议设置调度策略为 `SCHED_FIFO`，优先级 `90`（需 root 权限或 limits 配置）。
*   **通讯线程 (Thread A/ROS)**：保持默认 `SCHED_OTHER` 或较低优先级的 `SCHED_FIFO` (如 `50`)。

### 4.3 编译依赖
确保 `CMakeLists.txt` 中链接了 `pthread` 库（通常 Linux 环境默认支持，但需检查）。

## 5. 验收测试标准

1.  **通讯测试**：在机械臂运动（1000Hz 回调运行中）时，使用 `ros2 topic echo` 监控 Sensor 数据，数据应流畅无卡顿。
2.  **实时性测试**：在控制回调中打印时间戳（或记录日志），相邻两次回调的时间差应稳定在 1ms ± 0.1ms，不应出现因等待锁而导致的超时（Overrun）。
3.  **功能验证**：Sensor 数据变化应能实时反映在机械臂的动作上（如力控拖动）。

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
