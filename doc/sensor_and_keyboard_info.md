## 附录：Sensor 节点接口说明 (补充信息)

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

## 附录：Keyboard 节点接口说明 (补充信息)

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