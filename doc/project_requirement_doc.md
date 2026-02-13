# 项目实施记录书：Rokae 机械臂 ROS 2 实时控制重构（已实现状态）

## 文档状态
- 文档类型：实现对齐文档（非方案草案）
- 更新日期：2026-02-13
- 对齐范围：当前仓库代码实现
- 验收状态：代码改造完成，实机验收待执行

## 必须严格遵守的 SDK 调用约束
以下约束继续作为后续迭代的硬性边界：

| 约束项 | 位置要求 | 当前实现状态 |
|--------|----------|--------------|
| `startReceiveRobotState()` | `startLoop(false)` 之前 | 已满足 |
| `getStateData()` | 控制 callback 内部 | 已满足 |
| `is_control_running_` | 类成员原子变量 | 已满足 |
| `output.setFinished()` | callback 内部，结束时触发 | 已满足 |
| `stopLoop -> stopMove -> stopReceiveRobotState` | 主线程监控路径触发 | 已满足 |

## 1. 重构目标与结论
### 1.1 目标
将阻塞式控制链路重构为“非阻塞启动 + 状态监控清理”模式，避免控制回调长期占用导致的回调饥饿。

### 1.2 当前结论
目标已在代码层落地，核心变化如下：
1. `usr_rt_cartesian_v_control` 已改为非阻塞返回。
2. 控制结束信号由 callback 内状态位维护，清理由监控回调执行。
3. 已引入共享传感器数据结构 `SensorSharedData`，控制回调通过 `try_get` 读取，失败采用 ZOH（保留上次值）。
4. 已显式拆分键盘/传感器/监控 callback group。

## 2. 架构与线程模型（当前实现）
采用“ROS 通信与监控 + SDK 实时控制 + 共享数据”的三层结构：

1. ROS 层：
- `MultiThreadedExecutor` 运行节点回调。
- 键盘订阅、传感器订阅、监控定时器分属不同 callback group（均为 `MutuallyExclusive`）。
- 监控定时器周期为 50ms（20Hz）。

2. SDK 控制层：
- 控制回调由 `setControlLoop` 注册并通过 `startLoop(false)` 启动。
- 轨迹结束时在 callback 内调用 `output.setFinished()` 并设置 `is_control_running_ = false`。
- callback 内不直接执行 stop 清理。

3. 数据共享层：
- 新增 `SensorSharedData`（`std::array<double,6> + timestamp + atomic_flag`）。
- 订阅回调写入 `update`。
- 控制回调使用 `try_get`，失败时使用上一个成功值（ZOH）。

## 3. 文件级实现映射
### 3.1 新增文件
- `include/rokae_node/sensor_shared_data.hpp`
  - 新增 `SensorSharedData`。
  - 提供 `update(...)` 与 `try_get(...)`。

### 3.2 控制器接口改造
- `include/rokae_node/rokae_robot_controller.hpp`
  - 构造函数增加 `SensorSharedData* shared_data`。
  - 新增成员：`is_control_running_`、`cleanup_needed_`、`stop_control_mutex_`、`shared_data_`。
  - 新增接口：`is_running()`、`needs_cleanup()`、`stop_control()`。

- `src/rokae_robot_controller.cpp`
  - `usr_rt_cartesian_v_control`：
    1. 移除阻塞 `while(stopManually)`。
    2. 启动前设置 `is_control_running_ = true`、`cleanup_needed_ = true`。
    3. `startLoop(false)` 后立即返回。
  - SDK callback：
    1. 使用 `shared_data_->try_get(...)` 读取外部传感器数据。
    2. 结束条件仅执行 `output.setFinished()` 与状态位更新。
  - `stop_control()`：
    1. 幂等检查 `cleanup_needed_`。
    2. 按顺序执行 `stopLoop -> stopMove -> stopReceiveRobotState`。
    3. 重置发布定时器并清空清理标志。

### 3.3 节点通信与监控改造
- `include/rokae_node/rokae_move_node.hpp`
  - 新增：`SensorSharedData sensor_data_`、`status_monitor_timer_`。
  - 新增回调：`sensor_callback(...)`、`monitor_loop_callback()`。
  - 新增 callback group 与 `WrenchStamped` 订阅成员。

- `src/rokae_move_node.cpp`
  - `RobotController` 构造传入 `&sensor_data_`。
  - 新建 callback group：键盘 / 传感器 / 监控。
  - `sensor_callback` 将 `WrenchStamped` 写入共享数据。
  - `monitor_loop_callback` 检测 `!is_running && needs_cleanup` 后执行 `stop_control()`。

## 4. 配置与依赖状态
### 4.1 QoS（当前实现）
- 键盘订阅：`RELIABLE + VOLATILE`。
- 传感器订阅：`BEST_EFFORT + VOLATILE`。
- 实时状态发布：`BEST_EFFORT + VOLATILE`。

### 4.2 构建依赖（已补齐）
- `CMakeLists.txt`：
  - 新增 `find_package(geometry_msgs REQUIRED)`。
  - 新增 `find_package(rcl_interfaces REQUIRED)`。
  - 新增 `find_package(Threads REQUIRED)` 并链接 `Threads::Threads`。
- `package.xml`：
  - 新增 `geometry_msgs`、`rcl_interfaces`、`tf2`、`moveit_ros_planning_interface`。

## 5. 待实机验收项（由操作者执行）
以下结果需在 Ubuntu 20.04 + ROS2 + 实机环境回填：

1. 非阻塞验证
- 步骤：
  - 先按 `v` 启动速度控制。
  - 运动过程中继续发送键盘指令（建议先按无动作键，如 `x`）。
  - 再按一次 `v` 或 `b`。
- 预期：
  - 节点终端可立即打印键盘回调日志（例如“收到键盘按下的消息---x”），证明主线程未被阻塞。
  - 当前运动不会被无动作键自动打断（符合当前实现）。
  - 再次按 `v` 或 `b` 时，出现“已有控制循环正在运行，本次启动请求已忽略”。
- 结果：`[待回填]`

2. 自动清理验证
- 步骤：让轨迹自然结束，观察节点终端与日志文件。
- 预期：
  - 在运行 `ros2 run rokae_move rokae_move_node` 的终端看到 `Control Loop Stopped & Cleaned up`。
  - 机器人停稳且无异常报错。
- 日志位置：
  - 实时输出：`rokae_move_node` 运行终端。
  - 文件日志：`~/.ros/log/latest/`。
- 结果：`[待回填]`

3. 力触发切换验证
- 步骤：运动中施加超过阈值的外力输入。
- 预期：触发轨迹切换或相应保护行为，响应无明显延迟。
- 结果：`[待回填]`

## 6. 当前限制与说明
1. 当前对接话题为 `/force_sensor_z`（`WrenchStamped`），若需三轴独立输入（`/force_sensor_x/y/z`）需补充聚合逻辑。
2. 本轮仅完成代码与文档对齐，未在当前 Windows 沙箱完成 ROS 依赖下的编译和实机测试。
3. 后续若修改 stop 逻辑，必须保持“SDK callback 不做 stop 清理”的边界。
