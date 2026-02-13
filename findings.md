# 发现与决策记录

## 需求拆解（来自用户）
- 仅将 PRD 中“现状”与“调用约束”视为真实事实。
- 其余 PRD 内容视为历史推测，需要结合代码重新评估。
- 按 planning-with-files 要求建立文件化计划，并可持续更新。

## 代码核查结论
- `usr_rt_cartesian_v_control` 仍存在阻塞等待：`while(stopManually.load())`。
- 当前 `stopManually` 是局部原子变量，不是类成员。
- 当前清理序列在控制函数末尾同步执行，不是主线程监控异步执行。
- 未发现 `status_monitor_timer_`、`monitor_loop_callback()`、`stop_control()` 等目标接口。
- 未发现 `SensorSharedData` 文件与共享内存读写实现。
- 目前传感器订阅逻辑（`z_force_subscription_`）在实现中处于注释状态。
- 当前 executor 为 `MultiThreadedExecutor`，但未显式 callback group 隔离。

## SDK 约束对齐检查
- `startReceiveRobotState()` 在 `startLoop(false)` 之前：已满足。
- `getStateData()` 主要在控制回调中调用：已满足当前实现。
- `output.setFinished()` 在回调内：已满足。
- “主线程执行 stopLoop/stopMove 清理”目前未满足。

## 技术决策
| 决策 | 理由 |
|------|------|
| 先做文档分层（事实/目标） | 防止后续改造基线错误 |
| 将“阻塞导致主线程不能 spin”改写为“阻塞回调导致同组回调饥饿” | 更符合当前 executor + 回调组实际 |
| 后续接口改造以最小侵入方式推进 | 降低对现有运动逻辑的回归风险 |

## 已识别风险
| 问题 | 影响 | 缓解思路 |
|------|------|---------|
| 局部 `stopManually` 生命周期依赖闭包引用 | 非阻塞化时可能引入悬垂访问风险 | 升级为类成员原子状态 |
| 清理时机耦合在控制函数尾部 | 非阻塞后资源回收不确定 | 引入主线程监控定时器统一回收 |
| 传感器链路未落地 | PRD 验收项无法验证 | 明确 topic + QoS + 共享数据结构 |

## 资源索引
- `doc/project_requirement_doc.md`
- `src/rokae_robot_controller.cpp`
- `src/rokae_move_node.cpp`
- `include/rokae_node/rokae_robot_controller.hpp`
- `include/rokae_node/rokae_move_node.hpp`
- `include/rokae_move/motion_control_rt.h`
- `include/rokae_move/robot.h`

## 视觉/外部检索记录
- 本轮未使用图片/PDF/网页检索。

## 方案可行性全面评估（2026-02-12）

### 一、与 PRD“现状/问题”的映射结论
| PRD 问题 | 方案对应机制 | 可行性结论 |
|------|------|------|
| `usr_rt_cartesian_v_control` 内阻塞等待导致回调长期占用 | 控制函数非阻塞返回 + 监控定时器异步收尾 | 可行，可直接消除阻塞等待路径 |
| 去掉 while 后局部变量生命周期失效风险 | 局部状态改为类成员原子变量（`is_control_running_` / `cleanup_needed_`） | 可行，可消除悬垂访问 |
| 非阻塞后函数末尾无法清理 | 主线程回调执行 `stopLoop -> stopMove -> stopReceiveRobotState` | 可行，且符合 SDK 约束 |

### 二、约束一致性检查
- `startReceiveRobotState()` 在 `startLoop` 之前：方案保持一致。
- `getStateData()` 位于控制回调内部：方案保持一致。
- `output.setFinished()` 位于控制回调内：方案保持一致。
- 清理动作不在 SDK 回调线程执行：方案明确迁移到主线程监控回调，方向正确。

### 三、落地风险与必要修正
1. 当前代码未显式 callback group 隔离；仅有 `MultiThreadedExecutor` 但无分组策略，需补齐。
2. `package.xml` 目前仅声明 `rclcpp/std_msgs`，而代码使用了 `geometry_msgs` 与参数描述类型，依赖声明需完善。
3. QoS 当前多为深度 `10` 默认配置，未落实 BEST_EFFORT + VOLATILE，需按链路统一。
4. `stop_control()` 需保证幂等与异常安全，防止监控回调重复触发清理。

### 四、最终判断
- 在补齐上述前置条件后，阶段 3~5 的既定方案可以解决 PRD 中“现状/问题”描述的核心矛盾。
- 该方案在现有代码结构上可最小侵入演进，不要求推翻现有轨迹生成与控制逻辑。

## 实施落地记录（2026-02-13）
- 已新增 `include/rokae_node/sensor_shared_data.hpp`，提供 `update/try_get`，并使用 `atomic_flag` 实现 try-lock + ZOH 读取模式。
- `RobotController` 已接入状态原子变量：`is_control_running_` 与 `cleanup_needed_`，并新增 `is_running()`、`needs_cleanup()`、`stop_control()`。
- `usr_rt_cartesian_v_control` 已移除阻塞等待，改为 `startLoop(false)` 后立即返回。
- SDK 控制回调结束路径已改为仅 `output.setFinished()` + 状态位切换，清理逻辑迁移到监控回调触发的 `stop_control()`。
- `Rokae_Move` 已新增 `status_monitor_timer_`（50ms）与 `monitor_loop_callback()`，用于检测 `!is_running && needs_cleanup` 并执行收尾。
- 传感器回调已改为 `geometry_msgs::msg::WrenchStamped`，并写入 `SensorSharedData`；控制回调中已接入 `try_get` 与 ZOH。
- 回调组已显式拆分为键盘/传感器/监控三个 `MutuallyExclusive` callback group，避免互相饥饿。
- QoS 已落地：传感器链路与实时数据发布改为 `BEST_EFFORT + VOLATILE`；键盘指令保持 `RELIABLE + VOLATILE`。
- 构建依赖已补齐：`geometry_msgs`、`rcl_interfaces`，并在 `CMakeLists.txt` 增加 `Threads::Threads` 链接。
