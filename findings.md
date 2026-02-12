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
