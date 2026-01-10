# Rokae 机械臂 ROS 2 实时控制节点 (多线程重构版)

本项目是针对 Rokae xMateErPro 机械臂开发的 ROS 2 控制节点。

**当前分支 (`refactor`) 已完成核心架构的 v2.0 重构**，采用了 **"ROS 主线程 (通讯+监控) + SDK 后台线程 (控制) + 共享内存"** 的多线程设计模式，解决了原单线程架构下的阻塞问题，实现了高频实时控制与 ROS 通讯的并发安全。

详细的重构设计与架构说明请参考：
👉 **[多线程需求与架构设计文档](doc/project_requirement_multiple_thread.md)**

## 核心特性

*   **多线程并发架构**：主线程负责 ROS 话题通讯与状态监控，后台线程负责 SDK 1kHz 实时控制。
*   **非阻塞交互**：控制循环运行时，主线程仍能实时响应键盘指令和传感器数据。
*   **共享内存通讯**：利用无锁（Lock-free）或自旋锁机制在 ROS 回调与实时控制线程间传递高频力控数据。
*   **安全状态机**：具备完善的生命周期管理，支持力阈值触发后的自动轨迹切换与安全停止。

## 环境依赖

*   **操作系统**: Linux (推荐 Ubuntu 22.04 / 20.04)
*   **ROS 2**: Humble / Foxy
*   **硬件依赖**:
    *   Rokae xMateErPro 机械臂 (需配置 IP: 192.168.0.160)
    *   六维力传感器 (发布话题 `/force_sensor_x`)
*   **第三方库**:
    *   Rokae xCore SDK (已集成在 `lib/` 和 `include/` 中)
    *   Eigen3

## 编译指南

```bash
# 进入工作空间根目录
cd <your_colcon_workspace>

# 编译项目
colcon build --packages-select rokae_move

# source 环境
source install/setup.bash
```

## 运行指南

由于涉及到实时网络通讯（EtherCAT/TCP）和线程优先级设置，建议使用 `sudo` 或配置好实时权限运行。

```bash
# 启动控制节点
ros2 run rokae_move rokae_move_node
```

### 运行时交互

节点启动后会订阅 `/keystroke` 话题接收键盘指令。推荐配合键盘发布节点使用，或手动发布消息：

```bash
ros2 topic pub --once /keystroke std_msgs/msg/String "data: 'v'"
```

## 使用方法 (目前支持的指令)

在重构阶段，为了确保安全，仅保留并验证了以下核心控制功能。按下对应键盘按键即可触发：

| 按键 | 功能描述 | 对应函数 |
| :---: | :--- | :--- |
| **`v`** | **Z轴速度控制测试**<br>执行“空中加速 -> 匀速侵入 -> 减速缓冲 -> 停止”的四阶段 Z 轴运动。 | `usr_rt_cartesian_v_control_z` |
| **`b`** | **Z-Y 复合轨迹控制**<br>先执行 Z 轴下探，接触或到位后衔接 Y 轴水平划动。支持力控触发回退（Z轴受力 > 40N 时自动抬起）。 | `usr_rt_cartesian_v_control` |

> ⚠️ **注意**：其他按键（如 `q`, `w`, `e`, `r`, `n` 等）在当前分支均处于 **禁用/维护** 状态，触发时会打印警告信息。

### 参数配置

可以通过 ROS 2 参数动态调整运动特性（单位：米, 米/秒）：

*   `air_distance` / `y_air_distance`: 加速段距离
*   `cruise_distance` / `y_cruise_distance`: 匀速段距离
*   `target_speed` / `y_target_speed`: 期望作业速度
*   `desired_force_z`: 期望恒力阈值 (目前主要用于触发判断)

## 项目结构

```text
D:\CodeProject\rokae_move\
├── archive/                # 归档的旧代码与实验性代码
├── build/                  # 编译生成目录
├── doc/                    # 项目文档
│   ├── project_requirement_multiple_thread.md  # 核心：多线程重构设计文档
│   └── ...
├── example/                # SDK 官方示例代码
├── external/               # 第三方依赖库 (Eigen 等)
├── include/                # 头文件目录
│   └── rokae_node/
│       ├── rokae_move_node.hpp         # ROS 节点类定义
│       ├── rokae_robot_controller.hpp  # 机械臂控制类定义
│       └── trajectory_generator.hpp    # 轨迹生成算法库
├── lib/                    # 编译链接库 (xCoreSDK 等)
├── src/                    # 源代码目录
│   ├── rokae_move_node.cpp             # ROS 节点实现 (主线程逻辑)
│   ├── rokae_robot_controller.cpp      # 机械臂控制实现 (SDK线程逻辑)
│   └── trajectory_generator.cpp        # 轨迹生成算法实现
├── CMakeLists.txt          # 构建脚本
├── package.xml             # ROS 包描述文件
└── README.md               # 项目说明文档
```
