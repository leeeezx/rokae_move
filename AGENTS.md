# 项目级别说明
## 上下文记忆
读取 task_plan.md、findings.md 和 progress.md 来恢复上下文。从第 [X] 阶段继续。
## 项目背景
本项目用于控制 rokae ER7 pro（七自由度协作机械臂），程序运行环境为 Ubuntu 20.04 的实时内核，请在涉及部署、时序或系统配置的说明中考虑实时性要求。

本项目中，rokae 机器人的官方 SDK 均位于 `D:\CodeProject\rokae_move\include\rokae_move` 文件夹中，其中包含 API 接口以及对应说明注释。请在阅读与调用相关接口时优先参考该目录下的头文件注释。

此外，项目中对官方 SDK（C++ API）对象的命名存在不同上下文的差异：在 `src/rokae_robot_controller.cpp` 的 `RobotController` 类中，这些 SDK 对象以成员变量形式保存，命名为 `robot_` 与 `rtCon_`；而在 `src/rokae_move_node.cpp` 中，同一类对象通常以局部变量或参数形式出现，命名为 `robot` 与 `rtCon`。请在阅读与编写代码时注意这只是命名与作用域的区别，指代的是同一套 SDK 接口对象。


