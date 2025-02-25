# 珞石七轴机械臂控制系统

## 项目介绍
本项目用于控制七轴珞石机械臂，实现笛卡尔空间的运动控制、力控制和阻抗控制等功能。项目基于ROS 2开发，提供了机械臂的轨迹规划、力控制和实时控制等功能。

如果需要接收外置传感器力信息，请参考sensor仓库中的内容，或者本仓库中src/archive/test...ros.py

### 主要功能
- 笛卡尔空间轨迹规划与控制
- 力控制与阻抗控制
- 实时运动控制
- 多段轨迹生成（包括贝塞尔曲线、线性轨迹等）

## 环境要求
- Ubuntu系统（推荐Ubuntu 20.04或更高版本）
- ROS 2（项目基于ROS 2开发）
- Eigen3
- C++17标准
- 珞石机械臂SDK（xCoreSDK和xMateModel库）

## 项目结构
```
rokae_move/
├── src/                   # 源代码目录
│   ├── archive/          # 归档文件，一般是废弃的或者不再使用的程序文件
│   ├── test_impedance_force_node.cpp  # 机械臂力控制与阻抗控制实现
│   ├── trajectory_verification.cpp    # 轨迹验证程序
│   └── trajectory_verifi_plot.py      # 轨迹数据可视化脚本
├── include/               # 头文件目录
│   └── rokae_move/       # 项目头文件
│       ├── base.h        # 基础定义
│       ├── data_types.h  # 数据类型定义
│       ├── exception.h   # 异常处理
│       ├── model.h       # 机械臂模型
│       ├── motion_control_rt.h  # 实时运动控制
│       ├── planner.h     # 路径规划
│       ├── print_helper.hpp  # 打印辅助函数
│       ├── robot.h       # 机械臂控制接口
│       └── utility.h     # 工具函数
├── example/               # 示例代码目录
│   ├── cartesian_impedance_control.cpp  # 笛卡尔空间阻抗控制示例
│   ├── cartesian_s_line.cpp             # 笛卡尔空间直线轨迹示例
│   └── trajectoryPoint.md               # 轨迹点使用说明
├── lib/                   # 库文件目录
├── external/              # 外部依赖目录
├── build/                 # 构建目录
├── CMakeLists.txt         # CMake构建配置
└── package.xml            # ROS 2包信息
```

## 编译与安装
1. 确保已安装所有依赖项
2. 克隆本项目到ROS 2工作空间的src目录下
3. 返回工作空间根目录，执行编译命令：
   ```bash
   colcon build --packages-select rokae_move
   ```
4. 编译完成后，请先加载环境变量：
   ```bash
   source install/setup.bash
   ```

## 使用方法
### 运行力控与阻抗控制节点
```bash
ros2 run rokae_move test_impedance_force_node
```

### 轨迹测试
项目提供了多种轨迹规划方法，可参考`example/trajectoryPoint.md`进行测试，包括：
- 垂直向下-水平向右的轨迹
- 不同深度和期望力的组合测试

## 主要接口
项目提供的主要功能接口包括：
- 笛卡尔空间运动控制
- 力控制
- 阻抗控制
- 实时轨迹生成

具体接口使用方法请参考源代码注释和示例。

## 归档内容
archive目录包含了历史版本的代码或不再使用的程序文件，仅供参考，不建议在实际应用中使用。

## 许可证
待定

## 贡献者
- 乐正祥（主要开发者）