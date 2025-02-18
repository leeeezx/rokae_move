# 路径测试点
## 垂直向下-水平向右
std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};  // 起始点  
std::array<double, 6> via_point = {0.4, 0.0, 0.4, M_PI, 0.0, M_PI};  // 中间点  
std::array<double, 6> end = {0.4, 0.3, 0.4, M_PI, 0.0, M_PI};  // 终点  

## 机械臂阻抗模式逻辑判断
1 验证官方的阻抗模式，对于期望力和设定轨迹的逻辑，哪个优先  
较深的垂直向下轨迹-较小的期望力  
std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};  // 起始点  
std::array<double, 6> via_point = {0.4, 0.0, 0.3, M_PI, 0.0, M_PI};  // 中间点  
desired_force=30N（期望力在test_impedance_force_node.cpp中通过rqt设置）

较浅的垂直向下轨迹-较大的期望力  
std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};  // 起始点  
std::array<double, 6> via_point = {0.4, 0.0, 0.4, M_PI, 0.0, M_PI};  // 中间点  
desired_force=50N（期望力在test_impedance_force_node.cpp中通过rqt设置）