#include "rokae_node/rokae_robot_controller.hpp"
#include "rokae_node/rokae_move_node.hpp"      
#include "rokae_node/trajectory_generator.hpp" 
#include "rokae_move/print_helper.hpp"         

#include <iostream>

using namespace rokae;
using namespace std;

/**
 * @brief 构造函数。构造函数的函数体可以为空
 * @param robot 指向机械臂对象的共享指针
 * @param rtCon 指向实时运动控制对象的共享指针
 * @param node 指向ROS节点的指针，用于日志输出
 */
RobotController::RobotController(std::shared_ptr<xMateErProRobot> robot,
                                 std::shared_ptr<RtMotionControlCobot<7U>> rtCon,
                                 Rokae_Move* node)
    : robot_(robot), rtCon_(rtCon), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "RobotController初始化完成");
}

RobotController::~RobotController() {
    RCLCPP_INFO(node_->get_logger(), "RobotController is being destroyed.");
}


/**
 * @brief 实时模式纯轨迹控制，速度可控
 * @param ari_dist 空中加速段距离
 * @param cruise_dist 匀速侵入段距离
 * @param decel_dist 减速缓冲段距离
 * @param target_speed 期望侵入速度
 * @param y_air_dist y轴空中加速段距离（复合轨迹使用）
 * @param y_cruise_dist y轴匀速侵入段距离（复合轨迹使用）
 * @param y_decel_dist y轴减速缓冲段距离（复合轨迹使用）
 * @param y_target_speed y轴期望侵入速度（复合轨迹使用）
 * @param y_direction y轴运动方向，1为正方向，-1为负方向（复合轨迹使用）
 */
void RobotController::usr_rt_cartesian_v_control(
    double z_air_dist, double z_cruise_dist, double z_decel_dist, double z_target_speed,
    double y_air_dist, double y_cruise_dist, double y_decel_dist, 
    double y_target_speed, int y_direction)
    {
        try {
            // 参数校验，传入的距离和速度必须大于等于0
            const double EPSILON = 1e-10;
            if(z_air_dist <= EPSILON || z_cruise_dist <= EPSILON || z_decel_dist <= EPSILON || z_target_speed <= EPSILON) {
                RCLCPP_ERROR(node_->get_logger(), "无效的z轴传入参数，立即检查速度控制的传入参数!");
                return;
            }

            // ============ 重置状态变量 ============
            trajectory_state_.store(TrajectoryState::INITIAL_TRAJECTORY);  // 重置轨迹状态

            // 停止定时器发布
            node_->pose_timer_->cancel();
            node_->extTau_timer_->cancel();
            node_->poseAndextTau_timer_->cancel();

            CartesianPosition reset_start, target;
            Utils::postureToTransArray(robot_->posture(rokae::CoordinateType::flangeInBase, ec_), reset_start.pos); // 当前位姿
            std::array<double, 6UL> init_point = {0.45, 0.0, 0.45, 3.14154, 0.0, 3.14154};
            Utils::postureToTransArray(init_point, target.pos); // 将初始位姿设置为目标位姿
            rtCon_->MoveL(0.05, reset_start, target);
            RCLCPP_INFO(node_->get_logger(), "一阶段完成");

            // 设置需要接收的机器人状态数据
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m, 
                                                RtSupportedFields::tcpPose_m,
                                                RtSupportedFields::tau_m,
                                                RtSupportedFields::tauExt_inBase, //< 基坐标系中外部力矩 [Nm] - Array6D
                                                RtSupportedFields::tauExt_inStiff,//< 力控坐标系中外部力矩 [Nm] - Array6D
                                            }; 
            robot_->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 启动接收机器人数据。1ms采样周期
            
            // 获取当前位置
            std::array<double, 6UL> start = robot_->posture(rokae::CoordinateType::flangeInBase, ec_);
            print(std::cout, "当前初始位置：", start);
            
            std::vector<std::array<double, 6>> trajectory;
            // 判断是否为复合轨迹
            bool is_composite = (y_air_dist > EPSILON && y_cruise_dist > EPSILON && 
                                y_decel_dist > EPSILON && y_target_speed > EPSILON);
            // 贝塞尔轨迹预生成
            if(is_composite) {
                // 生成复合轨迹
                trajectory = TrajectoryGenerator::generateZYCompositePath(
                    start, 
                    z_air_dist, z_cruise_dist, z_decel_dist, z_target_speed,
                    y_air_dist, y_cruise_dist, y_decel_dist, y_target_speed, y_direction
            );
            RCLCPP_INFO(node_->get_logger(), "已生成Z-Y复合轨迹，共%d个点", (int)trajectory.size());
            } else {
                // 仅生成Z轴轨迹
                trajectory = TrajectoryGenerator::generateFourPhasePath(
                    start, z_air_dist, z_cruise_dist, z_decel_dist, z_target_speed
            );
            RCLCPP_INFO(node_->get_logger(), "已生成Z轴轨迹，共%d个点", (int)trajectory.size());
            }

            // 启动笛卡尔空间位置控制模式
            rtCon_->startMove(RtControllerMode::cartesianPosition);
            
            std::atomic<bool> stopManually{true}; // 停止标志。厂家示例中出现的标准形式。如果为false就会跳出while循环，然后执行一系列停止命令
            int index = 0; // 轨迹1的索引

            double time = 0.0; // 轨迹2的计算频率数值
            std::array<double, 6> tra2_start_pose; // 轨迹2的实时起点 ,array6D
            std::array<double, 16> tra2_start_pose_m; //轨迹2的实时起点，array16D行优先矩阵
            bool tra2_init = false; // 轨迹2的起点初始化标志.默认为false，如果初始化成功则为true

            const double total_lift = 0.20; // 轨迹2的上升总距离，单位m。
            const double lift_duration_time = 16.0; // 轨迹2的上升持续时间,单位s。

            // 时间触发。备用
            // int callback_count = 0;
            // const int trigger_count = 15000; // 假设1ms周期,5000次 = 5秒
            // bool time_triggered = false; // 防止重复触发

            const double FORCE_THRESHOLD = 40.0; // 力触发阈值，单位：N

            // ------------------------------------------------机械臂控制循环回调函数--------------------------------------------------------
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{}; // CartesianPosition output{}是给output进行类型定义
                
                robot_->getStateData(RtSupportedFields::tauExt_inBase, latest_current_ext_tau_base_);
                
                // 时间触发检测。测试有效，但是触发后切换较慢。保留作参考/备用
                // callback_count++;
                // if (!time_triggered && callback_count >= trigger_count) {
                //     force_trigger_.store(true);
                //     time_triggered = true;
                //     RCLCPP_INFO(node_->get_logger(), "计时器触发: %d次回调", callback_count);
                // }
                // RCLCPP_INFO(node_->get_logger(), "计时器检测完毕2");
                

                // 检测：力阈值触发标志 与 轨迹状态。当力触发状态为真且当前轨迹为初始轨迹时，
                if (latest_current_ext_tau_base_[2] > FORCE_THRESHOLD && trajectory_state_.load() == TrajectoryState::INITIAL_TRAJECTORY){ 
                    RCLCPP_INFO(node_->get_logger(), "======================达到力阈值，触发轨迹切换请求,运行新轨迹======================");
                    
                    trajectory_state_.store(TrajectoryState::TRAJECTORY_2); // 切换轨迹状态为轨迹2

                    if (index > 0 && index < trajectory.size()) {
                        // ！！！ 使用上一个周期的目标点作为起点，确保连续性 ！！！
                        tra2_start_pose = trajectory[index - 1]; 
                        Utils::postureToTransArray(tra2_start_pose, tra2_start_pose_m); // 需要一个将array6d转为array16d的工具函数
                        tra2_init = true; 
                        time = 0.0;
                        RCLCPP_INFO(node_->get_logger(), 
                                "轨迹2起点 (从轨迹1继承): [%.3f, %.3f, %.3f]",
                                index - 1,
                                tra2_start_pose[0], tra2_start_pose[1], tra2_start_pose[2]);
                    } else {
                        // 如果索引无效，则切换失败
                        tra2_init = false;
                        RCLCPP_ERROR(node_->get_logger(), "无效的轨迹索引，无法切换轨迹！");
                        output.setFinished();
                        stopManually.store(false);
                    }
                }

                if (trajectory_state_.load() == TrajectoryState::INITIAL_TRAJECTORY) {
                    // ==================================初始轨迹==================================
                    if (index < int(trajectory.size())) {
                        auto target_pose = trajectory[index]; // 获取目标轨迹点。这里的target_pose是当前的理论轨迹规划点
                        Utils::postureToTransArray(target_pose, output.pos); // 把target_pose转换成output.pos格式（16位矩阵）
                        
                        // 获取实时位姿
                        std::array<double, 6> current_pose;
                        std::array<double, 7> current_tau_m;
                        std::array<double, 6> current_ext_tau_base;
                        std::array<double, 6> current_ext_tau_stiff;
                        if(robot_->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0 &&
                            robot_->getStateData(RtSupportedFields::tau_m, current_tau_m) == 0 &&
                            robot_->getStateData(RtSupportedFields::tauExt_inBase, current_ext_tau_base) == 0 &&
                            robot_->getStateData(RtSupportedFields::tauExt_inStiff, current_ext_tau_stiff) == 0) {
                            // 更新最新位姿数据
                            std::lock_guard<std::mutex> lock(pose_data_mutex_);
                            latest_current_pose_ = current_pose; // 机械臂末端实际位姿
                            // latest_target_pose_ = target_pose; // 机械臂末端目标位姿。本地计算轨迹(需要放置在上方轨迹赋值内部)，非控制器计算轨迹。如需控制器计算轨迹，另调用tcpPose_c？不确定

                            latest_current_tau_m_ = current_tau_m; // 关节力矩

                            latest_current_ext_tau_base_ = current_ext_tau_base; // 机械臂基坐标系中外部力-力矩
                            latest_current_ext_tau_stiff_ = current_ext_tau_stiff; // 机械臂基坐标系中外部力-力矩

                            node_->publish_realtime_poseAndextTau(latest_current_pose_, latest_current_ext_tau_base_);
                        }

                        index++;
                    } else {
                        output.setFinished();
                        stopManually.store(false);
                    }
                }else{
                    try{
                        // ================================== 轨迹2 ==================================
                        if (!tra2_init) {
                        RCLCPP_ERROR(node_->get_logger(), "轨迹2未初始化!");
                        output.setFinished();
                        stopManually.store(false);
                        return output;
                        } 
    
                        // 轨迹2方案：直线上升一段距离
                        double t_norm = time / lift_duration_time;  // 归一化 [0,1]
                        
                        // 余弦函数轨迹。此处表上升
                        double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * t_norm));
                        double delta_z = -total_lift * (std::cos(angle) - 1);  // 加负号反转
                        // Utils::postureToTransArray(tra2_start_pose, output.pos);
                        output.pos = tra2_start_pose_m;
                        output.pos[11] += delta_z; // 直接修改16d矩阵的11位（第四列第三行）

                        // 获取实时位姿
                        std::array<double, 6> current_pose;
                        std::array<double, 7> current_tau_m;
                        std::array<double, 6> current_ext_tau_base;
                        std::array<double, 6> current_ext_tau_stiff;
                        if(robot_->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0 &&
                            robot_->getStateData(RtSupportedFields::tau_m, current_tau_m) == 0 &&
                            robot_->getStateData(RtSupportedFields::tauExt_inBase, current_ext_tau_base) == 0 &&
                            robot_->getStateData(RtSupportedFields::tauExt_inStiff, current_ext_tau_stiff) == 0) {
                            // 更新最新位姿数据
                            std::lock_guard<std::mutex> lock(pose_data_mutex_);
                            latest_current_pose_ = current_pose; // 机械臂末端实际位姿
                            // latest_target_pose_ = target_pose; // 机械臂末端目标位姿。本地计算轨迹(需要放置在上方轨迹赋值内部)，非控制器计算轨迹。如需控制器计算轨迹，另调用tcpPose_c？不确定

                            latest_current_tau_m_ = current_tau_m; // 关节力矩

                            latest_current_ext_tau_base_ = current_ext_tau_base; // 机械臂基坐标系中外部力-力矩
                            latest_current_ext_tau_stiff_ = current_ext_tau_stiff; // 机械臂基坐标系中外部力-力矩

                            node_->publish_realtime_poseAndextTau(latest_current_pose_, latest_current_ext_tau_base_);
                        }

                        time += 0.001;

                        if(time > lift_duration_time){
                            RCLCPP_INFO(node_->get_logger(), "轨迹2超时结束");
                            output.setFinished();
                            stopManually.store(false);
                        }

                    }catch(const std::exception& e) {
                        RCLCPP_ERROR(node_->get_logger(), "轨迹2执行错误: %s", e.what());
                        output.setFinished();
                        stopManually.store(false);
                        return output;
                    }
                }

                return output;
            };

            // 设置控制循环回调函数
            rtCon_->setControlLoop(callback, 0, true); // setControlLoop的返回值只能为关节角度/笛卡尔位姿/力矩
            rtCon_->startLoop(false); // 启动循环

            // 控制循环进行
            while(stopManually.load()) {
                // std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            }
            
            // std::array<double, 16> reset_matrix = {
            // 1, 0, 0, 0, 
            // 0, 1, 0, 0, 
            // 0, 0, 1, 0, 
            // 0, 0, 0, 1
            // };
            // rtCon_->setFcCoor(reset_matrix, FrameType::world, ec_);
            rtCon_->stopLoop();
            rtCon_->stopMove();
            robot_->stopReceiveRobotState(); // 停止接收机器人状态数据
            node_->pose_timer_->reset();
            node_->extTau_timer_->reset();
            node_->poseAndextTau_timer_->reset();
            RCLCPP_INFO(node_->get_logger(), "实时轨迹控制完成");
            // ============ 退出时再次重置状态 ============
            force_trigger_.store(false);
            trajectory_state_.store(TrajectoryState::INITIAL_TRAJECTORY);

        } catch (const std::exception &e) {
            node_->pose_timer_->reset();
            node_->extTau_timer_->reset();
            node_->poseAndextTau_timer_->reset();
            robot_->stopReceiveRobotState(); // 确保停止接收数据
            rtCon_->stopLoop();
            rtCon_->stopMove();
            force_trigger_.store(false);
            trajectory_state_.store(TrajectoryState::INITIAL_TRAJECTORY);
            RCLCPP_ERROR(node_->get_logger(), "实时轨迹控制错误: %s", e.what());
        }
    }

void RobotController::usr_rt_cartesian_v_control_z(double air_dist, double cruise_dist, double decel_dist, double target_speed)
    {
        usr_rt_cartesian_v_control(air_dist, cruise_dist, decel_dist, target_speed, 0.0, 0.0, 0.0, 0.0, 1);
    }