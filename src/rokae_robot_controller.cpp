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
                                 Rokae_Move* node,
                                 SensorSharedData* shared_data)
    : robot_(robot), rtCon_(rtCon), node_(node), shared_data_(shared_data)
{
    RCLCPP_INFO(node_->get_logger(), "RobotController初始化完成");
}

RobotController::~RobotController() {
    RCLCPP_INFO(node_->get_logger(), "RobotController is being destroyed.");
}


/**
 * @brief 停止控制循环并执行清理序列
 */
void RobotController::stop_control() {
    // 1. 停止 SDK 控制循环
    try {
        rtCon_->stopLoop();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "rtCon_->stopLoop() failed: %s", e.what());
    }

    // 2. 停止运动
    try {
        rtCon_->stopMove();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "rtCon_->stopMove() failed: %s", e.what());
    }

    // 3. 停止接收数据
    try {
        robot_->stopReceiveRobotState();
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "robot_->stopReceiveRobotState() failed: %s", e.what());
    }

    // 4. 重置定时器
    if (node_->pose_timer_) node_->pose_timer_->reset();
    if (node_->extTau_timer_) node_->extTau_timer_->reset();
    if (node_->poseAndextTau_timer_) node_->poseAndextTau_timer_->reset();

    // 5. 重置标志位
    trajectory_state_.store(TrajectoryState::INITIAL_TRAJECTORY);
    force_trigger_.store(false);
    is_control_running_.store(false);
    cleanup_needed_.store(false);

    RCLCPP_INFO(node_->get_logger(), "Control Loop Stopped & Cleaned up");
}

/**
 *  @brief 回到初始位置。自动回到设定的初始位置，如果初始位置定义改变，需要修改此函数的 init_point
 */
void RobotController::move_init()
{
    if (is_control_running_.load()) {
        RCLCPP_WARN(node_->get_logger(), "控制器正在运行中，忽略 move_init 请求");
        return;
    }

    try
    {
        CartesianPosition start, target;
        Utils::postureToTransArray(robot_->posture(rokae::CoordinateType::flangeInBase, ec_), start.pos); // 当前位姿
        // std::array<double, 6UL> init_point = {0.45, 0.0, 0.5, 3.14154, 0.0, 3.14154}; // 原始起始位姿
        std::array<double, 6UL> init_point = {0.45, 0.0, 0.58, 3.14154, 0.0, 3.14154};
        Utils::postureToTransArray(init_point, target.pos); // 将初始位姿设置为目标位姿

        RCLCPP_INFO(node_->get_logger(), "\n--- 正在归位 !---.");
        rtCon_->MoveL(0.05, start, target);
        RCLCPP_INFO(node_->get_logger(), "--- 已回到初始位置 ---\n\n.");

        rtCon_->stopMove();
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec_);


    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "move_init 异常: %s", e.what());
    }
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
            
            // std::atomic<bool> stopManually{true}; // 移除局部变量
            int index = 0; // 轨迹1的索引

            double time = 0.0; // 轨迹2的计算频率数值
            std::array<double, 6> tra2_start_pose; // 轨迹2的实时起点 ,array6D
            std::array<double, 16> tra2_start_pose_m; //轨迹2的实时起点，array16D行优先矩阵
            bool tra2_init = false; // 轨迹2的起点初始化标志.默认为false，如果初始化成功则为true

            const double total_lift = 0.20; // 轨迹2的上升总距离，单位m。
            const double lift_duration_time = 16.0; // 轨迹2的上升持续时间,单位s。

            const double FORCE_THRESHOLD = 40.0; // 力触发阈值，单位：N

            // ------------------------------------------------机械臂控制循环回调函数--------------------------------------------------------
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{}; // CartesianPosition output{}是给output进行类型定义
                
                // 从共享内存获取最新的传感器数据 (Try-Get 非阻塞)
                std::array<double, 6> current_force_data = {0.0};
                std::chrono::steady_clock::time_point ts;
                bool has_new_data = false;
                if (shared_data_) {
                    has_new_data = shared_data_->try_get(current_force_data, ts);
                }

                // 如果没有新数据，可以使用上一帧数据或保持0 (根据具体需求，这里暂不做特殊处理，直接使用 current_force_data)
                // 这里为了兼容原有逻辑，我们将获取到的 shared data 用于阈值判断
                // 注意：这里假设 shared_data 的索引 2 对应 Z 轴力
                double z_force = current_force_data[2];

                // 原有逻辑: robot_->getStateData(RtSupportedFields::tauExt_inBase, latest_current_ext_tau_base_);
                // 如果仍需要机器人内部状态用于发布，可以保留 getStateData，但逻辑判断使用 shared_data
                
                // 检测：力阈值触发标志 与 轨迹状态。当力触发状态为真且当前轨迹为初始轨迹时，
                if (std::abs(z_force) > FORCE_THRESHOLD && trajectory_state_.load() == TrajectoryState::INITIAL_TRAJECTORY){ 
                    RCLCPP_INFO(node_->get_logger(), "======================达到力阈值(%.2f N)，触发轨迹切换请求,运行新轨迹======================", z_force);
                    
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
                        is_control_running_.store(false); // 标记控制结束
                    }
                }

                if (trajectory_state_.load() == TrajectoryState::INITIAL_TRAJECTORY) {
                    // ==================================初始轨迹==================================
                    if (index < int(trajectory.size())) {
                        auto target_pose = trajectory[index]; // 获取目标轨迹点。这里的target_pose是当前的理论轨迹规划点
                        Utils::postureToTransArray(target_pose, output.pos); // 把target_pose转换成output.pos格式（16位矩阵）
                        
                        // 获取实时位姿 (仅用于发布/日志，不用于控制逻辑)
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
                            latest_current_pose_ = current_pose; 
                            latest_current_tau_m_ = current_tau_m; 
                            latest_current_ext_tau_base_ = current_ext_tau_base; 
                            latest_current_ext_tau_stiff_ = current_ext_tau_stiff; 

                            node_->publish_realtime_poseAndextTau(latest_current_pose_, latest_current_ext_tau_base_);
                        }

                        index++;
                    } else {
                        output.setFinished();
                        is_control_running_.store(false); // 标记控制结束
                    }
                }else{
                    try{
                        // ================================== 轨迹2 ==================================
                        if (!tra2_init) {
                        RCLCPP_ERROR(node_->get_logger(), "轨迹2未初始化!");
                        output.setFinished();
                        is_control_running_.store(false); // 标记控制结束
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
                            latest_current_pose_ = current_pose; 
                            latest_current_tau_m_ = current_tau_m; 
                            latest_current_ext_tau_base_ = current_ext_tau_base; 
                            latest_current_ext_tau_stiff_ = current_ext_tau_stiff; 

                            node_->publish_realtime_poseAndextTau(latest_current_pose_, latest_current_ext_tau_base_);
                        }

                        time += 0.001;

                        if(time > lift_duration_time){
                            RCLCPP_INFO(node_->get_logger(), "轨迹2超时结束");
                            output.setFinished();
                            is_control_running_.store(false); // 标记控制结束
                        }

                    }catch(const std::exception& e) {
                        RCLCPP_ERROR(node_->get_logger(), "轨迹2执行错误: %s", e.what());
                        output.setFinished();
                        is_control_running_.store(false); // 标记控制结束
                        return output;
                    }
                }

                return output;
            };

            // 设置控制循环回调函数
            rtCon_->setControlLoop(callback, 0, true); // setControlLoop的返回值只能为关节角度/笛卡尔位姿/力矩
            
            // 设置状态标志，通知 Monitor 线程
            is_control_running_.store(true);
            cleanup_needed_.store(true);

            rtCon_->startLoop(false); // 启动循环 (false 表示非阻塞启动，但 startLoop 本身行为取决于参数，如果这里是 ROKAE API，通常 false 意味着不阻塞主线程？ 
                                      // 实际上原代码中 startLoop(false) 后跟了一个 while 循环，说明 startLoop 可能是异步的或者非阻塞的。
                                      // 这里的关键是移除 while 循环并立即返回)
            
            RCLCPP_INFO(node_->get_logger(), "实时控制线程已启动 (非阻塞模式)");
            return; // 立即返回，主线程继续运行

        } catch (const std::exception &e) {
            // 发生异常时的清理 (仅在启动阶段出错时同步清理)
            stop_control();
            RCLCPP_ERROR(node_->get_logger(), "实时轨迹控制启动错误: %s", e.what());
        }
    }

void RobotController::usr_rt_cartesian_v_control_z(double air_dist, double cruise_dist, double decel_dist, double target_speed)
    {
        usr_rt_cartesian_v_control(air_dist, cruise_dist, decel_dist, target_speed, 0.0, 0.0, 0.0, 0.0, 1);
    }