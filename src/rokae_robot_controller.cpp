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

// ==============================================================================================
// ====================== 机器人动作函数的实现 ======================
// ===============================================================================================


// 之前写的自定义力控方式。                     目前未使用
// 计算力控制调整量
double RobotController::ForceController::calculateAdjustment(double current_force) {
    force_error = desired_force_z - current_force;
    force_integral += force_error;
    double adjustment = kp * force_error + ki * force_integral;
    return std::clamp(adjustment, -max_adjust, max_adjust);
}

// 之前写的自定义力控方式。                     目前未使用
void RobotController::ForceController::reset() {
    force_integral = 0.0;
    force_error = 0.0;
}

// /**
//  * @brief 启动拖拽模式
//  */
// void RobotController::move_enableDrag()
// {
//     try
//     {
//         robot_->enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec_);
//         RCLCPP_INFO(node_->get_logger(), "---拖拽模式已启动，按下末端侧边pilot以拖动 ！---.");
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << e.what() << '\n';
//     }
// }

// /**
//  * @brief 停止拖拽模式
//  */
// void RobotController::move_disableDrag()
// {
//     try
//     {
//         print(std::cout, "当前位置:", robot_->posture(rokae::CoordinateType::flangeInBase, ec_));
//         robot_->disableDrag(ec_); // 停止拖拽模式
//         // 重新设置机器人模式，并上电
//         robot_->setOperateMode(rokae::OperateMode::automatic, ec_);
//         // 若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效
//         robot_->setRtNetworkTolerance(20, ec_);
//         robot_->setMotionControlMode(MotionControlMode::RtCommand, ec_); // 实时模式
//         robot_->setPowerState(true, ec_);

//         RCLCPP_ERROR(node_->get_logger(), "---拖拽模式已关闭。当前为自动-实时模式，上电状态 !---.");
//     }
//     catch (const std::exception &e)
//     {
//         std::cerr << e.what() << '\n';
//     }
// }

/** 
 * @brief 使用控制command进行轨迹控制。运动指令使用有更多扩展性，详见官方C++ api文档。
 * @param car_vec 目标笛卡尔位置
 * @param velocity_command 运动速度
*/
void RobotController::move_command(const std::array<double, 6UL> &car_vec, double velocity)
{
    try
    {
        RCLCPP_INFO(node_->get_logger(), "Start Tracking ...");
        CartesianPosition start, target;

        Utils::postureToTransArray(robot_->posture(rokae::CoordinateType::flangeInBase, ec_), start.pos);
        Utils::postureToTransArray(car_vec, target.pos);
        print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);

        rtCon_->MoveL(velocity, start, target);
        print(std::cout, "完成到达笛卡尔空间点位\n");

        Eigen::Matrix3d rot_start;
        Eigen::Vector3d trans_start, trans_end;
        /////////////////////////////////////
        Utils::postureToTransArray(robot_->posture(rokae::CoordinateType::flangeInBase, ec_), start.pos);
        Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
        trans_end = trans_start;
        trans_end[1] += 0.1;  // 沿Y轴正方向移动
        Utils::transMatrixToArray(rot_start, trans_end, target.pos);
        print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);

        rtCon_->MoveL(0.1, start, target);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}


/**
 *  @brief 回到初始位置。自动回到设定的初始位置，如果初始位置定义改变，需要修改此函数的 init_point
 */
void RobotController::move_init()
{
    try
    {
        CartesianPosition start, target;
        Utils::postureToTransArray(robot_->posture(rokae::CoordinateType::flangeInBase, ec_), start.pos); // 当前位姿
        std::array<double, 6UL> init_point = {0.45, 0.0, 0.5, 3.14154, 0.0, 3.14154};
        Utils::postureToTransArray(init_point, target.pos); // 将初始位姿设置为目标位姿

        RCLCPP_INFO(node_->get_logger(), "\n--- 正在归位 !---.");
        rtCon_->MoveL(0.05, start, target);
        RCLCPP_INFO(node_->get_logger(), "--- 已回到初始位置 ---\n\n.");
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}


/** 
 * @brief 使用官方笛卡尔阻抗控制实现轨迹跟踪和恒力控制。测试时，需要先用较大的两段时间，以防出现碰撞等意外！！！
 * @param 
 * 
 * 
 * 利用
 */
void RobotController::cartesian_impedance_control(
    double air_dist, double cruise_dist, double decel_dist, double target_speed, double gamma_deg)
{
    try {
        // 参数校验，传入的距离和速度必须大于等于0
        const double EPSILON = 1e-10;
        if(air_dist <= EPSILON || cruise_dist <= EPSILON || decel_dist <= EPSILON || target_speed <= EPSILON || gamma_deg <= EPSILON) {
            RCLCPP_ERROR(node_->get_logger(), "无效的传入参数，立即检查速度控制的传入参数!");
            return;
        }

        // 停止定时器发布
        node_->pose_timer_->cancel();
        node_->extTau_timer_->cancel();
        node_->poseAndextTau_timer_->cancel();

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
        print(std::cout, "当前位姿:", start);

        std::vector<std::array<double, 6>> trajectory;
        trajectory = TrajectoryGenerator::generateDiagonalPhasePath(
            start,
            air_dist, cruise_dist, decel_dist, target_speed, gamma_deg
        ); // 速度方向需要测试。
        RCLCPP_INFO(node_->get_logger(), "已生成轨迹，共%d个点", (int)trajectory.size());

        // 设置力控坐标系为末端坐标系。需要根据实际加装的末端工具进行调整
        std::array<double, 16> toolToFlange = {
            -1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, -1, 0, 
            0, 0, 0, 1
        };
        rtCon_->setFcCoor(toolToFlange, FrameType::world, ec_);
        RCLCPP_INFO(node_->get_logger(), "力控坐标系设置完成");

        // 设置笛卡尔阻抗参数。最大值为 { 1500, 1500, 1500, 100, 100, 100 }, 单位: N/m, Nm/rad。
        // XYZ方向：设置较低刚度以实现柔顺性
        // 姿态方向：保持较高刚度以保持姿态
        rtCon_->setCartesianImpedance({3000, 3000, 2995, 300, 300, 300 }, ec_);
        RCLCPP_INFO(node_->get_logger(), "笛卡尔阻抗参数设置完毕");
        
        // 启动笛卡尔阻抗控制
        rtCon_->startMove(RtControllerMode::cartesianImpedance);
        RCLCPP_INFO(node_->get_logger(), "startMove成功");

        std::atomic<bool> stopManually{true};
        int index = 0;

        // 控制循环回调函数
        std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
            CartesianPosition output{};
            
            if (index < int(trajectory.size())) {
                // 获取当前规划点位
                auto target_pose = trajectory[index];
                Utils::postureToTransArray(target_pose, output.pos);

                index++;
            } else {
                output.setFinished();
                stopManually.store(false);
            }
            RCLCPP_INFO(node_->get_logger(), "1");
            return output;
        };

        RCLCPP_INFO(node_->get_logger(), "回调函数结束");
        rtCon_->setControlLoop(callback, 0, true);
        rtCon_->startLoop(false);
        
        while(stopManually.load()) {
        }

        rtCon_->stopLoop();
        rtCon_->stopMove();
        robot_->stopReceiveRobotState(); // 停止接收机器人状态数据
        // 设置力控坐标系为末端坐标系。需要根据实际加装的末端工具进行调整
        std::array<double, 16> reset_Coor = {
            -1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, -1, 0, 
            0, 0, 0, 1
        };
        rtCon_->setFcCoor(reset_Coor, FrameType::world, ec_);
        // robot_->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec_);
        // if (ec_) {
        //     RCLCPP_ERROR(node_->get_logger(), "切换到非实时模式失败: %s", ec_.message().c_str());
        // }
        // // **再切换回实时模式，为下一次运动做准备**
        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec_);
        if (ec_) {
            RCLCPP_ERROR(node_->get_logger(), "切换回实时模式失败: %s", ec_.message().c_str());
        }
        node_->pose_timer_->reset();
        node_->extTau_timer_->reset();
        node_->poseAndextTau_timer_->reset();
        RCLCPP_INFO(node_->get_logger(), "实时轨迹控制完成");
    }catch (const std::exception &e) {
        rtCon_->stopLoop();
        rtCon_->stopMove();
        robot_->stopReceiveRobotState(); // 停止接收机器人状态数据

        robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec_);
        if (ec_) {
            RCLCPP_ERROR(node_->get_logger(), "切换回实时模式失败: %s", ec_.message().c_str());
        }
        node_->pose_timer_->reset();
        node_->extTau_timer_->reset();
        node_->poseAndextTau_timer_->reset();
        RCLCPP_ERROR(node_->get_logger(), "实时轨迹控制错误: %s", e.what());
    }
}


// 目前未使用，未经过验证
/** 
 * @brief 自己写的：使用位置控制+PI控制器实现轨迹跟踪和恒力控制
 * @param desired_force_z 期望的z方向力(用于阻抗控制)
 * @param first_time 第一段时间
 * @param second_time 第二段时间
 */
void RobotController::usr_cartesian_force_control(double desired_force_z, double first_time, double second_time)
{
    try {
            // 获取当前位置
            std::array<double, 6UL> init_position = robot_->posture(rokae::CoordinateType::flangeInBase, ec_);
            print(std::cout, "Current position:", init_position);

            // 定义测试路径点。x/y/z/roll/pitch/yaw
            std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};         // 起始点
            std::array<double, 6> via_point = {0.4, 0.0, 0.3, M_PI, 0.0, M_PI};     // 中间点
            std::array<double, 6> end = {0.4, 0.1, 0.4, M_PI, 0.0, M_PI};           // 终点

            auto trajectory = TrajectoryGenerator::generateLinearSegmentPath(start, via_point, end, first_time, second_time);
            
            // 初始化力控参数
            force_controller_.desired_force_z = desired_force_z;
            force_controller_.reset();
            
            // 启动位置控制模式
            rtCon_->startMove(RtControllerMode::cartesianPosition);

            std::atomic<bool> stopManually{true};
            int index = 0;

            // 控制循环回调函数
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{};
                
                if (index < int(trajectory.size())) {
                    // 获取当前力数据并计算位置补偿
                    double current_force;
                    {
                        std::lock_guard<std::mutex> lock(force_data_mutex_);
                        current_force = latest_force_data_[2];
                    }
                    
                    double z_adjustment = force_controller_.calculateAdjustment(current_force);
                    
                    // 获取当前规划点位并添加补偿
                    auto target_pose = trajectory[index];
                    target_pose[2] += z_adjustment;
                    Utils::postureToTransArray(target_pose, output.pos);
                    index++;
                    
                    RCLCPP_INFO(node_->get_logger(), 
                               "Force: %.3f N, Error: %.3f N, Adjustment: %.6f m",
                               current_force, 
                               force_controller_.force_error,
                               z_adjustment);
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon_->setControlLoop(callback);
            rtCon_->startLoop(false);

            while(stopManually.load()) {
                // node_->publish_force_data(); // 废弃的发布函数。如需调用，要用getEndTorque二次封装实现，如publish_initial_ext_FandTau所示
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            rtCon_->stopLoop();
            rtCon_->stopMove();
        }
        catch (const std::exception &e) {
            std::cerr << e.what();
        }
    }


/**
 * @brief 实时模式纯轨迹控制，速度不可控
 * @param first_time 第一段时间
 * @param second_time 第二段时间
 */
void RobotController::usr_rt_cartesian_control(double first_time, double second_time)
    {
        try {
            // 停止初始位资发布（因为只需要让plojuggler读取到）
            node_->pose_timer_->cancel();

            // 设置需要接收的机器人状态数据
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m}; // 接收末端位姿数据
            robot_->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 1ms采样周期
            
            // 获取当前位置
            std::array<double, 6UL> start = robot_->posture(rokae::CoordinateType::flangeInBase, ec_);
            print(std::cout, "Current position:", start);

            // 定义三个路径点，实现先下压后平移的轨迹
            std::array<double, 6> via_point = start; // 中间点(下压xxxm)
            via_point[2] -= 0.180;

            std::array<double, 6> end = via_point;      // 终点(平移xxxm)
            end[1] += 0.1;

            auto trajectory = TrajectoryGenerator::generateLinearSegmentPath(
                start, via_point, end, first_time, second_time);
            
            // 启动笛卡尔空间位置控制模式
            rtCon_->startMove(RtControllerMode::cartesianPosition);
            
            std::atomic<bool> stopManually{true};
            int index = 0;

            // 控制循环回调函数
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{};

                RCLCPP_INFO(node_->get_logger(), "进入回调函数1");
                if (index < int(trajectory.size())) {
                    // 获取目标轨迹点
                    auto target_pose = trajectory[index];
                    Utils::postureToTransArray(target_pose, output.pos);
                    RCLCPP_INFO(node_->get_logger(), "轨迹控制2");
                    
                    // 获取实时位姿
                    // robot_->updateRobotState(std::chrono::milliseconds(1));
                    std::array<double, 6> current_pose;
                    if(robot_->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0) {
                        RCLCPP_INFO(node_->get_logger(), "更新位姿信息3: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                                    current_pose[0], current_pose[1], current_pose[2],
                                    current_pose[3], current_pose[4], current_pose[5]);
                        // 更新最新位姿数据
                        std::lock_guard<std::mutex> lock(pose_data_mutex_);
                        latest_current_pose_ = current_pose;
                        latest_target_pose_ = target_pose;

                        node_->publish_realtime_poseAndTargetPose(latest_current_pose_, latest_target_pose_);
                    }

                    index++;
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon_->setControlLoop(callback, 0, true);
            rtCon_->startLoop(false);

            // 控制循环
            while(stopManually.load()) {
                // std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            }
            
            // 停止接收机器人状态数据
            rtCon_->stopLoop();
            rtCon_->stopMove();
            robot_->stopReceiveRobotState();
            node_->pose_timer_->reset();
            RCLCPP_INFO(node_->get_logger(), "实时轨迹控制完成");

        } catch (const std::exception &e) {
            node_->pose_timer_->reset();
            robot_->stopReceiveRobotState(); // 确保停止接收数据
            RCLCPP_ERROR(node_->get_logger(), "实时轨迹控制错误: %s", e.what());
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

            const double total_lift = 0.10; // 轨迹2的上升总距离
            const double lift_duration_time = 10.0; // 轨迹2的上升持续时间

            // 时间触发。备用
            // int callback_count = 0;
            // const int trigger_count = 15000; // 假设1ms周期,5000次 = 5秒
            // bool time_triggered = false; // 防止重复触发

            const double FORCE_THRESHOLD = 0.0; // 力触发阈值，单位：N

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
            
            std::array<double, 16> reset_matrix = {
            1, 0, 0, 0, 
            0, 1, 0, 0, 
            0, 0, 1, 0, 
            0, 0, 0, 1
            };
            rtCon_->setFcCoor(reset_matrix, FrameType::world, ec_);
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

/**
 * @brief 实时模式：垂直下压 + 特定角度轨迹运动控制
 * @param vertical_air_dist 垂直下压加速段距离
 * @param vertical_cruise_dist 垂直下压匀速段距离
 * @param vertical_decel_dist 垂直下压减速段距离
 * @param vertical_target_speed 垂直下压目标速度
 * @param diagonal_air_dist 对角线加速段距离
 * @param diagonal_cruise_dist 对角线匀速段距离
 * @param diagonal_decel_dist 对角线减速段距离
 * @param diagonal_target_speed 对角线目标速度
 * @param gamma_deg Z-Y平面角度(度)
 */
void RobotController::usr_rt_vertical_diagonal_control(
    double vertical_air_dist, double vertical_cruise_dist, double vertical_decel_dist, double vertical_target_speed,
    double diagonal_air_dist, double diagonal_cruise_dist, double diagonal_decel_dist, double diagonal_target_speed,
    double gamma_deg)
    {
        try {
            // 参数校验
            const double EPSILON = 1e-10;
            if(vertical_air_dist <= EPSILON || vertical_cruise_dist <= EPSILON || 
            vertical_decel_dist <= EPSILON || vertical_target_speed <= EPSILON) {
                RCLCPP_ERROR(node_->get_logger(), "无效的垂直段参数!");
                return;
            }
            if(diagonal_air_dist <= EPSILON || diagonal_cruise_dist <= EPSILON || 
            diagonal_decel_dist <= EPSILON || diagonal_target_speed <= EPSILON) {
                RCLCPP_ERROR(node_->get_logger(), "无效的对角线段参数!");
                return;
            }
            if(gamma_deg < 0 || gamma_deg > 90) {
                RCLCPP_ERROR(node_->get_logger(), "角度gamma必须在0-90度之间!");
                return;
            }

            // 停止初始位姿发布
            node_->pose_timer_->cancel();

            // 设置需要接收的机器人状态数据
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m}; // 接收末端位姿数据
            robot_->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 1ms采样周期
            
            // 获取当前位置
            std::array<double, 6UL> start = robot_->posture(rokae::CoordinateType::flangeInBase, ec_);
            print(std::cout, "当前初始位置：", start);
            
            std::vector<std::array<double, 6>> trajectory;
            
            try {
                // 生成垂直+对角复合轨迹
                trajectory = TrajectoryGenerator::generateVerticalDiagonalPath(
                    start, 
                    vertical_air_dist, vertical_cruise_dist, vertical_decel_dist, vertical_target_speed,
                    diagonal_air_dist, diagonal_cruise_dist, diagonal_decel_dist, diagonal_target_speed,
                    gamma_deg
                );
                RCLCPP_INFO(node_->get_logger(), "已生成垂直+对角轨迹，共%d个点, 角度: %.1f度", 
                            (int)trajectory.size(), gamma_deg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "轨迹生成失败: %s", e.what());
                robot_->stopReceiveRobotState();
                node_->pose_timer_->reset();
                return;
            }

            // 启动笛卡尔空间位置控制模式
            rtCon_->startMove(RtControllerMode::cartesianPosition);
            
            std::atomic<bool> stopManually{true};
            int index = 0;

            // 控制循环回调函数
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{};

                if (index < int(trajectory.size())) {
                    // 获取目标轨迹点
                    auto target_pose = trajectory[index];
                    Utils::postureToTransArray(target_pose, output.pos);
                    
                    if (publish_counter_ ++ % 2 == 0) {
                        std::array<double, 6> current_pose;
                        if(robot_->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0) {
                            std::lock_guard<std::mutex> lock(pose_data_mutex_);
                            latest_current_pose_ = current_pose;
                            latest_target_pose_ = target_pose;
                            node_->publish_realtime_poseAndTargetPose(latest_current_pose_, latest_target_pose_);
                        }
                    }

                    index++;
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon_->setControlLoop(callback, 0, true);
            rtCon_->startLoop(false);

            // 控制循环
            while(stopManually.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            
            // 停止接收机器人状态数据
            rtCon_->stopLoop();
            rtCon_->stopMove();
            robot_->stopReceiveRobotState();
            node_->pose_timer_->reset();
            RCLCPP_INFO(node_->get_logger(), "垂直+对角轨迹控制完成\n-------------------------------");

        } catch (const std::exception &e) {
            node_->pose_timer_->reset();
            robot_->stopReceiveRobotState(); // 确保停止接收数据
            rtCon_->stopLoop();
            rtCon_->stopMove();
            RCLCPP_ERROR(node_->get_logger(), "垂直+对角轨迹控制错误: %s", e.what());
        }
    }


/**
 * @brief 实时模式：保持当前位置静止
 * @param hold_duration 静止持续时间（秒）
 */
void RobotController::usr_rt_stationary_control(double hold_duration = 20)
{
    try {
        // 参数校验
        if(hold_duration <= 0.0) {
            RCLCPP_ERROR(node_->get_logger(), "静止时间必须大于0!");
            return;
        }

        // 停止初始位姿发布
        node_->pose_timer_->cancel();
        node_->extTau_timer_->cancel();
        node_->poseAndextTau_timer_->cancel();

        // 设置需要接收的机器人状态数据
        std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m,
                                        RtSupportedFields::tcpPose_m, // 末端位姿，16D矩阵
                                        RtSupportedFields::tauExt_inBase,}; 
        robot_->startReceiveRobotState(std::chrono::milliseconds(1), fields);
        
        // 获取当前位置作为保持位置
        std::array<double, 6UL> hold_pose = robot_->posture(rokae::CoordinateType::flangeInBase, ec_);
        RCLCPP_INFO(node_->get_logger(), "保持位置: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                    hold_pose[0], hold_pose[1], hold_pose[2],
                    hold_pose[3], hold_pose[4], hold_pose[5]);
        
        // 启动笛卡尔空间位置控制模式
        rtCon_->startMove(RtControllerMode::cartesianPosition);
        
        std::atomic<bool> stopManually{true};
        double elapsed_time = 0.0;  // 已经过时间（秒）
        const double dt = 0.001;    // 控制周期 1ms
        
        // 控制循环回调函数
        std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
            CartesianPosition output{};

            if (elapsed_time < hold_duration) {
                // 持续输出相同的目标位置
                Utils::postureToTransArray(hold_pose, output.pos);
                
                
                std::array<double, 6> current_pose;
                std::array<double, 6> current_ext_tau_base;
                if(robot_->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0 &&
                   robot_->getStateData(RtSupportedFields::tauExt_inBase, current_ext_tau_base) == 0) {

                    latest_current_pose_ = current_pose;
                    latest_current_ext_tau_base_ = current_ext_tau_base;
                    // latest_target_pose_ = hold_pose;
                    node_->publish_realtime_extTau(latest_current_ext_tau_base_);
                }

                elapsed_time += dt;
                
            } else {
                output.setFinished();
                stopManually.store(false);
                RCLCPP_INFO(node_->get_logger(), "静止控制完成，总时长: %.2f 秒", elapsed_time);
            }
            
            return output;
        };

        rtCon_->setControlLoop(callback, 0, true);
        rtCon_->startLoop(false);

        // 控制循环
        while(stopManually.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        // 停止
        rtCon_->stopLoop();
        rtCon_->stopMove();
        robot_->stopReceiveRobotState();
        node_->pose_timer_->reset();
        node_->extTau_timer_->reset();
        node_->poseAndextTau_timer_->reset();
        RCLCPP_INFO(node_->get_logger(), "实时静止控制完成\n-------------------------------");

    } catch (const std::exception &e) {
        node_->pose_timer_->reset();
        node_->extTau_timer_->reset();
        node_->poseAndextTau_timer_->reset();
        robot_->stopReceiveRobotState();
        rtCon_->stopLoop();
        rtCon_->stopMove();
        RCLCPP_ERROR(node_->get_logger(), "实时静止控制错误: %s", e.what());
    }
}


void RobotController::move_enableDrag(){
    error_code ec;
    robot_->setOperateMode(rokae::OperateMode::manual, ec);
    if(ec) {
        RCLCPP_ERROR(node_->get_logger(), "切换手动模式失败: %s", ec.message().c_str());
        return;
    }
    robot_->setPowerState(false, ec);
    if(ec) {
        RCLCPP_ERROR(node_->get_logger(), "下电失败: %s", ec.message().c_str());
        return;
    }
    RCLCPP_INFO(node_->get_logger(), "等待切换拖拽模式...");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    robot_->enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
    if(ec) {
        RCLCPP_ERROR(node_->get_logger(), "启用拖拽失败: %s", ec.message().c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "拖拽模式已启用");
}


void RobotController::move_disableDrag() {
    error_code ec;
    
    // 1. 关闭拖拽
    robot_->disableDrag(ec);
    if(ec) {
        RCLCPP_ERROR(node_->get_logger(), "关闭拖拽失败: %s", ec.message().c_str());
        return;
    }
    
    // 2. 等待模式切换
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 3. 恢复到自动模式并上电
    robot_->setOperateMode(rokae::OperateMode::automatic, ec);
    robot_->setPowerState(true, ec);
    
    RCLCPP_INFO(node_->get_logger(), "拖拽模式已关闭，恢复到自动模式");
}




