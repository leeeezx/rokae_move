# pragma once

#include "rclcpp/rclcpp.hpp"
#include "rokae_move/robot.h"
#include "rokae_move/motion_control_rt.h"
#include "rokae_move/utility.h"

#include <functional>
#include <memory>
#include <vector>
#include <array>

class TrajectoryGenerator; 
class Rokae_Move;

class RobotController {
public:
    // 轨迹状态枚举
    enum class TrajectoryState {
        INITIAL_TRAJECTORY,    // 初始轨迹
        TRAJECTORY_2   // 轨迹2.目前设置为紧急切换轨迹
    };

     /**
     * @brief 构造函数
     * @param robot 指向机械臂对象的共享指针
     * @param rtCon 指向实时运动控制对象的共享指针
     * @param node 指向ROS节点的指针，用于日志输出
     */
    RobotController(std::shared_ptr<rokae::xMateErProRobot> robot,
                    std::shared_ptr<rokae::RtMotionControlCobot<7U>> rtCon,
                    Rokae_Move* node);
    /**
     * @brief 析构函数，确保资源正确释放
     */
    ~RobotController();


    // =======================================================================================
    // ====================== 从 Rokae_Move 类迁移过来的所有机器人动作函数 ======================
    // =======================================================================================

    void move_enableDrag();
    void move_disableDrag();


    void move_command(const std::array<double, 6UL> &car_vec, double velocity);
    void move_init();

    void wangliuwei_exp_1(const std::array<double, 6>& point_1, const std::array<double, 6>& point_2);
    void wangliuwei_exp_2(const std::array<double, 6>& point_3);

    void cartesian_impedance_control(double air_dist, double cruise_dist, double decel_dist, double target_speed, double gamma_deg);
    void usr_cartesian_force_control(double desired_force_z, double first_time, double second_time);
    void usr_rt_cartesian_control(double first_time, double second_time);
    void usr_rt_cartesian_v_control(double z_air_dist, double z_cruise_dist, double z_decel_dist, double z_target_speed,
                                    double y_air_dist = 0.0, double y_cruise_dist = 0.0, double y_decel_dist = 0.0, 
                                    double y_target_speed = 0.0, int y_direction = 1);
    void usr_rt_cartesian_v_control_z(double air_dist, double cruise_dist, double decel_dist, double target_speed);
    void usr_rt_vertical_diagonal_control(double vertical_air_dist, double vertical_cruise_dist, double vertical_decel_dist, double vertical_target_speed,
                                            double diagonal_air_dist, double diagonal_cruise_dist, double diagonal_decel_dist, double diagonal_target_speed,
                                            double gamma_deg);
    void usr_rt_stationary_control(double hold_duration);

private:
    // 指向机械臂和实时控制器的共享指针
    std::shared_ptr<rokae::xMateErProRobot> robot_;
    std::shared_ptr<rokae::RtMotionControlCobot<7U>> rtCon_;
    std::error_code ec_;

    Rokae_Move* node_; 


    std::atomic<int> publish_counter_{0}; // 计数器，用于控制回调函数内数据发布频率
    
    std::mutex force_data_mutex_;
    std::array<double, 3> latest_force_data_{{0.0, 0.0, 0.0}};
    std::mutex pose_data_mutex_;

    std::array<double, 6> latest_current_pose_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> latest_target_pose_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 7> latest_current_tau_m_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> latest_current_ext_tau_base_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> latest_current_ext_tau_stiff_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    
    
    std::atomic<TrajectoryState> trajectory_state_{TrajectoryState::INITIAL_TRAJECTORY}; //轨迹状态标志
    std::atomic<bool>force_trigger_{false}; // 力控触发标志。原子布尔变量。

    // 力控结构体                           目前未使用
    struct ForceController {
        double desired_force_z = -5.0;
        double force_error = 0.0;
        double force_integral = 0.0;
        double kp = 0.0005;
        double ki = 0.0001;
        double max_adjust = 0.001;
        
        double calculateAdjustment(double current_force);
        void reset();
    } force_controller_;
};




