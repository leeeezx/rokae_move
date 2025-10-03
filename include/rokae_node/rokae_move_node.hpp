#pragma once // 防止头文件重复包含

// 包含所有在类声明中需要用到的头文件
// rclcpp库
#include "rclcpp/rclcpp.hpp" 
// 基本消息类型库
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
// 珞石机械臂需要的库
#include <thread>
#include <cmath>
#include "rokae_move/robot.h"
#include "rokae_move/utility.h"
#include "rokae_move/print_helper.hpp"
#include "rokae_move/motion_control_rt.h"
#include <memory>
#include "nlohmann/json.hpp"
#include <sstream>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <mutex>
#include <condition_variable>
#include <filesystem> 


// 创建Rokae_Move类。直接继承rclcpp::Node，使rokae_move成为ros2的节点
class Rokae_Move : public rclcpp::Node
{
public:
    Rokae_Move(std::string name); // 构造函数声明
    ~Rokae_Move();   

private:
    // ---------------------------------初始化------------------------------------
    void setup_ros_communications(); // 用于设置所有ROS相关的部分
    void initialize_robot();         // 用于初始化珞石机器人


    // 所有成员函数声明
    void publish_force_data();
    void timer_callback();

    // 键盘输入回调函数
    std::string keyborad_callback(const std_msgs::msg::String::SharedPtr msg);

    void move_enableDrag();
    void move_disableDrag();

    std::array<double, 6UL> string_to_array(const std::string &str);

    void go2cartesian(const std::array<double, 6UL> &car_vec);
    void move_init();
    void cartesian_impedance_control(double desired_force_z, double first_time, double second_time);
    void usr_cartesian_force_control(double desired_force_z, double first_time, double second_time);
    void usr_rt_cartesian_control(double first_time, double second_time);
    void usr_rt_cartesian_v_control(double z_air_dist, double z_cruise_dist, double z_decel_dist, double z_target_speed,
                                   double y_air_dist = 0.0, double y_cruise_dist = 0.0, double y_decel_dist = 0.0, 
                                   double y_target_speed = 0.0, int y_direction = 1);
    void usr_rt_cartesian_v_control_z(double air_dist, double cruise_dist, double decel_dist, double target_speed);
    void usr_rt_vertical_diagonal_control(double vertical_air_dist, double vertical_cruise_dist, double vertical_decel_dist, double vertical_target_speed,
                                         double diagonal_air_dist, double diagonal_cruise_dist, double diagonal_decel_dist, double diagonal_target_speed,
                                         double gamma_deg);

    void force_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void publish_realtime_pose(const std::array<double, 6>& current_pose, const std::array<double, 6>& target_pose);
    void pubilsh_initial_pose();

    // 所有成员变量声明
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_publisher_;

    std::shared_ptr<rokae::xMateErProRobot> robot; // 机械臂对象
    std::shared_ptr<rokae::RtMotionControlCobot<7U>> rtCon; // 机械臂实时运动控制对象

    std::error_code ec; // 错误玛ec

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyborad;
    std::string key;
    
    std::string cartesian_points_string;
    
    std::array<double, 6UL> cartesian_points_array;
    
    std::array<double, 7> joint_torque_measured;
    std::array<double, 7> external_torque_measured;
    std::array<double, 3> cart_torque;
    std::array<double, 3> cart_force;
    std::array<double, 6> cartesian_array;
    
    std::string velocity_command;
    std::atomic<int> publish_counter{0};




    // 力控结构体
    struct ForceController {
        double desired_force_z = -5.0;
        double force_error = 0.0;
        double force_integral = 0.0;
        double kp = 0.0005;
        double ki = 0.0001;
        double max_adjust = 0.001;
        
        double calculateAdjustment(double current_force);
        void reset();
    } force_controller;


    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_subscription_;
    std::unique_ptr<std::thread> force_thread_;
    bool force_thread_running_ = false;
    std::mutex force_data_mutex_;
    std::condition_variable force_data_cv_;
    std::array<double, 3> latest_force_data_{{0.0, 0.0, 0.0}};
    
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr realtime_pose_publisher_;
    std::mutex pose_data_mutex_;
    std::array<double, 6> latest_current_pose_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> latest_target_pose_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    rclcpp::TimerBase::SharedPtr pose_timer_;
};