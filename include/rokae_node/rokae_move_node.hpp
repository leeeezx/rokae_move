#pragma once // 防止头文件重复包含

// 包含所有在类声明中需要用到的头文件
// rclcpp库
#include "rclcpp/rclcpp.hpp" 
// 基本消息类型库
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
// 珞石机械臂需要的库
#include <memory>
#include <mutex>

#include "rokae_node/rokae_robot_controller.hpp"


namespace rokae {
    class xMateErProRobot;
    template <unsigned short DoF> class RtMotionControlCobot;
}


// 创建Rokae_Move类。直接继承rclcpp::Node，使rokae_move成为ros2的节点
class Rokae_Move : public rclcpp::Node
{
public:
    Rokae_Move(std::string name); // 构造函数
    ~Rokae_Move(); // 析构函数
    
    // ================================================= 包含发布者的函数 =================================================
    // Note:名词定义
    // pose——末端位姿，包含位置和姿态，一般用6维向量表示；
    // extTau——末端外部力-力矩，一般用6维向量表示，前三维是力，后三维是力矩（机械臂自己算的）；
    // TargetPose——目标位姿，一般用6维向量表示(是计算的trajectory的数据)；
    void publish_realtime_pose(const std::array<double, 6>& current_pose); // 发布实时位姿
    void publish_realtime_extTau(const std::array<double, 6>& current_extTau); // 发布实时末端力-力矩
    void publish_realtime_poseAndTargetPose(const std::array<double, 6>& current_pose, const std::array<double, 6>& target_pose); // 发布实时位姿和目标位姿
    void publish_realtime_poseAndextTau(const std::array<double, 6>& current_pose, const std::array<double, 6>& current_extTau); // 发布实时位姿和力-力矩
    
    
    //  定时器 
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::TimerBase::SharedPtr extTau_timer_;
    rclcpp::TimerBase::SharedPtr poseAndextTau_timer_;

    rclcpp::TimerBase::SharedPtr force_trigger_timer_;

    // 回调函数
    bool z_force_check(double force_threshold = 2.0); 


private:
    // ================================= 初始化 ====================================
    void setup_ros_communications(); // 用于设置所有ROS相关的部分
    void initialize_robot();         // 用于初始化珞石机器人


    // 所有成员函数声明
    

    // ==================================== 回调函数 ====================================
    void keyborad_callback(const std_msgs::msg::String::SharedPtr msg); // 键盘输入回调函数

    void z_force_callback(const std_msgs::msg::Float32::SharedPtr msg);
    
    
    // ======================================== 定时器调用的发布函数 ========================================
    void publish_initial_pose();
    void publish_initial_poseAndextTau();
    void publish_initial_extTau();
    
    // ======================================== 辅助函数 ========================================
    std::array<double, 6UL> string_to_array(const std::string &str);
    

    // ======================================================= 成员变量 =============================================
    std::shared_ptr<rokae::xMateErProRobot> robot; // 机械臂对象
    std::shared_ptr<rokae::RtMotionControlCobot<7U>> rtCon; // 机械臂实时运动控制对象
    std::error_code ec; // 错误码ec

    // --- 机器人控制器 ---
    std::unique_ptr<RobotController> robot_controller_;

    // ====================================== ROS通信 ======================================
    // 发布者 
    // 机械臂信息
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr realtime_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr realtime_extTau_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr realtime_poseAndTargetPose_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr realtime_poseAndextTau_publisher_;
    
    // rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr realtime_extTau_publisher_; // 加了Stamped
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr realtime_pose_publisher_; // 加了Stamped

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyborad;
    
    std::string cartesian_points_string;
    // std::string velocity;
    std::atomic<double> latest_force_z_{0.0}; // 最新的Z轴力值，原子变量以确保线程安全
    
    std::array<double, 6UL> points_array;
        
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr z_force_subscription_;
        
};