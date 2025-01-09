/**
 * @file 将管道传输改为ros2话题传输
 */
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

// 添加头文件
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <mutex>
#include <condition_variable>
#include <filesystem> 

/**
 * @addindex 任务目标
 * @brief 按下q到达笛卡尔点位
 * @brief mission 1 -->开启力控并往下压0.1m -->z
 * @brief mission 2 -->往后划0.3m -->y
 */


// 开始使用珞石SDK
using namespace rokae;
using namespace std;

// Convert euler angles to quaternion
// 欧拉角转四元数
tf2::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

// Convert quaternion to euler angles
// 四元数转欧拉角
std::array<double, 3> quaternion_to_euler(const tf2::Quaternion &q)
{
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}


// 贝塞尔曲线轨迹生成器类
class TrajectoryGenerator
{
public:
    /**
     * @brief 贝塞尔曲线生成器
     * @param controlPoints 控制点
     * @param total_duration 总时长
     * @return trajectory 轨迹
     */
    static std::vector<std::array<double, 6>> generateBezierTrajectory(
        const std::vector<std::array<double, 6>> &controlPoints,
        double total_duration)
    {
        int num_points = static_cast<int>(total_duration * 1000);
        std::vector<std::array<double, 6>> trajectory;

        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / (num_points - 1) * total_duration;
            trajectory.push_back(bezierInterpolate(controlPoints, t, total_duration));
        }

        return trajectory;
    }

    /**
     * @brief 简易轨迹生成：生成线性轨迹。包括两个阶段：垂直下压和水平滑移
     * @param start 起始点
     * @param via_point 中间点
     * @param end 终点
     * @param duration_first_segment 第一段时长
     * @param duration_second_segment 第二段时长
     * @return trajectory1 最终轨迹
     */
    static std::vector<std::array<double, 6>> generateLinearSegmentPath(
            const std::array<double, 6>& start,
            const std::array<double, 6>& via_point,
            const std::array<double, 6>& end,
            double duration_first_segment,
            double duration_second_segment)
        {
            // 第一段：垂直下压 - 使用5个控制点强制直线
            std::vector<std::array<double, 6>> controlPoints1 = {
                start,
                start,
                start,
                via_point,
                via_point
            };
            auto trajectory1 = generateBezierTrajectory(controlPoints1, duration_first_segment);

            // 第二段：水平滑移 - 使用5个控制点强制直线
            std::vector<std::array<double, 6>> controlPoints2 = {
                via_point,
                via_point,
                via_point,
                end,
                end
            };
            auto trajectory2 = generateBezierTrajectory(controlPoints2, duration_second_segment);

            // 合并轨迹
            trajectory1.insert(trajectory1.end(), trajectory2.begin(), trajectory2.end());
            return trajectory1;
        }


private:
    static double smoothTrajectory(double t, double total_duration)
    {
        if (t <= 0)
            return 0;
        if (t >= total_duration)
            return 1;
        double normalizedT = t / total_duration;
        return 10 * std::pow(normalizedT, 3) - 15 * std::pow(normalizedT, 4) + 6 * std::pow(normalizedT, 5);
    }

    static double calculatePosition(double t, double start, double end, double total_duration)
    {
        double normalizedPosition = smoothTrajectory(t, total_duration);
        return start + (end - start) * normalizedPosition;
    }
    static std::array<double, 6> bezierInterpolate(
        const std::vector<std::array<double, 6>> &points,
        double t,
        double total_duration)
    {
        if (points.size() == 1)
        {
            return points[0];
        }

        std::vector<std::array<double, 6>> newPoints;
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            std::array<double, 6> interpolated;
            // S-curve interpolation for position
            for (int j = 0; j < 3; ++j)
            {
                interpolated[j] = calculatePosition(t, points[i][j], points[i + 1][j], total_duration);
            }

            // Quaternion interpolation for orientation
            tf2::Quaternion q1 = euler_to_quaternion(points[i][3], points[i][4], points[i][5]);
            tf2::Quaternion q2 = euler_to_quaternion(points[i + 1][3], points[i + 1][4], points[i + 1][5]);
            tf2::Quaternion q_interp = q1.slerp(q2, smoothTrajectory(t, total_duration));

            // Convert interpolated quaternion back to Euler angles
            auto euler = quaternion_to_euler(q_interp);
            interpolated[3] = euler[0];
            interpolated[4] = euler[1];
            interpolated[5] = euler[2];

            newPoints.push_back(interpolated);
        }

        return bezierInterpolate(newPoints, t, total_duration);
    }
};

// 珞石机械臂节点类
class Rokae_Force : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Rokae_Force(std::string name) : Node(name)
    {
        // 输出日志信息
        RCLCPP_INFO(this->get_logger(), "Start to cartesian impedance control");
        // 声明参数
        this->declare_parameter("cartesian_point(command)", "0.45 0.0 0.5 3.14154 0.0 3.14154");
        this->declare_parameter("velocity(command)", "0.1");
        this->declare_parameter("desired_force_z", -5.0);
        this->declare_parameter("first_time", 10.0);
        this->declare_parameter("second_time", 10.0);

        // 订阅键盘输入
        keyborad = this->create_subscription<std_msgs::msg::String>("/keystroke", 10, std::bind(&Rokae_Force::keyborad_callback, this, std::placeholders::_1));
        // 发布笛卡尔位置
        command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("cartesian_pos", 10);

        try
        {
            // 连接到机械臂
            std::string remoteIP = "192.168.0.160";
            std::string localIP = "192.168.0.10";
            robot = std::make_shared<xMateErProRobot>(remoteIP, localIP);

            RCLCPP_INFO(this->get_logger(), "---已连接到Rokae机械臂接口, 正在进行初始化---");

            // 一些常规预设置
            robot->setRtNetworkTolerance(50, ec);
            robot->setOperateMode(rokae::OperateMode::automatic, ec);
            // 若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效
            robot->setMotionControlMode(MotionControlMode::RtCommand, ec); // 实时模式
            robot->setPowerState(true, ec);
            RCLCPP_INFO(this->get_logger(), "---Robot powered on !---");
            // 初始化 rtCon
            rtCon = robot->getRtMotionController().lock();
            RCLCPP_INFO(this->get_logger(), "---Robot initialization completed---");
            // std::array<double, 7> q_drag_xm7p = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};
            // rtCon->MoveJ(0.5, robot->jointPos(ec), q_drag_xm7p);
            RCLCPP_INFO(this->get_logger(), "---Robot initial pose completed---");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what();
        }
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Rokae_Force::timer_callback, this));
        
        // 添加力传感器数据订阅者
        force_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "force_sensor_data", 10,
            std::bind(&Rokae_Force::force_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "已创建力传感器数据订阅者");

        // 添加实时位姿数据发布者
        realtime_pose_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "realtime_robot_pose", 10);

        // 创建位姿数据发布定时器(100Hz)
        pose_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Rokae_Force::publish_pose_timer_callback, this));
    }
    ~Rokae_Force()
    {
        // 一些关闭操作
        robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        robot->setOperateMode(rokae::OperateMode::manual, ec);
        robot->setPowerState(false, ec);
        RCLCPP_INFO(this->get_logger(), "---珞石机械臂运动节点已关闭---.");
    }

private:
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者指针
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_publisher_;
    std::shared_ptr<xMateErProRobot> robot;
    std::shared_ptr<rokae::RtMotionControlCobot<7U>> rtCon;
    std::error_code ec;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyborad;
    std::string key = {};
    std::string cartesian_points_string;
    std::array<double, 6UL> cartesian_points_array;

    // 用来给getEndTorque函数传递参数，如果需要机械臂自带计算的话，可以启用。
    std::array<double, 7> joint_torque_measured;
    std::array<double, 7> external_torque_measured;
    std::array<double, 3> cart_torque;
    std::array<double, 3> cart_force;
    std::array<double, 6> cartesian_array;

    std::string velocity_command;

    // 力控结构体，包括力控制参数和函数
    struct ForceController {
        double desired_force_z = -5.0; // 机械臂末端期望的z方向力
        double force_error = 0.0;       // 力误差
        double force_integral = 0.0;    // PI控制器积分项
        double kp = 0.0005;             // PI控制器比例项系数
        double ki = 0.0001;             // PI控制器积分项系数
        double max_adjust = 0.001;      // 最大调整量
        
        // 计算力控制调整量
        double calculateAdjustment(double current_force) {
            force_error = desired_force_z - current_force;
            force_integral += force_error;
            double adjustment = kp * force_error + ki * force_integral;
            return std::clamp(adjustment, -max_adjust, max_adjust);
        }
        
        void reset() {
            force_integral = 0.0;
            force_error = 0.0;
        }
    } force_controller;

    /**
     * @brief 用于在ros2中发布机械臂末端力/力矩、位姿数据消息
     */
    void publish_force_data()
    {
        try
        {
            // 获取机械臂末端力/力矩数据（官方自带api）
            // robot->getEndTorque(rokae::FrameType::flange, joint_torque_measured, external_torque_measured, cart_torque, cart_force, ec);
            // 获取机器人当前位姿
            cartesian_array = robot->posture(rokae::CoordinateType::endInRef, ec);

            // 计算角度（弧度转度）
            double degree = (M_PI - abs(double(cartesian_array[3]))) / M_PI * 180.0;

            // cout << "cartesian_array[3]: " << fixed << setprecision(3) << degree << " degree" << endl;
            
            // 将位姿数据转换为vector用于发布
            // std::vector<float> cart_force_vector(cart_force.begin(), cart_force.end());
            std::vector<float> cartesian_vector(cartesian_array.begin(), cartesian_array.end());
            
            // 创建ROS2消息
            std_msgs::msg::Float32MultiArray message;
            // message.data = cart_force_vector;
            message.data = cartesian_vector;

            // 发布消息
            command_publisher_->publish(message);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Error getting force data: %s", e.what());
        }
    }

    void timer_callback()
    {
        publish_force_data();
    }

    /**
     * @brief 使用键盘按键进行相应的控制功能
     */
    std::string keyborad_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到键盘按下的消息---%s", msg->data.c_str());
        key = msg->data.c_str();
        // 获取需要的参数
        this->get_parameter("cartesian_point", cartesian_points_string);
        this->get_parameter("velocity", velocity_command);
        double desired_force = this->get_parameter("desired_force_z").as_double();
        double first_time = this->get_parameter("first_time").as_double();
        double second_time = this->get_parameter("second_time").as_double();

        cartesian_points_array = string_to_array(cartesian_points_string);

        switch (key[0])
        {
        case 'q':
            cout << "array大小: " << cartesian_points_array.size() << endl;
            if (cartesian_points_array.size() != 6)
            {
                cout << "Error : 应该输入6个数字且之间用空格连接" << endl;
                break;
            }
            else
            {
                std::cout << "We will go to -> " << cartesian_points_array << std::endl;
                go2cartesian(cartesian_points_array);
                break;
            }
        case 'w':
            cout << "Misson  : Start cartesian impedance controller and press down 0.05m " << endl;
            cout << "Waiting for 1 second and pushing back 0.3m" << endl;
            cartesian_impedance_control(desired_force, first_time, second_time);
            break;
        case 'e':
            move_enableDrag();
            break;
        case 'd':
            move_disableDrag();
            break;
        case '`':
            move_init();
            break;
        case 'r':
            RCLCPP_INFO(this->get_logger(), "启动实时轨迹控制");
            usr_rt_cartesian_control(5.0, 5.0);
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "你在狗叫什么");
            break;
        }

        return key;
    }

    /**
     * @brief 启动拖拽模式
     */
    void move_enableDrag()
    {
        try
        {
            robot->enableDrag(DragParameter::cartesianSpace, DragParameter::freely, ec);
            RCLCPP_INFO(this->get_logger(), "---Robot Drag mode is enable !---.");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void move_disableDrag()
    {
        try
        {
            print(std::cout, "Now cartesian position:", robot->posture(rokae::CoordinateType::flangeInBase, ec));
            robot->disableDrag(ec);
            robot->setOperateMode(rokae::OperateMode::automatic, ec);
            // 若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效
            robot->setRtNetworkTolerance(20, ec);
            robot->setMotionControlMode(MotionControlMode::RtCommand, ec); // 实时模式
            robot->setPowerState(true, ec);

            RCLCPP_ERROR(this->get_logger(), "---DO NOT TURN THE ROBOT OFF !---.");
            RCLCPP_WARN(this->get_logger(), "---此时不要关闭机器人 !---.");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    std::array<double, 6UL> string_to_array(const std::string &str)
    {
        std::array<double, 6UL> array;
        std::vector<double> vec;
        std::stringstream ss(str);
        std::string buf;
        while (ss >> buf)
            vec.push_back(atof(buf.c_str()));
        for (uint64_t index = 0; index < vec.size(); ++index)
            array[index] = vec[index];

        return array;
    }

    /** 
     * @brief 使用控制command进行轨迹控制
     * @param 
    */
    void go2cartesian(const std::array<double, 6UL> &car_vec)
    {
        try
        {
            RCLCPP_INFO(this->get_logger(), "Start Tracking ...");
            CartesianPosition start, target;

            // // 1. 获取当前位置
            // std::array<double, 6UL> current_pos = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            
            // // 2. 创建姿态调整的中间点 - 保持当前位置,只改变姿态
            // std::array<double, 6UL> adjust_pose = current_pos;
            // adjust_pose[3] = car_vec[3];  // roll
            // adjust_pose[4] = car_vec[4];  // pitch 
            // adjust_pose[5] = car_vec[5];  // yaw

            // // 3. 首先调整姿态
            // Utils::postureToTransArray(current_pos, start.pos);
            // Utils::postureToTransArray(adjust_pose, target.pos);
            // RCLCPP_INFO(this->get_logger(), "Adjusting orientation...");
            // rtCon->MoveL(0.1, start, target);  // 低速调整姿态

            // // 4. 等待姿态调整完成
            // std::this_thread::sleep_for(std::chrono::milliseconds(500));

            Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
            RCLCPP_INFO(this->get_logger(), "car_vec_array");

            // Utils::postureToTransArray(adjust_pose, start.pos);  // 从姿态调整后的位置开始

            Utils::postureToTransArray(car_vec, target.pos);
            print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);
            // 速度在这里！！！！！
            rtCon->MoveL(atof(velocity_command.c_str()), start, target);
            print(std::cout, "完成到达笛卡尔空间点位\n");

            Eigen::Matrix3d rot_start;
            Eigen::Vector3d trans_start, trans_end;
            /////////////////////////////////////
            Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
            Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
            trans_end = trans_start;
            // 划移距离在这里改！！！！
            trans_end[1] += 0.1;
            Utils::transMatrixToArray(rot_start, trans_end, target.pos);
            print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);
            rtCon->MoveL(0.1, start, target);
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }


    /**
     *  @brief reset robot 。自动回到设定的初始位置，如果初始位置定义改变，需要修改此函数的init_point
     */
    void move_init()
    {
        try
        {
            CartesianPosition start, target;
            Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
            std::array<double, 6UL> init_point = {0.45, 0.0, 0.5, 3.14154, 0.0, 3.14154};
            Utils::postureToTransArray(init_point, target.pos);

            RCLCPP_INFO(this->get_logger(), "---Back to initial pose !---.");
            rtCon->MoveL(0.05, start, target);
            RCLCPP_INFO(this->get_logger(), "---Reset robot finish---.");
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }


    /** 
     * @brief 使用官方笛卡尔阻抗控制实现轨迹跟踪和恒力控制。测试时，需要先用较大的两段时间，以防出现碰撞等意外！！！
     * @param desired_force_z 期望的z方向力(用于阻抗控制)
     * @param first_time 第一段时间
     * @param second_time 第二段时间
     */
    void cartesian_impedance_control(double desired_force_z, double first_time, double second_time)
    {
        try {
            // 获取当前位置
            std::array<double, 6UL> init_position = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            print(std::cout, "Current position:", init_position);

            // 定义测试路径点。x/y/z/roll/pitch/yaw
            std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};         // 起始点
            std::array<double, 6> via_point = {0.4, 0.0, 0.3, M_PI, 0.0, M_PI};     // 中间点
            std::array<double, 6> end = {0.4, 0.1, 0.4, M_PI, 0.0, M_PI};           // 终点

            auto trajectory = TrajectoryGenerator::generateLinearSegmentPath(start, via_point, end, first_time, second_time);

            // 设置力控坐标系为末端坐标系。需要根据实际加装的末端工具进行调整
            // 相关数据记录：
            // 1.双头螺栓+半球足，x、y一致，z长了0.199m。
            // 2.双头螺栓+半圆柱足，x、y一致，z长了？？？m。
            std::array<double, 16> toolToFlange = {
                1, 0, 0, 0, 
                0, 1, 0, 0, 
                0, 0, 1, 0.199, 
                0, 0, 0, 1
            };
            rtCon->setFcCoor(toolToFlange, FrameType::tool, ec);

            // 设置笛卡尔阻抗参数。！最大值为 { 1500, 1500, 1500, 100, 100, 100 }, 单位: N/m, Nm/rad。
            // XYZ方向：设置较低刚度以实现柔顺性
            // 姿态方向：保持较高刚度以保持姿态
            rtCon->setCartesianImpedance({500, 500, 200, 100, 100, 100}, ec);
            
            // 设置期望力
            rtCon->setCartesianImpedanceDesiredTorque({0, 0, desired_force_z, 0, 0, 0}, ec);
            
            // 启动笛卡尔阻抗控制
            rtCon->startMove(RtControllerMode::cartesianImpedance);
            
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
                    
                    // 记录当前力数据
                    std::lock_guard<std::mutex> lock(force_data_mutex_);
                    // RCLCPP_INFO(this->get_logger(), "Current force in Z: %.3f N", latest_force_data_[2]);
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon->setControlLoop(callback);
            rtCon->startLoop(false);
            
            while(stopManually.load()) {
                publish_force_data();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            rtCon->stopLoop();
            rtCon->stopMove();
        }
        catch (const std::exception &e) {
            std::cerr << e.what();
        }
    }

    /** 
     * @brief 自己写的：使用位置控制+PI控制器实现轨迹跟踪和恒力控制
     * @param desired_force_z 期望的z方向力(用于阻抗控制)
     * @param first_time 第一段时间
     * @param second_time 第二段时间
     */
    void usr_cartesian_force_control(double desired_force_z, double first_time, double second_time)
    {
        try {
            // 获取当前位置
            std::array<double, 6UL> init_position = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            print(std::cout, "Current position:", init_position);

            // 定义测试路径点。x/y/z/roll/pitch/yaw
            std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};         // 起始点
            std::array<double, 6> via_point = {0.4, 0.0, 0.3, M_PI, 0.0, M_PI};     // 中间点
            std::array<double, 6> end = {0.4, 0.1, 0.4, M_PI, 0.0, M_PI};           // 终点

            auto trajectory = TrajectoryGenerator::generateLinearSegmentPath(start, via_point, end, first_time, second_time);
            
            // 初始化力控参数
            force_controller.desired_force_z = desired_force_z;
            force_controller.reset();
            
            // 启动位置控制模式
            rtCon->startMove(RtControllerMode::cartesianPosition);
            
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
                    
                    double z_adjustment = force_controller.calculateAdjustment(current_force);
                    
                    // 获取当前规划点位并添加补偿
                    auto target_pose = trajectory[index];
                    target_pose[2] += z_adjustment;
                    Utils::postureToTransArray(target_pose, output.pos);
                    index++;
                    
                    RCLCPP_INFO(this->get_logger(), 
                               "Force: %.3f N, Error: %.3f N, Adjustment: %.6f m",
                               current_force, 
                               force_controller.force_error,
                               z_adjustment);
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon->setControlLoop(callback);
            rtCon->startLoop(false);
            
            while(stopManually.load()) {
                publish_force_data();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            rtCon->stopLoop();
            rtCon->stopMove();
        }
        catch (const std::exception &e) {
            std::cerr << e.what();
        }
    }

    /**
     * @brief 实时模式纯轨迹控制
     * @param first_time 第一段时间
     * @param second_time 第二段时间
     */
    void usr_rt_cartesian_control(double first_time, double second_time)
    {
        try {
            // 设置需要接收的机器人状态数据
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m}; // 接收末端位姿数据
            robot->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 1ms采样周期
            
            // 获取当前位置
            std::array<double, 6UL> init_position = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            print(std::cout, "Current position:", init_position);

            // 定义三个路径点，实现先下压后平移的轨迹
            std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};    // 起始点 
            std::array<double, 6> via_point = {0.4, 0.0, 0.3, M_PI, 0.0, M_PI}; // 中间点(下压0.2m)
            std::array<double, 6> end = {0.4, 0.3, 0.3, M_PI, 0.0, M_PI};      // 终点(平移0.3m)

            auto trajectory = TrajectoryGenerator::generateLinearSegmentPath(
                start, via_point, end, first_time, second_time);
            
            // 启动笛卡尔空间位置控制模式
            rtCon->startMove(RtControllerMode::cartesianPosition);
            
            std::atomic<bool> stopManually{true};
            int index = 0;

            // 控制循环回调函数
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{};
                
                if (index < int(trajectory.size())) {
                    // 获取目标轨迹点
                    auto target_pose = trajectory[index];
                    Utils::postureToTransArray(target_pose, output.pos);
                    index++;
                    
                    // 获取实时位姿
                    robot->updateRobotState(std::chrono::milliseconds(1));
                    std::array<double, 6> current_pose;
                    if(robot->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0) {
                        // 更新最新位姿数据
                        {
                            std::lock_guard<std::mutex> lock(pose_data_mutex_);
                            latest_current_pose_ = current_pose;
                            latest_target_pose_ = target_pose;
                        }
                        
                        // // 保持原有的日志输出
                        // if(index % 10 == 0) {
                        //     RCLCPP_INFO(this->get_logger(), 
                        //               "Target pos: x=%.3f, y=%.3f, z=%.3f\n"
                        //               "Current pos: x=%.3f, y=%.3f, z=%.3f\n"
                        //               "Position error: x=%.3f, y=%.3f, z=%.3f", 
                        //               target_pose[0], target_pose[1], target_pose[2],
                        //               current_pose[0], current_pose[1], current_pose[2],
                        //               std::abs(target_pose[0]-current_pose[0]),
                        //               std::abs(target_pose[1]-current_pose[1]), 
                        //               std::abs(target_pose[2]-current_pose[2]));
                        // }
                    }
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon->setControlLoop(callback);
            rtCon->startLoop(true);
            
            // 停止接收机器人状态数据
            robot->stopReceiveRobotState();
            RCLCPP_INFO(this->get_logger(), "实时轨迹控制完成");

        } catch (const std::exception &e) {
            robot->stopReceiveRobotState(); // 确保停止接收数据
            RCLCPP_ERROR(this->get_logger(), "实时轨迹控制错误: %s", e.what());
        }
    }

    // 添加订阅者成员变量
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr force_subscription_;
    // 添加线程相关成员变量
    std::unique_ptr<std::thread> force_thread_;              // 力数据读取线程
    bool force_thread_running_ = false;                      // 线程运行标志
    std::mutex force_data_mutex_;                           // 互斥锁
    std::condition_variable force_data_cv_;                 // 条件变量
    std::array<double, 3> latest_force_data_{{0.0, 0.0, 0.0}}; // 最新力数据

    /**
     * @brief 力数据订阅回调函数，用来获取最新的力数据
     * @param msg 力数据消息
     */
    void force_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(force_data_mutex_);
        if(msg->data.size() >= 3) {
            latest_force_data_[0] = msg->data[0];
            latest_force_data_[1] = msg->data[1];
            latest_force_data_[2] = msg->data[2];
        }
        force_data_cv_.notify_one();
    }

    // 添加发布者成员变量
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr realtime_pose_publisher_;
    
    // 添加成员变量存储最新的位姿数据
    std::mutex pose_data_mutex_;
    std::array<double, 6> latest_current_pose_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 6> latest_target_pose_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    
    // 添加位姿数据发布定时器
    rclcpp::TimerBase::SharedPtr pose_timer_;

    /**
     * @brief 位姿数据发布定时器回调函数
     */
    void publish_pose_timer_callback() {
        std::lock_guard<std::mutex> lock(pose_data_mutex_);
        publish_realtime_pose(latest_current_pose_, latest_target_pose_);
    }

    /**
     * @brief 发布实时位姿数据到话题
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿
     */
    void publish_realtime_pose(const std::array<double, 6>& current_pose, 
                             const std::array<double, 6>& target_pose) 
    {
        std_msgs::msg::Float32MultiArray msg;
        // 将当前位姿和目标位姿打包到一起发布
        // [current_x, current_y, current_z, target_x, target_y, target_z]
        msg.data = {
            static_cast<float>(current_pose[0]),
            static_cast<float>(current_pose[1]),
            static_cast<float>(current_pose[2]),
            static_cast<float>(target_pose[0]),
            static_cast<float>(target_pose[1]),
            static_cast<float>(target_pose[2])
        };
        realtime_pose_publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<Rokae_Force>("rokae_force");
    
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
