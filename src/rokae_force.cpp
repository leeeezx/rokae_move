
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
tf2::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

// Convert quaternion to euler angles
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

class Rokae_Force : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    Rokae_Force(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Start to cartesian impedance control");
        this->declare_parameter("cartesian_point", "0.45 0.0 0.5 3.14154 0.0 3.14154");
        this->declare_parameter("velocity", "0.1");
        keyborad = this->create_subscription<std_msgs::msg::String>("/keystroke", 10, std::bind(&Rokae_Force::keyborad_callback, this, std::placeholders::_1));
        command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("cartesian_pos", 10);

        try
        {
            std::string remoteIP = "192.168.0.160";
            std::string localIP = "192.168.0.10";
            robot = std::make_shared<xMateErProRobot>(remoteIP, localIP);

            RCLCPP_INFO(this->get_logger(), "---已连接到Rokae机械臂接口, 正在进行初始化---");

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
    }
    ~Rokae_Force()
    {
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

    std::array<double, 7> joint_torque_measured;
    std::array<double, 7> external_torque_measured;
    std::array<double, 3> cart_torque;
    std::array<double, 3> cart_force;
    std::array<double, 6> cartesian_array;

    std::string velocity_command;

    void publish_force_data()
    {
        try
        {
            // robot->getEndTorque(rokae::FrameType::flange, joint_torque_measured, external_torque_measured, cart_torque, cart_force, ec);
            cartesian_array = robot->posture(rokae::CoordinateType::endInRef, ec);

            double degree = (M_PI - abs(double(cartesian_array[3]))) / M_PI * 180.0;

            // cout << "cartesian_array[3]: " << fixed << setprecision(3) << degree << " degree" << endl;
            // std::vector<float> cart_force_vector(cart_force.begin(), cart_force.end());
            std::vector<float> cartesian_vector(cartesian_array.begin(), cartesian_array.end());
            std_msgs::msg::Float32MultiArray message;
            // message.data = cart_force_vector;
            message.data = cartesian_vector;
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

    std::string keyborad_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到键盘按下的消息---%s", msg->data.c_str());
        key = msg->data.c_str();
        this->get_parameter("cartesian_point", cartesian_points_string);
        this->get_parameter("velocity", velocity_command);
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
            mission(20.0, -5, cartesian_points_array);
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
        default:
            RCLCPP_INFO(this->get_logger(), "你在狗叫什么");
            break;
        }

        return key;
    }
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
    // 到达笛卡尔空间点位函数
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
    /** \brief reset robot */
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
    /** \param total_duration time
     *  \param force_in_z the force in z direction unit:N
     *  \param car_vec 6D position and orientation
     *  @brief 开始进行笛卡尔阻抗任务
     */
    void mission(double total_duration, double force_in_z, const std::array<double, 6UL> &car_vec)
    {
        try
        {
            // 获取当前位置。std::array表示一个固定大小的数组，double表示数组中元素的类型，6UL表示数组的大小。
            std::array<double, 6UL> init_position;
            init_position = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            std::cout << "We get init_position.pos: " << std::endl;
            print(std::cout, "init_position :", init_position);

            // 定义贝塞尔曲线的控制点
            std::vector<std::array<double, 6>> controlPoints = {
                init_position, // 0.6 0.0 0.5 3.14154 0.0 3.14154 计算实际点位，减小误差
                {car_vec[0], car_vec[1] - 0.3, car_vec[2] - 0.2, -2.0, car_vec[4], car_vec[5]},
                {car_vec[0], car_vec[1] + 0.1, car_vec[2] - 0.2, 2.5, car_vec[4], car_vec[5]}};

            // 设置笛卡尔阻抗系数  
            rtCon->setCartesianImpedance({1000, 1000, 1000, 100, 100, 100}, ec);
            std::cout << "---setCartesianImpedance---" << std::endl;
            /*danger*/
            // 设置方向期望力
            rtCon->setCartesianImpedanceDesiredTorque({0, 0, force_in_z, 0, 0, 0}, ec);

            std::cout << "---setCartesianImpedanceDesiredTorque---" << std::endl;
            // calculate S curves
            std::cout << "---start calculate S curves---" << std::endl;
            /**
             * @addindex start caculating
             */
            auto trajectory = TrajectoryGenerator::generateBezierTrajectory(controlPoints, total_duration);
            std::cout << "Bezier Trajectory has been Generated " << std::endl;

            // 等待2s
            std::this_thread::sleep_for(std::chrono::seconds(2));

            // 启动笛卡尔阻抗控制
            rtCon->startMove(RtControllerMode::cartesianImpedance);
            
            // c++控制台打印文本。<<用于将右边的内容"送入"输出流。std::endl是换行符。
            std::cout << "\033[31m*---cartesian_impedance---*\033[0m" << std::endl;
            std::atomic<bool> stopManually{true};
            int index = 0;

            
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition
            {
                CartesianPosition output{};
                Utils::postureToTransArray(trajectory[index], output.pos);
                index++;
                if (index > int((trajectory.size())))
                {
                    std::cout << "运动结束" << std::endl;
                    output.setFinished();
                    stopManually.store(false); // loop为非阻塞，和主线程同步停止状态
                }
                print(std::cout, "index :", index);
                return output;
            };
            rtCon->setControlLoop(callback);

            rtCon->startLoop(false);
            while (stopManually.load())
            {
                publish_force_data();
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            rtCon->stopLoop();
            rtCon->stopMove();
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<Rokae_Force>("rokae_force");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
