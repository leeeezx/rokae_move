#include "rokae_node/rokae_move_node.hpp"      // 包含节点类的头文件
#include "rokae_node/trajectory_generator.hpp" // 包含轨迹生成器的头文件

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



/** 
 * @brief 构造函数，初始化rokae_move节点。
 * @param name 节点名称
 * @note 构造函数与析构函数的名字必须与类名相同
 */
 Rokae_Move::Rokae_Move(std::string name) : Node(name)
{
    setup_ros_communications(); // 设置ROS通信
    initialize_robot();        // 初始化机械臂
}

/**
 * @brief 析构函数，关闭节点时进行清理工作
 */
Rokae_Move::~Rokae_Move()
{
    // 一些关闭操作
    robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot->setOperateMode(rokae::OperateMode::manual, ec);
    robot->setPowerState(false, ec);
    RCLCPP_INFO(this->get_logger(), "---珞石机械臂运动节点已关闭---.");
}


// -----------------------------------------------------------------------------------------------------------
// ---------------------------------------------------私有成员函数----------------------------------------------
// -----------------------------------------------------------------------------------------------------------

/**
 * @brief 初始化ROS通信，包括参数声明、话题订阅和发布
 */
void Rokae_Move::setup_ros_communications()
{
    // 输出日志信息
    // RCLCPP_INFO(this->get_logger(), "Start to cartesian impedance control");

    /** 
    * @brief 通用浮点参数描述生成器。用于rqt等界面显示参数信息
    * @param desc 人类可读名称
    * @param min 最小值
    * @param max 最大值
    * @param step 调整步长
    */
    auto floatDesc = [](const std::string& desc, double min, double max, double step) {
        rcl_interfaces::msg::ParameterDescriptor d;
        d.description = desc;
        d.floating_point_range.resize(1);
        d.floating_point_range[0].from_value = min;
        d.floating_point_range[0].to_value = max;
        d.floating_point_range[0].step = step;
        return d;
    };


    // 参数声明
    this->declare_parameter("cartesian_point(command)", "0.45 0.0 0.5 3.14154 0.0 3.14154");
    this->declare_parameter("velocity(command)", "0.1");
    this->declare_parameter("desired_force_z", -5.0);

    this->declare_parameter("first_time", 10.0);
    this->declare_parameter("second_time", 10.0);

    this->declare_parameter("air_distance", 0.12, 
        floatDesc("空中加速段距离（米）", 0.01, 1.0, 0.001));       //空中加速段距离
    this->declare_parameter("cruise_distance", 0.05,
        floatDesc("匀速侵入段距离（米）", 0.01, 1.0, 0.001));    //匀速侵入段距离
    this->declare_parameter("decel_distance", 0.02,
        floatDesc("减速缓冲段距离（米）", 0.00, 1.0, 0.001));     //减速缓冲段距离
    this->declare_parameter("target_speed", 0.01,
        floatDesc("期望侵入速度（米/秒）", 0.01, 2.5, 0.01));   //期望侵入速度

    // 添加Y轴参数
    this->declare_parameter("y_air_distance", 0.04, 
        floatDesc("Y轴加速段距离（米）", 0.01, 1.0, 0.001));     //Y轴加速段距离
    this->declare_parameter("y_cruise_distance", 0.05,
        floatDesc("Y轴匀速段距离（米）", 0.01, 1.0, 0.001));     //Y轴匀速段距离
    this->declare_parameter("y_decel_distance", 0.01,
        floatDesc("Y轴减速段距离（米）", 0.00, 1.0, 0.001));     //Y轴减速段距离
    this->declare_parameter("y_target_speed", 0.01,
        floatDesc("Y轴期望速度（米/秒）", 0.01, 2.5, 0.01));    //Y轴期望速度
    this->declare_parameter("y_direction", 1,
        floatDesc("Y轴方向(1=正向,-1=负向)", -1, 1, 2));       //Y轴方向

    // 添加对角线参数
    this->declare_parameter("diagonal_air_distance", 0.01, 
        floatDesc("对角线加速段距离（米）", 0.0, 1.0, 0.001));
    this->declare_parameter("diagonal_cruise_distance", 0.01,
        floatDesc("对角线匀速段距离（米）", 0.0, 1.0, 0.001));
    this->declare_parameter("diagonal_decel_distance", 0.01,
        floatDesc("对角线减速段距离（米）", 0.0, 1.0, 0.001));
    this->declare_parameter("diagonal_target_speed", 0.01,
        floatDesc("对角线期望速度（米/秒）", 0.01, 2.5, 0.01));
    this->declare_parameter("gamma_angle", 45.00,
        floatDesc("对角线Z-Y平面角度（度，0=Y轴负方向，90=Z轴负方向）", 0.00, 90.00, 0.01));

    // 订阅键盘输入
    keyborad = this->create_subscription<std_msgs::msg::String>("/keystroke", 10, std::bind(&Rokae_Move::keyborad_callback, this, std::placeholders::_1));
    // 发布笛卡尔位置
    command_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("cartesian_pos", 10);
    
    // 创建定时器，500ms为周期，定时发布
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Rokae_Move::timer_callback, this));
    
    // 添加力传感器数据订阅者
    force_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "force_sensor_data", 10,
        std::bind(&Rokae_Move::force_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "已创建力传感器数据订阅者");

    // 添加实时位姿数据发布者
    realtime_pose_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "realtime_robot_pose", 10);

    // 创建位姿数据发布定时器(10Hz)
    pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Rokae_Move::pubilsh_initial_pose, this));

}


/**
 * @brief 初始化机械臂，包括连接、上电和控制模式设置等。均调用珞石SDK的API实现。
 * 各注意事项见官方文档
 */
void Rokae_Move::initialize_robot()
{
    // --------------------------------------连接机械臂--------------------------------------
    try
    {
        // 连接到机械臂
        std::string remoteIP = "192.168.0.160";
        std::string localIP = "192.168.0.10";
        robot = std::make_shared<xMateErProRobot>(remoteIP, localIP);

        RCLCPP_INFO(this->get_logger(), "---已连接到Rokae机械臂接口, 正在进行初始化---");

        // 机械臂初始化设置
        robot->setRtNetworkTolerance(50, ec); // 网络延迟。若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效
        
        robot->setOperateMode(rokae::OperateMode::automatic, ec); // 操作模式。自动
        robot->setMotionControlMode(MotionControlMode::RtCommand, ec); // 控制模式：实时模式

        robot->setPowerState(true, ec); // 上电
        RCLCPP_INFO(this->get_logger(), "---机器人上电成功！操作模式：自动，控制模式：实时---");

        // 初始化 rtCon
        // 获取robot中的实时运动控制器对象getRtMotionController()，通过lock()将其转化为强指针并赋值给rtCon
        rtCon = robot->getRtMotionController().lock(); 
        RCLCPP_INFO(this->get_logger(), "---机器人初始化完成---");
        // std::array<double, 7> q_drag_xm7p = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};
        // rtCon->MoveJ(0.5, robot->jointPos(ec), q_drag_xm7p);
        // RCLCPP_INFO(this->get_logger(), "---Robot initial pose completed---");
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(this->get_logger(), "Robot initialization failed: %s", e.what());
        // 可以选择在这里关闭节点
        rclcpp::shutdown();
    }
}

/**
 * @brief 用于在ros2中发布机械臂末端力/力矩、位姿数据消息
 */
void Rokae_Move::publish_force_data()
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

void Rokae_Move::timer_callback()
{
    publish_force_data();
}

/**
 * @brief 使用键盘按键进行相应的控制功能
 */
std::string Rokae_Move::keyborad_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // ！！！键盘按键触发后立刻获取需要的参数的最新值。这些函数会在rqt中受操作者改动。！！！
    // ！！！所以必须在每次触发时获取最新值。！！！当前方式比较直接，后续可以改为参数回调函数。
    // TODO: 改为参数回调函数
    this->get_parameter("cartesian_point", cartesian_points_string);
    this->get_parameter("velocity", velocity_command);

    double desired_force = this->get_parameter("desired_force_z").as_double();

    double first_time = this->get_parameter("first_time").as_double();
    double second_time = this->get_parameter("second_time").as_double();

    double air_dist = this->get_parameter("air_distance").as_double();
    double cruise_dist = this->get_parameter("cruise_distance").as_double();
    double decel_dist = this->get_parameter("decel_distance").as_double();
    double target_speed = this->get_parameter("target_speed").as_double();

    double y_air_dist = this->get_parameter("y_air_distance").as_double();
    double y_cruise_dist = this->get_parameter("y_cruise_distance").as_double();
    double y_decel_dist = this->get_parameter("y_decel_distance").as_double();
    double y_target_speed = this->get_parameter("y_target_speed").as_double();
    int y_direction = this->get_parameter("y_direction").as_int();

    double diagonal_air_dist = this->get_parameter("diagonal_air_distance").as_double();
    double diagonal_cruise_dist = this->get_parameter("diagonal_cruise_distance").as_double();
    double diagonal_decel_dist = this->get_parameter("diagonal_decel_distance").as_double();
    double diagonal_target_speed = this->get_parameter("diagonal_target_speed").as_double();
    double gamma_angle = this->get_parameter("gamma_angle").as_double();
    
    // 收到键盘消息
    RCLCPP_INFO(this->get_logger(), "收到键盘按下的消息---%s", msg->data.c_str());
    key = msg->data.c_str();
    
    // 将为了rqt的字符串形式转换为可用的参数形式
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
        usr_rt_cartesian_control(10.0, 10.0);
        break;
    case 'v':
        RCLCPP_INFO(this->get_logger(), "启动速度控制");
        usr_rt_cartesian_v_control_z(air_dist, cruise_dist, decel_dist, target_speed);
        break;
    case 'b':  // 新增键位支持复合轨迹控制
        RCLCPP_INFO(this->get_logger(), "启动速度控制（Z-Y复合轨迹控制）");
        usr_rt_cartesian_v_control(
            air_dist, cruise_dist, decel_dist, target_speed,
            y_air_dist, y_cruise_dist, y_decel_dist, y_target_speed, y_direction  // 默认Y轴参数
        );
        break;
    case 'n': // 新增键位支持垂直+对角轨迹控制
        RCLCPP_INFO(this->get_logger(), "启动垂直下压+对角线运动控制（角度:%.1f度）", gamma_angle);
        usr_rt_vertical_diagonal_control(
            air_dist, cruise_dist, decel_dist, target_speed,  // 垂直段参数
            diagonal_air_dist, diagonal_cruise_dist, diagonal_decel_dist, diagonal_target_speed,  // 对角线段参数
            gamma_angle  // 对角线角度
        );
        break;
    case 't': // 用于测试布尔原子变量是否可以传递进机器人回调函数中
        RCLCPP_INFO(this->get_logger(), "力控标志位触发，轨迹切换");
        force_trigger_.store(true); // 设置为true
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
void Rokae_Move::move_enableDrag()
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

void Rokae_Move::move_disableDrag()
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

std::array<double, 6UL> Rokae_Move::string_to_array(const std::string &str)
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
void Rokae_Move::go2cartesian(const std::array<double, 6UL> &car_vec)
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
void Rokae_Move::move_init()
{
    try
    {
        CartesianPosition start, target;
        Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
        std::array<double, 6UL> init_point = {0.45, 0.0, 0.5, 3.14154, 0.0, 3.14154};
        Utils::postureToTransArray(init_point, target.pos);

        RCLCPP_INFO(this->get_logger(), "\n---Back to initial pose !---.");
        rtCon->MoveL(0.05, start, target);
        RCLCPP_INFO(this->get_logger(), "---Reset robot finish---\n\n.");
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
void Rokae_Move::cartesian_impedance_control(double desired_force_z, double first_time, double second_time)
{
    try {
            // 获取当前位置
            std::array<double, 6UL> start = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            print(std::cout, "Current position:", start);

            // 定义三个路径点，实现先下压后平移的轨迹
            std::array<double, 6> via_point = start; // 中间点(下压xxxm)
            via_point[2] -= 0.190;

            std::array<double, 6> end = via_point;      // 终点(平移xxxm)
            end[1] += 0.1;

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
            rtCon->setCartesianImpedance({1500, 1500, 1500, 100, 100, 100}, ec);
            
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
void Rokae_Move::usr_cartesian_force_control(double desired_force_z, double first_time, double second_time)
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
     * @brief 实时模式纯轨迹控制，速度不可控
     * @param first_time 第一段时间
     * @param second_time 第二段时间
     */
    void Rokae_Move::usr_rt_cartesian_control(double first_time, double second_time)
    {
        try {
            // 停止初始位资发布（因为只需要让plojuggler读取到）
            pose_timer_->cancel();

            // 设置需要接收的机器人状态数据
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m}; // 接收末端位姿数据
            robot->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 1ms采样周期
            
            // 获取当前位置
            std::array<double, 6UL> start = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            print(std::cout, "Current position:", start);

            // 定义三个路径点，实现先下压后平移的轨迹
            std::array<double, 6> via_point = start; // 中间点(下压xxxm)
            via_point[2] -= 0.180;

            std::array<double, 6> end = via_point;      // 终点(平移xxxm)
            end[1] += 0.1;

            auto trajectory = TrajectoryGenerator::generateLinearSegmentPath(
                start, via_point, end, first_time, second_time);
            
            // 启动笛卡尔空间位置控制模式
            rtCon->startMove(RtControllerMode::cartesianPosition);
            
            std::atomic<bool> stopManually{true};
            int index = 0;

            // 控制循环回调函数
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{};

                RCLCPP_INFO(this->get_logger(), "进入回调函数1");
                if (index < int(trajectory.size())) {
                    // 获取目标轨迹点
                    auto target_pose = trajectory[index];
                    Utils::postureToTransArray(target_pose, output.pos);
                    RCLCPP_INFO(this->get_logger(), "轨迹控制2");
                    
                    // 获取实时位姿
                    // robot->updateRobotState(std::chrono::milliseconds(1));
                    std::array<double, 6> current_pose;
                    if(robot->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0) {
                        RCLCPP_INFO(this->get_logger(), "更新位姿信息3: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                                    current_pose[0], current_pose[1], current_pose[2],
                                    current_pose[3], current_pose[4], current_pose[5]);
                        // 更新最新位姿数据
                        std::lock_guard<std::mutex> lock(pose_data_mutex_);
                        latest_current_pose_ = current_pose;
                        latest_target_pose_ = target_pose;

                        publish_realtime_pose(latest_current_pose_, latest_target_pose_);
                    }

                    index++;
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon->setControlLoop(callback, 0, true);
            rtCon->startLoop(false);

            // 控制循环
            while(stopManually.load()) {
                // std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            }
            
            // 停止接收机器人状态数据
            rtCon->stopLoop();
            rtCon->stopMove();
            robot->stopReceiveRobotState();
            pose_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "实时轨迹控制完成");

        } catch (const std::exception &e) {
            pose_timer_->reset();
            robot->stopReceiveRobotState(); // 确保停止接收数据
            RCLCPP_ERROR(this->get_logger(), "实时轨迹控制错误: %s", e.what());
        }
    }

    /**
     * @brief 实时模式纯轨迹控制，速度可控
     * @param ari_dist 空中加速段距离
     * @param cruise_dist 匀速侵入段距离
     * @param decel_dist 减速缓冲段距离
     * @param target_speed 期望侵入速度
     * @
     */
    void Rokae_Move::usr_rt_cartesian_v_control(
        double z_air_dist, double z_cruise_dist, double z_decel_dist, double z_target_speed,
        double y_air_dist, double y_cruise_dist, double y_decel_dist, 
        double y_target_speed, int y_direction)
    {
        try {
            // 参数校验，传入的距离和速度必须大于等于0
            const double EPSILON = 1e-10;
            if(z_air_dist <= EPSILON || z_cruise_dist <= EPSILON || z_decel_dist <= EPSILON || z_target_speed <= EPSILON) {
                RCLCPP_ERROR(this->get_logger(), "无效的z轴传入参数，立即检查速度控制的传入参数!");
                return;
            }

            // 停止初始位资发布（因为只需要让plojuggler读取到）
            pose_timer_->cancel();

            // 设置需要接收的机器人状态数据
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m, RtSupportedFields::tcpPose_m}; 
            robot->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 启动接收机器人数据。1ms采样周期
            
            // 获取当前位置
            std::array<double, 6UL> start = robot->posture(rokae::CoordinateType::flangeInBase, ec);
            print(std::cout, "当前初始位置：", start);
            
            std::vector<std::array<double, 6>> trajectory;

            // 判断是否为复合轨迹
            bool is_composite = (y_air_dist > EPSILON && y_cruise_dist > EPSILON && 
                                y_decel_dist > EPSILON && y_target_speed > EPSILON);

            if(is_composite) {
                // 生成复合轨迹
                trajectory = TrajectoryGenerator::generateZYCompositePath(
                    start, 
                    z_air_dist, z_cruise_dist, z_decel_dist, z_target_speed,
                    y_air_dist, y_cruise_dist, y_decel_dist, y_target_speed, y_direction
            );
            RCLCPP_INFO(this->get_logger(), "已生成Z-Y复合轨迹，共%d个点", (int)trajectory.size());
            } else {
                // 仅生成Z轴轨迹
                trajectory = TrajectoryGenerator::generateFourPhasePath(
                    start, z_air_dist, z_cruise_dist, z_decel_dist, z_target_speed
            );
            RCLCPP_INFO(this->get_logger(), "已生成Z轴轨迹，共%d个点", (int)trajectory.size());
            }

            // 启动笛卡尔空间位置控制模式
            rtCon->startMove(RtControllerMode::cartesianPosition);
            
            std::atomic<bool> stopManually{true}; // 停止标志，如果为false就会跳出while循环，然后执行一系列停止命令
            int index = 0;

            double time = 0.0; // 轨迹2的计算频率数值

            std::array<double, 6> tra2_start_pose; // 轨迹2的实时起点 ,array6D
            std::array<double, 16> tra2_start_pose_m; //轨迹2的实时起点，array16D行优先矩阵

            bool tra2_init = false; // 轨迹2的起点初始化标志.默认为false，如果初始化成功则为true

            const double total_lift = 0.1;
            const double lift_duration_time = 3.0;

            // ------------------------------------------------机械臂控制循环回调函数--------------------------------------------------------
            // CartesianPosition output{}是给output进行类型定义
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{};
                

                // 检测：力阈值触发标志 与 轨迹状态。当力触发状态为真且当前轨迹为初始轨迹时，将
                if (force_trigger_.load() && trajectory_state_.load() == TrajectoryState::INITIAL_TRAJECTORY){ 
                    RCLCPP_INFO(this->get_logger(), "达到力阈值，触发轨迹切换请求,运行新轨迹...");
                    trajectory_state_.store(TrajectoryState::TRAJECTORY_2); // 切换轨迹状态为轨迹2

                    // 获取切换时刻的当前位置作为轨迹2起点
                    // tcpPoseAbc_m:末端位姿, 相对于基坐标系 [X,Y,Z,Rx,Ry,Rz] - Array6d
                    // tcpPose_m:末端位姿, 相对于基坐标系, 行优先齐次变换矩阵 - Array16D
                    if (robot->getStateData(RtSupportedFields::tcpPoseAbc_m, tra2_start_pose) == 0 &&
                        robot->getStateData(RtSupportedFields::tcpPose_m, tra2_start_pose_m) == 0) {
                        tra2_init = true; // 标志位置为true，表示轨迹2起点已初始化
                        time = 0.0;  // 重置时间
                        RCLCPP_INFO(this->get_logger(), 
                                "轨迹2起点: [%.3f, %.3f, %.3f]",
                                tra2_start_pose[0], tra2_start_pose[1], tra2_start_pose[2]);
                    } 
                }

                if (trajectory_state_.load() == TrajectoryState::INITIAL_TRAJECTORY) {
                    // ==================================初始轨迹==================================
                    if (index < int(trajectory.size())) {
                        auto target_pose = trajectory[index]; // 获取目标轨迹点。这里的target_pose是当前的理论轨迹规划点
                        Utils::postureToTransArray(target_pose, output.pos); // 把target_pose转换成output.pos格式（16位矩阵）
                        
                        // 获取实时位姿
                        std::array<double, 6> current_pose;
                        if(robot->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0) {
                            // 更新最新位姿数据
                            std::lock_guard<std::mutex> lock(pose_data_mutex_);
                            latest_current_pose_ = current_pose;
                            latest_target_pose_ = target_pose;
    
                            publish_realtime_pose(latest_current_pose_, latest_target_pose_);
                        }
    
                        index++;
                    } else {
                        output.setFinished();
                        stopManually.store(false);
                    }

                }else{
                    // ==================================轨迹2==================================
                    // TODO 发布位姿数据。
                    // 初始化检查
                    if (!tra2_init) {
                    RCLCPP_ERROR(this->get_logger(), "轨迹2未初始化!");
                    output.setFinished();
                    stopManually.store(false);
                    return output;
                    }

                    // 轨迹2方案：直线上升一段距离
                    if (time < lift_duration_time){
                        double t_norm = time / lift_duration_time;  // 归一化 [0,1]
                        
                        // 余弦函数轨迹。此处表上升
                        double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * t_norm));
                        double delta_z = -total_lift * (std::cos(angle) - 1);  // 加负号反转
                        // Utils::postureToTransArray(tra2_start_pose, output.pos);
                        output.pos = tra2_start_pose_m;
                        output.pos[11] += delta_z; // 直接修改16d矩阵的11位（第四列第三行）

                        time += 0.001;
                    }else{
                        output.setFinished();
                        stopManually.store(false);
                    }
                }
                
                return output;
            };

            // 设置控制循环回调函数
            rtCon->setControlLoop(callback, 0, true); // setControlLoop的返回值只能为关节角度/笛卡尔位姿/力矩
            rtCon->startLoop(false); // 启动循环

            // 控制循环进行
            while(stopManually.load()) {
                // std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            }
            
            // 停止
            rtCon->stopLoop();
            rtCon->stopMove();
            robot->stopReceiveRobotState(); // 停止接收机器人状态数据
            pose_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "实时轨迹控制完成");

        } catch (const std::exception &e) {
            pose_timer_->reset();
            robot->stopReceiveRobotState(); // 确保停止接收数据
            rtCon->stopLoop();
            rtCon->stopMove();
            RCLCPP_ERROR(this->get_logger(), "实时轨迹控制错误: %s", e.what());
        }
    }

    void Rokae_Move::usr_rt_cartesian_v_control_z(double air_dist, double cruise_dist, double decel_dist, double target_speed)
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
    void Rokae_Move::usr_rt_vertical_diagonal_control(
        double vertical_air_dist, double vertical_cruise_dist, double vertical_decel_dist, double vertical_target_speed,
        double diagonal_air_dist, double diagonal_cruise_dist, double diagonal_decel_dist, double diagonal_target_speed,
        double gamma_deg)
    {
        try {
            // 参数校验
            const double EPSILON = 1e-10;
            if(vertical_air_dist <= EPSILON || vertical_cruise_dist <= EPSILON || 
            vertical_decel_dist <= EPSILON || vertical_target_speed <= EPSILON) {
                RCLCPP_ERROR(this->get_logger(), "无效的垂直段参数!");
                return;
            }
            if(diagonal_air_dist <= EPSILON || diagonal_cruise_dist <= EPSILON || 
            diagonal_decel_dist <= EPSILON || diagonal_target_speed <= EPSILON) {
                RCLCPP_ERROR(this->get_logger(), "无效的对角线段参数!");
                return;
            }
            if(gamma_deg < 0 || gamma_deg > 90) {
                RCLCPP_ERROR(this->get_logger(), "角度gamma必须在0-90度之间!");
                return;
            }

            // 停止初始位姿发布
            pose_timer_->cancel();

            // 设置需要接收的机器人状态数据
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m}; // 接收末端位姿数据
            robot->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 1ms采样周期
            
            // 获取当前位置
            std::array<double, 6UL> start = robot->posture(rokae::CoordinateType::flangeInBase, ec);
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
                RCLCPP_INFO(this->get_logger(), "已生成垂直+对角轨迹，共%d个点, 角度: %.1f度", 
                            (int)trajectory.size(), gamma_deg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "轨迹生成失败: %s", e.what());
                robot->stopReceiveRobotState();
                pose_timer_->reset();
                return;
            }

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
                    
                    if (publish_counter++ % 2 == 0) {
                        std::array<double, 6> current_pose;
                        if(robot->getStateData(RtSupportedFields::tcpPoseAbc_m, current_pose) == 0) {
                            std::lock_guard<std::mutex> lock(pose_data_mutex_);
                            latest_current_pose_ = current_pose;
                            latest_target_pose_ = target_pose;
                            publish_realtime_pose(latest_current_pose_, latest_target_pose_);
                        }
                    }

                    index++;
                } else {
                    output.setFinished();
                    stopManually.store(false);
                }
                
                return output;
            };

            rtCon->setControlLoop(callback, 0, true);
            rtCon->startLoop(false);

            // 控制循环
            while(stopManually.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            
            // 停止接收机器人状态数据
            rtCon->stopLoop();
            rtCon->stopMove();
            robot->stopReceiveRobotState();
            pose_timer_->reset();
            RCLCPP_INFO(this->get_logger(), "垂直+对角轨迹控制完成\n-------------------------------");

        } catch (const std::exception &e) {
            pose_timer_->reset();
            robot->stopReceiveRobotState(); // 确保停止接收数据
            rtCon->stopLoop();
            rtCon->stopMove();
            RCLCPP_ERROR(this->get_logger(), "垂直+对角轨迹控制错误: %s", e.what());
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
    void Rokae_Move::force_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(force_data_mutex_);
        if(msg->data.size() >= 3) {
            latest_force_data_[0] = msg->data[0];
            latest_force_data_[1] = msg->data[1];
            latest_force_data_[2] = msg->data[2];
        }
        force_data_cv_.notify_one();
    }

    /**
     * @brief 发布实时位姿数据到话题。本质上是发布current_pose的前三位和target_pose的前三位，组成一个六位数据，
     *        可以自定义传入参数从而改变发送数据。
     * @param current_pose 当前位姿
     * @param target_pose 目标位姿
     */
    void Rokae_Move::publish_realtime_pose(const std::array<double, 6>& current_pose, 
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

    /**
     * @brief 初始位资发布。用于：让plotjuggler读取到初始位资，不然只有在callback运行时才会读取到。
     */
    void Rokae_Move::pubilsh_initial_pose(){
        try{
            // 获取当前位姿
            std::array<double, 6> current_pose = robot->posture(rokae::CoordinateType::flangeInBase, ec);

            // 发布当前位姿(目标位姿设置为当前位姿)
            std::lock_guard<std::mutex> lock(pose_data_mutex_);
            publish_realtime_pose(current_pose, current_pose);
        }catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Error publishing initial pose: %s", e.what());
        }
    }

    // 计算力控制调整量
    double Rokae_Move::ForceController::calculateAdjustment(double current_force) {
        force_error = desired_force_z - current_force;
        force_integral += force_error;
        double adjustment = kp * force_error + ki * force_integral;
        return std::clamp(adjustment, -max_adjust, max_adjust);
    }

    void Rokae_Move::ForceController::reset() {
        force_integral = 0.0;
        force_error = 0.0;
    }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<Rokae_Move>("Rokae_Move");
    
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
