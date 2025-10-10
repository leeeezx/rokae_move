#include "rokae_node/rokae_move_node.hpp"
#include "rokae_move/robot.h"
#include "rokae_move/motion_control_rt.h"



// 开始使用珞石SDK
using namespace rokae;
using namespace std;

// -----------------------------------------------------------------------------------------------------------
// ---------------------------------------------------公共成员函数----------------------------------------------
// -----------------------------------------------------------------------------------------------------------


/** 
 * @brief 构造函数，初始化rokae_move节点。
 * @param name 节点名称
 * @note 构造函数与析构函数的名字必须与类名相同
 */
Rokae_Move::Rokae_Move(std::string name) : Node(name)
{
    initialize_robot();        // 初始化机械臂
    setup_ros_communications(); // 设置ROS通信

    // 创建并初始化机器人控制器，将硬件连接和当前节点指针传给它
    if (robot && rtCon) {
        robot_controller_ = std::make_unique<RobotController>(robot, rtCon, this);
    } else {
        RCLCPP_FATAL(this->get_logger(), "Robot SDK objects 不存在，初始化可能失败, 无法创建 RobotController.");
        rclcpp::shutdown();
    }
}

/**
 * @brief 析构函数，关闭节点时进行清理工作。即按下ctrl+c后会执行的操作。
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
    this->declare_parameter("velocity", 0.1);
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

    // // 订阅
    // // 订阅 force_sensor_data 话题topic，使force_subscription_指向本订阅者(智能指针)
    // z_force_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    //     "force_sensor_z", // 要订阅的topic的名称
    //     10, // 队列深度（QoS）。表示最多缓存多少条未处理的消息。
    //     std::bind(&Rokae_Move::z_force_callback, this, std::placeholders::_1)); // 回调函数。当订阅收到消息时，就会调用这个回调函数
    
    // 发布
    // 添加实时位姿数据发布者
    realtime_pose_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "realtime_robot_pose", // 要发布的topic的名称 
        10); // 队列深度（QoS）。表示最多缓存多少条未处理的消息。

    // realtime_FandTau_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
    //     "realtime_robot_FandTau", 
    //     10);
    realtime_FandTau_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>( 
        "realtime_robot_FandTau", 
        10); // 这个是加了stamped的

    // 定时器原理：每隔设定的时间周期，就会调用一次指定的回调函数，回调函数中还可以进一步调用其他函数，例如调用发布函数，发布的相关数据都与最终调用的发布函数有关
    // 创建位姿数据发布定时器(10Hz)
    pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Rokae_Move::publish_initial_pose, this));
    
    FandTau_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&Rokae_Move::publish_initial_ext_FandTau, this));

}


/**
 * @brief 初始化机械臂，包括连接、上电和控制模式设置等。均调用珞石SDK的API实现。
 * 各注意事项见官方文档
 */
void Rokae_Move::initialize_robot()
{
    try
    {
        // 连接到机械臂
        std::string remoteIP = "192.168.0.160";
        std::string localIP = "192.168.0.10";
        robot = std::make_shared<xMateErProRobot>(remoteIP, localIP);

        RCLCPP_INFO(this->get_logger(), "---已连接到Rokae机械臂接口, 正在进行初始化---");

        // 机械臂初始化设置
        robot->setRtNetworkTolerance(50, ec); // 网络延迟。若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效

        auto robotinfo = robot->robotInfo(ec);
        RCLCPP_INFO(this->get_logger(), "控制器版本号:%s,机型:%s", robotinfo.version.c_str(), robotinfo.type.c_str());
        RCLCPP_INFO(this->get_logger(), "xCore-SDK 版本: %s", robot->sdkVersion().c_str());
        
        robot->setOperateMode(rokae::OperateMode::automatic, ec); // 操作模式。自动
        robot->setMotionControlMode(MotionControlMode::RtCommand, ec); // 控制模式：实时模式

        robot->setPowerState(true, ec); // 上电
        RCLCPP_INFO(this->get_logger(), "---机器人上电成功！操作模式：自动，控制模式：实时---");

        // 初始化 rtCon
        // 获取robot中的实时运动控制器对象getRtMotionController()，通过lock()将其转化为强指针并赋值给rtCon
        rtCon = robot->getRtMotionController().lock(); 
        RCLCPP_INFO(this->get_logger(), "---机器人初始化完成---");

    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(this->get_logger(), "机器人初始化失败: %s", e.what());
        // 可以选择在这里关闭节点
        rclcpp::shutdown();
    }
}


/**
 * @brief 使用键盘按键进行相应的控制功能
 */
void Rokae_Move::keyborad_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // ！！！键盘按键触发后立刻获取需要的参数的最新值。这些函数会在rqt中受操作者改动。！！！
    // ！！！所以必须在每次触发时获取最新值。！！！当前方式比较直接，后续可以改为参数回调函数。
    // TODO: 改为参数回调函数
    this->get_parameter("cartesian_point", cartesian_points_string);
    double velocity = this->get_parameter("velocity").as_double();

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

    double hold_duration;
    
    // 收到键盘消息
    RCLCPP_INFO(this->get_logger(), "收到键盘按下的消息---%s", msg->data.c_str());
    std::string key = msg->data;
    
    switch (key[0])
    {
    case 'q':
        points_array = string_to_array(cartesian_points_string);
        if (points_array.size() == 6) {
            // std::cout << " We will go to -> " << points_array << std::endl;
            robot_controller_->move_command(points_array, velocity); // 委托
        } else {
            RCLCPP_ERROR(get_logger(), "Error : 应该输入6个数字且之间用空格连接");
        }
        break;
    case 'w':
        cout << "Misson  : Start cartesian impedance controller and press down 0.05m " << endl;
        cout << "Waiting for 1 second and pushing back 0.3m" << endl;
        robot_controller_->cartesian_impedance_control(desired_force, first_time, second_time);
        break;
    case 'e':
        robot_controller_->move_enableDrag();
        break;
    case 'd':
        robot_controller_->move_disableDrag();
        break;
    case '`':
        robot_controller_->move_init();
        break;
    case 'r':
        RCLCPP_INFO(this->get_logger(), "启动实时轨迹控制");
        robot_controller_->usr_rt_cartesian_control(10.0, 10.0);
        break;
    case 'v':
        RCLCPP_INFO(this->get_logger(), "启动速度控制");
        robot_controller_->usr_rt_cartesian_v_control_z(air_dist, cruise_dist, decel_dist, target_speed);
        break;
    case 'b':  // 新增键位支持复合轨迹控制
        RCLCPP_INFO(this->get_logger(), "启动速度控制（Z-Y复合轨迹控制）");
        robot_controller_->usr_rt_cartesian_v_control(
            air_dist, cruise_dist, decel_dist, target_speed,
            y_air_dist, y_cruise_dist, y_decel_dist, y_target_speed, y_direction  // 默认Y轴参数
        );
        break;
    case 'n': // 新增键位支持垂直+对角轨迹控制
        RCLCPP_INFO(this->get_logger(), "启动垂直下压+对角线运动控制（角度:%.1f度）", gamma_angle);
        robot_controller_->usr_rt_vertical_diagonal_control(
            air_dist, cruise_dist, decel_dist, target_speed,  // 垂直段参数
            diagonal_air_dist, diagonal_cruise_dist, diagonal_decel_dist, diagonal_target_speed,  // 对角线段参数
            gamma_angle  // 对角线角度
        );
        break;
    case 't': // 用于测试布尔原子变量是否可以传递进机器人回调函数中
        robot_controller_->usr_rt_stationary_control(hold_duration = 20);
        break;
    default:
        RCLCPP_INFO(this->get_logger(), "你在狗叫什么");
        break;
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


// =====================================================================================================
// =========================================== 发布接受信息相关 ===========================================
// =====================================================================================================

/**
 * @brief 力数据订阅回调函数，用来获取最新的力数据                          未使用
 * @param msg 力数据消息
 */
void Rokae_Move::z_force_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_force_z_.store(msg->data); 
    // RCLCPP_INFO(this->get_logger(), "接收到力数据: %.2f N", msg->data);
    // RCLCPP_INFO(this->get_logger(), "=================ros订阅者接收到力数据: %.2f N=====================", latest_force_z_.load());
}

// 让这个函数去检查力阈值，然后让callback来调用这个函数，这个函数应该返回一个bool。             未使用
bool Rokae_Move::z_force_check(double force_threshold)
{
    // RCLCPP_INFO(this->get_logger(),"进入z_force_check");
    constexpr double DEFAULT_FORCE_THRESHOLD = 2.0;
    const double threshold = force_threshold > 0.0 ? force_threshold : DEFAULT_FORCE_THRESHOLD;
    // RCLCPP_INFO(this->get_logger(),"开始判断");
    bool is_triggered = latest_force_z_.load() >= threshold;
    // RCLCPP_INFO(this->get_logger(), "调用z_force_check成功， 当前力: %.2f N, 阈值: %.2f N", latest_force_z_.load(), threshold);
    return is_triggered;
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


void Rokae_Move::publish_realtime_pose_extFTau(const std::array<double, 6>& current_pose, 
                        const std::array<double, 6>& current_ext_tau) 
{
    std_msgs::msg::Float32MultiArray msg;
    // 将当前位姿和目标位姿打包到一起发布
    // [current_x, current_y, current_z, target_x, target_y, target_z]
    msg.data = {
        static_cast<float>(current_pose[0]),
        static_cast<float>(current_pose[1]),
        static_cast<float>(current_pose[2]),
        static_cast<float>(current_ext_tau[0]),
        static_cast<float>(current_ext_tau[1]),
        static_cast<float>(current_ext_tau[2]),
        static_cast<float>(current_ext_tau[3]),
        static_cast<float>(current_ext_tau[4]),
        static_cast<float>(current_ext_tau[5])
    };
    realtime_pose_publisher_->publish(msg);
}

/**
 * @brief 发布实时外部力/力矩数据到话题
 * @param current_ext_tau 当前外部力/力矩数据
 */
void Rokae_Move::publish_realtime_ext_FandTau(const std::array<double, 6>& current_ext_tau)
{
    // std_msgs::msg::Float32MultiArray msg;
    geometry_msgs::msg::WrenchStamped msg; // 使用WrenchStamped消息类型
    msg.header.stamp = this->now(); // 设置时间戳
    
    // 普通Float32MultiArray消息设置
    // msg.data = {
    //     static_cast<float>(current_ext_tau[0]),
    //     static_cast<float>(current_ext_tau[1]),
    //     static_cast<float>(current_ext_tau[2]),
    //     static_cast<float>(current_ext_tau[3]),
    //     static_cast<float>(current_ext_tau[4]),
    //     static_cast<float>(current_ext_tau[5])
    // };
    // Wrench消息设置
    msg.wrench.force.x = current_ext_tau[0];
    msg.wrench.force.y = current_ext_tau[1];
    msg.wrench.force.z = current_ext_tau[2];
    msg.wrench.torque.x = current_ext_tau[3];
    msg.wrench.torque.y = current_ext_tau[4];
    msg.wrench.torque.z = current_ext_tau[5];

    realtime_FandTau_publisher_->publish(msg);
}

/**
 * @brief 初始位姿发布。用于：让plotjuggler读取到初始位资，不然只有在callback运行时才会读取到。
 */
void Rokae_Move::publish_initial_pose(){
    try{
        // 获取当前位姿
        std::array<double, 6> current_pose = robot->posture(rokae::CoordinateType::flangeInBase, ec);

        // 发布当前位姿(目标位姿设置为当前位姿)
        publish_realtime_pose(current_pose, current_pose);
    }catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Error publishing initial pose: %s", e.what());
    }
}


/**
 * @brief 初始位姿发布。用于：让plotjuggler读取到初始位资，不然只有在callback运行时才会读取到。
 */
void Rokae_Move::publish_initial_ext_FandTau(){
    try{
        // 声明所有需要的输出参数
        std::array<double, 7> joint_torque_measured;      // 各轴测量力
        std::array<double, 7> external_torque_measured;   // 各轴外部力
        std::array<double, 3> cart_torque;                 // 笛卡尔空间力矩 [X, Y, Z]
        std::array<double, 3> cart_force;                  // 笛卡尔空间力 [X, Y, Z]

        robot->getEndTorque(
            FrameType::flange,           // 或 FrameType::flange, FrameType::tool
            joint_torque_measured,      // 输出：各轴测量力
            external_torque_measured,   // 输出：各轴外部力
            cart_torque,                // 输出：笛卡尔空间力矩
            cart_force,                 // 输出：笛卡尔空间力
            ec                          // 输出：错误码
        );

        // 检查是否有错误
        if(ec) {
            RCLCPP_WARN(this->get_logger(), "获取力/力矩数据失败: %s", ec.message().c_str());
            return;
        }

        std::array<double, 6> combined_force_torque;
        combined_force_torque[0] = -cart_force[0]; // 取反
        combined_force_torque[1] = cart_force[1]; 
        combined_force_torque[2] = -cart_force[2]; // 取反
        combined_force_torque[3] = cart_torque[0];
        combined_force_torque[4] = cart_torque[1];
        combined_force_torque[5] = cart_torque[2];

        // 发布当前位姿(目标位姿设置为当前位姿)
        publish_realtime_ext_FandTau(combined_force_torque);
    }catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Error publishing initial pose: %s", e.what());
    }
}

// TODO 将位姿与外部力一起初始发布
/**
 * @brief 初始位姿与关节力矩发布。用于：让plotjuggler读取到初始位资，不然只有在callback运行时才会读取到。
 */
void Rokae_Move::publish_initial_pose_extFTau(){
    try{
        // 获取当前位姿
        std::array<double, 6> current_pose = robot->posture(rokae::CoordinateType::flangeInBase, ec);
        // std::array<double, 6> current_joint_torque = robot->jointTorque(ec);

        // 发布当前位姿(目标位姿设置为当前位姿)
        publish_realtime_pose_extFTau(current_pose, current_pose);
    }catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Error publish_initial_pose_JointTau: %s", e.what());
    }
}




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // 创建节点
    auto node = std::make_shared<Rokae_Move>("Rokae_Move");

    // 使用多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin(); // 执行器会用它的线程池来处理所有回调
    
    rclcpp::shutdown();
    return 0;
}
