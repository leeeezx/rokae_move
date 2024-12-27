#include <string.h>
// rclcpp库
#include "rclcpp/rclcpp.hpp"
// 基本消息类型库
#include "std_msgs/msg/string.hpp"
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



// 开始使用珞石SDK
using namespace rokae;
using namespace std;

// 占位符,下面会详细说
using std::placeholders::_1;

// 创建一个类节点，起名叫做ROKAE_MOVE,继承自Node,这样就能使用Node所有的功能了
class ROKAE_MOVE : public rclcpp::Node
{
public:
  // 构造函数
  ROKAE_MOVE(const std::string &name) : Node(name)
  {

    RCLCPP_INFO(this->get_logger(), "---Rokae机械臂ROS2接口---%s@作者penzi", name.c_str());
    sub_yolo_predict = this->create_subscription<std_msgs::msg::String>("/yolo/prediction/item_dict", 10, std::bind(&ROKAE_MOVE::predict_callback, this, std::placeholders::_1));
    keyborad = this->create_subscription<std_msgs::msg::String>("/keystroke", 10, std::bind(&ROKAE_MOVE::keyborad_callback, this, _1));
    // 此为测试矩阵的初始化

    this->declare_parameter("safe_dis", 0.08);
    pub_gripper_command = this->create_publisher<std_msgs::msg::String>("/gripper_command", 10);
    // this->declare_parameter("cartesian_point_start", "0.455 0.0 0.439");
    this->declare_parameter("cartesian_point_start", "0.455017 0.0 0.438");
    this->declare_parameter("cartesian_trans", "0.51143 0.00410 0.5217 3.14154 0.0 3.14154"); // pingmian
    // this->declare_parameter("cartesian_point_end", "0.455 0.15 0.487");//pingmian
    //  this->declare_parameter("cartesian_point_start", "0.455 0 0.57");
    //  this->declare_parameter("cartesian_point_end", "0.455 0.15 0.586");//qiumian
    //  this->declare_parameter("cartesian_point_start", "0.455 0 0.558");
    //  this->declare_parameter("cartesian_point_end", "0.455 0.15 0.586");//humian
    this->declare_parameter("press_dis", 0.05);
    this->declare_parameter("grasp_dis", 0.146); // guanzi
    // this->declare_parameter("grasp_dis", 0.135);//qiumian
    // this->declare_parameter("grasp_dis", 0.138);//humian
    this->declare_parameter("grasp_msg", true); // pingmian
    try
    {
      std::string remoteIP = "192.168.0.160";
      std::string localIP = "192.168.0.10";
      robot = std::make_shared<xMateErProRobot>(remoteIP, localIP);

      RCLCPP_INFO(this->get_logger(), "---已连接到Rokae机械臂接口, 正在进行初始化---");
      robot->setOperateMode(rokae::OperateMode::automatic, ec);
      // 若程序运行时控制器已经是实时模式，需要先切换到非实时模式后再更改网络延迟阈值，否则不生效
      robot->setRtNetworkTolerance(20, ec);
      robot->setMotionControlMode(MotionControlMode::RtCommand, ec); // 实时模式
      robot->setPowerState(true, ec);
      RCLCPP_INFO(this->get_logger(), "---Robot powered on !---");
      auto rtCon = robot->getRtMotionController().lock();
      RCLCPP_INFO(this->get_logger(), "---Robot initialization completed---");
      std::array<double, 7> q_drag_xm7p = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};
      rtCon->MoveJ(0.5, robot->jointPos(ec), q_drag_xm7p);
      RCLCPP_INFO(this->get_logger(), "---Robot initial pose completed---");
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what();
    }
  }

  ~ROKAE_MOVE()
  {
    robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    robot->setOperateMode(rokae::OperateMode::manual, ec);
    robot->setPowerState(false, ec);
    RCLCPP_INFO(this->get_logger(), "---珞石机械臂运动节点已关闭---.");
  }

  void track(const std::string &name)
  {
    try
    {

      RCLCPP_INFO(this->get_logger(), "Start tracking %s ...", name.c_str());
      CartesianPosition start, target;
      auto rtCon = robot->getRtMotionController().lock();
      posture = robot->posture(CoordinateType::flangeInBase, ec);
      RCLCPP_INFO(this->get_logger(), "TCP---xyzabc : %lf %lf %lf %lf %lf %lf ", posture[0], posture[1], posture[2], posture[3], posture[4], posture[5]);
      // 直线运动
      Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
      Eigen::Matrix4d startPosMatrix = convertArrayToEigenMatrix(start.pos);
      highest_confidence_position(2, 3) -= safe_distance;                             // 默认8厘米安全距离
      Eigen::Matrix4d targetPosMatrix = startPosMatrix * highest_confidence_position; // 这里是右乘gTo,公式为bTo = bTg * gTo
      targetPosMatrix.topLeftCorner(3, 3) = startPosMatrix.topLeftCorner(3, 3);       // 开始位姿的姿态全部传给taget，未来可以根据6d位姿估计改变其姿态，目前只沿用开始姿态
      target.pos = convertEigenMatrixToArray(targetPosMatrix);
      print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);
      rtCon->MoveL(0.1, start, target);
      posture = robot->posture(CoordinateType::flangeInBase, ec);
      RCLCPP_INFO(this->get_logger(), "TCP---xyzabc : %lf %lf %lf %lf %lf %lf ", posture[0], posture[1], posture[2], posture[3], posture[4], posture[5]);
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  void move_init()
  {
    try
    {
      auto rtCon = robot->getRtMotionController().lock();
      std::array<double, 7> q_drag_xm7p = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};
      rtCon->MoveJ(0.5, robot->jointPos(ec), q_drag_xm7p);
      RCLCPP_INFO(this->get_logger(), "---Back to initial pose !---.");
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
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

  Eigen::Matrix4d access_grouped_data(const std::string &item_class)
  {
    double highest_confidence = -1.0;

    // 检查是否存在该类别
    if (grouped_data.find(item_class) != grouped_data.end())
    {
      // 获取当前类别下的物体向量
      const auto &items = grouped_data[item_class];

      // 遍历该类别下的所有物体
      for (size_t i = 0; i < items.size(); ++i)
      {
        const auto &item = items[i];
        double confidence = std::get<0>(item);
        const Eigen::Matrix4d &position = std::get<1>(item);

        // 如果当前物体的置信度高于之前找到的物体，更新最高置信度和位置
        if (confidence > highest_confidence)
        {
          highest_confidence = confidence;
          highest_confidence_position = position;
        }

        std::stringstream ss;
        ss << position.format(Eigen::IOFormat(Eigen::FullPrecision));
        // RCLCPP_INFO(this->get_logger(), "  Item %zu: Confidence: %.1f, Position:\n%s", i, confidence, ss.str().c_str());
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Class %s does not exist in the grouped data.", item_class.c_str());
      highest_confidence_position = Eigen::MatrixXd::Identity(4, 4); // 如果没有那就是单位矩阵
    }

    // 返回置信度最高的物体的位置
    std::stringstream ss;
    ss << highest_confidence_position.format(Eigen::IOFormat(Eigen::FullPrecision));
    RCLCPP_INFO(this->get_logger(), " \nThe Most confidence Position:\n%s", ss.str().c_str());

    return highest_confidence_position;
  }

private:
  std::array<double, 6> posture = {};
  std::string key = {};
  std::string target_class;
  std::unordered_map<std::string, std::vector<std::tuple<double, Eigen::Matrix4d>>> grouped_data;
  Eigen::Matrix4d highest_confidence_position;
  Eigen::Matrix4d test_matrix = Eigen::Matrix4d::Identity();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_yolo_predict;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyborad;
  std::shared_ptr<xMateErProRobot> robot;
  std::error_code ec;
  double safe_distance;  // 定义安全距离，通过ros2 param 控制,默认0.08
  double press_distance; // 定义下压距离，通过ros2 param 控制,默认0.05
  double grasp_distance; // 定义抓取距离，通过ros2 param 控制,默认0.14

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_gripper_command; // 发布者,用于发送到"/gripper_command"主题
  std::string cartesian_points_string1;
  std::vector<float> cartesian_points_vec1;
  std::string cartesian_trans_string;
  std::vector<float> cartesian_trans;
  bool grasp_msg = true;

  // 订阅者回调函数(有参数, 参数类型跟上面订阅者订阅的参数类型相同, 注意要加上::SharedPtr, 因为传进来的是一个指针)
  void predict_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "收到YOLO_predict的消息oTg---%s", msg->data.c_str());
    // 使用nlohmann/json库解析JSON字符串
    nlohmann::json j = nlohmann::json::parse(msg->data);
    grouped_data.clear();
    // 使用无序映射按类别分组数据

    // 遍历JSON对象中的每个项目
    for (auto &item : j.items())
    {
      std::string item_class = item.value()["class"];
      double item_confidence = item.value()["confidence"];
      auto item_position = item.value()["position"];

      // 将 position 数据存储在 Eigen::Matrix4d 中
      Eigen::Matrix4d position_matrix;
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          position_matrix(i, j) = item_position[i][j];
        }
      }

      // 将当前项目添加到对应类别的向量中
      grouped_data[item_class].emplace_back(item_confidence, position_matrix);
    }
  }

  std::string keyborad_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到键盘按下的消息---%s", msg->data.c_str());
    key = msg->data.c_str();
    this->get_parameter("cartesian_point_start", cartesian_points_string1);
    this->get_parameter("cartesian_trans", cartesian_trans_string);
    this->get_parameter("press_dis", press_distance);
    this->get_parameter("safe_dis", safe_distance);
    cartesian_points_vec1 = string_to_vec(cartesian_points_string1);
    cartesian_trans = string_to_vec(cartesian_trans_string);
    switch (key[0])
    {
    case 'b':
      whether2track("bottle");
      break;
    case 'p':
      whether2track("person");
      break;
    case 'c':
      whether2track("cell phone");
      break;
    case 'o':
      whether2track("orange");
      break;
    case 'u':
      whether2track("cup");
      break;
    case 'a':
      whether2track("apple");
      break;
    case 't':
      test_matrix(0, 3) = 0.2;
      test_matrix(1, 3) = 0.3;
      test_matrix(2, 3) = 0.2;
      highest_confidence_position = test_matrix;
      track("test_matrix");
      break;
    case '`':
      move_init();
      break;
    case 'e':
      move_enableDrag();
      break;
    case 'd':
      move_disableDrag();
      break;
    case '/':
      grasp();
      break;
    case 'q':
      cout << "vector容器大小: " << cartesian_points_vec1.size() << endl;
      if (cartesian_points_vec1.size() != 3)
      {
        cout << "Error : 应该输入3个数字且之间用空格连接" << endl;
        break;
      }
      else
      {
        for (auto ff : cartesian_points_vec1)
          cout << ff << ",";
        cout << endl;
        go2cartesian1(cartesian_points_vec1);
        break;
      }
    case 'w':
      cout << "vector容器大小: " << cartesian_trans.size() << endl;
      if (cartesian_trans.size() != 6)
      {
        cout << "Error : 应该输入6个数字且之间用空格连接 x y z rx ry rz" << endl;
        break;
      }
      else
      {
        for (auto ff : cartesian_trans)
          cout << ff << ",";
        cout << endl;
        go2cartesian_trans(cartesian_trans);
        break;
      }
    case '.':
      go2cartesian3();
      break;

    default:
      RCLCPP_INFO(this->get_logger(), "---目前还不支持这个类别");
      break;
    }

    return key;
  }
  void whether2track(const std::string &object_class) ////安全保障，没检测到就不进入追踪函数
  {
    target_class = object_class;
    highest_confidence_position = access_grouped_data(object_class);
    if (highest_confidence_position != Eigen::MatrixXd::Identity(4, 4))
    {
      track(object_class);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "---没有检测到%s", object_class.c_str());
    }
  }

  std::array<double, 16> convertEigenMatrixToArray(const Eigen::Matrix4d &matrix) // 将eigen矩阵转换为rokae支持的array矩阵
  {
    std::array<double, 16> result;
    int index = 0;
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        result[index++] = matrix(i, j);
      }
    }
    return result;
  }
  Eigen::Matrix4d convertArrayToEigenMatrix(const std::array<double, 16> &arr) // 将rokae支持的array矩阵转换为eigen矩阵
  {
    Eigen::Matrix4d result;
    int index = 0;
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        result(i, j) = arr[index++];
      }
    }
    return result;
  }
  std::vector<float> string_to_vec(const std::string &str)
  {
    std::vector<float> vec;
    std::stringstream ss(str);
    std::string buf;
    while (ss >> buf)
      vec.push_back(atof(buf.c_str()));
    return vec;
  }
  void go2cartesian1(const std::vector<float> &car_vec)
  {
    try
    {

      RCLCPP_INFO(this->get_logger(), "Go To Start");
      CartesianPosition start, target;

      Eigen::Matrix3d rot_start;
      Eigen::Vector3d trans_start, trans_end;
      auto rtCon = robot->getRtMotionController().lock();

      /////////////////////////////////////
      Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
      Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
      trans_end = trans_start;
      // go to cartesian
      trans_end[0] = car_vec[0];
      trans_end[1] = car_vec[1];
      trans_end[2] = car_vec[2] + 0.1;
      Utils::transMatrixToArray(rot_start, trans_end, target.pos);

      print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);

      rtCon->MoveL(0.1, start, target);
      //////////////////////////////////////////
      sleep(2); //
      Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
      Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);

      trans_end = trans_start;
      // go to cartesian
      trans_end[0] = car_vec[0];
      trans_end[1] = car_vec[1];
      trans_end[2] = car_vec[2];
      Utils::transMatrixToArray(rot_start, trans_end, target.pos);

      print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);
      rtCon->MoveL(0.02, start, target);
      // 下压然后立马抬高
      ////////////////////////////////////////
      // sleep(10);
      sleep(0.2); //
      print(std::cout, "完成到达笛卡尔空间点位\n");

      //////////////////////////////////
      // sleep(0.2);

      Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
      Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
      trans_end = trans_start;
      // go to cartesian
      trans_end[2] += 0.05;
      Utils::transMatrixToArray(rot_start, trans_end, target.pos);

      print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);

      rtCon->MoveL(0.3, start, target);
      print(std::cout, "\033[32m--- Ready to grasp !---\033[0m");
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  void go2cartesian_trans(const std::vector<float> &car_vec)
  {
    try
    {
      RCLCPP_INFO(this->get_logger(), "Go To Cartesian trans with rx ry rz...");
      CartesianPosition start, target;

      auto rtCon = robot->getRtMotionController().lock();
      RCLCPP_INFO(this->get_logger(), "robot get");
      Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
      // vector2array
      std::array<double, 6UL> car_vec_array = {};
      RCLCPP_INFO(this->get_logger(), "car_vec_array");
      for (uint64_t index = 0; index < car_vec.size(); index++)
        car_vec_array[index] = car_vec[index];
      print(std::cout, "MoveL start position:", car_vec_array);
      Utils::postureToTransArray(car_vec_array, target.pos);

      print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);
      rtCon->MoveL(0.03, start, target);
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  void go2cartesian3()
  {
    try
    {
      RCLCPP_INFO(this->get_logger(), "\033[32mpress\033[0m");
      CartesianPosition start, target;

      Eigen::Matrix3d rot_start;
      Eigen::Vector3d trans_start, trans_end;
      auto rtCon = robot->getRtMotionController().lock();

      /////////////////////////////////////
      Utils::postureToTransArray(robot->posture(rokae::CoordinateType::flangeInBase, ec), start.pos);
      Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);
      trans_end = trans_start;
      // go to cartesian

      trans_end[2] += 0.045;
      Utils::transMatrixToArray(rot_start, trans_end, target.pos);

      print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);

      rtCon->MoveL(0.05, start, target);
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }
  void grasp()
  {
    this->get_parameter("grasp_dis", grasp_distance);
    if (grasp_msg == true)
    {

      RCLCPP_INFO(this->get_logger(), "Start pressing ---grasp_msg is %d", grasp_msg);
    }
    else
    {
      grasp_distance = -grasp_distance;
      RCLCPP_INFO(this->get_logger(), "Start rising ---grasp_msg is %d", grasp_msg);
    }

    try
    {

      CartesianPosition start, target;
      Eigen::Matrix3d rot_start;
      Eigen::Vector3d trans_start, trans_end;
      auto rtCon = robot->getRtMotionController().lock();
      Utils::postureToTransArray(robot->posture(rokae::CoordinateType::endInRef, ec), start.pos);
      Utils::arrayToTransMatrix(start.pos, rot_start, trans_start);

      trans_end = trans_start;
      trans_end[2] -= grasp_distance;
      Utils::transMatrixToArray(rot_start, trans_end, target.pos);

      print(std::cout, "MoveL start position:", start.pos, "Target:", target.pos);
      rtCon->MoveL(0.01, start, target);
      grasp_msg = !grasp_msg;
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROKAE_MOVE>("ROKAE_MOVE");
  // spin函数: 一旦进入spin函数，相当于它在自己的函数里面死循环了。只要回调函数队列里面有callback函数在，它就会马上去执行callback函数。如果没有的话，它就会阻塞，不会占用CPU。注意不要再spin后面放其他东西, 他们都不会执行的
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
