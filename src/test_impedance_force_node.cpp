/*
文件名: test_impedance_force_node.cpp
作者: 乐正祥
创建日期: 忘记了 // 请根据实际修改更新
修改日期: 2025-02-18
描述: 
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

    /**
     * @brief 生成一段线性轨迹
     * @param duration 轨迹持续时间
     * @return trajectory 轨迹
     */
    static std::vector<std::array<double, 6>> LinearSegmentPath_one(
        const std::array<double, 6>& start,
        const std::array<double, 6>& end,
        double duration) 
    {
        // 使用5个控制点强制直线运动
        std::vector<std::array<double, 6>> controlPoints = {
            start,
            start,
            start,
            end,
            end
        };

        // 生成轨迹
        auto trajectory = generateBezierTrajectory(controlPoints, duration);

        return trajectory;
    }    


    /**
     * @brief 四阶段对角线轨迹生成(Z-Y平面特定角度，加速+匀速+减速+停止)
     * @param start 起始点
     * @param air_dist 加速段距离
     * @param cruise_dist 匀速段距离
     * @param decel_dist 减速段距离
     * @param target_speed 目标速度
     * @param gamma_deg 角度gamma(度)，Y轴负方向为0度，Z轴负方向为90度
     */
    static std::vector<std::array<double, 6>> generateDiagonalPhasePath(
        const std::array<double, 6>& start,
        double air_dist,
        double cruise_dist,
        double decel_dist,
        double target_speed,
        double gamma_deg)
    {
        // 参数校验
        const double EPSILON = 1e-10;
        if(air_dist <= EPSILON || cruise_dist <= EPSILON || decel_dist <= EPSILON || target_speed <= EPSILON)
            throw std::invalid_argument("距离和速度参数必须为正数");
        if(gamma_deg < 0 || gamma_deg > 90)
            throw std::invalid_argument("角度gamma必须在0-90度之间");

        // 将角度转换为弧度
        double gamma_rad = gamma_deg * M_PI / 180.0;
        
        // 使用固定时间步长进行欧拉积分仿真
        double dt = 0.001;
        std::vector<std::array<double, 6>> trajectory;
        
        // 计算y和z方向的分量
        double cos_gamma = std::cos(gamma_rad);  // Y轴分量比例
        double sin_gamma = std::sin(gamma_rad);  // Z轴分量比例
        
        // 初始状态：位置、速度均为0
        double pos = 0.0;      // 从起始点到当前的累计位移(沿对角线方向)
        double v = 0.0;        // 当前速度
        auto point = start;
        trajectory.push_back(point);
        
        // 阶段1：加速（从0加速到 target_speed）
        double a1 = target_speed * target_speed / (2 * air_dist);
        double t1 = target_speed / a1;
        for(double t = 0; t < t1; t += dt) {
            v += a1 * dt;       // 累计加速
            pos += v * dt;      // 更新位移
            point = start;
            
            // 注意：这里Y轴负方向移动，Z轴负方向移动，所以都减去位移
            point[1] = start[1] - pos * cos_gamma;
            point[2] = start[2] - pos * sin_gamma;
            trajectory.push_back(point);
        }
        
        // 阶段2：匀速运动（以 target_speed 匀速前进 cruise_dist）
        v = target_speed; // 保持匀速
        double cruise_time = cruise_dist / target_speed;
        double cruise_start_pos = pos; // 记录匀速阶段起始累计位移
        
        for(double t = 0; t < cruise_time; t += dt) {
            pos += v * dt;
            point = start;
            
            point[1] = start[1] - pos * cos_gamma;
            point[2] = start[2] - pos * sin_gamma;
            trajectory.push_back(point);
        }
        
        // 阶段3：减速缓冲（从 target_speed 减速到0）
        double a3 = target_speed * target_speed / (2 * decel_dist);
        double decel_pos = 0.0;  // 本阶段累计位移
        double decel_start_pos = pos; // 记录减速阶段起始累计位移
        v = target_speed;
        
        while(decel_pos < decel_dist && v > 1e-6) {
            v -= a3 * dt;
            if(v < 0) {
                v = 0;
            }
            double dp = v * dt;
            decel_pos += dp;
            pos += dp;
            point = start;
            
            point[1] = start[1] - pos * cos_gamma;
            point[2] = start[2] - pos * sin_gamma;
            trajectory.push_back(point);
        }
        
        // 最终位置补偿：确保最终位姿精确
        double total_dist = air_dist + cruise_dist + decel_dist;
        double final_y = start[1] - total_dist * cos_gamma;
        double final_z = start[2] - total_dist * sin_gamma;
        
        if (std::abs(trajectory.back()[1] - final_y) > 1e-3 || 
            std::abs(trajectory.back()[2] - final_z) > 1e-3) {
            point = trajectory.back();
            point[1] = final_y;
            point[2] = final_z;
            trajectory.push_back(point);
        }
        
        std::cout << "对角轨迹生成完成，角度: " << gamma_deg << "度" << std::endl;
        return trajectory;
    }


    /**
     * @brief 四阶段侵入轨迹生成（空中加速+匀速侵入+减速缓冲+最终停止）
     * @param start 起始点
     * @param air_dist 空中加速段长度（到沙土表面的距离）
     * @param cruise_dist 匀速侵入段长度 
     * @param decel_dist 减速缓冲段长度
     * @param target_speed 目标侵入速度
     */
    static std::vector<std::array<double, 6>> generateFourPhasePath(
        const std::array<double, 6>& start,
        double air_dist,
        double cruise_dist,
        double decel_dist,
        double target_speed)
    {
        // 参数校验
        const double EPSILON = 1e-10;
        if(air_dist <= EPSILON || cruise_dist <= EPSILON || decel_dist <= EPSILON)
            throw std::invalid_argument("距离参数必须为正数");

        // 使用固定时间步长进行欧拉积分仿真
        double dt = 0.001;
        std::vector<std::array<double, 6>> trajectory;
        
        // 初始状态：位置、速度均为0（沿z方向运动，朝下为正位移）为了方便计算假设向下速度为正。加速时加速度是正值
        double pos = 0.0;      // 从起始点到当前的累计位移
        double v = 0.0;        // 当前速度
        auto point = start;
        trajectory.push_back(point);
        
        //
        // 阶段1：空中加速（从0加速到 target_speed）
        //
        // 根据公式： v^2 = 2 * a1 * air_dist  =>  a1 = target_speed^2/(2*air_dist)
        double a1 = target_speed * target_speed / (2 * air_dist);
        // 加速所需时间 t1 = target_speed / a1
        double t1 = target_speed / a1;
        for(double t = 0; t < t1; t += dt) {
            v += a1 * dt;       // 累计加速
            pos += v * dt;      // 更新位移
            point = start;
            // 注意：这里假设运动方向为沿z轴负方向，所以减去位移
            point[2] = start[2] - pos;
            trajectory.push_back(point);
        }
        
        //
        // 阶段2：匀速运动（以 target_speed 匀速前进 cruise_dist）
        //
        v = target_speed; // 保持匀速
        double cruise_time = cruise_dist / target_speed;
        for(double t = 0; t < cruise_time; t += dt) {
            pos += v * dt;
            point = start;
            point[2] = start[2] - pos;
            trajectory.push_back(point);
        }
        
        //
        // 阶段3：减速缓冲（从 target_speed 减速到0）
        //
        // 根据公式： 0 = target_speed^2 - 2 * a3 * decel_dist  =>  a3 = target_speed^2/(2*decel_dist)
        double a3 = target_speed * target_speed / (2 * decel_dist);
        // 开始减速，使用 while 循环直到缓冲段累计位移达到 decel_dist
        double decel_pos = 0.0;  // 本阶段累计位移
        v = target_speed;
        while(decel_pos < decel_dist && v > 1e-6) {
            v -= a3 * dt;
            if(v < 0) {
                v = 0;
            }
            double dp = v * dt;
            decel_pos += dp;
            pos += dp;
            point = start;
            point[2] = start[2] - pos;
            trajectory.push_back(point);
        }
        
        // 最终位置补偿：确保最终位姿精确
        double final_z = start[2] - (air_dist + cruise_dist + decel_dist);
        if (std::abs(trajectory.back()[2] - final_z) > 1e-3) {
            point = trajectory.back();
            point[2] = final_z;
            trajectory.push_back(point);
        }
        
        std::cout << "四段轨迹生成完成" << std::endl;

        return trajectory;
    }


    /**
     * @brief 四阶段Y轴运动轨迹生成（空中加速+匀速运动+减速缓冲+最终停止）
     * @param start 起始点
     * @param air_dist Y轴加速段长度
     * @param cruise_dist Y轴匀速段长度 
     * @param decel_dist Y轴减速缓冲段长度
     * @param target_speed Y轴目标速度
     * @param direction Y轴方向（1表示正方向，-1表示负方向）
     */
    static std::vector<std::array<double, 6>> YgenerateFourPhasePath(
        const std::array<double, 6>& start,
        double air_dist,
        double cruise_dist,
        double decel_dist,
        double target_speed,
        int direction = 1)
    {
        // 参数校验
        const double EPSILON = 1e-10;
        if(air_dist <= EPSILON|| cruise_dist <= EPSILON || decel_dist <= EPSILON)
            throw std::invalid_argument("距离参数必须为正数");

        // 确保方向参数为1或-1
        if(direction != 1 && direction != -1)
            direction = direction >= 0 ? 1 : -1;

        // 使用固定时间步长进行欧拉积分仿真
        double dt = 0.001;
        std::vector<std::array<double, 6>> trajectory;
        
        // 初始状态：位置、速度均为0
        double pos = 0.0;      // 从起始点到当前的累计位移
        double v = 0.0;        // 当前速度
        auto point = start;
        trajectory.push_back(point);
        
        //
        // 阶段1：空中加速（从0加速到 target_speed）
        //
        double a1 = target_speed * target_speed / (2 * air_dist);
        double t1 = target_speed / a1;
        for(double t = 0; t < t1; t += dt) {
            v += a1 * dt;       // 累计加速
            pos += v * dt;      // 更新位移
            point = start;
            // 注意：这里应用Y轴位移，根据direction决定方向
            point[1] = start[1] + direction * pos;
            trajectory.push_back(point);
        }
        
        //
        // 阶段2：匀速运动（以 target_speed 匀速前进 cruise_dist）
        //
        v = target_speed; // 保持匀速
        double cruise_time = cruise_dist / target_speed;
        for(double t = 0; t < cruise_time; t += dt) {
            pos += v * dt;
            point = start;
            point[1] = start[1] + direction * pos;
            trajectory.push_back(point);
        }
        
        //
        // 阶段3：减速缓冲（从 target_speed 减速到0）
        //
        double a3 = target_speed * target_speed / (2 * decel_dist);
        double decel_pos = 0.0;  // 本阶段累计位移
        v = target_speed;
        while(decel_pos < decel_dist && v > 1e-6) {
            v -= a3 * dt;
            if(v < 0) {
                v = 0;
            }
            double dp = v * dt;
            decel_pos += dp;
            pos += dp;
            point = start;
            point[1] = start[1] + direction * pos;
            trajectory.push_back(point);
        }
        
        // 最终位置补偿：确保最终位姿精确
        double final_y = start[1] + direction * (air_dist + cruise_dist + decel_dist);
        if (std::abs(trajectory.back()[1] - final_y) > 1e-3) {
            point = trajectory.back();
            point[1] = final_y;
            trajectory.push_back(point);
        }
        
        std::cout << "Y轴四段轨迹生成完成，方向:" << (direction > 0 ? "正向" : "负向") << std::endl;

        return trajectory;
    }

    /**
     * @brief 生成Z-Y复合轨迹（先向下再水平运动）
     * @param start 起始点
     * @param z_air_dist Z轴加速段距离
     * @param z_cruise_dist Z轴匀速段距离
     * @param z_decel_dist Z轴减速段距离
     * @param z_target_speed Z轴目标速度
     * @param y_air_dist Y轴加速段距离
     * @param y_cruise_dist Y轴匀速段距离
     * @param y_decel_dist Y轴减速段距离
     * @param y_target_speed Y轴目标速度
     * @param y_direction Y轴方向（1表示正方向，-1表示负方向）
     */
    static std::vector<std::array<double, 6>> generateZYCompositePath(
        const std::array<double, 6>& start,
        double z_air_dist, double z_cruise_dist, double z_decel_dist, double z_target_speed,
        double y_air_dist, double y_cruise_dist, double y_decel_dist, double y_target_speed,
        int y_direction = 1)
    {
        // 首先生成Z轴向下轨迹
        auto z_trajectory = generateFourPhasePath(
            start, z_air_dist, z_cruise_dist, z_decel_dist, z_target_speed
        );
        
        if(z_trajectory.empty()) {
            throw std::runtime_error("Z轴轨迹生成失败");
        }
        
        // 使用Z轴轨迹的最终点作为Y轴轨迹的起点
        auto y_start = z_trajectory.back();
        
        // 生成Y轴水平轨迹
        auto y_trajectory = YgenerateFourPhasePath(
            y_start, y_air_dist, y_cruise_dist, y_decel_dist, y_target_speed, y_direction
        );
        
        if(y_trajectory.empty()) {
            throw std::runtime_error("Y轴轨迹生成失败");
        }
        
        // 合并两段轨迹
        std::vector<std::array<double, 6>> combined_trajectory = z_trajectory;
        // 跳过y_trajectory的第一个点（与z_trajectory的最后一个点重复）
        combined_trajectory.insert(combined_trajectory.end(), 
                                y_trajectory.begin() + 1, 
                                y_trajectory.end());
        
        std::cout << "ZY复合轨迹生成完成，总点数: " << combined_trajectory.size() 
                << " (Z段: " << z_trajectory.size() 
                << ", Y段: " << y_trajectory.size() - 1 << ")" << std::endl;
        
        return combined_trajectory;
    }


    /**
     * @brief 生成垂直下压 + 特定角度的复合轨迹
     * @param start 起始点 
     * @param vertical_air_dist 垂直下压加速段长度
     * @param vertical_cruise_dist 垂直下压匀速段长度
     * @param vertical_decel_dist 垂直下压减速段长度
     * @param vertical_target_speed 垂直下压目标速度
     * @param diagonal_air_dist 对角线加速段长度
     * @param diagonal_cruise_dist 对角线匀速段长度
     * @param diagonal_decel_dist 对角线减速段长度
     * @param diagonal_target_speed 对角线目标速度
     * @param gamma_deg Z-Y平面角度(度)
     */
    static std::vector<std::array<double, 6>> generateVerticalDiagonalPath(
        const std::array<double, 6>& start,
        double vertical_air_dist, double vertical_cruise_dist, double vertical_decel_dist, 
        double vertical_target_speed,
        double diagonal_air_dist, double diagonal_cruise_dist, double diagonal_decel_dist, 
        double diagonal_target_speed,
        double gamma_deg)
    {
        // 首先生成垂直下压轨迹
        auto vertical_trajectory = generateFourPhasePath(
            start, vertical_air_dist, vertical_cruise_dist, vertical_decel_dist, vertical_target_speed
        );
        
        if(vertical_trajectory.empty()) {
            throw std::runtime_error("垂直轨迹生成失败");
        }
        
        // 以垂直轨迹的末端点作为对角线轨迹的起点
        auto diagonal_start = vertical_trajectory.back();
        
        // 生成对角线轨迹
        auto diagonal_trajectory = generateDiagonalPhasePath(
            diagonal_start, diagonal_air_dist, diagonal_cruise_dist, diagonal_decel_dist, 
            diagonal_target_speed, gamma_deg
        );
        
        if(diagonal_trajectory.empty()) {
            throw std::runtime_error("对角线轨迹生成失败");
        }
        
        // 合并两段轨迹
        std::vector<std::array<double, 6>> combined_trajectory = vertical_trajectory;
        // 跳过diagonal_trajectory的第一个点（与vertical_trajectory的最后一个点重复）
        combined_trajectory.insert(combined_trajectory.end(), 
                                diagonal_trajectory.begin() + 1, 
                                diagonal_trajectory.end());
        
        std::cout << "垂直+对角复合轨迹生成完成，总点数: " << combined_trajectory.size() 
                << " (垂直段: " << vertical_trajectory.size() 
                << ", 对角段: " << diagonal_trajectory.size() - 1 
                << "), 角度: " << gamma_deg << "度" << std::endl;
        
        return combined_trajectory;
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
        // RCLCPP_INFO(this->get_logger(), "Start to cartesian impedance control");

        /*
        * @brief 通用浮点参数描述生成器
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
        // 声明参数
        this->declare_parameter("cartesian_point(command)", "0.45 0.0 0.5 3.14154 0.0 3.14154");
        this->declare_parameter("velocity(command)", "0.1");
        this->declare_parameter("desired_force_z", -5.0);

        this->declare_parameter("first_time", 10.0);
        this->declare_parameter("second_time", 10.0);

        this->declare_parameter("air_distance", 0.12, 
            floatDesc("空中加速段距离（米）", 0.01, 1.0, 0.01));       //空中加速段距离
        this->declare_parameter("cruise_distance", 0.05,
            floatDesc("匀速侵入段距离（米）", 0.01, 1.0, 0.01));    //匀速侵入段距离
        this->declare_parameter("decel_distance", 0.02,
            floatDesc("减速缓冲段距离（米）", 0.00, 1.0, 0.01));     //减速缓冲段距离
        this->declare_parameter("target_speed", 0.05,
            floatDesc("期望侵入速度（米/秒）", 0.01, 2.5, 0.01));   //期望侵入速度

        // 添加Y轴参数
        this->declare_parameter("y_air_distance", 0.04, 
            floatDesc("Y轴加速段距离（米）", 0.01, 1.0, 0.01));     //Y轴加速段距离
        this->declare_parameter("y_cruise_distance", 0.05,
            floatDesc("Y轴匀速段距离（米）", 0.01, 1.0, 0.01));     //Y轴匀速段距离
        this->declare_parameter("y_decel_distance", 0.01,
            floatDesc("Y轴减速段距离（米）", 0.00, 1.0, 0.01));     //Y轴减速段距离
        this->declare_parameter("y_target_speed", 0.01,
            floatDesc("Y轴期望速度（米/秒）", 0.01, 2.5, 0.01));    //Y轴期望速度
        this->declare_parameter("y_direction", 1,
            floatDesc("Y轴方向(1=正向,-1=负向)", -1, 1, 2));       //Y轴方向

        // 添加对角线参数
        this->declare_parameter("diagonal_air_distance", 0.04, 
            floatDesc("对角线加速段距离（米）", 0.0, 1.0, 0.01));
        this->declare_parameter("diagonal_cruise_distance", 0.05,
            floatDesc("对角线匀速段距离（米）", 0.0, 1.0, 0.01));
        this->declare_parameter("diagonal_decel_distance", 0.01,
            floatDesc("对角线减速段距离（米）", 0.0, 1.0, 0.01));
        this->declare_parameter("diagonal_target_speed", 0.03,
            floatDesc("对角线期望速度（米/秒）", 0.01, 2.5, 0.01));
        this->declare_parameter("gamma_angle", 45.00,
            floatDesc("对角线Z-Y平面角度（度，0=Y轴负方向，90=Z轴负方向）", 0.00, 90.00, 0.01));

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
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Rokae_Force::timer_callback, this));
        
        // 添加力传感器数据订阅者
        force_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "force_sensor_data", 10,
            std::bind(&Rokae_Force::force_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "已创建力传感器数据订阅者");

        // 添加实时位姿数据发布者
        realtime_pose_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "realtime_robot_pose", 10);

        // 创建位姿数据发布定时器(10Hz)
        pose_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Rokae_Force::pubilsh_initial_pose, this));
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
        // 获取需要的参数
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
    void cartesian_impedance_control(double desired_force_z, double first_time, double second_time)
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
     * @brief 实时模式纯轨迹控制，速度不可控
     * @param first_time 第一段时间
     * @param second_time 第二段时间
     */
    void usr_rt_cartesian_control(double first_time, double second_time)
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
    void usr_rt_cartesian_v_control(
        double z_air_dist, double z_cruise_dist, double z_decel_dist, double z_target_speed,
        double y_air_dist = 0.0, double y_cruise_dist = 0.0, double y_decel_dist = 0.0, 
        double y_target_speed = 0.0, int y_direction = 1)
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
            std::vector<std::string> fields = {RtSupportedFields::tcpPoseAbc_m}; // 接收末端位姿数据
            robot->startReceiveRobotState(std::chrono::milliseconds(1), fields); // 1ms采样周期
            
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
            
            std::atomic<bool> stopManually{true};
            int index = 0;

            // 控制循环回调函数
            // CartesianPosition output{}是给output进行类型定义
            std::function<CartesianPosition(void)> callback = [&, this]() -> CartesianPosition {
                CartesianPosition output{};

                RCLCPP_INFO(this->get_logger(), "进入回调函数1");

                if (index < int(trajectory.size())) {
                    // 获取目标轨迹点。这里的target_pose是当前的理论轨迹规划点
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
                    // 标记任务完成
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
            rtCon->stopLoop();
            rtCon->stopMove();
            RCLCPP_ERROR(this->get_logger(), "实时轨迹控制错误: %s", e.what());
        }
    }

    void usr_rt_cartesian_v_control_z(double air_dist, double cruise_dist, double decel_dist, double target_speed)
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
    void usr_rt_vertical_diagonal_control(
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
                    
                    // 获取实时位姿并发布
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
     * @brief 发布实时位姿数据到话题。本质上是发布current_pose的前三位和target_pose的前三位，组成一个六位数据，
     *        可以自定义传入参数从而改变发送数据。
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

    /**
     * @brief 初始位资发布。用于：让plotjuggler读取到初始位资，不然只有在callback运行时才会读取到。
     */
    void pubilsh_initial_pose(){
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
