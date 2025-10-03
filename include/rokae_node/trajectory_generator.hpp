#pragma once // 防止头文件重复包含

#include <vector>
#include <array>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


// 辅助函数声明
tf2::Quaternion euler_to_quaternion(double roll, double pitch, double yaw); // 欧拉角转四元数
std::array<double, 3> quaternion_to_euler(const tf2::Quaternion &q); // 四元数转欧拉角


// 贝塞尔曲线轨迹生成器类
class TrajectoryGenerator
{
public:
    
    static std::vector<std::array<double, 6>> generateBezierTrajectory(
        const std::vector<std::array<double, 6>> &controlPoints,
        double total_duration);

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
            double duration_second_segment);
    
    /**
     * @brief 生成一段线性轨迹
     * @param duration 轨迹持续时间
     * @return trajectory 轨迹
     */
    static std::vector<std::array<double, 6>> LinearSegmentPath_one(
        const std::array<double, 6>& start,
        const std::array<double, 6>& end,
        double duration);

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
        double gamma_deg);

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
        double target_speed);

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
        int direction);

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
        int y_direction);

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
        double gamma_deg);

private:
    static double smoothTrajectory(double t, double total_duration);

    static double calculatePosition(double t, double start, double end, double total_duration);

    static std::array<double, 6> bezierInterpolate(
        const std::vector<std::array<double, 6>> &points,
        double t,
        double total_duration);
};
