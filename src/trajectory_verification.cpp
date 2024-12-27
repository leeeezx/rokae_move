/**
 * @file 用于验证生成的机械臂三维轨迹是否符合预期。
 */

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>

// 四元数转欧拉角
std::array<double, 3> quaternion_to_euler(const tf2::Quaternion &q) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}

// 欧拉角转四元数
tf2::Quaternion euler_to_quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

class TrajectoryVerification {
private:
    static double smoothTrajectory(double t, double total_duration) {
        if (t <= 0) return 0;
        if (t >= total_duration) return 1;
        double normalizedT = t / total_duration;
        return 10 * std::pow(normalizedT, 3) - 15 * std::pow(normalizedT, 4) + 6 * std::pow(normalizedT, 5);
    }

    static double calculatePosition(double t, double start, double end, double total_duration) {
        double normalizedPosition = smoothTrajectory(t, total_duration);
        return start + (end - start) * normalizedPosition;
    }

    static std::array<double, 6> bezierInterpolate(
        const std::vector<std::array<double, 6>> &points,
        double t,
        double total_duration) 
    {
        if (points.size() == 1) {
            return points[0];
        }

        std::vector<std::array<double, 6>> newPoints;
        for (size_t i = 0; i < points.size() - 1; ++i) {
            std::array<double, 6> interpolated;
            // 位置插值
            for (int j = 0; j < 3; ++j) {
                interpolated[j] = calculatePosition(t, points[i][j], points[i + 1][j], total_duration);
            }

            // 姿态插值（使用四元数）
            tf2::Quaternion q1 = euler_to_quaternion(points[i][3], points[i][4], points[i][5]);
            tf2::Quaternion q2 = euler_to_quaternion(points[i + 1][3], points[i + 1][4], points[i + 1][5]);
            tf2::Quaternion q_interp = q1.slerp(q2, smoothTrajectory(t, total_duration));

            // 将插值后的四元数转回欧拉角
            auto euler = quaternion_to_euler(q_interp);
            interpolated[3] = euler[0];
            interpolated[4] = euler[1];
            interpolated[5] = euler[2];

            newPoints.push_back(interpolated);
        }

        return bezierInterpolate(newPoints, t, total_duration);
    }

    static std::vector<std::array<double, 6>> generateBezierTrajectory(
        const std::vector<std::array<double, 6>> &controlPoints,
        double total_duration)
    {
        int num_points = static_cast<int>(total_duration * 1000); // 采样率1000Hz
        std::vector<std::array<double, 6>> trajectory;

        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / (num_points - 1) * total_duration;
            trajectory.push_back(bezierInterpolate(controlPoints, t, total_duration));
        }

        return trajectory;
    }

    static void saveTrajectoryToFile(const std::vector<std::array<double, 6>>& trajectory, 
                                   const std::string& filename) 
    {
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return;
        }

        // 写入CSV头
        file << "x,y,z,roll,pitch,yaw\n";

        // 写入轨迹数据
        for (const auto& point : trajectory) {
            file << point[0] << "," << point[1] << "," << point[2] << ","
                 << point[3] << "," << point[4] << "," << point[5] << "\n";
        }

        file.close();
        std::cout << "轨迹数据已保存到: " << filename << std::endl;
    }

public:
    static void verifyLinearSegmentPath() {
        // 定义测试路径点
        std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};  // 起始点
        std::array<double, 6> via_point = {0.4, 0.0, 0.4, M_PI, 0.0, M_PI};  // 中间点
        std::array<double, 6> end = {0.4, 0.3, 0.4, M_PI, 0.0, M_PI};  // 终点

        // 第一段：垂直下压 - 使用5个控制点强制直线
        std::vector<std::array<double, 6>> controlPoints1 = {
            start,
            start,
            start,
            via_point,
            via_point
        };
        
        // 第二段：水平滑移 - 使用5个控制点强制直线
        std::vector<std::array<double, 6>> controlPoints2 = {
            via_point,
            via_point,
            via_point,
            end,
            end
        };

        // 生成轨迹
        auto trajectory1 = generateBezierTrajectory(controlPoints1, 2.0);  // 2秒下压
        auto trajectory2 = generateBezierTrajectory(controlPoints2, 3.0);  // 3秒滑移

        // 合并轨迹
        trajectory1.insert(trajectory1.end(), trajectory2.begin(), trajectory2.end());

        // 保存轨迹数据到CSV文件
        saveTrajectoryToFile(trajectory1, "trajectory_data.csv");
    }
};

int main() {
    TrajectoryVerification::verifyLinearSegmentPath();
    return 0;
}

