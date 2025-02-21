/**
 * @file 用于验证生成的机械臂三维轨迹是否符合预期。
 */

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
// #include </opt/ros/foxy/include/tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <string>
#include <fmt/format.h>

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
    // 新增速度数据结构
    struct TrajectoryPoint {
        std::array<double, 6> pose;  // x,y,z,roll,pitch,yaw
        std::array<double, 3> velocity; // vx,vy,vz
    };

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

    // 修改后的保存函数
    static void saveTrajectoryToFileWithV(const std::vector<TrajectoryPoint>& trajectory, 
                                   const std::string& filename) 
    {
        std::ofstream file(filename);
        file << "x,y,z,roll,pitch,yaw,vx,vy,vz\n";

        for (const auto& point : trajectory) {
            // file << fmt::format("{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n",
            //     point.pose[0], point.pose[1], point.pose[2],
            //     point.pose[3], point.pose[4], point.pose[5],
            //     point.velocity[0], point.velocity[1], point.velocity[2]);
            file << point.pose[0] << "," << point.pose[1] << "," << point.pose[2] << ","
                 << point.pose[3] << "," << point.pose[4] << "," << point.pose[5] << ","
                 << point.velocity[0] << "," << point.velocity[1] << "," << point.velocity[2] << "\n";
        }

        std::cout << "轨迹数据已保存到: " << filename << std::endl;
    }


    /**
     * @brief 四阶段侵入轨迹生成（空中加速+匀速侵入+减速缓冲+最终停止）
     * @param start 起始点
     * @param air_dist 空中加速段长度（到沙土表面的距离）
     * @param cruise_dist 匀速侵入段长度 
     * @param decel_dist 减速缓冲段长度
     * @param target_speed 目标侵入速度
     */
    static std::vector<TrajectoryPoint> generateFourPhasePath(
        const std::array<double, 6>& start,
        double air_dist,
        double cruise_dist,
        double decel_dist,
        double target_speed)
    {
        // 新增参数校验
        if (target_speed <= 0 || air_dist <=0 || cruise_dist <0 || decel_dist <=0) {
            throw std::invalid_argument("参数必须为正数");
        }

        std::vector<TrajectoryPoint> trajectory;
        double dt = 0.001;
        double pos = 0.0;
        double v = 0.0;

        // 阶段1：空中加速
        double a1 = target_speed * target_speed / (2 * air_dist);
        double t1 = target_speed / a1;
        for(double t = 0; t < t1; t += dt) {
            TrajectoryPoint point;
            point.pose = start;
            v += a1 * dt;
            pos += v * dt;
            point.pose[2] = start[2] - pos;
            point.velocity = {0, 0, v}; // 假设只有z轴运动
            trajectory.push_back(point);
        }

        // 阶段2：匀速
        double cruise_time = cruise_dist / target_speed;
        for(double t = 0; t < cruise_time; t += dt) {
            TrajectoryPoint point;
            point.pose = start;
            pos += target_speed * dt;
            point.pose[2] = start[2] - pos;
            point.velocity = {0, 0, target_speed};
            trajectory.push_back(point);
        }

        std::cout << "阶段2结束时轨迹点数量：" << trajectory.size() << std::endl;

        // 阶段3：减速
        double a3 = target_speed * target_speed / (2 * decel_dist);
        double decel_pos = 0.0;
        v = target_speed;
        while(decel_pos < decel_dist && v > 1e-6) {
            TrajectoryPoint point;
            point.pose = start;
            v -= a3 * dt;
            if(v < 0) v = 0;
            double dp = v * dt;
            decel_pos += dp;
            pos += dp;
            point.pose[2] = start[2] - pos;
            point.velocity = {0, 0, v};
            trajectory.push_back(point);
            auto& last = trajectory.back();

            std::cout << "减速阶段位置：" << last.pose[2] << std::endl;
        }
        
        std::cout << "四段轨迹生成（没有最终位置修正）" << std::endl;

        // 最终位置修正
        if (!trajectory.empty()) {
            auto& last = trajectory.back();
            double final_z = start[2] - (air_dist + cruise_dist + decel_dist);
            last.pose[2] = final_z;
            last.velocity = {0, 0, 0};
        }

        std::cout << "四段轨迹生成完成" << std::endl;
        return trajectory;
    }
public:
    /**
     * @brief 生成两段线性轨迹，最终合成一段轨迹并输出为csv文件。可以通过设置起始点、中间点和终点来验证生成的轨迹是否符合预期。
     * @param durationOne 第一段轨迹持续时间
     * @param durationTwo 第二段轨迹持续时间
     */
    static void verifyLinearSegmentPath_two(double durationOne=2.0, double durationTwo=3.0) {
        // 定义测试路径点
        // 测试过的路径点保存在了本项目rokae_move/example/trajectoryPoint.md文件中
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
        auto trajectory1 = generateBezierTrajectory(controlPoints1, durationOne);  // 2秒下压
        auto trajectory2 = generateBezierTrajectory(controlPoints2, durationTwo);  // 3秒滑移

        // 合并轨迹
        trajectory1.insert(trajectory1.end(), trajectory2.begin(), trajectory2.end());

        // 保存轨迹数据到CSV文件
        saveTrajectoryToFile(trajectory1, "trajectory_data.csv");
    }


    /**
     * @brief 生成一段线性轨迹，最终输出为csv文件。可以通过设置起始点和终点来验证生成的轨迹是否符合预期。
     * @param duration 轨迹持续时间
     */
    static void verifyLinearSegmentPath_one(double duration = 2.0) {
        // 定义测试路径点
        std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};    // 起始点
        std::array<double, 6> end = {0.4, 0.0, 0.3, M_PI, 0.0, M_PI};      // 终点

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

        // 保存轨迹数据到CSV文件
        saveTrajectoryToFile(trajectory, "single_trajectory_data.csv");
    }

    // 新增验证接口
    static void verifyFourPhase() {
        std::array<double, 6> start = {0.4, 0.0, 0.5, M_PI, 0.0, M_PI};
        auto trajectory = generateFourPhasePath(start, 0.2, 0.1, 0.05, 0.5);
        saveTrajectoryToFileWithV(trajectory, "/home/le/dev_ws/src/rokae_move/build/data/four_phase_trajectory.csv");
    }
};

int main() {
    TrajectoryVerification::verifyFourPhase();
    return 0;
}
// g++ -o trajectory_verification src/trajectory_verification.cpp -I/opt/ros/foxy/include 这是编译命令
