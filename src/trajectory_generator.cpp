#include "rokae_node/trajectory_generator.hpp" 


// 辅助函数实现
// 欧拉角转四元数
tf2::Quaternion euler_to_quaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

// 四元数转欧拉角
std::array<double, 3> quaternion_to_euler(const tf2::Quaternion &q)
{
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return {roll, pitch, yaw};
}


// ----------------------------------------------- TrajectoryGenerator 类的函数实现-----------------------------------------------------------

// public 方法实现--------------------------------------------------------------------------------------------------------
std::vector<std::array<double, 6>> TrajectoryGenerator::generateBezierTrajectory(
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
std::vector<std::array<double, 6>> TrajectoryGenerator::generateLinearSegmentPath(
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
std::vector<std::array<double, 6>> TrajectoryGenerator::LinearSegmentPath_one(
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
std::vector<std::array<double, 6>> TrajectoryGenerator::generateDiagonalPhasePath(
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
std::vector<std::array<double, 6>> TrajectoryGenerator::generateFourPhasePath(
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
std::vector<std::array<double, 6>> TrajectoryGenerator::YgenerateFourPhasePath(
    const std::array<double, 6>& start,
    double air_dist,
    double cruise_dist,
    double decel_dist,
    double target_speed,
    int direction)
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
std::vector<std::array<double, 6>> TrajectoryGenerator::generateZYCompositePath(
    const std::array<double, 6>& start,
    double z_air_dist, double z_cruise_dist, double z_decel_dist, double z_target_speed,
    double y_air_dist, double y_cruise_dist, double y_decel_dist, double y_target_speed,
    int y_direction)
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
std::vector<std::array<double, 6>> TrajectoryGenerator::generateVerticalDiagonalPath(
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


// private 方法实现--------------------------------------------------------------------------------------------------------

double TrajectoryGenerator::smoothTrajectory(double t, double total_duration)
{
    if (t <= 0)
        return 0;
    if (t >= total_duration)
        return 1;
    double normalizedT = t / total_duration;
    return 10 * std::pow(normalizedT, 3) - 15 * std::pow(normalizedT, 4) + 6 * std::pow(normalizedT, 5);
}


double TrajectoryGenerator::calculatePosition(double t, double start, double end, double total_duration)
{
    double normalizedPosition = smoothTrajectory(t, total_duration);
    return start + (end - start) * normalizedPosition;
}


std::array<double, 6> TrajectoryGenerator::bezierInterpolate(
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
