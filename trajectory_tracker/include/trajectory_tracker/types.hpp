#ifndef TRAJECTORY_TRACKER_TYPES_HPP
#define TRAJECTORY_TRACKER_TYPES_HPP

/**
 * @file types.hpp
 * @brief 轨迹跟踪共用类型：机器人状态、控制输出、参考点等
 */

namespace trajectory_tracker {

/** 机器人状态（与 odom + tf 一致，后轴中心） */
struct RobotState {
    double x = 0.0;       // 世界坐标 x (m)
    double y = 0.0;       // 世界坐标 y (m)
    double theta = 0.0;   // 航向角 (rad)
    double v = 0.0;       // 纵向速度 (m/s)
};

/** 控制输出：阿克曼模型分离线速度与前轮转角 */
struct ControlCommand {
    double velocity = 0.0;        // 线速度指令 (m/s)，发布到 cmd_vel.linear.x
    double steering_angle = 0.0;  // 前轮转角 (rad)，发布到 /steering_angle
};

/** 参考路径上的一个点（用于跟踪器内部） */
struct ReferencePoint {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;
    double kappa = 0.0;   // 曲率 (1/m)，可选
};

/** 车辆参数（轴距等，用于 LQR/Pure Pursuit 等） */
struct VehicleParams {
    double wheelbase = 0.32;
    double max_steering_angle = 0.5236;  // rad, ~30°
    double max_speed = 1.0;
    double min_speed = 0.0;
};

} // namespace trajectory_tracker

#endif // TRAJECTORY_TRACKER_TYPES_HPP
