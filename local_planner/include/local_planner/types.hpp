#ifndef LOCAL_PLANNER_TYPES_HPP
#define LOCAL_PLANNER_TYPES_HPP

/**
 * @file types.hpp
 * @brief 局部路径规划的基础类型定义
 * 
 * 本文件定义了局部路径规划中使用的所有基础数据结构，包括：
 * - 车辆状态 (State)
 * - 控制输入 (Control)
 * - 轨迹点 (TrajectoryPoint)
 * - 轨迹 (Trajectory)
 * - 障碍物 (Obstacle)
 */

#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>

namespace local_planner {

/**
 * @brief 车辆状态
 * 
 * 描述车辆在某一时刻的完整状态，包括位置、姿态、速度等。
 * 使用 Frenet 坐标系和笛卡尔坐标系的混合表示。
 */
struct State {
    // ========== 笛卡尔坐标系 (Cartesian) ==========
    double x = 0.0;           // x 坐标 (m)
    double y = 0.0;           // y 坐标 (m)
    double theta = 0.0;       // 航向角 (rad)，从 x 轴正方向逆时针为正
    double kappa = 0.0;       // 曲率 (1/m)，左转为正
    
    // ========== 运动状态 ==========
    double v = 0.0;           // 纵向速度 (m/s)
    double a = 0.0;           // 纵向加速度 (m/s²)
    
    // ========== Frenet 坐标系 (沿参考线) ==========
    double s = 0.0;           // 沿参考线的弧长 (m)
    double s_dot = 0.0;       // s 的变化率 (m/s)
    double s_ddot = 0.0;      // s 的二阶导数 (m/s²)
    double d = 0.0;           // 到参考线的横向偏移 (m)，左正右负
    double d_dot = 0.0;       // d 的变化率 (m/s)
    double d_ddot = 0.0;      // d 的二阶导数 (m/s²)
    
    // ========== 时间戳 ==========
    double t = 0.0;           // 时间戳 (s)
    
    /**
     * @brief 计算与另一个状态的欧几里得距离
     */
    double distanceTo(const State& other) const {
        return std::hypot(x - other.x, y - other.y);
    }
};

/**
 * @brief 控制输入
 * 
 * 描述作用在车辆上的控制指令。
 * 对于自行车模型，控制输入是加速度和前轮转角。
 */
struct Control {
    double acceleration = 0.0;     // 纵向加速度 (m/s²)
    double steering_angle = 0.0;   // 前轮转角 (rad)，左转为正
    double steering_rate = 0.0;    // 转向角速度 (rad/s)
    
    // 用于 Eigen 矩阵运算
    Eigen::Vector2d toVector() const {
        return Eigen::Vector2d(acceleration, steering_angle);
    }
    
    static Control fromVector(const Eigen::Vector2d& vec) {
        Control ctrl;
        ctrl.acceleration = vec(0);
        ctrl.steering_angle = vec(1);
        return ctrl;
    }
};

/**
 * @brief 轨迹点
 * 
 * 轨迹上的一个离散点，包含状态和控制输入。
 */
struct TrajectoryPoint {
    State state;
    Control control;
    double cost = 0.0;        // 该点的累积代价
    bool is_collision = false; // 是否碰撞
};

/**
 * @brief 完整轨迹
 * 
 * 由一系列轨迹点组成的完整路径。
 */
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double total_cost = std::numeric_limits<double>::max();
    double total_time = 0.0;
    double total_length = 0.0;
    bool is_valid = false;
    
    /**
     * @brief 获取轨迹长度
     */
    double getLength() const {
        if (points.size() < 2) return 0.0;
        double len = 0.0;
        for (size_t i = 1; i < points.size(); ++i) {
            len += points[i].state.distanceTo(points[i-1].state);
        }
        return len;
    }
    
    /**
     * @brief 清空轨迹
     */
    void clear() {
        points.clear();
        total_cost = std::numeric_limits<double>::max();
        total_time = 0.0;
        total_length = 0.0;
        is_valid = false;
    }
};

/**
 * @brief 障碍物
 * 
 * 描述环境中的障碍物，可以是静态或动态的。
 */
struct Obstacle {
    // 位置和形状
    double x = 0.0;           // 中心 x 坐标 (m)
    double y = 0.0;           // 中心 y 坐标 (m)
    double theta = 0.0;       // 朝向 (rad)
    double length = 0.0;      // 长度 (m)，沿朝向方向
    double width = 0.0;       // 宽度 (m)，垂直朝向方向
    double radius = 0.0;      // 等效圆半径 (m)，用于快速碰撞检测
    
    // 运动状态（动态障碍物）
    double vx = 0.0;          // x 方向速度 (m/s)
    double vy = 0.0;          // y 方向速度 (m/s)
    
    bool is_static = true;    // 是否为静态障碍物
    int id = -1;              // 障碍物 ID
    
    /**
     * @brief 预测未来位置
     * @param dt 时间增量 (s)
     */
    Obstacle predictAt(double dt) const {
        Obstacle pred = *this;
        pred.x += vx * dt;
        pred.y += vy * dt;
        return pred;
    }
};

/**
 * @brief 车辆参数
 * 
 * 描述车辆的物理参数，用于运动学和动力学模型。
 */
struct VehicleParams {
    // 几何参数
    double wheelbase = 0.32;           // 轴距 (m)
    double front_overhang = 0.1;      // 前悬 (m)
    double rear_overhang = 0.1;       // 后悬 (m)
    double width = 0.3;               // 车宽 (m)
    double length = 0.45;              // 车长 (m)
    
    // 运动学约束
    double max_steering_angle = 0.5236;  // 最大前轮转角 (rad) ≈ 30°
    double max_steering_rate = 0.5;   // 最大转向角速度 (rad/s)
    double max_speed = 1.0;           // 最大速度 (m/s)
    double min_speed = -0.5;          // 最小速度 (m/s)，负值表示倒车
    double max_acceleration = 2.0;    // 最大加速度 (m/s²)
    double max_deceleration = 3.0;    // 最大减速度 (m/s²)
    double max_curvature = 1.0;       // 最大曲率 (1/m)
    
    /**
     * @brief 计算转向角对应的曲率
     * 自行车模型：kappa = tan(delta) / L
     */
    double steeringToKappa(double steering_angle) const {
        return std::tan(steering_angle) / wheelbase;
    }
    
    /**
     * @brief 计算曲率对应的转向角
     */
    double kappaToSteering(double kappa) const {
        return std::atan(kappa * wheelbase);
    }
};

/**
 * @brief 底盘模型类型枚举
 */
enum class ChassisModelType {
    ACKERMANN = 0,   // 阿克曼底盘（自行车模型）
    DIFFERENTIAL = 1, // 差速底盘（两轮差速）
    OMNIWHEEL = 2    // 全向轮底盘
};

/**
 * @brief 抽象底盘模型基类
 * 
 * 定义不同底盘模型的统一接口，用于规划时根据底盘类型
 * 选择合适的机动策略（如倒车、原地掉头等）。
 */
class ChassisModel {
public:
    /**
     * @brief 构造函数
     * @param params 车辆参数
     */
    explicit ChassisModel(const VehicleParams& params) : params_(params) {}
    
    /**
     * @brief 虚析构函数
     */
    virtual ~ChassisModel() = default;
    
    /**
     * @brief 判断是否需要特殊机动
     * @param current_state 当前状态
     * @param target_state 目标状态
     * @return true 如果当前底盘类型需要特殊机动（如倒车、原地掉头）
     */
    virtual bool needsSpecialManeuver(const State& current_state, 
                                       const State& target_state) const = 0;
    
    /**
     * @brief 生成特殊机动轨迹
     * @param current_state 当前状态
     * @param target_state 目标状态
     * @param obstacles 障碍物列表
     * @return 机动轨迹（倒车、原地掉头等）
     */
    virtual Trajectory generateManeuverTrajectory(
        const State& current_state,
        const State& target_state,
        const std::vector<Obstacle>& obstacles) const = 0;
    
    /**
     * @brief 获取底盘类型名称
     * @return 字符串形式的类型名称
     */
    virtual std::string getTypeName() const = 0;
    
    /**
     * @brief 获取车辆参数
     */
    const VehicleParams& getParams() const { return params_; }

protected:
    VehicleParams params_;
};

/**
 * @brief 阿克曼底盘模型
 * 
 * 阿克曼底盘（自行车模型）的特点：
 * - 无法原地旋转，需要空间转弯
 * - 航向差异过大时，需要倒车调整方向
 * - 使用自行车运动学模型
 * - 倒车方向锁定机制：一旦决定倒车方向，保持稳定直到完成
 */
class AckermannModel : public ChassisModel {
public:
    explicit AckermannModel(const VehicleParams& params) : ChassisModel(params) {}
    
    bool needsSpecialManeuver(const State& current_state, 
                               const State& target_state) const override;
    
    Trajectory generateManeuverTrajectory(
        const State& current_state,
        const State& target_state,
        const std::vector<Obstacle>& obstacles) const override;
    
    std::string getTypeName() const override { return "Ackermann"; }
    
    /**
     * @brief 重置倒车状态（当倒车完成或需要重新开始时调用）
     */
    static void resetReverseState();

private:
    // ========== 静态变量用于倒车方向锁定 ==========
    static bool s_in_reverse_mode_;           // 是否正在倒车模式中
    static int s_locked_steering_direction_;  // 锁定的转向方向: 1=左转(正), -1=右转(负), 0=未锁定
    static double s_initial_heading_diff_;    // 进入倒车模式时的初始航向差
    static int s_reverse_hold_counter_;       // 模式保持计数器
};

/**
 * @brief 差速底盘模型
 * 
 * 差速底盘的特点：
 * - 左右轮速度独立控制
 * - 可以原地旋转（旋转中心在车辆中心）
 * - 航向差异过大时，原地掉头比倒车更高效
 */
class DifferentialModel : public ChassisModel {
public:
    explicit DifferentialModel(const VehicleParams& params) : ChassisModel(params) {}
    
    bool needsSpecialManeuver(const State& current_state, 
                               const State& target_state) const override;
    
    Trajectory generateManeuverTrajectory(
        const State& current_state,
        const State& target_state,
        const std::vector<Obstacle>& obstacles) const override;
    
    std::string getTypeName() const override { return "Differential"; }
};

/**
 * @brief 全向轮底盘模型
 * 
 * 全向轮底盘的特点：
 * - 可以向任意方向移动
 * - 可以原地旋转
 * - 无需特殊机动策略，直接朝向目标即可
 */
class OmniWheelModel : public ChassisModel {
public:
    explicit OmniWheelModel(const VehicleParams& params) : ChassisModel(params) {}
    
    bool needsSpecialManeuver(const State& current_state, 
                               const State& target_state) const override;
    
    Trajectory generateManeuverTrajectory(
        const State& current_state,
        const State& target_state,
        const std::vector<Obstacle>& obstacles) const override;
    
    std::string getTypeName() const override { return "OmniWheel"; }
};

/**
 * @brief 从字符串解析底盘模型类型
 * @param str 字符串（如 "ackermann", "differential", "omniwheel"）
 * @return 对应的 ChassisModelType 枚举值
 */
ChassisModelType parseChassisModelType(const std::string& str);

/**
 * @brief 底盘模型类型转字符串
 * @param type 枚举类型
 * @return 字符串表示
 */
std::string chassisModelTypeToString(ChassisModelType type);

/**
 * @brief Lattice Planner 配置参数
 */
struct LatticeConfig {
    // 采样参数
    int num_width_samples = 7;        // 横向采样数量
    int num_time_samples = 10;         // 时间采样数量
    double max_lateral_offset = 2.0;  // 最大横向偏移 (m)
    double min_planning_time = 2.0;   // 最小规划时间 (s)
    double max_planning_time = 6.0;   // 最大规划时间 (s)
    double time_resolution = 0.1;     // 时间分辨率 (s)
    
    // 代价权重
    double weight_lateral_offset = 5.0;     // 横向偏移代价权重（越大越沿全局路径走）
    double weight_lateral_velocity = 1.0;   // 横向速度代价权重
    double weight_longitudinal_jerk = 1.0;  // 纵向 jerk 代价权重
    double weight_lateral_jerk = 1.0;       // 横向 jerk 代价权重
    double weight_time = 1.0;               // 时间代价权重
    double weight_obstacle = 10.0;          // 障碍物代价权重
    double weight_deviation = 10.0;         // 偏离参考路径的惩罚权重
    
    // 安全参数
    double safe_distance = 0.2;       // 障碍物安全距离/膨胀半径 (m)
};

/**
 * @brief iLQR 配置参数
 */
struct ILQRConfig {
    // 优化参数
    int max_iterations = 50;          // 最大迭代次数
    double tolerance = 1e-4;          // 收敛阈值
    double initial_lambda = 1.0;      // 初始正则化参数
    double lambda_factor = 10.0;      // lambda 调整因子
    double min_lambda = 1e-6;         // 最小 lambda
    double max_lambda = 1e6;          // 最大 lambda
    
    // 代价权重 (Q 矩阵对角元素)
    double weight_x = 1.0;            // x 位置误差权重
    double weight_y = 1.0;            // y 位置误差权重
    double weight_theta = 0.5;        // 航向角误差权重
    double weight_v = 0.1;            // 速度误差权重
    
    // 控制代价权重 (R 矩阵对角元素)
    double weight_acceleration = 0.1; // 加速度代价权重
    double weight_steering = 0.1;     // 转向代价权重
    
    // 终端代价权重 (Qf 矩阵)
    double weight_terminal = 10.0;    // 终端状态权重因子
    
    // 规划参数
    double horizon_time = 3.0;        // 规划时域 (s)
    double dt = 0.1;                  // 时间步长 (s)
    
    // 避障参数
    double weight_obstacle = 50.0;    // 障碍物排斥代价权重
    double safe_distance = 0.5;       // 安全距离 (m)，小于此距离时产生排斥代价
};

} // namespace local_planner

#endif // LOCAL_PLANNER_TYPES_HPP
