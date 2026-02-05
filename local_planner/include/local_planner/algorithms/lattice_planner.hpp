#ifndef LOCAL_PLANNER_LATTICE_PLANNER_HPP
#define LOCAL_PLANNER_LATTICE_PLANNER_HPP

/**
 * @file lattice_planner.hpp
 * @brief Lattice Planner (状态格规划器)
 * 
 * ============================================================================
 *                         Lattice Planner 算法原理
 * ============================================================================
 * 
 * 1. 什么是 Lattice Planner？
 * ---------------------------
 * Lattice Planner 是一种基于采样的轨迹规划算法，主要用于自动驾驶和移动机器人。
 * 它的核心思想是：
 *   - 在状态空间中生成大量候选轨迹
 *   - 通过代价函数评估每条轨迹的质量
 *   - 选择代价最小的轨迹作为最优解
 * 
 * 2. Frenet 坐标系
 * ----------------
 * Lattice Planner 通常在 Frenet 坐标系中工作：
 * 
 *     参考线（全局路径）
 *     =====================>  s 方向（纵向，沿参考线）
 *            |
 *            |  d 方向（横向，垂直参考线）
 *            v
 * 
 * 优点：将复杂的 2D 规划分解为两个 1D 规划：
 *   - 纵向规划：决定速度曲线 s(t)
 *   - 横向规划：决定横向偏移 d(t)
 * 
 * 3. 轨迹生成方法
 * ---------------
 * 使用五次多项式（Quintic Polynomial）连接起点和终点：
 * 
 *   p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
 * 
 * 五次多项式可以满足 6 个边界条件：
 *   - 起点：位置、速度、加速度
 *   - 终点：位置、速度、加速度
 * 
 * 4. 采样策略
 * -----------
 *   ┌─────────────────────────────────────┐
 *   │  终点采样范围                         │
 *   │                                     │
 *   │    ○ ─ ─ ○ ─ ─ ○ ─ ─ ○ ─ ─ ○      │  ← t₁ 时刻
 *   │    │     │     │     │     │      │
 *   │    ○ ─ ─ ○ ─ ─ ○ ─ ─ ○ ─ ─ ○      │  ← t₂ 时刻
 *   │    │     │     │     │     │      │
 *   │    ○ ─ ─ ○ ─ ─ ○ ─ ─ ○ ─ ─ ○      │  ← t₃ 时刻
 *   │    ↑           ↑           ↑      │
 *   │   d=-2m       d=0        d=+2m    │
 *   └─────────────────────────────────────┘
 *         ↑ 当前位置
 * 
 * 5. 代价函数
 * -----------
 *   J = w₁·J_lateral + w₂·J_longitudinal + w₃·J_time + w₄·J_obstacle
 * 
 *   - J_lateral: 横向偏移和横向速度的代价
 *   - J_longitudinal: 纵向 jerk（加速度变化率）代价
 *   - J_time: 时间代价（鼓励快速到达）
 *   - J_obstacle: 障碍物碰撞代价
 * 
 * 6. 算法流程
 * -----------
 *   (1) 接收参考路径和当前状态
 *   (2) 将状态转换到 Frenet 坐标系
 *   (3) 生成横向轨迹候选集
 *   (4) 生成纵向轨迹候选集
 *   (5) 组合横向和纵向轨迹
 *   (6) 转换回笛卡尔坐标系
 *   (7) 碰撞检测和约束检查
 *   (8) 计算代价，选择最优轨迹
 * 
 * ============================================================================
 */

#include "local_planner/types.hpp"
#include "local_planner/utils/vehicle_model.hpp"
#include <vector>
#include <memory>

namespace local_planner {

/**
 * @brief 五次多项式
 * 
 * 用于生成满足边界条件的平滑曲线：
 *   p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
 */
class QuinticPolynomial {
public:
    /**
     * @brief 构造函数
     * @param x0 起点位置
     * @param v0 起点速度
     * @param a0 起点加速度
     * @param x1 终点位置
     * @param v1 终点速度
     * @param a1 终点加速度
     * @param T 时间长度
     */
    QuinticPolynomial(double x0, double v0, double a0,
                      double x1, double v1, double a1, double T);
    
    /** @brief 计算 t 时刻的位置 */
    double calcPosition(double t) const;
    
    /** @brief 计算 t 时刻的速度 */
    double calcVelocity(double t) const;
    
    /** @brief 计算 t 时刻的加速度 */
    double calcAcceleration(double t) const;
    
    /** @brief 计算 t 时刻的 jerk（加速度变化率） */
    double calcJerk(double t) const;

private:
    double a0_, a1_, a2_, a3_, a4_, a5_;  // 多项式系数
    double T_;  // 时间长度
};

/**
 * @brief 四次多项式
 * 
 * 用于纵向速度规划，满足 5 个边界条件：
 *   p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴
 */
class QuarticPolynomial {
public:
    /**
     * @brief 构造函数
     * @param x0 起点位置
     * @param v0 起点速度
     * @param a0 起点加速度
     * @param v1 终点速度
     * @param a1 终点加速度
     * @param T 时间长度
     */
    QuarticPolynomial(double x0, double v0, double a0,
                      double v1, double a1, double T);
    
    double calcPosition(double t) const;
    double calcVelocity(double t) const;
    double calcAcceleration(double t) const;
    double calcJerk(double t) const;

private:
    double a0_, a1_, a2_, a3_, a4_;
    double T_;
};

/**
 * @brief Frenet 轨迹
 * 
 * 在 Frenet 坐标系下的轨迹表示
 */
struct FrenetPath {
    // 时间序列
    std::vector<double> t;
    
    // 横向（d 方向）
    std::vector<double> d;
    std::vector<double> d_dot;
    std::vector<double> d_ddot;
    std::vector<double> d_dddot;  // jerk
    
    // 纵向（s 方向）
    std::vector<double> s;
    std::vector<double> s_dot;
    std::vector<double> s_ddot;
    std::vector<double> s_dddot;  // jerk
    
    // 笛卡尔坐标（转换后）
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> theta;
    std::vector<double> kappa;
    
    // 代价
    double cost_lateral = 0.0;
    double cost_longitudinal = 0.0;
    double cost_obstacle = 0.0;
    double cost_total = std::numeric_limits<double>::max();
    
    bool is_valid = false;
};

/**
 * @brief Lattice Planner 主类
 */
class LatticePlanner {
public:
    /**
     * @brief 构造函数（使用默认阿克曼底盘）
     * @param config 配置参数
     * @param vehicle_params 车辆参数
     */
    LatticePlanner(const LatticeConfig& config, const VehicleParams& vehicle_params);
    
    /**
     * @brief 构造函数（指定底盘模型类型）
     * @param config 配置参数
     * @param vehicle_params 车辆参数
     * @param chassis_type 底盘模型类型
     */
    LatticePlanner(const LatticeConfig& config, const VehicleParams& vehicle_params, 
                   ChassisModelType chassis_type);
    
    /**
     * @brief 析构函数
     */
    ~LatticePlanner();
    
    /**
     * @brief 规划轨迹
     * @param current_state 当前状态
     * @param reference_path 参考路径（全局路径）
     * @param obstacles 障碍物列表
     * @param stop_at_end 是否需要在终点停止（用于接近全局终点时）
     * @param target_end_s 目标终点的弧长位置（仅当 stop_at_end=true 时使用，-1表示使用参考路径终点）
     * @return 最优轨迹
     */
    Trajectory plan(const State& current_state,
                    const std::vector<State>& reference_path,
                    const std::vector<Obstacle>& obstacles,
                    bool stop_at_end = false,
                    double target_end_s = -1.0);
    
    /**
     * @brief 设置底盘模型类型
     * @param type 底盘模型类型
     */
    void setChassisModel(ChassisModelType type);
    
    /**
     * @brief 获取当前底盘模型类型
     * @return 底盘模型类型
     */
    ChassisModelType getChassisModelType() const { return chassis_type_; }
    
    /**
     * @brief 获取所有候选轨迹（用于可视化）
     */
    const std::vector<FrenetPath>& getCandidatePaths() const { return candidate_paths_; }

private:
    // ========== 内部方法 ==========
    
    /**
     * @brief 生成横向轨迹候选集
     * 
     * 采样不同的终点横向偏移 d_target 和时间 T，
     * 使用五次多项式生成轨迹。
     * 
     * @param current_state 当前状态
     * @param target_speed 目标速度
     * @param stop_at_end 是否在终点停止（如果是，则只生成收敛到 d=0 的轨迹）
     */
    std::vector<FrenetPath> generateLateralPaths(const State& current_state, double target_speed,
                                                  bool stop_at_end = false);
    
    /**
     * @brief 生成纵向轨迹候选集
     * 
     * 采样不同的终点速度和时间，
     * 使用四次或五次多项式生成轨迹。
     * 
     * @param paths 横向轨迹候选集
     * @param current_state 当前状态
     * @param stop_at_end 是否在终点停止（使用五次多项式约束终点位置和速度）
     * @param target_s 目标终点弧长（仅当 stop_at_end=true 时使用）
     */
    void generateLongitudinalPaths(std::vector<FrenetPath>& paths, const State& current_state,
                                   bool stop_at_end = false, double target_s = -1.0);
    
    /**
     * @brief 将 Frenet 轨迹转换为笛卡尔坐标
     */
    void convertToCartesian(FrenetPath& path);
    
    /**
     * @brief 检查轨迹是否满足约束
     */
    bool checkConstraints(const FrenetPath& path) const;
    
    /**
     * @brief 检查轨迹是否与障碍物碰撞
     */
    bool checkCollision(const FrenetPath& path, const std::vector<Obstacle>& obstacles) const;
    
    /**
     * @brief 计算轨迹代价
     */
    void calculateCost(FrenetPath& path) const;
    
    /**
     * @brief 将 Frenet 轨迹转换为 Trajectory 格式
     */
    Trajectory convertToTrajectory(const FrenetPath& path) const;
    
    /**
     * @brief 在参考路径上找到最近点
     */
    size_t findNearestPoint(double x, double y) const;
    
    /**
     * @brief 将笛卡尔坐标转换为 Frenet 坐标
     */
    void cartesianToFrenet(double x, double y, double& s, double& d) const;
    
    /**
     * @brief 将 Frenet 坐标转换为笛卡尔坐标
     */
    void frenetToCartesian(double s, double d, double& x, double& y, double& theta) const;
    
    // ========== 成员变量 ==========
    LatticeConfig config_;
    VehicleParams vehicle_params_;
    std::unique_ptr<BicycleModel> vehicle_model_;
    std::unique_ptr<ChassisModel> chassis_model_;  // 底盘模型指针（用于机动决策）
    ChassisModelType chassis_type_;  // 当前底盘模型类型
    
    std::vector<State> reference_path_;  // 缓存的参考路径
    std::vector<double> reference_s_;    // 参考路径的累积弧长
    
    std::vector<FrenetPath> candidate_paths_;  // 候选轨迹集合
    
    // ========== 静态变量 ==========
    static bool g_in_reverse_mode;  // 是否处于倒车/机动模式
    static int g_reverse_hold_counter;  // 模式保持计数器
};

// 静态变量在 cpp 中定义
// bool LatticePlanner::g_in_reverse_mode = false;
// int LatticePlanner::g_reverse_hold_counter = 0;

} // namespace local_planner

#endif // LOCAL_PLANNER_LATTICE_PLANNER_HPP
