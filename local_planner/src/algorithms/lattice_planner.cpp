#include "local_planner/algorithms/lattice_planner.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace local_planner {

// ============================================================================
//                          五次多项式实现
// ============================================================================

QuinticPolynomial::QuinticPolynomial(double x0, double v0, double a0,
                                     double x1, double v1, double a1, double T) 
    : T_(T) {
    /**
     * 五次多项式：p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
     * 
     * 边界条件：
     *   p(0) = x0,   p'(0) = v0,   p''(0) = a0
     *   p(T) = x1,   p'(T) = v1,   p''(T) = a1
     * 
     * 由边界条件可得前三个系数：
     *   a₀ = x0
     *   a₁ = v0
     *   a₂ = a0 / 2
     * 
     * 后三个系数需要解线性方程组。
     * 设 A·[a₃, a₄, a₅]ᵀ = b，其中：
     * 
     *     ┌ T³    T⁴    T⁵   ┐     ┌ x1 - x0 - v0·T - a0·T²/2 ┐
     * A = │ 3T²   4T³   5T⁴  │, b = │ v1 - v0 - a0·T           │
     *     └ 6T    12T²  20T³ ┘     └ a1 - a0                   └
     */
    
    a0_ = x0;
    a1_ = v0;
    a2_ = a0 / 2.0;
    
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    // 构建矩阵 A 和向量 b
    Eigen::Matrix3d A;
    A << T3,      T4,       T5,
         3.0*T2,  4.0*T3,   5.0*T4,
         6.0*T,   12.0*T2,  20.0*T3;
    
    Eigen::Vector3d b;
    b << x1 - a0_ - a1_*T - a2_*T2,
         v1 - a1_ - 2.0*a2_*T,
         a1 - 2.0*a2_;
    
    // 解线性方程组
    Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
    
    a3_ = x(0);
    a4_ = x(1);
    a5_ = x(2);
}

double QuinticPolynomial::calcPosition(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    return a0_ + a1_*t + a2_*t2 + a3_*t3 + a4_*t4 + a5_*t5;
}

double QuinticPolynomial::calcVelocity(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    return a1_ + 2.0*a2_*t + 3.0*a3_*t2 + 4.0*a4_*t3 + 5.0*a5_*t4;
}

double QuinticPolynomial::calcAcceleration(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    return 2.0*a2_ + 6.0*a3_*t + 12.0*a4_*t2 + 20.0*a5_*t3;
}

double QuinticPolynomial::calcJerk(double t) const {
    double t2 = t * t;
    return 6.0*a3_ + 24.0*a4_*t + 60.0*a5_*t2;
}

// ============================================================================
//                          四次多项式实现
// ============================================================================

QuarticPolynomial::QuarticPolynomial(double x0, double v0, double a0,
                                     double v1, double a1, double T)
    : T_(T) {
    /**
     * 四次多项式：p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴
     * 
     * 边界条件：
     *   p(0) = x0,   p'(0) = v0,   p''(0) = a0
     *   p'(T) = v1,  p''(T) = a1
     * 
     * 注意：四次多项式只能满足 5 个边界条件，不能约束终点位置。
     * 适用于纵向速度规划（只关心速度和加速度）。
     */
    
    a0_ = x0;
    a1_ = v0;
    a2_ = a0 / 2.0;
    
    double T2 = T * T;
    double T3 = T2 * T;
    
    // 构建 2x2 线性方程组
    Eigen::Matrix2d A;
    A << 3.0*T2,  4.0*T3,
         6.0*T,   12.0*T2;
    
    Eigen::Vector2d b;
    b << v1 - a1_ - 2.0*a2_*T,
         a1 - 2.0*a2_;
    
    Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);
    
    a3_ = x(0);
    a4_ = x(1);
}

double QuarticPolynomial::calcPosition(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    return a0_ + a1_*t + a2_*t2 + a3_*t3 + a4_*t4;
}

double QuarticPolynomial::calcVelocity(double t) const {
    double t2 = t * t;
    double t3 = t2 * t;
    return a1_ + 2.0*a2_*t + 3.0*a3_*t2 + 4.0*a4_*t3;
}

double QuarticPolynomial::calcAcceleration(double t) const {
    double t2 = t * t;
    return 2.0*a2_ + 6.0*a3_*t + 12.0*a4_*t2;
}

double QuarticPolynomial::calcJerk(double t) const {
    return 6.0*a3_ + 24.0*a4_*t;
}

// ============================================================================
//                          Lattice Planner 实现
// ============================================================================

LatticePlanner::LatticePlanner(const LatticeConfig& config, const VehicleParams& vehicle_params)
    : config_(config), vehicle_params_(vehicle_params) {
    vehicle_model_ = std::make_unique<BicycleModel>(vehicle_params_);
}

Trajectory LatticePlanner::plan(const State& current_state,
                                 const std::vector<State>& reference_path,
                                 const std::vector<Obstacle>& obstacles) {
    /**
     * Lattice Planner 主流程：
     * 
     * Step 1: 缓存参考路径并计算累积弧长
     * Step 2: 生成横向轨迹候选集（不同的 d_target 和 T）
     * Step 3: 生成纵向轨迹候选集（不同的速度曲线）
     * Step 4: 组合轨迹并转换到笛卡尔坐标系
     * Step 5: 碰撞检测和约束检查
     * Step 6: 计算代价并选择最优轨迹
     */
    
    // Step 1: 缓存参考路径
    reference_path_ = reference_path;
    reference_s_.clear();
    reference_s_.push_back(0.0);
    for (size_t i = 1; i < reference_path_.size(); ++i) {
        double ds = reference_path_[i].distanceTo(reference_path_[i-1]);
        reference_s_.push_back(reference_s_.back() + ds);
    }
    
    if (reference_path_.empty()) {
        std::cerr << "[LatticePlanner] Error: Empty reference path!" << std::endl;
        return Trajectory();
    }
    
    // Step 2 & 3: 生成候选轨迹
    // 注：横向轨迹生成不需要 target_speed，纵向速度在 generateLongitudinalPaths 中独立处理
    candidate_paths_ = generateLateralPaths(current_state, vehicle_params_.max_speed);
    
    // 生成纵向分量（对所有候选轨迹）
    generateLongitudinalPaths(candidate_paths_, current_state);
    
    // Step 4 & 5 & 6: 处理每条候选轨迹
    FrenetPath* best_path = nullptr;
    double min_cost = std::numeric_limits<double>::max();
    
    for (auto& path : candidate_paths_) {
        
        // 转换到笛卡尔坐标系
        convertToCartesian(path);
        
        // 检查约束
        if (!checkConstraints(path)) {
            path.is_valid = false;
            continue;
        }
        
        // 碰撞检测
        if (checkCollision(path, obstacles)) {
            path.is_valid = false;
            path.cost_obstacle = config_.weight_obstacle * 1000.0;
        }
        
        // 计算代价
        calculateCost(path);
        
        if (path.is_valid && path.cost_total < min_cost) {
            min_cost = path.cost_total;
            best_path = &path;
        }
    }
    
    // 返回最优轨迹
    if (best_path != nullptr) {
        // std::cout << "[LatticePlanner] Best path found, cost = " << min_cost << std::endl;
        return convertToTrajectory(*best_path);
    } else {
        std::cerr << "[LatticePlanner] No valid path found!" << std::endl;
        return Trajectory();
    }
}

std::vector<FrenetPath> LatticePlanner::generateLateralPaths(const State& current_state, 
                                                              double target_speed) {
    /**
     * 横向轨迹生成：
     * 
     * 1. 将当前状态转换到 Frenet 坐标系
     * 2. 采样不同的终点横向偏移 d_target ∈ [-max_lateral_offset, +max_lateral_offset]
     * 3. 采样不同的规划时间 T ∈ [min_planning_time, max_planning_time]
     * 4. 对于每个 (d_target, T) 组合，使用五次多项式生成横向轨迹
     * 
     *            d
     *            ^
     *            |     target 1 (d = +2m)
     *            |    ╱
     *            |   ╱
     *    --------+--●-----> s
     *            |   ╲
     *            |    ╲
     *            |     target 2 (d = -2m)
     *           当前位置
     */
    
    std::vector<FrenetPath> paths;
    
    // 获取当前 Frenet 坐标
    double current_s, current_d;
    cartesianToFrenet(current_state.x, current_state.y, current_s, current_d);
    
    // 计算当前的 d_dot 和 d_ddot（近似）
    double current_d_dot = current_state.v * std::sin(current_state.theta);  // 简化
    double current_d_ddot = 0.0;  // 简化为 0
    
    // 采样横向终点偏移
    double d_step = 2.0 * config_.max_lateral_offset / (config_.num_width_samples - 1);
    
    for (int i = 0; i < config_.num_width_samples; ++i) {
        double d_target = -config_.max_lateral_offset + i * d_step;
        
        // 采样规划时间
        double t_step = (config_.max_planning_time - config_.min_planning_time) / 
                        (config_.num_time_samples - 1);
        
        for (int j = 0; j < config_.num_time_samples; ++j) {
            double T = config_.min_planning_time + j * t_step;
            
            // 使用五次多项式生成横向轨迹
            // 边界条件：
            //   起点：d(0) = current_d, d'(0) = current_d_dot, d''(0) = current_d_ddot
            //   终点：d(T) = d_target, d'(T) = 0, d''(T) = 0
            QuinticPolynomial lat_poly(current_d, current_d_dot, current_d_ddot,
                                       d_target, 0.0, 0.0, T);
            
            FrenetPath path;
            
            // 离散化轨迹
            for (double t = 0.0; t <= T; t += config_.time_resolution) {
                path.t.push_back(t);
                path.d.push_back(lat_poly.calcPosition(t));
                path.d_dot.push_back(lat_poly.calcVelocity(t));
                path.d_ddot.push_back(lat_poly.calcAcceleration(t));
                path.d_dddot.push_back(lat_poly.calcJerk(t));
            }
            
            // 计算横向代价
            // J_lateral = ∫(d_dddot² + d_target² + d_dot²) dt
            double cost_lateral = 0.0;
            for (size_t k = 0; k < path.t.size(); ++k) {
                cost_lateral += path.d_dddot[k] * path.d_dddot[k];  // jerk 平方
            }
            cost_lateral *= config_.time_resolution;
            cost_lateral += config_.weight_lateral_offset * d_target * d_target;
            cost_lateral += config_.weight_time * T;
            
            path.cost_lateral = cost_lateral;
            path.is_valid = true;
            
            paths.push_back(path);
        }
    }
    
    return paths;
}

void LatticePlanner::generateLongitudinalPaths(std::vector<FrenetPath>& paths,
                                                const State& current_state) {
    /**
     * 纵向轨迹生成：
     * 
     * 对于每条横向轨迹，生成对应的纵向运动（速度曲线）。
     * 使用四次多项式，只约束终点速度，不约束终点位置。但是如果要求机器人停止到某个位置的话还是得用五次多项式
     * 
     * 边界条件：
     *   s(0) = current_s
     *   s'(0) = current_v (当前速度)
     *   s''(0) = current_a (当前加速度)
     *   s'(T) = target_v (目标速度)
     *   s''(T) = 0 (终点加速度为零)
     */
    
    double current_s, current_d;
    cartesianToFrenet(current_state.x, current_state.y, current_s, current_d);
    
    double current_v = current_state.v;
    double current_a = current_state.a;
    double target_v = vehicle_params_.max_speed * 0.8;  // 目标速度
    
    for (auto& path : paths) {
        if (!path.is_valid) continue;
        
        double T = path.t.back();  // 使用横向轨迹的时间
        
        // 四次多项式
        QuarticPolynomial lon_poly(current_s, current_v, current_a, target_v, 0.0, T);
        
        // 生成纵向轨迹
        path.s.clear();
        path.s_dot.clear();
        path.s_ddot.clear();
        path.s_dddot.clear();
        
        for (double t : path.t) {
            path.s.push_back(lon_poly.calcPosition(t));
            path.s_dot.push_back(lon_poly.calcVelocity(t));
            path.s_ddot.push_back(lon_poly.calcAcceleration(t));
            path.s_dddot.push_back(lon_poly.calcJerk(t));
        }
        
        // 计算纵向代价
        double cost_longitudinal = 0.0;
        for (size_t k = 0; k < path.t.size(); ++k) {
            cost_longitudinal += path.s_dddot[k] * path.s_dddot[k];  // jerk 平方
        }
        cost_longitudinal *= config_.time_resolution;
        
        path.cost_longitudinal = cost_longitudinal;
    }
}

void LatticePlanner::convertToCartesian(FrenetPath& path) {
    /**
     * 将 Frenet 坐标转换为笛卡尔坐标
     * 
     * 对于 Frenet 坐标 (s, d)：
     *   1. 在参考路径上找到 s 对应的点 (x_ref, y_ref, theta_ref)
     *   2. 计算横向偏移后的点：
     *      x = x_ref - d · sin(theta_ref)
     *      y = y_ref + d · cos(theta_ref)
     *   3. 计算航向角和曲率
     */
    
    path.x.clear();
    path.y.clear();
    path.theta.clear();
    path.kappa.clear();
    
    for (size_t i = 0; i < path.s.size(); ++i) {
        double x, y, theta;
        frenetToCartesian(path.s[i], path.d[i], x, y, theta);
        
        path.x.push_back(x);
        path.y.push_back(y);
        path.theta.push_back(theta);
        
        // 计算曲率（数值微分）
        if (i > 0 && i < path.s.size() - 1) {
            double dx = path.x[i] - path.x[i-1];
            double dy = path.y[i] - path.y[i-1];
            double ds = std::hypot(dx, dy);
            
            if (ds > 1e-6) {
                double dtheta = path.theta[i] - path.theta[i-1];
                // 归一化角度
                while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
                while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
                path.kappa.push_back(dtheta / ds);
            } else {
                path.kappa.push_back(0.0);
            }
        } else {
            path.kappa.push_back(0.0);
        }
    }
}

bool LatticePlanner::checkConstraints(const FrenetPath& path) const {
    /**
     * 检查轨迹是否满足运动学约束：
     *   - 速度约束
     *   - 加速度约束
     *   - 曲率约束
     */
    
    for (size_t i = 0; i < path.s.size(); ++i) {
        // 速度约束
        if (path.s_dot[i] > vehicle_params_.max_speed || 
            path.s_dot[i] < vehicle_params_.min_speed) {
            return false;
        }
        
        // 加速度约束
        if (path.s_ddot[i] > vehicle_params_.max_acceleration ||
            path.s_ddot[i] < -vehicle_params_.max_deceleration) {
            return false;
        }
        
        // 曲率约束
        if (i < path.kappa.size() && 
            std::abs(path.kappa[i]) > vehicle_params_.max_curvature) {
            return false;
        }
    }
    
    return true;
}

bool LatticePlanner::checkCollision(const FrenetPath& path, 
                                     const std::vector<Obstacle>& obstacles) const {
    /**
     * 碰撞检测：检查轨迹上的每个点是否与障碍物碰撞
     * 
     * 使用圆形近似进行快速检测：
     *   如果 distance(trajectory_point, obstacle) < safe_distance + obstacle_radius
     *   则认为发生碰撞
     */
    
    for (size_t i = 0; i < path.x.size(); ++i) {
        double t = path.t[i];
        
        for (const auto& obs : obstacles) {
            // 预测障碍物在 t 时刻的位置
            Obstacle pred_obs = obs.predictAt(t);
            
            double dist = std::hypot(path.x[i] - pred_obs.x, path.y[i] - pred_obs.y);
            double safe_dist = config_.safe_distance + pred_obs.radius;
            
            if (dist < safe_dist) {
                return true;  // 碰撞
            }
        }
    }
    
    return false;  // 无碰撞
}

void LatticePlanner::calculateCost(FrenetPath& path) const {
    /**
     * 计算轨迹总代价：
     * 
     * J_total = w_lateral · J_lateral + w_longitudinal · J_longitudinal + J_obstacle
     * 
     * 其中：
     *   - J_lateral: 横向代价（横向 jerk、横向偏移）
     *   - J_longitudinal: 纵向代价（纵向 jerk）
     *   - J_obstacle: 障碍物代价（碰撞惩罚）
     */
    
    path.cost_total = config_.weight_lateral_jerk * path.cost_lateral +
                      config_.weight_longitudinal_jerk * path.cost_longitudinal +
                      path.cost_obstacle;
}

Trajectory LatticePlanner::convertToTrajectory(const FrenetPath& path) const {
    Trajectory traj;
    traj.is_valid = path.is_valid;
    traj.total_cost = path.cost_total;
    
    for (size_t i = 0; i < path.x.size(); ++i) {
        TrajectoryPoint pt;
        pt.state.x = path.x[i];
        pt.state.y = path.y[i];
        pt.state.theta = path.theta[i];
        pt.state.v = path.s_dot[i];
        pt.state.a = path.s_ddot[i];
        pt.state.t = path.t[i];
        
        if (i < path.kappa.size()) {
            pt.state.kappa = path.kappa[i];
        }
        
        traj.points.push_back(pt);
    }
    
    traj.total_time = path.t.back();
    traj.total_length = traj.getLength();
    
    return traj;
}

size_t LatticePlanner::findNearestPoint(double x, double y) const {
    size_t nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < reference_path_.size(); ++i) {
        double dist = std::hypot(x - reference_path_[i].x, y - reference_path_[i].y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    return nearest_idx;
}

void LatticePlanner::cartesianToFrenet(double x, double y, double& s, double& d) const {
    /**
     * 笛卡尔坐标 → Frenet 坐标
     * 
     * 1. 找到参考路径上的最近点
     * 2. s = 最近点的累积弧长
     * 3. d = 到参考线的横向距离（带符号）
     */
    
    size_t idx = findNearestPoint(x, y);
    s = reference_s_[idx];
    
    // 计算横向偏移（带符号）
    double dx = x - reference_path_[idx].x;
    double dy = y - reference_path_[idx].y;
    double ref_theta = reference_path_[idx].theta;
    
    // d = (dx, dy) 在法向量方向的投影
    // 法向量 = (-sin(theta), cos(theta))
    d = -dx * std::sin(ref_theta) + dy * std::cos(ref_theta);
}

void LatticePlanner::frenetToCartesian(double s, double d, double& x, double& y, double& theta) const {
    /**
     * Frenet 坐标 → 笛卡尔坐标
     * 
     * 1. 在参考路径上找到 s 对应的点（插值）
     * 2. 沿法向量方向偏移 d
     */
    
    // 找到 s 在参考路径上的位置（线性插值）
    size_t idx = 0;
    for (size_t i = 0; i < reference_s_.size() - 1; ++i) {
        if (s >= reference_s_[i] && s <= reference_s_[i + 1]) {
            idx = i;
            break;
        }
    }
    
    // 限制索引范围
    if (idx >= reference_path_.size() - 1) {
        idx = reference_path_.size() - 2;
    }
    
    // 线性插值
    double ds = reference_s_[idx + 1] - reference_s_[idx];
    double t = (ds > 1e-6) ? (s - reference_s_[idx]) / ds : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    
    double x_ref = reference_path_[idx].x + t * (reference_path_[idx + 1].x - reference_path_[idx].x);
    double y_ref = reference_path_[idx].y + t * (reference_path_[idx + 1].y - reference_path_[idx].y);
    double theta_ref = reference_path_[idx].theta + t * (reference_path_[idx + 1].theta - reference_path_[idx].theta);
    
    // 沿法向量偏移
    x = x_ref - d * std::sin(theta_ref);
    y = y_ref + d * std::cos(theta_ref);
    theta = theta_ref;  // 简化处理
}

} // namespace local_planner
