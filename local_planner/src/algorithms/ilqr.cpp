#include "local_planner/algorithms/ilqr.hpp"
#include <iostream>
#include <algorithm>

namespace local_planner {

ILQR::ILQR(const ILQRConfig& config, const VehicleParams& vehicle_params)
    : config_(config), vehicle_params_(vehicle_params) {
    
    vehicle_model_ = std::make_unique<BicycleModel>(vehicle_params_);
    
    // 计算规划步数
    horizon_ = static_cast<int>(config_.horizon_time / config_.dt);
    dt_ = config_.dt;
    
    // 初始化代价权重矩阵
    // Q 矩阵：状态跟踪代价权重
    Q_ = StateMatrix::Zero();
    Q_(0, 0) = config_.weight_x;      // x 误差权重
    Q_(1, 1) = config_.weight_y;      // y 误差权重
    Q_(2, 2) = config_.weight_theta;  // theta 误差权重
    Q_(3, 3) = config_.weight_v;      // v 误差权重
    
    // R 矩阵：控制代价权重
    R_ = ControlMatrix::Zero();
    R_(0, 0) = config_.weight_acceleration;  // 加速度代价
    R_(1, 1) = config_.weight_steering;      // 转向代价
    
    // Qf 矩阵：终端代价权重（通常比 Q 大，强调到达目标）
    Qf_ = Q_ * config_.weight_terminal;
    
    // 初始化正则化参数
    lambda_ = config_.initial_lambda;
    
    std::cout << "[iLQR] Initialized with horizon = " << horizon_ 
              << ", dt = " << dt_ << "s" << std::endl;
}

Trajectory ILQR::optimize(const State& initial_state,
                           const State& goal_state,
                           const Trajectory& initial_trajectory,
                           const std::vector<Obstacle>& obstacles) {
    /**
     * iLQR 主优化循环
     * 
     * 算法步骤：
     * 1. 初始化控制序列（从初始轨迹或零初始化）
     * 2. 前向仿真得到名义轨迹
     * 3. 反向传播计算增益
     * 4. 前向滚动应用增益
     * 5. 检查收敛，重复 3-4
     */
    
    obstacles_ = obstacles;
    goal_state_ = BicycleModel::stateToVector(goal_state);
    
    // Step 1: 初始化
    if (initial_trajectory.is_valid && !initial_trajectory.points.empty()) {
        // 从初始轨迹提取控制序列
        x_.resize(horizon_ + 1);
        u_.resize(horizon_);
        
        for (int k = 0; k <= horizon_; ++k) {
            size_t idx = std::min(static_cast<size_t>(k), initial_trajectory.points.size() - 1);
            x_[k] = BicycleModel::stateToVector(initial_trajectory.points[idx].state);
            if (k < horizon_) {
                u_[k] = initial_trajectory.points[idx].control.toVector();
            }
        }
    } else {
        initializeControlSequence(initial_state, goal_state);
    }
    
    // Step 2: 前向仿真得到初始轨迹
    x_[0] = BicycleModel::stateToVector(initial_state);
    forwardSimulation();
    current_cost_ = computeTotalCost();
    cost_history_.clear();
    cost_history_.push_back(current_cost_);
    
    std::cout << "[iLQR] Initial cost: " << current_cost_ << std::endl;
    
    // 主迭代循环
    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        // Step 3: 反向传播
        bool backward_success = backwardPass();
        
        if (!backward_success) {
            // Quu 不正定，增加正则化
            adjustRegularization(false);
            std::cout << "[iLQR] Iter " << iter << ": Backward pass failed, lambda = " 
                      << lambda_ << std::endl;
            continue;
        }
        
        // Step 4: 前向滚动
        bool forward_success = forwardPass();
        
        if (forward_success) {
            // 接受新轨迹
            x_ = x_new_;
            u_ = u_new_;
            current_cost_ = computeTotalCost();
            cost_history_.push_back(current_cost_);
            
            adjustRegularization(true);
            
            std::cout << "[iLQR] Iter " << iter << ": cost = " << current_cost_ 
                      << ", lambda = " << lambda_ << std::endl;
            
            // Step 5: 检查收敛
            if (checkConvergence()) {
                std::cout << "[iLQR] Converged after " << iter + 1 << " iterations" << std::endl;
                break;
            }
        } else {
            // 线搜索失败，增加正则化
            adjustRegularization(false);
            std::cout << "[iLQR] Iter " << iter << ": Forward pass failed, lambda = " 
                      << lambda_ << std::endl;
        }
    }
    
    // 构建输出轨迹
    Trajectory result;
    result.is_valid = true;
    result.total_cost = current_cost_;
    result.total_time = horizon_ * dt_;
    
    for (int k = 0; k <= horizon_; ++k) {
        TrajectoryPoint pt;
        pt.state = BicycleModel::vectorToState(x_[k]);
        pt.state.t = k * dt_;
        
        if (k < horizon_) {
            pt.control = Control::fromVector(u_[k]);
        }
        
        result.points.push_back(pt);
    }
    
    result.total_length = result.getLength();
    
    return result;
}

void ILQR::initializeControlSequence(const State& initial_state, const State& goal_state) {
    /**
     * 初始化控制序列
     * 
     * 简单策略：使用零控制或朝向目标的简单控制
     */
    
    x_.resize(horizon_ + 1);
    u_.resize(horizon_);
    x_new_.resize(horizon_ + 1);
    u_new_.resize(horizon_);
    k_.resize(horizon_);
    K_.resize(horizon_);
    
    // 计算初始控制（简单的 PD 控制器近似）
    double dx = goal_state.x - initial_state.x;
    double dy = goal_state.y - initial_state.y;
    double dist = std::hypot(dx, dy);
    double desired_speed = std::min(dist / config_.horizon_time, vehicle_params_.max_speed);
    
    for (int k = 0; k < horizon_; ++k) {
        // 简单的加速控制
        double t = k * dt_;
        double progress = t / config_.horizon_time;
        
        // 加速 - 巡航 - 减速
        double acc = 0.0;
        if (progress < 0.3) {
            acc = vehicle_params_.max_acceleration * 0.5;
        } else if (progress > 0.7) {
            acc = -vehicle_params_.max_deceleration * 0.5;
        }
        
        u_[k] = ControlVector(acc, 0.0);
        k_[k] = ControlVector::Zero();
        K_[k] = ControlStateMatrix::Zero();
    }
    
    x_[0] = BicycleModel::stateToVector(initial_state);
}

void ILQR::forwardSimulation() {
    /**
     * 前向仿真：使用当前控制序列模拟轨迹
     * 
     * x_{k+1} = f(x_k, u_k)
     */
    
    for (int k = 0; k < horizon_; ++k) {
        State current = BicycleModel::vectorToState(x_[k]);
        Control control = Control::fromVector(u_[k]);
        
        // 裁剪控制输入
        control = vehicle_model_->clipControl(control);
        u_[k] = control.toVector();
        
        State next = vehicle_model_->forward(current, control, dt_);
        x_[k + 1] = BicycleModel::stateToVector(next);
    }
}

bool ILQR::backwardPass() {
    /**
     * 反向传播：计算 Q 函数的二次近似和最优增益
     * 
     * 从终端开始反向计算：
     * 1. 初始化 Vx, Vxx 为终端代价的导数
     * 2. 对于 k = N-1, ..., 0:
     *    - 计算 Q 函数的系数 Qx, Qu, Qxx, Quu, Qux
     *    - 计算最优增益 k, K
     *    - 更新 Vx, Vxx
     */
    
    // 终端代价初始化
    StateVector Vx;
    StateMatrix Vxx;
    computeTerminalCostDerivatives(x_[horizon_], Vx, Vxx);
    
    // 预期代价变化
    delta_V_[0] = 0.0;
    delta_V_[1] = 0.0;
    
    // 反向迭代
    for (int k = horizon_ - 1; k >= 0; --k) {
        // 获取动力学雅可比矩阵
        State state_k = BicycleModel::vectorToState(x_[k]);
        Control control_k = Control::fromVector(u_[k]);
        
        Eigen::Matrix4d A = vehicle_model_->getStateJacobian(state_k, control_k, dt_);
        Eigen::Matrix<double, 4, 2> B = vehicle_model_->getControlJacobian(state_k, control_k, dt_);
        
        // 获取代价函数导数
        StateVector Lx;
        ControlVector Lu;
        StateMatrix Lxx;
        ControlMatrix Luu;
        StateControlMatrix Lxu;
        computeCostDerivatives(x_[k], u_[k], k, Lx, Lu, Lxx, Luu, Lxu);
        
        // 计算 Q 函数系数
        // Qx = Lx + A^T * Vx
        StateVector Qx = Lx + A.transpose() * Vx;
        
        // Qu = Lu + B^T * Vx
        ControlVector Qu = Lu + B.transpose() * Vx;
        
        // Qxx = Lxx + A^T * Vxx * A
        StateMatrix Qxx = Lxx + A.transpose() * Vxx * A;
        
        // Quu = Luu + B^T * Vxx * B
        ControlMatrix Quu = Luu + B.transpose() * Vxx * B;
        
        // Qux = Lxu^T + B^T * Vxx * A
        ControlStateMatrix Qux = Lxu.transpose() + B.transpose() * Vxx * A;
        
        // 添加正则化（确保 Quu 正定）
        ControlMatrix Quu_reg = Quu + lambda_ * ControlMatrix::Identity();
        
        // 检查 Quu_reg 是否正定
        Eigen::LLT<ControlMatrix> llt(Quu_reg);
        if (llt.info() != Eigen::Success) {
            // 不正定，返回失败
            return false;
        }
        
        // 计算增益
        // k = -Quu^{-1} * Qu
        // K = -Quu^{-1} * Qux
        ControlMatrix Quu_inv = Quu_reg.inverse();
        k_[k] = -Quu_inv * Qu;
        K_[k] = -Quu_inv * Qux;
        
        // 更新值函数
        // Vx = Qx + K^T * Quu * k + K^T * Qu + Qux^T * k
        Vx = Qx + K_[k].transpose() * Quu * k_[k] + K_[k].transpose() * Qu + Qux.transpose() * k_[k];
        
        // Vxx = Qxx + K^T * Quu * K + K^T * Qux + Qux^T * K
        Vxx = Qxx + K_[k].transpose() * Quu * K_[k] + K_[k].transpose() * Qux + Qux.transpose() * K_[k];
        
        // 确保 Vxx 对称
        Vxx = 0.5 * (Vxx + Vxx.transpose());
        
        // 累积预期代价变化
        delta_V_[0] += (k_[k].transpose() * Qu)(0, 0);
        delta_V_[1] += 0.5 * (k_[k].transpose() * Quu * k_[k])(0, 0);
    }
    
    return true;
}

bool ILQR::forwardPass() {
    /**
     * 前向滚动：应用控制修正，使用线搜索
     * 
     * 对于不同的步长 α：
     *   δx = x̂ - x̄
     *   û = ū + α*k + K*δx
     *   x̂_{next} = f(x̂, û)
     */
    
    // 线搜索步长
    std::vector<double> alphas = {1.0, 0.5, 0.25, 0.125, 0.0625};
    
    for (double alpha : alphas) {
        x_new_[0] = x_[0];  // 初始状态不变
        double new_cost = 0.0;
        
        // 前向滚动
        for (int k = 0; k < horizon_; ++k) {
            // 状态偏差
            StateVector dx = x_new_[k] - x_[k];
            
            // 控制修正
            u_new_[k] = u_[k] + alpha * k_[k] + K_[k] * dx;
            
            // 裁剪控制
            Control control = Control::fromVector(u_new_[k]);
            control = vehicle_model_->clipControl(control);
            u_new_[k] = control.toVector();
            
            // 状态转移
            State state_k = BicycleModel::vectorToState(x_new_[k]);
            State next = vehicle_model_->forward(state_k, control, dt_);
            x_new_[k + 1] = BicycleModel::stateToVector(next);
            
            // 累积代价
            new_cost += computeStageCost(x_new_[k], u_new_[k], k);
        }
        
        // 终端代价
        new_cost += computeTerminalCost(x_new_[horizon_]);
        
        // 检查代价是否下降（Armijo 条件）
        double expected_reduction = -alpha * (delta_V_[0] + alpha * delta_V_[1]);
        double actual_reduction = current_cost_ - new_cost;
        
        if (actual_reduction > 0.0 && actual_reduction > 0.0001 * expected_reduction) {
            // 接受
            return true;
        }
    }
    
    // 所有步长都失败
    return false;
}

double ILQR::computeTotalCost() {
    double cost = 0.0;
    
    for (int k = 0; k < horizon_; ++k) {
        cost += computeStageCost(x_[k], u_[k], k);
    }
    
    cost += computeTerminalCost(x_[horizon_]);
    
    return cost;
}

double ILQR::computeStageCost(const StateVector& x, const ControlVector& u, int k) const {
    /**
     * 运行代价（二次形式）：
     *   L = ½(x - x_ref)^T Q (x - x_ref) + ½ u^T R u
     * 
     * 如果有参考路径，使用参考路径上的对应点
     * 否则使用目标状态作为参考
     */
    
    StateVector x_ref;
    if (!reference_states_.empty() && k < static_cast<int>(reference_states_.size())) {
        x_ref = reference_states_[k];
    } else {
        // 使用从起点到目标的线性插值
        double progress = static_cast<double>(k) / horizon_;
        x_ref = (1.0 - progress) * x_[0] + progress * goal_state_;
    }
    
    StateVector dx = x - x_ref;
    
    // 归一化角度误差
    while (dx(2) > M_PI) dx(2) -= 2.0 * M_PI;
    while (dx(2) < -M_PI) dx(2) += 2.0 * M_PI;
    
    double state_cost = 0.5 * dx.transpose() * Q_ * dx;
    double control_cost = 0.5 * u.transpose() * R_ * u;
    
    return state_cost + control_cost;
}

double ILQR::computeTerminalCost(const StateVector& x) const {
    /**
     * 终端代价：
     *   Φ = ½(x - x_goal)^T Qf (x - x_goal)
     */
    
    StateVector dx = x - goal_state_;
    
    // 归一化角度误差
    while (dx(2) > M_PI) dx(2) -= 2.0 * M_PI;
    while (dx(2) < -M_PI) dx(2) += 2.0 * M_PI;
    
    return 0.5 * dx.transpose() * Qf_ * dx;
}

void ILQR::computeCostDerivatives(const StateVector& x, const ControlVector& u, int k,
                                   StateVector& Lx, ControlVector& Lu,
                                   StateMatrix& Lxx, ControlMatrix& Luu,
                                   StateControlMatrix& Lxu) const {
    /**
     * 代价函数的一阶和二阶导数
     * 
     * 对于二次代价 L = ½(x-xref)^T Q (x-xref) + ½ u^T R u：
     *   Lx = Q (x - xref)
     *   Lu = R u
     *   Lxx = Q
     *   Luu = R
     *   Lxu = 0
     */
    
    StateVector x_ref;
    if (!reference_states_.empty() && k < static_cast<int>(reference_states_.size())) {
        x_ref = reference_states_[k];
    } else {
        double progress = static_cast<double>(k) / horizon_;
        x_ref = (1.0 - progress) * x_[0] + progress * goal_state_;
    }
    
    StateVector dx = x - x_ref;
    
    // 归一化角度误差
    while (dx(2) > M_PI) dx(2) -= 2.0 * M_PI;
    while (dx(2) < -M_PI) dx(2) += 2.0 * M_PI;
    
    Lx = Q_ * dx;
    Lu = R_ * u;
    Lxx = Q_;
    Luu = R_;
    Lxu = StateControlMatrix::Zero();
}

void ILQR::computeTerminalCostDerivatives(const StateVector& x,
                                           StateVector& Vx, StateMatrix& Vxx) const {
    StateVector dx = x - goal_state_;
    
    while (dx(2) > M_PI) dx(2) -= 2.0 * M_PI;
    while (dx(2) < -M_PI) dx(2) += 2.0 * M_PI;
    
    Vx = Qf_ * dx;
    Vxx = Qf_;
}

bool ILQR::checkConvergence() const {
    /**
     * 收敛判断：
     * 1. 代价变化小于阈值
     * 2. 增益 k 的范数小于阈值
     */
    
    if (cost_history_.size() < 2) {
        return false;
    }
    
    double cost_change = std::abs(cost_history_.back() - cost_history_[cost_history_.size() - 2]);
    double relative_change = cost_change / (cost_history_.back() + 1e-10);
    
    return relative_change < config_.tolerance;
}

void ILQR::adjustRegularization(bool success) {
    /**
     * 调整正则化参数 λ
     * 
     * 成功时减小 λ（允许更大的步长）
     * 失败时增大 λ（更保守的步长）
     */
    
    if (success) {
        lambda_ = std::max(config_.min_lambda, lambda_ / config_.lambda_factor);
    } else {
        lambda_ = std::min(config_.max_lambda, lambda_ * config_.lambda_factor);
    }
}

void ILQR::setReferencePath(const std::vector<State>& reference_path) {
    reference_states_.clear();
    for (const auto& state : reference_path) {
        reference_states_.push_back(BicycleModel::stateToVector(state));
    }
}

} // namespace local_planner
