#ifndef LOCAL_PLANNER_ILQR_HPP
#define LOCAL_PLANNER_ILQR_HPP

/**
 * @file ilqr.hpp
 * @brief iLQR (iterative Linear Quadratic Regulator) 算法
 * 
 * ============================================================================
 *                           iLQR 算法原理详解
 * ============================================================================
 * 
 * 1. 什么是 iLQR？
 * ----------------
 * iLQR 是一种轨迹优化算法，用于求解非线性最优控制问题。
 * 它是 DDP (Differential Dynamic Programming) 的简化版本，
 * 忽略了动力学的二阶导数，计算更高效。
 * 
 * 问题形式：
 *   min  J = Φ(x_N) + Σ_{k=0}^{N-1} L(x_k, u_k)
 *   s.t. x_{k+1} = f(x_k, u_k)
 * 
 * 其中：
 *   - x_k: 第 k 步的状态
 *   - u_k: 第 k 步的控制输入
 *   - f: 系统动力学（状态转移函数）
 *   - L: 运行代价（stage cost）
 *   - Φ: 终端代价（terminal cost）
 * 
 * 2. 算法核心思想
 * ----------------
 * iLQR 采用迭代优化的方式：
 *   (1) 从初始控制序列开始，前向仿真得到名义轨迹
 *   (2) 反向传播计算代价函数的二次近似和控制修正
 *   (3) 前向滚动应用控制修正，得到新轨迹
 *   (4) 重复直到收敛
 * 
 * 3. 代价函数的二次近似
 * ---------------------
 * 在名义轨迹附近，将代价函数泰勒展开到二阶：
 * 
 *   L(x+δx, u+δu) ≈ L + Lₓᵀδx + Lᵤᵀδu + ½δxᵀLₓₓδx + ½δuᵀLᵤᵤδu + δxᵀLₓᵤδu
 * 
 * 其中：
 *   - Lₓ = ∂L/∂x: 代价对状态的梯度
 *   - Lᵤ = ∂L/∂u: 代价对控制的梯度
 *   - Lₓₓ = ∂²L/∂x²: 状态 Hessian
 *   - Lᵤᵤ = ∂²L/∂u²: 控制 Hessian
 *   - Lₓᵤ = ∂²L/∂x∂u: 交叉 Hessian
 * 
 * 4. 反向传播（Backward Pass）
 * ----------------------------
 * 定义值函数 V_k(δx) = min_{δu} [Q_k(δx, δu)]
 * 
 * Q 函数（动作价值函数）的二次近似：
 *   Q_k(δx, δu) = Qₖ + Qₓᵀδx + Qᵤᵀδu + ½δxᵀQₓₓδx + ½δuᵀQᵤᵤδu + δxᵀQₓᵤδu
 * 
 * 其中 Q 函数的各项系数：
 *   Qₓ  = Lₓ  + Aᵀ Vₓ'
 *   Qᵤ  = Lᵤ  + Bᵀ Vₓ'
 *   Qₓₓ = Lₓₓ + Aᵀ Vₓₓ' A
 *   Qᵤᵤ = Lᵤᵤ + Bᵀ Vₓₓ' B
 *   Qₓᵤ = Lₓᵤ + Aᵀ Vₓₓ' B
 * 
 * 其中 A = ∂f/∂x, B = ∂f/∂u 是动力学的雅可比矩阵，
 * Vₓ', Vₓₓ' 是下一步值函数的梯度和 Hessian。
 * 
 * 最优控制修正：
 *   δu* = -Qᵤᵤ⁻¹(Qᵤ + Qₓᵤᵀδx) = k + K·δx
 * 
 * 其中：
 *   k = -Qᵤᵤ⁻¹ Qᵤ      (前馈项)
 *   K = -Qᵤᵤ⁻¹ Qₓᵤᵀ    (反馈增益矩阵)
 * 
 * 值函数更新：
 *   Vₓ  = Qₓ  + Kᵀ Qᵤᵤ k + Kᵀ Qᵤ + Qₓᵤ k
 *   Vₓₓ = Qₓₓ + Kᵀ Qᵤᵤ K + Kᵀ Qₓᵤᵀ + Qₓᵤ K
 * 
 * 5. 前向滚动（Forward Pass）
 * ---------------------------
 * 应用控制修正，考虑线搜索（line search）以保证代价下降：
 * 
 *   for α in [1.0, 0.5, 0.25, ...]:
 *       x̂₀ = x₀
 *       for k = 0 to N-1:
 *           δx = x̂_k - x̄_k
 *           û_k = ū_k + α·k_k + K_k·δx
 *           x̂_{k+1} = f(x̂_k, û_k)
 *       if J(x̂, û) < J(x̄, ū):
 *           accept new trajectory
 *           break
 * 
 * 6. 正则化（Regularization）
 * ---------------------------
 * 为了保证 Qᵤᵤ 正定（可逆），添加正则化项：
 *   Qᵤᵤ_reg = Qᵤᵤ + λ·I
 * 
 * λ 的调整策略：
 *   - 如果迭代成功（代价下降），减小 λ
 *   - 如果迭代失败（代价不下降），增大 λ
 * 
 * ============================================================================
 *                              算法流程图
 * ============================================================================
 * 
 *   初始化控制序列 ū
 *         ↓
 *   ┌─→ 前向仿真，得到名义轨迹 x̄
 *   │         ↓
 *   │   反向传播，计算 k, K
 *   │         ↓
 *   │   前向滚动 + 线搜索
 *   │         ↓
 *   │   收敛？ ─── Yes ──→ 输出最优轨迹
 *   │     │
 *   │    No
 *   │     │
 *   └─────┘
 * 
 * ============================================================================
 */

#include "local_planner/types.hpp"
#include "local_planner/utils/vehicle_model.hpp"
#include <vector>
#include <memory>
#include <Eigen/Dense>

namespace local_planner {

/**
 * @brief iLQR 优化器
 * 
 * 用于轨迹优化，将初始轨迹优化为满足终端约束且代价最小的轨迹。
 */
class ILQR {
public:
    // 类型定义
    static constexpr int STATE_DIM = 4;    // 状态维度 [x, y, theta, v]
    static constexpr int CONTROL_DIM = 2;  // 控制维度 [a, delta]
    
    using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
    using ControlVector = Eigen::Matrix<double, CONTROL_DIM, 1>;
    using StateMatrix = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    using ControlMatrix = Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>;
    using ControlStateMatrix = Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>;
    using StateControlMatrix = Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>;
    
    /**
     * @brief 构造函数
     * @param config iLQR 配置参数
     * @param vehicle_params 车辆参数
     */
    ILQR(const ILQRConfig& config, const VehicleParams& vehicle_params);
    
    /**
     * @brief 优化轨迹
     * 
     * @param initial_state 初始状态
     * @param goal_state 目标状态
     * @param initial_trajectory 初始轨迹猜测（可选）
     * @param obstacles 障碍物列表（用于避障代价）
     * @return 优化后的轨迹
     */
    Trajectory optimize(const State& initial_state,
                        const State& goal_state,
                        const Trajectory& initial_trajectory = Trajectory(),
                        const std::vector<Obstacle>& obstacles = {});
    
    /**
     * @brief 获取迭代历史（用于调试）
     */
    const std::vector<double>& getCostHistory() const { return cost_history_; }
    
    /**
     * @brief 设置参考路径（用于路径跟踪）
     */
    void setReferencePath(const std::vector<State>& reference_path);

private:
    // ========== 核心算法步骤 ==========
    
    /**
     * @brief 前向仿真
     * 
     * 给定初始状态和控制序列，模拟系统轨迹。
     */
    void forwardSimulation();
    
    /**
     * @brief 反向传播
     * 
     * 计算 Q 函数的二次近似，求解反馈增益 k 和 K。
     * 
     * @return 是否成功（Quu 正定）
     */
    bool backwardPass();
    
    /**
     * @brief 前向滚动
     * 
     * 应用控制修正，使用线搜索确保代价下降。
     * 
     * @return 是否成功（代价下降）
     */
    bool forwardPass();
    
    /**
     * @brief 计算总代价
     */
    double computeTotalCost();
    
    // ========== 代价函数相关 ==========
    
    /**
     * @brief 计算运行代价 L(x, u)
     * 
     * 二次代价函数：
     *   L = ½(x - x_ref)ᵀ Q (x - x_ref) + ½ uᵀ R u
     */
    double computeStageCost(const StateVector& x, const ControlVector& u, int k) const;
    
    /**
     * @brief 计算终端代价 Φ(x)
     * 
     *   Φ = ½(x - x_goal)ᵀ Qf (x - x_goal)
     */
    double computeTerminalCost(const StateVector& x) const;
    
    /**
     * @brief 计算代价函数的一阶和二阶导数
     */
    void computeCostDerivatives(const StateVector& x, const ControlVector& u, int k,
                                 StateVector& Lx, ControlVector& Lu,
                                 StateMatrix& Lxx, ControlMatrix& Luu,
                                 StateControlMatrix& Lxu) const;
    
    /**
     * @brief 计算终端代价的导数
     */
    void computeTerminalCostDerivatives(const StateVector& x,
                                         StateVector& Vx, StateMatrix& Vxx) const;
    
    // ========== 辅助函数 ==========
    
    /**
     * @brief 初始化控制序列
     */
    void initializeControlSequence(const State& initial_state, const State& goal_state);
    
    /**
     * @brief 检查收敛
     */
    bool checkConvergence() const;
    
    /**
     * @brief 调整正则化参数
     */
    void adjustRegularization(bool success);
    
    // ========== 成员变量 ==========
    ILQRConfig config_;
    VehicleParams vehicle_params_;
    std::unique_ptr<BicycleModel> vehicle_model_;
    
    // 规划参数
    int horizon_;                // 规划步数
    double dt_;                  // 时间步长
    StateVector goal_state_;     // 目标状态
    
    // 轨迹数据
    std::vector<StateVector> x_;         // 状态序列
    std::vector<ControlVector> u_;       // 控制序列
    std::vector<StateVector> x_new_;     // 新状态序列（前向滚动）
    std::vector<ControlVector> u_new_;   // 新控制序列
    
    // 反向传播结果
    std::vector<ControlVector> k_;       // 前馈增益
    std::vector<ControlStateMatrix> K_;  // 反馈增益矩阵
    
    // 参考路径（可选）
    std::vector<StateVector> reference_states_;
    
    // 代价权重矩阵
    StateMatrix Q_;     // 状态代价权重
    ControlMatrix R_;   // 控制代价权重
    StateMatrix Qf_;    // 终端代价权重
    
    // 正则化参数
    double lambda_;     // 当前正则化参数
    double delta_V_[2]; // 预期代价变化 [线性项, 二次项]
    
    // 迭代信息
    double current_cost_;
    std::vector<double> cost_history_;
    
    // 障碍物（用于避障）
    std::vector<Obstacle> obstacles_;
};

} // namespace local_planner

#endif // LOCAL_PLANNER_ILQR_HPP
