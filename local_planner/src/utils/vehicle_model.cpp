#include "local_planner/utils/vehicle_model.hpp"
#include <algorithm>

namespace local_planner {

BicycleModel::BicycleModel(const VehicleParams& params) : params_(params) {}

State BicycleModel::dynamics(const State& state, const Control& control) const {
    /**
     * 自行车运动学方程（连续时间）：
     * 
     *   ẋ = v · cos(θ)
     *   ẏ = v · sin(θ)
     *   θ̇ = v · tan(δ) / L
     *   v̇ = a
     * 
     * 其中 δ 是前轮转角，L 是轴距，a 是加速度
     */
    
    State deriv;
    
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double tan_delta = std::tan(control.steering_angle);
    
    deriv.x = state.v * cos_theta;
    deriv.y = state.v * sin_theta;
    deriv.theta = state.v * tan_delta / params_.wheelbase;
    deriv.v = control.acceleration;
    
    // 曲率变化率（可用于更高阶模型）
    deriv.kappa = 0.0;  // 简化处理
    
    return deriv;
}

State BicycleModel::forward(const State& state, const Control& control, double dt) const {
    /**
     * 使用四阶龙格-库塔法 (RK4) 进行数值积分
     * 
     * RK4 公式：
     *   k1 = f(x, u)
     *   k2 = f(x + dt/2 · k1, u)
     *   k3 = f(x + dt/2 · k2, u)
     *   k4 = f(x + dt · k3, u)
     *   x_next = x + dt/6 · (k1 + 2·k2 + 2·k3 + k4)
     * 
     * RK4 比欧拉法精度更高，尤其在转弯时误差更小。
     */
    
    // k1 = f(state, control)
    State k1 = dynamics(state, control);
    
    // k2 = f(state + dt/2 * k1, control)
    State s2;
    s2.x = state.x + dt / 2.0 * k1.x;
    s2.y = state.y + dt / 2.0 * k1.y;
    s2.theta = state.theta + dt / 2.0 * k1.theta;
    s2.v = state.v + dt / 2.0 * k1.v;
    State k2 = dynamics(s2, control);
    
    // k3 = f(state + dt/2 * k2, control)
    State s3;
    s3.x = state.x + dt / 2.0 * k2.x;
    s3.y = state.y + dt / 2.0 * k2.y;
    s3.theta = state.theta + dt / 2.0 * k2.theta;
    s3.v = state.v + dt / 2.0 * k2.v;
    State k3 = dynamics(s3, control);
    
    // k4 = f(state + dt * k3, control)
    State s4;
    s4.x = state.x + dt * k3.x;
    s4.y = state.y + dt * k3.y;
    s4.theta = state.theta + dt * k3.theta;
    s4.v = state.v + dt * k3.v;
    State k4 = dynamics(s4, control);
    
    // 合成结果
    State next;
    next.x = state.x + dt / 6.0 * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    next.y = state.y + dt / 6.0 * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y);
    next.theta = state.theta + dt / 6.0 * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta);
    next.v = state.v + dt / 6.0 * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
    
    // 归一化航向角到 [-π, π]
    while (next.theta > M_PI) next.theta -= 2.0 * M_PI;
    while (next.theta < -M_PI) next.theta += 2.0 * M_PI;
    
    // 计算曲率
    next.kappa = params_.steeringToKappa(control.steering_angle);
    
    // 更新时间
    next.t = state.t + dt;
    
    return next;
}

Eigen::Matrix4d BicycleModel::getStateJacobian(const State& state, const Control& control, double dt) const {
    /**
     * 计算状态转移的雅可比矩阵 A = ∂f/∂x
     * 
     * 状态向量：x = [x, y, θ, v]ᵀ
     * 
     * 连续时间雅可比矩阵（解析解）：
     *       ∂ẋ/∂x  ∂ẋ/∂y  ∂ẋ/∂θ  ∂ẋ/∂v       0  0  -v·sin(θ)  cos(θ)
     * Ac = ∂ẏ/∂x  ∂ẏ/∂y  ∂ẏ/∂θ  ∂ẏ/∂v   =   0  0   v·cos(θ)  sin(θ)
     *       ∂θ̇/∂x  ∂θ̇/∂y  ∂θ̇/∂θ  ∂θ̇/∂v       0  0      0      tan(δ)/L
     *       ∂v̇/∂x  ∂v̇/∂y  ∂v̇/∂θ  ∂v̇/∂v       0  0      0         0
     * 
     * 离散化（一阶近似）：A = I + dt · Ac
     */
    
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double tan_delta = std::tan(control.steering_angle);
    
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    
    // 填充非零元素
    A(0, 2) = -state.v * sin_theta * dt;  // ∂x/∂θ
    A(0, 3) = cos_theta * dt;              // ∂x/∂v
    A(1, 2) = state.v * cos_theta * dt;    // ∂y/∂θ
    A(1, 3) = sin_theta * dt;              // ∂y/∂v
    A(2, 3) = tan_delta / params_.wheelbase * dt;  // ∂θ/∂v
    
    return A;
}

Eigen::Matrix<double, 4, 2> BicycleModel::getControlJacobian(
    const State& state, const Control& control, double dt) const {
    /**
     * 计算控制雅可比矩阵 B = ∂f/∂u
     * 
     * 控制向量：u = [a, δ]ᵀ
     * 
     * 连续时间雅可比矩阵：
     *       ∂ẋ/∂a  ∂ẋ/∂δ       0        0
     * Bc = ∂ẏ/∂a  ∂ẏ/∂δ   =   0        0
     *       ∂θ̇/∂a  ∂θ̇/∂δ       0   v·sec²(δ)/L
     *       ∂v̇/∂a  ∂v̇/∂δ       1        0
     * 
     * 其中 sec(δ) = 1/cos(δ)
     * 
     * 离散化：B = dt · Bc
     */
    
    double cos_delta = std::cos(control.steering_angle);
    double sec2_delta = 1.0 / (cos_delta * cos_delta);  // sec²(δ)
    
    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();
    
    B(2, 1) = state.v * sec2_delta / params_.wheelbase * dt;  // ∂θ/∂δ
    B(3, 0) = dt;  // ∂v/∂a
    
    return B;
}

Eigen::Vector4d BicycleModel::stateToVector(const State& state) {
    return Eigen::Vector4d(state.x, state.y, state.theta, state.v);
}

State BicycleModel::vectorToState(const Eigen::Vector4d& vec) {
    State state;
    state.x = vec(0);
    state.y = vec(1);
    state.theta = vec(2);
    state.v = vec(3);
    return state;
}

bool BicycleModel::isStateValid(const State& state) const {
    // 检查速度约束
    if (state.v > params_.max_speed || state.v < params_.min_speed) {
        return false;
    }
    
    // 检查曲率约束
    if (std::abs(state.kappa) > params_.max_curvature) {
        return false;
    }
    
    return true;
}

bool BicycleModel::isControlValid(const Control& control) const {
    // 检查加速度约束
    if (control.acceleration > params_.max_acceleration || 
        control.acceleration < -params_.max_deceleration) {
        return false;
    }
    
    // 检查转向角约束
    if (std::abs(control.steering_angle) > params_.max_steering_angle) {
        return false;
    }
    
    return true;
}

Control BicycleModel::clipControl(const Control& control) const {
    Control clipped = control;
    
    // 裁剪加速度
    clipped.acceleration = std::clamp(control.acceleration, 
                                       -params_.max_deceleration, 
                                       params_.max_acceleration);
    
    // 裁剪转向角
    clipped.steering_angle = std::clamp(control.steering_angle,
                                         -params_.max_steering_angle,
                                         params_.max_steering_angle);
    
    return clipped;
}

} // namespace local_planner
