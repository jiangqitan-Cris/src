#include "local_planner/utils/vehicle_model.hpp"
#include <algorithm>

namespace local_planner {

// ============================================================================
//                          底盘模型实现
// ============================================================================

// -------------------- AckermannModel --------------------

// 静态变量初始化
bool AckermannModel::s_in_reverse_mode_ = false;
int AckermannModel::s_locked_steering_direction_ = 0;  // 0=未锁定, 1=左打方向盘, -1=右打方向盘
double AckermannModel::s_initial_heading_diff_ = 0.0;
int AckermannModel::s_reverse_hold_counter_ = 0;

void AckermannModel::resetReverseState() {
    s_in_reverse_mode_ = false;
    s_locked_steering_direction_ = 0;
    s_initial_heading_diff_ = 0.0;
    s_reverse_hold_counter_ = 0;
    // 重置时不打印，减少输出
}

bool AckermannModel::needsSpecialManeuver(const State& current_state, 
                                           const State& target_state) const {
    // 计算目标航向
    double target_theta = target_state.theta;
    if (std::abs(target_state.v) < 0.1 && target_state.x != 0 && target_state.y != 0) {
        target_theta = std::atan2(target_state.y - current_state.y, 
                                  target_state.x - current_state.x);
    }
    
    // 计算航向差异
    double heading_diff = target_theta - current_state.theta;
    while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
    
    // 使用迟滞机制：进入阈值和退出阈值不同
    double enter_threshold = M_PI * 100.0 / 180.0;  // 100度进入倒车（更保守，减少不必要的倒车）
    double exit_threshold = M_PI * 75.0 / 180.0;    // 75度退出倒车（更早退出，不用倒太多）
    
    if (s_in_reverse_mode_) {
        // 当前在倒车模式中
        
        // 检查是否应该退出
        bool should_exit = false;
        
        // 条件1：航向差已经足够小
        if (std::abs(heading_diff) < exit_threshold) {
            should_exit = true;
        }
        
        // 条件2：航向差已经从初始值减少了足够多（至少减少了40%）
        // 这意味着倒车已经起到效果了
        if (s_initial_heading_diff_ != 0 && 
            std::abs(heading_diff) < std::abs(s_initial_heading_diff_) * 0.6) {
            should_exit = true;
        }
        
        // 条件3：倒车已经持续很长时间（200个周期，约20秒），强制退出
        if (s_reverse_hold_counter_ > 200) {
            std::cout << "[AckermannModel] Reverse forced exit due to timeout" << std::endl;
            should_exit = true;
        }
        
        if (should_exit) {
            std::cout << "[AckermannModel] Reverse complete! heading_diff=" 
                      << heading_diff * 180.0 / M_PI << " deg (initial=" 
                      << s_initial_heading_diff_ * 180.0 / M_PI << " deg)" << std::endl;
            s_in_reverse_mode_ = false;
            s_locked_steering_direction_ = 0;
            s_initial_heading_diff_ = 0.0;
            s_reverse_hold_counter_ = 0;
            return false;
        }
        
        // 继续保持倒车模式
        return true;
    } else {
        // 当前不在倒车模式，检查是否需要进入
        if (std::abs(heading_diff) > enter_threshold) {
            // 需要进入倒车模式
            s_in_reverse_mode_ = true;
            s_initial_heading_diff_ = heading_diff;
            
            // 倒车时转向效果相反：
            // heading_diff > 0 表示目标在左边，需要车头逆时针转 → 倒车时向右打方向盘
            // heading_diff < 0 表示目标在右边，需要车头顺时针转 → 倒车时向左打方向盘
            s_locked_steering_direction_ = (heading_diff > 0) ? -1 : 1;
            
            s_reverse_hold_counter_ = 0;
            std::cout << "[AckermannModel] Starting reverse: heading_diff=" 
                      << heading_diff * 180.0 / M_PI << " deg, steering=" 
                      << (s_locked_steering_direction_ > 0 ? "LEFT" : "RIGHT") << std::endl;
            return true;
        }
        return false;
    }
}

Trajectory AckermannModel::generateManeuverTrajectory(
    const State& current_state,
    const State& target_state,
    const std::vector<Obstacle>& obstacles) const {
    
    double target_theta = target_state.theta;
    if (std::abs(target_state.v) < 0.1 && target_state.x != 0 && target_state.y != 0) {
        target_theta = std::atan2(target_state.y - current_state.y, 
                                  target_state.x - current_state.x);
    }
    
    double heading_diff = target_theta - current_state.theta;
    while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
    
    // 倒车参数
    double reverse_speed = params_.min_speed;
    if (reverse_speed >= 0) reverse_speed = -0.3;
    double wheelbase = params_.wheelbase;
    const double reverse_safe_margin = 0.12;  // 倒车安全裕度（更小，允许更近距离通过）
    
    // 增加保持计数器
    s_reverse_hold_counter_++;
    
    // 每50个周期打印一次状态
    if (s_reverse_hold_counter_ % 50 == 0) {
        std::cout << "[AckermannModel] Reversing... heading_diff=" 
                  << heading_diff * 180.0 / M_PI << " deg, counter=" 
                  << s_reverse_hold_counter_ << std::endl;
    }
    
    // 辅助函数：生成轨迹（不做碰撞检测）
    auto generateTrajectory = [&](int steer_direction) -> Trajectory {
        Trajectory traj;
        double steering_angle = steer_direction * params_.max_steering_angle;
        
        // 计算倒车时间（但限制在合理范围内）
        double angular_rate = std::abs(reverse_speed * std::tan(steering_angle) / wheelbase);
        double required_time = std::abs(heading_diff) / angular_rate;
        required_time = std::clamp(required_time, 1.0, 3.0);  // 缩短最大倒车时间
        
        // 生成轨迹
        double dt = 0.1;
        int N = static_cast<int>(required_time / dt);
        State state = current_state;
        
        for (int k = 0; k <= N; ++k) {
            TrajectoryPoint pt;
            pt.state = state;
            pt.state.t = k * dt;
            pt.control.acceleration = 0.0;
            pt.control.steering_angle = steering_angle;
            traj.points.push_back(pt);
            
            if (k < N) {
                state.x += reverse_speed * std::cos(state.theta) * dt;
                state.y += reverse_speed * std::sin(state.theta) * dt;
                state.theta += reverse_speed * std::tan(steering_angle) / wheelbase * dt;
                state.v = reverse_speed;
                while (state.theta > M_PI) state.theta -= 2.0 * M_PI;
                while (state.theta < -M_PI) state.theta += 2.0 * M_PI;
            }
        }
        return traj;
    };
    
    // 辅助函数：检测碰撞
    auto checkCollision = [&](const Trajectory& traj) -> bool {
        for (const auto& pt : traj.points) {
            for (const auto& obs : obstacles) {
                double dist = std::hypot(pt.state.x - obs.x, pt.state.y - obs.y);
                double safe_dist = reverse_safe_margin + obs.radius;
                if (dist < safe_dist) {
                    return true;
                }
            }
        }
        return false;
    };
    
    // ========== 坚持使用锁定的方向 ==========
    Trajectory best_traj;
    int chosen_direction = s_locked_steering_direction_;
    
    // 如果没有锁定方向（不应该发生），使用理想方向
    if (chosen_direction == 0) {
        chosen_direction = (heading_diff > 0) ? -1 : 1;
        s_locked_steering_direction_ = chosen_direction;
    }
    
    // 生成锁定方向的轨迹
    best_traj = generateTrajectory(chosen_direction);
    bool has_collision = checkCollision(best_traj);
    
    // 如果锁定方向碰撞了，才尝试另一个方向
    if (has_collision) {
        // 记录切换前的方向
        int old_direction = chosen_direction;
        int alternative_direction = -chosen_direction;
        
        Trajectory alt_traj = generateTrajectory(alternative_direction);
        bool alt_collision = checkCollision(alt_traj);
        
        if (!alt_collision) {
            // 另一个方向可行，但要谨慎切换
            // 增加一个切换计数器，避免频繁切换
            static int switch_cooldown = 0;
            
            if (switch_cooldown <= 0) {
                // 可以切换
                std::cout << "[AckermannModel] WARNING: Switching reverse direction "
                          << (old_direction > 0 ? "LEFT" : "RIGHT") << " -> " 
                          << (alternative_direction > 0 ? "LEFT" : "RIGHT")
                          << " due to collision" << std::endl;
                
                best_traj = alt_traj;
                chosen_direction = alternative_direction;
                s_locked_steering_direction_ = alternative_direction;
                switch_cooldown = 30;  // 切换后冷却30个周期
            } else {
                // 冷却中，保持原方向，即使碰撞也继续（让上层处理失败）
                switch_cooldown--;
            }
        }
        // 如果两个方向都碰撞，保持原方向
    }
    
    // 设置轨迹有效性
    if (!best_traj.points.empty()) {
        best_traj.is_valid = true;
        best_traj.total_time = best_traj.points.back().state.t;
        best_traj.total_length = best_traj.getLength();
    } else {
        best_traj.is_valid = false;
    }
    
    return best_traj;
}

// -------------------- DifferentialModel --------------------

bool DifferentialModel::needsSpecialManeuver(const State& current_state, 
                                              const State& target_state) const {
    double target_theta = target_state.theta;
    if (std::abs(target_state.v) < 0.1 && target_state.x != 0 && target_state.y != 0) {
        target_theta = std::atan2(target_state.y - current_state.y, 
                                  target_state.x - current_state.x);
    }
    
    double heading_diff = target_theta - current_state.theta;
    while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
    
    // 差速底盘阈值设为30度
    double enter_threshold = M_PI * 30.0 / 180.0;
    return std::abs(heading_diff) > enter_threshold;
}

Trajectory DifferentialModel::generateManeuverTrajectory(
    const State& current_state,
    const State& target_state,
    const std::vector<Obstacle>& obstacles) const {
    
    Trajectory traj;
    double target_theta = target_state.theta;
    if (std::abs(target_state.v) < 0.1 && target_state.x != 0 && target_state.y != 0) {
        target_theta = std::atan2(target_state.y - current_state.y, 
                                  target_state.x - current_state.x);
    }
    
    double heading_diff = target_theta - current_state.theta;
    while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
    
    // 原地旋转
    double rotation_speed = std::max(params_.max_speed * 0.3, 0.3);
    double angular_velocity = (heading_diff > 0) ? rotation_speed : -rotation_speed;
    double required_time = std::abs(heading_diff) / rotation_speed;
    required_time = std::clamp(required_time, 0.5, 3.0);
    
    double dt = 0.1;
    int N = static_cast<int>(required_time / dt);
    State state = current_state;
    
    for (int k = 0; k <= N; ++k) {
        TrajectoryPoint pt;
        pt.state = state;
        pt.state.t = k * dt;
        pt.state.v = 0.0;
        pt.state.a = 0.0;
        pt.control.acceleration = 0.0;
        pt.control.steering_angle = angular_velocity * 2.0;
        traj.points.push_back(pt);
        
        if (k < N) {
            state.theta += angular_velocity * dt;
            while (state.theta > M_PI) state.theta -= 2.0 * M_PI;
            while (state.theta < -M_PI) state.theta += 2.0 * M_PI;
        }
    }
    
    if (!traj.points.empty()) {
        traj.points.back().state.theta = target_theta;
    }
    
    // 碰撞检测
    bool has_collision = false;
    for (const auto& pt : traj.points) {
        for (const auto& obs : obstacles) {
            if (std::hypot(pt.state.x - obs.x, pt.state.y - obs.y) < 0.2 + obs.radius) {
                has_collision = true;
                break;
            }
        }
        if (has_collision) break;
    }
    
    if (has_collision) traj.points.clear();
    traj.is_valid = !traj.points.empty() && !has_collision;
    traj.total_time = traj.is_valid ? traj.points.back().state.t : 0.0;
    traj.total_length = 0.0;
    
    return traj;
}

// -------------------- OmniWheelModel --------------------

bool OmniWheelModel::needsSpecialManeuver(const State&, const State&) const {
    return false;
}

Trajectory OmniWheelModel::generateManeuverTrajectory(
    const State&, const State&, const std::vector<Obstacle>&) const {
    return Trajectory();
}

// -------------------- 辅助函数 --------------------

ChassisModelType parseChassisModelType(const std::string& str) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    
    if (lower == "ackermann" || lower == "bicycle") {
        return ChassisModelType::ACKERMANN;
    } else if (lower == "differential" || lower == "diff") {
        return ChassisModelType::DIFFERENTIAL;
    } else if (lower == "omniwheel" || lower == "omni") {
        return ChassisModelType::OMNIWHEEL;
    }
    return ChassisModelType::ACKERMANN;
}

std::string chassisModelTypeToString(ChassisModelType type) {
    switch (type) {
        case ChassisModelType::ACKERMANN: return "Ackermann";
        case ChassisModelType::DIFFERENTIAL: return "Differential";
        case ChassisModelType::OMNIWHEEL: return "OmniWheel";
        default: return "Unknown";
    }
}

// ============================================================================
//                          自行车运动学模型
// ============================================================================

BicycleModel::BicycleModel(const VehicleParams& params) : params_(params) {}

State BicycleModel::dynamics(const State& state, const Control& control) const {
    State deriv;
    
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double tan_delta = std::tan(control.steering_angle);
    
    deriv.x = state.v * cos_theta;
    deriv.y = state.v * sin_theta;
    deriv.theta = state.v * tan_delta / params_.wheelbase;
    deriv.v = control.acceleration;
    deriv.kappa = 0.0;
    
    return deriv;
}

State BicycleModel::forward(const State& state, const Control& control, double dt) const {
    State k1 = dynamics(state, control);
    
    State s2;
    s2.x = state.x + dt / 2.0 * k1.x;
    s2.y = state.y + dt / 2.0 * k1.y;
    s2.theta = state.theta + dt / 2.0 * k1.theta;
    s2.v = state.v + dt / 2.0 * k1.v;
    State k2 = dynamics(s2, control);
    
    State s3;
    s3.x = state.x + dt / 2.0 * k2.x;
    s3.y = state.y + dt / 2.0 * k2.y;
    s3.theta = state.theta + dt / 2.0 * k2.theta;
    s3.v = state.v + dt / 2.0 * k2.v;
    State k3 = dynamics(s3, control);
    
    State s4;
    s4.x = state.x + dt * k3.x;
    s4.y = state.y + dt * k3.y;
    s4.theta = state.theta + dt * k3.theta;
    s4.v = state.v + dt * k3.v;
    State k4 = dynamics(s4, control);
    
    State next;
    next.x = state.x + dt / 6.0 * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    next.y = state.y + dt / 6.0 * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y);
    next.theta = state.theta + dt / 6.0 * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta);
    next.v = state.v + dt / 6.0 * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v);
    
    while (next.theta > M_PI) next.theta -= 2.0 * M_PI;
    while (next.theta < -M_PI) next.theta += 2.0 * M_PI;
    
    next.kappa = params_.steeringToKappa(control.steering_angle);
    next.t = state.t + dt;
    
    return next;
}

Eigen::Matrix4d BicycleModel::getStateJacobian(const State& state, const Control& control, double dt) const {
    double cos_theta = std::cos(state.theta);
    double sin_theta = std::sin(state.theta);
    double tan_delta = std::tan(control.steering_angle);
    
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    
    A(0, 2) = -state.v * sin_theta * dt;
    A(0, 3) = cos_theta * dt;
    A(1, 2) = state.v * cos_theta * dt;
    A(1, 3) = sin_theta * dt;
    A(2, 3) = tan_delta / params_.wheelbase * dt;
    
    return A;
}

Eigen::Matrix<double, 4, 2> BicycleModel::getControlJacobian(
    const State& state, const Control& control, double dt) const {
    double cos_delta = std::cos(control.steering_angle);
    double sec2_delta = 1.0 / (cos_delta * cos_delta);
    
    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();
    
    B(2, 1) = state.v * sec2_delta / params_.wheelbase * dt;
    B(3, 0) = dt;
    
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
    if (state.v > params_.max_speed || state.v < params_.min_speed) {
        return false;
    }
    if (std::abs(state.kappa) > params_.max_curvature) {
        return false;
    }
    return true;
}

bool BicycleModel::isControlValid(const Control& control) const {
    if (control.acceleration > params_.max_acceleration || 
        control.acceleration < -params_.max_deceleration) {
        return false;
    }
    if (std::abs(control.steering_angle) > params_.max_steering_angle) {
        return false;
    }
    return true;
}

Control BicycleModel::clipControl(const Control& control) const {
    Control clipped = control;
    clipped.acceleration = std::clamp(control.acceleration, 
                                       -params_.max_deceleration, 
                                       params_.max_acceleration);
    clipped.steering_angle = std::clamp(control.steering_angle,
                                         -params_.max_steering_angle,
                                         params_.max_steering_angle);
    return clipped;
}

} // namespace local_planner
