#include "trajectory_tracker/algorithms/lqr_controller.hpp"
#include <rclcpp/node.hpp>
#include <cmath>
#include <tf2/utils.h>

namespace trajectory_tracker {

namespace {
constexpr double DT_LQR = 0.01;
constexpr int DARE_ITERATIONS = 200;
}  // namespace

void LQRController::configure(rclcpp::Node* node) {
    params_.wheelbase = node->declare_parameter("vehicle.wheelbase", 0.32);
    params_.max_steering_angle = node->declare_parameter("vehicle.max_steering_angle", 0.5236);
    params_.max_speed = node->declare_parameter("vehicle.max_speed", 1.0);
    params_.min_speed = node->declare_parameter("vehicle.min_speed", -1.0);  // 允许倒车
    lookahead_distance_ = node->declare_parameter("lqr.lookahead_distance", 2.0);
    q_lateral_ = node->declare_parameter("lqr.q_lateral", 1.0);
    q_heading_ = node->declare_parameter("lqr.q_heading", 1.0);
    r_steering_ = node->declare_parameter("lqr.r_steering", 0.1);
    
    RCLCPP_INFO(node->get_logger(), "LQR Controller configured: min_speed=%.2f, max_speed=%.2f", 
                params_.min_speed, params_.max_speed);
}

void LQRController::updateGain(double v_ref) {
    if (v_ref < 0.05) {
        v_ref = 0.05;
    }
    double L = params_.wheelbase;
    Eigen::Matrix2d A;
    A << 0, v_ref, 0, 0;
    Eigen::Vector2d B(0.0, v_ref / L);

    Eigen::Matrix2d Ad = Eigen::Matrix2d::Identity() + A * DT_LQR;
    Eigen::Vector2d Bd = B * DT_LQR;
    Eigen::Matrix2d Q = Eigen::Vector2d(q_lateral_, q_heading_).asDiagonal();
    double R = r_steering_;

    Eigen::Matrix2d P = Q;
    for (int i = 0; i < DARE_ITERATIONS; ++i) {
        Eigen::Matrix2d Pn = Q + Ad.transpose() * P * Ad -
            Ad.transpose() * P * Bd * (1.0 / (R + (Bd.transpose() * P * Bd)(0, 0))) * Bd.transpose() * P * Ad;
        if ((Pn - P).cwiseAbs().maxCoeff() < 1e-9) break;
        P = Pn;
    }
    K_ = (1.0 / (R + (Bd.transpose() * P * Bd)(0, 0))) * Bd.transpose() * P * Ad;
    gain_computed_ = true;
}

bool LQRController::getReferencePoint(const nav_msgs::msg::Path& path,
                                       const RobotState& state,
                                       ReferencePoint& ref) const {
    if (path.poses.size() < 2) return false;
    double best_s = 0.0;
    double best_d = 1e9;
    size_t best_i = 0;
    double s = 0.0;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
        double x0 = path.poses[i].pose.position.x;
        double y0 = path.poses[i].pose.position.y;
        double x1 = path.poses[i + 1].pose.position.x;
        double y1 = path.poses[i + 1].pose.position.y;
        double dx = x1 - x0, dy = y1 - y0;
        double seg = std::hypot(dx, dy);
        if (seg < 1e-9) continue;
        double t = std::clamp(((state.x - x0) * dx + (state.y - y0) * dy) / (seg * seg), 0.0, 1.0);
        double px = x0 + t * dx, py = y0 + t * dy;
        double d = std::hypot(state.x - px, state.y - py);
        if (d < best_d) {
            best_d = d;
            best_s = s + t * seg;
            best_i = i;
        }
        s += seg;
    }
    double total_len = 0.0;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i)
        total_len += std::hypot(
            path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x,
            path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y);
    s = std::min(best_s + lookahead_distance_, total_len);
    s = std::max(s, 0.0);
    double acc = 0.0;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
        double x0 = path.poses[i].pose.position.x;
        double y0 = path.poses[i].pose.position.y;
        double x1 = path.poses[i + 1].pose.position.x;
        double y1 = path.poses[i + 1].pose.position.y;
        double seg = std::hypot(x1 - x0, y1 - y0);
        if (acc + seg >= s || i == path.poses.size() - 2) {
            double alpha = (seg > 1e-9) ? std::clamp((s - acc) / seg, 0.0, 1.0) : 0.0;
            ref.x = x0 + alpha * (x1 - x0);
            ref.y = y0 + alpha * (y1 - y0);
            ref.theta = tf2::getYaw(path.poses[i].pose.orientation) +
                alpha * (tf2::getYaw(path.poses[i + 1].pose.orientation) - tf2::getYaw(path.poses[i].pose.orientation));
            ref.kappa = 0.0;
            ref.v = params_.max_speed * 0.6;
            return true;
        }
        acc += seg;
    }
    ref.x = path.poses.back().pose.position.x;
    ref.y = path.poses.back().pose.position.y;
    ref.theta = tf2::getYaw(path.poses.back().pose.orientation);
    ref.kappa = 0.0;
    ref.v = 0.0;
    return true;
}

bool LQRController::computeControl(const nav_msgs::msg::Path& path,
                                   const RobotState& state,
                                   ControlCommand& cmd) {
    ReferencePoint ref;
    if (!getReferencePoint(path, state, ref)) return false;

    // 检查是否已到达路径终点附近
    if (path.poses.size() >= 1) {
        double goal_x = path.poses.back().pose.position.x;
        double goal_y = path.poses.back().pose.position.y;
        double dist_to_goal = std::hypot(state.x - goal_x, state.y - goal_y);
        
        // 如果距离终点小于0.15m，停止
        if (dist_to_goal < 0.15) {
            cmd.velocity = 0.0;
            cmd.steering_angle = 0.0;
            return true;
        }
        
        // 如果接近终点（小于0.5m），逐渐减速
        if (dist_to_goal < 0.5) {
            ref.v = std::min(ref.v, dist_to_goal * 0.5);
        }
    }

    // 检测是否需要倒车
    // 简化逻辑：直接基于轨迹的瞬时方向判断，信任规划器
    // 规划器已经有了滞后逻辑，控制器不需要额外的复杂滞后
    bool need_reverse = false;
    
    if (path.poses.size() >= 3) {
        // 计算轨迹开始几个点的累积前进投影
        double cumulative_projection = 0.0;
        int valid_segments = 0;
        
        for (size_t i = 1; i < std::min(size_t(8), path.poses.size()); ++i) {
            double dx = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
            double dy = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
            double seg_len = std::hypot(dx, dy);
            
            if (seg_len > 0.01) {
                // 计算这一段相对于机器人朝向的投影
                double proj = dx * std::cos(state.theta) + dy * std::sin(state.theta);
                cumulative_projection += proj;
                valid_segments++;
            }
        }
        
        // 如果累积投影明显为负，则是倒车轨迹
        if (valid_segments > 0 && cumulative_projection < -0.05) {
            need_reverse = true;
        }
    }
    
    // 简单的状态变化日志
    static bool last_reverse = false;
    if (need_reverse != last_reverse) {
        RCLCPP_INFO(rclcpp::get_logger("lqr_controller"), 
                    "Mode: %s", need_reverse ? "REVERSE" : "FORWARD");
        last_reverse = need_reverse;
    }

    double e_y = (state.x - ref.x) * (-std::sin(ref.theta)) + (state.y - ref.y) * std::cos(ref.theta);
    double e_theta = state.theta - ref.theta;
    while (e_theta > M_PI) e_theta -= 2.0 * M_PI;
    while (e_theta < -M_PI) e_theta += 2.0 * M_PI;

    // 只有在非停止状态时才强制最小速度
    double v_ref = (ref.v > 0.01) ? std::max(ref.v, 0.1) : 0.0;
    
    // 倒车时使用负速度，但不修改转向逻辑
    // 对于阿克曼模型，倒车时前轮转角方向不变，只是速度为负
    if (need_reverse) {
        v_ref = -std::abs(v_ref) * 0.5;  // 倒车时速度减半
    }
    
    if (std::abs(v_ref) > 0.01) {
        updateGain(std::abs(v_ref));  // LQR 增益计算使用速度绝对值
    }

    double delta_ref = std::atan(params_.wheelbase * ref.kappa);
    delta_ref = std::clamp(delta_ref, -params_.max_steering_angle, params_.max_steering_angle);
    Eigen::Vector2d e(e_y, e_theta);
    double delta = (std::abs(v_ref) > 0.01) ? (delta_ref - K_.dot(e)) : 0.0;
    
    // 倒车时需要反转转向角！
    // 原因：LQR 是基于前进时的线性化模型设计的
    // 自行车模型：theta_dot = v * tan(delta) / L
    // 前进(v>0) + 正转向角(delta>0) → theta增加 → 向左转
    // 倒车(v<0) + 正转向角(delta>0) → theta减少 → 向右转
    // 所以倒车时，为了让 LQR 的控制效果正确，需要反转转向角
    if (need_reverse) {
        delta = -delta;
    }
    
    delta = std::clamp(delta, -params_.max_steering_angle, params_.max_steering_angle);

    cmd.velocity = v_ref;
    // 允许负速度（倒车），使用配置的 min_speed 参数
    cmd.velocity = std::clamp(cmd.velocity, params_.min_speed, params_.max_speed);
    cmd.steering_angle = delta;
    return true;
}

}  // namespace trajectory_tracker
