#include "trajectory_tracker/algorithms/mpc_controller.hpp"
#include <rclcpp/node.hpp>
#include <cmath>
#include <tf2/utils.h>

namespace trajectory_tracker {

/**
 * 占位实现：当前使用与 Pure Pursuit 类似的前视 + 曲率逻辑，
 * 后续可替换为真实 MPC 优化（ACADO / OsQP / CasADi 等）。
 */
void MPCController::configure(rclcpp::Node* node) {
    params_.wheelbase = node->declare_parameter("vehicle.wheelbase", 0.32);
    params_.max_steering_angle = node->declare_parameter("vehicle.max_steering_angle", 0.5236);
    params_.max_speed = node->declare_parameter("vehicle.max_speed", 1.0);
    lookahead_distance_ = node->declare_parameter("mpc.lookahead_distance", 2.0);
    default_speed_ = node->declare_parameter("mpc.default_speed", 0.5);
}

bool MPCController::computeControl(const nav_msgs::msg::Path& path,
                                   const RobotState& state,
                                   ControlCommand& cmd) {
    if (path.poses.size() < 2) return false;
    double L = lookahead_distance_;
    double x0 = state.x, y0 = state.y;
    double ldx = 0.0, ldy = 0.0;
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
        double px = path.poses[i].pose.position.x;
        double py = path.poses[i].pose.position.y;
        double nx = path.poses[i + 1].pose.position.x;
        double ny = path.poses[i + 1].pose.position.y;
        double seg = std::hypot(nx - px, ny - py);
        if (seg < 1e-9) continue;
        for (double t = 0; t <= 1.0; t += 0.1) {
            double x = px + t * (nx - px);
            double y = py + t * (ny - py);
            double d = std::hypot(x - x0, y - y0);
            if (d >= L) {
                ldx = x - x0;
                ldy = y - y0;
                double Ld = std::hypot(ldx, ldy);
                if (Ld < 1e-9) break;
                double alpha = std::atan2(ldy, ldx) - state.theta;
                while (alpha > M_PI) alpha -= 2.0 * M_PI;
                while (alpha < -M_PI) alpha += 2.0 * M_PI;
                double delta = std::atan(2.0 * params_.wheelbase * std::sin(alpha) / Ld);
                delta = std::clamp(delta, -params_.max_steering_angle, params_.max_steering_angle);
                cmd.velocity = std::clamp(default_speed_, params_.min_speed, params_.max_speed);
                cmd.steering_angle = delta;
                return true;
            }
        }
    }
    size_t last = path.poses.size() - 1;
    ldx = path.poses[last].pose.position.x - x0;
    ldy = path.poses[last].pose.position.y - y0;
    double Ld = std::hypot(ldx, ldy);
    if (Ld < 1e-9) {
        cmd.velocity = 0.0;
        cmd.steering_angle = 0.0;
        return true;
    }
    double alpha = std::atan2(ldy, ldx) - state.theta;
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;
    double delta = std::atan(2.0 * params_.wheelbase * std::sin(alpha) / Ld);
    delta = std::clamp(delta, -params_.max_steering_angle, params_.max_steering_angle);
    cmd.velocity = 0.0;
    cmd.steering_angle = delta;
    return true;
}

}  // namespace trajectory_tracker
