#include "trajectory_tracker/algorithms/pure_pursuit_controller.hpp"
#include <rclcpp/node.hpp>
#include <cmath>
#include <tf2/utils.h>

namespace trajectory_tracker {

void PurePursuitController::configure(rclcpp::Node* node) {
    params_.wheelbase = node->declare_parameter("vehicle.wheelbase", 0.32);
    params_.max_steering_angle = node->declare_parameter("vehicle.max_steering_angle", 0.5236);
    params_.max_speed = node->declare_parameter("vehicle.max_speed", 1.0);
    lookahead_distance_ = node->declare_parameter("pure_pursuit.lookahead_distance", 2.0);
    lookahead_gain_ = node->declare_parameter("pure_pursuit.lookahead_gain", 0.5);
    use_adaptive_lookahead_ = node->declare_parameter("pure_pursuit.use_adaptive_lookahead", false);
    default_speed_ = node->declare_parameter("pure_pursuit.default_speed", 0.5);
}

bool PurePursuitController::getLookaheadPoint(const nav_msgs::msg::Path& path,
                                              const RobotState& state,
                                              double& ldx, double& ldy, double& ref_v) const {
    if (path.poses.empty()) return false;
    double L = use_adaptive_lookahead_
        ? lookahead_gain_ * state.v + lookahead_distance_
        : lookahead_distance_;
    L = std::max(L, 0.5);
    double x0 = state.x, y0 = state.y;
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
                ref_v = default_speed_;
                return true;
            }
        }
    }
    size_t last = path.poses.size() - 1;
    ldx = path.poses[last].pose.position.x - x0;
    ldy = path.poses[last].pose.position.y - y0;
    ref_v = 0.0;
    return std::hypot(ldx, ldy) > 1e-6;
}

bool PurePursuitController::computeControl(const nav_msgs::msg::Path& path,
                                           const RobotState& state,
                                           ControlCommand& cmd) {
    double ldx, ldy, ref_v;
    if (!getLookaheadPoint(path, state, ldx, ldy, ref_v)) return false;

    double Ld = std::hypot(ldx, ldy);
    if (Ld < 1e-9) return false;

    double alpha = std::atan2(ldy, ldx) - state.theta;
    while (alpha > M_PI) alpha -= 2.0 * M_PI;
    while (alpha < -M_PI) alpha += 2.0 * M_PI;

    double delta = std::atan(2.0 * params_.wheelbase * std::sin(alpha) / Ld);
    delta = std::clamp(delta, -params_.max_steering_angle, params_.max_steering_angle);

    cmd.velocity = ref_v;
    cmd.velocity = std::clamp(cmd.velocity, params_.min_speed, params_.max_speed);
    cmd.steering_angle = delta;
    return true;
}

}  // namespace trajectory_tracker
