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
    lookahead_distance_ = node->declare_parameter("lqr.lookahead_distance", 2.0);
    q_lateral_ = node->declare_parameter("lqr.q_lateral", 1.0);
    q_heading_ = node->declare_parameter("lqr.q_heading", 1.0);
    r_steering_ = node->declare_parameter("lqr.r_steering", 0.1);
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

    double e_y = (state.x - ref.x) * (-std::sin(ref.theta)) + (state.y - ref.y) * std::cos(ref.theta);
    double e_theta = state.theta - ref.theta;
    while (e_theta > M_PI) e_theta -= 2.0 * M_PI;
    while (e_theta < -M_PI) e_theta += 2.0 * M_PI;

    double v_ref = std::max(ref.v, 0.1);
    updateGain(v_ref);

    double delta_ref = std::atan(params_.wheelbase * ref.kappa);
    delta_ref = std::clamp(delta_ref, -params_.max_steering_angle, params_.max_steering_angle);
    Eigen::Vector2d e(e_y, e_theta);
    double delta = delta_ref - K_.dot(e);
    delta = std::clamp(delta, -params_.max_steering_angle, params_.max_steering_angle);

    cmd.velocity = v_ref;
    cmd.velocity = std::clamp(cmd.velocity, params_.min_speed, params_.max_speed);
    cmd.steering_angle = delta;
    return true;
}

}  // namespace trajectory_tracker
