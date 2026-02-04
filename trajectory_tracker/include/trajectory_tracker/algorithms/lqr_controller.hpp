#ifndef TRAJECTORY_TRACKER_LQR_CONTROLLER_HPP
#define TRAJECTORY_TRACKER_LQR_CONTROLLER_HPP

#include "trajectory_tracker/controller_base.hpp"
#include <Eigen/Dense>

namespace trajectory_tracker {

/**
 * @brief LQR 轨迹跟踪控制器
 *
 * 在参考点处线性化自行车模型，状态 [e_y, e_theta]，控制 delta，
 * 求解 LQR 得到反馈增益，输出 steering = delta_ref - K*e；速度跟踪 v_ref。
 */
class LQRController : public ControllerBase {
public:
    LQRController() = default;
    bool computeControl(const nav_msgs::msg::Path& path,
                        const RobotState& state,
                        ControlCommand& cmd) override;
    void configure(rclcpp::Node* node) override;
    std::string name() const override { return "lqr"; }

private:
    VehicleParams params_;
    double lookahead_distance_ = 2.0;
    double q_lateral_ = 1.0;   // 横向误差权重
    double q_heading_ = 1.0;   // 航向误差权重
    double r_steering_ = 0.1;  // 转向权重

    Eigen::Vector2d K_;        // LQR 增益 [e_y, e_theta] -> delta
    bool gain_computed_ = false;

    void updateGain(double v_ref);
    bool getReferencePoint(const nav_msgs::msg::Path& path,
                           const RobotState& state,
                           ReferencePoint& ref) const;
};

} // namespace trajectory_tracker

#endif // TRAJECTORY_TRACKER_LQR_CONTROLLER_HPP
