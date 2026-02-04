#ifndef TRAJECTORY_TRACKER_PURE_PURSUIT_CONTROLLER_HPP
#define TRAJECTORY_TRACKER_PURE_PURSUIT_CONTROLLER_HPP

#include "trajectory_tracker/controller_base.hpp"

namespace trajectory_tracker {

/**
 * @brief Pure Pursuit 轨迹跟踪控制器
 *
 * 前视距离 + 曲率计算前轮转角，速度按参考或恒定。
 */
class PurePursuitController : public ControllerBase {
public:
    PurePursuitController() = default;
    bool computeControl(const nav_msgs::msg::Path& path,
                        const RobotState& state,
                        ControlCommand& cmd) override;
    void configure(rclcpp::Node* node) override;
    std::string name() const override { return "pure_pursuit"; }

private:
    VehicleParams params_;
    double lookahead_distance_ = 2.0;
    double lookahead_gain_ = 0.5;   // 可选：速度相关前视 k * v + L0
    bool use_adaptive_lookahead_ = false;
    double default_speed_ = 0.5;

    bool getLookaheadPoint(const nav_msgs::msg::Path& path,
                           const RobotState& state,
                           double& ldx, double& ldy, double& ref_v) const;
};

} // namespace trajectory_tracker

#endif // TRAJECTORY_TRACKER_PURE_PURSUIT_CONTROLLER_HPP
