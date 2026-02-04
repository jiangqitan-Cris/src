#ifndef TRAJECTORY_TRACKER_MPC_CONTROLLER_HPP
#define TRAJECTORY_TRACKER_MPC_CONTROLLER_HPP

#include "trajectory_tracker/controller_base.hpp"

namespace trajectory_tracker {

/**
 * @brief MPC 轨迹跟踪控制器（占位实现）
 *
 * 当前为占位：与 Pure Pursuit 类似逻辑，保证接口可用；
 * 后续可替换为真实 MPC 优化（如 ACADO、OsQP、CasADi 等）。
 */
class MPCController : public ControllerBase {
public:
    MPCController() = default;
    bool computeControl(const nav_msgs::msg::Path& path,
                        const RobotState& state,
                        ControlCommand& cmd) override;
    void configure(rclcpp::Node* node) override;
    std::string name() const override { return "mpc"; }

private:
    VehicleParams params_;
    double lookahead_distance_ = 2.0;
    double default_speed_ = 0.5;
};

} // namespace trajectory_tracker

#endif // TRAJECTORY_TRACKER_MPC_CONTROLLER_HPP
