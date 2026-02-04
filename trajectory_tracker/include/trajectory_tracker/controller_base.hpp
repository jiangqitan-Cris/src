#ifndef TRAJECTORY_TRACKER_CONTROLLER_BASE_HPP
#define TRAJECTORY_TRACKER_CONTROLLER_BASE_HPP

/**
 * @file controller_base.hpp
 * @brief 轨迹跟踪控制器基类接口，支持 LQR / MPC / Pure Pursuit 等插拔实现
 */

#include "trajectory_tracker/types.hpp"
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/node.hpp>
#include <string>

namespace trajectory_tracker {

/**
 * @brief 轨迹跟踪控制器抽象基类
 *
 * 输入：参考路径（Path）、当前机器人状态（RobotState）
 * 输出：阿克曼控制量（线速度 + 前轮转角）
 */
class ControllerBase {
public:
    virtual ~ControllerBase() = default;

    /**
     * @brief 根据参考路径和当前状态计算控制量
     * @param path 参考路径（通常为 local_path，map 系）
     * @param state 当前机器人状态（map 系）
     * @param cmd 输出控制指令
     * @return 是否成功计算出有效控制（false 表示无路径或不可用，应停车）
     */
    virtual bool computeControl(
        const nav_msgs::msg::Path& path,
        const RobotState& state,
        ControlCommand& cmd) = 0;

    /**
     * @brief 从 ROS 参数加载配置（算法相关参数在子类中声明，需非 const Node 以调用 declare_parameter）
     */
    virtual void configure(rclcpp::Node* node) = 0;

    /** 控制器类型名称，用于日志和参数命名 */
    virtual std::string name() const = 0;
};

} // namespace trajectory_tracker

#endif // TRAJECTORY_TRACKER_CONTROLLER_BASE_HPP
