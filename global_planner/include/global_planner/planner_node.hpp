#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "algorithms/base_algorithm.hpp"
#include "planner_component.hpp"
#include "algorithms/astar.hpp"
#include "smoother/smoother.hpp"

namespace global_planner {

/**
 * @brief 全局路径规划节点
 * 
 * 使用 Lifecycle Node 管理节点生命周期：
 * - unconfigured -> inactive: on_configure()
 * - inactive -> active: on_activate()
 * - active -> inactive: on_deactivate()
 * - inactive -> finalized: on_cleanup()
 */
class GlobalPlannerNode : public rclcpp_lifecycle::LifecycleNode {

public:
    using ComputePath = nav2_msgs::action::ComputePathToPose;
    using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * @brief 构造函数
     */
    explicit GlobalPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    /**
     * @brief 析构函数
     */
    ~GlobalPlannerNode() override;

protected:
    // ========== Lifecycle 回调 ==========
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private: 
    // ========== ROS 回调 ==========
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // ========== Action Server 处理 ==========
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ComputePath::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleComputePath> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleComputePath> goal_handle);

    // ========== 业务逻辑 ==========
    void executePlanning(const std::shared_ptr<GoalHandleComputePath> goal_handle);
    bool getCurrentPose(Pose2D& pose);
    bool checkCollision(double x, double y) const;
    double calculatePathLength(const std::vector<Pose2D>& path) const;

    // ========== 诊断和监控 ==========
    void publishDiagnostics();
    void diagnosticsTimerCallback();

    // ========== 成员变量 ==========
    // 规划器
    std::unique_ptr<BaseAlgorithm> planner_;
    std::unique_ptr<PlannerComponent> component_;
    std::unique_ptr<IntegratedSmoother> smoother_;

    // ROS 接口
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;
    rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp_action::Server<ComputePath>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 线程安全
    mutable std::mutex map_mutex_;
    mutable std::mutex planning_mutex_;
    std::atomic<bool> is_planning_{false};
    std::atomic<bool> cancel_requested_{false};
    std::shared_ptr<std::thread> planning_thread_;

    // 地图数据
    nav_msgs::msg::OccupancyGrid current_inflated_map_;
    bool map_received_ = false;

    // 配置参数
    std::string global_frame_ = "map";
    std::string robot_frame_ = "base_link";
    PlannerConfig planner_config_;
    SmootherConfig smoother_config_;

    // 统计信息
    struct Statistics {
        std::atomic<uint64_t> total_plans{0};
        std::atomic<uint64_t> successful_plans{0};
        std::atomic<uint64_t> failed_plans{0};
        std::atomic<double> last_planning_time_ms{0.0};
        std::atomic<double> last_path_length{0.0};
        std::chrono::steady_clock::time_point last_plan_time;
    } stats_;
};

} // namespace global_planner

#endif // PLANNER_NODE_HPP
