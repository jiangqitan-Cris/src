#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "algorithms/base_algorithm.hpp"
#include "planner_component.hpp"
#include "algorithms/astar.hpp"

namespace global_planner {

class GlobalPlannerNode : public rclcpp::Node {

public:
    using ComputePath = nav2_msgs::action::ComputePathToPose;
    using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>;

    /**
     * @brief 构造函数
     */
    GlobalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private: 
    // 1. ROS接口回调
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // 2. Action Server核心处理
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ComputePath::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleComputePath> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleComputePath> goal_handle);

    // 3. 业务执行逻辑
    void execute(const shared_ptr<GoalHandleComputePath> goal_handle);

    // 辅助函数：通过TF获取当前位姿
    bool getCurrentPose(Pose2D& pose);

    // Parameters
    unique_ptr<BaseAlgorithm> planner_;
    unique_ptr<PlannerComponent> component_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;
    rclcpp_action::Server<ComputePath>::SharedPtr action_server_;

    mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;

    shared_ptr<tf2_ros::Buffer> tf_buffer_;
    shared_ptr<tf2_ros::TransformListener> tf_listener_;

    string global_frame_ = "map";
    string robot_frame_ = "base_link";

};

} // namespace global_planner

#endif // PLANNER_NODE_HPP