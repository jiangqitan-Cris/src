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
#include "smoother/smoother.hpp"

namespace global_planner {

class GlobalPlannerNode : public rclcpp::Node {

public:
    using ComputePath = nav2_msgs::action::ComputePathToPose; // 计算去某个位姿的路径，这是Nav2预定义的action接口
    using GoalHandleComputePath = rclcpp_action::ServerGoalHandle<ComputePath>; // action服务端
    /*
    通过这个 GoalHandle，服务端可以做到：

    publish_feedback()：给客户端发进度。

    succeed()：告诉客户端任务成功了。

    abort()：任务失败，中断。

    is_canceling()：检查客户端是不是想取消这个任务。
    */

    /**
     * @brief 构造函数
     */
    GlobalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private: 
    // 1. ROS接口回调
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // 2. Action Server核心处理
    rclcpp_action::GoalResponse handle_goal( // 审查action请求
        const rclcpp_action::GoalUUID & uuid, // action请求的ID，由ROS生成，自己不用管
        std::shared_ptr<const ComputePath::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel( // 请求取消
        const std::shared_ptr<GoalHandleComputePath> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleComputePath> goal_handle); // 正式开工

    // 3. 业务执行逻辑
    void execute(const shared_ptr<GoalHandleComputePath> goal_handle);

    // 辅助函数：通过TF获取当前位姿
    bool getCurrentPose(Pose2D& pose);
    bool checkCollision(double x, double y);

    // Parameters
    unique_ptr<BaseAlgorithm> planner_;
    unique_ptr<PlannerComponent> component_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smooth_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_map_pub_;
    rclcpp_action::Server<ComputePath>::SharedPtr action_server_;

    mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid current_inflated_map_;

    shared_ptr<tf2_ros::Buffer> tf_buffer_;
    shared_ptr<tf2_ros::TransformListener> tf_listener_;

    string global_frame_ = "map";
    string robot_frame_ = "base_link";

};

} // namespace global_planner

#endif // PLANNER_NODE_HPP