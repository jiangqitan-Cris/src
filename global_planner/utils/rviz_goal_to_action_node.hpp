#ifndef RVIZ_GOAL_TO_ACTION_NODE_HPP
#define RVIZ_GOAL_TO_ACTION_NODE_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// 引入 RViz 发送的 PoseStamped 消息类型
#include "geometry_msgs/msg/pose_stamped.hpp" 
// 引入 Action 类型
#include "nav2_msgs/action/compute_path_to_pose.hpp"

// 使用别名简化代码
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePathToPose = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

class RvizGoalToActionNode : public rclcpp::Node
{
public:
  // 构造函数
  explicit RvizGoalToActionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Action Client 对象
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;

  // 订阅 RViz 目标位姿的订阅者
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_sub_;

  // 订阅 RViz 目标位姿的回调函数
  void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // 以下是 Action Client 的异步回调函数
  void goalResponseCallback(GoalHandleComputePathToPose::SharedPtr future);
  void feedbackCallback(
    GoalHandleComputePathToPose::SharedPtr future,
    const std::shared_ptr<const ComputePathToPose::Feedback> feedback);
  void resultCallback(const GoalHandleComputePathToPose::WrappedResult & result);
};

#endif // RVIZ_GOAL_TO_ACTION_NODE_HPP