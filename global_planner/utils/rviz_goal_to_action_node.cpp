#include "rviz_goal_to_action_node.hpp"

#include <functional> // for std::bind

RvizGoalToActionNode::RvizGoalToActionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("rviz_goal_to_action_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing RvizGoalToActionNode...");

  // 1. 初始化 Action Client
  // "compute_path_to_pose" 应该和你的 Action Server 监听的名字一致
  action_client_ = rclcpp_action::create_client<ComputePathToPose>(
    this,
    "compute_path_to_pose"); 

  // 2. 初始化订阅者，订阅 RViz 发布的 '/goal_pose' 话题
  // RViz 默认发布到 /goal_pose，你可以根据实际情况调整话题名
  rviz_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, 
    std::bind(&RvizGoalToActionNode::rvizGoalCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Finish initializing, waiting for RViz goal...");
}

// RViz 目标位姿的回调函数
void RvizGoalToActionNode::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 检查 Action Server 是否可用
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Action Server unavailable, cannot send goal");
    return;
  }

  // 构造 Action Goal
  auto goal_msg = ComputePathToPose::Goal();
  goal_msg.goal = *msg; // 直接将 RViz 传来的 PoseStamped 赋值给 Goal 中的 PoseStamped

  RCLCPP_INFO(this->get_logger(), 
              "Receive RViz goal: [x: %.2f, y: %.2f, z: %.2f] frame_id: %s",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->header.frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "Sending Action Goal...");

  // 配置 Action Client 的异步回调
  auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = 
    std::bind(&RvizGoalToActionNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&RvizGoalToActionNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = 
    std::bind(&RvizGoalToActionNode::resultCallback, this, std::placeholders::_1);

  // 发送 Goal
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

// Action Server 响应 Goal 请求的回调 (是否接受)
void RvizGoalToActionNode::goalResponseCallback(GoalHandleComputePathToPose::SharedPtr future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Action Server reject");
  } else {
    RCLCPP_INFO(this->get_logger(), "Action Server accept");
  }
}

// Action Server 持续发送反馈的回调
void RvizGoalToActionNode::feedbackCallback(
  GoalHandleComputePathToPose::SharedPtr,
  const std::shared_ptr<const ComputePathToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Receiving the feedback...");
  // 你可以在这里处理具体的反馈信息，比如打印剩余距离等
}

// Action Server 最终结果的回调 (成功、失败或取消)
void RvizGoalToActionNode::resultCallback(const GoalHandleComputePathToPose::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action success: finish global planning!");
      // 你可以在这里访问 result.result->path 等信息
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Action abort: (ABORTED)!");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Action cancel: (CANCELED)!");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown Action!");
      break;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // 使用 SingleThreadedExecutor 运行节点
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<RvizGoalToActionNode>();
  executor.add_node(node);
  executor.spin(); // 开始循环处理回调
  rclcpp::shutdown();
  return 0;
}