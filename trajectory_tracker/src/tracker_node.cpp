#include "trajectory_tracker/tracker_node.hpp"
#include "trajectory_tracker/algorithms/lqr_controller.hpp"
#include "trajectory_tracker/algorithms/pure_pursuit_controller.hpp"
#include "trajectory_tracker/algorithms/mpc_controller.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace trajectory_tracker {

static std::unique_ptr<ControllerBase> createController(const std::string& type) {
    if (type == "lqr") return std::make_unique<LQRController>();
    if (type == "pure_pursuit") return std::make_unique<PurePursuitController>();
    if (type == "mpc") return std::make_unique<MPCController>();
    return nullptr;
}

TrackerNode::TrackerNode(const rclcpp::NodeOptions& options)
    : Node("tracker_node", options) {
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    robot_frame_ = declare_parameter("robot_frame", std::string("base_footprint"));
    controller_type_ = declare_parameter("controller_type", std::string("lqr"));
    velocity_scale_ = declare_parameter("velocity_scale", 1.0);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.1);  // 到达终点的容差，适当放宽

    controller_ = createController(controller_type_);
    if (!controller_) {
        RCLCPP_ERROR(get_logger(), "Unknown controller_type: '%s'. Use lqr, pure_pursuit, or mpc.", controller_type_.c_str());
        throw std::runtime_error("trajectory_tracker: invalid controller_type");
    }
    controller_->configure(this);
    RCLCPP_INFO(get_logger(), "Trajectory tracker using controller: %s", controller_->name().c_str());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅局部路径
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/local_path", 10, std::bind(&TrackerNode::pathCallback, this, std::placeholders::_1));
    
    // 订阅全局路径以获取真正的终点
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/global_path", 10, std::bind(&TrackerNode::globalPathCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TrackerNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);
    goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

    double control_freq = declare_parameter("control_frequency", 50.0);
    control_timer_ = create_wall_timer(
        std::chrono::microseconds(static_cast<int64_t>(1e6 / control_freq)),
        std::bind(&TrackerNode::controlTimerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Trajectory tracker initialized with goal_tolerance: %.2f m", goal_tolerance_);
}

void TrackerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    last_path_ = msg;
    has_path_ = (msg && msg->poses.size() > 0);
}

void TrackerNode::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg && !msg->poses.empty()) {
        global_path_ = msg;
        // 提取全局路径的终点作为真正的目标
        global_goal_x_ = msg->poses.back().pose.position.x;
        global_goal_y_ = msg->poses.back().pose.position.y;
        has_global_goal_ = true;
        goal_reached_ = false;  // 收到新路径时重置目标到达状态
        RCLCPP_INFO_ONCE(get_logger(), "Received global goal: (%.2f, %.2f)", 
                         global_goal_x_, global_goal_y_);
    }
}

void TrackerNode::publishStop() {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);
    
    std_msgs::msg::Float64 steering;
    steering.data = 0.0;
    steering_pub_->publish(steering);
}

void TrackerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    last_state_.v = std::hypot(vx, vy);
    last_state_.x = msg->pose.pose.position.x;
    last_state_.y = msg->pose.pose.position.y;
    last_state_.theta = tf2::getYaw(msg->pose.pose.orientation);
    has_odom_ = true;
}

bool TrackerNode::getRobotStateInMap(RobotState& state) const {
    if (!has_odom_) return false;
    try {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
            map_frame_, robot_frame_, tf2::TimePointZero);
        state.x = tf.transform.translation.x;
        state.y = tf.transform.translation.y;
        state.theta = tf2::getYaw(tf.transform.rotation);
        state.v = last_state_.v;
        return true;
    } catch (const tf2::TransformException&) {
        state = last_state_;
        return true;
    }
}

void TrackerNode::controlTimerCallback() {
    // 如果已经到达目标，持续发布停止指令
    if (goal_reached_) {
        publishStop();
        // 持续发布目标到达状态
        std_msgs::msg::Bool reached_msg;
        reached_msg.data = true;
        goal_reached_pub_->publish(reached_msg);
        return;
    }

    if (!has_path_ || !last_path_) {
        publishStop();
        return;
    }

    RobotState state;
    if (!getRobotStateInMap(state)) {
        return;
    }

    // 检查是否到达全局终点（优先使用全局路径终点）
    if (has_global_goal_) {
        double dist_to_global_goal = std::hypot(state.x - global_goal_x_, state.y - global_goal_y_);
        if (dist_to_global_goal <= goal_tolerance_) {
            goal_reached_ = true;
            publishStop();
            RCLCPP_INFO(get_logger(), "Goal reached! Distance to goal: %.3f m", dist_to_global_goal);
            
            // 发布目标到达状态
            std_msgs::msg::Bool reached_msg;
            reached_msg.data = true;
            goal_reached_pub_->publish(reached_msg);
            return;
        }
    }

    nav_msgs::msg::Path path_to_use = *last_path_;

    // 如果局部路径为空，停止
    if (path_to_use.poses.empty()) {
        publishStop();
        return;
    }

    // 备用检查：如果没有全局目标，检查局部路径终点
    if (!has_global_goal_) {
        double goal_x = path_to_use.poses.back().pose.position.x;
        double goal_y = path_to_use.poses.back().pose.position.y;
        double dist_to_goal = std::hypot(state.x - goal_x, state.y - goal_y);
        if (dist_to_goal <= goal_tolerance_) {
            publishStop();
            return;
        }
    }

    ControlCommand cmd;
    if (!controller_->computeControl(path_to_use, state, cmd)) {
        cmd.velocity = 0.0;
        cmd.steering_angle = 0.0;
    }

    // 当接近全局终点时，逐渐减速（仅对前进有效，倒车时不减速）
    if (has_global_goal_ && cmd.velocity > 0) {
        double dist_to_global_goal = std::hypot(state.x - global_goal_x_, state.y - global_goal_y_);
        double slow_down_distance = 1.0;  // 在距终点 1m 内开始减速
        if (dist_to_global_goal < slow_down_distance) {
            double speed_factor = std::max(0.2, dist_to_global_goal / slow_down_distance);
            cmd.velocity *= speed_factor;
            RCLCPP_DEBUG(get_logger(), "Slowing down near goal: dist=%.2f, factor=%.2f", 
                        dist_to_global_goal, speed_factor);
        }
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x = velocity_scale_ * cmd.velocity;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub_->publish(twist);

    std_msgs::msg::Float64 steering;
    steering.data = cmd.steering_angle;
    steering_pub_->publish(steering);
}

}  // namespace trajectory_tracker

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<trajectory_tracker::TrackerNode>());
    rclcpp::shutdown();
    return 0;
}
