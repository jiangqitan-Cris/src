#ifndef TRAJECTORY_TRACKER_TRACKER_NODE_HPP
#define TRAJECTORY_TRACKER_TRACKER_NODE_HPP

#include "trajectory_tracker/controller_base.hpp"
#include "trajectory_tracker/types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

namespace trajectory_tracker {

class TrackerNode : public rclcpp::Node {
public:
    explicit TrackerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlTimerCallback();
    bool getRobotStateInMap(RobotState& state) const;
    void publishStop();

    std::unique_ptr<ControllerBase> controller_;
    std::string controller_type_;
    std::string map_frame_;
    std::string robot_frame_;
    double velocity_scale_ = 1.0;
    double goal_tolerance_ = 0.15;

    nav_msgs::msg::Path::SharedPtr last_path_;
    nav_msgs::msg::Path::SharedPtr global_path_;
    RobotState last_state_;
    bool has_path_ = false;
    bool has_odom_ = false;
    bool has_global_goal_ = false;
    bool goal_reached_ = false;
    double global_goal_x_ = 0.0;
    double global_goal_y_ = 0.0;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace trajectory_tracker

#endif  // TRAJECTORY_TRACKER_TRACKER_NODE_HPP
