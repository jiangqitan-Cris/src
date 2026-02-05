#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class AckermannOdomNode : public rclcpp::Node {
public:
  AckermannOdomNode()
  : Node("ackermann_odom_node"),
    wheelbase_(declare_parameter<double>("wheelbase", 0.32)),
    x_(0.0), y_(0.0), yaw_(0.0), v_(0.0), delta_(0.0)
  {
    using std::placeholders::_1;
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&AckermannOdomNode::cmdCb, this, _1));
    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/steering_angle", 10, std::bind(&AckermannOdomNode::steerCb, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&AckermannOdomNode::update, this));
  }

private:
  void cmdCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    v_ = msg->linear.x;
  }

  void steerCb(const std_msgs::msg::Float64::SharedPtr msg) {
    delta_ = msg->data;
  }

  void update() {
    rclcpp::Time t = now();
    double dt = (t - last_time_).seconds();
    if (dt <= 0.0) return;
    last_time_ = t;

    double x_dot = v_ * std::cos(yaw_);
    double y_dot = v_ * std::sin(yaw_);
    double yaw_dot = v_ * std::tan(delta_) / wheelbase_;

    x_   += x_dot * dt;
    y_   += y_dot * dt;
    yaw_ += yaw_dot * dt;

    while (yaw_ > M_PI)  yaw_ -= 2.0 * M_PI;
    while (yaw_ < -M_PI) yaw_ += 2.0 * M_PI;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    double cy = std::cos(yaw_ * 0.5);
    double sy = std::sin(yaw_ * 0.5);
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = sy;
    odom.pose.pose.orientation.w = cy;

    odom.twist.twist.linear.x = v_;
    odom.twist.twist.angular.z = yaw_dot;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  double wheelbase_;
  double x_, y_, yaw_;
  double v_, delta_;
  rclcpp::Time last_time_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannOdomNode>());
  rclcpp::shutdown();
  return 0;
}

