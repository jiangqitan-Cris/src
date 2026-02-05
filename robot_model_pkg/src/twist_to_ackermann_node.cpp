/**
 * @file twist_to_ackermann_node.cpp
 * @brief 将 Twist 消息转换为 Ackermann 控制命令
 * 
 * 订阅: /cmd_vel_raw (geometry_msgs/Twist)
 * 发布: /cmd_vel (geometry_msgs/Twist) - 只用 linear.x
 *       /steering_angle (std_msgs/Float64) - 转向角
 * 
 * 转换公式:
 *   steering_angle = atan(wheelbase * angular.z / linear.x)
 *   当 linear.x 接近 0 时，使用原地转向近似
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <algorithm>

class TwistToAckermannNode : public rclcpp::Node {
public:
    TwistToAckermannNode()
        : Node("twist_to_ackermann_node") {
        
        // 参数
        wheelbase_ = declare_parameter<double>("wheelbase", 0.32);
        max_steering_angle_ = declare_parameter<double>("max_steering_angle", 0.5236);  // 30度
        min_speed_for_steering_ = declare_parameter<double>("min_speed_for_steering", 0.05);
        
        // 订阅原始 Twist
        twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10,
            std::bind(&TwistToAckermannNode::twistCallback, this, std::placeholders::_1));
        
        // 发布速度和转向角
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);
        
        RCLCPP_INFO(get_logger(), "Twist to Ackermann converter started");
        RCLCPP_INFO(get_logger(), "  Wheelbase: %.3f m", wheelbase_);
        RCLCPP_INFO(get_logger(), "  Max steering angle: %.3f rad (%.1f deg)", 
                    max_steering_angle_, max_steering_angle_ * 180.0 / M_PI);
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 提取线速度和角速度
        double v = msg->linear.x;
        double omega = msg->angular.z;
        
        // 计算转向角
        double steering_angle = 0.0;
        
        if (std::abs(v) > min_speed_for_steering_) {
            // 正常行驶: steering = atan(L * omega / v)
            steering_angle = std::atan(wheelbase_ * omega / v);
        } else if (std::abs(omega) > 0.01) {
            // 低速或原地转向: 使用最大转向角
            steering_angle = (omega > 0) ? max_steering_angle_ : -max_steering_angle_;
            // 低速时给一点前进速度以便转向
            if (std::abs(v) < 0.01) {
                v = 0.1 * ((omega > 0) ? 1.0 : 1.0);  // 小速度前进
            }
        }
        
        // 限制转向角范围
        steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
        
        // 发布速度命令
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = v;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;  // Ackermann 不使用角速度
        cmd_vel_pub_->publish(cmd_vel);
        
        // 发布转向角
        std_msgs::msg::Float64 steering_msg;
        steering_msg.data = steering_angle;
        steering_pub_->publish(steering_msg);
        
        RCLCPP_DEBUG(get_logger(), "v=%.2f, omega=%.2f -> steering=%.3f rad", 
                     v, omega, steering_angle);
    }

    double wheelbase_;
    double max_steering_angle_;
    double min_speed_for_steering_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToAckermannNode>());
    rclcpp::shutdown();
    return 0;
}
