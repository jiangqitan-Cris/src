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

    controller_ = createController(controller_type_);
    if (!controller_) {
        RCLCPP_ERROR(get_logger(), "Unknown controller_type: '%s'. Use lqr, pure_pursuit, or mpc.", controller_type_.c_str());
        throw std::runtime_error("trajectory_tracker: invalid controller_type");
    }
    controller_->configure(this);
    RCLCPP_INFO(get_logger(), "Trajectory tracker using controller: %s", controller_->name().c_str());

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/local_path", 10, std::bind(&TrackerNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TrackerNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    steering_pub_ = create_publisher<std_msgs::msg::Float64>("/steering_angle", 10);

    double control_freq = declare_parameter("control_frequency", 50.0);
    control_timer_ = create_wall_timer(
        std::chrono::microseconds(static_cast<int64_t>(1e6 / control_freq)),
        std::bind(&TrackerNode::controlTimerCallback, this));
}

void TrackerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    last_path_ = msg;
    has_path_ = (msg && msg->poses.size() > 0);
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
    if (!has_path_ || !last_path_) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_pub_->publish(twist);
        std_msgs::msg::Float64 steering;
        steering.data = 0.0;
        steering_pub_->publish(steering);
        return;
    }

    RobotState state;
    if (!getRobotStateInMap(state)) {
        return;
    }

    nav_msgs::msg::Path path_to_use = *last_path_;
    if (path_to_use.poses.size() >= 2) {
        double dx_last = path_to_use.poses.back().pose.position.x - state.x;
        double dy_last = path_to_use.poses.back().pose.position.y - state.y;
        double ahead = dx_last * std::cos(state.theta) + dy_last * std::sin(state.theta);
        if (ahead < 0.0) {
            std::reverse(path_to_use.poses.begin(), path_to_use.poses.end());
        }
    }

    ControlCommand cmd;
    if (!controller_->computeControl(path_to_use, state, cmd)) {
        cmd.velocity = 0.0;
        cmd.steering_angle = 0.0;
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
