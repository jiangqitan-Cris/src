#include "planner_node.hpp"
#include <chrono>

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode(const rclcpp::NodeOptions & options)
                    : Node("global_planner_node", options) {
    RCLCPP_INFO(this->get_logger(), "Initializing Global Planner Node...");

    // 1. Initialize components, parameters and algorithm
    this->declare_parameter("planner_type", "ASTAR");
    this->declare_parameter("use_inflation", true);
    this->declare_parameter("inflation_radius", 0.3);
    this->declare_parameter("global_frame", "map");
    this->declare_parameter("robot_frame", "base_link");

    string type_str = this->get_parameter("planner_type").as_string();
    PlannerConfig config;
    config.use_inflation = this->get_parameter("use_inflation").as_bool();
    config.inflation_radius = this->get_parameter("inflation_radius").as_double();
    global_frame_ = this->get_parameter("global_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();

    component_ = std::make_unique<PlannerComponent>();
    planner_ = component_->create(component_->stringToType(type_str));
    if (!planner_) {
        RCLCPP_FATAL(this->get_logger(), "Failed to create planner: %s", type_str.c_str());
        return;
    }
    planner_->initialize(type_str);
    planner_->configure(config);

    // 2. Initialize TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 3. Subscribe map and initialize publisher
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", map_qos, std::bind(&GlobalPlannerNode::mapCallback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);
    inflated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                            "inflated_map", 
                            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    // 4. launch Action Server
    action_server_ = rclcpp_action::create_server<ComputePath>(
        this,
        "compute_path_to_pose",
        std::bind(&GlobalPlannerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&GlobalPlannerNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&GlobalPlannerNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Planner Node Ready. Current Algorithm: %s", planner_->getName().c_str());
}

void GlobalPlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    lock_guard<mutex> lock(map_mutex_);
    current_map_ = msg;

    GridMapData neutral_map;
    neutral_map.data = msg->data;
    neutral_map.width = msg->info.width;
    neutral_map.height = msg->info.height;
    neutral_map.resolution = msg->info.resolution;
    neutral_map.origin_x = msg->info.origin.position.x;
    neutral_map.origin_y = msg->info.origin.position.y;

    if (planner_) {
        planner_->updateMap(neutral_map);
    }

    // get and publish inflated map
    auto inflated_grid = *msg;
    inflated_grid.data = planner_->getProcessedMap();
    inflated_grid.header.stamp = this->now();
    inflated_map_pub_->publish(inflated_grid);

    RCLCPP_DEBUG(this->get_logger(), "Map updated.");
}

bool GlobalPlannerNode::getCurrentPose(Pose2D& pose) {
    try {
        auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        pose.x = t.transform.translation.x;
        pose.y = t.transform.translation.y;
        pose.theta = 0.0; // 简化处理，2D 规划通常不需要初始角度
        return true;
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get robot pose: %s", ex.what());
        return false;
    }
}

rclcpp_action::GoalResponse GlobalPlannerNode::handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePath::Goal>) {
    RCLCPP_INFO(this->get_logger(), "Received new path request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlannerNode::handle_cancel(const std::shared_ptr<GoalHandleComputePath>) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlannerNode::handle_accepted(const std::shared_ptr<GoalHandleComputePath> goal_handle) {
    // 必须要开线程，否则会卡死 ROS 执行器
    std::thread{std::bind(&GlobalPlannerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GlobalPlannerNode::execute(const std::shared_ptr<GoalHandleComputePath> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePath::Result>();
    nav_msgs::msg::Path global_path;
    
    // 1. 获取起点 (从 TF 获取) 和 终点 (从 Goal 获取)
    Pose2D start_pose, goal_pose;
    if (!getCurrentPose(start_pose)) {
        RCLCPP_ERROR(this->get_logger(), "Planning failed: Start pose unavailable");
        goal_handle->abort(result);
        return;
    }
    
    goal_pose.x = goal->goal.pose.position.x;
    goal_pose.y = goal->goal.pose.position.y;

    // 2. 规划执行 (带耗时统计)
    auto start_time = std::chrono::steady_clock::now();
    std::vector<Pose2D> path_points;
    bool success = false;

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!current_map_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for map...");
        } else {
            success = planner_->makePlan(start_pose, goal_pose, path_points);
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // 3. 结果处理
    if (success) {
        for (const auto& p : path_points) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.frame_id = global_frame_;
            ps.header.stamp = this->now();
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            result->path.poses.push_back(ps);
            global_path.poses.push_back(ps);
        }
        result->path.header.frame_id = global_frame_;
        result->path.header.stamp = this->now();
        global_path.header.frame_id = global_frame_;
        global_path.header.stamp = this->now();
        
        goal_handle->succeed(result);
        path_pub_->publish(global_path);
        RCLCPP_INFO(this->get_logger(), "Plan found in %ld ms with %zu points", duration, path_points.size());
    } else {
        goal_handle->abort(result);
        RCLCPP_ERROR(this->get_logger(), "Planner failed to find a path!");
    }
}

} // namespace global_planner

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<global_planner::GlobalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}