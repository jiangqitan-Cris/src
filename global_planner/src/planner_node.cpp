#include "planner_node.hpp"
#include <chrono>
#include <tf2/utils.h>

namespace global_planner {

GlobalPlannerNode::GlobalPlannerNode(const rclcpp::NodeOptions& options)
    : LifecycleNode("global_planner_node", options) {
    
    RCLCPP_INFO(get_logger(), "Creating GlobalPlannerNode...");
    
    // 声明参数
    declare_parameter("planner_type", "ASTAR");
    declare_parameter("use_inflation", true);
    declare_parameter("inflation_radius", 0.3);
    declare_parameter("planning_timeout", 5.0);
    declare_parameter("rrt_step_size", 0.2);
    declare_parameter("rrt_max_iterations", 5000);
    declare_parameter("global_frame", "map");
    declare_parameter("robot_frame", "base_link");
    
    // 平滑器参数
    declare_parameter("smoother.corner_radius", 0.5);
    declare_parameter("smoother.sample_step", 0.05);
    declare_parameter("smoother.density_step", 0.1);
    declare_parameter("smoother.control_point_density", 30);
    declare_parameter("smoother.smooth_iterations", 5);
    declare_parameter("smoother.smooth_weight", 0.5);
    declare_parameter("smoother.data_weight", 0.2);
    declare_parameter("smoother.obstacle_weight", 0.3);
    declare_parameter("smoother.obstacle_distance", 0.4);
}

GlobalPlannerNode::~GlobalPlannerNode() {
    // 确保规划线程已结束
    cancel_requested_ = true;
    if (planning_thread_ && planning_thread_->joinable()) {
        planning_thread_->join();
    }
}

GlobalPlannerNode::CallbackReturn 
GlobalPlannerNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring...");
    
    try {
        // 获取参数
        std::string type_str = get_parameter("planner_type").as_string();
        planner_config_.use_inflation = get_parameter("use_inflation").as_bool();
        planner_config_.inflation_radius = get_parameter("inflation_radius").as_double();
        planner_config_.planning_timeout = get_parameter("planning_timeout").as_double();
        planner_config_.rrt_step_size = get_parameter("rrt_step_size").as_double();
        planner_config_.rrt_max_iterations = get_parameter("rrt_max_iterations").as_int();
        global_frame_ = get_parameter("global_frame").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();
        
        // 平滑器参数
        smoother_config_.corner_radius = get_parameter("smoother.corner_radius").as_double();
        smoother_config_.sample_step = get_parameter("smoother.sample_step").as_double();
        smoother_config_.density_step = get_parameter("smoother.density_step").as_double();
        smoother_config_.control_point_density = get_parameter("smoother.control_point_density").as_int();
        smoother_config_.smooth_iterations = get_parameter("smoother.smooth_iterations").as_int();
        smoother_config_.smooth_weight = get_parameter("smoother.smooth_weight").as_double();
        smoother_config_.data_weight = get_parameter("smoother.data_weight").as_double();
        smoother_config_.obstacle_weight = get_parameter("smoother.obstacle_weight").as_double();
        smoother_config_.obstacle_distance = get_parameter("smoother.obstacle_distance").as_double();

        // 创建规划器
        component_ = std::make_unique<PlannerComponent>();
        planner_ = component_->create(component_->stringToType(type_str));
        if (!planner_) {
            RCLCPP_ERROR(get_logger(), "Failed to create planner: %s", type_str.c_str());
            return CallbackReturn::FAILURE;
        }
        planner_->initialize(type_str);
        planner_->configure(planner_config_);
        
        // 创建平滑器
        smoother_ = std::make_unique<IntegratedSmoother>();
        smoother_->configure(smoother_config_);

        // 初始化 TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建发布者（lifecycle 版本）
        path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", 10);
        raw_path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_raw_path", 10);
        inflated_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "inflated_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
        diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        // 订阅地图
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", map_qos, 
            std::bind(&GlobalPlannerNode::mapCallback, this, std::placeholders::_1));

        // 创建 Action Server
        action_server_ = rclcpp_action::create_server<ComputePath>(
            this,
            "compute_path_to_pose",
            std::bind(&GlobalPlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GlobalPlannerNode::handleCancel, this, std::placeholders::_1),
            std::bind(&GlobalPlannerNode::handleAccepted, this, std::placeholders::_1));

        // 诊断定时器
        diag_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GlobalPlannerNode::diagnosticsTimerCallback, this));

        RCLCPP_INFO(get_logger(), "Configuration complete. Planner: %s", planner_->getName().c_str());
        return CallbackReturn::SUCCESS;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Configuration failed: %s", e.what());
        return CallbackReturn::FAILURE;
    }
}

GlobalPlannerNode::CallbackReturn 
GlobalPlannerNode::on_activate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Activating...");
    
    path_pub_->on_activate();
    raw_path_pub_->on_activate();
    inflated_map_pub_->on_activate();
    diag_pub_->on_activate();
    
    RCLCPP_INFO(get_logger(), "Activated. Ready for planning requests.");
    return CallbackReturn::SUCCESS;
}

GlobalPlannerNode::CallbackReturn 
GlobalPlannerNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    
    // 取消正在进行的规划
    cancel_requested_ = true;
    if (planning_thread_ && planning_thread_->joinable()) {
        planning_thread_->join();
        planning_thread_.reset();
    }
    
    path_pub_->on_deactivate();
    raw_path_pub_->on_deactivate();
    inflated_map_pub_->on_deactivate();
    diag_pub_->on_deactivate();
    
    RCLCPP_INFO(get_logger(), "Deactivated.");
    return CallbackReturn::SUCCESS;
}

GlobalPlannerNode::CallbackReturn 
GlobalPlannerNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    
    planner_.reset();
    component_.reset();
    smoother_.reset();
    tf_buffer_.reset();
    tf_listener_.reset();
    
    map_sub_.reset();
    path_pub_.reset();
    raw_path_pub_.reset();
    inflated_map_pub_.reset();
    diag_pub_.reset();
    action_server_.reset();
    diag_timer_.reset();
    
    RCLCPP_INFO(get_logger(), "Cleanup complete.");
    return CallbackReturn::SUCCESS;
}

GlobalPlannerNode::CallbackReturn 
GlobalPlannerNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    return CallbackReturn::SUCCESS;
}

void GlobalPlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);

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

    current_inflated_map_ = *msg;
    current_inflated_map_.data = planner_->getProcessedMap();
    current_inflated_map_.header.stamp = now();
    
    if (inflated_map_pub_->is_activated()) {
        inflated_map_pub_->publish(current_inflated_map_);
    }
    
    map_received_ = true;
    RCLCPP_DEBUG(get_logger(), "Map updated: %dx%d", msg->info.width, msg->info.height);
}

bool GlobalPlannerNode::getCurrentPose(Pose2D& pose) {
    try {
        auto t = tf_buffer_->lookupTransform(global_frame_, robot_frame_, tf2::TimePointZero);
        pose.x = t.transform.translation.x;
        pose.y = t.transform.translation.y;
        
        // 从四元数获取 yaw 角
        tf2::Quaternion q(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w);
        pose.theta = tf2::getYaw(q);
        
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF error: %s", ex.what());
        return false;
    }
}

bool GlobalPlannerNode::checkCollision(double x, double y) const {
    if (current_inflated_map_.data.empty()) {
        return true;
    }

    double origin_x = current_inflated_map_.info.origin.position.x;
    double origin_y = current_inflated_map_.info.origin.position.y;
    double resolution = current_inflated_map_.info.resolution;
    int width = static_cast<int>(current_inflated_map_.info.width);
    int height = static_cast<int>(current_inflated_map_.info.height);

    int grid_x = static_cast<int>((x - origin_x) / resolution);
    int grid_y = static_cast<int>((y - origin_y) / resolution);

    if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height) {
        return true;
    }

    int index = grid_y * width + grid_x;
    return current_inflated_map_.data[index] > 50 || current_inflated_map_.data[index] == -1;
}

double GlobalPlannerNode::calculatePathLength(const std::vector<Pose2D>& path) const {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
    }
    return length;
}

rclcpp_action::GoalResponse GlobalPlannerNode::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ComputePath::Goal> /*goal*/) {
    
    RCLCPP_INFO(get_logger(), "Received path planning request");
    
    // 检查节点是否处于激活状态 (PRIMARY_STATE_ACTIVE = 3)
    constexpr uint8_t ACTIVE_STATE = 3;
    if (get_current_state().id() != ACTIVE_STATE) {
        RCLCPP_WARN(get_logger(), "Node not active, rejecting goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    // 如果已经在规划，拒绝新请求（或者可以选择排队）
    if (is_planning_) {
        RCLCPP_WARN(get_logger(), "Already planning, rejecting new goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlannerNode::handleCancel(
    const std::shared_ptr<GoalHandleComputePath> /*goal_handle*/) {
    
    RCLCPP_INFO(get_logger(), "Cancel request received");
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlannerNode::handleAccepted(const std::shared_ptr<GoalHandleComputePath> goal_handle) {
    // 等待上一个规划线程结束
    if (planning_thread_ && planning_thread_->joinable()) {
        planning_thread_->join();
    }
    
    cancel_requested_ = false;
    planning_thread_ = std::make_shared<std::thread>(
        std::bind(&GlobalPlannerNode::executePlanning, this, goal_handle));
}

void GlobalPlannerNode::executePlanning(const std::shared_ptr<GoalHandleComputePath> goal_handle) {
    std::lock_guard<std::mutex> planning_lock(planning_mutex_);
    is_planning_ = true;
    
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ComputePath::Result>();
    auto feedback = std::make_shared<ComputePath::Feedback>();
    
    auto start_time = std::chrono::steady_clock::now();
    stats_.total_plans++;
    stats_.last_plan_time = start_time;
    
    PlanningResult planning_result;
    planning_result.status = PlannerStatus::UNKNOWN_ERROR;
    
    // 获取起点
    Pose2D start_pose, goal_pose;
    if (!getCurrentPose(start_pose)) {
        planning_result.status = PlannerStatus::TF_ERROR;
        planning_result.message = "Failed to get current pose from TF";
        RCLCPP_ERROR(get_logger(), "%s", planning_result.message.c_str());
        goal_handle->abort(result);
        stats_.failed_plans++;
        is_planning_ = false;
        return;
    }
    
    goal_pose.x = goal->goal.pose.position.x;
    goal_pose.y = goal->goal.pose.position.y;
    
    // 检查取消请求
    if (cancel_requested_ || goal_handle->is_canceling()) {
        result->path.header.frame_id = global_frame_;
        goal_handle->canceled(result);
        is_planning_ = false;
        return;
    }
    
    // 执行规划
    std::vector<Pose2D> path_points;
    bool success = false;
    
    {
        std::lock_guard<std::mutex> map_lock(map_mutex_);
        if (!map_received_) {
            planning_result.status = PlannerStatus::NO_MAP;
            planning_result.message = "No map received";
            RCLCPP_ERROR(get_logger(), "%s", planning_result.message.c_str());
            goal_handle->abort(result);
            stats_.failed_plans++;
            is_planning_ = false;
            return;
        }
        success = planner_->makePlan(start_pose, goal_pose, path_points);
    }
    
    if (!success || path_points.empty()) {
        planning_result.status = PlannerStatus::NO_PATH_FOUND;
        planning_result.message = "Planner failed to find a path";
        RCLCPP_ERROR(get_logger(), "%s", planning_result.message.c_str());
        goal_handle->abort(result);
        stats_.failed_plans++;
        is_planning_ = false;
        return;
    }
    
    // 再次检查取消
    if (cancel_requested_ || goal_handle->is_canceling()) {
        result->path.header.frame_id = global_frame_;
        goal_handle->canceled(result);
        is_planning_ = false;
        return;
    }
    
    // 路径平滑
    auto is_occupied = [this](double x, double y) {
        return checkCollision(x, y);
    };
    
    auto final_path = smoother_->segmentedSmooth(path_points, is_occupied, smoother_config_.sample_step);
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    
    // 构建结果
    nav_msgs::msg::Path ros_path;
    ros_path.header.frame_id = global_frame_;
    ros_path.header.stamp = now();
    
    for (const auto& p : final_path) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = ros_path.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, p.theta);
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        
        ros_path.poses.push_back(ps);
    }
    
    result->path = ros_path;
    
    // 发布路径
    if (path_pub_->is_activated()) {
        path_pub_->publish(ros_path);
    }
    
    // 发布原始路径（用于调试）
    if (raw_path_pub_->is_activated()) {
        nav_msgs::msg::Path raw_ros_path;
        raw_ros_path.header = ros_path.header;
        for (const auto& p : path_points) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = ros_path.header;
            ps.pose.position.x = p.x;
            ps.pose.position.y = p.y;
            raw_ros_path.poses.push_back(ps);
        }
        raw_path_pub_->publish(raw_ros_path);
    }
    
    // 更新统计
    stats_.successful_plans++;
    stats_.last_planning_time_ms = static_cast<double>(duration_ms);
    stats_.last_path_length = calculatePathLength(final_path);
    
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), 
        "Planning succeeded: %zu points, %.1f m, %ld ms",
        final_path.size(), stats_.last_path_length.load(), duration_ms);
    
    is_planning_ = false;
}

void GlobalPlannerNode::diagnosticsTimerCallback() {
    publishDiagnostics();
}

void GlobalPlannerNode::publishDiagnostics() {
    if (!diag_pub_->is_activated()) {
        return;
    }
    
    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = now();
    
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "GlobalPlanner";
    status.hardware_id = "global_planner_node";
    
    // 确定状态级别 (PRIMARY_STATE_ACTIVE = 3)
    constexpr uint8_t ACTIVE_STATE = 3;
    if (get_current_state().id() == ACTIVE_STATE) {
        if (map_received_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Planner ready";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Waiting for map";
        }
    } else {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "Node not active";
    }
    
    // 添加键值对
    auto add_kv = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(kv);
    };
    
    add_kv("planner_type", planner_ ? planner_->getName() : "none");
    add_kv("total_plans", std::to_string(stats_.total_plans.load()));
    add_kv("successful_plans", std::to_string(stats_.successful_plans.load()));
    add_kv("failed_plans", std::to_string(stats_.failed_plans.load()));
    add_kv("last_planning_time_ms", std::to_string(stats_.last_planning_time_ms.load()));
    add_kv("last_path_length_m", std::to_string(stats_.last_path_length.load()));
    add_kv("is_planning", is_planning_ ? "true" : "false");
    add_kv("map_received", map_received_ ? "true" : "false");
    
    diag_array.status.push_back(status);
    diag_pub_->publish(diag_array);
}

} // namespace global_planner

// Main 函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<global_planner::GlobalPlannerNode>();
    
    // 使用多线程执行器以支持 lifecycle 和 action
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
