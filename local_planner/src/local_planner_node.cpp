#include "local_planner/local_planner_node.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace local_planner {

LocalPlannerNode::LocalPlannerNode() 
    : Node("local_planner_node"),
      has_map_(false), has_path_(false), has_odom_(false), visualize_lattice_(true),
      goal_reached_(false) {
    
    // 声明参数
    declare_parameter("global_frame", "map");
    declare_parameter("robot_frame", "base_link");
    declare_parameter("chassis_model_type", "ackermann");  // 底盘模型类型
    declare_parameter("planning_frequency", 10.0);
    declare_parameter("local_range", 5.0);
    declare_parameter("lookahead_distance", 3.0);
    declare_parameter("obstacle_radius", 0.15);
    declare_parameter("obstacle_threshold", 50);
    declare_parameter("obstacle_cluster_dist", 0.3);
    declare_parameter("visualize_lattice", true);
    declare_parameter("planner_type", "ilqr");   // "lattice" 或 "ilqr"
    declare_parameter("goal_tolerance", 0.3);
    
    // Lattice 配置参数
    declare_parameter("lattice.num_width_samples", 7);
    declare_parameter("lattice.num_time_samples", 5);
    declare_parameter("lattice.max_lateral_offset", 1.0);
    declare_parameter("lattice.min_planning_time", 2.0);
    declare_parameter("lattice.max_planning_time", 4.0);
    declare_parameter("lattice.time_resolution", 0.1);
    declare_parameter("lattice.safe_distance", 0.2);          // 障碍物膨胀半径
    declare_parameter("lattice.weight_lateral_offset", 5.0);  // 横向偏移代价权重
    declare_parameter("lattice.weight_deviation", 10.0);      // 偏离参考路径惩罚
    
    // 车辆参数
    declare_parameter("vehicle.wheelbase", 0.5);
    declare_parameter("vehicle.width", 0.4);
    declare_parameter("vehicle.max_speed", 1.0);
    declare_parameter("vehicle.max_acceleration", 1.0);
    declare_parameter("vehicle.max_steering_angle", 0.5);
    declare_parameter("vehicle.max_curvature", 1.0);
    
    // iLQR 配置参数
    declare_parameter("ilqr.max_iterations", 50);
    declare_parameter("ilqr.tolerance", 1e-4);
    declare_parameter("ilqr.initial_lambda", 1.0);
    declare_parameter("ilqr.horizon_time", 3.0);
    declare_parameter("ilqr.dt", 0.1);
    declare_parameter("ilqr.weight_x", 1.0);
    declare_parameter("ilqr.weight_y", 1.0);
    declare_parameter("ilqr.weight_theta", 0.5);
    declare_parameter("ilqr.weight_v", 0.1);
    declare_parameter("ilqr.weight_acceleration", 0.1);
    declare_parameter("ilqr.weight_steering", 0.1);
    declare_parameter("ilqr.weight_terminal", 10.0);
    declare_parameter("ilqr.weight_obstacle", 50.0);
    declare_parameter("ilqr.safe_distance", 0.5);
    
    // 获取参数
    global_frame_ = get_parameter("global_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    
    // 获取底盘模型类型
    std::string chassis_type_str = get_parameter("chassis_model_type").as_string();
    std::transform(chassis_type_str.begin(), chassis_type_str.end(), chassis_type_str.begin(), ::tolower);
    ChassisModelType chassis_type = ChassisModelType::ACKERMANN;  // 默认
    if (chassis_type_str == "differential") {
        chassis_type = ChassisModelType::DIFFERENTIAL;
    } else if (chassis_type_str == "omniwheel") {
        chassis_type = ChassisModelType::OMNIWHEEL;
    } else {
        chassis_type = ChassisModelType::ACKERMANN;
    }
    
    planning_frequency_ = get_parameter("planning_frequency").as_double();
    local_range_ = get_parameter("local_range").as_double();
    lookahead_distance_ = get_parameter("lookahead_distance").as_double();
    obstacle_radius_ = get_parameter("obstacle_radius").as_double();
    obstacle_threshold_ = get_parameter("obstacle_threshold").as_int();
    obstacle_cluster_dist_ = get_parameter("obstacle_cluster_dist").as_double();
    visualize_lattice_ = get_parameter("visualize_lattice").as_bool();
    planner_type_ = get_parameter("planner_type").as_string();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    
    // 归一化 planner_type
    std::string pt = planner_type_;
    std::transform(pt.begin(), pt.end(), pt.begin(), ::tolower);
    if (pt == "lattice") {
        planner_type_ = "lattice";
    } else {
        planner_type_ = "ilqr";
    }
    
    // Lattice 配置
    lattice_config_.num_width_samples = get_parameter("lattice.num_width_samples").as_int();
    lattice_config_.num_time_samples = get_parameter("lattice.num_time_samples").as_int();
    lattice_config_.max_lateral_offset = get_parameter("lattice.max_lateral_offset").as_double();
    lattice_config_.min_planning_time = get_parameter("lattice.min_planning_time").as_double();
    lattice_config_.max_planning_time = get_parameter("lattice.max_planning_time").as_double();
    lattice_config_.time_resolution = get_parameter("lattice.time_resolution").as_double();
    lattice_config_.safe_distance = get_parameter("lattice.safe_distance").as_double();
    lattice_config_.weight_lateral_offset = get_parameter("lattice.weight_lateral_offset").as_double();
    lattice_config_.weight_deviation = get_parameter("lattice.weight_deviation").as_double();
    
    // 车辆参数
    vehicle_params_.wheelbase = get_parameter("vehicle.wheelbase").as_double();
    vehicle_params_.width = get_parameter("vehicle.width").as_double();
    vehicle_params_.max_speed = get_parameter("vehicle.max_speed").as_double();
    vehicle_params_.max_acceleration = get_parameter("vehicle.max_acceleration").as_double();
    vehicle_params_.max_steering_angle = get_parameter("vehicle.max_steering_angle").as_double();
    vehicle_params_.max_curvature = get_parameter("vehicle.max_curvature").as_double();
    
    // iLQR 配置
    ilqr_config_.max_iterations = get_parameter("ilqr.max_iterations").as_int();
    ilqr_config_.tolerance = get_parameter("ilqr.tolerance").as_double();
    ilqr_config_.initial_lambda = get_parameter("ilqr.initial_lambda").as_double();
    ilqr_config_.horizon_time = get_parameter("ilqr.horizon_time").as_double();
    ilqr_config_.dt = get_parameter("ilqr.dt").as_double();
    ilqr_config_.weight_x = get_parameter("ilqr.weight_x").as_double();
    ilqr_config_.weight_y = get_parameter("ilqr.weight_y").as_double();
    ilqr_config_.weight_theta = get_parameter("ilqr.weight_theta").as_double();
    ilqr_config_.weight_v = get_parameter("ilqr.weight_v").as_double();
    ilqr_config_.weight_acceleration = get_parameter("ilqr.weight_acceleration").as_double();
    ilqr_config_.weight_steering = get_parameter("ilqr.weight_steering").as_double();
    ilqr_config_.weight_terminal = get_parameter("ilqr.weight_terminal").as_double();
    ilqr_config_.weight_obstacle = get_parameter("ilqr.weight_obstacle").as_double();
    ilqr_config_.safe_distance = get_parameter("ilqr.safe_distance").as_double();
    
    // 创建规划器
    if (planner_type_ == "lattice") {
        lattice_planner_ = std::make_unique<LatticePlanner>(lattice_config_, vehicle_params_, chassis_type);
        RCLCPP_INFO(get_logger(), "Using Lattice Planner with %s chassis", chassis_type_str.c_str());
    } else {
        ilqr_planner_ = std::make_unique<ILQR>(ilqr_config_, vehicle_params_);
        RCLCPP_INFO(get_logger(), "Using iLQR Planner");
    }
    
    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 订阅者
    // map 话题使用 Transient Local QoS (与 nav2_map_server 匹配)
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos,
        std::bind(&LocalPlannerNode::mapCallback, this, std::placeholders::_1));
    
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/global_path", 10,
        std::bind(&LocalPlannerNode::globalPathCallback, this, std::placeholders::_1));
    
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&LocalPlannerNode::odomCallback, this, std::placeholders::_1));
    
    // 发布者
    local_path_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/local_planner_viz", 10);
    lattice_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lattice_candidates", 10);
    
    // 定时器
    auto period = std::chrono::duration<double>(1.0 / planning_frequency_);
    planning_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&LocalPlannerNode::planningTimerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Local planner node initialized");
    RCLCPP_INFO(get_logger(), "  Planning frequency: %.1f Hz", planning_frequency_);
    RCLCPP_INFO(get_logger(), "  Local range: %.1f m", local_range_);
    RCLCPP_INFO(get_logger(), "  Lookahead distance: %.1f m", lookahead_distance_);
    RCLCPP_INFO(get_logger(), "  Goal tolerance: %.1f m", goal_tolerance_);
}

void LocalPlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    current_map_ = msg;
    has_map_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "Received map: %dx%d, resolution: %.3f",
                     msg->info.width, msg->info.height, msg->info.resolution);
}

void LocalPlannerNode::globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    global_path_ = msg;
    has_path_ = true;
    
    // 保存全局终点
    if (msg && !msg->poses.empty()) {
        global_goal_ = msg->poses.back();
        goal_reached_ = false;  // 收到新路径时重置目标到达状态
        RCLCPP_INFO(get_logger(), "Received global path with %zu points, goal: (%.2f, %.2f)", 
                    msg->poses.size(),
                    global_goal_.pose.position.x,
                    global_goal_.pose.position.y);
    }
}

void LocalPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = msg;
    has_odom_ = true;
}

void LocalPlannerNode::planningTimerCallback() {
    // 检查是否有必要的数据
    if (!has_map_ || !has_path_ || !has_odom_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Waiting for data: map=%s, path=%s, odom=%s",
            has_map_ ? "OK" : "NO",
            has_path_ ? "OK" : "NO", 
            has_odom_ ? "OK" : "NO");
        return;
    }
    
    // 获取当前状态
    State current_state;
    if (!getCurrentState(current_state)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                             "Failed to get current state");
        return;
    }
    
    // 检查是否已经到达全局终点
    double dist_to_global_goal = std::hypot(
        current_state.x - global_goal_.pose.position.x,
        current_state.y - global_goal_.pose.position.y);
    
    if (dist_to_global_goal <= goal_tolerance_) {
        if (!goal_reached_) {
            goal_reached_ = true;
            RCLCPP_INFO(get_logger(), "Goal reached! Distance: %.3f m (tolerance: %.3f m)",
                        dist_to_global_goal, goal_tolerance_);
        }
        // 到达目标后，发布一个只包含当前位置的路径（让控制器停止）
        nav_msgs::msg::Path stop_path;
        stop_path.header.stamp = now();
        stop_path.header.frame_id = global_frame_;
        geometry_msgs::msg::PoseStamped current_pose;
        current_pose.header = stop_path.header;
        current_pose.pose.position.x = current_state.x;
        current_pose.pose.position.y = current_state.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, current_state.theta);
        current_pose.pose.orientation = tf2::toMsg(q);
        stop_path.poses.push_back(current_pose);
        local_path_pub_->publish(stop_path);
        return;
    }
    
    // 重置目标到达状态（如果之前到达过但现在不在范围内）
    goal_reached_ = false;
    
    // 提取局部障碍物
    auto obstacles = extractLocalObstacles(current_state.x, current_state.y, local_range_);
    
    // 调试：定期输出障碍物数量
    // static int obs_debug_counter = 0;
    // if (++obs_debug_counter % 50 == 0) {
    //     RCLCPP_INFO(get_logger(), "Detected %zu obstacles in local range %.1fm", 
    //                 obstacles.size(), local_range_);
    // }
    
    // 获取局部参考路径
    auto reference_path = getLocalReferencePath(current_state, lookahead_distance_);
    if (reference_path.size() < 2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Reference path too short");
        return;
    }
    
    // 当接近终点时，确保局部路径终点是全局终点
    // 这样可以让轨迹规划器正确地规划到终点
    if (dist_to_global_goal < lookahead_distance_) {
        State global_goal_state;
        global_goal_state.x = global_goal_.pose.position.x;
        global_goal_state.y = global_goal_.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(global_goal_.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        global_goal_state.theta = yaw;
        global_goal_state.v = 0.0;  // 终点速度为0
        global_goal_state.a = 0.0;
        
        // 确保参考路径以全局终点结束
        if (!reference_path.empty()) {
            double last_dist = std::hypot(
                reference_path.back().x - global_goal_state.x,
                reference_path.back().y - global_goal_state.y);
            if (last_dist > 0.1) {
                reference_path.push_back(global_goal_state);
            } else {
                reference_path.back() = global_goal_state;
            }
        }
    }
    
    State goal_state = reference_path.back();
    
    // 判断是否接近终点（需要停止模式）
    // 计算减速停止所需的距离：v²/(2*a) + buffer
    double stopping_distance = (current_state.v * current_state.v) / 
                               (2.0 * vehicle_params_.max_deceleration) + 1.0;
    // 只有当距离小于 减速距离+2米 时才进入停止模式，而不是用 lookahead_distance（太大了）
    double approach_threshold = std::max(stopping_distance + 2.0, 3.0);  // 至少 3 米
    bool approaching_goal = (dist_to_global_goal < approach_threshold);
    
    // 当非常接近终点时，直接生成收敛到终点的简单轨迹
    // 这样可以避免 Frenet 坐标转换的误差，并确保精确停在终点
    double very_close_threshold = std::max(goal_tolerance_ * 5.0, 1.5);  // 至少 1.5m
    bool very_close_to_goal = (dist_to_global_goal < very_close_threshold);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    Trajectory trajectory;
    
    if (very_close_to_goal) {
        // 非常接近终点时，直接生成简单的减速停止轨迹
        trajectory = generateFinalApproachTrajectory(current_state, global_goal_, dist_to_global_goal);
        RCLCPP_DEBUG(get_logger(), "Using final approach trajectory, dist: %.3f m", dist_to_global_goal);
    } else if (planner_type_ == "lattice") {
        // Lattice Planner：传递停止模式参数
        trajectory = lattice_planner_->plan(current_state, reference_path, obstacles, 
                                            approaching_goal, -1.0);
    } else {
        // iLQR: 用参考路径作为初始轨迹猜测
        Trajectory initial_traj = referencePathToInitialTrajectory(
            reference_path, current_state, goal_state);
        
        // Use the time-sampled initial trajectory states as the reference for the cost function
        // This ensures the cost function targets correspond to the solver's time steps
        std::vector<State> time_sampled_ref;
        if (initial_traj.is_valid) {
            for (const auto& pt : initial_traj.points) {
                time_sampled_ref.push_back(pt.state);
            }
        } else {
             // Fallback if initial traj generation failed
            time_sampled_ref = reference_path; 
        }

        ilqr_planner_->setReferencePath(time_sampled_ref);
        trajectory = ilqr_planner_->optimize(
            current_state, goal_state, initial_traj, obstacles);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double planning_time_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    
    if (trajectory.is_valid) {
        RCLCPP_DEBUG(get_logger(), "Planning succeeded in %.2f ms, %zu points, dist_to_goal: %.2f m",
                     planning_time_ms, trajectory.points.size(), dist_to_global_goal);
        
        publishTrajectory(trajectory);
        publishVisualization(trajectory, obstacles);
        
        // 仅 Lattice 时发布候选轨迹可视化
        if (planner_type_ == "lattice" && visualize_lattice_) {
            publishLatticeVisualization();
        }
    } else {
        // 打印更详细的规划失败原因
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Planning failed: traj_points=%zu, obstacles=%zu, ref_path=%zu",
                             trajectory.points.size(), obstacles.size(), reference_path.size());
        
        if (planner_type_ == "lattice" && visualize_lattice_) {
            publishLatticeVisualization();
        }
    }
}

std::vector<Obstacle> LocalPlannerNode::extractLocalObstacles(
    double robot_x, double robot_y, double local_range) {
    
    std::vector<Obstacle> obstacles;
    
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!current_map_) return obstacles;
    
    const auto& map = *current_map_;
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;
    int width = map.info.width;
    int height = map.info.height;
    
    // 计算局部范围在栅格地图中的索引
    int range_cells = static_cast<int>(local_range / resolution);
    int robot_i = static_cast<int>((robot_x - origin_x) / resolution);
    int robot_j = static_cast<int>((robot_y - origin_y) / resolution);
    
    int min_i = std::max(0, robot_i - range_cells);
    int max_i = std::min(width - 1, robot_i + range_cells);
    int min_j = std::max(0, robot_j - range_cells);
    int max_j = std::min(height - 1, robot_j + range_cells);
    
    // 收集障碍物点
    std::vector<std::pair<double, double>> obstacle_points;
    
    for (int j = min_j; j <= max_j; ++j) {
        for (int i = min_i; i <= max_i; ++i) {
            int idx = j * width + i;
            int8_t value = map.data[idx];
            
            // 检查是否为障碍物
            if (value >= obstacle_threshold_) {
                double wx = origin_x + (i + 0.5) * resolution;
                double wy = origin_y + (j + 0.5) * resolution;
                
                // 检查是否在圆形范围内
                double dist = std::hypot(wx - robot_x, wy - robot_y);
                if (dist <= local_range) {
                    obstacle_points.emplace_back(wx, wy);
                }
            }
        }
    }
    
    // 改进的聚类：将相邻的障碍物点合并，但对于大型簇（如墙壁）使用多个小圆覆盖
    std::vector<bool> visited(obstacle_points.size(), false);
    
    // 定义小圆的最大覆盖半径（用于分割大型簇）
    const double max_single_obstacle_radius = obstacle_radius_ * 3.0;  // 单个障碍物圆的最大半径
    const size_t max_cluster_size_for_single_circle = 15;  // 超过此点数的簇需要分割
    const double sub_grid_size = obstacle_radius_ * 2.0;  // 子网格大小，用于分割大型簇
    
    for (size_t i = 0; i < obstacle_points.size(); ++i) {
        if (visited[i]) continue;
        
        // 开始一个新的聚类
        std::vector<size_t> cluster;
        cluster.push_back(i);
        visited[i] = true;
        
        // BFS 扩展
        size_t front = 0;
        while (front < cluster.size()) {
            size_t current = cluster[front++];
            double cx = obstacle_points[current].first;
            double cy = obstacle_points[current].second;
            
            for (size_t j = 0; j < obstacle_points.size(); ++j) {
                if (visited[j]) continue;
                
                double dist = std::hypot(obstacle_points[j].first - cx,
                                         obstacle_points[j].second - cy);
                if (dist <= obstacle_cluster_dist_) {
                    cluster.push_back(j);
                    visited[j] = true;
                }
            }
        }
        
        // 根据簇的大小决定如何生成障碍物
        if (cluster.size() <= max_cluster_size_for_single_circle) {
            // 小簇：使用单个圆表示
            double sum_x = 0.0, sum_y = 0.0;
            for (size_t idx : cluster) {
                sum_x += obstacle_points[idx].first;
                sum_y += obstacle_points[idx].second;
            }
            
            Obstacle obs;
            obs.x = sum_x / cluster.size();
            obs.y = sum_y / cluster.size();
            // 限制最大半径
            obs.radius = std::min(obstacle_radius_ + std::sqrt(cluster.size()) * resolution * 0.5,
                                  max_single_obstacle_radius);
            obs.is_static = true;
            obs.id = static_cast<int>(obstacles.size());
            
            obstacles.push_back(obs);
        } else {
            // 大簇（如墙壁）：使用子网格分割，生成多个小圆覆盖
            // 找到簇的边界框
            double min_x = std::numeric_limits<double>::max();
            double max_x = std::numeric_limits<double>::lowest();
            double min_y = std::numeric_limits<double>::max();
            double max_y = std::numeric_limits<double>::lowest();
            
            for (size_t idx : cluster) {
                min_x = std::min(min_x, obstacle_points[idx].first);
                max_x = std::max(max_x, obstacle_points[idx].first);
                min_y = std::min(min_y, obstacle_points[idx].second);
                max_y = std::max(max_y, obstacle_points[idx].second);
            }
            
            // 计算子网格数量
            int grid_nx = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / sub_grid_size)));
            int grid_ny = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / sub_grid_size)));
            
            // 为每个子网格收集点
            std::vector<std::vector<size_t>> sub_grids(grid_nx * grid_ny);
            
            for (size_t idx : cluster) {
                double px = obstacle_points[idx].first;
                double py = obstacle_points[idx].second;
                
                int gx = std::min(grid_nx - 1, static_cast<int>((px - min_x) / sub_grid_size));
                int gy = std::min(grid_ny - 1, static_cast<int>((py - min_y) / sub_grid_size));
                
                sub_grids[gy * grid_nx + gx].push_back(idx);
            }
            
            // 为每个非空子网格生成一个小圆
            for (int gy = 0; gy < grid_ny; ++gy) {
                for (int gx = 0; gx < grid_nx; ++gx) {
                    const auto& sub_cluster = sub_grids[gy * grid_nx + gx];
                    if (sub_cluster.empty()) continue;
                    
                    // 计算子网格中心
                    double sum_x = 0.0, sum_y = 0.0;
                    for (size_t idx : sub_cluster) {
                        sum_x += obstacle_points[idx].first;
                        sum_y += obstacle_points[idx].second;
                    }
                    
                    Obstacle obs;
                    obs.x = sum_x / sub_cluster.size();
                    obs.y = sum_y / sub_cluster.size();
                    // 子网格的障碍物半径固定为较小值
                    obs.radius = obstacle_radius_ + sub_grid_size * 0.5;
                    obs.is_static = true;
                    obs.id = static_cast<int>(obstacles.size());
                    
                    obstacles.push_back(obs);
                }
            }
        }
    }
    
    RCLCPP_DEBUG(get_logger(), "Extracted %zu obstacles from %zu points",
                 obstacles.size(), obstacle_points.size());
    
    return obstacles;
}

bool LocalPlannerNode::getCurrentState(State& state) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!current_odom_) return false;
    
    // 尝试从 TF 获取位置（更准确）
    try {
        auto transform = tf_buffer_->lookupTransform(
            global_frame_, robot_frame_, tf2::TimePointZero);
        
        state.x = transform.transform.translation.x;
        state.y = transform.transform.translation.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        state.theta = yaw;
        
    } catch (const tf2::TransformException& ex) {
        // 使用里程计数据
        state.x = current_odom_->pose.pose.position.x;
        state.y = current_odom_->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(current_odom_->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        state.theta = yaw;
    }
    
    // 速度
    state.v = std::hypot(current_odom_->twist.twist.linear.x,
                         current_odom_->twist.twist.linear.y);
    state.a = 0.0;  // 简化
    
    return true;
}

std::vector<State> LocalPlannerNode::getLocalReferencePath(
    const State& current_state, double lookahead_distance) {
    
    std::vector<State> local_path;
    
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!global_path_ || global_path_->poses.empty()) {
        return local_path;
    }
    
    const auto& poses = global_path_->poses;
    
    // 找到最近点
    size_t nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < poses.size(); ++i) {
        double dist = std::hypot(poses[i].pose.position.x - current_state.x,
                                  poses[i].pose.position.y - current_state.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    
    // 从最近点开始，提取前视距离内的路径
    double accumulated_dist = 0.0;
    
    for (size_t i = nearest_idx; i < poses.size() && accumulated_dist < lookahead_distance; ++i) {
        State s;
        s.x = poses[i].pose.position.x;
        s.y = poses[i].pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(poses[i].pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        s.theta = yaw;
        
        local_path.push_back(s);
        
        if (i > nearest_idx) {
            accumulated_dist += std::hypot(
                poses[i].pose.position.x - poses[i-1].pose.position.x,
                poses[i].pose.position.y - poses[i-1].pose.position.y);
        }
    }
    
    return local_path;
}

void LocalPlannerNode::publishTrajectory(const Trajectory& traj) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = global_frame_;
    
    for (const auto& pt : traj.points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = pt.state.x;
        pose.pose.position.y = pt.state.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, pt.state.theta);
        pose.pose.orientation = tf2::toMsg(q);
        
        path_msg.poses.push_back(pose);
    }
    
    local_path_pub_->publish(path_msg);
}

void LocalPlannerNode::publishVisualization(const Trajectory& traj,
                                             const std::vector<Obstacle>& obstacles) {
    visualization_msgs::msg::MarkerArray markers;
    auto stamp = now();
    
    // 清除旧标记
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = stamp;
    delete_marker.header.frame_id = global_frame_;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // 绘制障碍物
    int id = 0;
    for (const auto& obs : obstacles) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = global_frame_;
        marker.ns = "obstacles";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = obs.x;
        marker.pose.position.y = obs.y;
        marker.pose.position.z = 0.1;
        marker.scale.x = obs.radius * 2;
        marker.scale.y = obs.radius * 2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        markers.markers.push_back(marker);
    }
    
    // 绘制轨迹点
    if (traj.is_valid) {
        visualization_msgs::msg::Marker traj_marker;
        traj_marker.header.stamp = stamp;
        traj_marker.header.frame_id = global_frame_;
        traj_marker.ns = "trajectory";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::msg::Marker::ADD;
        traj_marker.scale.x = 0.05;
        traj_marker.color.r = 0.0;
        traj_marker.color.g = 1.0;
        traj_marker.color.b = 0.0;
        traj_marker.color.a = 1.0;
        traj_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        
        for (const auto& pt : traj.points) {
            geometry_msgs::msg::Point p;
            p.x = pt.state.x;
            p.y = pt.state.y;
            p.z = 0.05;
            traj_marker.points.push_back(p);
        }
        markers.markers.push_back(traj_marker);
    }
    
    viz_pub_->publish(markers);
}

Trajectory LocalPlannerNode::referencePathToInitialTrajectory(
    const std::vector<State>& reference_path,
    const State& current_state,
    const State& goal_state) {
    
    Trajectory traj;
    if (reference_path.size() < 2) return traj;
    
    // 按 iLQR 时域采样参考路径
    double horizon_time = ilqr_config_.horizon_time;
    double dt = ilqr_config_.dt;
    int N = static_cast<int>(horizon_time / dt);
    
    double total_len = 0.0;
    for (size_t i = 1; i < reference_path.size(); ++i) {
        total_len += reference_path[i].distanceTo(reference_path[i - 1]);
    }
    if (total_len < 1e-6) return traj;
    
    // 计算到全局终点的距离
    double dist_to_goal = std::hypot(
        current_state.x - global_goal_.pose.position.x,
        current_state.y - global_goal_.pose.position.y);
    
    // 判断是否需要减速停止
    // 使用减速公式：v² = v0² + 2*a*d => 需要的减速距离 d = v0²/(2*a)
    double stopping_distance = (current_state.v * current_state.v) / 
                               (2.0 * vehicle_params_.max_deceleration);
    bool need_to_stop = (dist_to_goal < std::max(stopping_distance + 1.0, lookahead_distance_));
    
    // 如果需要停止，计算实际需要的时间
    double actual_horizon_time = horizon_time;
    if (need_to_stop && dist_to_goal < total_len) {
        // 使用梯形速度曲线估算停止时间
        // 假设匀速减速到0：t = v / a + d / v
        double avg_speed = (current_state.v + 0.0) / 2.0;
        if (avg_speed > 0.1) {
            actual_horizon_time = std::max(1.5, dist_to_goal / avg_speed);
        } else {
            actual_horizon_time = std::max(1.5, std::sqrt(2.0 * dist_to_goal / vehicle_params_.max_deceleration));
        }
        // 限制最大时间
        actual_horizon_time = std::min(actual_horizon_time, horizon_time);
    }
    
    traj.points.reserve(N + 1);
    
    for (int k = 0; k <= N; ++k) {
        double t = k * dt;
        if (t > actual_horizon_time) break;
        
        TrajectoryPoint pt;
        pt.state.t = t;
        pt.control.acceleration = 0.0;
        pt.control.steering_angle = 0.0;
        
        if (k == 0) {
            pt.state = current_state;
            pt.state.t = t;
        } else {
            // 计算沿路径的进度
            double progress = t / actual_horizon_time;
            double s = progress * std::min(total_len, dist_to_goal);
            
            double acc = 0.0;
            for (size_t i = 1; i < reference_path.size(); ++i) {
                double seg = reference_path[i].distanceTo(reference_path[i - 1]);
                if (acc + seg >= s || i == reference_path.size() - 1) {
                    double alpha = (seg > 1e-6) ? std::clamp((s - acc) / seg, 0.0, 1.0) : 0.0;
                    pt.state.x = reference_path[i - 1].x + alpha * (reference_path[i].x - reference_path[i - 1].x);
                    pt.state.y = reference_path[i - 1].y + alpha * (reference_path[i].y - reference_path[i - 1].y);
                    pt.state.theta = reference_path[i - 1].theta + alpha * (reference_path[i].theta - reference_path[i - 1].theta);
                    
                    // 根据是否需要停止来设置速度
                    if (need_to_stop) {
                        // 使用梯形减速曲线：从当前速度线性减速到0
                        // v(t) = v0 * (1 - t/T) 其中 T 是总时间
                        pt.state.v = current_state.v * (1.0 - progress);
                        // 加速度（减速）
                        pt.state.a = -current_state.v / actual_horizon_time;
                        pt.control.acceleration = pt.state.a;
                    } else {
                        // 普通模式：使用巡航速度
                        pt.state.v = vehicle_params_.max_speed * 0.5;
                        pt.state.a = 0.0;
                    }
                    break;
                }
                acc += seg;
            }
        }
        traj.points.push_back(pt);
    }
    
    // 如果需要停止，确保最后一个点的速度和加速度为0
    if (need_to_stop && !traj.points.empty()) {
        traj.points.back().state.v = 0.0;
        traj.points.back().state.a = 0.0;
        traj.points.back().control.acceleration = 0.0;
    }
    
    traj.is_valid = !traj.points.empty();
    return traj;
}

Trajectory LocalPlannerNode::generateFinalApproachTrajectory(
    const State& current_state,
    const geometry_msgs::msg::PoseStamped& goal_pose,
    double dist_to_goal) {
    /**
     * 生成最终接近轨迹
     * 
     * 当机器人非常接近终点时，直接在笛卡尔坐标系中生成一条
     * 从当前位置到终点的简单减速轨迹。
     * 
     * 使用线性插值位置 + 线性减速速度曲线
     */
    
    Trajectory traj;
    
    // 获取终点位姿
    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(goal_pose.pose.orientation, q);
    double roll, pitch, goal_theta;
    tf2::Matrix3x3(q).getRPY(roll, pitch, goal_theta);
    
    // 计算到达终点所需的时间
    // 使用梯形减速：从当前速度线性减速到0
    double current_v = std::max(current_state.v, 0.1);  // 至少保持 0.1 m/s
    double decel = std::min(vehicle_params_.max_deceleration * 0.5, current_v * current_v / (2.0 * dist_to_goal + 0.01));
    
    // 计算停止时间 t = v/a，但要确保能到达终点
    double stop_time;
    if (decel > 0.01) {
        stop_time = current_v / decel;
    } else {
        stop_time = dist_to_goal / (current_v * 0.5);
    }
    stop_time = std::max(0.5, std::min(stop_time, 5.0));  // 限制在 0.5-5 秒
    
    // 采样轨迹点
    double dt = 0.1;
    int N = static_cast<int>(stop_time / dt) + 1;
    
    for (int k = 0; k <= N; ++k) {
        double t = k * dt;
        double progress = std::min(t / stop_time, 1.0);
        
        // 使用平滑的 S 曲线进行插值（cubic easing）
        // progress' = 3*p^2 - 2*p^3 提供更平滑的加减速
        double smooth_progress = progress * progress * (3.0 - 2.0 * progress);
        
        TrajectoryPoint pt;
        pt.state.t = t;
        
        // 位置：从当前位置到终点的线性插值
        pt.state.x = current_state.x + smooth_progress * (goal_x - current_state.x);
        pt.state.y = current_state.y + smooth_progress * (goal_y - current_state.y);
        
        // 航向角：插值到终点航向
        double dtheta = goal_theta - current_state.theta;
        while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
        while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
        pt.state.theta = current_state.theta + smooth_progress * dtheta;
        
        // 速度：线性减速到 0
        pt.state.v = current_v * (1.0 - progress);
        
        // 加速度（减速）
        pt.state.a = -current_v / stop_time;
        
        // 控制输入
        pt.control.acceleration = pt.state.a;
        
        // 计算转向角（基于曲率）
        if (k > 0 && pt.state.v > 0.01) {
            double dx = pt.state.x - traj.points.back().state.x;
            double dy = pt.state.y - traj.points.back().state.y;
            double ds = std::hypot(dx, dy);
            if (ds > 0.001) {
                double dtheta_seg = pt.state.theta - traj.points.back().state.theta;
                while (dtheta_seg > M_PI) dtheta_seg -= 2.0 * M_PI;
                while (dtheta_seg < -M_PI) dtheta_seg += 2.0 * M_PI;
                double kappa = dtheta_seg / ds;
                pt.control.steering_angle = std::atan(kappa * vehicle_params_.wheelbase);
                pt.control.steering_angle = std::clamp(pt.control.steering_angle, 
                                                        -vehicle_params_.max_steering_angle,
                                                        vehicle_params_.max_steering_angle);
            }
        }
        
        traj.points.push_back(pt);
    }
    
    // 确保最后一个点精确在终点
    if (!traj.points.empty()) {
        traj.points.back().state.x = goal_x;
        traj.points.back().state.y = goal_y;
        traj.points.back().state.theta = goal_theta;
        traj.points.back().state.v = 0.0;
        traj.points.back().state.a = 0.0;
        traj.points.back().control.acceleration = 0.0;
        traj.points.back().control.steering_angle = 0.0;
    }
    
    traj.is_valid = !traj.points.empty();
    traj.total_time = stop_time;
    traj.total_length = dist_to_goal;
    
    return traj;
}

void LocalPlannerNode::publishLatticeVisualization() {
    if (!lattice_planner_) return;
    const auto& candidate_paths = lattice_planner_->getCandidatePaths();
    
    if (candidate_paths.empty()) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray markers;
    auto stamp = now();
    
    // 清除旧标记
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.stamp = stamp;
    delete_marker.header.frame_id = global_frame_;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    int id = 0;
    
    // 找到最优轨迹的索引
    int best_idx = -1;
    double min_cost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < candidate_paths.size(); ++i) {
        if (candidate_paths[i].is_valid && candidate_paths[i].cost_total < min_cost) {
            min_cost = candidate_paths[i].cost_total;
            best_idx = static_cast<int>(i);
        }
    }
    
    // 绘制所有候选轨迹
    for (size_t i = 0; i < candidate_paths.size(); ++i) {
        const auto& path = candidate_paths[i];
        
        if (path.x.empty()) continue;
        
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.stamp = stamp;
        line_marker.header.frame_id = global_frame_;
        line_marker.ns = "lattice_candidates";
        line_marker.id = id++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        line_marker.scale.x = 0.02;  // 线宽
        line_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
        
        // 根据状态设置颜色
        if (static_cast<int>(i) == best_idx) {
            // 最优轨迹 - 绿色，加粗
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;
            line_marker.scale.x = 0.06;
        } else if (path.is_valid) {
            // 有效轨迹 - 蓝色，半透明
            line_marker.color.r = 0.2;
            line_marker.color.g = 0.5;
            line_marker.color.b = 1.0;
            line_marker.color.a = 0.4;
        } else {
            // 无效轨迹（碰撞/约束违反）- 红色，更透明
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.2;
            line_marker.color.b = 0.2;
            line_marker.color.a = 0.2;
        }
        
        // 添加轨迹点
        for (size_t j = 0; j < path.x.size(); ++j) {
            geometry_msgs::msg::Point p;
            p.x = path.x[j];
            p.y = path.y[j];
            p.z = 0.02;
            line_marker.points.push_back(p);
        }
        
        markers.markers.push_back(line_marker);
    }
    
    // 绘制采样终点（横向偏移点）
    visualization_msgs::msg::Marker endpoints_marker;
    endpoints_marker.header.stamp = stamp;
    endpoints_marker.header.frame_id = global_frame_;
    endpoints_marker.ns = "lattice_endpoints";
    endpoints_marker.id = id++;
    endpoints_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    endpoints_marker.action = visualization_msgs::msg::Marker::ADD;
    endpoints_marker.scale.x = 0.1;
    endpoints_marker.scale.y = 0.1;
    endpoints_marker.scale.z = 0.1;
    endpoints_marker.color.r = 1.0;
    endpoints_marker.color.g = 0.8;
    endpoints_marker.color.b = 0.0;
    endpoints_marker.color.a = 0.8;
    endpoints_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
    
    // 添加每条轨迹的终点
    for (const auto& path : candidate_paths) {
        if (!path.x.empty()) {
            geometry_msgs::msg::Point p;
            p.x = path.x.back();
            p.y = path.y.back();
            p.z = 0.05;
            endpoints_marker.points.push_back(p);
        }
    }
    markers.markers.push_back(endpoints_marker);
    
    // 统计信息文本
    int valid_count = 0;
    int invalid_count = 0;
    for (const auto& path : candidate_paths) {
        if (path.is_valid) valid_count++;
        else invalid_count++;
    }
    
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.stamp = stamp;
    text_marker.header.frame_id = global_frame_;
    text_marker.ns = "lattice_info";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 获取当前位置
    State current_state;
    if (getCurrentState(current_state)) {
        text_marker.pose.position.x = current_state.x;
        text_marker.pose.position.y = current_state.y + 1.0;
        text_marker.pose.position.z = 0.5;
    }
    
    text_marker.scale.z = 0.2;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.15);
    
    std::ostringstream oss;
    oss << "Lattice: " << valid_count << " valid / " << invalid_count << " invalid";
    if (best_idx >= 0) {
        oss << "\nBest cost: " << std::fixed << std::setprecision(2) << min_cost;
    }
    text_marker.text = oss.str();
    markers.markers.push_back(text_marker);
    
    lattice_viz_pub_->publish(markers);
}

} // namespace local_planner

// ========== Main ==========

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_planner::LocalPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
