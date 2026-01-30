#ifndef LOCAL_PLANNER_NODE_HPP
#define LOCAL_PLANNER_NODE_HPP

/**
 * @file local_planner_node.hpp
 * @brief 局部路径规划器 ROS2 节点
 * 
 * 功能：
 * - 订阅全局路径作为参考路径
 * - 从全局地图提取局部障碍物
 * - 使用 Lattice Planner 生成局部轨迹
 * - 发布轨迹供控制器跟踪
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/types.hpp"
#include "local_planner/algorithms/lattice_planner.hpp"

#include <mutex>
#include <memory>

namespace local_planner {

/**
 * @brief 局部规划器节点
 */
class LocalPlannerNode : public rclcpp::Node {
public:
    LocalPlannerNode();
    ~LocalPlannerNode() = default;

private:
    // ========== 回调函数 ==========
    
    /**
     * @brief 全局地图回调
     */
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
    /**
     * @brief 全局路径回调
     */
    void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    
    /**
     * @brief 里程计回调
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief 定时器回调 - 执行规划
     */
    void planningTimerCallback();
    
    // ========== 核心功能 ==========
    
    /**
     * @brief 从全局地图提取局部障碍物
     * @param robot_x 机器人 x 坐标
     * @param robot_y 机器人 y 坐标
     * @param local_range 局部范围半径 (m)
     * @return 障碍物列表
     */
    std::vector<Obstacle> extractLocalObstacles(double robot_x, double robot_y, 
                                                 double local_range);
    
    /**
     * @brief 获取机器人当前状态
     */
    bool getCurrentState(State& state);
    
    /**
     * @brief 截取参考路径的局部部分
     */
    std::vector<State> getLocalReferencePath(const State& current_state, 
                                              double lookahead_distance);
    
    /**
     * @brief 发布轨迹
     */
    void publishTrajectory(const Trajectory& traj);
    
    /**
     * @brief 发布可视化标记
     */
    void publishVisualization(const Trajectory& traj, 
                               const std::vector<Obstacle>& obstacles);
    
    /**
     * @brief 发布 Lattice Planner 候选轨迹可视化
     */
    void publishLatticeVisualization();

    // ========== ROS2 接口 ==========
    
    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lattice_viz_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr planning_timer_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // ========== 数据存储 ==========
    
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
    nav_msgs::msg::Path::SharedPtr global_path_;
    nav_msgs::msg::Odometry::SharedPtr current_odom_;
    
    std::mutex map_mutex_;
    std::mutex path_mutex_;
    std::mutex odom_mutex_;
    
    // ========== 规划器 ==========
    
    std::unique_ptr<LatticePlanner> planner_;
    LatticeConfig lattice_config_;
    VehicleParams vehicle_params_;
    
    // ========== 参数 ==========
    
    std::string global_frame_;
    std::string robot_frame_;
    double planning_frequency_;      // 规划频率 (Hz)
    double local_range_;             // 局部障碍物提取范围 (m)
    double lookahead_distance_;      // 参考路径前视距离 (m)
    double obstacle_radius_;         // 障碍物等效半径 (m)
    int obstacle_threshold_;         // 障碍物阈值 (0-100)
    double obstacle_cluster_dist_;   // 障碍物聚类距离 (m)
    
    bool has_map_;
    bool has_path_;
    bool has_odom_;
    bool visualize_lattice_;         // 是否可视化 Lattice 候选轨迹
};

} // namespace local_planner

#endif // LOCAL_PLANNER_NODE_HPP
