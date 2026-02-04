#ifndef RRT_CONNECT_HPP
#define RRT_CONNECT_HPP

/**
 * @file rrt_connect.hpp
 * @brief RRT-Connect 全局路径规划算法
 *
 * 双树从起点和终点同时扩展，在自由空间中随机采样，
 * 两树相遇且连线无碰撞时得到路径。
 */

#include "base_algorithm.hpp"
#include <vector>
#include <random>
#include <cmath>

namespace global_planner {

/** RRT 树节点（世界坐标） */
struct RRTNode {
    double x = 0.0;
    double y = 0.0;
    int parent_index = -1;
    RRTNode() = default;
    RRTNode(double x_, double y_, int parent = -1) : x(x_), y(y_), parent_index(parent) {}
};

class RRTConnect : public BaseAlgorithm {
public:
    RRTConnect() = default;
    ~RRTConnect() override = default;

    RRTConnect(const RRTConnect&) = delete;
    RRTConnect& operator=(const RRTConnect&) = delete;

    void initialize(const std::string& name) override;
    std::string getName() const override { return name_; }
    std::vector<int8_t> getProcessedMap() override { return map_data_; }
    bool makePlan(const Pose2D& start, const Pose2D& goal, std::vector<Pose2D>& path) override;
    void updateMap(const GridMapData& map_data) override;
    void configure(const PlannerConfig& config) override;

private:
    /** 世界坐标 -> 栅格坐标 */
    void worldToGrid(double wx, double wy, int& gx, int& gy) const;
    /** 栅格是否在边界内 */
    inline bool isInside(int gx, int gy) const;
    /** 世界坐标点是否碰撞 */
    bool isCollision(double wx, double wy) const;
    /** 线段 (x1,y1)-(x2,y2) 是否无碰撞 */
    bool isPathClear(double x1, double y1, double x2, double y2) const;
    /** 在自由空间采样随机点 */
    bool sampleRandom(double& wx, double& wy);
    /** 树上离 (wx,wy) 最近的节点索引 */
    int nearest(const std::vector<RRTNode>& tree, double wx, double wy) const;
    /** 从 (from_x,from_y) 向 (to_x,to_y) 步进 step_size，输出新点 */
    void steer(double from_x, double from_y, double to_x, double to_y,
               double step_size, double& out_x, double& out_y) const;
    /** 扩展：从树向 q_rand 扩展一步，返回新节点索引，-1 表示未扩展 */
    int extend(std::vector<RRTNode>& tree, double q_rand_x, double q_rand_y);
    /** 连接：从树反复向 q 扩展直到到达或碰撞，返回最后添加的节点索引 */
    int connect(std::vector<RRTNode>& tree, double q_x, double q_y);
    /** 路径缩短：去掉可直线无碰撞连接的中间点，使路径更顺滑 */
    void shortenPath(std::vector<Pose2D>& path) const;

    void inflateMap(std::vector<int8_t>& data);

    PlannerConfig config_;
    std::string name_;
    std::vector<int8_t> map_data_;
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    double step_size_ = 0.2;
    int max_iterations_ = 5000;

    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_x_{0.0, 1.0};
    std::uniform_real_distribution<double> dist_y_{0.0, 1.0};
};

} // namespace global_planner

#endif // RRT_CONNECT_HPP
