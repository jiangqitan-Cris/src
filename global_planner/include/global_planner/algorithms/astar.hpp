#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "base_algorithm.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <functional>

namespace global_planner {

/**
 * @brief A* 搜索节点
 */
struct AStarNode {
    int x = 0;
    int y = 0;
    double g = 0.0;  // 从起点到当前节点的代价
    double h = 0.0;  // 启发式估计代价
    int parent_index = -1;  // 父节点在 node_pool_ 中的索引，-1 表示无父节点

    AStarNode() = default;
    AStarNode(int x_, int y_) : x(x_), y(y_) {}
    double f() const { return g + h; }
};

/**
 * @brief A* 算法实现
 * 
 * 特点：
 * - 使用对象池避免频繁内存分配
 * - 八邻域搜索
 * - 支持地图膨胀
 */
class AStar : public BaseAlgorithm {

public:
    AStar() = default;
    ~AStar() override = default;

    // 禁止拷贝
    AStar(const AStar&) = delete;
    AStar& operator=(const AStar&) = delete;

    void initialize(const std::string& name) override;
    std::string getName() const override { return name_; }
    std::vector<int8_t> getProcessedMap() override { return map_data_; }
    bool makePlan(const Pose2D& start, const Pose2D& goal,
                  std::vector<Pose2D>& path) override;
    void updateMap(const GridMapData& map_data) override;
    void configure(const PlannerConfig& config) override;
    
private:
    /**
     * @brief 膨胀地图
     * @param data 地图数据（会被修改）
     */
    void inflateMap(std::vector<int8_t>& data);
    
    /**
     * @brief 计算启发式代价（欧几里得距离）
     */
    double getHeuristic(int x1, int y1, int x2, int y2) const;
    
    /**
     * @brief 坐标转索引
     */
    inline int toIndex(int x, int y) const { return y * width_ + x; }
    
    /**
     * @brief 边界检查
     */
    inline bool isInside(int x, int y) const { 
        return x >= 0 && x < width_ && y >= 0 && y < height_; 
    }
    
    /**
     * @brief 重置搜索状态
     */
    void resetSearch();

    // 配置和地图参数
    PlannerConfig config_;
    std::string name_;
    std::vector<int8_t> map_data_;
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    // 对象池：避免频繁内存分配
    std::vector<AStarNode> node_pool_;
    std::vector<bool> visited_;      // 访问标记
    std::vector<int> node_index_;    // 栅格索引 -> node_pool_ 索引的映射

    // 八邻域搜索方向
    static constexpr int NUM_DIRECTIONS = 8;
    const int dx_[NUM_DIRECTIONS] = {0, 1, 0, -1, 1, 1, -1, -1};
    const int dy_[NUM_DIRECTIONS] = {1, 0, -1, 0, 1, -1, 1, -1};
    const double move_cost_[NUM_DIRECTIONS] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};
};

} // namespace global_planner

#endif // ASTAR_HPP