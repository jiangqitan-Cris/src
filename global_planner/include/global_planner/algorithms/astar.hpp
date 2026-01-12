#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <iostream>
#include "base_algorithm.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <memory>
#include <cmath>

namespace global_planner {

// A* search node
struct AStarNode {
    int x, y;
    double g = 0.0;
    double h = 0.0;
    AStarNode* parent = nullptr;

    AStarNode(int x, int y) : x(x), y(y) {}
    double f() const { return g + h; }
};

// compare operator, to compare the priority queue
struct NodeComparator {
    bool operator() (const AStarNode* a, const AStarNode* b) const {
        return a->f() > b->f();
    }
};

class AStar : public BaseAlgorithm {

public:
    // construction and destruction
    AStar() = default;
    ~AStar() override = default;

    void initialize(const string& name) override;
    string getName() const override { return name_; }
    vector<int8_t> getProcessedMap() override { return map_data_; }
    bool makePlan(const Pose2D& start, const Pose2D& goal,
                    vector<Pose2D>& path) override;
    void updateMap(const GridMapData& map_data) override;
    void configure(const PlannerConfig& config) override;
    
private:
    // functions
    void inflateMap(vector<int8_t>& data);
    double getHeuristic(int x1, int y1, int x2, int y2);
    inline int toIndex(int x, int y) { return y * width_ + x; };
    inline bool isInside(int x, int y) 
        { return x >= 0 && x < width_ && y >= 0 && y < height_; }

    // parameters
    PlannerConfig config_;
    string name_;
    vector<int8_t> map_data_;
    int width_ = 0;
    int height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    // 八邻域搜索方向
    const std::vector<std::pair<int, int>> motions_ = {
        {0, 1}, {1, 0}, {0, -1}, {-1, 0},   // 上下左右
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}  // 对角线
    };

};

} // namespace global_planner

#endif // ASTAR_HPP