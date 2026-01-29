#include "algorithms/astar.hpp"
#include <algorithm>
#include <queue>

namespace global_planner {

void AStar::initialize(const std::string& name) {
    name_ = name;
}

void AStar::configure(const PlannerConfig& config) {
    config_ = config;
}

void AStar::updateMap(const GridMapData& map_data) {
    width_ = static_cast<int>(map_data.width);
    height_ = static_cast<int>(map_data.height);
    resolution_ = map_data.resolution;
    origin_x_ = map_data.origin_x;
    origin_y_ = map_data.origin_y;

    std::vector<int8_t> processed_data = map_data.data;
    if (config_.use_inflation) {
        inflateMap(processed_data);
    }

    map_data_ = std::move(processed_data);
    
    // 预分配搜索相关的内存
    const size_t map_size = static_cast<size_t>(width_ * height_);
    visited_.resize(map_size, false);
    node_index_.resize(map_size, -1);
    node_pool_.reserve(map_size / 4);  // 预估最多访问 1/4 的节点
}

void AStar::inflateMap(std::vector<int8_t>& data) {
    const int inflation_cells = static_cast<int>(config_.inflation_radius / resolution_);
    if (inflation_cells <= 0) {
        return;
    }

    std::vector<int> dist_map(width_ * height_, 255); 
    std::queue<std::pair<int, int>> q;

    // 将所有原始障碍物入队
    for (int i = 0; i < width_ * height_; ++i) {
        if (data[i] > 50) {
            dist_map[i] = 0;
            int x = i % width_;
            int y = i / width_;
            q.push({x, y});
        }
    }

    // BFS 扩散
    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        int current_dist = dist_map[y * width_ + x];
        if (current_dist >= inflation_cells) continue;

        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            int nx = x + dx_[i];
            int ny = y + dy_[i];

            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                int idx = ny * width_ + nx;
                if (dist_map[idx] > current_dist + 1) {
                    dist_map[idx] = current_dist + 1;
                    data[idx] = 100;
                    q.push({nx, ny});
                }
            }
        }
    }
}

double AStar::getHeuristic(int x1, int y1, int x2, int y2) const {
    return std::hypot(x1 - x2, y1 - y2);
}

void AStar::resetSearch() {
    // 只重置使用过的节点（比全量重置快）
    for (const auto& node : node_pool_) {
        int idx = toIndex(node.x, node.y);
        visited_[idx] = false;
        node_index_[idx] = -1;
    }
    node_pool_.clear();
}

bool AStar::makePlan(const Pose2D& start, const Pose2D& goal, 
                     std::vector<Pose2D>& path) {
    path.clear();
    
    if (map_data_.empty()) {
        return false;
    }

    // 世界坐标转栅格坐标
    const int start_x = static_cast<int>(std::round((start.x - origin_x_) / resolution_));
    const int start_y = static_cast<int>(std::round((start.y - origin_y_) / resolution_));
    const int goal_x = static_cast<int>(std::round((goal.x - origin_x_) / resolution_));
    const int goal_y = static_cast<int>(std::round((goal.y - origin_y_) / resolution_));

    // 边界检查
    if (!isInside(start_x, start_y) || !isInside(goal_x, goal_y)) {
        return false;
    }
    
    // 检查起点和终点是否在障碍物上
    if (map_data_[toIndex(start_x, start_y)] > 50 || 
        map_data_[toIndex(goal_x, goal_y)] > 50) {
        return false;
    }

    // 重置搜索状态
    resetSearch();

    // 优先队列比较器
    auto cmp = [this](int a, int b) {
        return node_pool_[a].f() > node_pool_[b].f();
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open_list(cmp);

    // 插入起点
    node_pool_.emplace_back(start_x, start_y);
    node_pool_.back().h = getHeuristic(start_x, start_y, goal_x, goal_y);
    int start_pool_idx = static_cast<int>(node_pool_.size()) - 1;
    node_index_[toIndex(start_x, start_y)] = start_pool_idx;
    open_list.push(start_pool_idx);

    int goal_pool_idx = -1;

    // A* 搜索
    while (!open_list.empty()) {
        int current_pool_idx = open_list.top();
        open_list.pop();
        
        const AStarNode& current = node_pool_[current_pool_idx];
        int current_grid_idx = toIndex(current.x, current.y);
        
        // 已访问过则跳过
        if (visited_[current_grid_idx]) {
            continue;
        }
        visited_[current_grid_idx] = true;

        // 到达目标
        if (current.x == goal_x && current.y == goal_y) {
            goal_pool_idx = current_pool_idx;
            break;
        }

        // 扩展邻居
        for (int i = 0; i < NUM_DIRECTIONS; ++i) {
            int nx = current.x + dx_[i];
            int ny = current.y + dy_[i];
            
            if (!isInside(nx, ny)) continue;
            
            int next_grid_idx = toIndex(nx, ny);
            
            // 障碍物或已访问
            if (map_data_[next_grid_idx] > 50 || map_data_[next_grid_idx] < 0 ||
                visited_[next_grid_idx]) {
                continue;
            }
            
            double new_g = current.g + move_cost_[i];
            
            int existing_pool_idx = node_index_[next_grid_idx];
            if (existing_pool_idx == -1) {
                // 新节点
                node_pool_.emplace_back(nx, ny);
                int new_pool_idx = static_cast<int>(node_pool_.size()) - 1;
                node_pool_[new_pool_idx].g = new_g;
                node_pool_[new_pool_idx].h = getHeuristic(nx, ny, goal_x, goal_y);
                node_pool_[new_pool_idx].parent_index = current_pool_idx;
                node_index_[next_grid_idx] = new_pool_idx;
                open_list.push(new_pool_idx);
            } else if (new_g < node_pool_[existing_pool_idx].g) {
                // 找到更短路径
                node_pool_[existing_pool_idx].g = new_g;
                node_pool_[existing_pool_idx].parent_index = current_pool_idx;
                open_list.push(existing_pool_idx);  // 重新加入队列
            }
        }
    }

    // 回溯路径
    if (goal_pool_idx != -1) {
        int idx = goal_pool_idx;
        while (idx != -1) {
            const AStarNode& node = node_pool_[idx];
            Pose2D p;
            p.x = node.x * resolution_ + origin_x_;
            p.y = node.y * resolution_ + origin_y_;
            p.theta = 0.0;
            p.kappa = 0.0;
            path.push_back(p);
            idx = node.parent_index;
        }
        std::reverse(path.begin(), path.end());
        return true;
    }

    return false;
}

} // namespace global_planner
