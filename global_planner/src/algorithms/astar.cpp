#include "algorithms/astar.hpp"
#include <algorithm>

namespace global_planner {

void AStar::initialize(const string& name) {
    name_ = name;
}

void AStar::configure(const PlannerConfig& config) {
    config_ = config;
}

void AStar::updateMap(const GridMapData& map_data) {
    width_ = map_data.width;
    height_ = map_data.height;
    resolution_ = map_data.resolution;
    origin_x_ = map_data.origin_x;
    origin_y_ = map_data.origin_y;

    vector<int8_t> processed_data = map_data.data;
    if (config_.use_inflation) {
        inflateMap(processed_data);
    }

    map_data_ = std::move(processed_data);
}

void AStar::inflateMap(std::vector<int8_t>& data) {
    // 1. 计算膨胀半径对应的像素格数
    int inflation_cells = static_cast<int>(config_.inflation_radius / resolution_);
    if (inflation_cells <= 0) {
        return;
    }

    // 2. 准备：dist_map 记录到最近障碍物的距离（初始化为极大值）
    // 使用 int16_t 节省内存
    std::vector<int> dist_map(width_ * height_, 255); 
    std::queue<std::pair<int, int>> q;

    // 3. 将所有原始障碍物入队
    for (int i = 0; i < width_ * height_; ++i) {
        if (data[i] > 50) { // 原始障碍物
            dist_map[i] = 0;
            int x = i % width_;
            int y = i / width_;
            q.push({x, y});
        }
    }

    // 4. BFS 扩散
    int dx[] = {1, -1, 0, 0, 1, 1, -1, -1}; // 8 邻域
    int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        int current_dist = dist_map[y * width_ + x];
        if (current_dist >= inflation_cells) continue; // 超过半径，停止扩散

        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            // 边界检查
            if (nx >= 0 && nx < (int)width_ && ny >= 0 && ny < (int)height_) {
                int idx = ny * width_ + nx;
                // 如果邻居还没被访问过（或者找到更近的距离）
                if (dist_map[idx] > current_dist + 1) {
                    dist_map[idx] = current_dist + 1;
                    data[idx] = 100; // 标记为膨胀障碍物
                    q.push({nx, ny});
                }
            }
        }
    }
}

double AStar::getHeuristic(int x1, int y1, int x2, int y2) {
    // Use Euclidean distance
    return hypot(x1 - x2, y1 - y2);
}

bool AStar::makePlan(const Pose2D& start, const Pose2D& goal, 
                        std::vector<Pose2D>& path) {
    
    path.clear();

    // 1. world to index
    int start_x = round((start.x - origin_x_) / resolution_);
    int start_y = round((start.y - origin_y_) / resolution_);
    int goal_x = round((goal.x - origin_x_) / resolution_);
    int goal_y = round((goal.y - origin_y_) / resolution_);

    // boundary check
    if (!isInside(start_x, start_y) || !isInside(goal_x, goal_y)) {
        return false;
    }

    // 2. initialize containers
    priority_queue<AStarNode*, vector<AStarNode*>, NodeComparator> open_list;
    unordered_map<int, AStarNode*> all_nodes;

    // Insert start point
    AStarNode* start_node = new AStarNode(start_x, start_y);
    start_node->h = getHeuristic(start_x, start_y, goal_x, goal_y);
    open_list.push(start_node);
    all_nodes[toIndex(start_x, start_y)] = start_node;

    AStarNode* current_node = nullptr;
    bool found_goal = false;

    // 3. start searching
    while (!open_list.empty()) {
        current_node = open_list.top();
        open_list.pop();

        // reach to goal
        if (current_node->x == goal_x && current_node->y == goal_y) {
            found_goal = true;
            break;
        }

        //search neighbors
        for (const auto& m : motions_) {
            int next_x = current_node->x + m.first;
            int next_y = current_node->y + m.second;
            int next_idx = toIndex(next_x, next_y);

            // collision and boundary check
            if (!isInside(next_x, next_y) || map_data_[next_idx] > 50 
                    || map_data_[next_idx] < 0) {
                continue;
            }
            double move_cost = hypot(m.first, m.second);
            double new_g = current_node->g + move_cost;

            // if the node is not explored or find shorter path
            if (all_nodes.find(next_idx) == all_nodes.end() || 
                                new_g < all_nodes[next_idx]->g) {
                AStarNode* next_node;
                if (all_nodes.find(next_idx) == all_nodes.end()) {
                    next_node = new AStarNode(next_x, next_y);
                    all_nodes[next_idx] = next_node;
                } else {
                    next_node = all_nodes[next_idx];
                }

                next_node->g = new_g;
                next_node->h = next_node->h = getHeuristic(next_x, next_y, goal_x, goal_y);
                next_node->parent = current_node;
                open_list.push(next_node);
            }
        }
    }

    // 4. backtracking path
    if (found_goal) {
        AStarNode* temp = current_node;
        while (temp != nullptr) {
            Pose2D p;
            p.x = temp->x * resolution_ + origin_x_;
            p.y = temp->y * resolution_ + origin_y_;
            p.theta = 0.0; // no need yaw information;
            path.push_back(p);
            temp = temp->parent;
        }
        reverse(path.begin(), path.end());
    }

    // 5. clear the memory
    for (auto& entry : all_nodes) {
        delete entry.second;
    }

    return found_goal;
}

} // namespace global_planner