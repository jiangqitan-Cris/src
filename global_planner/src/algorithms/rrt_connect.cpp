#include "algorithms/rrt_connect.hpp"
#include <algorithm>
#include <queue>
#include <chrono>

namespace global_planner {

void RRTConnect::initialize(const std::string& name) {
    name_ = name;
}

void RRTConnect::configure(const PlannerConfig& config) {
    config_ = config;
    step_size_ = config_.rrt_step_size;
    max_iterations_ = config_.rrt_max_iterations;
}

void RRTConnect::updateMap(const GridMapData& map_data) {
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

    double x_min = origin_x_;
    double x_max = origin_x_ + width_ * resolution_;
    double y_min = origin_y_;
    double y_max = origin_y_ + height_ * resolution_;
    dist_x_ = std::uniform_real_distribution<double>(x_min, x_max);
    dist_y_ = std::uniform_real_distribution<double>(y_min, y_max);
    rng_.seed(static_cast<unsigned>(std::chrono::steady_clock::now().time_since_epoch().count()));
}

void RRTConnect::inflateMap(std::vector<int8_t>& data) {
    const int inflation_cells = static_cast<int>(config_.inflation_radius / resolution_);
    if (inflation_cells <= 0) return;

    std::vector<int> dist_map(width_ * height_, 255);
    std::queue<std::pair<int, int>> q;

    for (int i = 0; i < width_ * height_; ++i) {
        if (data[i] > 50) {
            dist_map[i] = 0;
            q.push({i % width_, i / width_});
        }
    }

    const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
    const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

    while (!q.empty()) {
        int x = q.front().first;
        int y = q.front().second;
        q.pop();
        int d = dist_map[y * width_ + x];
        if (d >= inflation_cells) continue;

        for (int i = 0; i < 8; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                int idx = ny * width_ + nx;
                if (dist_map[idx] > d + 1) {
                    dist_map[idx] = d + 1;
                    data[idx] = 100;
                    q.push({nx, ny});
                }
            }
        }
    }
}

void RRTConnect::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::round((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::round((wy - origin_y_) / resolution_));
}

bool RRTConnect::isInside(int gx, int gy) const {
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

bool RRTConnect::isCollision(double wx, double wy) const {
    int gx, gy;
    worldToGrid(wx, wy, gx, gy);
    if (!isInside(gx, gy)) return true;
    return map_data_[gy * width_ + gx] > 50;
}

bool RRTConnect::isPathClear(double x1, double y1, double x2, double y2) const {
    double dist = std::hypot(x2 - x1, y2 - y1);
    if (dist < 1e-9) return !isCollision(x1, y1);

    int steps = std::max(2, static_cast<int>(dist / (resolution_ * 0.5)));
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double x = x1 + t * (x2 - x1);
        double y = y1 + t * (y2 - y1);
        if (isCollision(x, y)) return false;
    }
    return true;
}

bool RRTConnect::sampleRandom(double& wx, double& wy) {
    for (int i = 0; i < 100; ++i) {
        wx = dist_x_(rng_);
        wy = dist_y_(rng_);
        if (!isCollision(wx, wy)) return true;
    }
    return false;
}

int RRTConnect::nearest(const std::vector<RRTNode>& tree, double wx, double wy) const {
    int best = 0;
    double best_d = std::hypot(tree[0].x - wx, tree[0].y - wy);
    for (size_t i = 1; i < tree.size(); ++i) {
        double d = std::hypot(tree[i].x - wx, tree[i].y - wy);
        if (d < best_d) {
            best_d = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

void RRTConnect::steer(double from_x, double from_y, double to_x, double to_y,
                       double step_size, double& out_x, double& out_y) const {
    double dist = std::hypot(to_x - from_x, to_y - from_y);
    if (dist <= step_size) {
        out_x = to_x;
        out_y = to_y;
        return;
    }
    double t = step_size / dist;
    out_x = from_x + t * (to_x - from_x);
    out_y = from_y + t * (to_y - from_y);
}

int RRTConnect::extend(std::vector<RRTNode>& tree, double q_rand_x, double q_rand_y) {
    int n = nearest(tree, q_rand_x, q_rand_y);
    double new_x, new_y;
    steer(tree[n].x, tree[n].y, q_rand_x, q_rand_y, step_size_, new_x, new_y);
    if (!isPathClear(tree[n].x, tree[n].y, new_x, new_y))
        return -1;
    tree.emplace_back(new_x, new_y, n);
    return static_cast<int>(tree.size()) - 1;
}

int RRTConnect::connect(std::vector<RRTNode>& tree, double q_x, double q_y) {
    while (true) {
        int n = nearest(tree, q_x, q_y);
        double dist = std::hypot(tree[n].x - q_x, tree[n].y - q_y);
        if (dist < step_size_ * 0.99) {
            if (isPathClear(tree[n].x, tree[n].y, q_x, q_y)) {
                tree.emplace_back(q_x, q_y, n);
                return static_cast<int>(tree.size()) - 1;
            }
            return -1;
        }
        double new_x, new_y;
        steer(tree[n].x, tree[n].y, q_x, q_y, step_size_, new_x, new_y);
        if (!isPathClear(tree[n].x, tree[n].y, new_x, new_y))
            return n;
        tree.emplace_back(new_x, new_y, n);
    }
}

bool RRTConnect::makePlan(const Pose2D& start, const Pose2D& goal, std::vector<Pose2D>& path) {
    path.clear();
    if (map_data_.empty()) return false;

    if (isCollision(start.x, start.y) || isCollision(goal.x, goal.y))
        return false;

    std::vector<RRTNode> tree_a, tree_b;
    tree_a.emplace_back(start.x, start.y, -1);
    tree_b.emplace_back(goal.x, goal.y, -1);

    bool tree_a_extends = true;
    int goal_a = -1, goal_b = -1;

    for (int iter = 0; iter < max_iterations_; ++iter) {
        double q_rand_x, q_rand_y;
        if (!sampleRandom(q_rand_x, q_rand_y)) continue;

        if (tree_a_extends) {
            int new_idx = extend(tree_a, q_rand_x, q_rand_y);
            if (new_idx >= 0) {
                double q_new_x = tree_a[new_idx].x;
                double q_new_y = tree_a[new_idx].y;
                int conn = connect(tree_b, q_new_x, q_new_y);
                if (conn >= 0) {
                    double end_x = tree_b[conn].x;
                    double end_y = tree_b[conn].y;
                    if (std::hypot(end_x - q_new_x, end_y - q_new_y) < step_size_ * 0.5) {
                        goal_a = new_idx;
                        goal_b = conn;
                        break;
                    }
                }
            }
        } else {
            int new_idx = extend(tree_b, q_rand_x, q_rand_y);
            if (new_idx >= 0) {
                double q_new_x = tree_b[new_idx].x;
                double q_new_y = tree_b[new_idx].y;
                int conn = connect(tree_a, q_new_x, q_new_y);
                if (conn >= 0) {
                    double end_x = tree_a[conn].x;
                    double end_y = tree_a[conn].y;
                    if (std::hypot(end_x - q_new_x, end_y - q_new_y) < step_size_ * 0.5) {
                        goal_a = conn;
                        goal_b = new_idx;
                        break;
                    }
                }
            }
        }
        tree_a_extends = !tree_a_extends;
    }

    if (goal_a < 0 || goal_b < 0) return false;

    path.clear();
    int idx = goal_a;
    while (idx >= 0) {
        path.push_back({tree_a[idx].x, tree_a[idx].y, 0.0, 0.0});
        idx = tree_a[idx].parent_index;
    }
    std::reverse(path.begin(), path.end());

    idx = goal_b;
    while (idx >= 0) {
        path.push_back({tree_b[idx].x, tree_b[idx].y, 0.0, 0.0});
        idx = tree_b[idx].parent_index;
    }

    shortenPath(path);
    return true;
}

void RRTConnect::shortenPath(std::vector<Pose2D>& path) const {
    if (path.size() <= 2) return;

    std::vector<Pose2D> out;
    out.reserve(path.size());
    out.push_back(path.front());

    size_t current = 0;
    const size_t n = path.size();

    while (current < n - 1) {
        size_t next = n - 1;
        for (; next > current + 1; --next) {
            if (isPathClear(path[current].x, path[current].y, path[next].x, path[next].y)) {
                break;
            }
        }
        out.push_back(path[next]);
        current = next;
    }

    path = std::move(out);
}

} // namespace global_planner
