#include "global_planner/smoother/smoother.hpp"

bool IntegratedSmoother::isLineClear(const Pose2D& p1, const Pose2D& p2, 
                                     const std::function<bool(double, double)>& is_occupied) {
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    int steps = std::max(static_cast<int>(dist / 0.05), 2); 
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        if (is_occupied(p1.x + t*(p2.x-p1.x), p1.y + t*(p2.y-p1.y))) return false;
    }
    return true;
}

std::vector<Pose2D> IntegratedSmoother::prunePath(const std::vector<Pose2D>& path,
                                                  const std::function<bool(double, double)>& is_occupied) {
    if (path.size() < 3) return path;
    std::vector<Pose2D> pruned;
    pruned.push_back(path.front());
    for (size_t i = 1; i < path.size() - 1; ++i) {
        if (!isLineClear(pruned.back(), path[i+1], is_occupied)) {
            pruned.push_back(path[i]);
        }
    }
    pruned.push_back(path.back());
    return pruned;
}

std::vector<Pose2D> IntegratedSmoother::fixDensity(const std::vector<Pose2D>& path, double step_size) {
    if (path.size() < 2) return path;

    std::vector<Pose2D> dense_path;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];

        double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
        dense_path.push_back(p1); // 加入当前段的起点

        if (dist > step_size) {
            int num_steps = static_cast<int>(dist / step_size);
            for (int j = 1; j < num_steps; ++j) {
                double t = static_cast<double>(j) / num_steps;
                Pose2D interpolated_point;
                interpolated_point.x = p1.x + t * (p2.x - p1.x);
                interpolated_point.y = p1.y + t * (p2.y - p1.y);
                // 如果 Pose2D 有角度 theta，也可以在这里做线性插值或保持不变
                dense_path.push_back(interpolated_point);
            }
        }
    }
    
    dense_path.push_back(path.back()); // 加入最后一个终点
    return dense_path;
}

std::vector<Pose2D> IntegratedSmoother::smooth(const std::vector<Pose2D>& path,
                                                const std::function<bool(double, double)>& is_occupied) {
    if (path.size() < 2) {
        return path;
    }
    
    std::vector<double> ctrl_x, ctrl_y;
    for (const auto& p : path) {
        ctrl_x.push_back(p.x);
        ctrl_y.push_back(p.y);
    }

    bool has_collision = true;
    int max_iterations = 10;
    int iter = 0;

    while (has_collision && iter < max_iterations) {
        has_collision = false;
        cpprobotics::Spline2D spline(ctrl_x, ctrl_y);

        std::vector<Pose2D> current_smooth_path;
        double collision_s = -1.0;

        // 采样检测碰撞
        for (double s = 0; s <= spline.s.back(); s += 0.1) {
            cpprobotics::Poi_d pos = spline.calc_position(s);
            if (is_occupied(pos[0], pos[1])) {
                has_collision = true;
                collision_s = s;
                break;
            }

            // 没有碰撞就插入
            Pose2D temp_pose;
            temp_pose.x = pos[0];
            temp_pose.y = pos[1];
            temp_pose.theta = spline.calc_yaw(s);
            current_smooth_path.push_back(temp_pose);
        }

        if (has_collision) {
            // 如果碰撞了，那就在碰撞发生的两个控制点中间插入新的控制点进行重新样条拟合
            auto it = std::lower_bound(spline.s.begin(), spline.s.end(), collision_s);
            int idx = std::distance(spline.s.begin(), it);

            if (idx > 0 && idx < ctrl_x.size()) {
                double insert_x = (ctrl_x[idx] + ctrl_x[idx - 1]) / 2;
                double insert_y = (ctrl_y[idx] + ctrl_y[idx - 1]) / 2;
                ctrl_x.insert(ctrl_x.begin() + idx - 1, insert_x);
                ctrl_y.insert(ctrl_y.begin() + idx - 1, insert_y);
            }
            iter++;
        } else {
            // 没有碰撞，直接返回路径
            return current_smooth_path;
        }
    }

    // 如果达到最大迭代次数仍有碰撞，返回原始路径（保底安全）
    std::cout << "Cannot find a cubic spline with no collision" << std::endl;
    return path;
}

std::vector<Pose2D> IntegratedSmoother::postProcess(const std::vector<Pose2D>& path,
                                              const std::function<bool(double, double)>& is_occupied,
                                              double w_smooth, double w_data, double w_obs, double obs_dist) {
    if (path.size() < 3) return path;
    std::cout << "Start post process smooth path" << std::endl;

    std::vector<Pose2D> new_path = path;
    int max_iters = 200;
    double alpha = 0.05; // learning rate
    double min_dist_to_obs = 0.3;

    for (int iter = 0; iter < max_iters; iter++) {
        for (size_t i = 1; i < new_path.size() - 1; i++) {
            double gx = 0, gy = 0;

            // 1. 平滑力
            gx += w_smooth * (new_path[i-1].x + new_path[i+1].x - 2.0 * new_path[i].x);
            gy += w_smooth * (new_path[i-1].y + new_path[i+1].y - 2.0 * new_path[i].y);

            // 2. 引导力
            gx += w_data * (path[i].x - new_path[i].x);
            gy += w_data * (path[i].y - new_path[i].y);

            // 3. 避障排斥力（寻找半径内最近障碍物）
            double ox, oy, dist;
            if (findNearestObstacle(new_path[i].x, new_path[i].y, min_dist_to_obs, is_occupied, ox, oy, dist)) {
                if (dist < min_dist_to_obs) {
                    double diff = dist - min_dist_to_obs;
                    double factor = (w_obs * diff) / dist;
                    gx += factor * (new_path[i].x - ox);
                    gy += factor * (new_path[i].y - oy);
                }
            }

            new_path[i].x += alpha * gx;
            new_path[i].y += alpha * gy;

            if (is_occupied(new_path[i].x, new_path[i].y)) {
                new_path[i] = path[i];
            }
        }
    }

    // 重新计算所有点的 theta (朝向)
    for (size_t i = 0; i < new_path.size() - 1; ++i) {
        new_path[i].theta = atan2(new_path[i+1].y - new_path[i].y, new_path[i+1].x - new_path[i].x);
    }
    new_path.back().theta = new_path[new_path.size()-2].theta;

    return new_path;
}

bool IntegratedSmoother::findNearestObstacle(double x, double y, double radius, 
                                            const std::function<bool(double, double)>& is_occupied,
                                            double& ox, double& oy, double& min_dist) {
    min_dist = radius + 1.0;
    bool found = false;
    double step = 0.1; // 采样精度，可根据地图分辨率调整

    for (double dx = -radius; dx <= radius; dx += step) {
        for (double dy = -radius; dy <= radius; dy += step) {
            if (dx*dx + dy*dy > radius*radius) continue;
            if (is_occupied(x + dx, y + dy)) {
                double d = sqrt(dx*dx + dy*dy);
                if (d < min_dist) {
                    min_dist = d;
                    ox = x + dx;
                    oy = y + dy;
                    found = true;
                }
            }
        }
    }
    return found;
}