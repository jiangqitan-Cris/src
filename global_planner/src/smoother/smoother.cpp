#include "global_planner/smoother/smoother.hpp"

namespace global_planner {

void IntegratedSmoother::configure(const SmootherConfig& config) {
    config_ = config;
}

bool IntegratedSmoother::isLineClear(const Pose2D& p1, const Pose2D& p2, 
                                     const CollisionChecker& is_occupied) {
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    int steps = std::max(static_cast<int>(dist / 0.05), 2); 
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        if (is_occupied(p1.x + t*(p2.x-p1.x), p1.y + t*(p2.y-p1.y))) return false;
    }
    return true;
}

std::vector<Pose2D> IntegratedSmoother::prunePath(const std::vector<Pose2D>& path,
                                                  const CollisionChecker& is_occupied) {
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
    
    // 使用配置值如果未指定
    double actual_step = (step_size < 0) ? config_.density_step : step_size;

    std::vector<Pose2D> dense_path;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const auto& p1 = path[i];
        const auto& p2 = path[i + 1];

        double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
        dense_path.push_back(p1);

        if (dist > actual_step) {
            int num_steps = static_cast<int>(dist / actual_step);
            for (int j = 1; j < num_steps; ++j) {
                double t = static_cast<double>(j) / num_steps;
                Pose2D interpolated_point;
                interpolated_point.x = p1.x + t * (p2.x - p1.x);
                interpolated_point.y = p1.y + t * (p2.y - p1.y);
                dense_path.push_back(interpolated_point);
            }
        }
    }
    
    dense_path.push_back(path.back());
    return dense_path;
}

std::vector<Pose2D> IntegratedSmoother::smooth(const std::vector<Pose2D>& path,
                                                const CollisionChecker& is_occupied,
                                                double sample_step) {
    if (path.size() < 2) {
        return path;
    }
    
    double actual_step = (sample_step < 0) ? config_.sample_step : sample_step;
    
    std::vector<double> ctrl_x, ctrl_y;
    for (const auto& p : path) {
        ctrl_x.push_back(p.x);
        ctrl_y.push_back(p.y);
    }

    bool has_collision = true;
    int max_iterations = 20;
    int iter = 0;

    while (has_collision && iter < max_iterations) {
        has_collision = false;
        cpprobotics::Spline2D spline(ctrl_x, ctrl_y);

        std::vector<Pose2D> current_smooth_path;
        double collision_s = -1.0;

        for (double s = 0; s <= spline.s.back(); s += actual_step) {
            cpprobotics::Poi_d pos = spline.calc_position(s);
            if (is_occupied(pos[0], pos[1])) {
                has_collision = true;
                collision_s = s;
                break;
            }

            Pose2D temp_pose;
            temp_pose.x = pos[0];
            temp_pose.y = pos[1];
            temp_pose.theta = spline.calc_yaw(s);
            temp_pose.kappa = spline.calc_curvature(s);
            current_smooth_path.push_back(temp_pose);
        }

        if (has_collision) {
            auto it = std::lower_bound(spline.s.begin(), spline.s.end(), collision_s);
            int idx = static_cast<int>(std::distance(spline.s.begin(), it));

            if (idx > 0 && idx < static_cast<int>(ctrl_x.size())) {
                double insert_x = (ctrl_x[idx] + ctrl_x[idx - 1]) / 2;
                double insert_y = (ctrl_y[idx] + ctrl_y[idx - 1]) / 2;
                ctrl_x.insert(ctrl_x.begin() + idx, insert_x);
                ctrl_y.insert(ctrl_y.begin() + idx, insert_y);
            }
            iter++;
        } else {
            return current_smooth_path;
        }
    }

    std::cout << "Cannot find a cubic spline with no collision" << std::endl;
    return path;
}

std::vector<Pose2D> IntegratedSmoother::postProcess(const std::vector<Pose2D>& path,
                                              const CollisionChecker& is_occupied,
                                              double w_smooth, double w_data, double w_obs, double obs_dist) {
    if (path.size() < 3) return path;
    
    // 使用配置值如果未指定
    double actual_w_smooth = (w_smooth < 0) ? config_.smooth_weight : w_smooth;
    double actual_w_data = (w_data < 0) ? config_.data_weight : w_data;
    double actual_w_obs = (w_obs < 0) ? config_.obstacle_weight : w_obs;
    double actual_obs_dist = (obs_dist < 0) ? config_.obstacle_distance : obs_dist;
    
    std::cout << "Start post process smooth path" << std::endl;

    std::vector<Pose2D> new_path = path;
    int max_iters = 500;
    double alpha = 0.1;
    double tolerance = 1e-6;

    for (int iter = 0; iter < max_iters; iter++) {
        double max_change = 0.0;
        
        for (size_t i = 1; i < new_path.size() - 1; i++) {
            double gx = 0, gy = 0;

            gx += actual_w_smooth * (new_path[i-1].x + new_path[i+1].x - 2.0 * new_path[i].x);
            gy += actual_w_smooth * (new_path[i-1].y + new_path[i+1].y - 2.0 * new_path[i].y);

            gx += actual_w_data * (path[i].x - new_path[i].x);
            gy += actual_w_data * (path[i].y - new_path[i].y);

            double ox, oy, dist;
            if (findNearestObstacle(new_path[i].x, new_path[i].y, actual_obs_dist, is_occupied, ox, oy, dist)) {
                if (dist < actual_obs_dist && dist > 0.01) {
                    double diff = actual_obs_dist - dist;
                    double factor = actual_w_obs * diff / dist;
                    gx += factor * (new_path[i].x - ox);
                    gy += factor * (new_path[i].y - oy);
                }
            }

            double dx = alpha * gx;
            double dy = alpha * gy;
            
            double new_x = new_path[i].x + dx;
            double new_y = new_path[i].y + dy;

            if (!is_occupied(new_x, new_y)) {
                new_path[i].x = new_x;
                new_path[i].y = new_y;
                max_change = std::max(max_change, std::hypot(dx, dy));
            }
        }
        
        if (max_change < tolerance) {
            std::cout << "PostProcess converged at iteration " << iter << std::endl;
            break;
        }
    }

    for (size_t i = 0; i < new_path.size() - 1; ++i) {
        new_path[i].theta = std::atan2(new_path[i+1].y - new_path[i].y, new_path[i+1].x - new_path[i].x);
    }
    new_path.back().theta = new_path[new_path.size()-2].theta;

    return new_path;
}

bool IntegratedSmoother::findNearestObstacle(double x, double y, double radius, 
                                            const CollisionChecker& is_occupied,
                                            double& ox, double& oy, double& min_dist) {
    min_dist = radius + 1.0;
    bool found = false;
    double step = 0.1;

    for (double dx = -radius; dx <= radius; dx += step) {
        for (double dy = -radius; dy <= radius; dy += step) {
            if (dx*dx + dy*dy > radius*radius) continue;
            if (is_occupied(x + dx, y + dy)) {
                double d = std::sqrt(dx*dx + dy*dy);
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

std::vector<Pose2D> IntegratedSmoother::gaussianSmooth(const std::vector<Pose2D>& path,
                                                        const CollisionChecker& is_occupied,
                                                        int kernel_size, double sigma) {
    if (path.size() < 3) return path;
    
    std::vector<double> kernel(kernel_size);
    double sum = 0.0;
    int half = kernel_size / 2;
    
    for (int i = 0; i < kernel_size; ++i) {
        double x = i - half;
        kernel[i] = std::exp(-x * x / (2 * sigma * sigma));
        sum += kernel[i];
    }
    for (int i = 0; i < kernel_size; ++i) {
        kernel[i] /= sum;
    }
    
    std::vector<Pose2D> smoothed_path = path;
    
    for (size_t i = static_cast<size_t>(half); i < path.size() - static_cast<size_t>(half); ++i) {
        double new_x = 0.0, new_y = 0.0;
        
        for (int k = 0; k < kernel_size; ++k) {
            int idx = static_cast<int>(i) - half + k;
            new_x += kernel[k] * path[idx].x;
            new_y += kernel[k] * path[idx].y;
        }
        
        if (!is_occupied(new_x, new_y)) {
            smoothed_path[i].x = new_x;
            smoothed_path[i].y = new_y;
        }
    }
    
    for (size_t i = 0; i < smoothed_path.size() - 1; ++i) {
        smoothed_path[i].theta = std::atan2(smoothed_path[i+1].y - smoothed_path[i].y, 
                                            smoothed_path[i+1].x - smoothed_path[i].x);
    }
    smoothed_path.back().theta = smoothed_path[smoothed_path.size()-2].theta;
    
    return smoothed_path;
}

void IntegratedSmoother::computeYawAndCurvature(std::vector<Pose2D>& path, double ds) {
    if (path.size() < 3) {
        for (size_t i = 0; i < path.size() - 1; ++i) {
            path[i].theta = std::atan2(path[i+1].y - path[i].y, path[i+1].x - path[i].x);
            path[i].kappa = 0.0;
        }
        if (path.size() >= 2) {
            path.back().theta = path[path.size()-2].theta;
            path.back().kappa = 0.0;
        }
        return;
    }
    
    for (size_t i = 0; i < path.size(); ++i) {
        double dx, dy, d2x, d2y;
        
        if (i == 0) {
            dx = (path[i+1].x - path[i].x) / ds;
            dy = (path[i+1].y - path[i].y) / ds;
            if (path.size() > 2) {
                d2x = (path[i+2].x - 2*path[i+1].x + path[i].x) / (ds * ds);
                d2y = (path[i+2].y - 2*path[i+1].y + path[i].y) / (ds * ds);
            } else {
                d2x = d2y = 0;
            }
        } else if (i == path.size() - 1) {
            dx = (path[i].x - path[i-1].x) / ds;
            dy = (path[i].y - path[i-1].y) / ds;
            if (path.size() > 2) {
                d2x = (path[i].x - 2*path[i-1].x + path[i-2].x) / (ds * ds);
                d2y = (path[i].y - 2*path[i-1].y + path[i-2].y) / (ds * ds);
            } else {
                d2x = d2y = 0;
            }
        } else {
            dx = (path[i+1].x - path[i-1].x) / (2 * ds);
            dy = (path[i+1].y - path[i-1].y) / (2 * ds);
            d2x = (path[i+1].x - 2*path[i].x + path[i-1].x) / (ds * ds);
            d2y = (path[i+1].y - 2*path[i].y + path[i-1].y) / (ds * ds);
        }
        
        path[i].theta = std::atan2(dy, dx);
        
        double denom = std::pow(dx*dx + dy*dy, 1.5);
        if (denom > 1e-6) {
            path[i].kappa = (dx * d2y - dy * d2x) / denom;
        } else {
            path[i].kappa = 0.0;
        }
    }
}

std::vector<Pose2D> IntegratedSmoother::linearInterpolate(const Pose2D& p1, const Pose2D& p2, double step) {
    std::vector<Pose2D> result;
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    int num_points = std::max(static_cast<int>(dist / step), 1);
    double theta = std::atan2(p2.y - p1.y, p2.x - p1.x);
    
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        Pose2D p;
        p.x = p1.x + t * (p2.x - p1.x);
        p.y = p1.y + t * (p2.y - p1.y);
        p.theta = theta;
        p.kappa = 0.0;
        result.push_back(p);
    }
    return result;
}

std::vector<Pose2D> IntegratedSmoother::smoothSegment(const std::vector<Pose2D>& segment, double sample_step) {
    if (segment.size() < 3) {
        if (segment.size() == 2) {
            return linearInterpolate(segment.front(), segment.back(), sample_step);
        }
        return segment;
    }
    
    std::vector<double> ctrl_x, ctrl_y;
    for (const auto& p : segment) {
        ctrl_x.push_back(p.x);
        ctrl_y.push_back(p.y);
    }
    
    cpprobotics::Spline2D spline(ctrl_x, ctrl_y);
    std::vector<Pose2D> smooth_path;
    
    for (double s = 0; s <= spline.s.back(); s += sample_step) {
        cpprobotics::Poi_d pos = spline.calc_position(s);
        Pose2D p;
        p.x = pos[0];
        p.y = pos[1];
        p.theta = spline.calc_yaw(s);
        p.kappa = spline.calc_curvature(s);
        smooth_path.push_back(p);
    }
    
    return smooth_path;
}

Pose2D IntegratedSmoother::cubicBezier(const Pose2D& p0, const Pose2D& p1, 
                                        const Pose2D& p2, const Pose2D& p3, double t) {
    double u = 1.0 - t;
    double tt = t * t;
    double uu = u * u;
    double uuu = uu * u;
    double ttt = tt * t;
    
    Pose2D result;
    result.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
    result.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
    
    double dx = 3 * uu * (p1.x - p0.x) + 6 * u * t * (p2.x - p1.x) + 3 * tt * (p3.x - p2.x);
    double dy = 3 * uu * (p1.y - p0.y) + 6 * u * t * (p2.y - p1.y) + 3 * tt * (p3.y - p2.y);
    result.theta = std::atan2(dy, dx);
    
    double d2x = 6 * u * (p2.x - 2*p1.x + p0.x) + 6 * t * (p3.x - 2*p2.x + p1.x);
    double d2y = 6 * u * (p2.y - 2*p1.y + p0.y) + 6 * t * (p3.y - 2*p2.y + p1.y);
    
    double denom = std::pow(dx*dx + dy*dy, 1.5);
    if (denom > 1e-6) {
        result.kappa = (dx * d2y - dy * d2x) / denom;
    } else {
        result.kappa = 0.0;
    }
    
    return result;
}

std::vector<Pose2D> IntegratedSmoother::bezierCorner(const Pose2D& before, const Pose2D& corner, 
                                                      const Pose2D& after, double radius, double step) {
    std::vector<Pose2D> result;
    
    double dx1 = corner.x - before.x;
    double dy1 = corner.y - before.y;
    double len1 = std::hypot(dx1, dy1);
    
    double dx2 = after.x - corner.x;
    double dy2 = after.y - corner.y;
    double len2 = std::hypot(dx2, dy2);
    
    if (len1 < 0.01 || len2 < 0.01) {
        result.push_back(corner);
        return result;
    }
    
    dx1 /= len1; dy1 /= len1;
    dx2 /= len2; dy2 /= len2;
    
    double actual_radius = std::min(radius, std::min(len1 * 0.4, len2 * 0.4));
    
    Pose2D p0, p3;
    p0.x = corner.x - dx1 * actual_radius;
    p0.y = corner.y - dy1 * actual_radius;
    p3.x = corner.x + dx2 * actual_radius;
    p3.y = corner.y + dy2 * actual_radius;
    
    double ctrl_dist = actual_radius * 0.55;
    Pose2D p1, p2;
    p1.x = p0.x + dx1 * ctrl_dist;
    p1.y = p0.y + dy1 * ctrl_dist;
    p2.x = p3.x - dx2 * ctrl_dist;
    p2.y = p3.y - dy2 * ctrl_dist;
    
    double curve_length = actual_radius * 1.5;
    int num_samples = std::max(static_cast<int>(curve_length / step), 5);
    
    for (int i = 0; i <= num_samples; ++i) {
        double t = static_cast<double>(i) / num_samples;
        result.push_back(cubicBezier(p0, p1, p2, p3, t));
    }
    
    return result;
}

std::vector<Pose2D> IntegratedSmoother::segmentedSmooth(const std::vector<Pose2D>& path,
                                                         const CollisionChecker& is_occupied,
                                                         double sample_step) {
    if (path.size() < 2) return path;
    
    double actual_step = (sample_step < 0) ? config_.sample_step : sample_step;
    double corner_radius = config_.corner_radius;
    int ctrl_density = config_.control_point_density;
    
    // Step 1: 路径裁剪，找到关键转折点
    std::vector<Pose2D> key_points;
    key_points.push_back(path.front());
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        if (!isLineClear(key_points.back(), path[i + 1], is_occupied)) {
            key_points.push_back(path[i]);
        }
    }
    key_points.push_back(path.back());
    
    std::cout << "Segmented smooth: found " << key_points.size() << " key points" << std::endl;
    
    if (key_points.size() < 2) return path;
    if (key_points.size() == 2) {
        return linearInterpolate(key_points[0], key_points[1], actual_step);
    }
    
    // Step 2: 使用贝塞尔圆角连接各个关键点
    std::vector<Pose2D> final_path;
    
    for (size_t i = 0; i < key_points.size(); ++i) {
        if (i == 0) {
            // Skip, handled with first corner
        } else if (i == key_points.size() - 1) {
            if (!final_path.empty()) {
                auto line = linearInterpolate(final_path.back(), key_points[i], actual_step);
                for (size_t j = 1; j < line.size(); ++j) {
                    if (!is_occupied(line[j].x, line[j].y)) {
                        final_path.push_back(line[j]);
                    }
                }
            } else {
                final_path.push_back(key_points[i]);
            }
        } else {
            const Pose2D& before = key_points[i - 1];
            const Pose2D& corner = key_points[i];
            const Pose2D& after = key_points[i + 1];
            
            auto bezier_curve = bezierCorner(before, corner, after, corner_radius, actual_step);
            
            if (bezier_curve.empty()) continue;
            
            Pose2D line_end = bezier_curve.front();
            if (i == 1) {
                auto line = linearInterpolate(key_points[0], line_end, actual_step);
                for (size_t j = 0; j < line.size() - 1; ++j) {
                    if (!is_occupied(line[j].x, line[j].y)) {
                        final_path.push_back(line[j]);
                    }
                }
            } else if (!final_path.empty()) {
                auto line = linearInterpolate(final_path.back(), line_end, actual_step);
                for (size_t j = 1; j < line.size() - 1; ++j) {
                    if (!is_occupied(line[j].x, line[j].y)) {
                        final_path.push_back(line[j]);
                    }
                }
            }
            
            for (const auto& p : bezier_curve) {
                if (!is_occupied(p.x, p.y)) {
                    final_path.push_back(p);
                }
            }
        }
    }
    
    // Step 3: 全局样条重采样
    if (final_path.size() > 5) {
        std::vector<Pose2D> control_points;
        int skip = std::max(1, static_cast<int>(final_path.size()) / ctrl_density);
        
        for (size_t i = 0; i < final_path.size(); i += skip) {
            control_points.push_back(final_path[i]);
        }
        if (control_points.back().x != final_path.back().x || 
            control_points.back().y != final_path.back().y) {
            control_points.push_back(final_path.back());
        }
        
        if (control_points.size() >= 3) {
            std::vector<double> ctrl_x, ctrl_y;
            for (const auto& p : control_points) {
                ctrl_x.push_back(p.x);
                ctrl_y.push_back(p.y);
            }
            
            try {
                cpprobotics::Spline2D spline(ctrl_x, ctrl_y);
                std::vector<Pose2D> smooth_path;
                
                double total_length = spline.s.back();
                for (double s = 0; s <= total_length; s += actual_step) {
                    cpprobotics::Poi_d pos = spline.calc_position(s);
                    
                    if (!is_occupied(pos[0], pos[1])) {
                        Pose2D p;
                        p.x = pos[0];
                        p.y = pos[1];
                        p.theta = spline.calc_yaw(s);
                        p.kappa = spline.calc_curvature(s);
                        smooth_path.push_back(p);
                    }
                }
                
                if (!smooth_path.empty()) {
                    cpprobotics::Poi_d end_pos = spline.calc_position(total_length);
                    if (std::hypot(smooth_path.back().x - end_pos[0], 
                                   smooth_path.back().y - end_pos[1]) > actual_step * 0.5) {
                        Pose2D end_point;
                        end_point.x = end_pos[0];
                        end_point.y = end_pos[1];
                        end_point.theta = spline.calc_yaw(total_length);
                        end_point.kappa = spline.calc_curvature(total_length);
                        smooth_path.push_back(end_point);
                    }
                    
                    std::cout << "Spline smoothing successful, path size: " << smooth_path.size() << std::endl;
                    return smooth_path;
                }
            } catch (const std::exception& e) {
                std::cout << "Spline fitting failed: " << e.what() << std::endl;
            } catch (...) {
                std::cout << "Spline fitting failed" << std::endl;
            }
        }
        
        // Fallback: iterative Gaussian smoothing
        std::vector<Pose2D> smoothed = final_path;
        int num_iterations = config_.smooth_iterations;
        int smooth_radius = 3;
        
        for (int iter = 0; iter < num_iterations; ++iter) {
            std::vector<Pose2D> temp = smoothed;
            
            for (size_t i = static_cast<size_t>(smooth_radius); i < smoothed.size() - static_cast<size_t>(smooth_radius); ++i) {
                double sum_x = 0, sum_y = 0, total_weight = 0;
                
                for (int k = -smooth_radius; k <= smooth_radius; ++k) {
                    double weight = std::exp(-0.5 * k * k / 4.0);
                    sum_x += weight * smoothed[i + k].x;
                    sum_y += weight * smoothed[i + k].y;
                    total_weight += weight;
                }
                
                double new_x = sum_x / total_weight;
                double new_y = sum_y / total_weight;
                
                if (!is_occupied(new_x, new_y)) {
                    temp[i].x = new_x;
                    temp[i].y = new_y;
                }
            }
            smoothed = temp;
        }
        
        computeYawAndCurvature(smoothed, actual_step);
        return smoothed;
    }
    
    return final_path;
}

} // namespace global_planner
