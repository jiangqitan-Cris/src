#ifndef GLOBAL_PLANNER_SMOOTHER_HPP
#define GLOBAL_PLANNER_SMOOTHER_HPP

#include <vector>
#include <functional>
#include <cmath>
#include <iostream>
#include "algorithms/base_algorithm.hpp"
#include "math/cpprobotics_types.hpp"
#include "math/cubic_spline.hpp"

namespace global_planner {

/**
 * @brief 集成路径平滑器
 * 
 * 功能：
 * - 路径裁剪（移除冗余点）
 * - 路径加密（均匀化点间距）
 * - 分段智能平滑（直线保持直线，转弯处使用贝塞尔曲线）
 * - 全局样条重采样
 * - 曲率连续性保证
 */
class IntegratedSmoother {

public:
    using CollisionChecker = std::function<bool(double, double)>;
    
    IntegratedSmoother() = default;
    ~IntegratedSmoother() = default;
    
    /**
     * @brief 配置平滑器参数
     * @param config 平滑器配置
     */
    void configure(const SmootherConfig& config);
    
    /**
     * @brief 路径裁剪 - 移除不必要的中间点
     */
    std::vector<Pose2D> prunePath(const std::vector<Pose2D>& path, 
                                  const CollisionChecker& is_occupied);
    
    /**
     * @brief 路径加密 - 保证点之间的间距均匀
     */
    std::vector<Pose2D> fixDensity(const std::vector<Pose2D>& path, 
                                   double step_size = -1.0);  // -1 表示使用配置值
    
    /**
     * @brief 三次样条平滑 - 生成连续可微曲线
     */
    std::vector<Pose2D> splineSmooth(const std::vector<Pose2D>& path,
                                    const CollisionChecker& is_occupied,
                                    double sample_step = -1.0);
    
    /**
     * @brief 梯度下降后处理 - 优化平滑度和避障
     */
    std::vector<Pose2D> postProcess(const std::vector<Pose2D>& path,
                                    const CollisionChecker& is_occupied,
                                    double w_smooth = -1.0, 
                                    double w_data = -1.0, 
                                    double w_obs = -1.0, 
                                    double obs_dist = -1.0);
    
    /**
     * @brief 高斯平滑 - 消除高频噪声
     */
    std::vector<Pose2D> gaussianSmooth(const std::vector<Pose2D>& path,
                                       const CollisionChecker& is_occupied,
                                       int kernel_size = 5, 
                                       double sigma = 1.0);
    
    /**
     * @brief 分段智能平滑 - 直线区域保持直线，转弯区域才平滑
     */
    std::vector<Pose2D> segmentedSmooth(const std::vector<Pose2D>& path,
                                        const CollisionChecker& is_occupied,
                                        double sample_step = -1.0);
    
    /**
     * @brief 寻找最近障碍物
     */
    bool findNearestObstacle(double x, double y, double radius, 
                             const CollisionChecker& is_occupied,
                             double& ox, double& oy, double& min_dist);

private:
    // 内部辅助方法
    bool isLineClear(const Pose2D& p1, const Pose2D& p2,
                     const CollisionChecker& is_occupied);
    
    std::vector<Pose2D> linearInterpolate(const Pose2D& p1, const Pose2D& p2, double step);
    
    std::vector<Pose2D> smoothSegment(const std::vector<Pose2D>& segment, double sample_step);
    
    Pose2D cubicBezier(const Pose2D& p0, const Pose2D& p1, 
                       const Pose2D& p2, const Pose2D& p3, double t);
    
    std::vector<Pose2D> bezierCorner(const Pose2D& before, const Pose2D& corner, 
                                     const Pose2D& after, double radius, double step);
    
    void computeYawAndCurvature(std::vector<Pose2D>& path, double ds);

    // 配置参数
    SmootherConfig config_;
};

} // namespace global_planner

#endif // GLOBAL_PLANNER_SMOOTHER_HPP
