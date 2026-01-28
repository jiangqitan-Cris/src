#ifndef GLOBAL_PLANNER_SMOOTHER_HPP
#define GLOBAL_PLANNER_SMOOTHER_HPP

#include <vector>
#include <functional>
#include <cmath>
#include "algorithms/base_algorithm.hpp"
#include "utils/math/cpprobotics_types.hpp"
#include "utils/math/cubic_spline.hpp"

using namespace std;

class IntegratedSmoother {

public:
    IntegratedSmoother() = default;
    std::vector<Pose2D> prunePath(const std::vector<Pose2D>& path, 
        const std::function<bool(double, double)>& is_occupied);
    std::vector<Pose2D> fixDensity(const std::vector<Pose2D>& path, double step_size = 0.5);
    std::vector<Pose2D> smooth(const std::vector<Pose2D>& path,
                                const std::function<bool(double, double)>& is_occupied);
    std::vector<Pose2D> postProcess(const std::vector<Pose2D>& path,
                                    const std::function<bool(double, double)>& is_occupied,
                                    double w_smooth = 0.8, double w_data = 0.1, double w_obs = 0.1, 
                                    double obs_dist = 0.5);
    bool findNearestObstacle(double x, double y, double radius, 
                                const std::function<bool(double, double)>& is_occupied,
                                double& ox, double& oy, double& min_dist);

private:
    bool isLineClear(const Pose2D& p1, const Pose2D& p2,
                        const std::function<bool(double, double)>& is_occupied);

};

#endif // GLOBAL_PLANNER_SMOOTHER_HPP