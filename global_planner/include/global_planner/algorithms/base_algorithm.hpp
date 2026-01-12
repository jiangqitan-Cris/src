#ifndef BASE_ALGORITHM_HPP
#define BASE_ALGORITHM_HPP

#include <vector>
#include <string>
#include <memory>

using namespace std;

struct Pose2D {
    double x;
    double y;
    double theta;
};

struct GridMapData {
    std::vector<int8_t> data;
    uint32_t width;
    uint32_t height;
    float resolution;
    double origin_x;
    double origin_y;
};

struct PlannerConfig {
    bool use_inflation = true;
    double inflation_radius = 0.1;
};

class BaseAlgorithm {

public:
    virtual ~BaseAlgorithm() = default;

    /**
     * @brief 统一规划接口
     * @param start 起点
     * @param goal  终点
     * @param map_ptr 这里的地图采用智能指针，可以是栅格地图，也可以是点云
     * @param path  输出的结果路径
     * @return true 规划成功 / false 规划失败
     */
    virtual bool makePlan(const Pose2D& start, const Pose2D& goal,
                            vector<Pose2D>& path) = 0;
    virtual void updateMap(const GridMapData& map_data) = 0;
    virtual void configure(const PlannerConfig& config) = 0;
    virtual void initialize(const std::string& name) = 0;
    virtual string getName() const = 0;
    virtual vector<int8_t> getProcessedMap() = 0;

};

#endif