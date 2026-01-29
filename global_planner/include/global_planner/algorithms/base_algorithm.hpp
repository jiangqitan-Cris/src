#ifndef BASE_ALGORITHM_HPP
#define BASE_ALGORITHM_HPP

#include <vector>
#include <string>
#include <memory>
#include <cstdint>

namespace global_planner {

/**
 * @brief 规划器状态/错误码
 */
enum class PlannerStatus {
    SUCCESS = 0,           // 规划成功
    NO_MAP,                // 没有地图
    INVALID_START,         // 起点无效（在障碍物上或超出边界）
    INVALID_GOAL,          // 终点无效
    NO_PATH_FOUND,         // 找不到路径
    TIMEOUT,               // 规划超时
    CANCELLED,             // 被取消
    TF_ERROR,              // TF 变换错误
    UNKNOWN_ERROR          // 未知错误
};

/**
 * @brief 状态码转字符串
 */
inline std::string statusToString(PlannerStatus status) {
    switch (status) {
        case PlannerStatus::SUCCESS:        return "SUCCESS";
        case PlannerStatus::NO_MAP:         return "NO_MAP";
        case PlannerStatus::INVALID_START:  return "INVALID_START";
        case PlannerStatus::INVALID_GOAL:   return "INVALID_GOAL";
        case PlannerStatus::NO_PATH_FOUND:  return "NO_PATH_FOUND";
        case PlannerStatus::TIMEOUT:        return "TIMEOUT";
        case PlannerStatus::CANCELLED:      return "CANCELLED";
        case PlannerStatus::TF_ERROR:       return "TF_ERROR";
        default:                            return "UNKNOWN_ERROR";
    }
}

/**
 * @brief 2D 位姿结构体
 */
struct Pose2D {
    double x = 0.0;       // 位置 x (m)
    double y = 0.0;       // 位置 y (m)
    double theta = 0.0;   // 航向角 yaw (rad)
    double kappa = 0.0;   // 曲率 (1/m)，正值表示左转，负值表示右转
};

/**
 * @brief 栅格地图数据结构
 */
struct GridMapData {
    std::vector<int8_t> data;
    uint32_t width = 0;
    uint32_t height = 0;
    float resolution = 0.0f;
    double origin_x = 0.0;
    double origin_y = 0.0;
};

/**
 * @brief 规划器配置
 */
struct PlannerConfig {
    bool use_inflation = true;
    double inflation_radius = 0.3;
    double planning_timeout = 5.0;  // 规划超时时间 (s)
};

/**
 * @brief 平滑器配置
 */
struct SmootherConfig {
    double corner_radius = 0.5;         // 转弯圆角半径 (m)
    double sample_step = 0.05;          // 采样步长 (m)
    double density_step = 0.1;          // 路径加密步长 (m)
    int control_point_density = 30;     // 样条控制点数量
    int smooth_iterations = 5;          // 平滑迭代次数
    double smooth_weight = 0.5;         // 平滑权重
    double data_weight = 0.2;           // 数据保持权重
    double obstacle_weight = 0.3;       // 避障权重
    double obstacle_distance = 0.4;     // 避障安全距离 (m)
};

/**
 * @brief 规划结果
 */
struct PlanningResult {
    PlannerStatus status = PlannerStatus::UNKNOWN_ERROR;
    std::vector<Pose2D> path;
    double planning_time_ms = 0.0;      // 规划耗时 (ms)
    double path_length = 0.0;           // 路径长度 (m)
    std::string message;                // 额外信息
};

/**
 * @brief 算法基类
 */
class BaseAlgorithm {

public:
    virtual ~BaseAlgorithm() = default;

    /**
     * @brief 初始化算法
     * @param name 算法名称
     */
    virtual void initialize(const std::string& name) = 0;
    
    /**
     * @brief 配置算法参数
     * @param config 配置参数
     */
    virtual void configure(const PlannerConfig& config) = 0;
    
    /**
     * @brief 更新地图
     * @param map_data 地图数据
     */
    virtual void updateMap(const GridMapData& map_data) = 0;
    
    /**
     * @brief 执行路径规划
     * @param start 起点
     * @param goal 终点
     * @param path 输出路径
     * @return true 规划成功 / false 规划失败
     */
    virtual bool makePlan(const Pose2D& start, const Pose2D& goal,
                          std::vector<Pose2D>& path) = 0;
    
    /**
     * @brief 获取算法名称
     */
    virtual std::string getName() const = 0;
    
    /**
     * @brief 获取处理后的地图（如膨胀后）
     */
    virtual std::vector<int8_t> getProcessedMap() = 0;

protected:
    // 禁止拷贝
    BaseAlgorithm() = default;
    BaseAlgorithm(const BaseAlgorithm&) = delete;
    BaseAlgorithm& operator=(const BaseAlgorithm&) = delete;
};

} // namespace global_planner

#endif // BASE_ALGORITHM_HPP
