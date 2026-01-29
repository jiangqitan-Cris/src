#ifndef PLANNER_COMPONENT_HPP
#define PLANNER_COMPONENT_HPP

#include <memory>
#include <string>
#include <map>
#include "algorithms/base_algorithm.hpp"

namespace global_planner {

/**
 * @brief 规划器类型枚举
 */
enum class PlannerType {
    ASTAR,
    RRTCONNECT,
    DIJKSTRA,
    UNKNOWN,
};

/**
 * @brief PlannerComponent 类：负责算法的实例化、切换及管理
 * 
 * 使用工厂模式创建不同的规划算法实例
 */
class PlannerComponent {

public:
    PlannerComponent() = default;
    ~PlannerComponent() = default;

    /**
     * @brief 核心工厂方法：根据类型创建算法实例
     * @param type 算法类型枚举
     * @return 成功返回唯一指针，失败返回 nullptr
     */
    std::unique_ptr<BaseAlgorithm> create(PlannerType type);

    /**
     * @brief 辅助方法：将字符串转换为枚举
     * @param type_str 字符串 (如 "ASTAR")
     * @return 对应的枚举值
     */
    PlannerType stringToType(const std::string& type_str);

    /**
     * @brief 打印当前支持的所有算法列表
     */
    void printSupportList() const;

private:
    // 禁止拷贝，保证组件安全性
    PlannerComponent(const PlannerComponent&) = delete;
    PlannerComponent& operator=(const PlannerComponent&) = delete;
};

} // namespace global_planner

#endif // PLANNER_COMPONENT_HPP
