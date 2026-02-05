#ifndef LOCAL_PLANNER_VEHICLE_MODEL_HPP
#define LOCAL_PLANNER_VEHICLE_MODEL_HPP

/**
 * @file vehicle_model.hpp
 * @brief 车辆运动学模型和底盘模型定义
 * 
 * 本文件包含：
 * 1. 底盘模型类（ChassisModel 系列）- 在 types.hpp 中定义
 * 2. 自行车运动学模型（BicycleModel）- 用于状态转移和雅可比计算
 */

#include "local_planner/types.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace local_planner {

// ============================================================================
//                          自行车运动学模型
// ============================================================================

/**
 * @brief 自行车运动学模型
 * 
 * 用于：
 * - 状态转移预测
 * - 雅可比矩阵计算（用于 LQR/iLQR）
 * - 曲率约束检查
 */
class BicycleModel {
public:
    explicit BicycleModel(const VehicleParams& params);
    
    /** @brief 前向传播状态（使用 RK4 积分） */
    State forward(const State& state, const Control& control, double dt) const;
    
    /** @brief 连续时间动力学方程 */
    State dynamics(const State& state, const Control& control) const;
    
    /** @brief 状态雅可比矩阵 A = ∂f/∂x */
    Eigen::Matrix4d getStateJacobian(const State& state, const Control& control, double dt) const;
    
    /** @brief 控制雅可比矩阵 B = ∂f/∂u */
    Eigen::Matrix<double, 4, 2> getControlJacobian(const State& state, const Control& control, double dt) const;
    
    /** @brief 状态向量转换 */
    static Eigen::Vector4d stateToVector(const State& state);
    
    /** @brief 向量转状态 */
    static State vectorToState(const Eigen::Vector4d& vec);
    
    /** @brief 检查状态是否有效 */
    bool isStateValid(const State& state) const;
    
    /** @brief 检查控制是否有效 */
    bool isControlValid(const Control& control) const;
    
    /** @brief 裁剪控制量到有效范围 */
    Control clipControl(const Control& control) const;
    
    /** @brief 获取车辆参数 */
    const VehicleParams& getParams() const { return params_; }

private:
    VehicleParams params_;
};

} // namespace local_planner

#endif // LOCAL_PLANNER_VEHICLE_MODEL_HPP
