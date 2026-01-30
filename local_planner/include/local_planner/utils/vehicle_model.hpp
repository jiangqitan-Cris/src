#ifndef LOCAL_PLANNER_VEHICLE_MODEL_HPP
#define LOCAL_PLANNER_VEHICLE_MODEL_HPP

/**
 * @file vehicle_model.hpp
 * @brief 车辆运动学模型
 * 
 * ============================================================================
 *                          自行车模型 (Bicycle Model)
 * ============================================================================
 * 
 * 自行车模型是车辆运动学的简化模型，将四轮车简化为两轮：
 * - 前轮：可转向
 * - 后轮：固定朝前
 * 
 *                    前轮 (δ = 前轮转角)
 *                      ○
 *                     /|
 *                    / |
 *                   /  | L (轴距)
 *                  /   |
 *                 /    |
 *                θ     |
 *               ○------○ 后轮 (参考点)
 *              (x, y)
 * 
 * 运动学方程（以后轴中心为参考点）：
 *   ẋ = v · cos(θ)
 *   ẏ = v · sin(θ)
 *   θ̇ = v · tan(δ) / L = v · κ
 *   v̇ = a
 * 
 * 其中：
 *   - (x, y): 后轴中心位置
 *   - θ: 航向角
 *   - v: 纵向速度
 *   - δ: 前轮转角
 *   - L: 轴距
 *   - κ = tan(δ)/L: 曲率
 *   - a: 纵向加速度
 * 
 * ============================================================================
 */

#include "local_planner/types.hpp"
#include <Eigen/Dense>

namespace local_planner {

/**
 * @brief 自行车运动学模型
 * 
 * 提供状态转移、雅可比矩阵计算等功能，是 iLQR 优化的基础。
 */
class BicycleModel {

public:
    /**
     * @brief 构造函数
     * @param params 车辆参数
     */
    explicit BicycleModel(const VehicleParams& params);
    
    /**
     * @brief 状态转移函数（离散时间）
     * 
     * 使用四阶龙格-库塔法 (RK4) 进行数值积分，精度更高。
     * 
     * @param state 当前状态
     * @param control 控制输入
     * @param dt 时间步长
     * @return 下一时刻状态
     */
    State forward(const State& state, const Control& control, double dt) const;
    
    /**
     * @brief 连续时间状态导数
     * 
     * 计算 ẋ = f(x, u)
     * 
     * @param state 当前状态
     * @param control 控制输入
     * @return 状态导数
     */
    State dynamics(const State& state, const Control& control) const;
    
    /**
     * @brief 计算状态转移的雅可比矩阵（关于状态）
     * 
     * A = ∂f/∂x
     * 
     * 用于 iLQR 的线性化。
     * 
     * @param state 当前状态
     * @param control 控制输入
     * @param dt 时间步长
     * @return 4x4 雅可比矩阵
     */
    Eigen::Matrix4d getStateJacobian(const State& state, const Control& control, double dt) const;
    
    /**
     * @brief 计算状态转移的雅可比矩阵（关于控制）
     * 
     * B = ∂f/∂u
     * 
     * @param state 当前状态
     * @param control 控制输入
     * @param dt 时间步长
     * @return 4x2 雅可比矩阵
     */
    Eigen::Matrix<double, 4, 2> getControlJacobian(const State& state, const Control& control, double dt) const;
    
    /**
     * @brief 将状态转换为向量形式
     * @param state 状态
     * @return [x, y, theta, v]
     */
    static Eigen::Vector4d stateToVector(const State& state);
    
    /**
     * @brief 将向量转换为状态
     * @param vec [x, y, theta, v]
     * @return 状态
     */
    static State vectorToState(const Eigen::Vector4d& vec);
    
    /**
     * @brief 检查状态是否满足约束
     */
    bool isStateValid(const State& state) const;
    
    /**
     * @brief 检查控制输入是否满足约束
     */
    bool isControlValid(const Control& control) const;
    
    /**
     * @brief 将控制输入裁剪到有效范围
     */
    Control clipControl(const Control& control) const;
    
    /**
     * @brief 获取车辆参数
     */
    const VehicleParams& getParams() const { return params_; }

private:
    VehicleParams params_;
};

} // namespace local_planner

#endif // LOCAL_PLANNER_VEHICLE_MODEL_HPP
