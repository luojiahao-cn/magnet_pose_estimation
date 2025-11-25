#pragma once

#include "mag_tracking_control/tracking_control_strategy_base.h"
#include <Eigen/Dense>

namespace mag_tracking_control {

/**
 * @brief 固定偏移策略
 * 
 * 最简单的控制策略：保持传感器阵列相对于磁铁的固定偏移。
 * 适用于磁铁运动范围较小、传感器阵列跟随运动的场景。
 * 
 * 配置示例：
 * ```yaml
 * strategy:
 *   type: fixed_offset
 *   fixed_offset:
 *     offset: [0.0, 0.0, 0.05]  # 在磁铁上方5cm
 *     # 或者使用球坐标
 *     distance: 0.05
 *     angle_vertical: 0.0
 *     angle_horizontal: 0.0
 * ```
 */
class FixedOffsetStrategy : public TrackingControlStrategyBase {
public:
    FixedOffsetStrategy();
    virtual ~FixedOffsetStrategy() = default;

    /**
     * @brief 初始化策略
     * @param offset 相对于磁铁的固定偏移 [x, y, z] (米)
     * @param max_movement_per_step 每步最大移动距离 (米)，用于平滑运动
     * @return 是否初始化成功
     */
    bool initialize(const Eigen::Vector3d &offset, double max_movement_per_step);

    bool initialize() override {
        // 使用默认参数初始化
        Eigen::Vector3d default_offset(0.0, 0.0, 0.05);
        return initialize(default_offset, 0.02);
    }

    bool computeTargetPose(
        const TrackingControlInput &input,
        TrackingControlOutput &output
    ) override;

    std::string name() const override {
        return "fixed_offset";
    }

    /**
     * @brief 设置偏移量（运行时更新）
     */
    void setOffset(const Eigen::Vector3d &offset) {
        offset_ = offset;
    }

    /**
     * @brief 设置最大移动距离（运行时更新）
     */
    void setMaxMovementPerStep(double max_movement) {
        max_movement_per_step_ = max_movement;
    }

private:
    Eigen::Vector3d offset_;           ///< 相对于磁铁的固定偏移
    double max_movement_per_step_;     ///< 每步最大移动距离（平滑运动）
    
    /**
     * @brief 限制移动距离，实现平滑运动
     */
    void limitMovement(
        const geometry_msgs::Point &current_pos,
        geometry_msgs::Point &target_pos
    ) const;
    
    /**
     * @brief 计算两点之间的距离
     */
    double computeDistance(
        const geometry_msgs::Point &a,
        const geometry_msgs::Point &b
    ) const;
};

}  // namespace mag_tracking_control

