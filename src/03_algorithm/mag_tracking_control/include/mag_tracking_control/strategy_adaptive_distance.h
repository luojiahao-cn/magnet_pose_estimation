#pragma once

#include "mag_tracking_control/tracking_control_strategy_base.h"
#include <Eigen/Dense>

namespace mag_tracking_control {

/**
 * @brief 自适应距离策略
 * 
 * 根据磁场强度自动调整传感器与磁铁的距离，保持磁场在最佳测量范围。
 * 
 * 配置示例：
 * ```yaml
 * strategy:
 *   type: adaptive_distance
 *   adaptive_distance:
 *     target_field_strength: 35.0  # mT
 *     min_field_strength: 20.0
 *     max_field_strength: 50.0
 *     adjustment_gain: 0.02
 *     max_movement_per_step: 0.02
 * ```
 */
class AdaptiveDistanceStrategy : public TrackingControlStrategyBase {
public:
    AdaptiveDistanceStrategy();
    virtual ~AdaptiveDistanceStrategy() = default;

    /**
     * @brief 初始化策略
     * @param target_field_strength 目标磁场强度 (mT)
     * @param min_field_strength 最小磁场强度 (mT)
     * @param max_field_strength 最大磁场强度 (mT)
     * @param adjustment_gain 调整增益系数
     * @param max_movement_per_step 每步最大移动距离 (米)
     * @return 是否初始化成功
     */
    bool initialize(
        double target_field_strength,
        double min_field_strength,
        double max_field_strength,
        double adjustment_gain,
        double max_movement_per_step
    );

    bool initialize() override {
        return initialize(35.0, 20.0, 50.0, 0.02, 0.02);
    }

    bool computeTargetPose(
        const TrackingControlInput &input,
        TrackingControlOutput &output
    ) override;

    std::string name() const override {
        return "adaptive_distance";
    }

private:
    double target_field_strength_;    ///< 目标磁场强度 (mT)
    double min_field_strength_;       ///< 最小磁场强度 (mT)
    double max_field_strength_;       ///< 最大磁场强度 (mT)
    double adjustment_gain_;          ///< 调整增益系数
    double max_movement_per_step_;    ///< 每步最大移动距离 (米)

    /**
     * @brief 计算平均磁场强度
     */
    double computeAverageFieldStrength(const std::vector<Eigen::Vector3d> &fields) const;

    /**
     * @brief 计算从传感器到磁铁的单位方向向量
     */
    Eigen::Vector3d computeDirectionToMagnet(
        const geometry_msgs::Point &sensor_pos,
        const geometry_msgs::Point &magnet_pos
    ) const;

    /**
     * @brief 限制移动距离
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

