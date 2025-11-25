#pragma once

#include <geometry_msgs/Pose.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mag_tracking_control {

/**
 * @brief 跟踪控制策略的输入信息
 */
struct TrackingControlInput {
    geometry_msgs::Pose current_sensor_pose;  ///< 当前传感器阵列位姿
    geometry_msgs::Pose magnet_pose;          ///< 磁铁的估计位姿
    std::vector<Eigen::Vector3d> field_strengths;  ///< 各传感器的磁场强度向量 (mT)
    ros::Time timestamp;                      ///< 时间戳
};

/**
 * @brief 跟踪控制策略的输出结果
 */
struct TrackingControlOutput {
    geometry_msgs::Pose target_pose;          ///< 目标传感器位姿
    bool is_valid;                            ///< 是否有效
    std::string message;                      ///< 状态消息
    double quality_score;                     ///< 质量评分（可选）
};

/**
 * @brief 跟踪控制策略基类
 * 
 * 所有跟踪控制算法都应继承此基类，实现统一的接口。
 * 主节点通过策略模式可以动态切换不同的控制算法。
 */
class TrackingControlStrategyBase {
public:
    virtual ~TrackingControlStrategyBase() = default;

    /**
     * @brief 初始化策略
     * @return 是否初始化成功
     */
    virtual bool initialize() = 0;

    /**
     * @brief 计算目标传感器位姿
     * 
     * 这是策略的核心方法，根据当前状态计算最优的传感器位姿。
     * 
     * @param input 输入信息（当前位姿、磁铁位姿、磁场数据等）
     * @param output 输出的目标位姿和控制信息
     * @return 是否成功计算出目标位姿
     */
    virtual bool computeTargetPose(
        const TrackingControlInput &input,
        TrackingControlOutput &output
    ) = 0;

    /**
     * @brief 获取策略名称
     * @return 策略类型名称（如 "fixed_offset", "adaptive_distance", "fisher_optimal"）
     */
    virtual std::string name() const = 0;

    /**
     * @brief 重置策略状态
     * 
     * 当切换策略或系统重启时调用，用于清理内部状态。
     */
    virtual void reset() {}

    /**
     * @brief 检查策略是否需要更新
     * 
     * 某些策略可能不需要每次都更新（例如固定偏移策略）。
     * 返回 false 时，控制节点可以跳过本次计算。
     * 
     * @return 是否需要更新目标位姿
     */
    virtual bool needsUpdate() const {
        return true;
    }

protected:
    bool initialized_ = false;  ///< 是否已初始化
};

}  // namespace mag_tracking_control

