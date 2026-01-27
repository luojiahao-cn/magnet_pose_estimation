#pragma once

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mag_sensor_calibration
{
  struct CalibrationParams;
}

namespace mag_pose_estimator
{

  // 前向声明配置结构体
  struct MagPoseEstimatorConfig;

  /**
   * @brief 磁场数据预处理器
   */
  class MagPreprocessor
  {
  public:
    MagPreprocessor();
    void configure(const MagPoseEstimatorConfig &config);

    /**
     * @brief 处理单个传感器数据（使用全局校正参数）
     */
    sensor_msgs::MagneticField process(const sensor_msgs::MagneticField &msg);

    /**
     * @brief 处理传感器数据（使用指定传感器ID的校正参数）
     * @param msg 磁场消息
     * @param sensor_id 传感器ID
     */
    sensor_msgs::MagneticField process(const sensor_msgs::MagneticField &msg, uint32_t sensor_id);

    /**
     * @brief 加载多传感器校正参数
     * @param params 校正参数
     */
    void loadMultiSensorCalibration(const mag_sensor_calibration::CalibrationParams &params);

  private:
    Eigen::Matrix3d soft_iron_matrix_; ///< 软铁校准矩阵（3×3）（全局/默认）
    Eigen::Vector3d hard_iron_offset_; ///< 硬铁偏移向量 [x, y, z] (mT)（全局/默认）
    double low_pass_alpha_;            ///< 低通滤波器系数（0-1）
    bool enable_filter_;               ///< 是否启用低通滤波器
    bool enable_calibration_;          ///< 是否启用软/硬铁校准
    bool filter_initialized_;          ///< 滤波器是否已初始化
    Eigen::Vector3d filtered_field_;   ///< 滤波后的磁场值

    // 多传感器校正参数
    std::map<uint32_t, Eigen::Matrix3d> sensor_soft_iron_matrices_; ///< 传感器ID -> 软铁矩阵
    std::map<uint32_t, Eigen::Vector3d> sensor_hard_iron_offsets_;  ///< 传感器ID -> 硬铁偏移
    bool use_multi_sensor_calibration_;                             ///< 是否使用多传感器校正
  };

} // 命名空间 mag_pose_estimator
