#pragma once

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace mag_pose_estimator {

// 前向声明配置结构体
struct MagPoseEstimatorConfig;

/**
 * @brief 磁场数据预处理器
 */
class MagPreprocessor {
public:
  MagPreprocessor();
  void configure(const MagPoseEstimatorConfig &config);
  sensor_msgs::MagneticField process(const sensor_msgs::MagneticField &msg);

private:
  Eigen::Matrix3d soft_iron_matrix_;  ///< 软铁校准矩阵（3×3）
  Eigen::Vector3d hard_iron_offset_;  ///< 硬铁偏移向量 [x, y, z] (mT)
  double low_pass_alpha_;  ///< 低通滤波器系数（0-1）
  bool enable_filter_;  ///< 是否启用低通滤波器
  bool enable_calibration_;  ///< 是否启用软/硬铁校准
  bool filter_initialized_;  ///< 滤波器是否已初始化
  Eigen::Vector3d filtered_field_;  ///< 滤波后的磁场值
};

}  // 命名空间 mag_pose_estimator
