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
 * 
 * 负责对原始磁场测量数据进行预处理，包括：
 * 1. 软/硬铁校准（soft/hard iron calibration）
 * 2. 低通滤波（抑制传感器噪声）
 * 
 * 注意：输入和输出数据的单位均为 mT（毫特斯拉）。
 */
class MagPreprocessor {
public:
  /**
   * @brief 构造函数
   * 初始化默认参数
   */
  MagPreprocessor();

  /**
   * @brief 配置预处理器参数
   * @param config 配置结构体，包含校准矩阵、偏移量、滤波参数等
   */
  void configure(const MagPoseEstimatorConfig &config);

  /**
   * @brief 处理磁场测量数据
   * @param msg 输入的磁场测量消息（单位：mT）
   * @return 处理后的磁场测量消息（单位：mT）
   * 
   * 处理流程：
   * 1. 如果启用校准：field = soft_iron_matrix * (field - hard_iron_offset)
   * 2. 如果启用滤波：使用指数平滑滤波器
   */
  sensor_msgs::MagneticField process(const sensor_msgs::MagneticField &msg);

private:
  Eigen::Matrix3d soft_iron_matrix_;  // 软铁校准矩阵（3×3）
  Eigen::Vector3d hard_iron_offset_;  // 硬铁偏移向量 [x, y, z] (mT)

  double low_pass_alpha_;  // 低通滤波器系数（0-1，越大越平滑）
  bool enable_filter_;  // 是否启用低通滤波器
  bool enable_calibration_;  // 是否启用软/硬铁校准

  bool filter_initialized_;  // 滤波器是否已初始化
  Eigen::Vector3d filtered_field_;  // 滤波后的磁场值（用于指数平滑）
};

}  // 命名空间 mag_pose_estimator
