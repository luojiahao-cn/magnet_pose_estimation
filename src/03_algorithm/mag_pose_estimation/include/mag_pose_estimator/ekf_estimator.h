#pragma once

#include <ros/time.h>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

/**
 * @brief 扩展卡尔曼滤波器（EKF）姿态估计器
 * 
 * 使用 EKF 同时估计磁铁的位置、姿态（四元数）和磁场偏置。
 * 
 * 状态向量（10 维）：
 *   [px, py, pz, qx, qy, qz, qw, bx, by, bz]^T
 *   - px, py, pz: 位置 [x, y, z] (米)
 *   - qx, qy, qz, qw: 姿态四元数（归一化）
 *   - bx, by, bz: 磁场偏置向量 (mT)
 * 
 * 观测模型：
 *   基于归一化的磁场方向，通过旋转世界坐标系磁场向量得到预测值。
 *   残差 = normalize(测量) - normalize(预测)
 * 
 * 注意：测量数据单位应为 mT（毫特斯拉），但 EKF 只使用归一化方向，单位不影响结果。
 */
class EKFEstimator : public EstimatorBase {
public:
  EKFEstimator();

  void initialize() override;
  void update(const sensor_msgs::MagneticField &mag) override;
  geometry_msgs::Pose getPose() const override;
  std::string name() const override { return "ekf"; }

private:
  /**
   * @brief 预测步骤（当前实现为随机游走模型）
   * TODO: 可扩展融合 IMU 线加速度以改进位置预测
   */
  void predict();

  /**
   * @brief 更新步骤（观测更新）
   * @param measurement 磁场测量向量 (mT)
   * 
   * 使用归一化的磁场方向进行更新，通过 EKF 标准公式修正状态和协方差。
   */
  void correct(const Eigen::Vector3d &measurement);

  /**
   * @brief 计算姿态对观测的雅可比矩阵
   * @param q 当前姿态四元数
   * @param field_world 世界坐标系下的磁场向量 (mT)
   * @return 3×4 雅可比矩阵，表示四元数扰动对归一化磁场方向的影响
   */
  Eigen::Matrix<double, 3, 4> orientationJacobian(const Eigen::Quaterniond &q, const Eigen::Vector3d &field_world) const;

  Eigen::Matrix<double, 10, 1> state_;  // 状态向量 [位置(3), 四元数(4), 偏置(3)]
  Eigen::Matrix<double, 10, 10> covariance_;  // 状态协方差矩阵

  ros::Time last_update_time_;  // 上次更新时间戳（用于计算时间间隔）
};

}  // 命名空间 mag_pose_estimator
