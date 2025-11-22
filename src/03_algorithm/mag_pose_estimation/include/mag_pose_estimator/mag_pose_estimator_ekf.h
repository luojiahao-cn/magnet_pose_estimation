#pragma once

#include <ros/time.h>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

/**
 * @brief EKF 姿态估计器
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
   * @brief 预测步骤
   */
  void predict();

  /**
   * @brief 更新步骤
   * @param measurement 磁场测量向量 (mT)
   */
  void correct(const Eigen::Vector3d &measurement);

  /**
   * @brief 计算姿态对观测的雅可比矩阵
   * @param q 当前姿态四元数
   * @param field_world 世界坐标系磁场向量 (mT)
   * @return 3×4 雅可比矩阵
   */
  Eigen::Matrix<double, 3, 4> orientationJacobian(const Eigen::Quaterniond &q, const Eigen::Vector3d &field_world) const;

  Eigen::Matrix<double, 10, 1> state_;  ///< 状态向量 [位置(3), 四元数(4), 偏置(3)]
  Eigen::Matrix<double, 10, 10> covariance_;  ///< 状态协方差矩阵
  ros::Time last_update_time_;  ///< 上次更新时间戳
};

}  // 命名空间 mag_pose_estimator
