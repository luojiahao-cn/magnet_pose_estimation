#include "mag_pose_estimator/estimator_ekf.h"

#include <Eigen/Geometry>
#include <ros/ros.h>

namespace mag_pose_estimator {

namespace {
Eigen::Quaterniond normalize(const Eigen::Quaterniond &q) {
  Eigen::Quaterniond copy = q;
  copy.normalize();
  return copy;
}

Eigen::Vector3d normalizeVec(const Eigen::Vector3d &v) {
  double norm = v.norm();
  if (norm < 1e-9) {
    return Eigen::Vector3d::UnitX();
  }
  return v / norm;
}
}

/**
 * @brief 构造函数
 */
EKFEstimator::EKFEstimator() {
  state_.setZero();
  state_(6) = 1.0;
  covariance_.setIdentity();
}

/**
 * @brief 初始化估计器
 */
void EKFEstimator::initialize() {
  covariance_.setIdentity();
  covariance_ *= 0.1;
  last_update_time_ = ros::Time(0);
  initialized_ = true;
}

/**
 * @brief 更新估计器状态
 * @param mag 磁场测量数据 (mT)
 */
void EKFEstimator::update(const sensor_msgs::MagneticField &mag) {
  if (!initialized_) {
    initialize();
  }

  ros::Time stamp = mag.header.stamp;
  if (!stamp.isZero() && !last_update_time_.isZero()) {
    double dt = (stamp - last_update_time_).toSec();
    if (dt > 0.0) {
      double q_pos = config_.ekf.process_noise_position;
      double q_ori = config_.ekf.process_noise_orientation;
      for (int i = 0; i < 3; ++i) {
        covariance_(i, i) += q_pos * dt;
      }
      for (int i = 0; i < 4; ++i) {
        covariance_(3 + i, 3 + i) += q_ori * dt;
      }
    }
  }
  last_update_time_ = stamp;

  Eigen::Vector3d measurement(mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
  correct(measurement);
}

/**
 * @brief 预测步骤
 */
void EKFEstimator::predict() {
}

/**
 * @brief 观测更新步骤
 * @param measurement 磁场测量向量 (mT)
 */
void EKFEstimator::correct(const Eigen::Vector3d &measurement) {
  Eigen::Quaterniond q(state_(6), state_(3), state_(4), state_(5));
  q = normalize(q);
  state_(3) = q.x();
  state_(4) = q.y();
  state_(5) = q.z();
  state_(6) = q.w();

  Eigen::Vector3d predicted = q * config_.ekf.world_field;
  Eigen::Vector3d residual = normalizeVec(measurement) - normalizeVec(predicted);

  Eigen::Matrix<double, 3, 10> H;
  H.setZero();
  Eigen::Matrix<double, 3, 4> dq_dq = orientationJacobian(q, config_.ekf.world_field);
  H.block<3, 4>(0, 3) = dq_dq;

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * config_.ekf.measurement_noise;
  Eigen::Matrix<double, 10, 3> K = covariance_ * H.transpose() * (H * covariance_ * H.transpose() + R).inverse();

  state_ += K * residual;
  covariance_ = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * covariance_;

  Eigen::Quaterniond q_update(state_(6), state_(3), state_(4), state_(5));
  q_update.normalize();
  state_(3) = q_update.x();
  state_(4) = q_update.y();
  state_(5) = q_update.z();
  state_(6) = q_update.w();

  Eigen::Vector3d bias(state_(7), state_(8), state_(9));
  bias += config_.ekf.position_gain * residual;
  state_(7) = bias.x();
  state_(8) = bias.y();
  state_(9) = bias.z();
}

/**
 * @brief 计算姿态对观测的雅可比矩阵
 * @param q 当前姿态四元数
 * @param field_world 世界坐标系磁场向量 (mT)
 * @return 3×4 雅可比矩阵
 */
Eigen::Matrix<double, 3, 4> EKFEstimator::orientationJacobian(const Eigen::Quaterniond &q,
                                                             const Eigen::Vector3d &field_world) const {
  Eigen::Matrix<double, 3, 4> J;
  J.setZero();
  Eigen::Matrix3d R = q.toRotationMatrix();
  Eigen::Matrix<double, 3, 3> skew;
  skew << 0.0, -field_world.z(), field_world.y(),
      field_world.z(), 0.0, -field_world.x(),
      -field_world.y(), field_world.x(), 0.0;
  J.block<3, 3>(0, 0) = -R * skew;
  J.col(3) = R * field_world;
  return J;
}

/**
 * @brief 获取当前估计的姿态
 * @return 姿态消息
 */
geometry_msgs::Pose EKFEstimator::getPose() const {
  geometry_msgs::Pose pose;
  pose.position.x = state_(0);
  pose.position.y = state_(1);
  pose.position.z = state_(2);
  pose.orientation.x = state_(3);
  pose.orientation.y = state_(4);
  pose.orientation.z = state_(5);
  pose.orientation.w = state_(6);
  return pose;
}

/**
 * @brief 获取估计的协方差矩阵
 * @param covariance_out 输出的协方差矩阵（6x6，位置3维+姿态3维）
 * @return 是否成功获取协方差矩阵
 */
bool EKFEstimator::getCovariance(Eigen::Matrix<double, 6, 6> &covariance_out) const {
  if (!initialized_) {
    return false;
  }
  
  covariance_out.setZero();
  
  // 提取位置协方差 [0:3, 0:3]
  covariance_out.block<3, 3>(0, 0) = covariance_.block<3, 3>(0, 0);
  
  // 将四元数协方差转换为方向向量协方差
  // 四元数协方差在 [3:7, 3:7]，需要转换为3维方向向量协方差
  // 简化方法：使用四元数前3个分量的协方差（x, y, z）作为方向向量协方差的近似
  // 注意：这不是完全准确的，但对于置信度计算足够
  Eigen::Matrix3d quat_cov = covariance_.block<3, 3>(3, 3);
  covariance_out.block<3, 3>(3, 3) = quat_cov;
  
  return true;
}

}  // 命名空间 mag_pose_estimator
