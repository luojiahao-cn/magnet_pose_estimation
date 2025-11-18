#include "mag_pose_estimator/ekf_estimator.h"

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
}  // 匿名命名空间

// EKF 状态向量依次存放位置(0-2)、四元数(x,y,z,w -> 3-6)以及残差偏置(7-9)。
EKFEstimator::EKFEstimator() {
  state_.setZero();
  state_(6) = 1.0;  // 单位四元数初值
  covariance_.setIdentity();
}

void EKFEstimator::initialize() {
  covariance_.setIdentity();
  covariance_ *= 0.1;
  last_update_time_ = ros::Time(0);
  initialized_ = true;
}

void EKFEstimator::update(const sensor_msgs::MagneticField &mag) {
  if (!initialized_) {
    initialize();
  }

  ros::Time stamp = mag.header.stamp;
  if (!stamp.isZero() && !last_update_time_.isZero()) {
    double dt = (stamp - last_update_time_).toSec();
    if (dt > 0.0) {
      double q_pos = config_.process_noise_position;
      double q_ori = config_.process_noise_orientation;
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

void EKFEstimator::predict() {
  // 预留运动模型接口。
  // TODO: 在此融合 IMU 线加速度以改进位置预测。
}

void EKFEstimator::correct(const Eigen::Vector3d &measurement) {
  Eigen::Quaterniond q(state_(6), state_(3), state_(4), state_(5));
  q = normalize(q);
  state_(3) = q.x();
  state_(4) = q.y();
  state_(5) = q.z();
  state_(6) = q.w();

  Eigen::Vector3d predicted = q * config_.world_field;
  Eigen::Vector3d residual = normalizeVec(measurement) - normalizeVec(predicted);

  Eigen::Matrix<double, 3, 10> H;
  H.setZero();

  // 姿态雅可比将四元数扰动映射到期望磁场。
  Eigen::Matrix<double, 3, 4> dq_dq = orientationJacobian(q, config_.world_field);
  H.block<3, 4>(0, 3) = dq_dq;

  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * config_.measurement_noise;
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
  bias += config_.position_gain * residual;
  state_(7) = bias.x();
  state_(8) = bias.y();
  state_(9) = bias.z();
}

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

}  // 命名空间 mag_pose_estimator
