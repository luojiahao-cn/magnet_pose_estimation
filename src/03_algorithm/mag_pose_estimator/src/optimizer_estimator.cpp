#include "mag_pose_estimator/optimizer_estimator.h"

#include <Eigen/Geometry>
#include <algorithm>

namespace mag_pose_estimator {

namespace {
Eigen::Vector3d normalizeVec(const Eigen::Vector3d &v) {
  double norm = v.norm();
  if (norm > 1e-9) {
    return v / norm;
  }
  Eigen::Vector3d fallback = Eigen::Vector3d::UnitX();
  return fallback;
}
}

OptimizerEstimator::OptimizerEstimator() {
  position_.setZero();
  orientation_.setIdentity();
}

void OptimizerEstimator::initialize() {
  initialized_ = true;
}

void OptimizerEstimator::update(const sensor_msgs::MagneticField &mag) {
  if (!initialized_) {
    initialize();
  }
  Eigen::Vector3d measurement(mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
  runGaussNewton(measurement);
}

void OptimizerEstimator::runGaussNewton(const Eigen::Vector3d &measurement) {
  Eigen::Quaterniond q = orientation_;
  Eigen::Vector3d m_norm = normalizeVec(measurement);
  Eigen::Vector3d world_norm = normalizeVec(config_.world_field);

  for (int i = 0; i < std::max(1, config_.optimizer_iterations); ++i) {
    Eigen::Vector3d prediction = q * world_norm;
    Eigen::Vector3d residual = m_norm - prediction;

    Eigen::Matrix<double, 3, 3> skew;
    skew << 0.0, -prediction.z(), prediction.y(),
        prediction.z(), 0.0, -prediction.x(),
        -prediction.y(), prediction.x(), 0.0;

    Eigen::Matrix3d J = -2.0 * skew;
    Eigen::Matrix3d H = J.transpose() * J + config_.optimizer_damping * Eigen::Matrix3d::Identity();
    Eigen::Vector3d g = J.transpose() * residual;

    Eigen::Vector3d delta = H.ldlt().solve(g);
    double angle = delta.norm();
    Eigen::Vector3d axis;
    if (angle > 1e-9) {
      axis = delta / angle;
    } else {
      axis = Eigen::Vector3d::UnitX();
    }
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
    q = dq * q;
    q.normalize();

    if (delta.norm() < 1e-4) {
      break;
    }
  }

  orientation_ = q;
  position_ += config_.position_gain * (m_norm - orientation_ * world_norm);
}

geometry_msgs::Pose OptimizerEstimator::getPose() const {
  geometry_msgs::Pose pose;
  pose.position.x = position_.x();
  pose.position.y = position_.y();
  pose.position.z = position_.z();
  pose.orientation.x = orientation_.x();
  pose.orientation.y = orientation_.y();
  pose.orientation.z = orientation_.z();
  pose.orientation.w = orientation_.w();
  return pose;
}

}  // namespace mag_pose_estimator
