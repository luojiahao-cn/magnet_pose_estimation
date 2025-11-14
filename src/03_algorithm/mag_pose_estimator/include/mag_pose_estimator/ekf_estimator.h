#pragma once

#include <ros/time.h>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

/**
 * @brief Example EKF that tracks position, quaternion orientation, and magnetic bias.
 *
 * State vector (size 10): [px, py, pz, qx, qy, qz, qw, bx, by, bz]^T.
 */
class EKFEstimator : public EstimatorBase {
public:
  EKFEstimator();

  void initialize() override;
  void update(const sensor_msgs::MagneticField &mag) override;
  geometry_msgs::Pose getPose() const override;
  std::string name() const override { return "ekf"; }

private:
  void predict();
  void correct(const Eigen::Vector3d &measurement);
  Eigen::Matrix<double, 3, 4> orientationJacobian(const Eigen::Quaterniond &q, const Eigen::Vector3d &field_world) const;

  Eigen::Matrix<double, 10, 1> state_;
  Eigen::Matrix<double, 10, 10> covariance_;

  ros::Time last_update_time_;
};

}  // namespace mag_pose_estimator
