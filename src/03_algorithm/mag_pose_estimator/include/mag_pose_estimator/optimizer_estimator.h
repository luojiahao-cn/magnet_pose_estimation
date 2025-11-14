#pragma once

#include <Eigen/Geometry>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

/**
 * @brief Simple non-linear optimizer that aligns measured magnetic direction with a nominal field.
 */
class OptimizerEstimator : public EstimatorBase {
public:
  OptimizerEstimator();

  void initialize() override;
  void update(const sensor_msgs::MagneticField &mag) override;
  geometry_msgs::Pose getPose() const override;
  std::string name() const override { return "optimizer"; }

private:
  void runGaussNewton(const Eigen::Vector3d &measurement);

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
};

}  // namespace mag_pose_estimator
