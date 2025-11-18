#pragma once

#include <ceres/ceres.h>

#include <Eigen/Geometry>
#include <vector>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

struct OptimizerMeasurement {
  int sensor_id = 0;
  Eigen::Vector3d sensor_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d magnetic_field = Eigen::Vector3d::Zero();
};

/**
 * @brief 非线性优化器：让实测磁方向与名义磁场对齐。
 */
class OptimizerEstimator : public EstimatorBase {
public:
  OptimizerEstimator();

  void initialize() override;
  void update(const sensor_msgs::MagneticField &mag) override;
  geometry_msgs::Pose getPose() const override;
  std::string name() const override { return "optimizer"; }

  bool estimateFromBatch(const std::vector<OptimizerMeasurement> &batch,
                         geometry_msgs::Pose &pose_out,
                         double *error_out);

  void reset();

private:
  void runGaussNewton(const Eigen::Vector3d &measurement);
  void resetState();
  ceres::LinearSolverType resolveLinearSolver(const std::string &solver) const;
  geometry_msgs::Pose poseFromDirection(const Eigen::Vector3d &direction) const;

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d magnet_direction_;
  double magnet_strength_ = 1.0;
  bool optimize_strength_ = false;
  double strength_min_ = 0.0;
  double strength_max_ = 0.0;
  ceres::LinearSolverType linear_solver_type_ = ceres::DENSE_QR;
  geometry_msgs::Pose last_pose_;
};

}  // 命名空间 mag_pose_estimator
