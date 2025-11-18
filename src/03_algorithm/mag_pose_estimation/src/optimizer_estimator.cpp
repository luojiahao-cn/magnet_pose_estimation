#include "mag_pose_estimator/optimizer_estimator.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mag_pose_estimator/magnetic_field_model.h"

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
  magnet_direction_ = Eigen::Vector3d::UnitZ();
  last_pose_.orientation.w = 1.0;
}

void OptimizerEstimator::initialize() {
  initialized_ = true;
  resetState();
}

void OptimizerEstimator::update(const sensor_msgs::MagneticField &mag) {
  if (!initialized_) {
    initialize();
  }
  Eigen::Vector3d measurement(mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
  runGaussNewton(measurement);
}

void OptimizerEstimator::runGaussNewton(const Eigen::Vector3d &measurement) {
  // 每条磁场读数都跑一次 Gauss-Newton，用来在批量求解之间保持姿态估计的新鲜度。
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
  magnet_direction_ = orientation_ * Eigen::Vector3d::UnitZ();
  last_pose_ = poseFromDirection(magnet_direction_);
}

void OptimizerEstimator::resetState() {
  // 重置时读取配置里的默认值，确保批量求解器总是从已知先验开始。
  optimize_strength_ = config_.optimizer.optimize_strength;
  magnet_direction_ = config_.optimizer.initial_direction.normalized();
  magnet_strength_ = config_.optimizer.initial_strength;
  position_ = config_.optimizer.initial_position;
  strength_min_ = std::max(0.0, config_.optimizer.initial_strength - std::abs(config_.optimizer.strength_delta));
  strength_max_ = config_.optimizer.initial_strength + std::abs(config_.optimizer.strength_delta);
  linear_solver_type_ = resolveLinearSolver(config_.optimizer.linear_solver);
  last_pose_ = geometry_msgs::Pose();
  last_pose_.orientation.w = 1.0;
}

ceres::LinearSolverType OptimizerEstimator::resolveLinearSolver(const std::string &solver) const {
  if (solver == "SPARSE_NORMAL_CHOLESKY") {
    return ceres::SPARSE_NORMAL_CHOLESKY;
  }
  if (solver == "DENSE_NORMAL_CHOLESKY") {
    return ceres::DENSE_NORMAL_CHOLESKY;
  }
  if (solver == "ITERATIVE_SCHUR") {
    return ceres::ITERATIVE_SCHUR;
  }
  return ceres::DENSE_QR;
}

geometry_msgs::Pose OptimizerEstimator::poseFromDirection(const Eigen::Vector3d &direction) const {
  geometry_msgs::Pose pose;
  pose.position.x = position_.x();
  pose.position.y = position_.y();
  pose.position.z = position_.z();

  tf2::Vector3 z_axis(0, 0, 1);
  tf2::Vector3 dir(direction.x(), direction.y(), direction.z());
  if (dir.length() < 1e-9) {
    dir = tf2::Vector3(0, 0, 1);
  }
  dir.normalize();
  tf2::Vector3 axis = z_axis.cross(dir);
  double dot = std::max(-1.0, std::min(1.0, z_axis.dot(dir)));
  double angle = std::acos(dot);
  if (axis.length() < 1e-9) {
    if (dir.z() >= 0.0) {
      pose.orientation.w = 1.0;
      pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
    } else {
      pose.orientation.w = 0.0;
      pose.orientation.x = 1.0;
      pose.orientation.y = pose.orientation.z = 0.0;
    }
  } else {
    axis.normalize();
    tf2::Quaternion q(axis, angle);
    q.normalize();
    pose.orientation = tf2::toMsg(q);
  }
  pose.orientation.w = std::max(-1.0, std::min(1.0, pose.orientation.w));
  return pose;
}

namespace {

struct FixedStrengthResidual {
  FixedStrengthResidual(const Eigen::Vector3d &sensor_pos,
                        const Eigen::Vector3d &measured_field,
                        double strength)
      : sensor_pos_(sensor_pos), measured_field_(measured_field), strength_(strength) {}

  template <typename T>
  bool operator()(const T *const position, const T *const direction, T *residuals) const {
    Eigen::Matrix<T, 3, 1> sensor_pos_T(T(sensor_pos_.x()), T(sensor_pos_.y()), T(sensor_pos_.z()));
    Eigen::Matrix<T, 3, 1> measured_T(T(measured_field_.x()), T(measured_field_.y()), T(measured_field_.z()));
    Eigen::Matrix<T, 3, 1> pos(position[0], position[1], position[2]);
    Eigen::Matrix<T, 3, 1> dir(direction[0], direction[1], direction[2]);
    Eigen::Matrix<T, 3, 1> predicted = MagneticFieldModel::dipoleField(sensor_pos_T, pos, dir, T(strength_));
    Eigen::Matrix<T, 3, 1> r = predicted - measured_T;
    residuals[0] = r.x();
    residuals[1] = r.y();
    residuals[2] = r.z();
    return true;
  }

  Eigen::Vector3d sensor_pos_;
  Eigen::Vector3d measured_field_;
  double strength_ = 1.0;
};

struct FreeStrengthResidual {
  FreeStrengthResidual(const Eigen::Vector3d &sensor_pos,
                       const Eigen::Vector3d &measured_field)
      : sensor_pos_(sensor_pos), measured_field_(measured_field) {}

  template <typename T>
  bool operator()(const T *const position, const T *const direction, const T *const strength, T *residuals) const {
    Eigen::Matrix<T, 3, 1> sensor_pos_T(T(sensor_pos_.x()), T(sensor_pos_.y()), T(sensor_pos_.z()));
    Eigen::Matrix<T, 3, 1> measured_T(T(measured_field_.x()), T(measured_field_.y()), T(measured_field_.z()));
    Eigen::Matrix<T, 3, 1> pos(position[0], position[1], position[2]);
    Eigen::Matrix<T, 3, 1> dir(direction[0], direction[1], direction[2]);
    Eigen::Matrix<T, 3, 1> predicted = MagneticFieldModel::dipoleField(sensor_pos_T, pos, dir, strength[0]);
    Eigen::Matrix<T, 3, 1> r = predicted - measured_T;
    residuals[0] = r.x();
    residuals[1] = r.y();
    residuals[2] = r.z();
    return true;
  }

  Eigen::Vector3d sensor_pos_;
  Eigen::Vector3d measured_field_;
};

}  // 匿名命名空间

bool OptimizerEstimator::estimateFromBatch(const std::vector<OptimizerMeasurement> &batch,
                                           geometry_msgs::Pose &pose_out,
                                           double *error_out) {
  if (batch.empty()) {
    return false;
  }

  double position[3] = {position_.x(), position_.y(), position_.z()};
  double direction[3] = {magnet_direction_.x(), magnet_direction_.y(), magnet_direction_.z()};
  double strength = magnet_strength_;

  // 每条测量对应一个 Ceres 残差块，按需把磁强当作优化变量。
  ceres::Problem problem;
  for (const auto &meas : batch) {
    Eigen::Vector3d measurement_T = meas.magnetic_field;
    Eigen::Vector3d sensor_pos = meas.sensor_position;
    if (optimize_strength_) {
      auto *cost = new ceres::AutoDiffCostFunction<FreeStrengthResidual, 3, 3, 3, 1>(
          new FreeStrengthResidual(sensor_pos, measurement_T));
      problem.AddResidualBlock(cost, nullptr, position, direction, &strength);
    } else {
      auto *cost = new ceres::AutoDiffCostFunction<FixedStrengthResidual, 3, 3, 3>(
          new FixedStrengthResidual(sensor_pos, measurement_T, magnet_strength_));
      problem.AddResidualBlock(cost, nullptr, position, direction);
    }
  }

  problem.SetParameterization(direction, new ceres::HomogeneousVectorParameterization(3));
  if (optimize_strength_) {
    problem.SetParameterLowerBound(&strength, 0, strength_min_);
    problem.SetParameterUpperBound(&strength, 0, strength_max_);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = linear_solver_type_;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = std::max(1, config_.optimizer.max_iterations);
  options.function_tolerance = config_.optimizer.function_tolerance;
  options.gradient_tolerance = config_.optimizer.gradient_tolerance;
  options.parameter_tolerance = config_.optimizer.parameter_tolerance;
  options.num_threads = std::max(1, config_.optimizer.num_threads);
  options.minimizer_progress_to_stdout = config_.optimizer.minimizer_progress;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (summary.termination_type == ceres::NO_CONVERGENCE) {
    ROS_WARN_THROTTLE(1.0, "optimizer_estimator: solver did not converge");
  }
  if (error_out) {
    *error_out = summary.final_cost;
  }

  // 将优化后的变量写回，方便下一批或流式更新延续当前状态。
  position_ = Eigen::Vector3d(position[0], position[1], position[2]);
  magnet_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
  if (optimize_strength_) {
    magnet_strength_ = strength;
  }

  last_pose_ = poseFromDirection(magnet_direction_);
  pose_out = last_pose_;
  return true;
}

geometry_msgs::Pose OptimizerEstimator::getPose() const {
  return last_pose_;
}

void OptimizerEstimator::reset() {
  resetState();
}

}  // 命名空间 mag_pose_estimator
