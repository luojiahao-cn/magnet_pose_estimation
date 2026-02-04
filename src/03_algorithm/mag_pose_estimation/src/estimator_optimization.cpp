#include "mag_pose_estimator/estimator_optimization.h"

#include <ceres/ceres.h>
#include <ceres/covariance.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mag_pose_estimator/magnetic_field_model.h"

namespace mag_pose_estimator {

namespace {
/**
 * @brief 固定磁矩强度的代价函数（使用解析雅可比）
 *
 * 用于 Ceres 优化器，磁矩强度固定，只优化位置和方向。
 * 使用公式计算雅可比矩阵，而不是数值微分或自动微分。
 */
class FixedStrengthCostFunction : public ceres::SizedCostFunction<3, 3, 3> {
 public:
  FixedStrengthCostFunction(const Eigen::Vector3d &sensor_pos,
                            const Eigen::Vector3d &measured_field,
                            double strength)
      : sensor_pos_(sensor_pos), measured_field_(measured_field), strength_(strength) {}

  virtual ~FixedStrengthCostFunction() {}

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const override {
    // 提取参数
    Eigen::Vector3d position(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d direction(parameters[1][0], parameters[1][1], parameters[1][2]);

    // 计算预测磁场
    Eigen::Vector3d predicted = MagneticFieldModel::dipoleField(
        sensor_pos_, position, direction, strength_);

    // 计算残差
    Eigen::Map<Eigen::Vector3d> residual_vec(residuals);
    residual_vec = predicted - measured_field_;

    // 计算雅可比矩阵（如果请求）
    if (jacobians != nullptr && (jacobians[0] != nullptr || jacobians[1] != nullptr)) {
      Eigen::Matrix3d jacobian_pos, jacobian_dir;
      MagneticFieldModel::dipoleFieldJacobian(
          sensor_pos_, position, direction, strength_,
          jacobian_pos, jacobian_dir, nullptr);

      if (jacobians[0] != nullptr) {
        // 对位置的雅可比 [3×3]
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_pos_map(jacobians[0]);
        jacobian_pos_map = jacobian_pos;
      }

      if (jacobians[1] != nullptr) {
        // 对方向的雅可比 [3×3]
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_dir_map(jacobians[1]);
        jacobian_dir_map = jacobian_dir;
      }
    }

    return true;
  }

 private:
  Eigen::Vector3d sensor_pos_;  // 传感器位置 (米)
  Eigen::Vector3d measured_field_;  // 测量磁场 (mT)
  double strength_;  // 磁矩强度 (Am²)
};

/**
 * @brief 自由磁矩强度的代价函数（使用解析雅可比）
 *
 * 用于 Ceres 优化器，同时优化位置、方向和磁矩强度。
 * 使用公式计算雅可比矩阵，而不是数值微分或自动微分。
 */
class FreeStrengthCostFunction : public ceres::SizedCostFunction<3, 3, 3, 1> {
 public:
  FreeStrengthCostFunction(const Eigen::Vector3d &sensor_pos,
                            const Eigen::Vector3d &measured_field)
      : sensor_pos_(sensor_pos), measured_field_(measured_field) {}

  virtual ~FreeStrengthCostFunction() {}

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const override {
    // 提取参数
    Eigen::Vector3d position(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Vector3d direction(parameters[1][0], parameters[1][1], parameters[1][2]);
    double strength = parameters[2][0];

    // 计算预测磁场
    Eigen::Vector3d predicted = MagneticFieldModel::dipoleField(
        sensor_pos_, position, direction, strength);

    // 计算残差
    Eigen::Map<Eigen::Vector3d> residual_vec(residuals);
    residual_vec = predicted - measured_field_;

    // 计算雅可比矩阵（如果请求）
    if (jacobians != nullptr) {
      Eigen::Matrix3d jacobian_pos, jacobian_dir;
      Eigen::Vector3d jacobian_strength;

      MagneticFieldModel::dipoleFieldJacobian(
          sensor_pos_, position, direction, strength,
          jacobian_pos, jacobian_dir, &jacobian_strength);

      if (jacobians[0] != nullptr) {
        // 对位置的雅可比 [3×3]
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_pos_map(jacobians[0]);
        jacobian_pos_map = jacobian_pos;
      }

      if (jacobians[1] != nullptr) {
        // 对方向的雅可比 [3×3]
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_dir_map(jacobians[1]);
        jacobian_dir_map = jacobian_dir;
      }

      if (jacobians[2] != nullptr) {
        // 对强度的雅可比 [3×1]
        Eigen::Map<Eigen::Vector3d> jacobian_strength_map(jacobians[2]);
        jacobian_strength_map = jacobian_strength;
      }
    }

    return true;
  }

 private:
  Eigen::Vector3d sensor_pos_;  // 传感器位置 (米)
  Eigen::Vector3d measured_field_;  // 测量磁场 (mT)
};

}  // 匿名命名空间

OptimizerEstimator::OptimizerEstimator() {
  position_.setZero();
  magnet_direction_ = Eigen::Vector3d::UnitZ();
  last_pose_.orientation.w = 1.0;
}

void OptimizerEstimator::initialize() {
  initialized_ = true;
  resetState();
}

void OptimizerEstimator::update(const sensor_msgs::MagneticField & /* mag */) {
  if (!initialized_) {
    initialize();
  }
  // 批量优化器不使用单次更新，保持空实现
}

void OptimizerEstimator::resetState() {
  optimize_strength_ = config_.optimizer.optimize_strength;
  magnet_direction_ = config_.optimizer.initial_direction.normalized();
  magnet_strength_ = config_.optimizer.initial_strength;
  position_ = config_.optimizer.initial_position;
  strength_min_ = std::max(0.0, config_.optimizer.initial_strength - std::abs(config_.optimizer.strength_delta));
  strength_max_ = config_.optimizer.initial_strength + std::abs(config_.optimizer.strength_delta);

  // 解析线性求解器类型
  const std::string &solver = config_.optimizer.linear_solver;
  if (solver == "SPARSE_NORMAL_CHOLESKY") {
    linear_solver_type_ = ceres::SPARSE_NORMAL_CHOLESKY;
  } else if (solver == "DENSE_NORMAL_CHOLESKY") {
    linear_solver_type_ = ceres::DENSE_NORMAL_CHOLESKY;
  } else if (solver == "ITERATIVE_SCHUR") {
    linear_solver_type_ = ceres::ITERATIVE_SCHUR;
  } else {
    linear_solver_type_ = ceres::DENSE_QR;
  }

  last_pose_ = geometry_msgs::Pose();
  // 初始化四元数为单位四元数，避免 "Uninitialized quaternion" 警告
  last_pose_.orientation.w = 1.0;
  last_pose_.orientation.x = 0.0;
  last_pose_.orientation.y = 0.0;
  last_pose_.orientation.z = 0.0;
}

void OptimizerEstimator::reset() {
  resetState();
}

geometry_msgs::Pose OptimizerEstimator::getPose() const {
  return last_pose_;
}

bool OptimizerEstimator::processBatch(
    const std::vector<sensor_msgs::MagneticField> &measurements,
    const std::function<bool(const std::string &, const ros::Time &, Eigen::Vector3d &, geometry_msgs::TransformStamped &)> &tf_query,
    const std::string & /* output_frame */,
    geometry_msgs::Pose &pose_out,
    double *error_out) {
  if (measurements.empty()) {
    // 返回上一次的估计结果，避免未初始化的四元数
    pose_out = last_pose_;
    if (error_out) {
      *error_out = 0.0;
    }
    return false;
  }

  std::vector<OptimizerMeasurement> batch;
  batch.reserve(measurements.size());

  for (const auto &mag : measurements) {
    Eigen::Vector3d position;
    geometry_msgs::TransformStamped transform;
    if (!tf_query(mag.header.frame_id, mag.header.stamp, position, transform)) {
      continue;
    }

    geometry_msgs::Vector3Stamped v_sensor, v_world;
    v_sensor.header = mag.header;
    v_sensor.vector = mag.magnetic_field;
    tf2::doTransform(v_sensor, v_world, transform);

    OptimizerMeasurement meas;
    meas.sensor_id = 0;
    meas.sensor_position = position;
    meas.magnetic_field = Eigen::Vector3d(v_world.vector.x, v_world.vector.y, v_world.vector.z);
    batch.push_back(meas);
  }

  return estimateFromBatch(batch, pose_out, error_out);
}

geometry_msgs::Pose OptimizerEstimator::buildPoseFromDirection(const Eigen::Vector3d &direction,
                                                                const Eigen::Vector3d &position) const {
  geometry_msgs::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();

  // 初始化四元数为单位四元数，避免 "Uninitialized quaternion" 警告
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;

  // 使用简洁的姿态计算方式（从 Z 轴到方向向量的旋转）
  tf2::Vector3 z_axis(0, 0, 1);
  tf2::Vector3 dir_v(direction.x(), direction.y(), direction.z());
  tf2::Vector3 axis = z_axis.cross(dir_v);
  double dot = std::max(-1.0, std::min(1.0, z_axis.dot(dir_v)));
  double angle = std::acos(dot);

  if (axis.length() < 1e-6) {
    // 方向向量与 Z 轴平行或反平行
    if (dir_v.z() < 0) {
      // 方向向量指向 -Z 方向，需要旋转 180 度
      pose.orientation.w = 0.0;
      pose.orientation.x = 1.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
    }
    // 如果 dir_v.z() > 0，已经是单位四元数 [1, 0, 0, 0]，无需修改
  } else {
    axis.normalize();
    tf2::Quaternion q(axis, angle);
    q.normalize();
    pose.orientation = tf2::toMsg(q);
  }

  return pose;
}

bool OptimizerEstimator::estimateFromBatch(const std::vector<OptimizerMeasurement> &batch,
                                          geometry_msgs::Pose &pose_out,
                                          double *error_out) {
  if (batch.empty()) {
    return false;
  }

  // 直接使用当前值作为初始值
  double position[3] = {position_.x(), position_.y(), position_.z()};
  double direction[3] = {magnet_direction_.x(), magnet_direction_.y(), magnet_direction_.z()};
  double strength = magnet_strength_;

  // 确保方向向量有效性
  Eigen::Map<Eigen::Vector3d> dir_map(direction);
  if (dir_map.norm() < 1e-9) {
    dir_map = Eigen::Vector3d::UnitZ();
  } else {
    dir_map.normalize();
  }

  // 构建核心优化问题
  ceres::Problem problem;
  for (const auto &meas : batch) {
    if (optimize_strength_) {
      problem.AddResidualBlock(new FreeStrengthCostFunction(meas.sensor_position, meas.magnetic_field),
                               nullptr, position, direction, &strength);
    } else {
      problem.AddResidualBlock(new FixedStrengthCostFunction(meas.sensor_position, meas.magnetic_field, magnet_strength_),
                               nullptr, position, direction);
    }
  }

  // 应用单位向量约束
#if CERES_VERSION_MAJOR > 2 || (CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 1)
  problem.SetManifold(direction, new ceres::SphereManifold<3>());
#else
  problem.SetParameterization(direction, new ceres::HomogeneousVectorParameterization(3));
#endif

  // 应用位置限制
  if (config_.optimizer.use_position_limit) {
    for (int i = 0; i < 3; ++i) {
      problem.SetParameterLowerBound(position, i, config_.optimizer.position_min[i]);
      problem.SetParameterUpperBound(position, i, config_.optimizer.position_max[i]);
    }
  }

  if (optimize_strength_) {
    if (config_.optimizer.use_strength_limit) {
      problem.SetParameterLowerBound(&strength, 0, strength_min_);
      problem.SetParameterUpperBound(&strength, 0, strength_max_);
    } else {
      // 全局优化模式：不限制强度上限，仅确保物理上为正
      problem.SetParameterLowerBound(&strength, 0, 0.0);
    }
  }

  // 基础求解器配置
  ceres::Solver::Options options;
  options.linear_solver_type = linear_solver_type_;
  options.max_num_iterations = config_.optimizer.max_iterations;
  options.num_threads = config_.optimizer.num_threads;
  options.logging_type = ceres::SILENT;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 只要优化收敛或结果可用，即更新状态并输出
  if (summary.termination_type == ceres::CONVERGENCE || summary.termination_type == ceres::USER_SUCCESS) {
    position_ = Eigen::Vector3d(position[0], position[1], position[2]);
    magnet_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
    if (optimize_strength_) {
      magnet_strength_ = strength;
    }

    pose_out = buildPoseFromDirection(magnet_direction_, position_);
    last_pose_ = pose_out;
    if (error_out) {
      *error_out = summary.final_cost;
    }
    return true;
  }

  return false;
}

}  // 命名空间 mag_pose_estimator
