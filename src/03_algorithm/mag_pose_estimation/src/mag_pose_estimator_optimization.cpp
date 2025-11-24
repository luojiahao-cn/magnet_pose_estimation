#include "mag_pose_estimator/mag_pose_estimator_optimization.h"

#include <ceres/ceres.h>
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

  if (measurements.size() < static_cast<size_t>(config_.optimizer.min_sensors)) {
    ROS_WARN_THROTTLE(2.0, "[mag_pose_estimator] 传感器数量不足 (当前: %zu, 需要: %zu)",
                      measurements.size(), static_cast<size_t>(config_.optimizer.min_sensors));
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

  if (batch.size() < static_cast<size_t>(config_.optimizer.min_sensors)) {
    ROS_WARN_THROTTLE(2.0, "[mag_pose_estimator] TF 查询后传感器数量不足 (当前: %zu, 需要: %zu)",
                      batch.size(), static_cast<size_t>(config_.optimizer.min_sensors));
    // 返回上一次的估计结果，避免未初始化的四元数
    pose_out = last_pose_;
    if (error_out) {
      *error_out = 0.0;
    }
    return false;
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
    ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 批量数据为空，无法估计");
    // 返回上一次的估计结果，避免未初始化的四元数
    pose_out = last_pose_;
    if (error_out) {
      *error_out = 0.0;
    }
    return false;
  }

  // 备份当前状态（优化失败时恢复）
  Eigen::Vector3d backup_position = position_;
  Eigen::Vector3d backup_direction = magnet_direction_;
  double backup_strength = magnet_strength_;

  // 使用当前状态作为初始值（warm start：使用上一次的优化结果）
  double position[3] = {position_.x(), position_.y(), position_.z()};
  double direction[3] = {magnet_direction_.x(), magnet_direction_.y(), magnet_direction_.z()};
  double strength = magnet_strength_;
  
  ROS_DEBUG_THROTTLE(2.0, "[mag_pose_estimator] 使用上一次优化结果作为初始值: 位置=[%.3f, %.3f, %.3f], 方向=[%.3f, %.3f, %.3f], 强度=%.2f",
                     position[0], position[1], position[2], direction[0], direction[1], direction[2], strength);
  
  // 归一化方向向量
  double dir_norm = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
  if (dir_norm > 1e-9) {
    direction[0] /= dir_norm;
    direction[1] /= dir_norm;
    direction[2] /= dir_norm;
  } else {
    // 如果方向未初始化，使用配置中的初始方向
    direction[0] = config_.optimizer.initial_direction.x();
    direction[1] = config_.optimizer.initial_direction.y();
    direction[2] = config_.optimizer.initial_direction.z();
    double init_norm = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (init_norm > 1e-9) {
      direction[0] /= init_norm;
      direction[1] /= init_norm;
      direction[2] /= init_norm;
    } else {
      direction[0] = 0.0;
      direction[1] = 0.0;
      direction[2] = 1.0;
    }
  }
  
  // 简单验证初始值（只在有 NaN/Inf 时重置）
  bool has_invalid = false;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(position[i]) || !std::isfinite(direction[i])) {
      has_invalid = true;
      break;
    }
  }
  if (!std::isfinite(strength) || strength <= 0.0) {
    has_invalid = true;
  }
  
  if (has_invalid) {
    ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 初始值无效，重置为配置默认值");
    position[0] = config_.optimizer.initial_position.x();
    position[1] = config_.optimizer.initial_position.y();
    position[2] = config_.optimizer.initial_position.z();
    direction[0] = config_.optimizer.initial_direction.x();
    direction[1] = config_.optimizer.initial_direction.y();
    direction[2] = config_.optimizer.initial_direction.z();
    strength = config_.optimizer.initial_strength;
    double norm = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (norm > 1e-9) {
      direction[0] /= norm;
      direction[1] /= norm;
      direction[2] /= norm;
    } else {
      direction[0] = 0.0;
      direction[1] = 0.0;
      direction[2] = 1.0;
    }
  }
  
  // 验证批量数据
  for (const auto &meas : batch) {
    if (!meas.sensor_position.allFinite() || !meas.magnetic_field.allFinite()) {
      ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 测量数据包含无效值");
      // 返回上一次的估计结果，避免未初始化的四元数
      pose_out = last_pose_;
      if (error_out) {
        *error_out = 0.0;
      }
      return false;
    }
  }

  // 构建 Ceres 优化问题（使用解析雅可比）
  ceres::Problem problem;
  for (const auto &meas : batch) {
    if (optimize_strength_) {
      // 使用解析雅可比计算（公式计算，不依赖数值微分或自动微分）
      auto *cost = new FreeStrengthCostFunction(meas.sensor_position, meas.magnetic_field);
      problem.AddResidualBlock(cost, nullptr, position, direction, &strength);
    } else {
      // 使用解析雅可比计算（公式计算，不依赖数值微分或自动微分）
      auto *cost = new FixedStrengthCostFunction(meas.sensor_position, meas.magnetic_field, magnet_strength_);
      problem.AddResidualBlock(cost, nullptr, position, direction);
    }
  }

  // 设置方向向量的参数化（确保归一化约束）
  problem.SetParameterization(direction, new ceres::HomogeneousVectorParameterization(3));
  
  // 设置强度变量的上下界（如果优化强度）
  if (optimize_strength_) {
    problem.SetParameterLowerBound(&strength, 0, strength_min_);
    problem.SetParameterUpperBound(&strength, 0, strength_max_);
  }

  // 配置 Ceres 求解器选项（使用 Levenberg-Marquardt）
  ceres::Solver::Options options;
  options.linear_solver_type = linear_solver_type_;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;  // LM 算法
  options.max_num_iterations = std::max(1, config_.optimizer.max_iterations);
  options.function_tolerance = config_.optimizer.function_tolerance;
  options.gradient_tolerance = config_.optimizer.gradient_tolerance;
  options.parameter_tolerance = config_.optimizer.parameter_tolerance;
  options.num_threads = std::max(1, config_.optimizer.num_threads);
  options.minimizer_progress_to_stdout = config_.optimizer.minimizer_progress;
  
  // 提高精度的额外选项
  options.use_nonmonotonic_steps = true;  // 允许非单调步长，有助于跳出局部最优
  options.max_consecutive_nonmonotonic_steps = 5;  // 非单调步长最大次数
  options.use_inner_iterations = false;  // 对于小规模问题，关闭内迭代
  options.logging_type = ceres::SILENT;  // 除非启用 minimizer_progress，否则静默

  // 执行优化
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 检查优化结果
  bool optimization_success = false;
  double final_error = summary.final_cost;
  double initial_error = summary.initial_cost;
  
  // 判断优化是否成功（收敛或达到合理精度）
  if (summary.termination_type == ceres::CONVERGENCE ||
      summary.termination_type == ceres::USER_SUCCESS) {
    optimization_success = true;
  } else if (summary.termination_type == ceres::NO_CONVERGENCE) {
    // 未完全收敛，但检查误差改善情况
    if (initial_error > 1e-10) {  // 避免除零
      double error_reduction = (initial_error - final_error) / initial_error;
      if (error_reduction > 0.1 && final_error < initial_error * 0.9) {
        // 误差有明显改善（至少10%），认为优化有效
        optimization_success = true;
        ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 优化未完全收敛但误差已改善，迭代次数: %d，初始误差: %.9f，最终误差: %.9f，改善: %.2f%%",
                         summary.num_successful_steps, initial_error, final_error, error_reduction * 100.0);
      } else {
        ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 优化未收敛且改善不足，迭代次数: %d，初始误差: %.9f，最终误差: %.9f，改善: %.2f%%",
                         summary.num_successful_steps, initial_error, final_error, error_reduction * 100.0);
      }
    } else {
      // 初始误差过小，无法判断改善情况
      ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 优化未收敛，初始误差过小 (%.9e)，无法判断改善情况",
                       initial_error);
    }
  } else if (summary.termination_type == ceres::FAILURE) {
    ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 优化失败: %s，初始误差: %.9f", 
                     summary.BriefReport().c_str(), initial_error);
  }
  
  // 如果优化失败，检查是否应该使用备份状态或更新状态
  if (!optimization_success) {
    double avg_initial_residual = std::sqrt(initial_error / (batch.size() * 3));
    double avg_final_residual = std::sqrt(final_error / (batch.size() * 3));
    
    // 即使优化未收敛，如果最终误差比初始误差小，也更新状态（用于下一次优化）
    if (final_error < initial_error && avg_final_residual < avg_initial_residual) {
      // 误差有改善，更新状态供下一次使用
      position_ = Eigen::Vector3d(position[0], position[1], position[2]);
      magnet_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
      if (optimize_strength_) {
        magnet_strength_ = strength;
      }
      pose_out = buildPoseFromDirection(magnet_direction_, position_);
      last_pose_ = pose_out;
      if (error_out) {
        *error_out = final_error;
      }
      ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 优化未收敛但误差已改善，更新状态供下次使用，平均残差: %.9f -> %.9f mT，初始误差: %.9f，最终误差: %.9f",
                        avg_initial_residual, avg_final_residual, initial_error, final_error);
      return false;  // 虽然误差改善了，但未收敛，仍返回 false
    }
    
    // 否则使用备份状态（优化前的状态）
    ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 优化未收敛且误差未改善，使用备份状态，平均残差: %.9f mT，初始误差: %.9f，最终误差: %.9f",
                      avg_final_residual, initial_error, final_error);
    position_ = backup_position;
    magnet_direction_ = backup_direction;
    magnet_strength_ = backup_strength;
    pose_out = buildPoseFromDirection(magnet_direction_, position_);
    last_pose_ = pose_out;
    if (error_out) {
      *error_out = initial_error;
    }
    return false;  // 优化失败，返回 false
  }
  
  // 优化成功，检查最终误差是否在合理范围内
  double avg_final_residual = std::sqrt(final_error / (batch.size() * 3));
  const double max_acceptable_residual = config_.optimizer.max_acceptable_residual;  // 从配置读取可接受的最大平均残差（mT）
  
  if (avg_final_residual > max_acceptable_residual) {
    // 误差过大，即使优化收敛也认为失败
    // 使用备份状态（上一次的优化结果），不重置为配置初始值
    // 因为实际应用中不知道磁铁的真实位置
    ROS_WARN_THROTTLE(1.0, "[mag_pose_estimator] 优化收敛但误差过大，平均残差: %.9f mT (阈值: %.9f mT)，使用备份状态，初始误差: %.9f，最终误差: %.9f",
                      avg_final_residual, max_acceptable_residual, initial_error, final_error);
    position_ = backup_position;
    magnet_direction_ = backup_direction;
    magnet_strength_ = backup_strength;
    pose_out = buildPoseFromDirection(magnet_direction_, position_);
    last_pose_ = pose_out;
    if (error_out) {
      *error_out = initial_error;
    }
    return false;  // 返回失败
  }

  // 误差在合理范围内，更新内部状态（用于下一次优化的初始值）
  if (error_out) {
    *error_out = final_error;
  }

  // 更新内部状态（保存优化结果，供下一次使用）
  position_ = Eigen::Vector3d(position[0], position[1], position[2]);
  magnet_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
  if (optimize_strength_) {
    magnet_strength_ = strength;
  }
  
  ROS_DEBUG_THROTTLE(2.0, "[mag_pose_estimator] 优化成功，更新状态供下次使用: 位置=[%.3f, %.3f, %.3f], 方向=[%.3f, %.3f, %.3f], 强度=%.2f",
                     position_.x(), position_.y(), position_.z(), 
                     magnet_direction_.x(), magnet_direction_.y(), magnet_direction_.z(), magnet_strength_);

  // 构造输出姿态
  pose_out = buildPoseFromDirection(magnet_direction_, position_);
  last_pose_ = pose_out;

  // 记录优化信息
  if (summary.termination_type == ceres::CONVERGENCE || summary.termination_type == ceres::USER_SUCCESS) {
    ROS_DEBUG_THROTTLE(1.0, "[mag_pose_estimator] 优化成功收敛，迭代次数: %d，初始误差: %.9f，最终误差: %.9f，平均残差: %.9f mT，改善: %.2f%%",
                      summary.num_successful_steps, initial_error, final_error, avg_final_residual,
                      (initial_error - final_error) / initial_error * 100.0);
  }

  return true;
}

}  // 命名空间 mag_pose_estimator
