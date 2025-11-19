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
/**
 * @brief 归一化向量
 * 如果向量长度过小，返回单位 X 向量作为默认值
 */
Eigen::Vector3d normalizeVec(const Eigen::Vector3d &v) {
  double norm = v.norm();
  if (norm > 1e-9) {
    return v / norm;
  }
  Eigen::Vector3d fallback = Eigen::Vector3d::UnitX();
  return fallback;
}
}  // 匿名命名空间

/**
 * @brief 构造函数
 * 初始化默认状态：位置为零，姿态为单位四元数，磁矩方向为 Z 轴
 */
OptimizerEstimator::OptimizerEstimator() {
  position_.setZero();
  orientation_.setIdentity();
  magnet_direction_ = Eigen::Vector3d::UnitZ();
  last_pose_.orientation.w = 1.0;
}

/**
 * @brief 初始化估计器
 * 调用 resetState() 从配置中加载初始值
 */
void OptimizerEstimator::initialize() {
  initialized_ = true;
  resetState();
}

/**
 * @brief 更新估计器状态（单次测量）
 * @param mag 磁场测量数据（单位：mT，毫特斯拉）
 * 
 * 对于单次测量，使用 Gauss-Newton 方法快速更新姿态。
 * 位置估计需要通过批量优化器（estimateFromBatch）来完成。
 */
void OptimizerEstimator::update(const sensor_msgs::MagneticField &mag) {
  if (!initialized_) {
    initialize();
  }
  Eigen::Vector3d measurement(mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z);
  runGaussNewton(measurement);
}

/**
 * @brief 使用 Gauss-Newton 方法快速更新姿态
 * @param measurement 磁场测量向量 (mT)
 * 
 * 此方法仅优化姿态（四元数），不优化位置。
 * 用于在批量优化之间保持姿态估计的新鲜度。
 * 
 * 算法流程：
 * 1. 归一化测量值和世界磁场向量
 * 2. 迭代优化四元数，使预测方向与测量方向对齐
 * 3. 使用 Gauss-Newton 方法求解：H * delta = g
 *   其中 H = J^T * J + damping，g = J^T * residual
 * 4. 通过指数映射更新四元数：q_new = exp(delta) * q_old
 * 
 * 注意：此方法不更新位置，位置估计应通过批量优化器完成。
 */
void OptimizerEstimator::runGaussNewton(const Eigen::Vector3d &measurement) {
  Eigen::Quaterniond q = orientation_;
  Eigen::Vector3d m_norm = normalizeVec(measurement);  // 归一化测量值
  Eigen::Vector3d world_norm = normalizeVec(config_.world_field);  // 归一化世界磁场

  // Gauss-Newton 迭代
  for (int i = 0; i < std::max(1, config_.optimizer_iterations); ++i) {
    // 计算预测方向（通过旋转世界磁场向量）
    Eigen::Vector3d prediction = q * world_norm;
    
    // 计算残差（归一化方向差）
    Eigen::Vector3d residual = m_norm - prediction;

    // 计算预测方向的反对称矩阵（用于计算雅可比）
    Eigen::Matrix<double, 3, 3> skew;
    skew << 0.0, -prediction.z(), prediction.y(),
        prediction.z(), 0.0, -prediction.x(),
        -prediction.y(), prediction.x(), 0.0;

    // 雅可比矩阵：J = -2 * skew(prediction)
    Eigen::Matrix3d J = -2.0 * skew;
    
    // Hessian 矩阵：H = J^T * J + damping * I
    Eigen::Matrix3d H = J.transpose() * J + config_.optimizer_damping * Eigen::Matrix3d::Identity();
    
    // 梯度：g = J^T * residual
    Eigen::Vector3d g = J.transpose() * residual;

    // 求解：H * delta = g
    Eigen::Vector3d delta = H.ldlt().solve(g);
    
    // 将 delta 转换为旋转角度和轴
    double angle = delta.norm();
    Eigen::Vector3d axis;
    if (angle > 1e-9) {
      axis = delta / angle;
    } else {
      axis = Eigen::Vector3d::UnitX();
    }
    
    // 通过指数映射更新四元数：q_new = exp(delta) * q_old
    Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
    q = dq * q;
    q.normalize();

    // 收敛检查
    if (delta.norm() < 1e-4) {
      break;
    }
  }

  orientation_ = q;
  // 注意：不更新位置，保持上次批量优化的结果
  magnet_direction_ = orientation_ * Eigen::Vector3d::UnitZ();
  last_pose_ = poseFromDirection(magnet_direction_);
}

/**
 * @brief 重置内部状态
 * 从配置中读取初始值并重置所有状态变量
 */
void OptimizerEstimator::resetState() {
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

/**
 * @brief 解析线性求解器类型字符串
 * @param solver 求解器类型字符串
 * @return Ceres 线性求解器类型
 * 
 * 支持的求解器类型：
 * - "SPARSE_NORMAL_CHOLESKY"
 * - "DENSE_NORMAL_CHOLESKY"
 * - "ITERATIVE_SCHUR"
 * - 默认：DENSE_QR
 */
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

/**
 * @brief 从方向向量构造姿态
 * @param direction 磁矩方向向量（归一化）
 * @return 包含位置和姿态的 Pose 消息
 * 
 * 将磁矩方向向量转换为四元数姿态：
 * 1. 计算从 Z 轴到方向向量的旋转
 * 2. 将旋转转换为四元数
 * 3. 位置使用当前估计值
 */
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
  
  // 计算旋转轴和角度
  tf2::Vector3 axis = z_axis.cross(dir);
  double dot = std::max(-1.0, std::min(1.0, z_axis.dot(dir)));
  double angle = std::acos(dot);
  
  // 处理特殊情况（方向向量与 Z 轴平行）
  if (axis.length() < 1e-9) {
    if (dir.z() >= 0.0) {
      // 与 Z 轴同向，使用单位四元数
      pose.orientation.w = 1.0;
      pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
    } else {
      // 与 Z 轴反向，旋转 180 度
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

/**
 * @brief 固定磁矩强度的残差函数
 * 
 * 用于 Ceres 优化器，磁矩强度固定，只优化位置和方向。
 * 残差 = predicted_field - measured_field (mT)
 */
struct FixedStrengthResidual {
  FixedStrengthResidual(const Eigen::Vector3d &sensor_pos,
                        const Eigen::Vector3d &measured_field,
                        double strength)
      : sensor_pos_(sensor_pos), measured_field_(measured_field), strength_(strength) {}

  /**
   * @brief 计算残差
   * @param position 磁铁位置 [x, y, z] (米)
   * @param direction 磁矩方向向量（归一化）
   * @param residuals 输出的残差向量 [rx, ry, rz] (mT)
   */
  template <typename T>
  bool operator()(const T *const position, const T *const direction, T *residuals) const {
    Eigen::Matrix<T, 3, 1> sensor_pos_T(T(sensor_pos_.x()), T(sensor_pos_.y()), T(sensor_pos_.z()));
    Eigen::Matrix<T, 3, 1> measured_T(T(measured_field_.x()), T(measured_field_.y()), T(measured_field_.z()));
    Eigen::Matrix<T, 3, 1> pos(position[0], position[1], position[2]);
    Eigen::Matrix<T, 3, 1> dir(direction[0], direction[1], direction[2]);
    
    // 使用磁偶极子模型计算预测磁场
    Eigen::Matrix<T, 3, 1> predicted = MagneticFieldModel::dipoleField(sensor_pos_T, pos, dir, T(strength_));
    
    // 计算残差
    Eigen::Matrix<T, 3, 1> r = predicted - measured_T;
    residuals[0] = r.x();
    residuals[1] = r.y();
    residuals[2] = r.z();
    return true;
  }

  Eigen::Vector3d sensor_pos_;  // 传感器位置 (米)
  Eigen::Vector3d measured_field_;  // 测量磁场 (mT)
  double strength_ = 1.0;  // 磁矩强度 (Am²)
};

/**
 * @brief 自由磁矩强度的残差函数
 * 
 * 用于 Ceres 优化器，同时优化位置、方向和磁矩强度。
 * 残差 = predicted_field - measured_field (mT)
 */
struct FreeStrengthResidual {
  FreeStrengthResidual(const Eigen::Vector3d &sensor_pos,
                       const Eigen::Vector3d &measured_field)
      : sensor_pos_(sensor_pos), measured_field_(measured_field) {}

  /**
   * @brief 计算残差
   * @param position 磁铁位置 [x, y, z] (米)
   * @param direction 磁矩方向向量（归一化）
   * @param strength 磁矩强度 (Am²)
   * @param residuals 输出的残差向量 [rx, ry, rz] (mT)
   */
  template <typename T>
  bool operator()(const T *const position, const T *const direction, const T *const strength, T *residuals) const {
    Eigen::Matrix<T, 3, 1> sensor_pos_T(T(sensor_pos_.x()), T(sensor_pos_.y()), T(sensor_pos_.z()));
    Eigen::Matrix<T, 3, 1> measured_T(T(measured_field_.x()), T(measured_field_.y()), T(measured_field_.z()));
    Eigen::Matrix<T, 3, 1> pos(position[0], position[1], position[2]);
    Eigen::Matrix<T, 3, 1> dir(direction[0], direction[1], direction[2]);
    
    // 使用磁偶极子模型计算预测磁场
    Eigen::Matrix<T, 3, 1> predicted = MagneticFieldModel::dipoleField(sensor_pos_T, pos, dir, strength[0]);
    
    // 计算残差
    Eigen::Matrix<T, 3, 1> r = predicted - measured_T;
    residuals[0] = r.x();
    residuals[1] = r.y();
    residuals[2] = r.z();
    return true;
  }

  Eigen::Vector3d sensor_pos_;  // 传感器位置 (米)
  Eigen::Vector3d measured_field_;  // 测量磁场 (mT)
};

}  // 匿名命名空间

/**
 * @brief 批量估计（主要方法）
 * @param batch 多个传感器的测量数据
 * @param pose_out 输出的姿态估计结果
 * @param error_out 输出的优化误差（可选，可为 nullptr）
 * @return 是否成功估计
 * 
 * 使用 Ceres 优化器，基于磁偶极子模型同时优化位置、方向和强度。
 * 
 * 优化流程：
 * 1. 初始化优化变量（位置、方向、强度）
 * 2. 为每个传感器测量添加残差块
 * 3. 设置方向向量的参数化（归一化约束）
 * 4. 设置强度变量的上下界（如果优化强度）
 * 5. 配置 Ceres 求解器选项
 * 6. 执行优化
 * 7. 更新内部状态
 * 
 * 注意：所有磁场数据的单位均为 mT（毫特斯拉）。
 */
bool OptimizerEstimator::estimateFromBatch(const std::vector<OptimizerMeasurement> &batch,
                                           geometry_msgs::Pose &pose_out,
                                           double *error_out) {
  if (batch.empty()) {
    ROS_WARN_THROTTLE(1.0, "optimizer_estimator: empty batch, cannot estimate");
    return false;
  }

  // 使用当前状态作为初始值
  double position[3] = {position_.x(), position_.y(), position_.z()};
  double direction[3] = {magnet_direction_.x(), magnet_direction_.y(), magnet_direction_.z()};
  double strength = magnet_strength_;
  
  // 验证和归一化方向向量
  double dir_norm = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
  if (dir_norm > 1e-9) {
    direction[0] /= dir_norm;
    direction[1] /= dir_norm;
    direction[2] /= dir_norm;
  } else {
    // 如果方向未初始化或为零向量，使用配置中的初始方向
    direction[0] = config_.optimizer.initial_direction.x();
    direction[1] = config_.optimizer.initial_direction.y();
    direction[2] = config_.optimizer.initial_direction.z();
    double init_norm = std::sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
    if (init_norm > 1e-9) {
      direction[0] /= init_norm;
      direction[1] /= init_norm;
      direction[2] /= init_norm;
    } else {
      // 默认方向为 Z 轴
      direction[0] = 0.0;
      direction[1] = 0.0;
      direction[2] = 1.0;
    }
  }
  
  // 验证初始值有效性（检查 NaN 和 Inf）
  bool has_invalid_value = false;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(position[i]) || !std::isfinite(direction[i])) {
      has_invalid_value = true;
      break;
    }
  }
  if (!std::isfinite(strength) || strength <= 0.0) {
    has_invalid_value = true;
  }
  
  if (has_invalid_value) {
    ROS_ERROR_THROTTLE(1.0, "optimizer_estimator: invalid initial values - position=[%.3f, %.3f, %.3f], "
                     "direction=[%.3f, %.3f, %.3f], strength=%.3f. Resetting to config defaults.",
                     position[0], position[1], position[2],
                     direction[0], direction[1], direction[2], strength);
    // 重置为配置中的初始值
    position[0] = config_.optimizer.initial_position.x();
    position[1] = config_.optimizer.initial_position.y();
    position[2] = config_.optimizer.initial_position.z();
    direction[0] = config_.optimizer.initial_direction.x();
    direction[1] = config_.optimizer.initial_direction.y();
    direction[2] = config_.optimizer.initial_direction.z();
    strength = config_.optimizer.initial_strength;
    // 重新归一化方向
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
  
  // 验证批量数据有效性
  if (batch.empty()) {
    ROS_WARN_THROTTLE(1.0, "optimizer_estimator: empty batch after validation");
    return false;
  }
  
  // 检查批量数据中是否有无效值
  for (const auto &meas : batch) {
    if (!meas.sensor_position.allFinite() || !meas.magnetic_field.allFinite()) {
      ROS_WARN_THROTTLE(1.0, "optimizer_estimator: invalid measurement data detected");
      return false;
    }
  }

  // 构建 Ceres 优化问题
  ceres::Problem problem;
  for (const auto &meas : batch) {
    Eigen::Vector3d measurement_T = meas.magnetic_field;  // 测量磁场 (mT)
    Eigen::Vector3d sensor_pos = meas.sensor_position;  // 传感器位置 (米)
    
    if (optimize_strength_) {
      // 优化强度：使用 FreeStrengthResidual
      auto *cost = new ceres::AutoDiffCostFunction<FreeStrengthResidual, 3, 3, 3, 1>(
          new FreeStrengthResidual(sensor_pos, measurement_T));
      problem.AddResidualBlock(cost, nullptr, position, direction, &strength);
    } else {
      // 固定强度：使用 FixedStrengthResidual
      auto *cost = new ceres::AutoDiffCostFunction<FixedStrengthResidual, 3, 3, 3>(
          new FixedStrengthResidual(sensor_pos, measurement_T, magnet_strength_));
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

  // 配置 Ceres 求解器选项
  ceres::Solver::Options options;
  options.linear_solver_type = linear_solver_type_;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = std::max(1, config_.optimizer.max_iterations);
  options.function_tolerance = config_.optimizer.function_tolerance;
  options.gradient_tolerance = config_.optimizer.gradient_tolerance;
  options.parameter_tolerance = config_.optimizer.parameter_tolerance;
  options.num_threads = std::max(1, config_.optimizer.num_threads);
  options.minimizer_progress_to_stdout = config_.optimizer.minimizer_progress;

  // 执行优化
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 检查优化结果
  if (summary.termination_type == ceres::NO_CONVERGENCE) {
    ROS_WARN_THROTTLE(1.0, "optimizer_estimator: solver did not converge after %d iterations", 
                      summary.num_successful_steps);
    // 即使未完全收敛，也使用当前结果
  } else if (summary.termination_type == ceres::FAILURE) {
    ROS_WARN_THROTTLE(1.0, "optimizer_estimator: solver failed: %s", summary.BriefReport().c_str());
    return false;
  }
  
  if (error_out) {
    *error_out = summary.final_cost;
  }

  // 将优化后的变量写回内部状态
  position_ = Eigen::Vector3d(position[0], position[1], position[2]);
  magnet_direction_ = Eigen::Vector3d(direction[0], direction[1], direction[2]).normalized();
  if (optimize_strength_) {
    magnet_strength_ = strength;
  }

  // 更新姿态（基于优化后的方向）
  orientation_ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), magnet_direction_);
  orientation_.normalize();
  
  last_pose_ = poseFromDirection(magnet_direction_);
  pose_out = last_pose_;
  
  ROS_DEBUG_THROTTLE(1.0, "optimizer_estimator: estimated position [%.4f, %.4f, %.4f], direction [%.4f, %.4f, %.4f], cost=%.6e",
                     position[0], position[1], position[2],
                     direction[0], direction[1], direction[2],
                     summary.final_cost);
  
  return true;
}

/**
 * @brief 获取当前估计的姿态
 * @return 包含位置和姿态的 Pose 消息
 */
geometry_msgs::Pose OptimizerEstimator::getPose() const {
  return last_pose_;
}

/**
 * @brief 重置估计器
 * 调用 resetState() 恢复到初始状态
 */
void OptimizerEstimator::reset() {
  resetState();
}

}  // 命名空间 mag_pose_estimator
