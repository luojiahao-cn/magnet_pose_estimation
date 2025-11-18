#pragma once

#include <ceres/ceres.h>

#include <Eigen/Geometry>
#include <vector>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

/**
 * @brief 优化器测量数据结构
 * 
 * 用于批量优化器，包含单个传感器的测量数据。
 */
struct OptimizerMeasurement {
  int sensor_id = 0;  // 传感器 ID
  Eigen::Vector3d sensor_position = Eigen::Vector3d::Zero();  // 传感器位置 [x, y, z] (米)
  Eigen::Vector3d magnetic_field = Eigen::Vector3d::Zero();  // 磁场测量值 [Bx, By, Bz] (mT)
};

/**
 * @brief 非线性优化器姿态估计器
 * 
 * 使用 Ceres 优化库和磁偶极子模型，通过非线性优化同时估计磁铁的位置、方向和强度。
 * 
 * 核心功能：
 * 1. 批量优化：使用多个传感器的测量数据，通过 Ceres 优化器同时优化位置、方向和强度
 * 2. 单次更新：使用 Gauss-Newton 方法快速更新姿态（仅优化姿态，不优化位置）
 * 
 * 优化变量：
 *   - position: 磁铁位置 [x, y, z] (米)
 *   - direction: 磁矩方向向量（归一化）
 *   - strength: 磁矩强度 (Am²，可选优化)
 * 
 * 残差函数：
 *   基于磁偶极子模型，计算预测磁场与测量磁场的差值。
 *   残差 = predicted_field - measured_field (mT)
 * 
 * 注意：所有磁场数据的单位均为 mT（毫特斯拉）。
 */
class OptimizerEstimator : public EstimatorBase {
public:
  OptimizerEstimator();

  void initialize() override;
  void update(const sensor_msgs::MagneticField &mag) override;
  geometry_msgs::Pose getPose() const override;
  double getMagneticStrength() const override { return magnet_strength_; }
  std::string name() const override { return "optimizer"; }

  /**
   * @brief 批量估计（主要方法）
   * @param batch 多个传感器的测量数据
   * @param pose_out 输出的姿态估计结果
   * @param error_out 输出的优化误差（可选，可为 nullptr）
   * @return 是否成功估计
   * 
   * 使用 Ceres 优化器，基于磁偶极子模型同时优化位置、方向和强度。
   * 这是位置估计的主要方法，能够利用多个传感器的空间分布信息。
   */
  bool estimateFromBatch(const std::vector<OptimizerMeasurement> &batch,
                         geometry_msgs::Pose &pose_out,
                         double *error_out);

  /**
   * @brief 重置估计器状态
   * 恢复到配置中的初始值
   */
  void reset();

private:
  /**
   * @brief 单次测量的快速姿态更新
   * @param measurement 磁场测量向量 (mT)
   * 
   * 使用 Gauss-Newton 方法快速更新姿态（仅优化姿态，不优化位置）。
   * 用于在批量优化之间保持姿态估计的新鲜度。
   * 
   * 注意：此方法只优化姿态，位置估计应通过批量优化器（estimateFromBatch）来完成。
   */
  void runGaussNewton(const Eigen::Vector3d &measurement);

  /**
   * @brief 重置内部状态
   * 从配置中读取初始值并重置所有状态变量
   */
  void resetState();

  /**
   * @brief 解析线性求解器类型字符串
   * @param solver 求解器类型字符串
   * @return Ceres 线性求解器类型
   */
  ceres::LinearSolverType resolveLinearSolver(const std::string &solver) const;

  /**
   * @brief 从方向向量构造姿态
   * @param direction 磁矩方向向量（归一化）
   * @return 包含位置和姿态的 Pose 消息
   * 
   * 将磁矩方向向量转换为四元数姿态，位置使用当前估计值。
   */
  geometry_msgs::Pose poseFromDirection(const Eigen::Vector3d &direction) const;

  Eigen::Vector3d position_;  // 当前估计的磁铁位置 [x, y, z] (米)
  Eigen::Quaterniond orientation_;  // 当前估计的姿态四元数
  Eigen::Vector3d magnet_direction_;  // 当前估计的磁矩方向向量（归一化）
  double magnet_strength_ = 1.0;  // 当前估计的磁矩强度 (Am²)
  bool optimize_strength_ = false;  // 是否优化磁矩强度
  double strength_min_ = 0.0;  // 磁矩强度优化下界
  double strength_max_ = 0.0;  // 磁矩强度优化上界
  ceres::LinearSolverType linear_solver_type_ = ceres::DENSE_QR;  // Ceres 线性求解器类型
  geometry_msgs::Pose last_pose_;  // 上次估计的姿态（用于 getPose()）
};

}  // 命名空间 mag_pose_estimator
