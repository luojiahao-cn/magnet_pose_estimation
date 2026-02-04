#pragma once

#include <ceres/ceres.h>

#include <Eigen/Geometry>
#include <vector>

#include "mag_pose_estimator/estimator_base.h"

namespace mag_pose_estimator {

/**
 * @brief 优化器测量数据结构
 */
struct OptimizerMeasurement {
  int sensor_id;  ///< 传感器 ID
  Eigen::Vector3d sensor_position;  ///< 传感器位置 [x, y, z] (米)
  Eigen::Vector3d magnetic_field;  ///< 磁场测量值 [Bx, By, Bz] (mT)
};

/**
 * @brief 优化器姿态估计器
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
   * @brief 处理批量传感器数据
   * @param measurements 预处理后的传感器数据列表
   * @param tf_query TF 查询函数
   * @param output_frame 输出坐标系
   * @param pose_out 输出的姿态估计结果
   * @param error_out 输出的估计误差（可选）
   * @return 是否成功估计
   */
  bool processBatch(
      const std::vector<sensor_msgs::MagneticField> &measurements,
      const std::function<bool(const std::string &, const ros::Time &, Eigen::Vector3d &, geometry_msgs::TransformStamped &)> &tf_query,
      const std::string &output_frame,
      geometry_msgs::Pose &pose_out,
      double *error_out = nullptr) override;

  /**
   * @brief 批量估计
   * @param batch 多个传感器的测量数据
   * @param pose_out 输出的姿态估计结果
   * @param error_out 输出的优化误差（可选）
   * @return 是否成功估计
   */
  bool estimateFromBatch(const std::vector<OptimizerMeasurement> &batch,
                         geometry_msgs::Pose &pose_out,
                         double *error_out);

  /**
   * @brief 重置估计器状态
   */
  void reset();

private:
  /**
   * @brief 重置内部状态
   */
  void resetState();

  /**
   * @brief 从方向向量构造姿态四元数
   * @param direction 磁矩方向向量（归一化）
   * @param position 磁铁位置
   * @return 包含位置和姿态的 Pose 消息
   */
  geometry_msgs::Pose buildPoseFromDirection(const Eigen::Vector3d &direction,
                                             const Eigen::Vector3d &position) const;

  Eigen::Vector3d position_;  ///< 当前估计的磁铁位置 [x, y, z] (米)
  Eigen::Vector3d magnet_direction_;  ///< 当前估计的磁矩方向向量（归一化）
  double magnet_strength_;  ///< 当前估计的磁矩强度 (Am²)
  bool optimize_strength_;  ///< 是否优化磁矩强度
  double strength_min_;  ///< 磁矩强度优化下界
  double strength_max_;  ///< 磁矩强度优化上界
  ceres::LinearSolverType linear_solver_type_;  ///< Ceres 线性求解器类型
  geometry_msgs::Pose last_pose_;  ///< 上次估计的姿态
};

}  // 命名空间 mag_pose_estimator
