#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/MagneticField.h>
#include <ros/time.h>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <functional>

namespace mag_pose_estimator {

/**
 * @brief EKF 估计器参数配置结构体
 */
struct EKFParameters {
  Eigen::Vector3d world_field;  ///< 世界坐标系下的磁场向量 [x, y, z] (mT)
  double process_noise_position;  ///< 位置过程噪声方差
  double process_noise_orientation;  ///< 姿态过程噪声方差
  double measurement_noise;  ///< 测量噪声方差
  double position_gain;  ///< 位置修正增益系数
};

/**
 * @brief 优化器参数配置结构体
 */
struct OptimizerParameters {
  int min_sensors;  ///< 最小有效传感器数量
  Eigen::Vector3d initial_position;  ///< 初始位置估计 [x, y, z] (米)
  Eigen::Vector3d initial_direction;  ///< 初始磁矩方向向量（归一化）
  double initial_strength;  ///< 初始磁矩强度 (Am²)
  double strength_delta;  ///< 磁矩强度优化范围 (±delta)
  bool optimize_strength;  ///< 是否优化磁矩强度
  int max_iterations;  ///< 最大迭代次数
  double function_tolerance;  ///< 函数值收敛容差
  double gradient_tolerance;  ///< 梯度收敛容差
  double parameter_tolerance;  ///< 参数收敛容差
  int num_threads;  ///< 并行计算线程数
  bool minimizer_progress;  ///< 是否输出优化进度信息
  std::string linear_solver;  ///< 线性求解器类型
  double max_acceptable_residual;  ///< 可接受的最大平均残差 (mT)，超过此值即使优化收敛也认为失败
};

/**
 * @brief 估计器配置结构体
 */
struct EstimatorConfig {
  EKFParameters ekf;  ///< EKF 估计器参数
  OptimizerParameters optimizer;  ///< 优化器参数
};

/**
 * @brief 估计器基类
 */
class EstimatorBase {
public:
  virtual ~EstimatorBase() = default;

  /**
   * @brief 初始化估计器
   */
  virtual void initialize() = 0;

  /**
   * @brief 更新估计器状态
   * @param mag 磁场测量数据 (mT)
   */
  virtual void update(const sensor_msgs::MagneticField &mag) = 0;

  /**
   * @brief 处理批量传感器数据
   * @param measurements 预处理后的传感器数据列表
   * @param tf_query TF 查询函数 (frame_id, stamp) -> (position, transform, success)
   * @param output_frame 输出坐标系
   * @param pose_out 输出的姿态估计结果
   * @param error_out 输出的估计误差（可选）
   * @return 是否成功估计
   */
  virtual bool processBatch(
      const std::vector<sensor_msgs::MagneticField> &measurements,
      const std::function<bool(const std::string &, const ros::Time &, Eigen::Vector3d &, geometry_msgs::TransformStamped &)> & /* tf_query */,
      const std::string & /* output_frame */,
      geometry_msgs::Pose &pose_out,
      double * /* error_out */ = nullptr) {
    for (const auto &mag : measurements) {
      update(mag);
    }
    pose_out = getPose();
    return true;
  }

  /**
   * @brief 获取当前估计的姿态
   * @return 包含位置和姿态的 Pose 消息
   */
  virtual geometry_msgs::Pose getPose() const = 0;

  /**
   * @brief 获取当前估计的磁矩强度
   * @return 磁矩强度 (Am²)
   */
  virtual double getMagneticStrength() const {
    return 0.0;
  }

  /**
   * @brief 获取估计器名称
   * @return 估计器类型名称
   */
  virtual std::string name() const = 0;

  /**
   * @brief 设置估计器配置
   * @param config 配置结构体
   */
  inline void setConfig(const EstimatorConfig &config) {
    config_ = config;
  }

protected:
  EstimatorConfig config_;  ///< 估计器配置参数
  bool initialized_ = false;  ///< 是否已初始化
};

}  // 命名空间 mag_pose_estimator
