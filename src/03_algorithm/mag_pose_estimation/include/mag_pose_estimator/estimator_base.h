#pragma once

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <string>

namespace mag_pose_estimator {

/**
 * @brief 优化器参数配置结构体
 * 
 * 用于配置 Ceres 批量优化器的各项参数，包括初始值、迭代次数、收敛容差等。
 */
struct OptimizerParameters {
  Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();  // 初始位置估计 [x, y, z] (米)
  Eigen::Vector3d initial_direction = Eigen::Vector3d::UnitZ();  // 初始磁矩方向向量（归一化）
  double initial_strength = 1.0;  // 初始磁矩强度 (Am²，安培·米²)
  double strength_delta = 0.0;  // 磁矩强度优化范围 (±delta)
  bool optimize_strength = false;  // 是否优化磁矩强度
  int max_iterations = 20;  // 最大迭代次数
  double function_tolerance = 1e-8;  // 函数值收敛容差
  double gradient_tolerance = 1e-10;  // 梯度收敛容差
  double parameter_tolerance = 1e-10;  // 参数收敛容差
  int num_threads = 2;  // 并行计算线程数
  bool minimizer_progress = false;  // 是否输出优化进度信息
  std::string linear_solver = "DENSE_QR";  // 线性求解器类型
};

/**
 * @brief 估计器配置结构体
 * 
 * 包含所有估计器（EKF、优化器等）共享的配置参数。
 */
struct EstimatorConfig {
  Eigen::Vector3d world_field;            // 世界坐标系下的磁场向量（mT，毫特斯拉）
  double process_noise_position;          // 位置过程噪声方差（随机游走模型）
  double process_noise_orientation;       // 姿态过程噪声方差（四元数随机游走）
  double measurement_noise;               // 归一化磁场向量的测量噪声方差
  double position_gain;                   // 将磁场幅值误差映射为位置修正的增益系数
  int optimizer_iterations;               // 传统优化器（Gauss-Newton）的单次迭代次数
  double optimizer_damping;               // 传统优化步骤使用的阻尼系数（Levenberg-Marquardt）
  OptimizerParameters optimizer;          // Ceres 批量优化器的完整配置参数
};

/**
 * @brief 估计器基类
 * 
 * 所有姿态估计算法（EKF、优化器等）的统一接口基类。
 * 提供初始化和更新接口，支持通过配置结构体设置参数。
 */
class EstimatorBase {
public:
  virtual ~EstimatorBase() = default;

  /**
   * @brief 初始化估计器
   * 设置初始状态和协方差矩阵
   */
  virtual void initialize() = 0;

  /**
   * @brief 更新估计器状态
   * @param mag 磁场测量数据（单位：mT，毫特斯拉）
   */
  virtual void update(const sensor_msgs::MagneticField &mag) = 0;

  /**
   * @brief 获取当前估计的姿态
   * @return 包含位置和姿态的 Pose 消息
   */
  virtual geometry_msgs::Pose getPose() const = 0;

  /**
   * @brief 获取当前估计的磁矩强度
   * @return 磁矩强度 (Am²，安培·米²)
   * 
   * 对于优化器，返回实际估计的磁矩强度。
   * 对于其他估计器，可能返回配置值或 0.0。
   */
  virtual double getMagneticStrength() const {
    return 0.0;  // 默认实现返回 0.0
  }

  /**
   * @brief 获取估计器名称
   * @return 估计器类型名称（如 "ekf", "optimizer"）
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
  EstimatorConfig config_;  // 估计器配置参数
  bool initialized_ = false;  // 是否已初始化
};

}  // 命名空间 mag_pose_estimator
