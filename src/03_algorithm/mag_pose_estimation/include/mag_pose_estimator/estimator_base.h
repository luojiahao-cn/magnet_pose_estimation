#pragma once

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <string>

namespace mag_pose_estimator {

struct OptimizerParameters {
  Eigen::Vector3d initial_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d initial_direction = Eigen::Vector3d::UnitZ();
  double initial_strength = 1.0;
  double strength_delta = 0.0;
  bool optimize_strength = false;
  int max_iterations = 20;
  double function_tolerance = 1e-8;
  double gradient_tolerance = 1e-10;
  double parameter_tolerance = 1e-10;
  int num_threads = 2;
  bool minimizer_progress = false;
  std::string linear_solver = "DENSE_QR";
};

struct EstimatorConfig {
  Eigen::Vector3d world_field;            // 世界坐标系下的磁场向量（特斯拉）
  double process_noise_position;          // XYZ 随机游走对应的位置过程噪声
  double process_noise_orientation;       // 四元数随机游走对应的姿态过程噪声
  double measurement_noise;               // 归一化磁场向量的测量噪声方差
  double position_gain;                   // 将磁场幅值误差映射为位置修正的增益
  int optimizer_iterations;               // 传统优化器的单次迭代次数
  double optimizer_damping;               // 传统优化步骤使用的阻尼系数
  OptimizerParameters optimizer;          // Ceres 批量优化专用的完整配置
};

class EstimatorBase {
public:
  virtual ~EstimatorBase() = default;

  virtual void initialize() = 0;
  virtual void update(const sensor_msgs::MagneticField &mag) = 0;
  virtual geometry_msgs::Pose getPose() const = 0;

  virtual std::string name() const = 0;

  inline void setConfig(const EstimatorConfig &config) {
    config_ = config;
  }

protected:
  EstimatorConfig config_;
  bool initialized_ = false;
};

}  // 命名空间 mag_pose_estimator
