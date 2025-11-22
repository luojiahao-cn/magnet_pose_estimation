#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/mag_pose_estimator_node.h"

#include <Eigen/Geometry>
#include <string>

namespace mag_pose_estimator {

/**
 * @brief 构造函数
 */
MagPreprocessor::MagPreprocessor()
    : low_pass_alpha_(0.3),
      enable_filter_(true),
      enable_calibration_(false),
      filter_initialized_(false) {
  soft_iron_matrix_.setIdentity();
  hard_iron_offset_.setZero();
}

/**
 * @brief 配置预处理器参数
 * @param config 配置结构体
 */
void MagPreprocessor::configure(const MagPoseEstimatorConfig &config) {
  enable_filter_ = config.enable_filter;
  enable_calibration_ = config.enable_calibration;
  low_pass_alpha_ = config.low_pass_alpha;
  soft_iron_matrix_ = config.soft_iron_matrix;
  hard_iron_offset_ = config.hard_iron_offset;
}

/**
 * @brief 处理磁场测量数据
 * @param msg 输入的磁场测量消息
 * @return 处理后的磁场测量消息
 */
sensor_msgs::MagneticField MagPreprocessor::process(const sensor_msgs::MagneticField &msg) {
  Eigen::Vector3d field(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

  if (enable_calibration_) {
    field = soft_iron_matrix_ * (field - hard_iron_offset_);
  }

  if (enable_filter_) {
    if (!filter_initialized_) {
      filtered_field_ = field;
      filter_initialized_ = true;
    } else {
      filtered_field_ = low_pass_alpha_ * field + (1.0 - low_pass_alpha_) * filtered_field_;
    }
    field = filtered_field_;
  }

  sensor_msgs::MagneticField calibrated = msg;
  calibrated.magnetic_field.x = field.x();
  calibrated.magnetic_field.y = field.y();
  calibrated.magnetic_field.z = field.z();
  return calibrated;
}

}  // 命名空间 mag_pose_estimator
