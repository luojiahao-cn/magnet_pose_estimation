#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/mag_pose_estimator_node.h"
#include "mag_sensor_calibration/calibration_config_loader.h"

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
      filter_initialized_(false),
      use_multi_sensor_calibration_(false) {
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
 * @brief 处理磁场测量数据（使用全局校正参数）
 * @param msg 输入的磁场测量消息
 * @return 处理后的磁场测量消息
 */
sensor_msgs::MagneticField MagPreprocessor::process(const sensor_msgs::MagneticField &msg) {
  Eigen::Vector3d field(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

  if (enable_calibration_ && !use_multi_sensor_calibration_) {
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

/**
 * @brief 处理磁场测量数据（使用指定传感器ID的校正参数）
 * @param msg 输入的磁场测量消息
 * @param sensor_id 传感器ID
 * @return 处理后的磁场测量消息
 */
sensor_msgs::MagneticField MagPreprocessor::process(const sensor_msgs::MagneticField &msg, uint32_t sensor_id) {
  Eigen::Vector3d field(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

  if (enable_calibration_) {
    if (use_multi_sensor_calibration_) {
      // 使用多传感器校正参数
      auto soft_it = sensor_soft_iron_matrices_.find(sensor_id);
      auto hard_it = sensor_hard_iron_offsets_.find(sensor_id);
      
      if (soft_it != sensor_soft_iron_matrices_.end() && hard_it != sensor_hard_iron_offsets_.end()) {
        field = soft_it->second * (field - hard_it->second);
      } else {
        // 如果没有找到该传感器的校正参数，使用全局参数
        field = soft_iron_matrix_ * (field - hard_iron_offset_);
      }
    } else {
      // 使用全局校正参数
      field = soft_iron_matrix_ * (field - hard_iron_offset_);
    }
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

/**
 * @brief 加载多传感器校正参数
 */
void MagPreprocessor::loadMultiSensorCalibration(const mag_sensor_calibration::CalibrationParams &params) {
  sensor_soft_iron_matrices_.clear();
  sensor_hard_iron_offsets_.clear();
  
  for (const auto &kv : params.sensors) {
    sensor_soft_iron_matrices_[kv.first] = kv.second.soft_iron_matrix;
    sensor_hard_iron_offsets_[kv.first] = kv.second.hard_iron_offset;
  }
  
  use_multi_sensor_calibration_ = !params.sensors.empty();
}

}  // 命名空间 mag_pose_estimator
