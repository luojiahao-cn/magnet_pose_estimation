#include "mag_pose_estimator/mag_preprocessor.h"
#include "mag_pose_estimator/mag_pose_estimator_node.h"

#include <Eigen/Geometry>
#include <string>

namespace mag_pose_estimator {

/**
 * @brief 构造函数
 * 初始化默认参数：启用滤波，禁用校准，滤波器系数 0.3
 */
MagPreprocessor::MagPreprocessor()
    : low_pass_alpha_(0.3),
      enable_filter_(true),
      enable_calibration_(false),
      filter_initialized_(false) {
  soft_iron_matrix_.setIdentity();  // 默认无软铁校准（单位矩阵）
  hard_iron_offset_.setZero();  // 默认无硬铁偏移
}

/**
 * @brief 配置预处理器参数
 * 从配置结构体中加载校准和滤波参数
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
 * 
 * 处理流程：
 * 1. 软/硬铁校准：field = soft_iron_matrix * (field - hard_iron_offset)
 *    用于补偿传感器周围的铁磁材料干扰
 * 2. 低通滤波：使用指数平滑滤波器抑制高频噪声
 *    filtered = α * current + (1-α) * previous
 * 
 * 注意：所有数据的单位均为 mT（毫特斯拉）
 */
sensor_msgs::MagneticField MagPreprocessor::process(const sensor_msgs::MagneticField &msg) {
  Eigen::Vector3d field(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

  // 软/硬铁校准
  if (enable_calibration_) {
    // 硬铁偏移补偿：减去固定偏移量
    // 软铁校准：通过矩阵变换补偿各向异性
    field = soft_iron_matrix_ * (field - hard_iron_offset_);
  }

  // 低通滤波（指数平滑）
  if (enable_filter_) {
    if (!filter_initialized_) {
      // 首次测量，直接使用当前值
      filtered_field_ = field;
      filter_initialized_ = true;
    } else {
      // 指数平滑：filtered = α * current + (1-α) * previous
      // α 越大，对当前值响应越快，但噪声抑制能力越弱
      filtered_field_ = low_pass_alpha_ * field + (1.0 - low_pass_alpha_) * filtered_field_;
    }
    field = filtered_field_;
  }

  // 构造输出消息
  sensor_msgs::MagneticField calibrated = msg;
  calibrated.magnetic_field.x = field.x();
  calibrated.magnetic_field.y = field.y();
  calibrated.magnetic_field.z = field.z();
  return calibrated;
}

}  // 命名空间 mag_pose_estimator
