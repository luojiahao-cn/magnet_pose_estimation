#include "mag_pose_estimator/mag_preprocessor.h"

#include <Eigen/Geometry>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <string>

namespace mag_pose_estimator {

MagPreprocessor::MagPreprocessor()
    : low_pass_alpha_(0.3),
      enable_filter_(true),
      enable_calibration_(false),
      filter_initialized_(false) {
  soft_iron_matrix_.setIdentity();
  hard_iron_offset_.setZero();
}

void MagPreprocessor::configure(const ros::NodeHandle &nh) {
  namespace rps = rosparam_shortcuts;
  const std::string ns = "mag_pose_estimator/preprocessor";
  std::size_t error = 0;
  std::vector<double> soft_iron_vec;
  std::vector<double> hard_iron_vec;

  // 通过这些开关即可在原始模式或标定模式之间复用同一组件。
  error += !rps::get(ns, nh, "enable_filter", enable_filter_);
  error += !rps::get(ns, nh, "enable_calibration", enable_calibration_);
  error += !rps::get(ns, nh, "low_pass_alpha", low_pass_alpha_);
  error += !rps::get(ns, nh, "soft_iron_matrix", soft_iron_vec);
  error += !rps::get(ns, nh, "hard_iron_offset", hard_iron_vec);

  if (soft_iron_vec.size() != 9) {
    ROS_ERROR("%s: soft_iron_matrix 必须是长度为 9 的数组", ns.c_str());
    ++error;
  } else {
    soft_iron_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(soft_iron_vec.data());
  }

  if (hard_iron_vec.size() != 3) {
    ROS_ERROR("%s: hard_iron_offset 必须是长度为 3 的数组", ns.c_str());
    ++error;
  } else {
    hard_iron_offset_ = Eigen::Map<const Eigen::Vector3d>(hard_iron_vec.data());
  }

  rps::shutdownIfError(ns, error);
}

sensor_msgs::MagneticField MagPreprocessor::process(const sensor_msgs::MagneticField &msg) {
  Eigen::Vector3d field(msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z);

  if (enable_calibration_) {
    field = soft_iron_matrix_ * (field - hard_iron_offset_);
  }

  if (enable_filter_) {
    // 在喂给估计器前用指数平滑抑制传感器噪声。
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
