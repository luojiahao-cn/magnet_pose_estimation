#include "mag_pose_estimator/mag_preprocessor.h"

#include <Eigen/Geometry>

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
  std::vector<double> soft_iron_vec;
  std::vector<double> hard_iron_vec;

  nh.param("enable_filter", enable_filter_, enable_filter_);
  nh.param("enable_calibration", enable_calibration_, enable_calibration_);
  nh.param("low_pass_alpha", low_pass_alpha_, low_pass_alpha_);

  if (nh.getParam("soft_iron_matrix", soft_iron_vec) && soft_iron_vec.size() == 9) {
    soft_iron_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(soft_iron_vec.data());
  } else {
    soft_iron_matrix_.setIdentity();
  }

  if (nh.getParam("hard_iron_offset", hard_iron_vec) && hard_iron_vec.size() == 3) {
    hard_iron_offset_ = Eigen::Map<const Eigen::Vector3d>(hard_iron_vec.data());
  } else {
    hard_iron_offset_.setZero();
  }
}

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

}  // namespace mag_pose_estimator
