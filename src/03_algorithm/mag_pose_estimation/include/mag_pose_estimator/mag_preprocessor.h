#pragma once

#include <ros/ros.h>
#include <sensor_msgs/MagneticField.h>

#include <Eigen/Dense>
#include <vector>

namespace mag_pose_estimator {

struct MagPoseEstimatorConfig;

/**
 * @brief 负责磁力计的软/硬铁校准与简单低通滤波。
 */
class MagPreprocessor {
public:
  MagPreprocessor();

  void configure(const MagPoseEstimatorConfig &config);
  sensor_msgs::MagneticField process(const sensor_msgs::MagneticField &msg);

private:
  Eigen::Matrix3d soft_iron_matrix_;
  Eigen::Vector3d hard_iron_offset_;

  double low_pass_alpha_;
  bool enable_filter_;
  bool enable_calibration_;

  bool filter_initialized_;
  Eigen::Vector3d filtered_field_;
};

}  // 命名空间 mag_pose_estimator
