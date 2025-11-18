#include "mag_pose_estimator/estimator_factory.h"

#include <algorithm>
#include <ros/ros.h>

#include "mag_pose_estimator/ekf_estimator.h"
#include "mag_pose_estimator/optimizer_estimator.h"

namespace mag_pose_estimator {

std::unique_ptr<EstimatorBase> createEstimator(const std::string &type) {
  std::string lower = type;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "ekf") {
    ROS_INFO("mag_pose_estimator: launching EKF estimator");
    return std::make_unique<EKFEstimator>();
  }

  if (lower == "optimizer") {
    ROS_INFO("mag_pose_estimator: launching optimizer estimator");
    return std::make_unique<OptimizerEstimator>();
  }

  // 未知的类型统一回退到 EKF，保证节点仍能以合理默认值启动。
  ROS_WARN("mag_pose_estimator: unknown estimator type '%s', defaulting to EKF", type.c_str());
  return std::make_unique<EKFEstimator>();
}

}  // 命名空间 mag_pose_estimator
